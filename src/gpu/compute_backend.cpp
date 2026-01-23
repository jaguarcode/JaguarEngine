/**
 * @file compute_backend.cpp
 * @brief Backend factory and utility implementations
 */

#include "jaguar/gpu/compute_backend.h"
#include <unordered_map>
#include <mutex>

namespace jaguar::gpu {

// ============================================================================
// Backend Factory Implementation
// ============================================================================

std::unordered_map<BackendType, BackendFactory::CreatorFunc>& BackendFactory::registry() {
    static std::unordered_map<BackendType, CreatorFunc> s_registry;
    return s_registry;
}

void BackendFactory::register_backend(BackendType type, CreatorFunc creator) {
    static std::mutex s_mutex;
    std::lock_guard<std::mutex> lock(s_mutex);
    registry()[type] = std::move(creator);
}

std::unique_ptr<IComputeBackend> BackendFactory::create(BackendType type) {
    // Ensure built-in backends are registered
    register_builtin_backends();

    if (type == BackendType::Auto) {
        return create_best_available();
    }

    auto& reg = registry();
    auto it = reg.find(type);
    if (it != reg.end() && it->second) {
        return it->second();
    }
    return nullptr;
}

std::unique_ptr<IComputeBackend> BackendFactory::create_best_available() {
    // Priority order: CUDA > Metal (macOS) > OpenCL > CPU
    static const BackendType priority[] = {
        BackendType::CUDA,
#ifdef __APPLE__
        BackendType::Metal,
#endif
        BackendType::OpenCL,
        BackendType::Vulkan,
        BackendType::CPU
    };

    for (auto type : priority) {
        if (is_available(type)) {
            auto backend = create(type);
            if (backend) {
                return backend;
            }
        }
    }

    return nullptr;
}

std::vector<BackendType> BackendFactory::available_backends() {
    // Ensure built-in backends are registered
    register_builtin_backends();

    std::vector<BackendType> result;
    result.reserve(registry().size());

    for (const auto& [type, creator] : registry()) {
        if (creator) {
            result.push_back(type);
        }
    }

    return result;
}

bool BackendFactory::is_available(BackendType type) {
    // Ensure built-in backends are registered
    register_builtin_backends();

    if (type == BackendType::Auto) {
        return !registry().empty();
    }

    auto& reg = registry();
    auto it = reg.find(type);
    return it != reg.end() && it->second != nullptr;
}

const char* BackendFactory::type_name(BackendType type) {
    switch (type) {
        case BackendType::CPU:     return "CPU";
        case BackendType::CUDA:    return "CUDA";
        case BackendType::OpenCL:  return "OpenCL";
        case BackendType::Metal:   return "Metal";
        case BackendType::Vulkan:  return "Vulkan";
        case BackendType::Auto:    return "Auto";
        default:                   return "Unknown";
    }
}

// ============================================================================
// Utility Function Implementations
// ============================================================================

const char* result_to_string(BackendResult result) {
    switch (result) {
        case BackendResult::Success:           return "Success";
        case BackendResult::DeviceNotFound:    return "Device not found";
        case BackendResult::OutOfMemory:       return "Out of memory";
        case BackendResult::InvalidArgument:   return "Invalid argument";
        case BackendResult::CompilationFailed: return "Compilation failed";
        case BackendResult::LaunchFailed:      return "Launch failed";
        case BackendResult::SyncFailed:        return "Synchronization failed";
        case BackendResult::NotInitialized:    return "Not initialized";
        case BackendResult::NotSupported:      return "Not supported";
        case BackendResult::InternalError:     return "Internal error";
        default:                               return "Unknown error";
    }
}

const char* memory_type_to_string(MemoryType type) {
    switch (type) {
        case MemoryType::DeviceLocal:  return "DeviceLocal";
        case MemoryType::HostVisible:  return "HostVisible";
        case MemoryType::HostCached:   return "HostCached";
        case MemoryType::Shared:       return "Shared";
        case MemoryType::Staging:      return "Staging";
        default:                       return "Unknown";
    }
}

const char* device_type_to_string(DeviceType type) {
    switch (type) {
        case DeviceType::Unknown:       return "Unknown";
        case DeviceType::CPU:           return "CPU";
        case DeviceType::DiscreteGPU:   return "Discrete GPU";
        case DeviceType::IntegratedGPU: return "Integrated GPU";
        case DeviceType::Accelerator:   return "Accelerator";
        default:                        return "Unknown";
    }
}

SizeT calculate_optimal_workgroup_size(const IKernel* kernel,
                                        const DeviceCapabilities& caps,
                                        SizeT problem_size) {
    if (!kernel) {
        return 256; // Default fallback
    }

    // Start with kernel's preferred size
    SizeT preferred = kernel->preferred_workgroup_size();
    SizeT max_size = kernel->max_workgroup_size();

    // Consider device limits
    if (caps.max_workgroup_size > 0) {
        max_size = std::min(max_size, caps.max_workgroup_size);
    }

    // Ensure preferred doesn't exceed max
    preferred = std::min(preferred, max_size);

    // For small problems, don't over-parallelize
    if (problem_size < preferred) {
        // Round up to warp size for efficiency
        SizeT warp = caps.warp_size > 0 ? caps.warp_size : 32;
        return std::max(warp, (problem_size + warp - 1) / warp * warp);
    }

    // For larger problems, use preferred size rounded to warp boundary
    SizeT warp = caps.warp_size > 0 ? caps.warp_size : 32;
    return (preferred / warp) * warp;
}

// Forward declarations of backend creators
namespace detail {

// CPU backend (always available)
std::unique_ptr<IComputeBackend> create_cpu_backend();

#ifdef JAGUAR_HAS_CUDA
// CUDA backend (defined in cuda/cuda_backend.cu)
std::unique_ptr<IComputeBackend> create_cuda_backend();
bool is_cuda_available();
#endif

#ifdef JAGUAR_HAS_OPENCL
// OpenCL backend (defined in opencl/opencl_backend.cpp)
std::unique_ptr<IComputeBackend> create_opencl_backend();
bool is_opencl_available();
#endif

#ifdef JAGUAR_HAS_METAL
// Metal backend (defined in metal/metal_backend.mm)
std::unique_ptr<IComputeBackend> create_metal_backend();
bool is_metal_available();
#endif

} // namespace detail

void register_builtin_backends() {
    static bool registered = false;
    if (registered) return;
    registered = true;

    // Register CPU backend (always available)
    BackendFactory::register_backend(BackendType::CPU, detail::create_cpu_backend);

#ifdef JAGUAR_HAS_CUDA
    // Register CUDA backend if available
    if (detail::is_cuda_available()) {
        BackendFactory::register_backend(BackendType::CUDA, detail::create_cuda_backend);
    }
#endif

#ifdef JAGUAR_HAS_OPENCL
    // Register OpenCL backend if available
    if (detail::is_opencl_available()) {
        BackendFactory::register_backend(BackendType::OpenCL, detail::create_opencl_backend);
    }
#endif

#ifdef JAGUAR_HAS_METAL
    // Register Metal backend if available (macOS only)
    if (detail::is_metal_available()) {
        BackendFactory::register_backend(BackendType::Metal, detail::create_metal_backend);
    }
#endif
}

} // namespace jaguar::gpu
