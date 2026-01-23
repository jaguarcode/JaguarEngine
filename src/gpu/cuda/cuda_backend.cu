/**
 * @file cuda_backend.cu
 * @brief NVIDIA CUDA implementation of IComputeBackend
 *
 * This file provides GPU acceleration via CUDA for JaguarEngine physics
 * computations. It implements the full IComputeBackend interface with:
 * - Device enumeration and selection
 * - Memory allocation and transfer (sync and async)
 * - Kernel compilation and dispatch
 * - Stream-based async execution
 * - Event-based synchronization
 * - Comprehensive error handling
 *
 * Requirements:
 * - CUDA Toolkit 11.0 or later
 * - NVIDIA GPU with Compute Capability 5.0+
 */

#include "jaguar/gpu/compute_backend.h"
#include <cuda.h>
#include <cuda_runtime.h>
#include <nvrtc.h>
#include <unordered_map>
#include <atomic>
#include <mutex>
#include <vector>
#include <cstring>
#include <sstream>

namespace jaguar::gpu {

// ============================================================================
// CUDA Error Checking Utilities
// ============================================================================

#define CUDA_CHECK(call)                                                       \
    do {                                                                       \
        cudaError_t err = call;                                               \
        if (err != cudaSuccess) {                                             \
            set_cuda_error(err, #call);                                       \
            return BackendResult::InternalError;                              \
        }                                                                      \
    } while (0)

#define CUDA_CHECK_RETURN(call, ret)                                          \
    do {                                                                       \
        cudaError_t err = call;                                               \
        if (err != cudaSuccess) {                                             \
            set_cuda_error(err, #call);                                       \
            return ret;                                                        \
        }                                                                      \
    } while (0)

#define CU_CHECK(call)                                                         \
    do {                                                                       \
        CUresult err = call;                                                  \
        if (err != CUDA_SUCCESS) {                                            \
            set_cu_error(err, #call);                                         \
            return BackendResult::InternalError;                              \
        }                                                                      \
    } while (0)

#define NVRTC_CHECK(call)                                                      \
    do {                                                                       \
        nvrtcResult err = call;                                               \
        if (err != NVRTC_SUCCESS) {                                           \
            set_nvrtc_error(err, #call);                                      \
            return nullptr;                                                    \
        }                                                                      \
    } while (0)

// ============================================================================
// CUDA Kernel Implementation
// ============================================================================

/**
 * @brief CUDA kernel wrapper implementing IKernel interface
 */
class CUDAKernel : public IKernel {
public:
    CUDAKernel(const std::string& name, CUfunction func, CUmodule module)
        : m_name(name)
        , m_function(func)
        , m_module(module)
        , m_owns_module(false)
    {
        query_kernel_attributes();
    }

    CUDAKernel(const std::string& name, CUfunction func, CUmodule module, bool owns_module)
        : m_name(name)
        , m_function(func)
        , m_module(module)
        , m_owns_module(owns_module)
    {
        query_kernel_attributes();
    }

    ~CUDAKernel() override {
        if (m_owns_module && m_module) {
            cuModuleUnload(m_module);
        }
    }

    const std::string& name() const override { return m_name; }

    BackendResult set_arg(UInt32 index, const KernelArg& arg) override {
        std::lock_guard<std::mutex> lock(m_mutex);

        if (index >= m_args.size()) {
            m_args.resize(index + 1);
            m_arg_types.resize(index + 1);
        }
        m_args[index] = arg.value;
        m_arg_types[index] = arg.type;
        m_args_dirty = true;

        return BackendResult::Success;
    }

    SizeT preferred_workgroup_size() const override {
        return static_cast<SizeT>(m_preferred_block_size);
    }

    SizeT max_workgroup_size() const override {
        return static_cast<SizeT>(m_max_threads_per_block);
    }

    SizeT local_memory_size() const override {
        return static_cast<SizeT>(m_shared_size_bytes);
    }

    UInt32 register_count() const override {
        return m_num_regs;
    }

    bool is_valid() const override {
        return m_function != nullptr;
    }

    CUfunction function() const { return m_function; }

    // Prepare kernel arguments for launch
    // Returns vector of pointers that can be passed to cuLaunchKernel
    std::vector<void*> prepare_args(
        const std::unordered_map<UInt64, void*>& buffer_map) const
    {
        std::lock_guard<std::mutex> lock(m_mutex);

        std::vector<void*> arg_ptrs;
        arg_ptrs.reserve(m_args.size());

        for (size_t i = 0; i < m_args.size(); ++i) {
            switch (m_arg_types[i]) {
                case ArgType::Buffer: {
                    auto handle = std::get<BufferHandle>(m_args[i]);
                    auto it = buffer_map.find(handle.id);
                    if (it != buffer_map.end()) {
                        // Need to store device pointer and point to it
                        m_device_ptrs.push_back(it->second);
                        arg_ptrs.push_back(&m_device_ptrs.back());
                    } else {
                        arg_ptrs.push_back(nullptr);
                    }
                    break;
                }
                case ArgType::Scalar_I32:
                case ArgType::Scalar_U32:
                case ArgType::Scalar_I64:
                case ArgType::Scalar_U64:
                case ArgType::Scalar_F32:
                case ArgType::Scalar_F64:
                case ArgType::LocalMem:
                    // Store pointer to the variant value
                    arg_ptrs.push_back(const_cast<void*>(
                        static_cast<const void*>(&m_args[i])));
                    break;
                default:
                    arg_ptrs.push_back(nullptr);
                    break;
            }
        }

        return arg_ptrs;
    }

    void clear_device_ptrs() const {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_device_ptrs.clear();
    }

private:
    void query_kernel_attributes() {
        if (!m_function) return;

        // Query kernel attributes using CUDA Driver API
        cuFuncGetAttribute(&m_max_threads_per_block,
                          CU_FUNC_ATTRIBUTE_MAX_THREADS_PER_BLOCK,
                          m_function);
        cuFuncGetAttribute(&m_shared_size_bytes,
                          CU_FUNC_ATTRIBUTE_SHARED_SIZE_BYTES,
                          m_function);
        cuFuncGetAttribute(&m_num_regs,
                          CU_FUNC_ATTRIBUTE_NUM_REGS,
                          m_function);

        // Calculate preferred block size for maximum occupancy
        int block_size = 0;
        int min_grid_size = 0;
        cuOccupancyMaxPotentialBlockSize(&min_grid_size, &block_size,
                                         m_function, nullptr, 0, 0);
        m_preferred_block_size = block_size;
    }

    std::string m_name;
    CUfunction m_function{nullptr};
    CUmodule m_module{nullptr};
    bool m_owns_module{false};

    // Kernel arguments
    mutable std::mutex m_mutex;
    std::vector<KernelArgValue> m_args;
    std::vector<ArgType> m_arg_types;
    mutable std::vector<void*> m_device_ptrs;
    bool m_args_dirty{false};

    // Kernel attributes
    int m_max_threads_per_block{0};
    int m_shared_size_bytes{0};
    int m_num_regs{0};
    int m_preferred_block_size{256};
};

// ============================================================================
// CUDA Backend Implementation
// ============================================================================

class CUDABackend : public IComputeBackend {
public:
    CUDABackend() = default;

    ~CUDABackend() override {
        shutdown();
    }

    // ========================================================================
    // Lifecycle
    // ========================================================================

    BackendResult initialize(const BackendOptions& options) override {
        if (m_initialized) {
            return BackendResult::Success;
        }

        // Initialize CUDA Driver API
        CUresult cu_err = cuInit(0);
        if (cu_err != CUDA_SUCCESS) {
            m_last_error = "Failed to initialize CUDA driver";
            return BackendResult::DeviceNotFound;
        }

        // Check device count
        int device_count = 0;
        cudaError_t err = cudaGetDeviceCount(&device_count);
        if (err != cudaSuccess || device_count == 0) {
            m_last_error = "No CUDA devices found";
            return BackendResult::DeviceNotFound;
        }
        m_device_count = static_cast<UInt32>(device_count);

        // Select device
        UInt32 device_index = options.device_index;
        if (device_index >= m_device_count) {
            device_index = 0;
        }

        CUDA_CHECK(cudaSetDevice(static_cast<int>(device_index)));
        m_current_device = device_index;

        // Get device properties
        cudaDeviceProp props;
        CUDA_CHECK(cudaGetDeviceProperties(&props, static_cast<int>(device_index)));

        // Create CUDA context
        CU_CHECK(cuDeviceGet(&m_cu_device, static_cast<int>(device_index)));
        CU_CHECK(cuCtxCreate(&m_cu_context, 0, m_cu_device));

        // Store options
        m_options = options;

        // Build name string
        m_name = std::string("CUDA ") + std::to_string(props.major) + "." +
                 std::to_string(props.minor) + " (" + props.name + ")";

        m_initialized = true;
        return BackendResult::Success;
    }

    void shutdown() override {
        if (!m_initialized) return;

        // Synchronize all streams
        synchronize();

        // Destroy all streams
        for (auto& [id, stream] : m_streams) {
            if (stream) {
                cudaStreamDestroy(stream);
            }
        }
        m_streams.clear();

        // Destroy all events
        for (auto& [id, event] : m_events) {
            if (event) {
                cudaEventDestroy(event);
            }
        }
        m_events.clear();

        // Free all buffers
        for (auto& [id, info] : m_buffers) {
            if (info.device_ptr) {
                cudaFree(info.device_ptr);
            }
            if (info.host_ptr) {
                cudaFreeHost(info.host_ptr);
            }
        }
        m_buffers.clear();
        m_allocated_memory = 0;

        // Destroy CUDA context
        if (m_cu_context) {
            cuCtxDestroy(m_cu_context);
            m_cu_context = nullptr;
        }

        m_initialized = false;
    }

    bool is_initialized() const override {
        return m_initialized;
    }

    BackendType type() const override {
        return BackendType::CUDA;
    }

    const std::string& name() const override {
        return m_name;
    }

    // ========================================================================
    // Device Management
    // ========================================================================

    UInt32 device_count() const override {
        return m_device_count;
    }

    DeviceCapabilities get_device_capabilities(UInt32 device_index) const override {
        DeviceCapabilities caps;

        if (device_index >= m_device_count) {
            return caps;
        }

        cudaDeviceProp props;
        cudaGetDeviceProperties(&props, static_cast<int>(device_index));

        // Identification
        caps.name = props.name;
        caps.vendor = "NVIDIA";
        caps.driver_version = std::to_string(props.major) + "." +
                              std::to_string(props.minor);
        caps.device_type = props.integrated ? DeviceType::IntegratedGPU
                                            : DeviceType::DiscreteGPU;
        caps.backend_type = BackendType::CUDA;
        caps.device_id = device_index;

        // Memory
        caps.global_memory = props.totalGlobalMem;
        caps.local_memory = props.sharedMemPerBlock;
        caps.constant_memory = props.totalConstMem;
        caps.max_allocation = props.totalGlobalMem;  // CUDA allows large allocs
        caps.memory_bandwidth = static_cast<SizeT>(
            props.memoryBusWidth / 8 * props.memoryClockRate * 2 * 1000ULL);

        // Compute units
        caps.compute_units = static_cast<UInt32>(props.multiProcessorCount);
        caps.max_threads_per_unit = static_cast<UInt32>(props.maxThreadsPerMultiProcessor);
        caps.warp_size = static_cast<UInt32>(props.warpSize);

        // Limits
        caps.max_workgroup_size = static_cast<SizeT>(props.maxThreadsPerBlock);
        caps.max_workgroup_dims[0] = static_cast<UInt32>(props.maxThreadsDim[0]);
        caps.max_workgroup_dims[1] = static_cast<UInt32>(props.maxThreadsDim[1]);
        caps.max_workgroup_dims[2] = static_cast<UInt32>(props.maxThreadsDim[2]);
        caps.max_grid_dims[0] = static_cast<UInt32>(props.maxGridSize[0]);
        caps.max_grid_dims[1] = static_cast<UInt32>(props.maxGridSize[1]);
        caps.max_grid_dims[2] = static_cast<UInt32>(props.maxGridSize[2]);

        // Features
        caps.supports_double = (props.major >= 2);
        caps.supports_atomics = true;
        caps.supports_images = true;
        caps.supports_printf = true;
        caps.supports_unified_memory = (props.managedMemory != 0);

        // Performance hints
        caps.preferred_vector_width_float = 1;  // CUDA uses scalar
        caps.preferred_vector_width_double = 1;
        caps.compute_capability = static_cast<Real>(props.major) +
                                  static_cast<Real>(props.minor) / 10.0;

        return caps;
    }

    UInt32 current_device() const override {
        return m_current_device;
    }

    BackendResult set_device(UInt32 device_index) override {
        if (device_index >= m_device_count) {
            m_last_error = "Device index out of range";
            return BackendResult::DeviceNotFound;
        }

        CUDA_CHECK(cudaSetDevice(static_cast<int>(device_index)));
        m_current_device = device_index;

        return BackendResult::Success;
    }

    // ========================================================================
    // Memory Management
    // ========================================================================

    BufferHandle allocate(SizeT size, MemoryType type,
                          MemoryAccess access) override {
        if (!m_initialized) {
            m_last_error = "Backend not initialized";
            return BufferHandle::Invalid();
        }

        if (size == 0) {
            m_last_error = "Cannot allocate zero-size buffer";
            return BufferHandle::Invalid();
        }

        BufferInfo info;
        info.size = size;
        info.type = type;
        info.access = access;

        cudaError_t err;

        switch (type) {
            case MemoryType::DeviceLocal:
                err = cudaMalloc(&info.device_ptr, size);
                break;

            case MemoryType::HostVisible:
                // Allocate host memory that's page-locked (faster transfers)
                err = cudaMallocHost(&info.host_ptr, size);
                if (err == cudaSuccess) {
                    // Also allocate device memory for staging
                    err = cudaMalloc(&info.device_ptr, size);
                }
                break;

            case MemoryType::HostCached:
                // Same as HostVisible but with caching hint
                err = cudaHostAlloc(&info.host_ptr, size,
                                   cudaHostAllocPortable | cudaHostAllocMapped);
                if (err == cudaSuccess) {
                    err = cudaHostGetDevicePointer(&info.device_ptr,
                                                   info.host_ptr, 0);
                }
                break;

            case MemoryType::Shared:
                // Use CUDA Unified Memory
                err = cudaMallocManaged(&info.device_ptr, size,
                                       cudaMemAttachGlobal);
                info.host_ptr = info.device_ptr;  // Same pointer
                break;

            case MemoryType::Staging:
                // Page-locked host memory for optimal transfer speed
                err = cudaHostAlloc(&info.host_ptr, size,
                                   cudaHostAllocWriteCombined);
                break;

            default:
                m_last_error = "Unknown memory type";
                return BufferHandle::Invalid();
        }

        if (err != cudaSuccess) {
            set_cuda_error(err, "Memory allocation");
            // Clean up partial allocation
            if (info.device_ptr && type != MemoryType::HostCached) {
                cudaFree(info.device_ptr);
            }
            if (info.host_ptr && type != MemoryType::Shared) {
                cudaFreeHost(info.host_ptr);
            }
            return BufferHandle::Invalid();
        }

        BufferHandle handle;
        handle.id = m_next_buffer_id++;
        handle.size = size;
        handle.type = type;
        handle.access = access;

        m_buffers[handle.id] = info;
        m_allocated_memory += size;

        return handle;
    }

    void free(BufferHandle buffer) override {
        auto it = m_buffers.find(buffer.id);
        if (it == m_buffers.end()) return;

        const auto& info = it->second;

        switch (info.type) {
            case MemoryType::DeviceLocal:
                if (info.device_ptr) {
                    cudaFree(info.device_ptr);
                }
                break;

            case MemoryType::HostVisible:
                if (info.device_ptr) cudaFree(info.device_ptr);
                if (info.host_ptr) cudaFreeHost(info.host_ptr);
                break;

            case MemoryType::HostCached:
                if (info.host_ptr) cudaFreeHost(info.host_ptr);
                break;

            case MemoryType::Shared:
                if (info.device_ptr) cudaFree(info.device_ptr);
                break;

            case MemoryType::Staging:
                if (info.host_ptr) cudaFreeHost(info.host_ptr);
                break;
        }

        m_allocated_memory -= info.size;
        m_buffers.erase(it);
    }

    BackendResult upload(BufferHandle buffer, const void* data, SizeT size,
                         SizeT offset, StreamHandle stream) override {
        auto it = m_buffers.find(buffer.id);
        if (it == m_buffers.end()) {
            m_last_error = "Invalid buffer handle";
            return BackendResult::InvalidArgument;
        }

        const auto& info = it->second;

        if (offset + size > info.size) {
            m_last_error = "Upload exceeds buffer size";
            return BackendResult::InvalidArgument;
        }

        void* dst = static_cast<char*>(info.device_ptr) + offset;
        cudaStream_t cuda_stream = get_cuda_stream(stream);

        cudaError_t err;
        if (cuda_stream) {
            err = cudaMemcpyAsync(dst, data, size,
                                 cudaMemcpyHostToDevice, cuda_stream);
        } else {
            err = cudaMemcpy(dst, data, size, cudaMemcpyHostToDevice);
        }

        if (err != cudaSuccess) {
            set_cuda_error(err, "cudaMemcpy upload");
            return BackendResult::InternalError;
        }

        return BackendResult::Success;
    }

    BackendResult download(BufferHandle buffer, void* data, SizeT size,
                           SizeT offset, StreamHandle stream) override {
        auto it = m_buffers.find(buffer.id);
        if (it == m_buffers.end()) {
            m_last_error = "Invalid buffer handle";
            return BackendResult::InvalidArgument;
        }

        const auto& info = it->second;

        if (offset + size > info.size) {
            m_last_error = "Download exceeds buffer size";
            return BackendResult::InvalidArgument;
        }

        void* src = static_cast<char*>(info.device_ptr) + offset;
        cudaStream_t cuda_stream = get_cuda_stream(stream);

        cudaError_t err;
        if (cuda_stream) {
            err = cudaMemcpyAsync(data, src, size,
                                 cudaMemcpyDeviceToHost, cuda_stream);
        } else {
            err = cudaMemcpy(data, src, size, cudaMemcpyDeviceToHost);
        }

        if (err != cudaSuccess) {
            set_cuda_error(err, "cudaMemcpy download");
            return BackendResult::InternalError;
        }

        return BackendResult::Success;
    }

    BackendResult copy(BufferHandle src, BufferHandle dst, SizeT size,
                       SizeT src_offset, SizeT dst_offset,
                       StreamHandle stream) override {
        auto src_it = m_buffers.find(src.id);
        auto dst_it = m_buffers.find(dst.id);

        if (src_it == m_buffers.end() || dst_it == m_buffers.end()) {
            m_last_error = "Invalid buffer handle";
            return BackendResult::InvalidArgument;
        }

        const auto& src_info = src_it->second;
        const auto& dst_info = dst_it->second;

        if (src_offset + size > src_info.size ||
            dst_offset + size > dst_info.size) {
            m_last_error = "Copy exceeds buffer bounds";
            return BackendResult::InvalidArgument;
        }

        void* src_ptr = static_cast<char*>(src_info.device_ptr) + src_offset;
        void* dst_ptr = static_cast<char*>(dst_info.device_ptr) + dst_offset;
        cudaStream_t cuda_stream = get_cuda_stream(stream);

        cudaError_t err;
        if (cuda_stream) {
            err = cudaMemcpyAsync(dst_ptr, src_ptr, size,
                                 cudaMemcpyDeviceToDevice, cuda_stream);
        } else {
            err = cudaMemcpy(dst_ptr, src_ptr, size, cudaMemcpyDeviceToDevice);
        }

        if (err != cudaSuccess) {
            set_cuda_error(err, "cudaMemcpy copy");
            return BackendResult::InternalError;
        }

        return BackendResult::Success;
    }

    BackendResult fill(BufferHandle buffer, const void* pattern,
                       SizeT pattern_size, SizeT size,
                       StreamHandle stream) override {
        auto it = m_buffers.find(buffer.id);
        if (it == m_buffers.end()) {
            m_last_error = "Invalid buffer handle";
            return BackendResult::InvalidArgument;
        }

        const auto& info = it->second;
        SizeT fill_size = size > 0 ? size : info.size;

        if (fill_size > info.size) {
            m_last_error = "Fill size exceeds buffer size";
            return BackendResult::InvalidArgument;
        }

        cudaStream_t cuda_stream = get_cuda_stream(stream);
        cudaError_t err;

        // CUDA only supports memset for single-byte patterns natively
        // For larger patterns, we need to use a kernel or repeated memset
        if (pattern_size == 1) {
            UInt8 value = *static_cast<const UInt8*>(pattern);
            if (cuda_stream) {
                err = cudaMemsetAsync(info.device_ptr, value, fill_size,
                                     cuda_stream);
            } else {
                err = cudaMemset(info.device_ptr, value, fill_size);
            }
        } else if (pattern_size == 4) {
            // Use cudaMemset2D for 4-byte patterns by treating as 2D
            UInt32 value = *static_cast<const UInt32*>(pattern);
            // For 4-byte fills, we can use a simple kernel or repeated operations
            // For now, fall back to host-side filling via Unified Memory or staging
            // This is a simplified implementation - production would use a kernel
            std::vector<UInt8> fill_data(fill_size);
            for (SizeT i = 0; i < fill_size; i += pattern_size) {
                SizeT copy_size = std::min(pattern_size, fill_size - i);
                std::memcpy(fill_data.data() + i, pattern, copy_size);
            }
            err = cudaMemcpy(info.device_ptr, fill_data.data(), fill_size,
                            cudaMemcpyHostToDevice);
        } else {
            // Generic pattern fill
            std::vector<UInt8> fill_data(fill_size);
            for (SizeT i = 0; i < fill_size; i += pattern_size) {
                SizeT copy_size = std::min(pattern_size, fill_size - i);
                std::memcpy(fill_data.data() + i, pattern, copy_size);
            }
            if (cuda_stream) {
                err = cudaMemcpyAsync(info.device_ptr, fill_data.data(),
                                     fill_size, cudaMemcpyHostToDevice,
                                     cuda_stream);
            } else {
                err = cudaMemcpy(info.device_ptr, fill_data.data(),
                                fill_size, cudaMemcpyHostToDevice);
            }
        }

        if (err != cudaSuccess) {
            set_cuda_error(err, "Buffer fill");
            return BackendResult::InternalError;
        }

        return BackendResult::Success;
    }

    void* map(BufferHandle buffer, MemoryAccess /*access*/) override {
        auto it = m_buffers.find(buffer.id);
        if (it == m_buffers.end()) {
            m_last_error = "Invalid buffer handle";
            return nullptr;
        }

        auto& info = it->second;

        // For Shared/HostCached memory, we can return the pointer directly
        if (info.type == MemoryType::Shared ||
            info.type == MemoryType::HostCached) {
            // Synchronize to ensure all GPU work is complete
            cudaDeviceSynchronize();
            info.mapped = true;
            return info.host_ptr;
        }

        // For HostVisible, return the host pointer
        if (info.type == MemoryType::HostVisible && info.host_ptr) {
            // Copy from device to host first
            cudaMemcpy(info.host_ptr, info.device_ptr, info.size,
                      cudaMemcpyDeviceToHost);
            info.mapped = true;
            return info.host_ptr;
        }

        // For DeviceLocal, we need to allocate host memory temporarily
        if (info.type == MemoryType::DeviceLocal) {
            if (!info.host_ptr) {
                cudaError_t err = cudaMallocHost(&info.host_ptr, info.size);
                if (err != cudaSuccess) {
                    m_last_error = "Failed to allocate host memory for mapping";
                    return nullptr;
                }
            }
            cudaMemcpy(info.host_ptr, info.device_ptr, info.size,
                      cudaMemcpyDeviceToHost);
            info.mapped = true;
            return info.host_ptr;
        }

        // For Staging, just return the host pointer
        if (info.type == MemoryType::Staging) {
            info.mapped = true;
            return info.host_ptr;
        }

        m_last_error = "Cannot map buffer of this type";
        return nullptr;
    }

    void unmap(BufferHandle buffer) override {
        auto it = m_buffers.find(buffer.id);
        if (it == m_buffers.end()) return;

        auto& info = it->second;
        if (!info.mapped) return;

        // For HostVisible and DeviceLocal, copy back to device
        if ((info.type == MemoryType::HostVisible ||
             info.type == MemoryType::DeviceLocal) &&
            info.host_ptr && info.device_ptr) {
            cudaMemcpy(info.device_ptr, info.host_ptr, info.size,
                      cudaMemcpyHostToDevice);
        }

        info.mapped = false;
    }

    SizeT allocated_memory() const override {
        return m_allocated_memory;
    }

    SizeT available_memory() const override {
        size_t free_mem = 0, total_mem = 0;
        cudaMemGetInfo(&free_mem, &total_mem);
        return static_cast<SizeT>(free_mem);
    }

    // ========================================================================
    // Kernel Management
    // ========================================================================

    std::unique_ptr<IKernel> create_kernel(
        const std::string& name,
        const std::string& source,
        const std::string& options) override
    {
        if (!m_initialized) {
            m_last_error = "Backend not initialized";
            return nullptr;
        }

        // Create NVRTC program
        nvrtcProgram prog;
        nvrtcResult nvrtc_err = nvrtcCreateProgram(
            &prog, source.c_str(), name.c_str(), 0, nullptr, nullptr);
        if (nvrtc_err != NVRTC_SUCCESS) {
            set_nvrtc_error(nvrtc_err, "nvrtcCreateProgram");
            return nullptr;
        }

        // Build compiler options
        std::vector<const char*> compile_options;
        std::string arch_opt = "--gpu-architecture=compute_" +
                               std::to_string(get_sm_version());
        compile_options.push_back(arch_opt.c_str());
        compile_options.push_back("--std=c++17");
        compile_options.push_back("--device-as-default-execution-space");

        // Add user options
        std::vector<std::string> user_opts;
        if (!options.empty()) {
            std::istringstream iss(options);
            std::string opt;
            while (iss >> opt) {
                user_opts.push_back(opt);
            }
            for (const auto& o : user_opts) {
                compile_options.push_back(o.c_str());
            }
        }

        // Compile the program
        nvrtc_err = nvrtcCompileProgram(prog,
                                        static_cast<int>(compile_options.size()),
                                        compile_options.data());

        if (nvrtc_err != NVRTC_SUCCESS) {
            // Get compilation log
            size_t log_size;
            nvrtcGetProgramLogSize(prog, &log_size);
            std::string log(log_size, '\0');
            nvrtcGetProgramLog(prog, log.data());
            m_last_error = "Kernel compilation failed: " + log;
            nvrtcDestroyProgram(&prog);
            return nullptr;
        }

        // Get PTX
        size_t ptx_size;
        nvrtcGetPTXSize(prog, &ptx_size);
        std::string ptx(ptx_size, '\0');
        nvrtcGetPTX(prog, ptx.data());
        nvrtcDestroyProgram(&prog);

        // Load module from PTX
        CUmodule module;
        CUresult cu_err = cuModuleLoadDataEx(&module, ptx.data(), 0,
                                             nullptr, nullptr);
        if (cu_err != CUDA_SUCCESS) {
            set_cu_error(cu_err, "cuModuleLoadDataEx");
            return nullptr;
        }

        // Get kernel function
        CUfunction func;
        cu_err = cuModuleGetFunction(&func, module, name.c_str());
        if (cu_err != CUDA_SUCCESS) {
            set_cu_error(cu_err, "cuModuleGetFunction");
            cuModuleUnload(module);
            return nullptr;
        }

        return std::make_unique<CUDAKernel>(name, func, module, true);
    }

    std::unique_ptr<IKernel> create_kernel_from_binary(
        const std::string& name,
        std::span<const UInt8> binary) override
    {
        if (!m_initialized) {
            m_last_error = "Backend not initialized";
            return nullptr;
        }

        // Load module from binary (CUBIN or PTX)
        CUmodule module;
        CUresult cu_err = cuModuleLoadData(&module, binary.data());
        if (cu_err != CUDA_SUCCESS) {
            set_cu_error(cu_err, "cuModuleLoadData");
            return nullptr;
        }

        // Get kernel function
        CUfunction func;
        cu_err = cuModuleGetFunction(&func, module, name.c_str());
        if (cu_err != CUDA_SUCCESS) {
            set_cu_error(cu_err, "cuModuleGetFunction");
            cuModuleUnload(module);
            return nullptr;
        }

        return std::make_unique<CUDAKernel>(name, func, module, true);
    }

    BackendResult dispatch(IKernel* kernel, const LaunchConfig& config) override {
        if (!m_initialized) {
            m_last_error = "Backend not initialized";
            return BackendResult::NotInitialized;
        }

        auto* cuda_kernel = dynamic_cast<CUDAKernel*>(kernel);
        if (!cuda_kernel || !cuda_kernel->is_valid()) {
            m_last_error = "Invalid kernel";
            return BackendResult::InvalidArgument;
        }

        // Build buffer map for argument resolution
        std::unordered_map<UInt64, void*> buffer_map;
        for (const auto& [id, info] : m_buffers) {
            buffer_map[id] = info.device_ptr;
        }

        // Prepare kernel arguments
        auto args = cuda_kernel->prepare_args(buffer_map);

        // Get stream
        cudaStream_t cuda_stream = get_cuda_stream(config.stream);

        // Launch kernel
        CUresult err = cuLaunchKernel(
            cuda_kernel->function(),
            static_cast<unsigned int>(config.grid_x),
            static_cast<unsigned int>(config.grid_y),
            static_cast<unsigned int>(config.grid_z),
            static_cast<unsigned int>(config.block_x),
            static_cast<unsigned int>(config.block_y),
            static_cast<unsigned int>(config.block_z),
            static_cast<unsigned int>(config.shared_memory),
            cuda_stream,
            args.data(),
            nullptr
        );

        // Clear temporary device pointers
        cuda_kernel->clear_device_ptrs();

        if (err != CUDA_SUCCESS) {
            set_cu_error(err, "cuLaunchKernel");
            return BackendResult::LaunchFailed;
        }

        return BackendResult::Success;
    }

    // ========================================================================
    // Stream/Queue Management
    // ========================================================================

    StreamHandle create_stream() override {
        cudaStream_t stream;
        cudaError_t err = cudaStreamCreate(&stream);
        if (err != cudaSuccess) {
            set_cuda_error(err, "cudaStreamCreate");
            return StreamHandle::Invalid();
        }

        StreamHandle handle;
        handle.id = m_next_stream_id++;
        m_streams[handle.id] = stream;

        return handle;
    }

    void destroy_stream(StreamHandle stream) override {
        auto it = m_streams.find(stream.id);
        if (it != m_streams.end()) {
            cudaStreamDestroy(it->second);
            m_streams.erase(it);
        }
    }

    BackendResult synchronize_stream(StreamHandle stream) override {
        cudaStream_t cuda_stream = get_cuda_stream(stream);
        if (!cuda_stream && stream.id != StreamHandle::Default().id) {
            m_last_error = "Invalid stream handle";
            return BackendResult::InvalidArgument;
        }

        cudaError_t err = cudaStreamSynchronize(cuda_stream);
        if (err != cudaSuccess) {
            set_cuda_error(err, "cudaStreamSynchronize");
            return BackendResult::SyncFailed;
        }

        return BackendResult::Success;
    }

    BackendResult synchronize() override {
        cudaError_t err = cudaDeviceSynchronize();
        if (err != cudaSuccess) {
            set_cuda_error(err, "cudaDeviceSynchronize");
            return BackendResult::SyncFailed;
        }

        return BackendResult::Success;
    }

    // ========================================================================
    // Events and Synchronization
    // ========================================================================

    EventHandle record_event(StreamHandle stream) override {
        cudaEvent_t event;
        cudaError_t err = cudaEventCreate(&event);
        if (err != cudaSuccess) {
            set_cuda_error(err, "cudaEventCreate");
            return EventHandle{0};
        }

        cudaStream_t cuda_stream = get_cuda_stream(stream);
        err = cudaEventRecord(event, cuda_stream);
        if (err != cudaSuccess) {
            set_cuda_error(err, "cudaEventRecord");
            cudaEventDestroy(event);
            return EventHandle{0};
        }

        EventHandle handle;
        handle.id = m_next_event_id++;
        m_events[handle.id] = event;

        return handle;
    }

    BackendResult wait_event(EventHandle event) override {
        auto it = m_events.find(event.id);
        if (it == m_events.end()) {
            m_last_error = "Invalid event handle";
            return BackendResult::InvalidArgument;
        }

        cudaError_t err = cudaEventSynchronize(it->second);
        if (err != cudaSuccess) {
            set_cuda_error(err, "cudaEventSynchronize");
            return BackendResult::SyncFailed;
        }

        return BackendResult::Success;
    }

    BackendResult stream_wait_event(StreamHandle stream,
                                    EventHandle event) override {
        auto event_it = m_events.find(event.id);
        if (event_it == m_events.end()) {
            m_last_error = "Invalid event handle";
            return BackendResult::InvalidArgument;
        }

        cudaStream_t cuda_stream = get_cuda_stream(stream);
        cudaError_t err = cudaStreamWaitEvent(cuda_stream, event_it->second, 0);
        if (err != cudaSuccess) {
            set_cuda_error(err, "cudaStreamWaitEvent");
            return BackendResult::SyncFailed;
        }

        return BackendResult::Success;
    }

    Real elapsed_time(EventHandle start, EventHandle end) override {
        auto start_it = m_events.find(start.id);
        auto end_it = m_events.find(end.id);

        if (start_it == m_events.end() || end_it == m_events.end()) {
            return 0.0;
        }

        float ms = 0.0f;
        cudaError_t err = cudaEventElapsedTime(&ms, start_it->second,
                                               end_it->second);
        if (err != cudaSuccess) {
            return 0.0;
        }

        return static_cast<Real>(ms);
    }

    void destroy_event(EventHandle event) override {
        auto it = m_events.find(event.id);
        if (it != m_events.end()) {
            cudaEventDestroy(it->second);
            m_events.erase(it);
        }
    }

    // ========================================================================
    // Error Handling
    // ========================================================================

    const std::string& last_error() const override {
        return m_last_error;
    }

    void clear_error() override {
        m_last_error.clear();
        // Also clear CUDA error state
        cudaGetLastError();
    }

    bool has_error() const override {
        return !m_last_error.empty();
    }

private:
    // Helper functions
    void set_cuda_error(cudaError_t err, const char* context) {
        m_last_error = std::string(context) + ": " + cudaGetErrorString(err);
    }

    void set_cu_error(CUresult err, const char* context) {
        const char* err_str = nullptr;
        cuGetErrorString(err, &err_str);
        m_last_error = std::string(context) + ": " +
                       (err_str ? err_str : "Unknown error");
    }

    void set_nvrtc_error(nvrtcResult err, const char* context) {
        m_last_error = std::string(context) + ": " +
                       nvrtcGetErrorString(err);
    }

    int get_sm_version() const {
        cudaDeviceProp props;
        cudaGetDeviceProperties(&props, static_cast<int>(m_current_device));
        return props.major * 10 + props.minor;
    }

    cudaStream_t get_cuda_stream(StreamHandle handle) const {
        if (handle.id == StreamHandle::Default().id) {
            return nullptr;  // Default stream
        }

        auto it = m_streams.find(handle.id);
        if (it != m_streams.end()) {
            return it->second;
        }

        return nullptr;
    }

    // Buffer storage information
    struct BufferInfo {
        void* device_ptr{nullptr};
        void* host_ptr{nullptr};
        SizeT size{0};
        MemoryType type{MemoryType::DeviceLocal};
        MemoryAccess access{MemoryAccess::ReadWrite};
        bool mapped{false};
    };

    // State
    bool m_initialized{false};
    std::string m_name;
    BackendOptions m_options;

    // CUDA handles
    CUdevice m_cu_device{0};
    CUcontext m_cu_context{nullptr};
    UInt32 m_device_count{0};
    UInt32 m_current_device{0};

    // Buffer management
    std::unordered_map<UInt64, BufferInfo> m_buffers;
    std::atomic<UInt64> m_next_buffer_id{1};
    SizeT m_allocated_memory{0};

    // Stream management
    std::unordered_map<UInt64, cudaStream_t> m_streams;
    std::atomic<UInt64> m_next_stream_id{2};  // 1 is default stream

    // Event management
    std::unordered_map<UInt64, cudaEvent_t> m_events;
    std::atomic<UInt64> m_next_event_id{1};

    // Error state
    mutable std::string m_last_error;
};

// ============================================================================
// Backend Creator Function
// ============================================================================

namespace detail {

std::unique_ptr<IComputeBackend> create_cuda_backend() {
    return std::make_unique<CUDABackend>();
}

bool is_cuda_available() {
    int device_count = 0;
    cudaError_t err = cudaGetDeviceCount(&device_count);
    return (err == cudaSuccess && device_count > 0);
}

} // namespace detail

} // namespace jaguar::gpu
