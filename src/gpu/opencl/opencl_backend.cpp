/**
 * @file opencl_backend.cpp
 * @brief OpenCL implementation of IComputeBackend
 *
 * This file provides cross-platform GPU acceleration via OpenCL for JaguarEngine
 * physics computations. It supports:
 * - AMD GPUs (ROCm/AMDGPU-PRO)
 * - Intel GPUs (NEO driver)
 * - NVIDIA GPUs (via NVIDIA OpenCL driver)
 * - Intel/AMD CPUs (via OpenCL CPU runtimes)
 * - FPGAs and other OpenCL accelerators
 *
 * The implementation follows OpenCL 1.2+ specification for broad compatibility
 * while utilizing OpenCL 2.0+ features when available.
 *
 * Requirements:
 * - OpenCL 1.2 or later runtime
 * - OpenCL ICD (Installable Client Driver) loader
 */

#include "jaguar/gpu/compute_backend.h"

#ifdef JAGUAR_HAS_OPENCL

#define CL_TARGET_OPENCL_VERSION 120
#define CL_USE_DEPRECATED_OPENCL_1_2_APIS

#ifdef __APPLE__
#include <OpenCL/opencl.h>
#else
#include <CL/cl.h>
#endif

#include <unordered_map>
#include <atomic>
#include <mutex>
#include <vector>
#include <cstring>
#include <sstream>
#include <algorithm>

namespace jaguar::gpu {

// ============================================================================
// OpenCL Error Checking Utilities
// ============================================================================

static const char* opencl_error_string(cl_int error) {
    switch (error) {
        case CL_SUCCESS:                         return "Success";
        case CL_DEVICE_NOT_FOUND:               return "Device not found";
        case CL_DEVICE_NOT_AVAILABLE:           return "Device not available";
        case CL_COMPILER_NOT_AVAILABLE:         return "Compiler not available";
        case CL_MEM_OBJECT_ALLOCATION_FAILURE:  return "Memory object allocation failure";
        case CL_OUT_OF_RESOURCES:               return "Out of resources";
        case CL_OUT_OF_HOST_MEMORY:             return "Out of host memory";
        case CL_PROFILING_INFO_NOT_AVAILABLE:   return "Profiling info not available";
        case CL_MEM_COPY_OVERLAP:               return "Memory copy overlap";
        case CL_IMAGE_FORMAT_MISMATCH:          return "Image format mismatch";
        case CL_IMAGE_FORMAT_NOT_SUPPORTED:     return "Image format not supported";
        case CL_BUILD_PROGRAM_FAILURE:          return "Build program failure";
        case CL_MAP_FAILURE:                    return "Map failure";
        case CL_INVALID_VALUE:                  return "Invalid value";
        case CL_INVALID_DEVICE_TYPE:            return "Invalid device type";
        case CL_INVALID_PLATFORM:               return "Invalid platform";
        case CL_INVALID_DEVICE:                 return "Invalid device";
        case CL_INVALID_CONTEXT:                return "Invalid context";
        case CL_INVALID_QUEUE_PROPERTIES:       return "Invalid queue properties";
        case CL_INVALID_COMMAND_QUEUE:          return "Invalid command queue";
        case CL_INVALID_HOST_PTR:               return "Invalid host pointer";
        case CL_INVALID_MEM_OBJECT:             return "Invalid memory object";
        case CL_INVALID_IMAGE_FORMAT_DESCRIPTOR: return "Invalid image format descriptor";
        case CL_INVALID_IMAGE_SIZE:             return "Invalid image size";
        case CL_INVALID_SAMPLER:                return "Invalid sampler";
        case CL_INVALID_BINARY:                 return "Invalid binary";
        case CL_INVALID_BUILD_OPTIONS:          return "Invalid build options";
        case CL_INVALID_PROGRAM:                return "Invalid program";
        case CL_INVALID_PROGRAM_EXECUTABLE:     return "Invalid program executable";
        case CL_INVALID_KERNEL_NAME:            return "Invalid kernel name";
        case CL_INVALID_KERNEL_DEFINITION:      return "Invalid kernel definition";
        case CL_INVALID_KERNEL:                 return "Invalid kernel";
        case CL_INVALID_ARG_INDEX:              return "Invalid argument index";
        case CL_INVALID_ARG_VALUE:              return "Invalid argument value";
        case CL_INVALID_ARG_SIZE:               return "Invalid argument size";
        case CL_INVALID_KERNEL_ARGS:            return "Invalid kernel arguments";
        case CL_INVALID_WORK_DIMENSION:         return "Invalid work dimension";
        case CL_INVALID_WORK_GROUP_SIZE:        return "Invalid work group size";
        case CL_INVALID_WORK_ITEM_SIZE:         return "Invalid work item size";
        case CL_INVALID_GLOBAL_OFFSET:          return "Invalid global offset";
        case CL_INVALID_EVENT_WAIT_LIST:        return "Invalid event wait list";
        case CL_INVALID_EVENT:                  return "Invalid event";
        case CL_INVALID_OPERATION:              return "Invalid operation";
        case CL_INVALID_GL_OBJECT:              return "Invalid GL object";
        case CL_INVALID_BUFFER_SIZE:            return "Invalid buffer size";
        case CL_INVALID_MIP_LEVEL:              return "Invalid mip level";
        case CL_INVALID_GLOBAL_WORK_SIZE:       return "Invalid global work size";
        default:                                return "Unknown OpenCL error";
    }
}

#define CL_CHECK(call)                                                         \
    do {                                                                       \
        cl_int err = call;                                                    \
        if (err != CL_SUCCESS) {                                              \
            set_opencl_error(err, #call);                                     \
            return BackendResult::InternalError;                              \
        }                                                                      \
    } while (0)

#define CL_CHECK_RETURN(call, ret)                                            \
    do {                                                                       \
        cl_int err = call;                                                    \
        if (err != CL_SUCCESS) {                                              \
            set_opencl_error(err, #call);                                     \
            return ret;                                                        \
        }                                                                      \
    } while (0)

// ============================================================================
// OpenCL Kernel Implementation
// ============================================================================

/**
 * @brief OpenCL kernel wrapper implementing IKernel interface
 */
class OpenCLKernel : public IKernel {
public:
    OpenCLKernel(const std::string& name, cl_kernel kernel, cl_program program,
                 cl_device_id device)
        : m_name(name)
        , m_kernel(kernel)
        , m_program(program)
        , m_device(device)
        , m_owns_program(false)
    {
        query_kernel_attributes();
    }

    OpenCLKernel(const std::string& name, cl_kernel kernel, cl_program program,
                 cl_device_id device, bool owns_program)
        : m_name(name)
        , m_kernel(kernel)
        , m_program(program)
        , m_device(device)
        , m_owns_program(owns_program)
    {
        query_kernel_attributes();
    }

    ~OpenCLKernel() override {
        if (m_kernel) {
            clReleaseKernel(m_kernel);
        }
        if (m_owns_program && m_program) {
            clReleaseProgram(m_program);
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

        return BackendResult::Success;
    }

    SizeT preferred_workgroup_size() const override {
        return m_preferred_workgroup_size;
    }

    SizeT max_workgroup_size() const override {
        return m_max_workgroup_size;
    }

    SizeT local_memory_size() const override {
        return m_local_mem_size;
    }

    bool is_valid() const override {
        return m_kernel != nullptr;
    }

    cl_kernel kernel() const { return m_kernel; }

    // Bind kernel arguments before dispatch
    BackendResult bind_args(
        const std::unordered_map<UInt64, cl_mem>& buffer_map) const
    {
        std::lock_guard<std::mutex> lock(m_mutex);

        for (size_t i = 0; i < m_args.size(); ++i) {
            cl_int err = CL_SUCCESS;

            switch (m_arg_types[i]) {
                case ArgType::Buffer: {
                    auto handle = std::get<BufferHandle>(m_args[i]);
                    auto it = buffer_map.find(handle.id);
                    if (it != buffer_map.end()) {
                        cl_mem mem = it->second;
                        err = clSetKernelArg(m_kernel,
                                            static_cast<cl_uint>(i),
                                            sizeof(cl_mem), &mem);
                    } else {
                        return BackendResult::InvalidArgument;
                    }
                    break;
                }
                case ArgType::Scalar_I32: {
                    Int32 val = std::get<Int32>(m_args[i]);
                    err = clSetKernelArg(m_kernel,
                                        static_cast<cl_uint>(i),
                                        sizeof(Int32), &val);
                    break;
                }
                case ArgType::Scalar_U32: {
                    UInt32 val = std::get<UInt32>(m_args[i]);
                    err = clSetKernelArg(m_kernel,
                                        static_cast<cl_uint>(i),
                                        sizeof(UInt32), &val);
                    break;
                }
                case ArgType::Scalar_I64: {
                    Int64 val = std::get<Int64>(m_args[i]);
                    err = clSetKernelArg(m_kernel,
                                        static_cast<cl_uint>(i),
                                        sizeof(Int64), &val);
                    break;
                }
                case ArgType::Scalar_U64: {
                    UInt64 val = std::get<UInt64>(m_args[i]);
                    err = clSetKernelArg(m_kernel,
                                        static_cast<cl_uint>(i),
                                        sizeof(UInt64), &val);
                    break;
                }
                case ArgType::Scalar_F32: {
                    float val = std::get<float>(m_args[i]);
                    err = clSetKernelArg(m_kernel,
                                        static_cast<cl_uint>(i),
                                        sizeof(float), &val);
                    break;
                }
                case ArgType::Scalar_F64: {
                    double val = std::get<double>(m_args[i]);
                    err = clSetKernelArg(m_kernel,
                                        static_cast<cl_uint>(i),
                                        sizeof(double), &val);
                    break;
                }
                case ArgType::LocalMem: {
                    SizeT size = std::get<SizeT>(m_args[i]);
                    err = clSetKernelArg(m_kernel,
                                        static_cast<cl_uint>(i),
                                        size, nullptr);
                    break;
                }
                default:
                    return BackendResult::InvalidArgument;
            }

            if (err != CL_SUCCESS) {
                return BackendResult::InvalidArgument;
            }
        }

        return BackendResult::Success;
    }

private:
    void query_kernel_attributes() {
        if (!m_kernel || !m_device) return;

        // Get workgroup info
        size_t wg_size = 0;
        clGetKernelWorkGroupInfo(m_kernel, m_device,
                                CL_KERNEL_WORK_GROUP_SIZE,
                                sizeof(wg_size), &wg_size, nullptr);
        m_max_workgroup_size = static_cast<SizeT>(wg_size);

        // Get preferred workgroup size multiple
        size_t preferred = 0;
        clGetKernelWorkGroupInfo(m_kernel, m_device,
                                CL_KERNEL_PREFERRED_WORK_GROUP_SIZE_MULTIPLE,
                                sizeof(preferred), &preferred, nullptr);
        m_preferred_workgroup_size = static_cast<SizeT>(preferred);

        // Get local memory size
        cl_ulong local_mem = 0;
        clGetKernelWorkGroupInfo(m_kernel, m_device,
                                CL_KERNEL_LOCAL_MEM_SIZE,
                                sizeof(local_mem), &local_mem, nullptr);
        m_local_mem_size = static_cast<SizeT>(local_mem);
    }

    std::string m_name;
    cl_kernel m_kernel{nullptr};
    cl_program m_program{nullptr};
    cl_device_id m_device{nullptr};
    bool m_owns_program{false};

    // Kernel arguments
    mutable std::mutex m_mutex;
    std::vector<KernelArgValue> m_args;
    std::vector<ArgType> m_arg_types;

    // Kernel attributes
    SizeT m_max_workgroup_size{256};
    SizeT m_preferred_workgroup_size{64};
    SizeT m_local_mem_size{0};
};

// ============================================================================
// OpenCL Backend Implementation
// ============================================================================

class OpenCLBackend : public IComputeBackend {
public:
    OpenCLBackend() = default;

    ~OpenCLBackend() override {
        shutdown();
    }

    // ========================================================================
    // Lifecycle
    // ========================================================================

    BackendResult initialize(const BackendOptions& options) override {
        if (m_initialized) {
            return BackendResult::Success;
        }

        // Get platforms
        cl_uint num_platforms = 0;
        cl_int err = clGetPlatformIDs(0, nullptr, &num_platforms);
        if (err != CL_SUCCESS || num_platforms == 0) {
            m_last_error = "No OpenCL platforms found";
            return BackendResult::DeviceNotFound;
        }

        std::vector<cl_platform_id> platforms(num_platforms);
        err = clGetPlatformIDs(num_platforms, platforms.data(), nullptr);
        if (err != CL_SUCCESS) {
            set_opencl_error(err, "clGetPlatformIDs");
            return BackendResult::DeviceNotFound;
        }

        // Find GPU devices across all platforms
        m_devices.clear();
        m_platform_for_device.clear();

        for (cl_platform_id platform : platforms) {
            cl_uint num_devices = 0;
            err = clGetDeviceIDs(platform, CL_DEVICE_TYPE_GPU, 0,
                                nullptr, &num_devices);
            if (err == CL_SUCCESS && num_devices > 0) {
                std::vector<cl_device_id> devices(num_devices);
                clGetDeviceIDs(platform, CL_DEVICE_TYPE_GPU, num_devices,
                              devices.data(), nullptr);
                for (cl_device_id device : devices) {
                    m_devices.push_back(device);
                    m_platform_for_device[device] = platform;
                }
            }

            // Also check for accelerators
            err = clGetDeviceIDs(platform, CL_DEVICE_TYPE_ACCELERATOR, 0,
                                nullptr, &num_devices);
            if (err == CL_SUCCESS && num_devices > 0) {
                std::vector<cl_device_id> devices(num_devices);
                clGetDeviceIDs(platform, CL_DEVICE_TYPE_ACCELERATOR,
                              num_devices, devices.data(), nullptr);
                for (cl_device_id device : devices) {
                    m_devices.push_back(device);
                    m_platform_for_device[device] = platform;
                }
            }
        }

        if (m_devices.empty()) {
            // Fall back to CPU devices if no GPU found
            for (cl_platform_id platform : platforms) {
                cl_uint num_devices = 0;
                err = clGetDeviceIDs(platform, CL_DEVICE_TYPE_CPU, 0,
                                    nullptr, &num_devices);
                if (err == CL_SUCCESS && num_devices > 0) {
                    std::vector<cl_device_id> devices(num_devices);
                    clGetDeviceIDs(platform, CL_DEVICE_TYPE_CPU, num_devices,
                                  devices.data(), nullptr);
                    for (cl_device_id device : devices) {
                        m_devices.push_back(device);
                        m_platform_for_device[device] = platform;
                    }
                }
            }
        }

        if (m_devices.empty()) {
            m_last_error = "No OpenCL devices found";
            return BackendResult::DeviceNotFound;
        }

        // Select device
        UInt32 device_index = options.device_index;
        if (device_index >= m_devices.size()) {
            device_index = 0;
        }
        m_current_device = device_index;

        cl_device_id device = m_devices[device_index];
        cl_platform_id platform = m_platform_for_device[device];

        // Create context
        cl_context_properties props[] = {
            CL_CONTEXT_PLATFORM,
            reinterpret_cast<cl_context_properties>(platform),
            0
        };

        m_context = clCreateContext(props, 1, &device, nullptr, nullptr, &err);
        if (err != CL_SUCCESS) {
            set_opencl_error(err, "clCreateContext");
            return BackendResult::InternalError;
        }

        // Create default command queue
        cl_command_queue_properties queue_props = 0;
        if (options.enable_profiling) {
            queue_props |= CL_QUEUE_PROFILING_ENABLE;
        }

#ifdef CL_VERSION_2_0
        // Try OpenCL 2.0 queue creation
        cl_queue_properties props2[] = {
            CL_QUEUE_PROPERTIES, queue_props,
            0
        };
        m_default_queue = clCreateCommandQueueWithProperties(
            m_context, device, props2, &err);
#else
        m_default_queue = clCreateCommandQueue(
            m_context, device, queue_props, &err);
#endif

        if (err != CL_SUCCESS) {
            set_opencl_error(err, "clCreateCommandQueue");
            clReleaseContext(m_context);
            m_context = nullptr;
            return BackendResult::InternalError;
        }

        m_options = options;

        // Build name string
        char device_name[256];
        clGetDeviceInfo(device, CL_DEVICE_NAME, sizeof(device_name),
                       device_name, nullptr);

        char version[128];
        clGetDeviceInfo(device, CL_DEVICE_VERSION, sizeof(version),
                       version, nullptr);

        m_name = std::string("OpenCL (") + device_name + ", " + version + ")";

        m_initialized = true;
        return BackendResult::Success;
    }

    void shutdown() override {
        if (!m_initialized) return;

        // Synchronize all queues
        synchronize();

        // Destroy all user queues
        for (auto& [id, queue] : m_queues) {
            if (queue) {
                clReleaseCommandQueue(queue);
            }
        }
        m_queues.clear();

        // Destroy all events
        for (auto& [id, event] : m_events) {
            if (event) {
                clReleaseEvent(event);
            }
        }
        m_events.clear();

        // Free all buffers
        for (auto& [id, info] : m_buffers) {
            if (info.mem) {
                clReleaseMemObject(info.mem);
            }
        }
        m_buffers.clear();
        m_allocated_memory = 0;

        // Release default queue and context
        if (m_default_queue) {
            clReleaseCommandQueue(m_default_queue);
            m_default_queue = nullptr;
        }

        if (m_context) {
            clReleaseContext(m_context);
            m_context = nullptr;
        }

        m_initialized = false;
    }

    bool is_initialized() const override {
        return m_initialized;
    }

    BackendType type() const override {
        return BackendType::OpenCL;
    }

    const std::string& name() const override {
        return m_name;
    }

    // ========================================================================
    // Device Management
    // ========================================================================

    UInt32 device_count() const override {
        return static_cast<UInt32>(m_devices.size());
    }

    DeviceCapabilities get_device_capabilities(UInt32 device_index) const override {
        DeviceCapabilities caps;

        if (device_index >= m_devices.size()) {
            return caps;
        }

        cl_device_id device = m_devices[device_index];

        // Device name
        char name[256];
        clGetDeviceInfo(device, CL_DEVICE_NAME, sizeof(name), name, nullptr);
        caps.name = name;

        // Vendor
        char vendor[256];
        clGetDeviceInfo(device, CL_DEVICE_VENDOR, sizeof(vendor), vendor, nullptr);
        caps.vendor = vendor;

        // Driver version
        char version[128];
        clGetDeviceInfo(device, CL_DRIVER_VERSION, sizeof(version), version, nullptr);
        caps.driver_version = version;

        // Device type
        cl_device_type dev_type;
        clGetDeviceInfo(device, CL_DEVICE_TYPE, sizeof(dev_type), &dev_type, nullptr);
        if (dev_type & CL_DEVICE_TYPE_GPU) {
            // Check if integrated
            cl_bool host_unified = CL_FALSE;
            clGetDeviceInfo(device, CL_DEVICE_HOST_UNIFIED_MEMORY,
                           sizeof(host_unified), &host_unified, nullptr);
            caps.device_type = host_unified ? DeviceType::IntegratedGPU
                                            : DeviceType::DiscreteGPU;
        } else if (dev_type & CL_DEVICE_TYPE_CPU) {
            caps.device_type = DeviceType::CPU;
        } else if (dev_type & CL_DEVICE_TYPE_ACCELERATOR) {
            caps.device_type = DeviceType::Accelerator;
        } else {
            caps.device_type = DeviceType::Unknown;
        }

        caps.backend_type = BackendType::OpenCL;
        caps.device_id = device_index;

        // Memory
        cl_ulong global_mem = 0;
        clGetDeviceInfo(device, CL_DEVICE_GLOBAL_MEM_SIZE,
                       sizeof(global_mem), &global_mem, nullptr);
        caps.global_memory = static_cast<SizeT>(global_mem);

        cl_ulong local_mem = 0;
        clGetDeviceInfo(device, CL_DEVICE_LOCAL_MEM_SIZE,
                       sizeof(local_mem), &local_mem, nullptr);
        caps.local_memory = static_cast<SizeT>(local_mem);

        cl_ulong const_mem = 0;
        clGetDeviceInfo(device, CL_DEVICE_MAX_CONSTANT_BUFFER_SIZE,
                       sizeof(const_mem), &const_mem, nullptr);
        caps.constant_memory = static_cast<SizeT>(const_mem);

        cl_ulong max_alloc = 0;
        clGetDeviceInfo(device, CL_DEVICE_MAX_MEM_ALLOC_SIZE,
                       sizeof(max_alloc), &max_alloc, nullptr);
        caps.max_allocation = static_cast<SizeT>(max_alloc);

        // Compute units
        cl_uint compute_units = 0;
        clGetDeviceInfo(device, CL_DEVICE_MAX_COMPUTE_UNITS,
                       sizeof(compute_units), &compute_units, nullptr);
        caps.compute_units = compute_units;

        // AMD wavefront / NVIDIA warp size
        // Try vendor-specific extensions
        size_t wavefront_width = 0;

#ifdef CL_DEVICE_WAVEFRONT_WIDTH_AMD
        if (std::string(vendor).find("AMD") != std::string::npos ||
            std::string(vendor).find("Advanced Micro") != std::string::npos) {
            clGetDeviceInfo(device, CL_DEVICE_WAVEFRONT_WIDTH_AMD,
                           sizeof(wavefront_width), &wavefront_width, nullptr);
        }
#endif

#ifdef CL_DEVICE_WARP_SIZE_NV
        if (std::string(vendor).find("NVIDIA") != std::string::npos) {
            clGetDeviceInfo(device, CL_DEVICE_WARP_SIZE_NV,
                           sizeof(wavefront_width), &wavefront_width, nullptr);
        }
#endif

        if (wavefront_width == 0) {
            // Default based on vendor
            if (std::string(vendor).find("NVIDIA") != std::string::npos) {
                wavefront_width = 32;
            } else if (std::string(vendor).find("AMD") != std::string::npos ||
                       std::string(vendor).find("Advanced Micro") != std::string::npos) {
                wavefront_width = 64;
            } else {
                wavefront_width = 32;  // Common default
            }
        }
        caps.warp_size = static_cast<UInt32>(wavefront_width);

        // Workgroup limits
        size_t max_wg_size = 0;
        clGetDeviceInfo(device, CL_DEVICE_MAX_WORK_GROUP_SIZE,
                       sizeof(max_wg_size), &max_wg_size, nullptr);
        caps.max_workgroup_size = static_cast<SizeT>(max_wg_size);

        size_t max_wg_dims[3];
        clGetDeviceInfo(device, CL_DEVICE_MAX_WORK_ITEM_SIZES,
                       sizeof(max_wg_dims), max_wg_dims, nullptr);
        caps.max_workgroup_dims[0] = static_cast<UInt32>(max_wg_dims[0]);
        caps.max_workgroup_dims[1] = static_cast<UInt32>(max_wg_dims[1]);
        caps.max_workgroup_dims[2] = static_cast<UInt32>(max_wg_dims[2]);

        // OpenCL doesn't have explicit grid size limits
        caps.max_grid_dims[0] = UINT32_MAX;
        caps.max_grid_dims[1] = UINT32_MAX;
        caps.max_grid_dims[2] = UINT32_MAX;

        // Features
        cl_device_fp_config fp_config = 0;
        clGetDeviceInfo(device, CL_DEVICE_DOUBLE_FP_CONFIG,
                       sizeof(fp_config), &fp_config, nullptr);
        caps.supports_double = (fp_config != 0);

        // Check extensions for atomics
        char extensions[4096];
        clGetDeviceInfo(device, CL_DEVICE_EXTENSIONS,
                       sizeof(extensions), extensions, nullptr);
        std::string ext_str(extensions);
        caps.supports_atomics = (ext_str.find("cl_khr_global_int32_base_atomics") != std::string::npos);
        caps.supports_images = true;  // All OpenCL devices support basic images
        caps.supports_printf = (ext_str.find("cl_intel_printf") != std::string::npos ||
                               ext_str.find("cl_amd_printf") != std::string::npos);

        // SVM support (OpenCL 2.0+)
        cl_device_svm_capabilities svm_caps = 0;
#ifdef CL_DEVICE_SVM_CAPABILITIES
        clGetDeviceInfo(device, CL_DEVICE_SVM_CAPABILITIES,
                       sizeof(svm_caps), &svm_caps, nullptr);
#endif
        caps.supports_unified_memory = (svm_caps != 0);

        // Preferred vector widths
        cl_uint vec_width = 0;
        clGetDeviceInfo(device, CL_DEVICE_PREFERRED_VECTOR_WIDTH_FLOAT,
                       sizeof(vec_width), &vec_width, nullptr);
        caps.preferred_vector_width_float = vec_width;

        clGetDeviceInfo(device, CL_DEVICE_PREFERRED_VECTOR_WIDTH_DOUBLE,
                       sizeof(vec_width), &vec_width, nullptr);
        caps.preferred_vector_width_double = vec_width;

        // Compute capability equivalent
        caps.compute_capability = 1.2;  // OpenCL 1.2 baseline

        return caps;
    }

    UInt32 current_device() const override {
        return m_current_device;
    }

    BackendResult set_device(UInt32 device_index) override {
        if (device_index >= m_devices.size()) {
            m_last_error = "Device index out of range";
            return BackendResult::DeviceNotFound;
        }

        if (device_index == m_current_device) {
            return BackendResult::Success;
        }

        // Need to recreate context and queue for new device
        // This is expensive, so warn if called frequently
        m_last_error = "Device switching not yet implemented in OpenCL backend";
        return BackendResult::NotSupported;
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

        cl_mem_flags flags = 0;

        // Set access flags
        switch (access) {
            case MemoryAccess::ReadOnly:
                flags |= CL_MEM_READ_ONLY;
                break;
            case MemoryAccess::WriteOnly:
                flags |= CL_MEM_WRITE_ONLY;
                break;
            case MemoryAccess::ReadWrite:
            default:
                flags |= CL_MEM_READ_WRITE;
                break;
        }

        // Set memory type flags
        BufferInfo info;
        info.size = size;
        info.type = type;
        info.access = access;

        cl_int err;

        switch (type) {
            case MemoryType::DeviceLocal:
                // Standard device memory
                info.mem = clCreateBuffer(m_context, flags, size, nullptr, &err);
                break;

            case MemoryType::HostVisible:
                // Allocate host memory that can be mapped
                flags |= CL_MEM_ALLOC_HOST_PTR;
                info.mem = clCreateBuffer(m_context, flags, size, nullptr, &err);
                break;

            case MemoryType::HostCached:
                // Use host pointer with caching
                flags |= CL_MEM_ALLOC_HOST_PTR;
                info.mem = clCreateBuffer(m_context, flags, size, nullptr, &err);
                break;

            case MemoryType::Shared:
                // Try SVM if available, fall back to host visible
#ifdef CL_MEM_SVM_FINE_GRAIN_BUFFER
                info.svm_ptr = clSVMAlloc(m_context, CL_MEM_READ_WRITE |
                                         CL_MEM_SVM_FINE_GRAIN_BUFFER,
                                         size, 0);
                if (info.svm_ptr) {
                    err = CL_SUCCESS;
                    break;
                }
#endif
                // Fall back to host visible
                flags |= CL_MEM_ALLOC_HOST_PTR;
                info.mem = clCreateBuffer(m_context, flags, size, nullptr, &err);
                break;

            case MemoryType::Staging:
                // Host memory for staging
                flags |= CL_MEM_ALLOC_HOST_PTR;
                info.mem = clCreateBuffer(m_context, flags, size, nullptr, &err);
                break;

            default:
                m_last_error = "Unknown memory type";
                return BufferHandle::Invalid();
        }

        if (err != CL_SUCCESS && !info.svm_ptr) {
            set_opencl_error(err, "clCreateBuffer");
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

        if (info.mem) {
            clReleaseMemObject(info.mem);
        }

#ifdef CL_VERSION_2_0
        if (info.svm_ptr) {
            clSVMFree(m_context, info.svm_ptr);
        }
#endif

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

        cl_command_queue queue = get_queue(stream);
        cl_int err;

#ifdef CL_VERSION_2_0
        if (info.svm_ptr) {
            void* dst = static_cast<char*>(info.svm_ptr) + offset;
            err = clEnqueueSVMMemcpy(queue, CL_TRUE, dst, data, size,
                                    0, nullptr, nullptr);
        } else
#endif
        {
            err = clEnqueueWriteBuffer(queue, info.mem, CL_TRUE,
                                      offset, size, data,
                                      0, nullptr, nullptr);
        }

        if (err != CL_SUCCESS) {
            set_opencl_error(err, "clEnqueueWriteBuffer");
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

        cl_command_queue queue = get_queue(stream);
        cl_int err;

#ifdef CL_VERSION_2_0
        if (info.svm_ptr) {
            void* src = static_cast<char*>(info.svm_ptr) + offset;
            err = clEnqueueSVMMemcpy(queue, CL_TRUE, data, src, size,
                                    0, nullptr, nullptr);
        } else
#endif
        {
            err = clEnqueueReadBuffer(queue, info.mem, CL_TRUE,
                                     offset, size, data,
                                     0, nullptr, nullptr);
        }

        if (err != CL_SUCCESS) {
            set_opencl_error(err, "clEnqueueReadBuffer");
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

        cl_command_queue queue = get_queue(stream);
        cl_int err = clEnqueueCopyBuffer(queue,
                                        src_info.mem, dst_info.mem,
                                        src_offset, dst_offset, size,
                                        0, nullptr, nullptr);

        if (err != CL_SUCCESS) {
            set_opencl_error(err, "clEnqueueCopyBuffer");
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

        cl_command_queue queue = get_queue(stream);

#ifdef CL_VERSION_1_2
        cl_int err = clEnqueueFillBuffer(queue, info.mem,
                                        pattern, pattern_size,
                                        0, fill_size,
                                        0, nullptr, nullptr);
        if (err != CL_SUCCESS) {
            set_opencl_error(err, "clEnqueueFillBuffer");
            return BackendResult::InternalError;
        }
#else
        // OpenCL 1.1 fallback: use write buffer with repeated pattern
        std::vector<UInt8> fill_data(fill_size);
        for (SizeT i = 0; i < fill_size; i += pattern_size) {
            SizeT copy_size = std::min(pattern_size, fill_size - i);
            std::memcpy(fill_data.data() + i, pattern, copy_size);
        }
        cl_int err = clEnqueueWriteBuffer(queue, info.mem, CL_TRUE,
                                         0, fill_size, fill_data.data(),
                                         0, nullptr, nullptr);
        if (err != CL_SUCCESS) {
            set_opencl_error(err, "clEnqueueWriteBuffer (fill fallback)");
            return BackendResult::InternalError;
        }
#endif

        return BackendResult::Success;
    }

    void* map(BufferHandle buffer, MemoryAccess access) override {
        auto it = m_buffers.find(buffer.id);
        if (it == m_buffers.end()) {
            m_last_error = "Invalid buffer handle";
            return nullptr;
        }

        auto& info = it->second;

#ifdef CL_VERSION_2_0
        if (info.svm_ptr) {
            info.mapped = true;
            return info.svm_ptr;
        }
#endif

        cl_map_flags flags = 0;
        switch (access) {
            case MemoryAccess::ReadOnly:
                flags = CL_MAP_READ;
                break;
            case MemoryAccess::WriteOnly:
                flags = CL_MAP_WRITE;
                break;
            case MemoryAccess::ReadWrite:
            default:
                flags = CL_MAP_READ | CL_MAP_WRITE;
                break;
        }

        cl_int err;
        void* ptr = clEnqueueMapBuffer(m_default_queue, info.mem, CL_TRUE,
                                      flags, 0, info.size,
                                      0, nullptr, nullptr, &err);
        if (err != CL_SUCCESS) {
            set_opencl_error(err, "clEnqueueMapBuffer");
            return nullptr;
        }

        info.mapped = true;
        info.mapped_ptr = ptr;

        return ptr;
    }

    void unmap(BufferHandle buffer) override {
        auto it = m_buffers.find(buffer.id);
        if (it == m_buffers.end()) return;

        auto& info = it->second;
        if (!info.mapped) return;

#ifdef CL_VERSION_2_0
        if (info.svm_ptr) {
            info.mapped = false;
            return;
        }
#endif

        if (info.mapped_ptr) {
            clEnqueueUnmapMemObject(m_default_queue, info.mem,
                                   info.mapped_ptr, 0, nullptr, nullptr);
            clFinish(m_default_queue);
        }

        info.mapped = false;
        info.mapped_ptr = nullptr;
    }

    SizeT allocated_memory() const override {
        return m_allocated_memory;
    }

    SizeT available_memory() const override {
        if (m_devices.empty()) return 0;

        cl_ulong mem_size = 0;
        clGetDeviceInfo(m_devices[m_current_device], CL_DEVICE_GLOBAL_MEM_SIZE,
                       sizeof(mem_size), &mem_size, nullptr);

        // Rough estimate of available
        return static_cast<SizeT>(mem_size) - m_allocated_memory;
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

        const char* src_ptr = source.c_str();
        size_t src_len = source.length();

        cl_int err;
        cl_program program = clCreateProgramWithSource(m_context, 1,
                                                       &src_ptr, &src_len,
                                                       &err);
        if (err != CL_SUCCESS) {
            set_opencl_error(err, "clCreateProgramWithSource");
            return nullptr;
        }

        cl_device_id device = m_devices[m_current_device];

        // Build options
        std::string build_opts = "-cl-std=CL1.2";
        if (!options.empty()) {
            build_opts += " " + options;
        }

        err = clBuildProgram(program, 1, &device, build_opts.c_str(),
                            nullptr, nullptr);
        if (err != CL_SUCCESS) {
            // Get build log
            size_t log_size;
            clGetProgramBuildInfo(program, device, CL_PROGRAM_BUILD_LOG,
                                 0, nullptr, &log_size);
            std::string log(log_size, '\0');
            clGetProgramBuildInfo(program, device, CL_PROGRAM_BUILD_LOG,
                                 log_size, log.data(), nullptr);
            m_last_error = "Kernel build failed: " + log;
            clReleaseProgram(program);
            return nullptr;
        }

        cl_kernel kernel = clCreateKernel(program, name.c_str(), &err);
        if (err != CL_SUCCESS) {
            set_opencl_error(err, "clCreateKernel");
            clReleaseProgram(program);
            return nullptr;
        }

        return std::make_unique<OpenCLKernel>(name, kernel, program,
                                              device, true);
    }

    std::unique_ptr<IKernel> create_kernel_from_binary(
        const std::string& name,
        std::span<const UInt8> binary) override
    {
        if (!m_initialized) {
            m_last_error = "Backend not initialized";
            return nullptr;
        }

        cl_device_id device = m_devices[m_current_device];
        const size_t bin_size = binary.size();
        const unsigned char* bin_ptr = binary.data();

        cl_int binary_status;
        cl_int err;
        cl_program program = clCreateProgramWithBinary(m_context, 1, &device,
                                                       &bin_size, &bin_ptr,
                                                       &binary_status, &err);
        if (err != CL_SUCCESS || binary_status != CL_SUCCESS) {
            set_opencl_error(err, "clCreateProgramWithBinary");
            return nullptr;
        }

        err = clBuildProgram(program, 1, &device, nullptr, nullptr, nullptr);
        if (err != CL_SUCCESS) {
            clReleaseProgram(program);
            set_opencl_error(err, "clBuildProgram (binary)");
            return nullptr;
        }

        cl_kernel kernel = clCreateKernel(program, name.c_str(), &err);
        if (err != CL_SUCCESS) {
            set_opencl_error(err, "clCreateKernel");
            clReleaseProgram(program);
            return nullptr;
        }

        return std::make_unique<OpenCLKernel>(name, kernel, program,
                                              device, true);
    }

    BackendResult dispatch(IKernel* kernel, const LaunchConfig& config) override {
        if (!m_initialized) {
            m_last_error = "Backend not initialized";
            return BackendResult::NotInitialized;
        }

        auto* cl_kernel = dynamic_cast<OpenCLKernel*>(kernel);
        if (!cl_kernel || !cl_kernel->is_valid()) {
            m_last_error = "Invalid kernel";
            return BackendResult::InvalidArgument;
        }

        // Build buffer map for argument binding
        std::unordered_map<UInt64, cl_mem> buffer_map;
        for (const auto& [id, info] : m_buffers) {
            buffer_map[id] = info.mem;
        }

        // Bind kernel arguments
        auto result = cl_kernel->bind_args(buffer_map);
        if (result != BackendResult::Success) {
            m_last_error = "Failed to bind kernel arguments";
            return result;
        }

        // Get command queue
        cl_command_queue queue = get_queue(config.stream);

        // Calculate global and local work sizes
        size_t global_work_size[3] = {
            config.grid_x * config.block_x,
            config.grid_y * config.block_y,
            config.grid_z * config.block_z
        };

        size_t local_work_size[3] = {
            config.block_x,
            config.block_y,
            config.block_z
        };

        // Determine work dimensions
        cl_uint work_dim = 1;
        if (config.grid_y > 1 || config.block_y > 1) work_dim = 2;
        if (config.grid_z > 1 || config.block_z > 1) work_dim = 3;

        // Launch kernel
        cl_int err = clEnqueueNDRangeKernel(queue, cl_kernel->kernel(),
                                           work_dim, nullptr,
                                           global_work_size,
                                           local_work_size,
                                           0, nullptr, nullptr);

        if (err != CL_SUCCESS) {
            set_opencl_error(err, "clEnqueueNDRangeKernel");
            return BackendResult::LaunchFailed;
        }

        return BackendResult::Success;
    }

    // ========================================================================
    // Stream/Queue Management
    // ========================================================================

    StreamHandle create_stream() override {
        cl_device_id device = m_devices[m_current_device];
        cl_int err;

        cl_command_queue_properties props = 0;
        if (m_options.enable_profiling) {
            props |= CL_QUEUE_PROFILING_ENABLE;
        }

#ifdef CL_VERSION_2_0
        cl_queue_properties props2[] = {
            CL_QUEUE_PROPERTIES, props,
            0
        };
        cl_command_queue queue = clCreateCommandQueueWithProperties(
            m_context, device, props2, &err);
#else
        cl_command_queue queue = clCreateCommandQueue(
            m_context, device, props, &err);
#endif

        if (err != CL_SUCCESS) {
            set_opencl_error(err, "clCreateCommandQueue");
            return StreamHandle::Invalid();
        }

        StreamHandle handle;
        handle.id = m_next_stream_id++;
        m_queues[handle.id] = queue;

        return handle;
    }

    void destroy_stream(StreamHandle stream) override {
        auto it = m_queues.find(stream.id);
        if (it != m_queues.end()) {
            clReleaseCommandQueue(it->second);
            m_queues.erase(it);
        }
    }

    BackendResult synchronize_stream(StreamHandle stream) override {
        cl_command_queue queue = get_queue(stream);
        cl_int err = clFinish(queue);

        if (err != CL_SUCCESS) {
            set_opencl_error(err, "clFinish");
            return BackendResult::SyncFailed;
        }

        return BackendResult::Success;
    }

    BackendResult synchronize() override {
        // Finish default queue
        cl_int err = clFinish(m_default_queue);
        if (err != CL_SUCCESS) {
            set_opencl_error(err, "clFinish");
            return BackendResult::SyncFailed;
        }

        // Finish all user queues
        for (auto& [id, queue] : m_queues) {
            err = clFinish(queue);
            if (err != CL_SUCCESS) {
                set_opencl_error(err, "clFinish");
                return BackendResult::SyncFailed;
            }
        }

        return BackendResult::Success;
    }

    // ========================================================================
    // Events and Synchronization
    // ========================================================================

    EventHandle record_event(StreamHandle stream) override {
        cl_command_queue queue = get_queue(stream);

        // Create a marker event
        cl_event event;
#ifdef CL_VERSION_1_2
        cl_int err = clEnqueueMarkerWithWaitList(queue, 0, nullptr, &event);
#else
        cl_int err = clEnqueueMarker(queue, &event);
#endif

        if (err != CL_SUCCESS) {
            set_opencl_error(err, "clEnqueueMarker");
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

        cl_int err = clWaitForEvents(1, &it->second);
        if (err != CL_SUCCESS) {
            set_opencl_error(err, "clWaitForEvents");
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

        cl_command_queue queue = get_queue(stream);

#ifdef CL_VERSION_1_2
        cl_int err = clEnqueueBarrierWithWaitList(queue, 1, &event_it->second,
                                                  nullptr);
#else
        cl_int err = clEnqueueWaitForEvents(queue, 1, &event_it->second);
#endif

        if (err != CL_SUCCESS) {
            set_opencl_error(err, "clEnqueueBarrierWithWaitList");
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

        // Ensure both events have completed
        clWaitForEvents(1, &start_it->second);
        clWaitForEvents(1, &end_it->second);

        cl_ulong start_time = 0, end_time = 0;

        cl_int err = clGetEventProfilingInfo(start_it->second,
                                            CL_PROFILING_COMMAND_END,
                                            sizeof(start_time),
                                            &start_time, nullptr);
        if (err != CL_SUCCESS) return 0.0;

        err = clGetEventProfilingInfo(end_it->second,
                                     CL_PROFILING_COMMAND_END,
                                     sizeof(end_time),
                                     &end_time, nullptr);
        if (err != CL_SUCCESS) return 0.0;

        // Convert nanoseconds to milliseconds
        return static_cast<Real>(end_time - start_time) / 1000000.0;
    }

    void destroy_event(EventHandle event) override {
        auto it = m_events.find(event.id);
        if (it != m_events.end()) {
            clReleaseEvent(it->second);
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
    }

    bool has_error() const override {
        return !m_last_error.empty();
    }

private:
    void set_opencl_error(cl_int err, const char* context) {
        m_last_error = std::string(context) + ": " + opencl_error_string(err);
    }

    cl_command_queue get_queue(StreamHandle stream) const {
        if (stream.id == StreamHandle::Default().id || stream.id == 0) {
            return m_default_queue;
        }

        auto it = m_queues.find(stream.id);
        if (it != m_queues.end()) {
            return it->second;
        }

        return m_default_queue;
    }

    // Buffer storage information
    struct BufferInfo {
        cl_mem mem{nullptr};
        void* svm_ptr{nullptr};
        void* mapped_ptr{nullptr};
        SizeT size{0};
        MemoryType type{MemoryType::DeviceLocal};
        MemoryAccess access{MemoryAccess::ReadWrite};
        bool mapped{false};
    };

    // State
    bool m_initialized{false};
    std::string m_name;
    BackendOptions m_options;

    // OpenCL handles
    cl_context m_context{nullptr};
    cl_command_queue m_default_queue{nullptr};
    std::vector<cl_device_id> m_devices;
    std::unordered_map<cl_device_id, cl_platform_id> m_platform_for_device;
    UInt32 m_current_device{0};

    // Buffer management
    std::unordered_map<UInt64, BufferInfo> m_buffers;
    std::atomic<UInt64> m_next_buffer_id{1};
    SizeT m_allocated_memory{0};

    // Queue management
    std::unordered_map<UInt64, cl_command_queue> m_queues;
    std::atomic<UInt64> m_next_stream_id{2};  // 1 is default

    // Event management
    std::unordered_map<UInt64, cl_event> m_events;
    std::atomic<UInt64> m_next_event_id{1};

    // Error state
    mutable std::string m_last_error;
};

// ============================================================================
// Backend Creator Functions
// ============================================================================

namespace detail {

std::unique_ptr<IComputeBackend> create_opencl_backend() {
    return std::make_unique<OpenCLBackend>();
}

bool is_opencl_available() {
    cl_uint num_platforms = 0;
    cl_int err = clGetPlatformIDs(0, nullptr, &num_platforms);
    if (err != CL_SUCCESS || num_platforms == 0) {
        return false;
    }

    // Check for any GPU or accelerator device
    std::vector<cl_platform_id> platforms(num_platforms);
    clGetPlatformIDs(num_platforms, platforms.data(), nullptr);

    for (cl_platform_id platform : platforms) {
        cl_uint num_devices = 0;
        err = clGetDeviceIDs(platform, CL_DEVICE_TYPE_GPU, 0,
                            nullptr, &num_devices);
        if (err == CL_SUCCESS && num_devices > 0) {
            return true;
        }

        err = clGetDeviceIDs(platform, CL_DEVICE_TYPE_ACCELERATOR, 0,
                            nullptr, &num_devices);
        if (err == CL_SUCCESS && num_devices > 0) {
            return true;
        }

        err = clGetDeviceIDs(platform, CL_DEVICE_TYPE_CPU, 0,
                            nullptr, &num_devices);
        if (err == CL_SUCCESS && num_devices > 0) {
            return true;
        }
    }

    return false;
}

} // namespace detail

} // namespace jaguar::gpu

#endif // JAGUAR_HAS_OPENCL
