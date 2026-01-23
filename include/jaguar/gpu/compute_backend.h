#pragma once
/**
 * @file compute_backend.h
 * @brief Unified compute backend abstraction for GPU acceleration
 *
 * This file defines the core abstraction layer for GPU compute backends,
 * supporting multiple implementations:
 * - CUDA (NVIDIA GPUs)
 * - OpenCL (Cross-platform)
 * - Metal (Apple Silicon)
 * - CPU (Fallback/Reference)
 *
 * The abstraction provides:
 * - Device enumeration and selection
 * - Memory allocation and transfer
 * - Kernel compilation and dispatch
 * - Async execution with streams/queues
 * - Error handling and recovery
 *
 * Design Principles:
 * 1. Backend-agnostic interface for physics code
 * 2. Zero-copy where possible (mapped memory)
 * 3. Async by default for maximum throughput
 * 4. Graceful fallback to CPU when GPU unavailable
 *
 * Usage:
 * @code
 * auto backend = BackendFactory::create(BackendType::CUDA);
 * if (!backend || !backend->initialize()) {
 *     backend = BackendFactory::create(BackendType::CPU);
 * }
 *
 * BufferHandle positions = backend->allocate(sizeof(Vec3) * n, MemoryType::DeviceLocal);
 * backend->upload(positions, host_data, sizeof(Vec3) * n);
 *
 * auto kernel = backend->create_kernel("integrate_positions", source);
 * kernel->set_arg(0, positions);
 * kernel->set_arg(1, dt);
 * backend->dispatch(kernel, {n, 1, 1}, {256, 1, 1});
 *
 * backend->download(positions, host_data, sizeof(Vec3) * n);
 * @endcode
 */

#include "jaguar/core/types.h"
#include <string>
#include <memory>
#include <vector>
#include <functional>
#include <variant>
#include <optional>
#include <span>

namespace jaguar::gpu {

// ============================================================================
// Forward Declarations
// ============================================================================

class IComputeBackend;
class IKernel;
class IStream;
struct DeviceCapabilities;
struct LaunchConfig;

// ============================================================================
// Enumerations
// ============================================================================

/**
 * @brief Supported compute backend types
 */
enum class BackendType : UInt8 {
    CPU     = 0,    ///< CPU reference implementation (always available)
    CUDA    = 1,    ///< NVIDIA CUDA
    OpenCL  = 2,    ///< OpenCL (cross-platform)
    Metal   = 3,    ///< Apple Metal
    Vulkan  = 4,    ///< Vulkan Compute (future)
    Auto    = 255   ///< Automatic selection (best available)
};

/**
 * @brief Memory allocation type
 */
enum class MemoryType : UInt8 {
    DeviceLocal     = 0,    ///< GPU-only memory (fastest for compute)
    HostVisible     = 1,    ///< CPU-accessible GPU memory (slower)
    HostCached      = 2,    ///< CPU-cached memory (for frequent reads)
    Shared          = 3,    ///< Unified memory (CPU/GPU shared)
    Staging         = 4     ///< Transfer buffer (upload/download)
};

/**
 * @brief Memory access flags
 */
enum class MemoryAccess : UInt8 {
    ReadOnly    = 0x01,
    WriteOnly   = 0x02,
    ReadWrite   = 0x03
};

/**
 * @brief Kernel argument data type
 */
enum class ArgType : UInt8 {
    Buffer      = 0,    ///< Buffer handle
    Scalar_I32  = 1,    ///< 32-bit integer
    Scalar_U32  = 2,    ///< 32-bit unsigned integer
    Scalar_I64  = 3,    ///< 64-bit integer
    Scalar_U64  = 4,    ///< 64-bit unsigned integer
    Scalar_F32  = 5,    ///< 32-bit float
    Scalar_F64  = 6,    ///< 64-bit double
    LocalMem    = 7,    ///< Local/shared memory size
    Image       = 8,    ///< Image/texture handle
    Sampler     = 9     ///< Sampler object
};

/**
 * @brief Backend operation result
 */
enum class BackendResult : UInt8 {
    Success             = 0,
    DeviceNotFound      = 1,
    OutOfMemory         = 2,
    InvalidArgument     = 3,
    CompilationFailed   = 4,
    LaunchFailed        = 5,
    SyncFailed          = 6,
    NotInitialized      = 7,
    NotSupported        = 8,
    InternalError       = 255
};

/**
 * @brief Device type classification
 */
enum class DeviceType : UInt8 {
    Unknown         = 0,
    CPU             = 1,
    DiscreteGPU     = 2,    ///< Dedicated GPU (high performance)
    IntegratedGPU   = 3,    ///< Integrated GPU (power efficient)
    Accelerator     = 4     ///< Specialized accelerator (TPU, etc.)
};

// ============================================================================
// Handle Types
// ============================================================================

/**
 * @brief Opaque handle to a GPU buffer
 *
 * Buffer handles are lightweight references to GPU memory allocations.
 * The actual memory is managed by the backend.
 */
struct BufferHandle {
    UInt64 id{0};           ///< Unique buffer identifier
    SizeT size{0};          ///< Buffer size in bytes
    MemoryType type{MemoryType::DeviceLocal};
    MemoryAccess access{MemoryAccess::ReadWrite};

    bool is_valid() const { return id != 0; }
    operator bool() const { return is_valid(); }

    static constexpr BufferHandle Invalid() { return BufferHandle{}; }
};

/**
 * @brief Opaque handle to a compute stream/queue
 */
struct StreamHandle {
    UInt64 id{0};
    bool is_valid() const { return id != 0; }
    operator bool() const { return is_valid(); }

    static constexpr StreamHandle Default() { return StreamHandle{1}; }
    static constexpr StreamHandle Invalid() { return StreamHandle{}; }
};

/**
 * @brief Event handle for synchronization
 */
struct EventHandle {
    UInt64 id{0};
    bool is_valid() const { return id != 0; }
    operator bool() const { return is_valid(); }
};

// ============================================================================
// Configuration Structures
// ============================================================================

/**
 * @brief Kernel launch configuration
 */
struct LaunchConfig {
    // Grid dimensions (global work size)
    SizeT grid_x{1};
    SizeT grid_y{1};
    SizeT grid_z{1};

    // Block/workgroup dimensions (local work size)
    SizeT block_x{1};
    SizeT block_y{1};
    SizeT block_z{1};

    // Shared memory per block (bytes)
    SizeT shared_memory{0};

    // Stream to execute on
    StreamHandle stream{StreamHandle::Default()};

    /**
     * @brief Create 1D launch config
     */
    static LaunchConfig Linear(SizeT total_threads, SizeT block_size = 256) {
        LaunchConfig config;
        config.grid_x = (total_threads + block_size - 1) / block_size;
        config.block_x = block_size;
        return config;
    }

    /**
     * @brief Create 2D launch config
     */
    static LaunchConfig Grid2D(SizeT width, SizeT height,
                                SizeT block_x = 16, SizeT block_y = 16) {
        LaunchConfig config;
        config.grid_x = (width + block_x - 1) / block_x;
        config.grid_y = (height + block_y - 1) / block_y;
        config.block_x = block_x;
        config.block_y = block_y;
        return config;
    }

    /**
     * @brief Create 3D launch config
     */
    static LaunchConfig Grid3D(SizeT x, SizeT y, SizeT z,
                                SizeT bx = 8, SizeT by = 8, SizeT bz = 8) {
        LaunchConfig config;
        config.grid_x = (x + bx - 1) / bx;
        config.grid_y = (y + by - 1) / by;
        config.grid_z = (z + bz - 1) / bz;
        config.block_x = bx;
        config.block_y = by;
        config.block_z = bz;
        return config;
    }

    /**
     * @brief Get total number of threads
     */
    SizeT total_threads() const {
        return grid_x * grid_y * grid_z * block_x * block_y * block_z;
    }
};

/**
 * @brief Device capabilities and properties
 */
struct DeviceCapabilities {
    // Identification
    std::string name;
    std::string vendor;
    std::string driver_version;
    DeviceType device_type{DeviceType::Unknown};
    BackendType backend_type{BackendType::CPU};
    UInt32 device_id{0};

    // Memory
    SizeT global_memory{0};         ///< Total global memory (bytes)
    SizeT local_memory{0};          ///< Per-workgroup local memory (bytes)
    SizeT constant_memory{0};       ///< Constant memory size (bytes)
    SizeT max_allocation{0};        ///< Maximum single allocation (bytes)
    SizeT memory_bandwidth{0};      ///< Memory bandwidth (bytes/sec)

    // Compute units
    UInt32 compute_units{0};        ///< Number of compute units/SMs
    UInt32 max_threads_per_unit{0}; ///< Max threads per compute unit
    UInt32 warp_size{0};            ///< Warp/wavefront size (32 for NVIDIA, 64 for AMD)

    // Limits
    SizeT max_workgroup_size{0};    ///< Maximum workgroup/block size
    UInt32 max_workgroup_dims[3]{0, 0, 0};  ///< Max dimensions
    UInt32 max_grid_dims[3]{0, 0, 0};       ///< Max grid dimensions

    // Features
    bool supports_double{false};    ///< Double precision support
    bool supports_atomics{false};   ///< Atomic operations
    bool supports_images{false};    ///< Image/texture support
    bool supports_printf{false};    ///< Printf from kernels
    bool supports_unified_memory{false};  ///< Unified addressing

    // Performance hints
    UInt32 preferred_vector_width_float{0};
    UInt32 preferred_vector_width_double{0};
    Real compute_capability{0.0};   ///< CUDA compute capability or equivalent

    /**
     * @brief Get theoretical FLOPS (single precision)
     */
    Real theoretical_flops_sp() const {
        // Rough estimate: CUs * threads * 2 (FMA) * clock
        // This is a simplified estimate; actual varies by architecture
        return static_cast<Real>(compute_units) *
               static_cast<Real>(max_threads_per_unit) * 2.0;
    }

    /**
     * @brief Check if device meets minimum requirements
     */
    bool meets_requirements(SizeT min_memory, UInt32 min_compute_units = 1) const {
        return global_memory >= min_memory && compute_units >= min_compute_units;
    }
};

/**
 * @brief Backend initialization options
 */
struct BackendOptions {
    UInt32 device_index{0};         ///< Preferred device index
    bool enable_profiling{false};   ///< Enable kernel profiling
    bool enable_debugging{false};   ///< Enable debug output
    bool prefer_unified_memory{false}; ///< Prefer unified memory if available
    SizeT memory_pool_size{0};      ///< Pre-allocated memory pool (0 = default)
    std::string cache_path;         ///< Path for kernel cache
};

// ============================================================================
// Kernel Argument
// ============================================================================

/**
 * @brief Kernel argument value (type-safe union)
 *
 * Note: We use explicit types to avoid duplicate types in variant when
 * UInt64 and SizeT resolve to the same underlying type on 64-bit platforms.
 */
using KernelArgValue = std::variant<
    BufferHandle,       // Buffer
    Int32,              // Scalar_I32
    UInt32,             // Scalar_U32
    Int64,              // Scalar_I64
    UInt64,             // Scalar_U64 and LocalMem size (same as SizeT on 64-bit)
    float,              // Scalar_F32
    double              // Scalar_F64
>;

/**
 * @brief Kernel argument with type information
 */
struct KernelArg {
    ArgType type;
    KernelArgValue value;

    // Default constructor
    KernelArg() : type(ArgType::Buffer), value(BufferHandle{}) {}

    // Explicit constructor
    KernelArg(ArgType t, KernelArgValue v) : type(t), value(std::move(v)) {}

    // Convenience constructors
    static KernelArg Buffer(BufferHandle h) {
        return KernelArg{ArgType::Buffer, KernelArgValue{h}};
    }
    static KernelArg Int(Int32 v) {
        return KernelArg{ArgType::Scalar_I32, KernelArgValue{v}};
    }
    static KernelArg UInt(UInt32 v) {
        return KernelArg{ArgType::Scalar_U32, KernelArgValue{v}};
    }
    static KernelArg Long(Int64 v) {
        return KernelArg{ArgType::Scalar_I64, KernelArgValue{v}};
    }
    static KernelArg ULong(UInt64 v) {
        return KernelArg{ArgType::Scalar_U64, KernelArgValue{v}};
    }
    static KernelArg Float(float v) {
        return KernelArg{ArgType::Scalar_F32, KernelArgValue{v}};
    }
    static KernelArg Double(double v) {
        return KernelArg{ArgType::Scalar_F64, KernelArgValue{v}};
    }
    static KernelArg LocalMemory(SizeT size) {
        // SizeT is typically UInt64 on 64-bit platforms
        return KernelArg{ArgType::LocalMem, KernelArgValue{static_cast<UInt64>(size)}};
    }
};

// ============================================================================
// Kernel Interface
// ============================================================================

/**
 * @brief Abstract interface for compiled compute kernels
 */
class IKernel {
public:
    virtual ~IKernel() = default;

    /**
     * @brief Get kernel name
     */
    virtual const std::string& name() const = 0;

    /**
     * @brief Set kernel argument by index
     */
    virtual BackendResult set_arg(UInt32 index, const KernelArg& arg) = 0;

    /**
     * @brief Set multiple arguments
     */
    virtual BackendResult set_args(std::span<const KernelArg> args) {
        for (UInt32 i = 0; i < args.size(); ++i) {
            auto result = set_arg(i, args[i]);
            if (result != BackendResult::Success) return result;
        }
        return BackendResult::Success;
    }

    /**
     * @brief Get preferred workgroup size
     */
    virtual SizeT preferred_workgroup_size() const = 0;

    /**
     * @brief Get maximum workgroup size for this kernel
     */
    virtual SizeT max_workgroup_size() const = 0;

    /**
     * @brief Get local memory usage
     */
    virtual SizeT local_memory_size() const = 0;

    /**
     * @brief Get number of registers per thread
     */
    virtual UInt32 register_count() const { return 0; }

    /**
     * @brief Check if kernel is valid and ready
     */
    virtual bool is_valid() const = 0;
};

// ============================================================================
// Compute Backend Interface
// ============================================================================

/**
 * @brief Abstract interface for compute backends
 *
 * IComputeBackend provides a unified interface for GPU compute operations
 * across different platforms (CUDA, OpenCL, Metal, CPU).
 */
class IComputeBackend {
public:
    virtual ~IComputeBackend() = default;

    // ========================================================================
    // Lifecycle
    // ========================================================================

    /**
     * @brief Initialize the backend
     * @param options Initialization options
     * @return Success or error code
     */
    virtual BackendResult initialize(const BackendOptions& options = {}) = 0;

    /**
     * @brief Shutdown the backend and release resources
     */
    virtual void shutdown() = 0;

    /**
     * @brief Check if backend is initialized
     */
    virtual bool is_initialized() const = 0;

    /**
     * @brief Get backend type
     */
    virtual BackendType type() const = 0;

    /**
     * @brief Get backend name (e.g., "CUDA 12.0", "OpenCL 3.0")
     */
    virtual const std::string& name() const = 0;

    // ========================================================================
    // Device Management
    // ========================================================================

    /**
     * @brief Get number of available devices
     */
    virtual UInt32 device_count() const = 0;

    /**
     * @brief Get capabilities of a device
     * @param device_index Device index (0-based)
     */
    virtual DeviceCapabilities get_device_capabilities(UInt32 device_index = 0) const = 0;

    /**
     * @brief Get current device index
     */
    virtual UInt32 current_device() const = 0;

    /**
     * @brief Set active device
     */
    virtual BackendResult set_device(UInt32 device_index) = 0;

    // ========================================================================
    // Memory Management
    // ========================================================================

    /**
     * @brief Allocate device memory
     * @param size Size in bytes
     * @param type Memory type
     * @param access Access flags
     * @return Buffer handle or invalid handle on failure
     */
    virtual BufferHandle allocate(SizeT size,
                                   MemoryType type = MemoryType::DeviceLocal,
                                   MemoryAccess access = MemoryAccess::ReadWrite) = 0;

    /**
     * @brief Free device memory
     */
    virtual void free(BufferHandle buffer) = 0;

    /**
     * @brief Upload data from host to device
     * @param buffer Target buffer
     * @param data Source data pointer
     * @param size Size in bytes
     * @param offset Offset in target buffer
     * @param stream Stream for async operation
     */
    virtual BackendResult upload(BufferHandle buffer,
                                  const void* data,
                                  SizeT size,
                                  SizeT offset = 0,
                                  StreamHandle stream = StreamHandle::Default()) = 0;

    /**
     * @brief Download data from device to host
     */
    virtual BackendResult download(BufferHandle buffer,
                                    void* data,
                                    SizeT size,
                                    SizeT offset = 0,
                                    StreamHandle stream = StreamHandle::Default()) = 0;

    /**
     * @brief Copy data between device buffers
     */
    virtual BackendResult copy(BufferHandle src,
                                BufferHandle dst,
                                SizeT size,
                                SizeT src_offset = 0,
                                SizeT dst_offset = 0,
                                StreamHandle stream = StreamHandle::Default()) = 0;

    /**
     * @brief Fill buffer with a pattern
     * @param buffer Target buffer
     * @param pattern Pattern to fill (up to 128 bytes)
     * @param pattern_size Size of pattern in bytes
     * @param size Total size to fill (0 = entire buffer)
     */
    virtual BackendResult fill(BufferHandle buffer,
                                const void* pattern,
                                SizeT pattern_size,
                                SizeT size = 0,
                                StreamHandle stream = StreamHandle::Default()) = 0;

    /**
     * @brief Map buffer to host memory
     * @param buffer Buffer to map
     * @param access Access mode
     * @return Host pointer or nullptr on failure
     */
    virtual void* map(BufferHandle buffer, MemoryAccess access = MemoryAccess::ReadWrite) = 0;

    /**
     * @brief Unmap previously mapped buffer
     */
    virtual void unmap(BufferHandle buffer) = 0;

    /**
     * @brief Get total allocated memory
     */
    virtual SizeT allocated_memory() const = 0;

    /**
     * @brief Get available memory on current device
     */
    virtual SizeT available_memory() const = 0;

    // ========================================================================
    // Kernel Management
    // ========================================================================

    /**
     * @brief Create kernel from source code
     * @param name Kernel function name
     * @param source Kernel source code
     * @param options Compiler options (e.g., "-DDEBUG")
     * @return Kernel object or nullptr on failure
     */
    virtual std::unique_ptr<IKernel> create_kernel(
        const std::string& name,
        const std::string& source,
        const std::string& options = "") = 0;

    /**
     * @brief Create kernel from precompiled binary
     * @param name Kernel function name
     * @param binary Compiled kernel binary
     * @return Kernel object or nullptr on failure
     */
    virtual std::unique_ptr<IKernel> create_kernel_from_binary(
        const std::string& name,
        std::span<const UInt8> binary) = 0;

    /**
     * @brief Dispatch kernel for execution
     * @param kernel Kernel to execute
     * @param config Launch configuration
     * @return Success or error code
     */
    virtual BackendResult dispatch(IKernel* kernel, const LaunchConfig& config) = 0;

    // ========================================================================
    // Stream/Queue Management
    // ========================================================================

    /**
     * @brief Create a new compute stream/queue
     * @return Stream handle or invalid handle on failure
     */
    virtual StreamHandle create_stream() = 0;

    /**
     * @brief Destroy a stream
     */
    virtual void destroy_stream(StreamHandle stream) = 0;

    /**
     * @brief Synchronize a specific stream
     */
    virtual BackendResult synchronize_stream(StreamHandle stream) = 0;

    /**
     * @brief Synchronize all streams on current device
     */
    virtual BackendResult synchronize() = 0;

    // ========================================================================
    // Events and Synchronization
    // ========================================================================

    /**
     * @brief Record an event on a stream
     */
    virtual EventHandle record_event(StreamHandle stream = StreamHandle::Default()) = 0;

    /**
     * @brief Wait for an event to complete
     */
    virtual BackendResult wait_event(EventHandle event) = 0;

    /**
     * @brief Make a stream wait for an event
     */
    virtual BackendResult stream_wait_event(StreamHandle stream, EventHandle event) = 0;

    /**
     * @brief Get elapsed time between events (milliseconds)
     */
    virtual Real elapsed_time(EventHandle start, EventHandle end) = 0;

    /**
     * @brief Destroy an event
     */
    virtual void destroy_event(EventHandle event) = 0;

    // ========================================================================
    // Error Handling
    // ========================================================================

    /**
     * @brief Get last error message
     */
    virtual const std::string& last_error() const = 0;

    /**
     * @brief Clear error state
     */
    virtual void clear_error() = 0;

    /**
     * @brief Check if an error occurred
     */
    virtual bool has_error() const = 0;
};

// ============================================================================
// Backend Factory
// ============================================================================

/**
 * @brief Factory for creating compute backends
 */
class BackendFactory {
public:
    /**
     * @brief Create a compute backend of the specified type
     * @param type Backend type to create
     * @return Backend instance or nullptr if not available
     */
    static std::unique_ptr<IComputeBackend> create(BackendType type);

    /**
     * @brief Create the best available backend
     *
     * Priority: CUDA > Metal (on macOS) > OpenCL > CPU
     */
    static std::unique_ptr<IComputeBackend> create_best_available();

    /**
     * @brief Get list of available backend types
     */
    static std::vector<BackendType> available_backends();

    /**
     * @brief Check if a backend type is available
     */
    static bool is_available(BackendType type);

    /**
     * @brief Get name string for backend type
     */
    static const char* type_name(BackendType type);

    /**
     * @brief Register a custom backend factory
     *
     * Allows external code to register additional backends.
     */
    using CreatorFunc = std::function<std::unique_ptr<IComputeBackend>()>;
    static void register_backend(BackendType type, CreatorFunc creator);

private:
    static std::unordered_map<BackendType, CreatorFunc>& registry();
};

// ============================================================================
// Utility Functions
// ============================================================================

/**
 * @brief Get human-readable name for backend result
 */
const char* result_to_string(BackendResult result);

/**
 * @brief Get human-readable name for memory type
 */
const char* memory_type_to_string(MemoryType type);

/**
 * @brief Get human-readable name for device type
 */
const char* device_type_to_string(DeviceType type);

/**
 * @brief Calculate optimal workgroup size for a kernel
 */
SizeT calculate_optimal_workgroup_size(const IKernel* kernel,
                                        const DeviceCapabilities& caps,
                                        SizeT problem_size);

// ============================================================================
// Backend Registration
// ============================================================================

/**
 * @brief Register all built-in backends
 *
 * Call this function before using the BackendFactory to ensure
 * all built-in backends are registered. This is done automatically
 * when using BackendFactory, but can be called explicitly if needed.
 */
void register_builtin_backends();

} // namespace jaguar::gpu
