/**
 * @file cpu_backend.cpp
 * @brief CPU fallback implementation for compute backend
 *
 * This provides a reference implementation that runs on the CPU,
 * allowing the physics engine to work without GPU hardware.
 * It uses multi-threading for parallel execution.
 */

#include "jaguar/gpu/compute_backend.h"
#include <unordered_map>
#include <atomic>
#include <cstring>
#include <thread>
#include <algorithm>
#include <functional>
#include <chrono>

#ifdef _OPENMP
#include <omp.h>
#endif

namespace jaguar::gpu {

// ============================================================================
// CPU Kernel Implementation
// ============================================================================

/**
 * @brief CPU kernel function signature
 *
 * @param global_id Global thread ID (flattened)
 * @param local_id Local ID within workgroup
 * @param group_id Workgroup ID
 * @param args Array of argument pointers
 * @param num_args Number of arguments
 */
using CPUKernelFunc = std::function<void(
    SizeT global_id,
    SizeT local_id,
    SizeT group_id,
    void** args,
    UInt32 num_args
)>;

/**
 * @brief CPU kernel implementation
 */
class CPUKernel : public IKernel {
public:
    explicit CPUKernel(const std::string& name, CPUKernelFunc func = nullptr)
        : m_name(name), m_function(std::move(func)) {}

    const std::string& name() const override { return m_name; }

    BackendResult set_arg(UInt32 index, const KernelArg& arg) override {
        if (index >= m_args.size()) {
            m_args.resize(index + 1);
            m_arg_types.resize(index + 1);
        }
        m_args[index] = arg.value;
        m_arg_types[index] = arg.type;
        return BackendResult::Success;
    }

    SizeT preferred_workgroup_size() const override {
        // Use number of hardware threads
        return std::thread::hardware_concurrency();
    }

    SizeT max_workgroup_size() const override {
        return 1024; // Reasonable limit for CPU
    }

    SizeT local_memory_size() const override {
        return 0; // CPU doesn't use local memory in the same way
    }

    bool is_valid() const override {
        return m_function != nullptr || !m_source.empty();
    }

    void set_function(CPUKernelFunc func) {
        m_function = std::move(func);
    }

    void set_source(const std::string& source) {
        m_source = source;
    }

    CPUKernelFunc get_function() const { return m_function; }

    const std::vector<KernelArgValue>& args() const { return m_args; }
    const std::vector<ArgType>& arg_types() const { return m_arg_types; }

private:
    std::string m_name;
    std::string m_source;
    CPUKernelFunc m_function;
    std::vector<KernelArgValue> m_args;
    std::vector<ArgType> m_arg_types;
};

// ============================================================================
// CPU Backend Implementation
// ============================================================================

class CPUBackend : public IComputeBackend {
public:
    CPUBackend() = default;
    ~CPUBackend() override { shutdown(); }

    // ========================================================================
    // Lifecycle
    // ========================================================================

    BackendResult initialize(const BackendOptions& options) override {
        if (m_initialized) {
            return BackendResult::Success;
        }

        m_options = options;

        // Determine number of threads
        m_num_threads = std::thread::hardware_concurrency();
        if (m_num_threads == 0) {
            m_num_threads = 4; // Fallback
        }

#ifdef _OPENMP
        if (m_options.enable_profiling) {
            omp_set_num_threads(static_cast<int>(m_num_threads));
        }
#endif

        m_initialized = true;
        return BackendResult::Success;
    }

    void shutdown() override {
        if (!m_initialized) return;

        // Free all buffers
        for (auto& [id, buffer] : m_buffers) {
            std::free(buffer.data);
        }
        m_buffers.clear();
        m_allocated_memory = 0;
        m_initialized = false;
    }

    bool is_initialized() const override {
        return m_initialized;
    }

    BackendType type() const override {
        return BackendType::CPU;
    }

    const std::string& name() const override {
        static const std::string s_name = "CPU Reference Backend";
        return s_name;
    }

    // ========================================================================
    // Device Management
    // ========================================================================

    UInt32 device_count() const override {
        return 1; // CPU is always available
    }

    DeviceCapabilities get_device_capabilities(UInt32 /*device_index*/) const override {
        DeviceCapabilities caps;

        // Get system info
        caps.name = "CPU";
        caps.vendor = "System";
        caps.driver_version = "1.0";
        caps.device_type = DeviceType::CPU;
        caps.backend_type = BackendType::CPU;
        caps.device_id = 0;

        // Memory - estimate based on system
        // In practice, we'd query this from the OS
        caps.global_memory = 8ULL * 1024 * 1024 * 1024; // Assume 8GB
        caps.local_memory = 64 * 1024;  // 64KB L1 cache (approximate)
        caps.constant_memory = 64 * 1024;
        caps.max_allocation = caps.global_memory / 2;
        caps.memory_bandwidth = 50ULL * 1024 * 1024 * 1024; // ~50 GB/s

        // Compute units = hardware threads
        caps.compute_units = std::thread::hardware_concurrency();
        if (caps.compute_units == 0) caps.compute_units = 4;
        caps.max_threads_per_unit = 1;
        caps.warp_size = 1; // No SIMD grouping in basic implementation

        // Limits
        caps.max_workgroup_size = 1024;
        caps.max_workgroup_dims[0] = 1024;
        caps.max_workgroup_dims[1] = 1024;
        caps.max_workgroup_dims[2] = 1024;
        caps.max_grid_dims[0] = UINT32_MAX;
        caps.max_grid_dims[1] = UINT32_MAX;
        caps.max_grid_dims[2] = UINT32_MAX;

        // Features
        caps.supports_double = true;
        caps.supports_atomics = true;
        caps.supports_images = false;
        caps.supports_printf = true;
        caps.supports_unified_memory = true; // CPU memory is unified

        // Performance hints
        caps.preferred_vector_width_float = 4;  // SSE
        caps.preferred_vector_width_double = 2; // SSE2
        caps.compute_capability = 1.0;

        return caps;
    }

    UInt32 current_device() const override {
        return 0;
    }

    BackendResult set_device(UInt32 device_index) override {
        return device_index == 0 ? BackendResult::Success : BackendResult::DeviceNotFound;
    }

    // ========================================================================
    // Memory Management
    // ========================================================================

    BufferHandle allocate(SizeT size, MemoryType type, MemoryAccess access) override {
        if (!m_initialized) {
            m_last_error = "Backend not initialized";
            return BufferHandle::Invalid();
        }

        if (size == 0) {
            m_last_error = "Cannot allocate zero-size buffer";
            return BufferHandle::Invalid();
        }

        void* data = std::malloc(size);
        if (!data) {
            m_last_error = "Failed to allocate memory";
            return BufferHandle::Invalid();
        }

        BufferHandle handle;
        handle.id = m_next_buffer_id++;
        handle.size = size;
        handle.type = type;
        handle.access = access;

        BufferInfo info;
        info.data = data;
        info.size = size;
        info.mapped = false;

        m_buffers[handle.id] = info;
        m_allocated_memory += size;

        return handle;
    }

    void free(BufferHandle buffer) override {
        auto it = m_buffers.find(buffer.id);
        if (it != m_buffers.end()) {
            std::free(it->second.data);
            m_allocated_memory -= it->second.size;
            m_buffers.erase(it);
        }
    }

    BackendResult upload(BufferHandle buffer, const void* data, SizeT size,
                          SizeT offset, StreamHandle /*stream*/) override {
        auto it = m_buffers.find(buffer.id);
        if (it == m_buffers.end()) {
            m_last_error = "Invalid buffer handle";
            return BackendResult::InvalidArgument;
        }

        if (offset + size > it->second.size) {
            m_last_error = "Upload exceeds buffer size";
            return BackendResult::InvalidArgument;
        }

        std::memcpy(static_cast<UInt8*>(it->second.data) + offset, data, size);
        return BackendResult::Success;
    }

    BackendResult download(BufferHandle buffer, void* data, SizeT size,
                            SizeT offset, StreamHandle /*stream*/) override {
        auto it = m_buffers.find(buffer.id);
        if (it == m_buffers.end()) {
            m_last_error = "Invalid buffer handle";
            return BackendResult::InvalidArgument;
        }

        if (offset + size > it->second.size) {
            m_last_error = "Download exceeds buffer size";
            return BackendResult::InvalidArgument;
        }

        std::memcpy(data, static_cast<UInt8*>(it->second.data) + offset, size);
        return BackendResult::Success;
    }

    BackendResult copy(BufferHandle src, BufferHandle dst, SizeT size,
                        SizeT src_offset, SizeT dst_offset, StreamHandle /*stream*/) override {
        auto src_it = m_buffers.find(src.id);
        auto dst_it = m_buffers.find(dst.id);

        if (src_it == m_buffers.end() || dst_it == m_buffers.end()) {
            m_last_error = "Invalid buffer handle";
            return BackendResult::InvalidArgument;
        }

        if (src_offset + size > src_it->second.size ||
            dst_offset + size > dst_it->second.size) {
            m_last_error = "Copy exceeds buffer bounds";
            return BackendResult::InvalidArgument;
        }

        std::memcpy(static_cast<UInt8*>(dst_it->second.data) + dst_offset,
                    static_cast<UInt8*>(src_it->second.data) + src_offset,
                    size);
        return BackendResult::Success;
    }

    BackendResult fill(BufferHandle buffer, const void* pattern,
                        SizeT pattern_size, SizeT size, StreamHandle /*stream*/) override {
        auto it = m_buffers.find(buffer.id);
        if (it == m_buffers.end()) {
            m_last_error = "Invalid buffer handle";
            return BackendResult::InvalidArgument;
        }

        SizeT fill_size = size > 0 ? size : it->second.size;
        if (fill_size > it->second.size) {
            m_last_error = "Fill size exceeds buffer size";
            return BackendResult::InvalidArgument;
        }

        auto* dst = static_cast<UInt8*>(it->second.data);
        for (SizeT i = 0; i < fill_size; i += pattern_size) {
            SizeT copy_size = std::min(pattern_size, fill_size - i);
            std::memcpy(dst + i, pattern, copy_size);
        }

        return BackendResult::Success;
    }

    void* map(BufferHandle buffer, MemoryAccess /*access*/) override {
        auto it = m_buffers.find(buffer.id);
        if (it == m_buffers.end()) {
            m_last_error = "Invalid buffer handle";
            return nullptr;
        }

        it->second.mapped = true;
        return it->second.data;
    }

    void unmap(BufferHandle buffer) override {
        auto it = m_buffers.find(buffer.id);
        if (it != m_buffers.end()) {
            it->second.mapped = false;
        }
    }

    SizeT allocated_memory() const override {
        return m_allocated_memory;
    }

    SizeT available_memory() const override {
        // Return a reasonable estimate
        return 4ULL * 1024 * 1024 * 1024 - m_allocated_memory;
    }

    // ========================================================================
    // Kernel Management
    // ========================================================================

    std::unique_ptr<IKernel> create_kernel(const std::string& name,
                                            const std::string& source,
                                            const std::string& /*options*/) override {
        // CPU backend stores source but can't JIT compile
        // Actual kernel functions must be registered separately
        auto kernel = std::make_unique<CPUKernel>(name);
        kernel->set_source(source);

        // Check if we have a pre-registered function for this kernel
        auto it = m_kernel_registry.find(name);
        if (it != m_kernel_registry.end()) {
            kernel->set_function(it->second);
        }

        return kernel;
    }

    std::unique_ptr<IKernel> create_kernel_from_binary(const std::string& name,
                                                        std::span<const UInt8> /*binary*/) override {
        // CPU doesn't use binary kernels, but we can still create a kernel
        // and look for a registered function
        auto kernel = std::make_unique<CPUKernel>(name);

        auto it = m_kernel_registry.find(name);
        if (it != m_kernel_registry.end()) {
            kernel->set_function(it->second);
        }

        return kernel;
    }

    BackendResult dispatch(IKernel* kernel, const LaunchConfig& config) override {
        if (!m_initialized) {
            m_last_error = "Backend not initialized";
            return BackendResult::NotInitialized;
        }

        auto* cpu_kernel = dynamic_cast<CPUKernel*>(kernel);
        if (!cpu_kernel || !cpu_kernel->is_valid()) {
            m_last_error = "Invalid kernel";
            return BackendResult::InvalidArgument;
        }

        auto func = cpu_kernel->get_function();
        if (!func) {
            m_last_error = "No kernel function registered for: " + kernel->name();
            return BackendResult::NotSupported;
        }

        // Calculate total work items
        SizeT total_groups = config.grid_x * config.grid_y * config.grid_z;
        SizeT group_size = config.block_x * config.block_y * config.block_z;
        SizeT total_items = total_groups * group_size;

        // Prepare argument pointers
        const auto& args = cpu_kernel->args();
        const auto& types = cpu_kernel->arg_types();
        std::vector<void*> arg_ptrs(args.size());

        for (size_t i = 0; i < args.size(); ++i) {
            if (types[i] == ArgType::Buffer) {
                auto handle = std::get<BufferHandle>(args[i]);
                auto it = m_buffers.find(handle.id);
                if (it != m_buffers.end()) {
                    arg_ptrs[i] = it->second.data;
                } else {
                    m_last_error = "Invalid buffer in kernel argument";
                    return BackendResult::InvalidArgument;
                }
            } else {
                // For scalar types, store pointer to the value
                arg_ptrs[i] = const_cast<void*>(static_cast<const void*>(&args[i]));
            }
        }

        // Execute kernel
#ifdef _OPENMP
        // OpenMP requires signed integral type for loop variable
        #pragma omp parallel for schedule(dynamic)
        for (Int64 omp_id = 0; omp_id < static_cast<Int64>(total_items); ++omp_id) {
            SizeT global_id = static_cast<SizeT>(omp_id);
            SizeT group_id = global_id / group_size;
            SizeT local_id = global_id % group_size;
            func(global_id, local_id, group_id, arg_ptrs.data(),
                 static_cast<UInt32>(arg_ptrs.size()));
        }
#else
        // Single-threaded fallback with basic parallelism using std::thread
        SizeT num_threads = std::min(static_cast<SizeT>(m_num_threads), total_items);
        std::vector<std::thread> threads;
        threads.reserve(num_threads);

        SizeT items_per_thread = (total_items + num_threads - 1) / num_threads;

        for (UInt32 t = 0; t < num_threads; ++t) {
            SizeT start = t * items_per_thread;
            SizeT end = std::min(start + items_per_thread, total_items);

            if (start >= end) break;

            threads.emplace_back([&, start, end]() {
                for (SizeT global_id = start; global_id < end; ++global_id) {
                    SizeT group_id = global_id / group_size;
                    SizeT local_id = global_id % group_size;
                    func(global_id, local_id, group_id, arg_ptrs.data(),
                         static_cast<UInt32>(arg_ptrs.size()));
                }
            });
        }

        for (auto& thread : threads) {
            thread.join();
        }
#endif

        return BackendResult::Success;
    }

    /**
     * @brief Register a CPU kernel function
     *
     * This allows C++ functions to be registered as "kernels" for the CPU backend.
     */
    void register_kernel(const std::string& name, CPUKernelFunc func) {
        m_kernel_registry[name] = std::move(func);
    }

    // ========================================================================
    // Stream/Queue Management
    // ========================================================================

    StreamHandle create_stream() override {
        // CPU doesn't have real streams, but we track them for API compatibility
        StreamHandle handle;
        handle.id = m_next_stream_id++;
        return handle;
    }

    void destroy_stream(StreamHandle /*stream*/) override {
        // No-op for CPU
    }

    BackendResult synchronize_stream(StreamHandle /*stream*/) override {
        // CPU execution is synchronous
        return BackendResult::Success;
    }

    BackendResult synchronize() override {
        // CPU execution is synchronous
        return BackendResult::Success;
    }

    // ========================================================================
    // Events and Synchronization
    // ========================================================================

    EventHandle record_event(StreamHandle /*stream*/) override {
        EventHandle handle;
        handle.id = m_next_event_id++;

        EventInfo info;
        info.timestamp = std::chrono::high_resolution_clock::now();
        m_events[handle.id] = info;

        return handle;
    }

    BackendResult wait_event(EventHandle /*event*/) override {
        // CPU execution is synchronous
        return BackendResult::Success;
    }

    BackendResult stream_wait_event(StreamHandle /*stream*/, EventHandle /*event*/) override {
        // CPU execution is synchronous
        return BackendResult::Success;
    }

    Real elapsed_time(EventHandle start, EventHandle end) override {
        auto start_it = m_events.find(start.id);
        auto end_it = m_events.find(end.id);

        if (start_it == m_events.end() || end_it == m_events.end()) {
            return 0.0;
        }

        auto duration = end_it->second.timestamp - start_it->second.timestamp;
        return std::chrono::duration<Real, std::milli>(duration).count();
    }

    void destroy_event(EventHandle event) override {
        m_events.erase(event.id);
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
    struct BufferInfo {
        void* data{nullptr};
        SizeT size{0};
        bool mapped{false};
    };

    struct EventInfo {
        std::chrono::high_resolution_clock::time_point timestamp;
    };

    bool m_initialized{false};
    BackendOptions m_options;
    UInt32 m_num_threads{4};

    // Buffer management
    std::unordered_map<UInt64, BufferInfo> m_buffers;
    std::atomic<UInt64> m_next_buffer_id{1};
    SizeT m_allocated_memory{0};

    // Stream/event management
    std::atomic<UInt64> m_next_stream_id{2}; // 1 is default stream
    std::atomic<UInt64> m_next_event_id{1};
    std::unordered_map<UInt64, EventInfo> m_events;

    // Kernel registry
    std::unordered_map<std::string, CPUKernelFunc> m_kernel_registry;

    // Error state
    std::string m_last_error;
};

// ============================================================================
// Backend Creator Function (called from compute_backend.cpp)
// ============================================================================

namespace detail {

std::unique_ptr<IComputeBackend> create_cpu_backend() {
    return std::make_unique<CPUBackend>();
}

} // namespace detail

} // namespace jaguar::gpu
