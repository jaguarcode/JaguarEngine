/**
 * @file metal_backend.mm
 * @brief Apple Metal compute backend implementation
 *
 * This file implements the IComputeBackend interface using Apple's Metal API,
 * providing high-performance GPU compute on macOS and iOS devices.
 *
 * Features:
 * - Apple Silicon and Intel GPU support
 * - Metal Shading Language (MSL) kernel compilation
 * - Unified Memory Architecture (UMA) support
 * - Compute command encoders for parallel execution
 * - MTLBuffer for memory management
 * - MTLComputePipelineState for kernel dispatch
 * - MTLCommandQueue for async execution
 * - MTLEvent and MTLSharedEvent for synchronization
 *
 * Requirements:
 * - macOS 10.13+ or iOS 11+
 * - Metal-capable GPU
 *
 * Note: This file requires Objective-C++ compilation (.mm extension)
 */

#ifdef JAGUAR_HAS_METAL

#import <Metal/Metal.h>
#import <Foundation/Foundation.h>

#include "jaguar/gpu/compute_backend.h"
#include <unordered_map>
#include <vector>
#include <mutex>
#include <atomic>
#include <chrono>

namespace jaguar::gpu {

// ============================================================================
// Forward Declarations
// ============================================================================

class MetalKernel;
class MetalBackend;

// ============================================================================
// Metal Kernel Implementation
// ============================================================================

/**
 * @brief Metal compute kernel wrapper
 */
class MetalKernel : public IKernel {
public:
    MetalKernel(id<MTLComputePipelineState> pipeline,
                id<MTLFunction> function,
                const std::string& kernel_name,
                MetalBackend* backend)
        : m_pipeline(pipeline)
        , m_function(function)
        , m_name(kernel_name)
        , m_backend(backend)
    {
        if (m_pipeline) {
            m_max_threads_per_threadgroup = m_pipeline.maxTotalThreadsPerThreadgroup;
            m_threadgroup_memory_length = m_pipeline.staticThreadgroupMemoryLength;
        }
    }

    ~MetalKernel() override {
        // ARC handles memory management
        m_pipeline = nil;
        m_function = nil;
    }

    const std::string& name() const override { return m_name; }

    BackendResult set_arg(UInt32 index, const KernelArg& arg) override;

    SizeT preferred_workgroup_size() const override {
        // Metal prefers threadgroup sizes that are multiples of thread execution width
        NSUInteger width = m_pipeline.threadExecutionWidth;
        return static_cast<SizeT>(width > 0 ? width * 4 : 256);
    }

    SizeT max_workgroup_size() const override {
        return static_cast<SizeT>(m_max_threads_per_threadgroup);
    }

    SizeT local_memory_size() const override {
        return static_cast<SizeT>(m_threadgroup_memory_length);
    }

    UInt32 register_count() const override {
        // Metal doesn't expose register count directly
        return 0;
    }

    bool is_valid() const override {
        return m_pipeline != nil;
    }

    // Metal-specific accessors
    id<MTLComputePipelineState> pipeline() const { return m_pipeline; }
    NSUInteger thread_execution_width() const {
        return m_pipeline ? m_pipeline.threadExecutionWidth : 32;
    }

    // Get stored arguments for encoding
    const std::vector<KernelArg>& arguments() const { return m_arguments; }

private:
    id<MTLComputePipelineState> m_pipeline;
    id<MTLFunction> m_function;
    std::string m_name;
    MetalBackend* m_backend;
    NSUInteger m_max_threads_per_threadgroup{256};
    NSUInteger m_threadgroup_memory_length{0};
    std::vector<KernelArg> m_arguments;
};

// ============================================================================
// Metal Backend Implementation
// ============================================================================

/**
 * @brief Metal compute backend
 */
class MetalBackend : public IComputeBackend {
public:
    MetalBackend() = default;
    ~MetalBackend() override { shutdown(); }

    // ========================================================================
    // Lifecycle
    // ========================================================================

    BackendResult initialize(const BackendOptions& options = {}) override {
        std::lock_guard<std::mutex> lock(m_mutex);

        if (m_initialized) {
            return BackendResult::Success;
        }

        // Get all Metal devices
        NSArray<id<MTLDevice>>* devices = MTLCopyAllDevices();
        if (devices == nil || devices.count == 0) {
            set_error("No Metal devices found");
            return BackendResult::DeviceNotFound;
        }

        m_devices.clear();
        for (id<MTLDevice> device in devices) {
            m_devices.push_back(device);
        }

        // Select device
        UInt32 device_index = options.device_index;
        if (device_index >= m_devices.size()) {
            device_index = 0;
        }

        m_current_device = device_index;
        m_device = m_devices[device_index];

        if (m_device == nil) {
            set_error("Failed to get Metal device");
            return BackendResult::DeviceNotFound;
        }

        // Create command queue
        m_default_queue = [m_device newCommandQueue];
        if (m_default_queue == nil) {
            set_error("Failed to create command queue");
            return BackendResult::InternalError;
        }

        // Build name string
        m_name = "Metal (";
        m_name += [m_device.name UTF8String];
        m_name += ")";

        m_enable_profiling = options.enable_profiling;
        m_initialized = true;

        return BackendResult::Success;
    }

    void shutdown() override {
        std::lock_guard<std::mutex> lock(m_mutex);

        if (!m_initialized) return;

        // Wait for all work to complete
        synchronize();

        // Free all buffers
        for (auto& [id, info] : m_buffers) {
            info.buffer = nil;
        }
        m_buffers.clear();

        // Destroy all streams
        for (auto& [id, queue] : m_streams) {
            queue = nil;
        }
        m_streams.clear();

        // Destroy all events
        for (auto& [id, event] : m_events) {
            event.event = nil;
        }
        m_events.clear();

        m_default_queue = nil;
        m_device = nil;
        m_devices.clear();

        m_initialized = false;
        m_allocated_memory = 0;
    }

    bool is_initialized() const override { return m_initialized; }

    BackendType type() const override { return BackendType::Metal; }

    const std::string& name() const override { return m_name; }

    // ========================================================================
    // Device Management
    // ========================================================================

    UInt32 device_count() const override {
        return static_cast<UInt32>(m_devices.size());
    }

    DeviceCapabilities get_device_capabilities(UInt32 device_index = 0) const override {
        DeviceCapabilities caps;

        if (device_index >= m_devices.size()) {
            return caps;
        }

        id<MTLDevice> device = m_devices[device_index];

        // Identification
        caps.name = [device.name UTF8String];
        caps.vendor = "Apple";
        caps.backend_type = BackendType::Metal;
        caps.device_id = device_index;

        // Determine device type
        if (device.isLowPower) {
            caps.device_type = DeviceType::IntegratedGPU;
        } else {
            caps.device_type = DeviceType::DiscreteGPU;
        }

        // Memory - Metal unified memory model
        // On Apple Silicon, CPU and GPU share memory
        caps.global_memory = device.recommendedMaxWorkingSetSize;
        caps.max_allocation = device.maxBufferLength;
        caps.local_memory = 32768;  // 32KB typical threadgroup memory

        // Compute units
        // Metal doesn't expose compute unit count directly
        // Use heuristics based on device characteristics
        if (@available(macOS 10.15, *)) {
            // Estimate based on max threadgroup memory and concurrent execution
            caps.compute_units = 8;  // Conservative default
        }

        caps.max_threads_per_unit = 1024;
        caps.warp_size = 32;  // SIMD width on Apple GPUs

        // Limits
        MTLSize maxThreadsPerThreadgroup = device.maxThreadsPerThreadgroup;
        caps.max_workgroup_size = maxThreadsPerThreadgroup.width *
                                   maxThreadsPerThreadgroup.height *
                                   maxThreadsPerThreadgroup.depth;
        caps.max_workgroup_dims[0] = static_cast<UInt32>(maxThreadsPerThreadgroup.width);
        caps.max_workgroup_dims[1] = static_cast<UInt32>(maxThreadsPerThreadgroup.height);
        caps.max_workgroup_dims[2] = static_cast<UInt32>(maxThreadsPerThreadgroup.depth);

        // Grid limits are very large in Metal
        caps.max_grid_dims[0] = UINT32_MAX;
        caps.max_grid_dims[1] = UINT32_MAX;
        caps.max_grid_dims[2] = UINT32_MAX;

        // Features
        caps.supports_atomics = true;
        caps.supports_images = true;

        // Check for 32-bit float atomics (Apple7+)
        if (@available(macOS 11.0, *)) {
            caps.supports_atomics = device.supportsFamily(MTLGPUFamilyApple7);
        }

        // Double precision - not available on Apple GPUs
        caps.supports_double = false;

        // Unified memory (all Metal devices support this)
        caps.supports_unified_memory = true;

        // Printf support via Metal debugging
        caps.supports_printf = true;

        // Performance hints
        caps.preferred_vector_width_float = 4;
        caps.preferred_vector_width_double = 0;  // No double support

        // Compute capability equivalent (based on GPU family)
        if (@available(macOS 10.15, *)) {
            if (device.supportsFamily(MTLGPUFamilyApple8)) {
                caps.compute_capability = 8.0;  // M2 and later
            } else if (device.supportsFamily(MTLGPUFamilyApple7)) {
                caps.compute_capability = 7.0;  // M1
            } else if (device.supportsFamily(MTLGPUFamilyMac2)) {
                caps.compute_capability = 2.0;  // Intel Mac
            } else {
                caps.compute_capability = 1.0;
            }
        }

        return caps;
    }

    UInt32 current_device() const override { return m_current_device; }

    BackendResult set_device(UInt32 device_index) override {
        std::lock_guard<std::mutex> lock(m_mutex);

        if (device_index >= m_devices.size()) {
            set_error("Invalid device index");
            return BackendResult::DeviceNotFound;
        }

        if (device_index == m_current_device) {
            return BackendResult::Success;
        }

        // Synchronize current device
        synchronize();

        m_current_device = device_index;
        m_device = m_devices[device_index];

        // Create new command queue for the device
        m_default_queue = [m_device newCommandQueue];
        if (m_default_queue == nil) {
            set_error("Failed to create command queue for device");
            return BackendResult::InternalError;
        }

        return BackendResult::Success;
    }

    // ========================================================================
    // Memory Management
    // ========================================================================

    BufferHandle allocate(SizeT size,
                          MemoryType type = MemoryType::DeviceLocal,
                          MemoryAccess access = MemoryAccess::ReadWrite) override {
        std::lock_guard<std::mutex> lock(m_mutex);

        if (!m_initialized || size == 0) {
            if (size == 0) {
                set_error("Cannot allocate zero-size buffer");
            }
            return BufferHandle::Invalid();
        }

        // Determine resource options based on memory type
        MTLResourceOptions options = MTLResourceHazardTrackingModeDefault;

        switch (type) {
            case MemoryType::DeviceLocal:
                // Private storage - fastest for GPU-only access
                options |= MTLResourceStorageModePrivate;
                break;

            case MemoryType::HostVisible:
            case MemoryType::Staging:
                // Shared storage - CPU can read/write directly
                options |= MTLResourceStorageModeShared;
                break;

            case MemoryType::HostCached:
                // Managed storage - automatic CPU/GPU sync on macOS
                #if TARGET_OS_OSX
                options |= MTLResourceStorageModeManaged;
                #else
                options |= MTLResourceStorageModeShared;
                #endif
                break;

            case MemoryType::Shared:
                // Shared for unified memory access
                options |= MTLResourceStorageModeShared;
                break;
        }

        // Create buffer
        id<MTLBuffer> buffer = [m_device newBufferWithLength:size options:options];
        if (buffer == nil) {
            set_error("Failed to allocate Metal buffer of size " + std::to_string(size));
            return BufferHandle::Invalid();
        }

        // Create handle
        BufferHandle handle;
        handle.id = m_next_buffer_id++;
        handle.size = size;
        handle.type = type;
        handle.access = access;

        // Store buffer info
        BufferInfo info;
        info.buffer = buffer;
        info.size = size;
        info.type = type;
        info.mapped_ptr = nullptr;

        m_buffers[handle.id] = info;
        m_allocated_memory += size;

        return handle;
    }

    void free(BufferHandle handle) override {
        std::lock_guard<std::mutex> lock(m_mutex);

        auto it = m_buffers.find(handle.id);
        if (it == m_buffers.end()) return;

        m_allocated_memory -= it->second.size;
        it->second.buffer = nil;
        m_buffers.erase(it);
    }

    BackendResult upload(BufferHandle handle,
                         const void* data,
                         SizeT size,
                         SizeT offset = 0,
                         StreamHandle stream = StreamHandle::Default()) override {
        if (!m_initialized) {
            set_error("Backend not initialized");
            return BackendResult::NotInitialized;
        }

        if (!handle.is_valid() || data == nullptr) {
            set_error("Invalid buffer handle or null data pointer");
            return BackendResult::InvalidArgument;
        }

        std::lock_guard<std::mutex> lock(m_mutex);

        auto it = m_buffers.find(handle.id);
        if (it == m_buffers.end()) {
            set_error("Buffer not found");
            return BackendResult::InvalidArgument;
        }

        BufferInfo& info = it->second;

        if (offset + size > info.size) {
            set_error("Upload exceeds buffer size");
            return BackendResult::InvalidArgument;
        }

        id<MTLBuffer> buffer = info.buffer;

        // For private storage, we need a blit encoder
        if (info.type == MemoryType::DeviceLocal) {
            // Create staging buffer
            id<MTLBuffer> staging = [m_device newBufferWithBytes:data
                                                          length:size
                                                         options:MTLResourceStorageModeShared];
            if (staging == nil) {
                set_error("Failed to create staging buffer");
                return BackendResult::OutOfMemory;
            }

            // Get command buffer
            id<MTLCommandQueue> queue = get_queue(stream);
            id<MTLCommandBuffer> cmdBuffer = [queue commandBuffer];

            // Create blit encoder
            id<MTLBlitCommandEncoder> blit = [cmdBuffer blitCommandEncoder];
            [blit copyFromBuffer:staging
                    sourceOffset:0
                        toBuffer:buffer
               destinationOffset:offset
                            size:size];
            [blit endEncoding];

            // Commit
            [cmdBuffer commit];

            // Wait if using default stream
            if (!stream.is_valid() || stream.id == 1) {
                [cmdBuffer waitUntilCompleted];
            }
        } else {
            // Shared/managed storage - direct copy
            void* dst = static_cast<UInt8*>(buffer.contents) + offset;
            memcpy(dst, data, size);

            // Synchronize managed storage
            #if TARGET_OS_OSX
            if (info.type == MemoryType::HostCached) {
                [buffer didModifyRange:NSMakeRange(offset, size)];
            }
            #endif
        }

        return BackendResult::Success;
    }

    BackendResult download(BufferHandle handle,
                           void* data,
                           SizeT size,
                           SizeT offset = 0,
                           StreamHandle stream = StreamHandle::Default()) override {
        if (!m_initialized) {
            set_error("Backend not initialized");
            return BackendResult::NotInitialized;
        }

        if (!handle.is_valid() || data == nullptr) {
            set_error("Invalid buffer handle or null data pointer");
            return BackendResult::InvalidArgument;
        }

        std::lock_guard<std::mutex> lock(m_mutex);

        auto it = m_buffers.find(handle.id);
        if (it == m_buffers.end()) {
            set_error("Buffer not found");
            return BackendResult::InvalidArgument;
        }

        BufferInfo& info = it->second;

        if (offset + size > info.size) {
            set_error("Download exceeds buffer size");
            return BackendResult::InvalidArgument;
        }

        id<MTLBuffer> buffer = info.buffer;

        // For private storage, we need a blit encoder
        if (info.type == MemoryType::DeviceLocal) {
            // Create staging buffer for download
            id<MTLBuffer> staging = [m_device newBufferWithLength:size
                                                          options:MTLResourceStorageModeShared];
            if (staging == nil) {
                set_error("Failed to create staging buffer");
                return BackendResult::OutOfMemory;
            }

            // Get command buffer
            id<MTLCommandQueue> queue = get_queue(stream);
            id<MTLCommandBuffer> cmdBuffer = [queue commandBuffer];

            // Create blit encoder
            id<MTLBlitCommandEncoder> blit = [cmdBuffer blitCommandEncoder];
            [blit copyFromBuffer:buffer
                    sourceOffset:offset
                        toBuffer:staging
               destinationOffset:0
                            size:size];
            [blit endEncoding];

            // Commit and wait
            [cmdBuffer commit];
            [cmdBuffer waitUntilCompleted];

            // Copy to host
            memcpy(data, staging.contents, size);
        } else {
            // Shared/managed storage - direct copy
            #if TARGET_OS_OSX
            if (info.type == MemoryType::HostCached) {
                // Synchronize managed storage before reading
                id<MTLCommandQueue> queue = get_queue(stream);
                id<MTLCommandBuffer> cmdBuffer = [queue commandBuffer];
                id<MTLBlitCommandEncoder> blit = [cmdBuffer blitCommandEncoder];
                [blit synchronizeResource:buffer];
                [blit endEncoding];
                [cmdBuffer commit];
                [cmdBuffer waitUntilCompleted];
            }
            #endif

            const void* src = static_cast<const UInt8*>(buffer.contents) + offset;
            memcpy(data, src, size);
        }

        return BackendResult::Success;
    }

    BackendResult copy(BufferHandle src,
                       BufferHandle dst,
                       SizeT size,
                       SizeT src_offset = 0,
                       SizeT dst_offset = 0,
                       StreamHandle stream = StreamHandle::Default()) override {
        if (!m_initialized) {
            return BackendResult::NotInitialized;
        }

        std::lock_guard<std::mutex> lock(m_mutex);

        auto src_it = m_buffers.find(src.id);
        auto dst_it = m_buffers.find(dst.id);

        if (src_it == m_buffers.end() || dst_it == m_buffers.end()) {
            set_error("Source or destination buffer not found");
            return BackendResult::InvalidArgument;
        }

        if (src_offset + size > src_it->second.size ||
            dst_offset + size > dst_it->second.size) {
            set_error("Copy exceeds buffer size");
            return BackendResult::InvalidArgument;
        }

        id<MTLCommandQueue> queue = get_queue(stream);
        id<MTLCommandBuffer> cmdBuffer = [queue commandBuffer];

        id<MTLBlitCommandEncoder> blit = [cmdBuffer blitCommandEncoder];
        [blit copyFromBuffer:src_it->second.buffer
                sourceOffset:src_offset
                    toBuffer:dst_it->second.buffer
           destinationOffset:dst_offset
                        size:size];
        [blit endEncoding];

        [cmdBuffer commit];

        if (!stream.is_valid() || stream.id == 1) {
            [cmdBuffer waitUntilCompleted];
        }

        return BackendResult::Success;
    }

    BackendResult fill(BufferHandle handle,
                       const void* pattern,
                       SizeT pattern_size,
                       SizeT size = 0,
                       StreamHandle stream = StreamHandle::Default()) override {
        if (!m_initialized) {
            return BackendResult::NotInitialized;
        }

        std::lock_guard<std::mutex> lock(m_mutex);

        auto it = m_buffers.find(handle.id);
        if (it == m_buffers.end()) {
            set_error("Buffer not found");
            return BackendResult::InvalidArgument;
        }

        BufferInfo& info = it->second;
        SizeT fill_size = (size == 0) ? info.size : size;

        if (fill_size > info.size) {
            set_error("Fill size exceeds buffer size");
            return BackendResult::InvalidArgument;
        }

        // Metal's fillBuffer only supports single-byte patterns
        // For larger patterns, we need to use a compute shader or CPU copy
        if (pattern_size == 1) {
            UInt8 value = *static_cast<const UInt8*>(pattern);

            id<MTLCommandQueue> queue = get_queue(stream);
            id<MTLCommandBuffer> cmdBuffer = [queue commandBuffer];

            id<MTLBlitCommandEncoder> blit = [cmdBuffer blitCommandEncoder];
            [blit fillBuffer:info.buffer range:NSMakeRange(0, fill_size) value:value];
            [blit endEncoding];

            [cmdBuffer commit];

            if (!stream.is_valid() || stream.id == 1) {
                [cmdBuffer waitUntilCompleted];
            }
        } else {
            // For multi-byte patterns, use CPU-side fill for shared memory
            // or create a staging buffer for private memory
            if (info.type != MemoryType::DeviceLocal) {
                UInt8* ptr = static_cast<UInt8*>(info.buffer.contents);
                for (SizeT i = 0; i < fill_size; i += pattern_size) {
                    SizeT copy_size = std::min(pattern_size, fill_size - i);
                    memcpy(ptr + i, pattern, copy_size);
                }

                #if TARGET_OS_OSX
                if (info.type == MemoryType::HostCached) {
                    [info.buffer didModifyRange:NSMakeRange(0, fill_size)];
                }
                #endif
            } else {
                // Create staging buffer and blit
                std::vector<UInt8> staging_data(fill_size);
                for (SizeT i = 0; i < fill_size; i += pattern_size) {
                    SizeT copy_size = std::min(pattern_size, fill_size - i);
                    memcpy(staging_data.data() + i, pattern, copy_size);
                }

                id<MTLBuffer> staging = [m_device newBufferWithBytes:staging_data.data()
                                                              length:fill_size
                                                             options:MTLResourceStorageModeShared];

                id<MTLCommandQueue> queue = get_queue(stream);
                id<MTLCommandBuffer> cmdBuffer = [queue commandBuffer];

                id<MTLBlitCommandEncoder> blit = [cmdBuffer blitCommandEncoder];
                [blit copyFromBuffer:staging
                        sourceOffset:0
                            toBuffer:info.buffer
                   destinationOffset:0
                                size:fill_size];
                [blit endEncoding];

                [cmdBuffer commit];

                if (!stream.is_valid() || stream.id == 1) {
                    [cmdBuffer waitUntilCompleted];
                }
            }
        }

        return BackendResult::Success;
    }

    void* map(BufferHandle handle, MemoryAccess access = MemoryAccess::ReadWrite) override {
        std::lock_guard<std::mutex> lock(m_mutex);

        auto it = m_buffers.find(handle.id);
        if (it == m_buffers.end()) {
            set_error("Buffer not found");
            return nullptr;
        }

        BufferInfo& info = it->second;

        // Private storage cannot be mapped directly
        if (info.type == MemoryType::DeviceLocal) {
            set_error("Cannot map private storage buffer");
            return nullptr;
        }

        info.mapped_ptr = info.buffer.contents;
        return info.mapped_ptr;
    }

    void unmap(BufferHandle handle) override {
        std::lock_guard<std::mutex> lock(m_mutex);

        auto it = m_buffers.find(handle.id);
        if (it == m_buffers.end()) return;

        BufferInfo& info = it->second;

        #if TARGET_OS_OSX
        if (info.type == MemoryType::HostCached && info.mapped_ptr) {
            [info.buffer didModifyRange:NSMakeRange(0, info.size)];
        }
        #endif

        info.mapped_ptr = nullptr;
    }

    SizeT allocated_memory() const override { return m_allocated_memory; }

    SizeT available_memory() const override {
        if (m_device) {
            return m_device.recommendedMaxWorkingSetSize - m_allocated_memory;
        }
        return 0;
    }

    // ========================================================================
    // Kernel Management
    // ========================================================================

    std::unique_ptr<IKernel> create_kernel(
        const std::string& kernel_name,
        const std::string& source,
        const std::string& options = "") override {

        if (!m_initialized) {
            set_error("Backend not initialized");
            return nullptr;
        }

        NSError* error = nil;

        // Compile options
        MTLCompileOptions* compile_options = [[MTLCompileOptions alloc] init];
        compile_options.fastMathEnabled = YES;
        compile_options.languageVersion = MTLLanguageVersion2_4;

        // Parse additional options
        if (!options.empty()) {
            // Handle preprocessor defines
            NSMutableDictionary* defines = [[NSMutableDictionary alloc] init];
            // Parse -D options from the string
            std::string opts = options;
            size_t pos = 0;
            while ((pos = opts.find("-D")) != std::string::npos) {
                opts = opts.substr(pos + 2);
                size_t end = opts.find_first_of(" \t");
                std::string define = opts.substr(0, end);
                size_t eq = define.find('=');
                if (eq != std::string::npos) {
                    NSString* key = [NSString stringWithUTF8String:define.substr(0, eq).c_str()];
                    NSString* val = [NSString stringWithUTF8String:define.substr(eq + 1).c_str()];
                    defines[key] = val;
                } else {
                    NSString* key = [NSString stringWithUTF8String:define.c_str()];
                    defines[key] = @"1";
                }
                if (end == std::string::npos) break;
                opts = opts.substr(end);
            }
            if (defines.count > 0) {
                compile_options.preprocessorMacros = defines;
            }
        }

        // Compile source
        NSString* ns_source = [NSString stringWithUTF8String:source.c_str()];
        id<MTLLibrary> library = [m_device newLibraryWithSource:ns_source
                                                        options:compile_options
                                                          error:&error];

        if (library == nil || error != nil) {
            std::string error_msg = "Metal compilation failed";
            if (error) {
                error_msg += ": ";
                error_msg += [error.localizedDescription UTF8String];
            }
            set_error(error_msg);
            return nullptr;
        }

        // Get function
        NSString* ns_name = [NSString stringWithUTF8String:kernel_name.c_str()];
        id<MTLFunction> function = [library newFunctionWithName:ns_name];

        if (function == nil) {
            set_error("Function '" + kernel_name + "' not found in compiled library");
            return nullptr;
        }

        // Create compute pipeline
        id<MTLComputePipelineState> pipeline = [m_device newComputePipelineStateWithFunction:function
                                                                                       error:&error];

        if (pipeline == nil || error != nil) {
            std::string error_msg = "Failed to create compute pipeline";
            if (error) {
                error_msg += ": ";
                error_msg += [error.localizedDescription UTF8String];
            }
            set_error(error_msg);
            return nullptr;
        }

        return std::make_unique<MetalKernel>(pipeline, function, kernel_name, this);
    }

    std::unique_ptr<IKernel> create_kernel_from_binary(
        const std::string& kernel_name,
        std::span<const UInt8> binary) override {

        if (!m_initialized) {
            set_error("Backend not initialized");
            return nullptr;
        }

        NSError* error = nil;

        // Create dispatch data from binary
        dispatch_data_t data = dispatch_data_create(
            binary.data(), binary.size(),
            dispatch_get_main_queue(),
            DISPATCH_DATA_DESTRUCTOR_DEFAULT);

        // Create library from metallib binary
        id<MTLLibrary> library = [m_device newLibraryWithData:data error:&error];

        if (library == nil || error != nil) {
            std::string error_msg = "Failed to load Metal library from binary";
            if (error) {
                error_msg += ": ";
                error_msg += [error.localizedDescription UTF8String];
            }
            set_error(error_msg);
            return nullptr;
        }

        // Get function
        NSString* ns_name = [NSString stringWithUTF8String:kernel_name.c_str()];
        id<MTLFunction> function = [library newFunctionWithName:ns_name];

        if (function == nil) {
            set_error("Function '" + kernel_name + "' not found in binary library");
            return nullptr;
        }

        // Create compute pipeline
        id<MTLComputePipelineState> pipeline = [m_device newComputePipelineStateWithFunction:function
                                                                                       error:&error];

        if (pipeline == nil) {
            set_error("Failed to create compute pipeline from binary");
            return nullptr;
        }

        return std::make_unique<MetalKernel>(pipeline, function, kernel_name, this);
    }

    BackendResult dispatch(IKernel* kernel, const LaunchConfig& config) override {
        if (!m_initialized) {
            set_error("Backend not initialized");
            return BackendResult::NotInitialized;
        }

        if (!kernel || !kernel->is_valid()) {
            set_error("Invalid kernel");
            return BackendResult::InvalidArgument;
        }

        MetalKernel* metal_kernel = static_cast<MetalKernel*>(kernel);
        id<MTLComputePipelineState> pipeline = metal_kernel->pipeline();

        // Get command buffer
        id<MTLCommandQueue> queue = get_queue(config.stream);
        id<MTLCommandBuffer> cmdBuffer = [queue commandBuffer];

        if (cmdBuffer == nil) {
            set_error("Failed to create command buffer");
            return BackendResult::InternalError;
        }

        // Create compute encoder
        id<MTLComputeCommandEncoder> encoder = [cmdBuffer computeCommandEncoder];
        if (encoder == nil) {
            set_error("Failed to create compute encoder");
            return BackendResult::InternalError;
        }

        // Set pipeline state
        [encoder setComputePipelineState:pipeline];

        // Set kernel arguments
        const auto& args = metal_kernel->arguments();
        UInt32 buffer_index = 0;

        for (UInt32 i = 0; i < args.size(); ++i) {
            const KernelArg& arg = args[i];

            switch (arg.type) {
                case ArgType::Buffer: {
                    BufferHandle handle = std::get<BufferHandle>(arg.value);
                    auto it = m_buffers.find(handle.id);
                    if (it != m_buffers.end()) {
                        [encoder setBuffer:it->second.buffer offset:0 atIndex:buffer_index++];
                    }
                    break;
                }
                case ArgType::Scalar_I32: {
                    Int32 value = std::get<Int32>(arg.value);
                    [encoder setBytes:&value length:sizeof(value) atIndex:buffer_index++];
                    break;
                }
                case ArgType::Scalar_U32: {
                    UInt32 value = std::get<UInt32>(arg.value);
                    [encoder setBytes:&value length:sizeof(value) atIndex:buffer_index++];
                    break;
                }
                case ArgType::Scalar_I64: {
                    Int64 value = std::get<Int64>(arg.value);
                    [encoder setBytes:&value length:sizeof(value) atIndex:buffer_index++];
                    break;
                }
                case ArgType::Scalar_U64: {
                    UInt64 value = std::get<UInt64>(arg.value);
                    [encoder setBytes:&value length:sizeof(value) atIndex:buffer_index++];
                    break;
                }
                case ArgType::Scalar_F32: {
                    float value = std::get<float>(arg.value);
                    [encoder setBytes:&value length:sizeof(value) atIndex:buffer_index++];
                    break;
                }
                case ArgType::Scalar_F64: {
                    double value = std::get<double>(arg.value);
                    [encoder setBytes:&value length:sizeof(value) atIndex:buffer_index++];
                    break;
                }
                case ArgType::LocalMem: {
                    SizeT size = std::get<SizeT>(arg.value);
                    [encoder setThreadgroupMemoryLength:size atIndex:0];
                    break;
                }
                default:
                    break;
            }
        }

        // Calculate thread dimensions
        // Metal uses threads per grid and threads per threadgroup
        MTLSize threadsPerGrid = MTLSizeMake(
            config.grid_x * config.block_x,
            config.grid_y * config.block_y,
            config.grid_z * config.block_z
        );

        MTLSize threadsPerThreadgroup = MTLSizeMake(
            config.block_x,
            config.block_y,
            config.block_z
        );

        // Clamp threadgroup size to maximum
        NSUInteger max_total = pipeline.maxTotalThreadsPerThreadgroup;
        NSUInteger total = config.block_x * config.block_y * config.block_z;
        if (total > max_total) {
            // Scale down proportionally
            double scale = std::sqrt(static_cast<double>(max_total) / total);
            threadsPerThreadgroup = MTLSizeMake(
                std::max(1UL, static_cast<NSUInteger>(config.block_x * scale)),
                std::max(1UL, static_cast<NSUInteger>(config.block_y * scale)),
                std::max(1UL, static_cast<NSUInteger>(config.block_z * scale))
            );
        }

        // Use dispatchThreads for non-uniform grids
        if (@available(macOS 10.13, *)) {
            [encoder dispatchThreads:threadsPerGrid
               threadsPerThreadgroup:threadsPerThreadgroup];
        } else {
            // Fall back to dispatchThreadgroups for older systems
            MTLSize threadgroups = MTLSizeMake(
                config.grid_x,
                config.grid_y,
                config.grid_z
            );
            [encoder dispatchThreadgroups:threadgroups
                    threadsPerThreadgroup:threadsPerThreadgroup];
        }

        [encoder endEncoding];
        [cmdBuffer commit];

        // Wait if using default stream or non-async
        if (!config.stream.is_valid() || config.stream.id == 1) {
            [cmdBuffer waitUntilCompleted];

            if (cmdBuffer.status == MTLCommandBufferStatusError) {
                set_error("Kernel execution failed");
                return BackendResult::LaunchFailed;
            }
        }

        return BackendResult::Success;
    }

    // ========================================================================
    // Stream Management
    // ========================================================================

    StreamHandle create_stream() override {
        std::lock_guard<std::mutex> lock(m_mutex);

        id<MTLCommandQueue> queue = [m_device newCommandQueue];
        if (queue == nil) {
            set_error("Failed to create command queue");
            return StreamHandle::Invalid();
        }

        StreamHandle handle;
        handle.id = m_next_stream_id++;

        m_streams[handle.id] = queue;

        return handle;
    }

    void destroy_stream(StreamHandle stream) override {
        std::lock_guard<std::mutex> lock(m_mutex);

        auto it = m_streams.find(stream.id);
        if (it != m_streams.end()) {
            it->second = nil;
            m_streams.erase(it);
        }
    }

    BackendResult synchronize_stream(StreamHandle stream) override {
        // Metal doesn't have stream-level synchronization in the same way
        // We need to wait for all commands in the queue
        // For now, synchronize globally
        return synchronize();
    }

    BackendResult synchronize() override {
        if (!m_initialized) return BackendResult::NotInitialized;

        // Create and commit an empty command buffer and wait
        id<MTLCommandBuffer> cmdBuffer = [m_default_queue commandBuffer];
        [cmdBuffer commit];
        [cmdBuffer waitUntilCompleted];

        return BackendResult::Success;
    }

    // ========================================================================
    // Events
    // ========================================================================

    EventHandle record_event(StreamHandle stream = StreamHandle::Default()) override {
        std::lock_guard<std::mutex> lock(m_mutex);

        EventInfo info;
        info.timestamp = std::chrono::high_resolution_clock::now();

        // Create Metal event for synchronization
        if (@available(macOS 10.14, *)) {
            info.event = [m_device newEvent];
        }

        EventHandle handle;
        handle.id = m_next_event_id++;

        m_events[handle.id] = info;

        return handle;
    }

    BackendResult wait_event(EventHandle event) override {
        // Events in our implementation are timestamps
        // Just ensure synchronization
        return synchronize();
    }

    BackendResult stream_wait_event(StreamHandle stream, EventHandle event) override {
        // Synchronize stream with event
        return synchronize();
    }

    Real elapsed_time(EventHandle start, EventHandle end) override {
        std::lock_guard<std::mutex> lock(m_mutex);

        auto start_it = m_events.find(start.id);
        auto end_it = m_events.find(end.id);

        if (start_it == m_events.end() || end_it == m_events.end()) {
            return 0.0;
        }

        auto duration = end_it->second.timestamp - start_it->second.timestamp;
        return std::chrono::duration<Real, std::milli>(duration).count();
    }

    void destroy_event(EventHandle event) override {
        std::lock_guard<std::mutex> lock(m_mutex);

        auto it = m_events.find(event.id);
        if (it != m_events.end()) {
            it->second.event = nil;
            m_events.erase(it);
        }
    }

    // ========================================================================
    // Error Handling
    // ========================================================================

    const std::string& last_error() const override { return m_last_error; }

    void clear_error() override {
        m_last_error.clear();
        m_has_error = false;
    }

    bool has_error() const override { return m_has_error; }

    // ========================================================================
    // Metal-specific helpers
    // ========================================================================

    id<MTLBuffer> get_buffer(BufferHandle handle) const {
        auto it = m_buffers.find(handle.id);
        if (it != m_buffers.end()) {
            return it->second.buffer;
        }
        return nil;
    }

private:
    void set_error(const std::string& error) {
        m_last_error = error;
        m_has_error = true;
    }

    id<MTLCommandQueue> get_queue(StreamHandle stream) {
        if (stream.is_valid() && stream.id != 1) {
            auto it = m_streams.find(stream.id);
            if (it != m_streams.end()) {
                return it->second;
            }
        }
        return m_default_queue;
    }

    struct BufferInfo {
        id<MTLBuffer> buffer;
        SizeT size;
        MemoryType type;
        void* mapped_ptr;
    };

    struct EventInfo {
        id<MTLEvent> event;
        std::chrono::high_resolution_clock::time_point timestamp;
    };

    // Core Metal objects
    id<MTLDevice> m_device{nil};
    id<MTLCommandQueue> m_default_queue{nil};
    std::vector<id<MTLDevice>> m_devices;

    // State
    bool m_initialized{false};
    bool m_enable_profiling{false};
    UInt32 m_current_device{0};
    std::string m_name;

    // Resource tracking
    std::unordered_map<UInt64, BufferInfo> m_buffers;
    std::unordered_map<UInt64, id<MTLCommandQueue>> m_streams;
    std::unordered_map<UInt64, EventInfo> m_events;

    // Memory tracking
    std::atomic<SizeT> m_allocated_memory{0};

    // Handle generation
    std::atomic<UInt64> m_next_buffer_id{1};
    std::atomic<UInt64> m_next_stream_id{2};  // 1 is default
    std::atomic<UInt64> m_next_event_id{1};

    // Error state
    mutable std::string m_last_error;
    mutable bool m_has_error{false};

    // Thread safety
    mutable std::mutex m_mutex;
};

// ============================================================================
// MetalKernel Implementation
// ============================================================================

BackendResult MetalKernel::set_arg(UInt32 index, const KernelArg& arg) {
    // Expand arguments vector if needed
    if (index >= m_arguments.size()) {
        m_arguments.resize(index + 1);
    }
    m_arguments[index] = arg;
    return BackendResult::Success;
}

// ============================================================================
// Backend Registration
// ============================================================================

namespace detail {

std::unique_ptr<IComputeBackend> create_metal_backend() {
    return std::make_unique<MetalBackend>();
}

bool is_metal_available() {
    // Check if Metal is available
    NSArray<id<MTLDevice>>* devices = MTLCopyAllDevices();
    return devices != nil && devices.count > 0;
}

} // namespace detail

} // namespace jaguar::gpu

#endif // JAGUAR_HAS_METAL
