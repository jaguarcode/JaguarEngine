/**
 * @file metal_tests.mm
 * @brief Unit tests for Metal compute backend
 *
 * These tests verify the Metal backend implementation, including:
 * - Backend initialization and device enumeration
 * - Memory allocation and transfer operations
 * - Kernel compilation and execution
 * - Stream and event management
 * - Error handling and recovery
 *
 * Note: These tests require a Metal-capable device to run (macOS/iOS).
 * Tests will be skipped if Metal is not available.
 */

#ifdef JAGUAR_HAS_METAL

#include <gtest/gtest.h>
#include "jaguar/gpu/compute_backend.h"
#include <numeric>
#include <vector>
#include <cmath>

using namespace jaguar;
using namespace jaguar::gpu;

// ============================================================================
// Test Fixture
// ============================================================================

class MetalBackendTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Try to create Metal backend
        backend = BackendFactory::create(BackendType::Metal);

        // Skip tests if Metal is not available
        if (!backend) {
            GTEST_SKIP() << "Metal backend not available";
        }

        // Initialize the backend
        auto result = backend->initialize();
        if (result != BackendResult::Success) {
            backend.reset();
            GTEST_SKIP() << "Failed to initialize Metal backend: "
                         << result_to_string(result);
        }
    }

    void TearDown() override {
        if (backend && backend->is_initialized()) {
            backend->shutdown();
        }
    }

    std::unique_ptr<IComputeBackend> backend;
};

// ============================================================================
// Backend Availability Test
// ============================================================================

TEST(MetalAvailabilityTest, CheckMetalAvailability) {
    bool is_available = BackendFactory::is_available(BackendType::Metal);
    if (is_available) {
        std::cout << "Metal backend is available" << std::endl;
    } else {
        std::cout << "Metal backend is NOT available (no Metal device found)"
                  << std::endl;
    }
    // This test always passes - it just reports availability
    SUCCEED();
}

// ============================================================================
// Initialization Tests
// ============================================================================

TEST_F(MetalBackendTest, BackendType) {
    EXPECT_EQ(backend->type(), BackendType::Metal);
}

TEST_F(MetalBackendTest, BackendName) {
    const std::string& name = backend->name();
    EXPECT_FALSE(name.empty());
    EXPECT_TRUE(name.find("Metal") != std::string::npos);
    std::cout << "Backend name: " << name << std::endl;
}

TEST_F(MetalBackendTest, IsInitialized) {
    EXPECT_TRUE(backend->is_initialized());
}

TEST_F(MetalBackendTest, DeviceCount) {
    EXPECT_GE(backend->device_count(), 1u);
}

TEST_F(MetalBackendTest, DeviceCapabilities) {
    auto caps = backend->get_device_capabilities(0);

    EXPECT_EQ(caps.backend_type, BackendType::Metal);
    EXPECT_TRUE(caps.device_type == DeviceType::DiscreteGPU ||
                caps.device_type == DeviceType::IntegratedGPU);

    // Metal devices should have resources
    EXPECT_GT(caps.global_memory, 0u);
    EXPECT_GT(caps.max_workgroup_size, 0u);

    // Print device info for debugging
    std::cout << "Device: " << caps.name << std::endl;
    std::cout << "  Vendor: " << caps.vendor << std::endl;
    std::cout << "  Device type: " << device_type_to_string(caps.device_type) << std::endl;
    std::cout << "  Global memory: " << (caps.global_memory / (1024*1024)) << " MB" << std::endl;
    std::cout << "  Local memory: " << (caps.local_memory / 1024) << " KB" << std::endl;
    std::cout << "  Max work-group size: " << caps.max_workgroup_size << std::endl;
    std::cout << "  SIMD width: " << caps.warp_size << std::endl;
    std::cout << "  Supports unified memory: " << (caps.supports_unified_memory ? "yes" : "no") << std::endl;
    std::cout << "  Compute capability: " << caps.compute_capability << std::endl;
}

TEST_F(MetalBackendTest, SetDevice) {
    EXPECT_EQ(backend->current_device(), 0u);
    EXPECT_EQ(backend->set_device(0), BackendResult::Success);

    // Try to set an invalid device
    if (backend->device_count() == 1) {
        EXPECT_EQ(backend->set_device(1), BackendResult::DeviceNotFound);
    }
}

// ============================================================================
// Memory Management Tests
// ============================================================================

TEST_F(MetalBackendTest, AllocateDeviceMemory) {
    constexpr SizeT size = 1024 * 1024;  // 1 MB
    auto buffer = backend->allocate(size, MemoryType::DeviceLocal);

    EXPECT_TRUE(buffer.is_valid());
    EXPECT_EQ(buffer.size, size);
    EXPECT_EQ(buffer.type, MemoryType::DeviceLocal);

    EXPECT_GE(backend->allocated_memory(), size);

    backend->free(buffer);
    EXPECT_EQ(backend->allocated_memory(), 0u);
}

TEST_F(MetalBackendTest, AllocateSharedMemory) {
    constexpr SizeT size = 1024;
    auto buffer = backend->allocate(size, MemoryType::Shared);

    EXPECT_TRUE(buffer.is_valid());
    EXPECT_EQ(buffer.type, MemoryType::Shared);

    backend->free(buffer);
}

TEST_F(MetalBackendTest, AllocateHostVisibleMemory) {
    constexpr SizeT size = 1024;
    auto buffer = backend->allocate(size, MemoryType::HostVisible);

    EXPECT_TRUE(buffer.is_valid());
    EXPECT_EQ(buffer.type, MemoryType::HostVisible);

    backend->free(buffer);
}

TEST_F(MetalBackendTest, AllocateZeroSize) {
    auto buffer = backend->allocate(0);
    EXPECT_FALSE(buffer.is_valid());
}

TEST_F(MetalBackendTest, UploadDownload) {
    constexpr SizeT count = 1024;
    constexpr SizeT size = count * sizeof(float);

    // Create source data
    std::vector<float> src(count);
    for (SizeT i = 0; i < count; ++i) {
        src[i] = static_cast<float>(i) * 0.5f;
    }

    // Allocate buffer (use shared for easier upload/download)
    auto buffer = backend->allocate(size, MemoryType::Shared);
    ASSERT_TRUE(buffer.is_valid());

    // Upload
    EXPECT_EQ(backend->upload(buffer, src.data(), size), BackendResult::Success);

    // Synchronize to ensure upload completes
    EXPECT_EQ(backend->synchronize(), BackendResult::Success);

    // Download
    std::vector<float> dst(count, -1.0f);
    EXPECT_EQ(backend->download(buffer, dst.data(), size), BackendResult::Success);

    // Verify
    EXPECT_EQ(src, dst);

    backend->free(buffer);
}

TEST_F(MetalBackendTest, UploadDownloadPrivate) {
    constexpr SizeT count = 1024;
    constexpr SizeT size = count * sizeof(float);

    // Create source data
    std::vector<float> src(count);
    for (SizeT i = 0; i < count; ++i) {
        src[i] = static_cast<float>(i) * 0.5f;
    }

    // Allocate private buffer (requires blit operations)
    auto buffer = backend->allocate(size, MemoryType::DeviceLocal);
    ASSERT_TRUE(buffer.is_valid());

    // Upload
    EXPECT_EQ(backend->upload(buffer, src.data(), size), BackendResult::Success);

    // Synchronize
    EXPECT_EQ(backend->synchronize(), BackendResult::Success);

    // Download
    std::vector<float> dst(count, -1.0f);
    EXPECT_EQ(backend->download(buffer, dst.data(), size), BackendResult::Success);

    // Verify
    EXPECT_EQ(src, dst);

    backend->free(buffer);
}

TEST_F(MetalBackendTest, UploadDownloadWithOffset) {
    constexpr SizeT size = 4096;
    auto buffer = backend->allocate(size, MemoryType::Shared);
    ASSERT_TRUE(buffer.is_valid());

    // Upload to middle of buffer
    std::vector<UInt8> data(1024, 0xAB);
    EXPECT_EQ(backend->upload(buffer, data.data(), data.size(), 1024),
              BackendResult::Success);

    backend->synchronize();

    // Download from same location
    std::vector<UInt8> result(1024, 0);
    EXPECT_EQ(backend->download(buffer, result.data(), result.size(), 1024),
              BackendResult::Success);

    EXPECT_EQ(data, result);

    backend->free(buffer);
}

TEST_F(MetalBackendTest, CopyBuffer) {
    constexpr SizeT count = 512;
    constexpr SizeT size = count * sizeof(Int32);

    std::vector<Int32> data(count);
    std::iota(data.begin(), data.end(), 100);

    auto src = backend->allocate(size, MemoryType::Shared);
    auto dst = backend->allocate(size, MemoryType::Shared);
    ASSERT_TRUE(src.is_valid());
    ASSERT_TRUE(dst.is_valid());

    // Upload to source
    EXPECT_EQ(backend->upload(src, data.data(), size), BackendResult::Success);

    // Copy to destination
    EXPECT_EQ(backend->copy(src, dst, size), BackendResult::Success);

    backend->synchronize();

    // Download and verify
    std::vector<Int32> result(count, 0);
    EXPECT_EQ(backend->download(dst, result.data(), size), BackendResult::Success);

    EXPECT_EQ(data, result);

    backend->free(src);
    backend->free(dst);
}

TEST_F(MetalBackendTest, FillBuffer) {
    constexpr SizeT size = 4096;
    auto buffer = backend->allocate(size, MemoryType::Shared);
    ASSERT_TRUE(buffer.is_valid());

    // Fill with single byte pattern
    UInt8 pattern = 0x42;
    EXPECT_EQ(backend->fill(buffer, &pattern, 1), BackendResult::Success);

    backend->synchronize();

    // Verify
    std::vector<UInt8> result(size);
    EXPECT_EQ(backend->download(buffer, result.data(), size), BackendResult::Success);

    for (const auto& val : result) {
        EXPECT_EQ(val, pattern);
    }

    backend->free(buffer);
}

TEST_F(MetalBackendTest, MapUnmap) {
    constexpr SizeT count = 64;
    constexpr SizeT size = count * sizeof(float);

    // Use shared memory for mapping
    auto buffer = backend->allocate(size, MemoryType::Shared);
    ASSERT_TRUE(buffer.is_valid());

    // Map buffer
    auto* ptr = static_cast<float*>(backend->map(buffer, MemoryAccess::ReadWrite));
    ASSERT_NE(ptr, nullptr);

    // Write data
    for (SizeT i = 0; i < count; ++i) {
        ptr[i] = static_cast<float>(i) * 2.5f;
    }

    // Unmap
    backend->unmap(buffer);

    // Synchronize
    backend->synchronize();

    // Download and verify
    std::vector<float> result(count);
    EXPECT_EQ(backend->download(buffer, result.data(), size), BackendResult::Success);

    for (SizeT i = 0; i < count; ++i) {
        EXPECT_FLOAT_EQ(result[i], static_cast<float>(i) * 2.5f);
    }

    backend->free(buffer);
}

TEST_F(MetalBackendTest, CannotMapPrivate) {
    constexpr SizeT size = 1024;
    auto buffer = backend->allocate(size, MemoryType::DeviceLocal);
    ASSERT_TRUE(buffer.is_valid());

    // Private storage cannot be mapped
    void* ptr = backend->map(buffer, MemoryAccess::ReadWrite);
    EXPECT_EQ(ptr, nullptr);
    EXPECT_TRUE(backend->has_error());

    backend->clear_error();
    backend->free(buffer);
}

TEST_F(MetalBackendTest, AvailableMemory) {
    SizeT available = backend->available_memory();
    EXPECT_GT(available, 0u);

    std::cout << "Available memory: " << (available / (1024*1024)) << " MB" << std::endl;
}

// ============================================================================
// Stream (Command Queue) Tests
// ============================================================================

TEST_F(MetalBackendTest, CreateStream) {
    auto stream = backend->create_stream();
    EXPECT_TRUE(stream.is_valid());
    backend->destroy_stream(stream);
}

TEST_F(MetalBackendTest, MultipleStreams) {
    std::vector<StreamHandle> streams;
    for (int i = 0; i < 4; ++i) {
        auto stream = backend->create_stream();
        EXPECT_TRUE(stream.is_valid());
        streams.push_back(stream);
    }

    for (auto stream : streams) {
        backend->destroy_stream(stream);
    }
}

TEST_F(MetalBackendTest, StreamSynchronize) {
    auto stream = backend->create_stream();
    ASSERT_TRUE(stream.is_valid());

    EXPECT_EQ(backend->synchronize_stream(stream), BackendResult::Success);

    backend->destroy_stream(stream);
}

// ============================================================================
// Event Tests
// ============================================================================

TEST_F(MetalBackendTest, RecordEvent) {
    auto event = backend->record_event();
    EXPECT_TRUE(event.is_valid());
    backend->destroy_event(event);
}

TEST_F(MetalBackendTest, WaitEvent) {
    auto event = backend->record_event();
    EXPECT_TRUE(event.is_valid());

    EXPECT_EQ(backend->wait_event(event), BackendResult::Success);

    backend->destroy_event(event);
}

TEST_F(MetalBackendTest, ElapsedTime) {
    auto start = backend->record_event();

    // Do some work
    constexpr SizeT size = 16 * 1024 * 1024;  // 16 MB
    auto buffer = backend->allocate(size, MemoryType::Shared);
    if (buffer.is_valid()) {
        std::vector<UInt8> data(size, 0xFF);
        backend->upload(buffer, data.data(), size);
        backend->synchronize();
        backend->free(buffer);
    }

    auto end = backend->record_event();

    backend->wait_event(end);

    Real elapsed = backend->elapsed_time(start, end);
    EXPECT_GE(elapsed, 0.0);

    std::cout << "Elapsed time for 16MB transfer: " << elapsed << " ms" << std::endl;

    backend->destroy_event(start);
    backend->destroy_event(end);
}

// ============================================================================
// Kernel Compilation and Execution Tests
// ============================================================================

TEST_F(MetalBackendTest, CompileKernel) {
    // Metal Shading Language (MSL) kernel
    const char* source = R"(
#include <metal_stdlib>
using namespace metal;

kernel void vector_add(device float* a [[buffer(0)]],
                       device float* b [[buffer(1)]],
                       device float* c [[buffer(2)]],
                       constant int& n [[buffer(3)]],
                       uint idx [[thread_position_in_grid]]) {
    if (idx < uint(n)) {
        c[idx] = a[idx] + b[idx];
    }
}
)";

    auto kernel = backend->create_kernel("vector_add", source);
    if (!kernel) {
        std::cout << "Kernel compilation failed: " << backend->last_error() << std::endl;
        GTEST_SKIP() << "Kernel compilation not supported or failed";
    }

    EXPECT_TRUE(kernel->is_valid());
    EXPECT_EQ(kernel->name(), "vector_add");
    EXPECT_GT(kernel->max_workgroup_size(), 0u);

    std::cout << "Kernel info:" << std::endl;
    std::cout << "  Max threads per threadgroup: " << kernel->max_workgroup_size() << std::endl;
    std::cout << "  Preferred threadgroup size: " << kernel->preferred_workgroup_size() << std::endl;
    std::cout << "  Threadgroup memory: " << kernel->local_memory_size() << " bytes" << std::endl;
}

TEST_F(MetalBackendTest, ExecuteKernel) {
    const char* source = R"(
#include <metal_stdlib>
using namespace metal;

kernel void vector_add(device float* a [[buffer(0)]],
                       device float* b [[buffer(1)]],
                       device float* c [[buffer(2)]],
                       constant int& n [[buffer(3)]],
                       uint idx [[thread_position_in_grid]]) {
    if (idx < uint(n)) {
        c[idx] = a[idx] + b[idx];
    }
}
)";

    auto kernel = backend->create_kernel("vector_add", source);
    if (!kernel) {
        GTEST_SKIP() << "Kernel compilation failed: " << backend->last_error();
    }

    // Test parameters
    constexpr int N = 1024;
    constexpr SizeT size = N * sizeof(float);

    // Prepare input data
    std::vector<float> h_a(N), h_b(N), h_c(N, 0.0f);
    for (int i = 0; i < N; ++i) {
        h_a[i] = static_cast<float>(i);
        h_b[i] = static_cast<float>(i * 2);
    }

    // Allocate device buffers
    auto d_a = backend->allocate(size, MemoryType::Shared);
    auto d_b = backend->allocate(size, MemoryType::Shared);
    auto d_c = backend->allocate(size, MemoryType::Shared);
    ASSERT_TRUE(d_a.is_valid());
    ASSERT_TRUE(d_b.is_valid());
    ASSERT_TRUE(d_c.is_valid());

    // Upload input data
    EXPECT_EQ(backend->upload(d_a, h_a.data(), size), BackendResult::Success);
    EXPECT_EQ(backend->upload(d_b, h_b.data(), size), BackendResult::Success);

    // Set kernel arguments
    EXPECT_EQ(kernel->set_arg(0, KernelArg::Buffer(d_a)), BackendResult::Success);
    EXPECT_EQ(kernel->set_arg(1, KernelArg::Buffer(d_b)), BackendResult::Success);
    EXPECT_EQ(kernel->set_arg(2, KernelArg::Buffer(d_c)), BackendResult::Success);
    EXPECT_EQ(kernel->set_arg(3, KernelArg::Int(N)), BackendResult::Success);

    // Launch kernel
    auto config = LaunchConfig::Linear(N, 256);
    EXPECT_EQ(backend->dispatch(kernel.get(), config), BackendResult::Success);

    // Synchronize
    EXPECT_EQ(backend->synchronize(), BackendResult::Success);

    // Download results
    EXPECT_EQ(backend->download(d_c, h_c.data(), size), BackendResult::Success);

    // Verify results
    for (int i = 0; i < N; ++i) {
        float expected = h_a[i] + h_b[i];
        EXPECT_FLOAT_EQ(h_c[i], expected) << "Mismatch at index " << i;
    }

    // Cleanup
    backend->free(d_a);
    backend->free(d_b);
    backend->free(d_c);
}

TEST_F(MetalBackendTest, ExecuteKernelWithThreadgroupMemory) {
    const char* source = R"(
#include <metal_stdlib>
using namespace metal;

kernel void reduce_sum(device float* input [[buffer(0)]],
                       device float* output [[buffer(1)]],
                       constant int& n [[buffer(2)]],
                       threadgroup float* shared [[threadgroup(0)]],
                       uint tid [[thread_index_in_threadgroup]],
                       uint idx [[thread_position_in_grid]],
                       uint group_id [[threadgroup_position_in_grid]],
                       uint group_size [[threads_per_threadgroup]]) {
    // Load into threadgroup memory
    shared[tid] = (idx < uint(n)) ? input[idx] : 0.0f;
    threadgroup_barrier(mem_flags::mem_threadgroup);

    // Reduction in threadgroup memory
    for (uint s = group_size / 2; s > 0; s >>= 1) {
        if (tid < s) {
            shared[tid] += shared[tid + s];
        }
        threadgroup_barrier(mem_flags::mem_threadgroup);
    }

    // Write result
    if (tid == 0) {
        output[group_id] = shared[0];
    }
}
)";

    auto kernel = backend->create_kernel("reduce_sum", source);
    if (!kernel) {
        GTEST_SKIP() << "Kernel compilation failed: " << backend->last_error();
    }

    constexpr int N = 1024;
    constexpr int BLOCK_SIZE = 256;
    constexpr int NUM_BLOCKS = (N + BLOCK_SIZE - 1) / BLOCK_SIZE;
    constexpr SizeT input_size = N * sizeof(float);
    constexpr SizeT output_size = NUM_BLOCKS * sizeof(float);

    // Prepare input
    std::vector<float> h_input(N);
    float expected_sum = 0.0f;
    for (int i = 0; i < N; ++i) {
        h_input[i] = static_cast<float>(i + 1);
        expected_sum += h_input[i];
    }

    // Allocate
    auto d_input = backend->allocate(input_size, MemoryType::Shared);
    auto d_output = backend->allocate(output_size, MemoryType::Shared);
    ASSERT_TRUE(d_input.is_valid());
    ASSERT_TRUE(d_output.is_valid());

    // Upload
    EXPECT_EQ(backend->upload(d_input, h_input.data(), input_size), BackendResult::Success);

    // Set arguments
    kernel->set_arg(0, KernelArg::Buffer(d_input));
    kernel->set_arg(1, KernelArg::Buffer(d_output));
    kernel->set_arg(2, KernelArg::Int(N));
    kernel->set_arg(3, KernelArg::LocalMemory(BLOCK_SIZE * sizeof(float)));

    // Launch
    LaunchConfig config;
    config.grid_x = N;  // Metal uses threads per grid
    config.block_x = BLOCK_SIZE;
    config.shared_memory = BLOCK_SIZE * sizeof(float);

    EXPECT_EQ(backend->dispatch(kernel.get(), config), BackendResult::Success);
    EXPECT_EQ(backend->synchronize(), BackendResult::Success);

    // Download partial results
    std::vector<float> h_output(NUM_BLOCKS);
    EXPECT_EQ(backend->download(d_output, h_output.data(), output_size), BackendResult::Success);

    // Sum partial results on CPU
    float total = 0.0f;
    for (int i = 0; i < NUM_BLOCKS; ++i) {
        total += h_output[i];
    }

    // Verify (allow small floating point tolerance)
    EXPECT_NEAR(total, expected_sum, expected_sum * 1e-5);

    backend->free(d_input);
    backend->free(d_output);
}

TEST_F(MetalBackendTest, ExecuteKernel2D) {
    const char* source = R"(
#include <metal_stdlib>
using namespace metal;

kernel void matrix_scale(device float* matrix [[buffer(0)]],
                         constant float& scale [[buffer(1)]],
                         constant int& rows [[buffer(2)]],
                         constant int& cols [[buffer(3)]],
                         uint2 pos [[thread_position_in_grid]]) {
    int row = pos.x;
    int col = pos.y;
    if (row < rows && col < cols) {
        int idx = row * cols + col;
        matrix[idx] *= scale;
    }
}
)";

    auto kernel = backend->create_kernel("matrix_scale", source);
    if (!kernel) {
        GTEST_SKIP() << "Kernel compilation failed: " << backend->last_error();
    }

    constexpr int ROWS = 64;
    constexpr int COLS = 128;
    constexpr SizeT size = ROWS * COLS * sizeof(float);
    constexpr float SCALE = 2.5f;

    // Prepare input
    std::vector<float> h_matrix(ROWS * COLS);
    for (int i = 0; i < ROWS * COLS; ++i) {
        h_matrix[i] = static_cast<float>(i);
    }

    auto d_matrix = backend->allocate(size, MemoryType::Shared);
    ASSERT_TRUE(d_matrix.is_valid());

    EXPECT_EQ(backend->upload(d_matrix, h_matrix.data(), size), BackendResult::Success);

    // Set arguments
    kernel->set_arg(0, KernelArg::Buffer(d_matrix));
    kernel->set_arg(1, KernelArg::Float(SCALE));
    kernel->set_arg(2, KernelArg::Int(ROWS));
    kernel->set_arg(3, KernelArg::Int(COLS));

    // Launch 2D kernel
    LaunchConfig config;
    config.grid_x = ROWS;
    config.grid_y = COLS;
    config.block_x = 16;
    config.block_y = 16;

    EXPECT_EQ(backend->dispatch(kernel.get(), config), BackendResult::Success);
    EXPECT_EQ(backend->synchronize(), BackendResult::Success);

    // Download and verify
    std::vector<float> result(ROWS * COLS);
    EXPECT_EQ(backend->download(d_matrix, result.data(), size), BackendResult::Success);

    for (int i = 0; i < ROWS * COLS; ++i) {
        float expected = static_cast<float>(i) * SCALE;
        EXPECT_FLOAT_EQ(result[i], expected) << "Mismatch at index " << i;
    }

    backend->free(d_matrix);
}

// ============================================================================
// Error Handling Tests
// ============================================================================

TEST_F(MetalBackendTest, InvalidBufferUpload) {
    BufferHandle invalid;
    std::vector<UInt8> data(100);

    EXPECT_EQ(backend->upload(invalid, data.data(), data.size()),
              BackendResult::InvalidArgument);
    EXPECT_TRUE(backend->has_error());
}

TEST_F(MetalBackendTest, InvalidBufferDownload) {
    BufferHandle invalid;
    std::vector<UInt8> data(100);

    EXPECT_EQ(backend->download(invalid, data.data(), data.size()),
              BackendResult::InvalidArgument);
}

TEST_F(MetalBackendTest, UploadExceedsSize) {
    auto buffer = backend->allocate(100, MemoryType::Shared);
    ASSERT_TRUE(buffer.is_valid());

    std::vector<UInt8> data(200);
    EXPECT_EQ(backend->upload(buffer, data.data(), data.size()),
              BackendResult::InvalidArgument);

    backend->free(buffer);
}

TEST_F(MetalBackendTest, ClearError) {
    // Cause an error
    BufferHandle invalid;
    backend->upload(invalid, nullptr, 0);

    EXPECT_TRUE(backend->has_error());
    EXPECT_FALSE(backend->last_error().empty());

    backend->clear_error();

    EXPECT_FALSE(backend->has_error());
    EXPECT_TRUE(backend->last_error().empty());
}

TEST_F(MetalBackendTest, KernelCompilationError) {
    const char* bad_source = R"(
#include <metal_stdlib>
using namespace metal;

kernel void bad_kernel() {
    this_function_does_not_exist();
}
)";

    auto kernel = backend->create_kernel("bad_kernel", bad_source);
    EXPECT_EQ(kernel, nullptr);
    EXPECT_TRUE(backend->has_error());
    std::cout << "Expected compilation error: " << backend->last_error() << std::endl;
}

// ============================================================================
// Performance Benchmark Tests
// ============================================================================

TEST_F(MetalBackendTest, BandwidthTest) {
    // Test host-to-device and device-to-host bandwidth
    constexpr SizeT sizes[] = {
        1 * 1024 * 1024,   // 1 MB
        16 * 1024 * 1024,  // 16 MB
        64 * 1024 * 1024,  // 64 MB
    };

    for (SizeT size : sizes) {
        std::vector<UInt8> h_data(size, 0x42);
        auto buffer = backend->allocate(size, MemoryType::Shared);

        if (!buffer.is_valid()) {
            std::cout << "Skipping " << (size / (1024*1024)) << " MB test (allocation failed)" << std::endl;
            continue;
        }

        // Measure upload bandwidth
        auto start = backend->record_event();
        backend->upload(buffer, h_data.data(), size);
        auto end = backend->record_event();
        backend->wait_event(end);

        Real upload_time = backend->elapsed_time(start, end);
        Real upload_bw = (upload_time > 0) ? (size / (upload_time / 1000.0) / (1024*1024*1024)) : 0;

        // Measure download bandwidth
        auto start2 = backend->record_event();
        backend->download(buffer, h_data.data(), size);
        auto end2 = backend->record_event();
        backend->wait_event(end2);

        Real download_time = backend->elapsed_time(start2, end2);
        Real download_bw = (download_time > 0) ? (size / (download_time / 1000.0) / (1024*1024*1024)) : 0;

        std::cout << "Size: " << (size / (1024*1024)) << " MB" << std::endl;
        std::cout << "  Upload:   " << upload_time << " ms (" << upload_bw << " GB/s)" << std::endl;
        std::cout << "  Download: " << download_time << " ms (" << download_bw << " GB/s)" << std::endl;

        backend->free(buffer);
        backend->destroy_event(start);
        backend->destroy_event(end);
        backend->destroy_event(start2);
        backend->destroy_event(end2);
    }
}

TEST_F(MetalBackendTest, KernelExecutionBenchmark) {
    const char* source = R"(
#include <metal_stdlib>
using namespace metal;

kernel void saxpy(device float* y [[buffer(0)]],
                  device const float* x [[buffer(1)]],
                  constant float& a [[buffer(2)]],
                  constant int& n [[buffer(3)]],
                  uint idx [[thread_position_in_grid]]) {
    if (idx < uint(n)) {
        y[idx] = a * x[idx] + y[idx];
    }
}
)";

    auto kernel = backend->create_kernel("saxpy", source);
    if (!kernel) {
        GTEST_SKIP() << "Kernel compilation failed";
    }

    constexpr int N = 1024 * 1024;  // 1M elements
    constexpr SizeT size = N * sizeof(float);
    constexpr float A = 2.0f;
    constexpr int ITERATIONS = 100;

    std::vector<float> h_x(N, 1.0f);
    std::vector<float> h_y(N, 1.0f);

    auto d_x = backend->allocate(size, MemoryType::Shared);
    auto d_y = backend->allocate(size, MemoryType::Shared);

    EXPECT_EQ(backend->upload(d_x, h_x.data(), size), BackendResult::Success);
    EXPECT_EQ(backend->upload(d_y, h_y.data(), size), BackendResult::Success);

    kernel->set_arg(0, KernelArg::Buffer(d_y));
    kernel->set_arg(1, KernelArg::Buffer(d_x));
    kernel->set_arg(2, KernelArg::Float(A));
    kernel->set_arg(3, KernelArg::Int(N));

    auto config = LaunchConfig::Linear(N, 256);

    // Warm up
    for (int i = 0; i < 10; ++i) {
        backend->dispatch(kernel.get(), config);
    }
    backend->synchronize();

    // Benchmark
    auto start = backend->record_event();
    for (int i = 0; i < ITERATIONS; ++i) {
        backend->dispatch(kernel.get(), config);
    }
    auto end = backend->record_event();
    backend->wait_event(end);

    Real elapsed = backend->elapsed_time(start, end);
    Real avg_time = elapsed / ITERATIONS;
    Real bandwidth = (2.0 * size / (1024*1024*1024)) / (avg_time / 1000.0);  // 1 read + 1 write

    std::cout << "SAXPY benchmark (N=" << N << ", " << ITERATIONS << " iterations):" << std::endl;
    std::cout << "  Total time: " << elapsed << " ms" << std::endl;
    std::cout << "  Average time: " << avg_time << " ms" << std::endl;
    std::cout << "  Effective bandwidth: " << bandwidth << " GB/s" << std::endl;

    backend->free(d_x);
    backend->free(d_y);
    backend->destroy_event(start);
    backend->destroy_event(end);
}

// ============================================================================
// Apple Silicon Specific Tests
// ============================================================================

TEST_F(MetalBackendTest, UnifiedMemoryArchitecture) {
    auto caps = backend->get_device_capabilities(0);

    // All Apple Silicon devices support unified memory
    EXPECT_TRUE(caps.supports_unified_memory);

    // Test that shared memory works efficiently (no copies on UMA)
    constexpr SizeT size = 1024 * sizeof(float);

    auto buffer = backend->allocate(size, MemoryType::Shared);
    ASSERT_TRUE(buffer.is_valid());

    // Map should return directly accessible pointer on UMA
    auto* ptr = static_cast<float*>(backend->map(buffer, MemoryAccess::ReadWrite));
    EXPECT_NE(ptr, nullptr);

    // Write directly
    for (int i = 0; i < 1024; ++i) {
        ptr[i] = static_cast<float>(i);
    }

    backend->unmap(buffer);

    // Should be readable immediately on UMA (after sync)
    backend->synchronize();

    std::vector<float> result(1024);
    backend->download(buffer, result.data(), size);

    for (int i = 0; i < 1024; ++i) {
        EXPECT_FLOAT_EQ(result[i], static_cast<float>(i));
    }

    backend->free(buffer);
}

// ============================================================================
// Main
// ============================================================================

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

#else // !JAGUAR_HAS_METAL

// Provide a stub main when Metal is not available
#include <iostream>

int main(int argc, char** argv) {
    std::cout << "Metal tests skipped - Metal backend not compiled" << std::endl;
    return 0;
}

#endif // JAGUAR_HAS_METAL
