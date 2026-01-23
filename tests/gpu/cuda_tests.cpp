/**
 * @file cuda_tests.cpp
 * @brief Unit tests for CUDA compute backend
 *
 * These tests verify the CUDA backend implementation, including:
 * - Backend initialization and device enumeration
 * - Memory allocation and transfer operations
 * - Kernel compilation and execution
 * - Stream and event management
 * - Error handling and recovery
 *
 * Note: These tests require a CUDA-capable GPU to run.
 * Tests will be skipped if CUDA is not available.
 */

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

class CUDABackendTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Try to create CUDA backend
        backend = BackendFactory::create(BackendType::CUDA);

        // Skip tests if CUDA is not available
        if (!backend) {
            GTEST_SKIP() << "CUDA backend not available";
        }

        // Initialize the backend
        auto result = backend->initialize();
        if (result != BackendResult::Success) {
            backend.reset();
            GTEST_SKIP() << "Failed to initialize CUDA backend: "
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

TEST(CUDAAvailabilityTest, CheckCUDAAvailability) {
    bool is_available = BackendFactory::is_available(BackendType::CUDA);
    if (is_available) {
        std::cout << "CUDA backend is available" << std::endl;
    } else {
        std::cout << "CUDA backend is NOT available (no CUDA device found)"
                  << std::endl;
    }
    // This test always passes - it just reports availability
    SUCCEED();
}

// ============================================================================
// Initialization Tests
// ============================================================================

TEST_F(CUDABackendTest, BackendType) {
    EXPECT_EQ(backend->type(), BackendType::CUDA);
}

TEST_F(CUDABackendTest, BackendName) {
    const std::string& name = backend->name();
    EXPECT_FALSE(name.empty());
    EXPECT_TRUE(name.find("CUDA") != std::string::npos);
    std::cout << "Backend name: " << name << std::endl;
}

TEST_F(CUDABackendTest, IsInitialized) {
    EXPECT_TRUE(backend->is_initialized());
}

TEST_F(CUDABackendTest, DeviceCount) {
    EXPECT_GE(backend->device_count(), 1u);
}

TEST_F(CUDABackendTest, DeviceCapabilities) {
    auto caps = backend->get_device_capabilities(0);

    EXPECT_EQ(caps.backend_type, BackendType::CUDA);
    EXPECT_TRUE(caps.device_type == DeviceType::DiscreteGPU ||
                caps.device_type == DeviceType::IntegratedGPU);

    // CUDA devices should have significant resources
    EXPECT_GT(caps.global_memory, 0u);
    EXPECT_GT(caps.compute_units, 0u);
    EXPECT_GT(caps.max_workgroup_size, 0u);
    EXPECT_EQ(caps.warp_size, 32u);  // NVIDIA warp size is always 32

    // Print device info for debugging
    std::cout << "Device: " << caps.name << std::endl;
    std::cout << "  Compute capability: " << caps.compute_capability << std::endl;
    std::cout << "  Global memory: " << (caps.global_memory / (1024*1024)) << " MB" << std::endl;
    std::cout << "  Compute units (SMs): " << caps.compute_units << std::endl;
    std::cout << "  Max threads/block: " << caps.max_workgroup_size << std::endl;
    std::cout << "  Supports double: " << (caps.supports_double ? "yes" : "no") << std::endl;
    std::cout << "  Supports unified memory: " << (caps.supports_unified_memory ? "yes" : "no") << std::endl;
}

TEST_F(CUDABackendTest, SetDevice) {
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

TEST_F(CUDABackendTest, AllocateDeviceMemory) {
    constexpr SizeT size = 1024 * 1024;  // 1 MB
    auto buffer = backend->allocate(size, MemoryType::DeviceLocal);

    EXPECT_TRUE(buffer.is_valid());
    EXPECT_EQ(buffer.size, size);
    EXPECT_EQ(buffer.type, MemoryType::DeviceLocal);

    EXPECT_GE(backend->allocated_memory(), size);

    backend->free(buffer);
    EXPECT_EQ(backend->allocated_memory(), 0u);
}

TEST_F(CUDABackendTest, AllocateSharedMemory) {
    auto caps = backend->get_device_capabilities(0);
    if (!caps.supports_unified_memory) {
        GTEST_SKIP() << "Device does not support unified memory";
    }

    constexpr SizeT size = 1024;
    auto buffer = backend->allocate(size, MemoryType::Shared);

    EXPECT_TRUE(buffer.is_valid());
    EXPECT_EQ(buffer.type, MemoryType::Shared);

    backend->free(buffer);
}

TEST_F(CUDABackendTest, AllocateZeroSize) {
    auto buffer = backend->allocate(0);
    EXPECT_FALSE(buffer.is_valid());
}

TEST_F(CUDABackendTest, UploadDownload) {
    constexpr SizeT count = 1024;
    constexpr SizeT size = count * sizeof(float);

    // Create source data
    std::vector<float> src(count);
    for (SizeT i = 0; i < count; ++i) {
        src[i] = static_cast<float>(i) * 0.5f;
    }

    // Allocate buffer
    auto buffer = backend->allocate(size);
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

TEST_F(CUDABackendTest, UploadDownloadWithOffset) {
    constexpr SizeT size = 4096;
    auto buffer = backend->allocate(size);
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

TEST_F(CUDABackendTest, CopyBuffer) {
    constexpr SizeT count = 512;
    constexpr SizeT size = count * sizeof(Int32);

    std::vector<Int32> data(count);
    std::iota(data.begin(), data.end(), 100);

    auto src = backend->allocate(size);
    auto dst = backend->allocate(size);
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

TEST_F(CUDABackendTest, FillBuffer) {
    constexpr SizeT size = 4096;
    auto buffer = backend->allocate(size);
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

TEST_F(CUDABackendTest, MapUnmap) {
    auto caps = backend->get_device_capabilities(0);
    if (!caps.supports_unified_memory) {
        GTEST_SKIP() << "Device does not support unified memory for mapping test";
    }

    constexpr SizeT count = 64;
    constexpr SizeT size = count * sizeof(double);

    auto buffer = backend->allocate(size, MemoryType::Shared);
    ASSERT_TRUE(buffer.is_valid());

    // Map buffer
    auto* ptr = static_cast<double*>(backend->map(buffer, MemoryAccess::ReadWrite));
    ASSERT_NE(ptr, nullptr);

    // Write data
    for (SizeT i = 0; i < count; ++i) {
        ptr[i] = static_cast<double>(i) * 2.5;
    }

    // Unmap
    backend->unmap(buffer);

    // Synchronize
    backend->synchronize();

    // Download and verify
    std::vector<double> result(count);
    EXPECT_EQ(backend->download(buffer, result.data(), size), BackendResult::Success);

    for (SizeT i = 0; i < count; ++i) {
        EXPECT_DOUBLE_EQ(result[i], static_cast<double>(i) * 2.5);
    }

    backend->free(buffer);
}

TEST_F(CUDABackendTest, AvailableMemory) {
    SizeT available = backend->available_memory();
    EXPECT_GT(available, 0u);

    std::cout << "Available memory: " << (available / (1024*1024)) << " MB" << std::endl;
}

// ============================================================================
// Stream Tests
// ============================================================================

TEST_F(CUDABackendTest, CreateStream) {
    auto stream = backend->create_stream();
    EXPECT_TRUE(stream.is_valid());
    backend->destroy_stream(stream);
}

TEST_F(CUDABackendTest, MultipleStreams) {
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

TEST_F(CUDABackendTest, StreamSynchronize) {
    auto stream = backend->create_stream();
    ASSERT_TRUE(stream.is_valid());

    EXPECT_EQ(backend->synchronize_stream(stream), BackendResult::Success);

    backend->destroy_stream(stream);
}

TEST_F(CUDABackendTest, AsyncUploadDownload) {
    auto stream = backend->create_stream();
    ASSERT_TRUE(stream.is_valid());

    constexpr SizeT count = 4096;
    constexpr SizeT size = count * sizeof(float);

    std::vector<float> src(count);
    for (SizeT i = 0; i < count; ++i) {
        src[i] = std::sin(static_cast<float>(i) * 0.01f);
    }

    auto buffer = backend->allocate(size);
    ASSERT_TRUE(buffer.is_valid());

    // Async upload
    EXPECT_EQ(backend->upload(buffer, src.data(), size, 0, stream),
              BackendResult::Success);

    // Synchronize stream
    EXPECT_EQ(backend->synchronize_stream(stream), BackendResult::Success);

    // Async download
    std::vector<float> dst(count, 0.0f);
    EXPECT_EQ(backend->download(buffer, dst.data(), size, 0, stream),
              BackendResult::Success);

    EXPECT_EQ(backend->synchronize_stream(stream), BackendResult::Success);

    // Verify
    for (SizeT i = 0; i < count; ++i) {
        EXPECT_FLOAT_EQ(src[i], dst[i]);
    }

    backend->free(buffer);
    backend->destroy_stream(stream);
}

// ============================================================================
// Event Tests
// ============================================================================

TEST_F(CUDABackendTest, RecordEvent) {
    auto event = backend->record_event();
    EXPECT_TRUE(event.is_valid());
    backend->destroy_event(event);
}

TEST_F(CUDABackendTest, WaitEvent) {
    auto event = backend->record_event();
    EXPECT_TRUE(event.is_valid());

    EXPECT_EQ(backend->wait_event(event), BackendResult::Success);

    backend->destroy_event(event);
}

TEST_F(CUDABackendTest, ElapsedTime) {
    auto start = backend->record_event();

    // Do some work
    constexpr SizeT size = 64 * 1024 * 1024;  // 64 MB
    auto buffer = backend->allocate(size);
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

    std::cout << "Elapsed time for 64MB transfer: " << elapsed << " ms" << std::endl;

    backend->destroy_event(start);
    backend->destroy_event(end);
}

TEST_F(CUDABackendTest, StreamWaitEvent) {
    auto stream1 = backend->create_stream();
    auto stream2 = backend->create_stream();
    ASSERT_TRUE(stream1.is_valid());
    ASSERT_TRUE(stream2.is_valid());

    // Record event on stream1
    auto event = backend->record_event(stream1);
    EXPECT_TRUE(event.is_valid());

    // Make stream2 wait for the event
    EXPECT_EQ(backend->stream_wait_event(stream2, event), BackendResult::Success);

    // Synchronize both streams
    EXPECT_EQ(backend->synchronize(), BackendResult::Success);

    backend->destroy_event(event);
    backend->destroy_stream(stream1);
    backend->destroy_stream(stream2);
}

// ============================================================================
// Kernel Compilation and Execution Tests
// ============================================================================

TEST_F(CUDABackendTest, CompileKernel) {
    const char* source = R"(
extern "C" __global__ void vector_add(float* a, float* b, float* c, int n) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx < n) {
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
    std::cout << "  Max threads/block: " << kernel->max_workgroup_size() << std::endl;
    std::cout << "  Preferred block size: " << kernel->preferred_workgroup_size() << std::endl;
    std::cout << "  Local memory: " << kernel->local_memory_size() << " bytes" << std::endl;
    std::cout << "  Registers: " << kernel->register_count() << std::endl;
}

TEST_F(CUDABackendTest, ExecuteKernel) {
    const char* source = R"(
extern "C" __global__ void vector_add(float* a, float* b, float* c, int n) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx < n) {
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
    auto d_a = backend->allocate(size);
    auto d_b = backend->allocate(size);
    auto d_c = backend->allocate(size);
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

TEST_F(CUDABackendTest, ExecuteKernelWithSharedMemory) {
    const char* source = R"(
extern "C" __global__ void reduce_sum(float* input, float* output, int n) {
    extern __shared__ float shared[];

    int tid = threadIdx.x;
    int idx = blockIdx.x * blockDim.x + threadIdx.x;

    // Load into shared memory
    shared[tid] = (idx < n) ? input[idx] : 0.0f;
    __syncthreads();

    // Reduction in shared memory
    for (int s = blockDim.x / 2; s > 0; s >>= 1) {
        if (tid < s) {
            shared[tid] += shared[tid + s];
        }
        __syncthreads();
    }

    // Write result
    if (tid == 0) {
        output[blockIdx.x] = shared[0];
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
    auto d_input = backend->allocate(input_size);
    auto d_output = backend->allocate(output_size);
    ASSERT_TRUE(d_input.is_valid());
    ASSERT_TRUE(d_output.is_valid());

    // Upload
    EXPECT_EQ(backend->upload(d_input, h_input.data(), input_size), BackendResult::Success);

    // Set arguments
    kernel->set_arg(0, KernelArg::Buffer(d_input));
    kernel->set_arg(1, KernelArg::Buffer(d_output));
    kernel->set_arg(2, KernelArg::Int(N));

    // Launch with shared memory
    LaunchConfig config;
    config.grid_x = NUM_BLOCKS;
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

// ============================================================================
// Error Handling Tests
// ============================================================================

TEST_F(CUDABackendTest, InvalidBufferUpload) {
    BufferHandle invalid;
    std::vector<UInt8> data(100);

    EXPECT_EQ(backend->upload(invalid, data.data(), data.size()),
              BackendResult::InvalidArgument);
    EXPECT_TRUE(backend->has_error());
}

TEST_F(CUDABackendTest, InvalidBufferDownload) {
    BufferHandle invalid;
    std::vector<UInt8> data(100);

    EXPECT_EQ(backend->download(invalid, data.data(), data.size()),
              BackendResult::InvalidArgument);
}

TEST_F(CUDABackendTest, UploadExceedsSize) {
    auto buffer = backend->allocate(100);
    ASSERT_TRUE(buffer.is_valid());

    std::vector<UInt8> data(200);
    EXPECT_EQ(backend->upload(buffer, data.data(), data.size()),
              BackendResult::InvalidArgument);

    backend->free(buffer);
}

TEST_F(CUDABackendTest, ClearError) {
    // Cause an error
    BufferHandle invalid;
    backend->upload(invalid, nullptr, 0);

    EXPECT_TRUE(backend->has_error());
    EXPECT_FALSE(backend->last_error().empty());

    backend->clear_error();

    EXPECT_FALSE(backend->has_error());
    EXPECT_TRUE(backend->last_error().empty());
}

TEST_F(CUDABackendTest, KernelCompilationError) {
    const char* bad_source = R"(
extern "C" __global__ void bad_kernel() {
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

TEST_F(CUDABackendTest, BandwidthTest) {
    // Test host-to-device and device-to-host bandwidth
    constexpr SizeT sizes[] = {
        1 * 1024 * 1024,   // 1 MB
        16 * 1024 * 1024,  // 16 MB
        64 * 1024 * 1024,  // 64 MB
    };

    for (SizeT size : sizes) {
        std::vector<UInt8> h_data(size, 0x42);
        auto buffer = backend->allocate(size);

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

// ============================================================================
// Main
// ============================================================================

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
