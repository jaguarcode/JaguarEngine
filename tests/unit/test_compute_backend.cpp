/**
 * @file test_compute_backend.cpp
 * @brief Unit tests for GPU compute backend abstraction
 */

#include <gtest/gtest.h>
#include "jaguar/gpu/compute_backend.h"
#include <numeric>
#include <vector>

using namespace jaguar;
using namespace jaguar::gpu;

// ============================================================================
// Backend Factory Tests
// ============================================================================

class BackendFactoryTest : public ::testing::Test {
protected:
    void SetUp() override {
        // CPU backend should always be available due to auto-registration
    }
};

TEST_F(BackendFactoryTest, CPUBackendIsAvailable) {
    EXPECT_TRUE(BackendFactory::is_available(BackendType::CPU));
}

TEST_F(BackendFactoryTest, CreateCPUBackend) {
    auto backend = BackendFactory::create(BackendType::CPU);
    ASSERT_NE(backend, nullptr);
    EXPECT_EQ(backend->type(), BackendType::CPU);
}

TEST_F(BackendFactoryTest, CreateAutoBackend) {
    auto backend = BackendFactory::create(BackendType::Auto);
    ASSERT_NE(backend, nullptr);
    // Should be CPU since that's the only registered backend
    EXPECT_EQ(backend->type(), BackendType::CPU);
}

TEST_F(BackendFactoryTest, CreateBestAvailable) {
    auto backend = BackendFactory::create_best_available();
    ASSERT_NE(backend, nullptr);
}

TEST_F(BackendFactoryTest, AvailableBackendsList) {
    auto backends = BackendFactory::available_backends();
    EXPECT_FALSE(backends.empty());
    EXPECT_NE(std::find(backends.begin(), backends.end(), BackendType::CPU), backends.end());
}

TEST_F(BackendFactoryTest, TypeNameConversion) {
    EXPECT_STREQ(BackendFactory::type_name(BackendType::CPU), "CPU");
    EXPECT_STREQ(BackendFactory::type_name(BackendType::CUDA), "CUDA");
    EXPECT_STREQ(BackendFactory::type_name(BackendType::OpenCL), "OpenCL");
    EXPECT_STREQ(BackendFactory::type_name(BackendType::Metal), "Metal");
    EXPECT_STREQ(BackendFactory::type_name(BackendType::Auto), "Auto");
}

// ============================================================================
// CPU Backend Lifecycle Tests
// ============================================================================

class CPUBackendTest : public ::testing::Test {
protected:
    void SetUp() override {
        backend = BackendFactory::create(BackendType::CPU);
        ASSERT_NE(backend, nullptr);
    }

    void TearDown() override {
        if (backend && backend->is_initialized()) {
            backend->shutdown();
        }
    }

    std::unique_ptr<IComputeBackend> backend;
};

TEST_F(CPUBackendTest, Initialization) {
    EXPECT_FALSE(backend->is_initialized());

    auto result = backend->initialize();
    EXPECT_EQ(result, BackendResult::Success);
    EXPECT_TRUE(backend->is_initialized());
}

TEST_F(CPUBackendTest, DoubleInitialization) {
    EXPECT_EQ(backend->initialize(), BackendResult::Success);
    EXPECT_EQ(backend->initialize(), BackendResult::Success); // Should be idempotent
    EXPECT_TRUE(backend->is_initialized());
}

TEST_F(CPUBackendTest, Shutdown) {
    EXPECT_EQ(backend->initialize(), BackendResult::Success);
    backend->shutdown();
    EXPECT_FALSE(backend->is_initialized());
}

TEST_F(CPUBackendTest, DeviceCount) {
    EXPECT_EQ(backend->initialize(), BackendResult::Success);
    EXPECT_EQ(backend->device_count(), 1u);
}

TEST_F(CPUBackendTest, DeviceCapabilities) {
    EXPECT_EQ(backend->initialize(), BackendResult::Success);

    auto caps = backend->get_device_capabilities(0);
    EXPECT_EQ(caps.device_type, DeviceType::CPU);
    EXPECT_EQ(caps.backend_type, BackendType::CPU);
    EXPECT_GT(caps.global_memory, 0u);
    EXPECT_GT(caps.compute_units, 0u);
    EXPECT_TRUE(caps.supports_double);
    EXPECT_TRUE(caps.supports_atomics);
    EXPECT_TRUE(caps.supports_unified_memory);
}

TEST_F(CPUBackendTest, SetDevice) {
    EXPECT_EQ(backend->initialize(), BackendResult::Success);
    EXPECT_EQ(backend->current_device(), 0u);
    EXPECT_EQ(backend->set_device(0), BackendResult::Success);
    EXPECT_EQ(backend->set_device(1), BackendResult::DeviceNotFound);
}

// ============================================================================
// Memory Management Tests
// ============================================================================

class MemoryManagementTest : public ::testing::Test {
protected:
    void SetUp() override {
        backend = BackendFactory::create(BackendType::CPU);
        ASSERT_NE(backend, nullptr);
        ASSERT_EQ(backend->initialize(), BackendResult::Success);
    }

    void TearDown() override {
        if (backend) {
            backend->shutdown();
        }
    }

    std::unique_ptr<IComputeBackend> backend;
};

TEST_F(MemoryManagementTest, AllocateBuffer) {
    constexpr SizeT size = 1024;
    auto buffer = backend->allocate(size);

    EXPECT_TRUE(buffer.is_valid());
    EXPECT_EQ(buffer.size, size);
    EXPECT_EQ(buffer.type, MemoryType::DeviceLocal);
    EXPECT_EQ(buffer.access, MemoryAccess::ReadWrite);

    backend->free(buffer);
}

TEST_F(MemoryManagementTest, AllocateWithType) {
    constexpr SizeT size = 512;
    auto buffer = backend->allocate(size, MemoryType::Shared, MemoryAccess::ReadOnly);

    EXPECT_TRUE(buffer.is_valid());
    EXPECT_EQ(buffer.type, MemoryType::Shared);
    EXPECT_EQ(buffer.access, MemoryAccess::ReadOnly);

    backend->free(buffer);
}

TEST_F(MemoryManagementTest, AllocateZeroSize) {
    auto buffer = backend->allocate(0);
    EXPECT_FALSE(buffer.is_valid());
}

TEST_F(MemoryManagementTest, AllocatedMemoryTracking) {
    SizeT initial = backend->allocated_memory();
    constexpr SizeT size = 2048;

    auto buffer = backend->allocate(size);
    EXPECT_EQ(backend->allocated_memory(), initial + size);

    backend->free(buffer);
    EXPECT_EQ(backend->allocated_memory(), initial);
}

TEST_F(MemoryManagementTest, UploadDownload) {
    constexpr SizeT count = 256;
    constexpr SizeT size = count * sizeof(float);

    std::vector<float> src(count);
    std::iota(src.begin(), src.end(), 0.0f);

    auto buffer = backend->allocate(size);
    ASSERT_TRUE(buffer.is_valid());

    EXPECT_EQ(backend->upload(buffer, src.data(), size), BackendResult::Success);

    std::vector<float> dst(count, -1.0f);
    EXPECT_EQ(backend->download(buffer, dst.data(), size), BackendResult::Success);

    EXPECT_EQ(src, dst);

    backend->free(buffer);
}

TEST_F(MemoryManagementTest, UploadWithOffset) {
    constexpr SizeT size = 1024;
    auto buffer = backend->allocate(size);
    ASSERT_TRUE(buffer.is_valid());

    std::vector<UInt8> data(256, 0xAB);
    EXPECT_EQ(backend->upload(buffer, data.data(), data.size(), 128), BackendResult::Success);

    std::vector<UInt8> result(256, 0);
    EXPECT_EQ(backend->download(buffer, result.data(), result.size(), 128), BackendResult::Success);

    EXPECT_EQ(data, result);

    backend->free(buffer);
}

TEST_F(MemoryManagementTest, CopyBuffer) {
    constexpr SizeT count = 128;
    constexpr SizeT size = count * sizeof(Int32);

    std::vector<Int32> data(count);
    std::iota(data.begin(), data.end(), 100);

    auto src = backend->allocate(size);
    auto dst = backend->allocate(size);
    ASSERT_TRUE(src.is_valid());
    ASSERT_TRUE(dst.is_valid());

    EXPECT_EQ(backend->upload(src, data.data(), size), BackendResult::Success);
    EXPECT_EQ(backend->copy(src, dst, size), BackendResult::Success);

    std::vector<Int32> result(count, 0);
    EXPECT_EQ(backend->download(dst, result.data(), size), BackendResult::Success);

    EXPECT_EQ(data, result);

    backend->free(src);
    backend->free(dst);
}

TEST_F(MemoryManagementTest, FillBuffer) {
    constexpr SizeT size = 512;
    auto buffer = backend->allocate(size);
    ASSERT_TRUE(buffer.is_valid());

    Int32 pattern = 0x12345678;
    EXPECT_EQ(backend->fill(buffer, &pattern, sizeof(pattern)), BackendResult::Success);

    std::vector<Int32> result(size / sizeof(Int32));
    EXPECT_EQ(backend->download(buffer, result.data(), size), BackendResult::Success);

    for (const auto& val : result) {
        EXPECT_EQ(val, pattern);
    }

    backend->free(buffer);
}

TEST_F(MemoryManagementTest, MapUnmap) {
    constexpr SizeT count = 64;
    constexpr SizeT size = count * sizeof(double);

    auto buffer = backend->allocate(size);
    ASSERT_TRUE(buffer.is_valid());

    auto* ptr = static_cast<double*>(backend->map(buffer, MemoryAccess::ReadWrite));
    ASSERT_NE(ptr, nullptr);

    for (SizeT i = 0; i < count; ++i) {
        ptr[i] = static_cast<double>(i) * 1.5;
    }

    backend->unmap(buffer);

    std::vector<double> result(count);
    EXPECT_EQ(backend->download(buffer, result.data(), size), BackendResult::Success);

    for (SizeT i = 0; i < count; ++i) {
        EXPECT_DOUBLE_EQ(result[i], static_cast<double>(i) * 1.5);
    }

    backend->free(buffer);
}

// ============================================================================
// Stream and Event Tests
// ============================================================================

class StreamEventTest : public ::testing::Test {
protected:
    void SetUp() override {
        backend = BackendFactory::create(BackendType::CPU);
        ASSERT_NE(backend, nullptr);
        ASSERT_EQ(backend->initialize(), BackendResult::Success);
    }

    void TearDown() override {
        if (backend) {
            backend->shutdown();
        }
    }

    std::unique_ptr<IComputeBackend> backend;
};

TEST_F(StreamEventTest, CreateStream) {
    auto stream = backend->create_stream();
    EXPECT_TRUE(stream.is_valid());
    backend->destroy_stream(stream);
}

TEST_F(StreamEventTest, SynchronizeStream) {
    auto stream = backend->create_stream();
    EXPECT_EQ(backend->synchronize_stream(stream), BackendResult::Success);
    backend->destroy_stream(stream);
}

TEST_F(StreamEventTest, Synchronize) {
    EXPECT_EQ(backend->synchronize(), BackendResult::Success);
}

TEST_F(StreamEventTest, RecordEvent) {
    auto event = backend->record_event();
    EXPECT_TRUE(event.is_valid());
    backend->destroy_event(event);
}

TEST_F(StreamEventTest, WaitEvent) {
    auto event = backend->record_event();
    EXPECT_EQ(backend->wait_event(event), BackendResult::Success);
    backend->destroy_event(event);
}

TEST_F(StreamEventTest, ElapsedTime) {
    auto start = backend->record_event();

    // Small delay
    volatile int sum = 0;
    for (int i = 0; i < 100000; ++i) {
        sum += i;
    }
    (void)sum;

    auto end = backend->record_event();

    Real elapsed = backend->elapsed_time(start, end);
    EXPECT_GE(elapsed, 0.0);

    backend->destroy_event(start);
    backend->destroy_event(end);
}

// ============================================================================
// Utility Function Tests
// ============================================================================

TEST(UtilityFunctionsTest, ResultToString) {
    EXPECT_STREQ(result_to_string(BackendResult::Success), "Success");
    EXPECT_STREQ(result_to_string(BackendResult::OutOfMemory), "Out of memory");
    EXPECT_STREQ(result_to_string(BackendResult::CompilationFailed), "Compilation failed");
    EXPECT_STREQ(result_to_string(BackendResult::NotInitialized), "Not initialized");
}

TEST(UtilityFunctionsTest, MemoryTypeToString) {
    EXPECT_STREQ(memory_type_to_string(MemoryType::DeviceLocal), "DeviceLocal");
    EXPECT_STREQ(memory_type_to_string(MemoryType::Shared), "Shared");
    EXPECT_STREQ(memory_type_to_string(MemoryType::Staging), "Staging");
}

TEST(UtilityFunctionsTest, DeviceTypeToString) {
    EXPECT_STREQ(device_type_to_string(DeviceType::CPU), "CPU");
    EXPECT_STREQ(device_type_to_string(DeviceType::DiscreteGPU), "Discrete GPU");
    EXPECT_STREQ(device_type_to_string(DeviceType::IntegratedGPU), "Integrated GPU");
}

// ============================================================================
// Launch Configuration Tests
// ============================================================================

TEST(LaunchConfigTest, LinearConfig) {
    auto config = LaunchConfig::Linear(1000, 256);
    EXPECT_EQ(config.grid_x, 4u);  // ceil(1000/256) = 4
    EXPECT_EQ(config.block_x, 256u);
    EXPECT_EQ(config.grid_y, 1u);
    EXPECT_EQ(config.grid_z, 1u);
}

TEST(LaunchConfigTest, Grid2DConfig) {
    auto config = LaunchConfig::Grid2D(100, 200, 16, 16);
    EXPECT_EQ(config.grid_x, 7u);   // ceil(100/16)
    EXPECT_EQ(config.grid_y, 13u);  // ceil(200/16)
    EXPECT_EQ(config.block_x, 16u);
    EXPECT_EQ(config.block_y, 16u);
}

TEST(LaunchConfigTest, Grid3DConfig) {
    auto config = LaunchConfig::Grid3D(32, 64, 128, 8, 8, 8);
    EXPECT_EQ(config.grid_x, 4u);
    EXPECT_EQ(config.grid_y, 8u);
    EXPECT_EQ(config.grid_z, 16u);
}

TEST(LaunchConfigTest, TotalThreads) {
    LaunchConfig config;
    config.grid_x = 10;
    config.grid_y = 10;
    config.grid_z = 1;
    config.block_x = 16;
    config.block_y = 16;
    config.block_z = 1;

    EXPECT_EQ(config.total_threads(), 10u * 10 * 16 * 16);
}

// ============================================================================
// Kernel Argument Tests
// ============================================================================

TEST(KernelArgTest, BufferArg) {
    BufferHandle handle{42, 1024, MemoryType::DeviceLocal, MemoryAccess::ReadWrite};
    auto arg = KernelArg::Buffer(handle);

    EXPECT_EQ(arg.type, ArgType::Buffer);
    EXPECT_EQ(std::get<BufferHandle>(arg.value).id, 42u);
}

TEST(KernelArgTest, ScalarArgs) {
    auto i32_arg = KernelArg::Int(123);
    EXPECT_EQ(i32_arg.type, ArgType::Scalar_I32);
    EXPECT_EQ(std::get<Int32>(i32_arg.value), 123);

    auto f32_arg = KernelArg::Float(3.14f);
    EXPECT_EQ(f32_arg.type, ArgType::Scalar_F32);
    EXPECT_FLOAT_EQ(std::get<float>(f32_arg.value), 3.14f);

    auto f64_arg = KernelArg::Double(2.718281828);
    EXPECT_EQ(f64_arg.type, ArgType::Scalar_F64);
    EXPECT_DOUBLE_EQ(std::get<double>(f64_arg.value), 2.718281828);
}

TEST(KernelArgTest, LocalMemoryArg) {
    auto arg = KernelArg::LocalMemory(4096);
    EXPECT_EQ(arg.type, ArgType::LocalMem);
    EXPECT_EQ(std::get<SizeT>(arg.value), 4096u);
}

// ============================================================================
// Handle Tests
// ============================================================================

TEST(HandleTest, BufferHandleValidity) {
    BufferHandle invalid;
    EXPECT_FALSE(invalid.is_valid());
    EXPECT_FALSE(static_cast<bool>(invalid));

    BufferHandle valid{1, 100, MemoryType::DeviceLocal, MemoryAccess::ReadWrite};
    EXPECT_TRUE(valid.is_valid());
    EXPECT_TRUE(static_cast<bool>(valid));
}

TEST(HandleTest, StreamHandleValidity) {
    StreamHandle invalid;
    EXPECT_FALSE(invalid.is_valid());

    auto default_stream = StreamHandle::Default();
    EXPECT_TRUE(default_stream.is_valid());
    EXPECT_EQ(default_stream.id, 1u);
}

TEST(HandleTest, EventHandleValidity) {
    EventHandle invalid;
    EXPECT_FALSE(invalid.is_valid());

    EventHandle valid{123};
    EXPECT_TRUE(valid.is_valid());
}

// ============================================================================
// Device Capabilities Tests
// ============================================================================

TEST(DeviceCapabilitiesTest, MeetsRequirements) {
    DeviceCapabilities caps;
    caps.global_memory = 4ULL * 1024 * 1024 * 1024;  // 4GB
    caps.compute_units = 8;

    EXPECT_TRUE(caps.meets_requirements(1ULL * 1024 * 1024 * 1024, 4));
    EXPECT_FALSE(caps.meets_requirements(8ULL * 1024 * 1024 * 1024, 4));
    EXPECT_FALSE(caps.meets_requirements(1ULL * 1024 * 1024 * 1024, 16));
}

// ============================================================================
// Error Handling Tests
// ============================================================================

class ErrorHandlingTest : public ::testing::Test {
protected:
    void SetUp() override {
        backend = BackendFactory::create(BackendType::CPU);
        ASSERT_NE(backend, nullptr);
    }

    void TearDown() override {
        if (backend && backend->is_initialized()) {
            backend->shutdown();
        }
    }

    std::unique_ptr<IComputeBackend> backend;
};

TEST_F(ErrorHandlingTest, OperationsBeforeInit) {
    // Most operations should handle not-initialized state
    auto buffer = backend->allocate(1024);
    EXPECT_FALSE(buffer.is_valid());
}

TEST_F(ErrorHandlingTest, InvalidBufferUpload) {
    ASSERT_EQ(backend->initialize(), BackendResult::Success);

    BufferHandle invalid;
    std::vector<UInt8> data(100);

    EXPECT_EQ(backend->upload(invalid, data.data(), data.size()),
              BackendResult::InvalidArgument);
}

TEST_F(ErrorHandlingTest, InvalidBufferDownload) {
    ASSERT_EQ(backend->initialize(), BackendResult::Success);

    BufferHandle invalid;
    std::vector<UInt8> data(100);

    EXPECT_EQ(backend->download(invalid, data.data(), data.size()),
              BackendResult::InvalidArgument);
}

TEST_F(ErrorHandlingTest, UploadExceedsSize) {
    ASSERT_EQ(backend->initialize(), BackendResult::Success);

    auto buffer = backend->allocate(100);
    ASSERT_TRUE(buffer.is_valid());

    std::vector<UInt8> data(200);
    EXPECT_EQ(backend->upload(buffer, data.data(), data.size()),
              BackendResult::InvalidArgument);

    backend->free(buffer);
}

TEST_F(ErrorHandlingTest, ClearError) {
    ASSERT_EQ(backend->initialize(), BackendResult::Success);

    // Cause an error
    BufferHandle invalid;
    std::vector<UInt8> data(100);
    backend->upload(invalid, data.data(), data.size());

    EXPECT_TRUE(backend->has_error());
    EXPECT_FALSE(backend->last_error().empty());

    backend->clear_error();

    EXPECT_FALSE(backend->has_error());
    EXPECT_TRUE(backend->last_error().empty());
}
