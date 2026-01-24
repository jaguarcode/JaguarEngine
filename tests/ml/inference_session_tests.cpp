/**
 * @file inference_session_tests.cpp
 * @brief Comprehensive unit tests for ML model inference session
 *
 * These tests verify the inference session implementation, including:
 * - TensorShape operations and computations
 * - TensorInfo construction and size calculations
 * - Tensor data access and typed views
 * - DataType utilities and conversions
 * - InferenceConfig factory methods
 * - InferenceSession lifecycle and model loading
 * - ExecutionProvider availability checks
 * - Inference execution and error handling
 * - Statistics tracking and performance metrics
 * - Helper functions and validators
 */

#include <gtest/gtest.h>
#include "jaguar/ml/inference_session.h"
#include <numeric>
#include <vector>
#include <cmath>
#include <thread>
#include <chrono>

using namespace jaguar;
using namespace jaguar::ml;

// ============================================================================
// InferenceResult Tests
// ============================================================================

TEST(InferenceResultTest, EnumValues) {
    EXPECT_EQ(static_cast<UInt8>(InferenceResult::Success), 0);
    EXPECT_NE(InferenceResult::InvalidConfiguration, InferenceResult::Success);
    EXPECT_NE(InferenceResult::InvalidModel, InferenceResult::Success);
    EXPECT_NE(InferenceResult::InvalidInput, InferenceResult::Success);
    EXPECT_NE(InferenceResult::RuntimeError, InferenceResult::Success);
    EXPECT_NE(InferenceResult::NotInitialized, InferenceResult::Success);
}

TEST(InferenceResultTest, ToString) {
    EXPECT_STREQ(inference_result_to_string(InferenceResult::Success), "Success");
    EXPECT_STREQ(inference_result_to_string(InferenceResult::InvalidConfiguration), "InvalidConfiguration");
    EXPECT_STREQ(inference_result_to_string(InferenceResult::InvalidModel), "InvalidModel");
    EXPECT_STREQ(inference_result_to_string(InferenceResult::InvalidInput), "InvalidInput");
    EXPECT_STREQ(inference_result_to_string(InferenceResult::InvalidOutput), "InvalidOutput");
    EXPECT_STREQ(inference_result_to_string(InferenceResult::RuntimeError), "RuntimeError");
    EXPECT_STREQ(inference_result_to_string(InferenceResult::ExecutionFailed), "ExecutionFailed");
    EXPECT_STREQ(inference_result_to_string(InferenceResult::TimeoutError), "TimeoutError");
    EXPECT_STREQ(inference_result_to_string(InferenceResult::NotInitialized), "NotInitialized");
    EXPECT_STREQ(inference_result_to_string(InferenceResult::AlreadyInitialized), "AlreadyInitialized");
    EXPECT_STREQ(inference_result_to_string(InferenceResult::ModelNotLoaded), "ModelNotLoaded");
    EXPECT_STREQ(inference_result_to_string(InferenceResult::OutOfMemory), "OutOfMemory");
    EXPECT_STREQ(inference_result_to_string(InferenceResult::DeviceUnavailable), "DeviceUnavailable");
    EXPECT_STREQ(inference_result_to_string(InferenceResult::ProviderNotSupported), "ProviderNotSupported");
}

// ============================================================================
// ExecutionProvider Tests
// ============================================================================

TEST(ExecutionProviderTest, EnumValues) {
    EXPECT_EQ(static_cast<UInt8>(ExecutionProvider::CPU), 0);
    EXPECT_NE(ExecutionProvider::CUDA, ExecutionProvider::CPU);
    EXPECT_NE(ExecutionProvider::TensorRT, ExecutionProvider::CPU);
}

TEST(ExecutionProviderTest, ToString) {
    EXPECT_STREQ(execution_provider_to_string(ExecutionProvider::CPU), "CPU");
    EXPECT_STREQ(execution_provider_to_string(ExecutionProvider::CUDA), "CUDA");
    EXPECT_STREQ(execution_provider_to_string(ExecutionProvider::TensorRT), "TensorRT");
    EXPECT_STREQ(execution_provider_to_string(ExecutionProvider::CoreML), "CoreML");
    EXPECT_STREQ(execution_provider_to_string(ExecutionProvider::DirectML), "DirectML");
    EXPECT_STREQ(execution_provider_to_string(ExecutionProvider::OpenVINO), "OpenVINO");
}

TEST(ExecutionProviderTest, CPUAlwaysAvailable) {
    EXPECT_TRUE(is_provider_available(ExecutionProvider::CPU));
}

TEST(ExecutionProviderTest, GetAvailableProviders) {
    auto providers = get_available_providers();
    EXPECT_FALSE(providers.empty());
    // CPU should always be in the list
    EXPECT_TRUE(std::find(providers.begin(), providers.end(), ExecutionProvider::CPU) != providers.end());
}

// ============================================================================
// DataType Tests
// ============================================================================

TEST(DataTypeTest, EnumValues) {
    EXPECT_EQ(static_cast<UInt8>(DataType::Float32), 0);
    EXPECT_NE(DataType::Float64, DataType::Float32);
    EXPECT_NE(DataType::Int32, DataType::Float32);
}

TEST(DataTypeTest, ToString) {
    EXPECT_STREQ(data_type_to_string(DataType::Float32), "Float32");
    EXPECT_STREQ(data_type_to_string(DataType::Float64), "Float64");
    EXPECT_STREQ(data_type_to_string(DataType::Int32), "Int32");
    EXPECT_STREQ(data_type_to_string(DataType::Int64), "Int64");
    EXPECT_STREQ(data_type_to_string(DataType::UInt8), "UInt8");
    EXPECT_STREQ(data_type_to_string(DataType::Bool), "Bool");
}

TEST(DataTypeTest, SizeInBytes) {
    EXPECT_EQ(data_type_size(DataType::Float32), 4);
    EXPECT_EQ(data_type_size(DataType::Float64), 8);
    EXPECT_EQ(data_type_size(DataType::Int32), 4);
    EXPECT_EQ(data_type_size(DataType::Int64), 8);
    EXPECT_EQ(data_type_size(DataType::UInt8), 1);
    EXPECT_EQ(data_type_size(DataType::Bool), 1);
}

// ============================================================================
// TensorShape Tests
// ============================================================================

TEST(TensorShapeTest, DefaultConstruction) {
    TensorShape shape;
    EXPECT_TRUE(shape.dimensions.empty());
    EXPECT_EQ(shape.rank(), 0);
    EXPECT_EQ(shape.element_count(), 0);
}

TEST(TensorShapeTest, InitializerListConstruction) {
    TensorShape shape{1, 3, 224, 224};
    EXPECT_EQ(shape.rank(), 4);
    EXPECT_EQ(shape.dimensions[0], 1);
    EXPECT_EQ(shape.dimensions[1], 3);
    EXPECT_EQ(shape.dimensions[2], 224);
    EXPECT_EQ(shape.dimensions[3], 224);
}

TEST(TensorShapeTest, VectorConstruction) {
    std::vector<Int64> dims = {2, 64, 128};
    TensorShape shape(dims);
    EXPECT_EQ(shape.rank(), 3);
    EXPECT_EQ(shape.dimensions[0], 2);
    EXPECT_EQ(shape.dimensions[1], 64);
    EXPECT_EQ(shape.dimensions[2], 128);
}

TEST(TensorShapeTest, ElementCountScalar) {
    TensorShape shape{1};
    EXPECT_EQ(shape.element_count(), 1);
}

TEST(TensorShapeTest, ElementCountVector) {
    TensorShape shape{10};
    EXPECT_EQ(shape.element_count(), 10);
}

TEST(TensorShapeTest, ElementCountMatrix) {
    TensorShape shape{3, 4};
    EXPECT_EQ(shape.element_count(), 12);
}

TEST(TensorShapeTest, ElementCountTensor) {
    TensorShape shape{1, 3, 224, 224};
    EXPECT_EQ(shape.element_count(), 150528);
}

TEST(TensorShapeTest, ElementCountBatch) {
    TensorShape shape{32, 128};
    EXPECT_EQ(shape.element_count(), 4096);
}

TEST(TensorShapeTest, ElementCountEmpty) {
    TensorShape shape;
    EXPECT_EQ(shape.element_count(), 0);
}

TEST(TensorShapeTest, DynamicShapeDetection) {
    TensorShape dynamic_shape{-1, 3, 224, 224};
    EXPECT_TRUE(dynamic_shape.is_dynamic());
    EXPECT_EQ(dynamic_shape.element_count(), 0);  // Dynamic shapes return 0
}

TEST(TensorShapeTest, DynamicShapeInMiddle) {
    TensorShape shape{1, -1, 224};
    EXPECT_TRUE(shape.is_dynamic());
    EXPECT_EQ(shape.element_count(), 0);
}

TEST(TensorShapeTest, StaticShapeNotDynamic) {
    TensorShape shape{1, 3, 224, 224};
    EXPECT_FALSE(shape.is_dynamic());
}

TEST(TensorShapeTest, RankDetermination) {
    EXPECT_EQ(TensorShape{}.rank(), 0);
    EXPECT_EQ(TensorShape{10}.rank(), 1);
    EXPECT_EQ((TensorShape{3, 4}.rank()), 2);
    EXPECT_EQ((TensorShape{2, 3, 4}.rank()), 3);
    EXPECT_EQ((TensorShape{1, 3, 224, 224}.rank()), 4);
}

TEST(TensorShapeTest, Equality) {
    TensorShape shape1{1, 3, 224, 224};
    TensorShape shape2{1, 3, 224, 224};
    TensorShape shape3{1, 3, 256, 256};

    EXPECT_EQ(shape1, shape2);
    EXPECT_NE(shape1, shape3);
}

TEST(TensorShapeTest, Inequality) {
    TensorShape shape1{1, 2, 3};
    TensorShape shape2{1, 2, 4};

    EXPECT_TRUE(shape1 != shape2);
    EXPECT_FALSE(shape1 != shape1);
}

// ============================================================================
// TensorInfo Tests
// ============================================================================

TEST(TensorInfoTest, DefaultConstruction) {
    TensorInfo info;
    EXPECT_TRUE(info.name.empty());
    EXPECT_EQ(info.data_type, DataType::Float32);
    EXPECT_TRUE(info.is_input);
}

TEST(TensorInfoTest, FullConstruction) {
    TensorInfo info("input", DataType::Float32, TensorShape{1, 3, 224, 224}, true);
    EXPECT_EQ(info.name, "input");
    EXPECT_EQ(info.data_type, DataType::Float32);
    EXPECT_EQ(info.shape.rank(), 4);
    EXPECT_TRUE(info.is_input);
}

TEST(TensorInfoTest, OutputTensor) {
    TensorInfo info("output", DataType::Int64, TensorShape{1, 1000}, false);
    EXPECT_EQ(info.name, "output");
    EXPECT_FALSE(info.is_input);
}

TEST(TensorInfoTest, SizeCalculationFloat32) {
    TensorInfo info("data", DataType::Float32, TensorShape{1, 3, 224, 224});
    EXPECT_EQ(info.size_in_bytes(), 150528 * 4);  // 602112 bytes
}

TEST(TensorInfoTest, SizeCalculationFloat64) {
    TensorInfo info("data", DataType::Float64, TensorShape{100, 100});
    EXPECT_EQ(info.size_in_bytes(), 10000 * 8);  // 80000 bytes
}

TEST(TensorInfoTest, SizeCalculationInt32) {
    TensorInfo info("data", DataType::Int32, TensorShape{1000});
    EXPECT_EQ(info.size_in_bytes(), 1000 * 4);  // 4000 bytes
}

TEST(TensorInfoTest, SizeCalculationInt64) {
    TensorInfo info("data", DataType::Int64, TensorShape{500});
    EXPECT_EQ(info.size_in_bytes(), 500 * 8);  // 4000 bytes
}

TEST(TensorInfoTest, SizeCalculationUInt8) {
    TensorInfo info("data", DataType::UInt8, TensorShape{1, 28, 28});
    EXPECT_EQ(info.size_in_bytes(), 784 * 1);  // 784 bytes
}

TEST(TensorInfoTest, SizeCalculationBool) {
    TensorInfo info("mask", DataType::Bool, TensorShape{64, 64});
    EXPECT_EQ(info.size_in_bytes(), 4096 * 1);  // 4096 bytes
}

TEST(TensorInfoTest, SizeCalculationDynamicShape) {
    TensorInfo info("dynamic", DataType::Float32, TensorShape{-1, 128});
    EXPECT_EQ(info.size_in_bytes(), 0);  // Dynamic shapes return 0
}

// ============================================================================
// Tensor Tests
// ============================================================================

TEST(TensorTest, DefaultConstruction) {
    Tensor tensor;
    EXPECT_TRUE(tensor.name.empty());
    EXPECT_EQ(tensor.data_type, DataType::Float32);
    EXPECT_TRUE(tensor.data.empty());
}

TEST(TensorTest, FullConstruction) {
    Tensor tensor("input", DataType::Float32, TensorShape{2, 3});
    EXPECT_EQ(tensor.name, "input");
    EXPECT_EQ(tensor.data_type, DataType::Float32);
    EXPECT_EQ(tensor.shape.rank(), 2);
    EXPECT_EQ(tensor.data.size(), 2 * 3 * 4);  // 6 floats = 24 bytes
}

TEST(TensorTest, ElementCount) {
    Tensor tensor("data", DataType::Float32, TensorShape{10, 20});
    EXPECT_EQ(tensor.element_count(), 200);
}

TEST(TensorTest, SizeInBytes) {
    Tensor tensor("data", DataType::Float64, TensorShape{5, 5});
    EXPECT_EQ(tensor.size_in_bytes(), 25 * 8);  // 200 bytes
}

TEST(TensorTest, DataAsFloat32) {
    Tensor tensor("data", DataType::Float32, TensorShape{3});
    float* data = tensor.data_as<float>();
    ASSERT_NE(data, nullptr);

    data[0] = 1.0f;
    data[1] = 2.0f;
    data[2] = 3.0f;

    const float* const_data = const_cast<const Tensor&>(tensor).data_as<float>();
    ASSERT_NE(const_data, nullptr);
    EXPECT_FLOAT_EQ(const_data[0], 1.0f);
    EXPECT_FLOAT_EQ(const_data[1], 2.0f);
    EXPECT_FLOAT_EQ(const_data[2], 3.0f);
}

TEST(TensorTest, DataAsFloat64) {
    Tensor tensor("data", DataType::Float64, TensorShape{2});
    double* data = tensor.data_as<double>();
    ASSERT_NE(data, nullptr);

    data[0] = 1.5;
    data[1] = 2.5;

    const double* const_data = const_cast<const Tensor&>(tensor).data_as<double>();
    EXPECT_DOUBLE_EQ(const_data[0], 1.5);
    EXPECT_DOUBLE_EQ(const_data[1], 2.5);
}

TEST(TensorTest, DataAsInt32) {
    Tensor tensor("data", DataType::Int32, TensorShape{4});
    Int32* data = tensor.data_as<Int32>();
    ASSERT_NE(data, nullptr);

    data[0] = 100;
    data[1] = -200;
    data[2] = 300;
    data[3] = -400;

    const Int32* const_data = const_cast<const Tensor&>(tensor).data_as<Int32>();
    EXPECT_EQ(const_data[0], 100);
    EXPECT_EQ(const_data[2], 300);
}

TEST(TensorTest, DataAsInt64) {
    Tensor tensor("data", DataType::Int64, TensorShape{2});
    Int64* data = tensor.data_as<Int64>();
    ASSERT_NE(data, nullptr);

    data[0] = 1000000000LL;
    data[1] = -1000000000LL;

    EXPECT_EQ(data[0], 1000000000LL);
    EXPECT_EQ(data[1], -1000000000LL);
}

TEST(TensorTest, DataAsUInt8) {
    Tensor tensor("data", DataType::UInt8, TensorShape{3});
    UInt8* data = tensor.data_as<UInt8>();
    ASSERT_NE(data, nullptr);

    data[0] = 255;
    data[1] = 128;
    data[2] = 0;

    EXPECT_EQ(data[0], 255);
    EXPECT_EQ(data[1], 128);
    EXPECT_EQ(data[2], 0);
}

TEST(TensorTest, DataAsBool) {
    Tensor tensor("data", DataType::Bool, TensorShape{2});
    bool* data = tensor.data_as<bool>();
    ASSERT_NE(data, nullptr);

    data[0] = true;
    data[1] = false;

    EXPECT_TRUE(data[0]);
    EXPECT_FALSE(data[1]);
}

TEST(TensorTest, DataAsTypeMismatch) {
    Tensor tensor("data", DataType::Float32, TensorShape{2});

    // Trying to access Float32 tensor as Int32 should return nullptr
    Int32* wrong_data = tensor.data_as<Int32>();
    EXPECT_EQ(wrong_data, nullptr);
}

TEST(TensorTest, SetDataFloat32) {
    Tensor tensor("data", DataType::Float32, TensorShape{4});
    std::vector<float> values = {1.0f, 2.0f, 3.0f, 4.0f};

    EXPECT_TRUE(tensor.set_data(values));

    const float* data = tensor.data_as<float>();
    ASSERT_NE(data, nullptr);
    EXPECT_FLOAT_EQ(data[0], 1.0f);
    EXPECT_FLOAT_EQ(data[1], 2.0f);
    EXPECT_FLOAT_EQ(data[2], 3.0f);
    EXPECT_FLOAT_EQ(data[3], 4.0f);
}

TEST(TensorTest, SetDataTypeMismatch) {
    Tensor tensor("data", DataType::Float32, TensorShape{4});
    std::vector<Int32> values = {1, 2, 3, 4};

    // Type mismatch should return false
    EXPECT_FALSE(tensor.set_data(values));
}

TEST(TensorTest, SetDataSizeMismatch) {
    Tensor tensor("data", DataType::Float32, TensorShape{4});
    std::vector<float> values = {1.0f, 2.0f};  // Only 2 elements, need 4

    // Size mismatch should return false
    EXPECT_FALSE(tensor.set_data(values));
}

TEST(TensorTest, SetDataLargeArray) {
    Tensor tensor("data", DataType::Int32, TensorShape{1000});
    std::vector<Int32> values(1000);
    std::iota(values.begin(), values.end(), 0);  // Fill with 0, 1, 2, ..., 999

    EXPECT_TRUE(tensor.set_data(values));

    const Int32* data = tensor.data_as<Int32>();
    ASSERT_NE(data, nullptr);
    EXPECT_EQ(data[0], 0);
    EXPECT_EQ(data[500], 500);
    EXPECT_EQ(data[999], 999);
}

// ============================================================================
// InferenceConfig Tests
// ============================================================================

TEST(InferenceConfigTest, DefaultConfig) {
    auto config = InferenceConfig::default_config();
    EXPECT_EQ(config.provider, ExecutionProvider::CPU);
    EXPECT_EQ(config.device_id, 0);
    EXPECT_EQ(config.intra_op_threads, 0);
    EXPECT_EQ(config.inter_op_threads, 0);
    EXPECT_TRUE(config.enable_memory_arena);
    EXPECT_FALSE(config.enable_profiling);
    EXPECT_EQ(config.timeout, std::chrono::milliseconds(1000));
}

TEST(InferenceConfigTest, CPUConfig) {
    auto config = InferenceConfig::cpu();
    EXPECT_EQ(config.provider, ExecutionProvider::CPU);
    EXPECT_EQ(config.intra_op_threads, 0);
    EXPECT_EQ(config.inter_op_threads, 0);
}

TEST(InferenceConfigTest, CPUConfigWithThreads) {
    auto config = InferenceConfig::cpu(4);
    EXPECT_EQ(config.provider, ExecutionProvider::CPU);
    EXPECT_EQ(config.intra_op_threads, 4);
    EXPECT_EQ(config.inter_op_threads, 4);
}

TEST(InferenceConfigTest, CUDAConfig) {
    auto config = InferenceConfig::cuda();
    EXPECT_EQ(config.provider, ExecutionProvider::CUDA);
    EXPECT_EQ(config.device_id, 0);
}

TEST(InferenceConfigTest, CUDAConfigWithDevice) {
    auto config = InferenceConfig::cuda(2);
    EXPECT_EQ(config.provider, ExecutionProvider::CUDA);
    EXPECT_EQ(config.device_id, 2);
}

TEST(InferenceConfigTest, TensorRTConfig) {
    auto config = InferenceConfig::tensorrt();
    EXPECT_EQ(config.provider, ExecutionProvider::TensorRT);
    EXPECT_EQ(config.device_id, 0);
    EXPECT_EQ(config.timeout, std::chrono::milliseconds(5000));  // Longer timeout
}

TEST(InferenceConfigTest, TensorRTConfigWithDevice) {
    auto config = InferenceConfig::tensorrt(1);
    EXPECT_EQ(config.provider, ExecutionProvider::TensorRT);
    EXPECT_EQ(config.device_id, 1);
}

TEST(InferenceConfigTest, CoreMLConfig) {
    auto config = InferenceConfig::coreml();
    EXPECT_EQ(config.provider, ExecutionProvider::CoreML);
}

TEST(InferenceConfigTest, DirectMLConfig) {
    auto config = InferenceConfig::directml();
    EXPECT_EQ(config.provider, ExecutionProvider::DirectML);
    EXPECT_EQ(config.device_id, 0);
}

TEST(InferenceConfigTest, DirectMLConfigWithDevice) {
    auto config = InferenceConfig::directml(3);
    EXPECT_EQ(config.provider, ExecutionProvider::DirectML);
    EXPECT_EQ(config.device_id, 3);
}

TEST(InferenceConfigTest, OpenVINOConfig) {
    auto config = InferenceConfig::openvino();
    EXPECT_EQ(config.provider, ExecutionProvider::OpenVINO);
}

TEST(InferenceConfigTest, CPUFastConfig) {
    auto config = InferenceConfig::cpu_fast();
    EXPECT_EQ(config.provider, ExecutionProvider::CPU);
    EXPECT_EQ(config.intra_op_threads, 0);  // Use all cores
    EXPECT_EQ(config.inter_op_threads, 0);
    EXPECT_TRUE(config.enable_memory_arena);
}

TEST(InferenceConfigTest, LowLatencyConfig) {
    auto config = InferenceConfig::low_latency();
    EXPECT_EQ(config.provider, ExecutionProvider::CPU);
    EXPECT_EQ(config.intra_op_threads, 1);  // Minimize thread overhead
    EXPECT_EQ(config.inter_op_threads, 1);
    EXPECT_TRUE(config.enable_memory_arena);
    EXPECT_EQ(config.timeout, std::chrono::milliseconds(100));
}

TEST(InferenceConfigTest, ProfilingConfig) {
    auto config = InferenceConfig::profiling();
    EXPECT_TRUE(config.enable_profiling);
    EXPECT_EQ(config.timeout, std::chrono::milliseconds(10000));
}

// ============================================================================
// InferenceStats Tests
// ============================================================================

TEST(InferenceStatsTest, DefaultConstruction) {
    InferenceStats stats;
    EXPECT_EQ(stats.total_inferences, 0);
    EXPECT_EQ(stats.successful_inferences, 0);
    EXPECT_EQ(stats.failed_inferences, 0);
    EXPECT_EQ(stats.total_inference_time.count(), 0);
}

TEST(InferenceStatsTest, AverageTimeNoInferences) {
    InferenceStats stats;
    EXPECT_EQ(stats.average_inference_time().count(), 0);
}

TEST(InferenceStatsTest, AverageTimeCalculation) {
    InferenceStats stats;
    stats.successful_inferences = 3;
    stats.total_inference_time = std::chrono::nanoseconds(3000);

    auto avg = stats.average_inference_time();
    EXPECT_EQ(avg.count(), 1000);
}

TEST(InferenceStatsTest, SuccessRateNoInferences) {
    InferenceStats stats;
    EXPECT_DOUBLE_EQ(stats.success_rate(), 0.0);
}

TEST(InferenceStatsTest, SuccessRateAllSuccess) {
    InferenceStats stats;
    stats.total_inferences = 10;
    stats.successful_inferences = 10;

    EXPECT_DOUBLE_EQ(stats.success_rate(), 1.0);
}

TEST(InferenceStatsTest, SuccessRatePartialSuccess) {
    InferenceStats stats;
    stats.total_inferences = 10;
    stats.successful_inferences = 7;

    EXPECT_NEAR(stats.success_rate(), 0.7, 0.001);
}

TEST(InferenceStatsTest, ThroughputCalculation) {
    InferenceStats stats;
    stats.successful_inferences = 1000;
    stats.total_inference_time = std::chrono::seconds(1);

    EXPECT_NEAR(stats.throughput(), 1000.0, 0.1);
}

TEST(InferenceStatsTest, ThroughputZeroTime) {
    InferenceStats stats;
    stats.successful_inferences = 100;
    stats.total_inference_time = std::chrono::nanoseconds(0);

    EXPECT_DOUBLE_EQ(stats.throughput(), 0.0);
}

TEST(InferenceStatsTest, Reset) {
    InferenceStats stats;
    stats.total_inferences = 100;
    stats.successful_inferences = 90;
    stats.failed_inferences = 10;
    stats.total_inference_time = std::chrono::seconds(10);

    stats.reset();

    EXPECT_EQ(stats.total_inferences, 0);
    EXPECT_EQ(stats.successful_inferences, 0);
    EXPECT_EQ(stats.failed_inferences, 0);
    EXPECT_EQ(stats.total_inference_time.count(), 0);
}

// ============================================================================
// InferenceSession Tests
// ============================================================================

class InferenceSessionTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Use CPU provider which should always be available
        InferenceConfig config = InferenceConfig::cpu();
        session = std::make_unique<InferenceSession>(config);
    }

    void TearDown() override {
        if (session && session->is_initialized()) {
            session->shutdown();
        }
    }

    std::unique_ptr<InferenceSession> session;
};

TEST_F(InferenceSessionTest, InitiallyNotInitialized) {
    EXPECT_FALSE(session->is_initialized());
}

TEST_F(InferenceSessionTest, Initialize) {
    auto result = session->initialize();
    EXPECT_EQ(result, InferenceResult::Success);
    EXPECT_TRUE(session->is_initialized());
}

TEST_F(InferenceSessionTest, InitializeTwice) {
    session->initialize();
    auto result = session->initialize();
    EXPECT_EQ(result, InferenceResult::AlreadyInitialized);
}

TEST_F(InferenceSessionTest, Shutdown) {
    session->initialize();
    auto result = session->shutdown();
    EXPECT_EQ(result, InferenceResult::Success);
    EXPECT_FALSE(session->is_initialized());
}

TEST_F(InferenceSessionTest, ShutdownWithoutInit) {
    auto result = session->shutdown();
    EXPECT_EQ(result, InferenceResult::NotInitialized);
}

TEST_F(InferenceSessionTest, GetConfig) {
    const auto& config = session->get_config();
    EXPECT_EQ(config.provider, ExecutionProvider::CPU);
}

TEST_F(InferenceSessionTest, GetProvider) {
    EXPECT_EQ(session->get_provider(), ExecutionProvider::CPU);
}

TEST_F(InferenceSessionTest, InitiallyModelNotLoaded) {
    session->initialize();
    EXPECT_FALSE(session->is_model_loaded());
}

TEST_F(InferenceSessionTest, LoadModelNotInitialized) {
    auto result = session->load_model("model.onnx");
    EXPECT_EQ(result, InferenceResult::NotInitialized);
}

TEST_F(InferenceSessionTest, LoadModelEmptyPath) {
    session->initialize();
    auto result = session->load_model("");
    EXPECT_EQ(result, InferenceResult::InvalidModel);
}

TEST_F(InferenceSessionTest, LoadModelSuccess) {
    session->initialize();
    auto result = session->load_model("model.onnx");
    EXPECT_EQ(result, InferenceResult::Success);
    EXPECT_TRUE(session->is_model_loaded());
}

TEST_F(InferenceSessionTest, LoadModelFromMemoryNotInitialized) {
    std::vector<UInt8> data = {1, 2, 3, 4};
    auto result = session->load_model_from_memory(data);
    EXPECT_EQ(result, InferenceResult::NotInitialized);
}

TEST_F(InferenceSessionTest, LoadModelFromMemoryEmpty) {
    session->initialize();
    std::vector<UInt8> empty_data;
    auto result = session->load_model_from_memory(empty_data);
    EXPECT_EQ(result, InferenceResult::InvalidModel);
}

TEST_F(InferenceSessionTest, LoadModelFromMemorySuccess) {
    session->initialize();
    std::vector<UInt8> data(1024, 0);  // Mock model data
    auto result = session->load_model_from_memory(data);
    EXPECT_EQ(result, InferenceResult::Success);
    EXPECT_TRUE(session->is_model_loaded());
}

TEST_F(InferenceSessionTest, GetInputInfoNoModel) {
    session->initialize();
    auto info = session->get_input_info();
    EXPECT_TRUE(info.empty());
}

TEST_F(InferenceSessionTest, GetInputInfoWithModel) {
    ASSERT_TRUE(session != nullptr);
    auto init_result = session->initialize();
    ASSERT_EQ(init_result, InferenceResult::Success);

    auto load_result = session->load_model("model.onnx");
    ASSERT_EQ(load_result, InferenceResult::Success);
    ASSERT_TRUE(session->is_model_loaded());

    auto info = session->get_input_info();
    ASSERT_FALSE(info.empty());
    EXPECT_EQ(info[0].name, "input");
    EXPECT_TRUE(info[0].is_input);
}

TEST_F(InferenceSessionTest, GetOutputInfoWithModel) {
    session->initialize();
    session->load_model("model.onnx");

    auto info = session->get_output_info();
    EXPECT_FALSE(info.empty());
    EXPECT_EQ(info[0].name, "output");
    EXPECT_FALSE(info[0].is_input);
}

TEST_F(InferenceSessionTest, RunNotInitialized) {
    std::vector<Tensor> inputs;
    std::vector<Tensor> outputs;

    auto result = session->run(inputs, outputs);
    EXPECT_EQ(result, InferenceResult::NotInitialized);
}

TEST_F(InferenceSessionTest, RunModelNotLoaded) {
    session->initialize();

    std::vector<Tensor> inputs;
    std::vector<Tensor> outputs;

    auto result = session->run(inputs, outputs);
    EXPECT_EQ(result, InferenceResult::ModelNotLoaded);
}

TEST_F(InferenceSessionTest, RunInvalidInputs) {
    session->initialize();
    session->load_model("model.onnx");

    // Create invalid inputs (wrong shape or count)
    std::vector<Tensor> inputs;
    std::vector<Tensor> outputs;

    auto result = session->run(inputs, outputs);
    EXPECT_EQ(result, InferenceResult::InvalidInput);
}

TEST_F(InferenceSessionTest, RunSuccess) {
    session->initialize();
    session->load_model("model.onnx");

    // Create valid inputs matching model expectations
    auto input_info = session->get_input_info();
    ASSERT_FALSE(input_info.empty());

    Tensor input(input_info[0].name, input_info[0].data_type, input_info[0].shape);

    // Fill input with test data
    std::vector<float> input_data(input.element_count(), 1.0f);
    input.set_data(input_data);

    std::vector<Tensor> inputs = {input};
    std::vector<Tensor> outputs;

    auto result = session->run(inputs, outputs);
    EXPECT_EQ(result, InferenceResult::Success);
    EXPECT_FALSE(outputs.empty());
}

TEST_F(InferenceSessionTest, RunAsyncSuccess) {
    session->initialize();
    session->load_model("model.onnx");

    auto input_info = session->get_input_info();
    ASSERT_FALSE(input_info.empty());

    Tensor input(input_info[0].name, input_info[0].data_type, input_info[0].shape);
    std::vector<float> input_data(input.element_count(), 1.0f);
    input.set_data(input_data);

    std::vector<Tensor> inputs = {input};

    bool callback_called = false;
    InferenceResult async_result = InferenceResult::RuntimeError;

    auto result = session->run_async(inputs, [&](InferenceResult res, std::vector<Tensor> outputs) {
        callback_called = true;
        async_result = res;
    });

    EXPECT_EQ(result, InferenceResult::Success);

    // Wait a bit for async operation to complete
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    EXPECT_TRUE(callback_called);
    EXPECT_EQ(async_result, InferenceResult::Success);
}

TEST_F(InferenceSessionTest, RunAsyncNullCallback) {
    session->initialize();
    session->load_model("model.onnx");

    std::vector<Tensor> inputs;
    auto result = session->run_async(inputs, nullptr);

    EXPECT_EQ(result, InferenceResult::InvalidConfiguration);
}

TEST_F(InferenceSessionTest, GetStatsInitial) {
    session->initialize();
    auto stats = session->get_stats();

    EXPECT_EQ(stats.total_inferences, 0);
    EXPECT_EQ(stats.successful_inferences, 0);
}

TEST_F(InferenceSessionTest, GetStatsAfterInference) {
    session->initialize();
    session->load_model("model.onnx");

    auto input_info = session->get_input_info();
    Tensor input(input_info[0].name, input_info[0].data_type, input_info[0].shape);
    std::vector<float> input_data(input.element_count(), 1.0f);
    input.set_data(input_data);

    std::vector<Tensor> inputs = {input};
    std::vector<Tensor> outputs;

    session->run(inputs, outputs);

    auto stats = session->get_stats();
    EXPECT_EQ(stats.total_inferences, 1);
    EXPECT_EQ(stats.successful_inferences, 1);
    EXPECT_GT(stats.total_inference_time.count(), 0);
}

TEST_F(InferenceSessionTest, ResetStats) {
    session->initialize();
    session->load_model("model.onnx");

    auto input_info = session->get_input_info();
    Tensor input(input_info[0].name, input_info[0].data_type, input_info[0].shape);
    std::vector<float> input_data(input.element_count(), 1.0f);
    input.set_data(input_data);

    std::vector<Tensor> inputs = {input};
    std::vector<Tensor> outputs;

    session->run(inputs, outputs);
    EXPECT_GT(session->get_stats().total_inferences, 0);

    session->reset_stats();
    EXPECT_EQ(session->get_stats().total_inferences, 0);
}

TEST_F(InferenceSessionTest, WarmupNotInitialized) {
    auto result = session->warmup(3);
    EXPECT_EQ(result, InferenceResult::NotInitialized);
}

TEST_F(InferenceSessionTest, WarmupModelNotLoaded) {
    session->initialize();
    auto result = session->warmup(3);
    EXPECT_EQ(result, InferenceResult::ModelNotLoaded);
}

TEST_F(InferenceSessionTest, WarmupSuccess) {
    session->initialize();
    session->load_model("model.onnx");

    auto result = session->warmup(3);
    EXPECT_EQ(result, InferenceResult::Success);

    // Should have run 3 warmup inferences
    auto stats = session->get_stats();
    EXPECT_GE(stats.total_inferences, 3);
}

TEST_F(InferenceSessionTest, SetProviderNotInitialized) {
    auto result = session->set_provider(ExecutionProvider::CUDA);
    EXPECT_EQ(result, InferenceResult::NotInitialized);
}

TEST_F(InferenceSessionTest, SetProviderUnavailable) {
    session->initialize();

    // CUDA is not available in stub implementation
    auto result = session->set_provider(ExecutionProvider::CUDA);
    EXPECT_EQ(result, InferenceResult::ProviderNotSupported);
}

TEST_F(InferenceSessionTest, SetProviderAvailable) {
    session->initialize();

    auto result = session->set_provider(ExecutionProvider::CPU);
    EXPECT_EQ(result, InferenceResult::Success);
    EXPECT_EQ(session->get_provider(), ExecutionProvider::CPU);
}

TEST_F(InferenceSessionTest, GetProfilingDataDisabled) {
    session->initialize();
    auto data = session->get_profiling_data();
    EXPECT_TRUE(data.empty());
}

TEST_F(InferenceSessionTest, MoveConstruction) {
    session->initialize();
    session->load_model("model.onnx");

    InferenceSession moved_session(std::move(*session));
    EXPECT_TRUE(moved_session.is_initialized());
    EXPECT_TRUE(moved_session.is_model_loaded());

    // Reset session to prevent TearDown from accessing moved-from object
    session.reset();
}

TEST_F(InferenceSessionTest, MoveAssignment) {
    session->initialize();
    session->load_model("model.onnx");

    InferenceSession other_session(InferenceConfig::cpu());
    other_session = std::move(*session);

    EXPECT_TRUE(other_session.is_initialized());
    EXPECT_TRUE(other_session.is_model_loaded());

    // Reset session to prevent TearDown from accessing moved-from object
    session.reset();
}

// ============================================================================
// Factory Function Tests
// ============================================================================

TEST(FactoryFunctionTest, CreateInferenceSession) {
    auto session = create_inference_session();
    ASSERT_NE(session, nullptr);
    EXPECT_FALSE(session->is_initialized());
}

TEST(FactoryFunctionTest, CreateInferenceSessionWithConfig) {
    auto config = InferenceConfig::cpu(4);
    auto session = create_inference_session(config);
    ASSERT_NE(session, nullptr);
    EXPECT_EQ(session->get_config().intra_op_threads, 4);
}

// ============================================================================
// Helper Function Tests
// ============================================================================

TEST(HelperFunctionTest, CreateTensorFloat32) {
    auto tensor = create_tensor<float>("data", TensorShape{2, 3}, {1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f});

    EXPECT_EQ(tensor.name, "data");
    EXPECT_EQ(tensor.data_type, DataType::Float32);
    EXPECT_EQ(tensor.shape.rank(), 2);

    const float* data = tensor.data_as<float>();
    ASSERT_NE(data, nullptr);
    EXPECT_FLOAT_EQ(data[0], 1.0f);
    EXPECT_FLOAT_EQ(data[5], 6.0f);
}

TEST(HelperFunctionTest, CreateTensorInt64) {
    auto tensor = create_tensor<Int64>("indices", TensorShape{4}, {10, 20, 30, 40});

    EXPECT_EQ(tensor.data_type, DataType::Int64);

    const Int64* data = tensor.data_as<Int64>();
    ASSERT_NE(data, nullptr);
    EXPECT_EQ(data[1], 20);
}

TEST(HelperFunctionTest, ValidateTensorsSuccess) {
    std::vector<Tensor> tensors;
    tensors.emplace_back("input", DataType::Float32, TensorShape{1, 32});
    std::vector<float> data(32, 1.0f);
    tensors[0].set_data(data);

    std::vector<TensorInfo> expected;
    expected.emplace_back("input", DataType::Float32, TensorShape{1, 32}, true);

    EXPECT_TRUE(validate_tensors(tensors, expected, true));
}

TEST(HelperFunctionTest, ValidateTensorsWrongCount) {
    std::vector<Tensor> tensors;
    tensors.emplace_back("input", DataType::Float32, TensorShape{1, 32});

    std::vector<TensorInfo> expected;
    expected.emplace_back("input1", DataType::Float32, TensorShape{1, 32});
    expected.emplace_back("input2", DataType::Float32, TensorShape{1, 32});

    EXPECT_FALSE(validate_tensors(tensors, expected, false));
}

TEST(HelperFunctionTest, ValidateTensorsWrongName) {
    std::vector<Tensor> tensors;
    tensors.emplace_back("wrong_name", DataType::Float32, TensorShape{1, 32});

    std::vector<TensorInfo> expected;
    expected.emplace_back("input", DataType::Float32, TensorShape{1, 32});

    EXPECT_FALSE(validate_tensors(tensors, expected, false));
}

TEST(HelperFunctionTest, ValidateTensorsWrongType) {
    std::vector<Tensor> tensors;
    tensors.emplace_back("input", DataType::Float32, TensorShape{1, 32});

    std::vector<TensorInfo> expected;
    expected.emplace_back("input", DataType::Int32, TensorShape{1, 32});

    EXPECT_FALSE(validate_tensors(tensors, expected, false));
}

TEST(HelperFunctionTest, ValidateTensorsWrongShape) {
    std::vector<Tensor> tensors;
    tensors.emplace_back("input", DataType::Float32, TensorShape{1, 32});

    std::vector<TensorInfo> expected;
    expected.emplace_back("input", DataType::Float32, TensorShape{1, 64});

    EXPECT_FALSE(validate_tensors(tensors, expected, false));
}

TEST(HelperFunctionTest, ValidateTensorsEmptyData) {
    std::vector<Tensor> tensors;
    Tensor tensor;
    tensor.name = "input";
    tensor.data_type = DataType::Float32;
    tensor.shape = TensorShape{1, 32};
    // Explicitly leave data empty (don't resize)
    tensors.push_back(std::move(tensor));

    std::vector<TensorInfo> expected;
    expected.emplace_back("input", DataType::Float32, TensorShape{1, 32});

    EXPECT_FALSE(validate_tensors(tensors, expected, true));  // check_data = true
}

TEST(HelperFunctionTest, ValidateTensorsDynamicShape) {
    std::vector<Tensor> tensors;
    tensors.emplace_back("input", DataType::Float32, TensorShape{2, 32});

    std::vector<TensorInfo> expected;
    expected.emplace_back("input", DataType::Float32, TensorShape{-1, 32});  // Dynamic batch

    EXPECT_TRUE(validate_tensors(tensors, expected, false));
}

TEST(HelperFunctionTest, CalculateTensorMemory) {
    std::vector<Tensor> tensors;
    tensors.emplace_back("t1", DataType::Float32, TensorShape{10});      // 40 bytes
    tensors.emplace_back("t2", DataType::Float64, TensorShape{5});       // 40 bytes
    tensors.emplace_back("t3", DataType::Int32, TensorShape{20});        // 80 bytes

    UInt64 total = calculate_tensor_memory(tensors);
    EXPECT_EQ(total, 160);
}

TEST(HelperFunctionTest, CalculateTensorMemoryEmpty) {
    std::vector<Tensor> tensors;
    UInt64 total = calculate_tensor_memory(tensors);
    EXPECT_EQ(total, 0);
}

TEST(HelperFunctionTest, FormatShapeStatic) {
    TensorShape shape{1, 3, 224, 224};
    std::string formatted = format_shape(shape);
    EXPECT_EQ(formatted, "[1, 3, 224, 224]");
}

TEST(HelperFunctionTest, FormatShapeDynamic) {
    TensorShape shape{-1, 3, 224, 224};
    std::string formatted = format_shape(shape);
    EXPECT_EQ(formatted, "[?, 3, 224, 224]");
}

TEST(HelperFunctionTest, FormatShapeEmpty) {
    TensorShape shape;
    std::string formatted = format_shape(shape);
    EXPECT_EQ(formatted, "[]");
}

TEST(HelperFunctionTest, FormatTensorInfo) {
    TensorInfo info("input", DataType::Float32, TensorShape{1, 3, 224, 224});
    std::string formatted = format_tensor_info(info);
    EXPECT_EQ(formatted, "input: Float32 [1, 3, 224, 224]");
}

TEST(HelperFunctionTest, RecommendBatchSizeNormal) {
    UInt64 model_size = 1024 * 1024;        // 1 MB per input
    UInt64 available_memory = 100 * 1024 * 1024;  // 100 MB

    UInt32 batch_size = recommend_batch_size(model_size, available_memory);

    // Should use 80% of memory: 80 MB / 1 MB = 80
    EXPECT_EQ(batch_size, 80);
}

TEST(HelperFunctionTest, RecommendBatchSizeZeroInput) {
    UInt32 batch_size = recommend_batch_size(0, 1024 * 1024);
    EXPECT_EQ(batch_size, 1);
}

TEST(HelperFunctionTest, RecommendBatchSizeTooSmall) {
    UInt64 model_size = 10 * 1024 * 1024;   // 10 MB per input
    UInt64 available_memory = 5 * 1024 * 1024;   // Only 5 MB available

    UInt32 batch_size = recommend_batch_size(model_size, available_memory);
    EXPECT_EQ(batch_size, 1);  // Clamp to minimum
}

TEST(HelperFunctionTest, RecommendBatchSizeTooLarge) {
    UInt64 model_size = 1024;               // 1 KB per input
    UInt64 available_memory = 10ULL * 1024 * 1024 * 1024;  // 10 GB

    UInt32 batch_size = recommend_batch_size(model_size, available_memory);
    EXPECT_EQ(batch_size, 1024);  // Clamp to maximum
}

// ============================================================================
// Edge Case Tests
// ============================================================================

TEST(EdgeCaseTest, UnavailableProvider) {
    InferenceConfig config = InferenceConfig::cuda();
    auto session = std::make_unique<InferenceSession>(config);

    auto result = session->initialize();
    // Should fail since CUDA is not available in stub
    EXPECT_EQ(result, InferenceResult::ProviderNotSupported);
}

TEST(EdgeCaseTest, InvalidThreadConfiguration) {
    InferenceConfig config = InferenceConfig::cpu();
    config.intra_op_threads = 2000;  // Exceeds limit

    auto session = std::make_unique<InferenceSession>(config);
    auto result = session->initialize();

    EXPECT_EQ(result, InferenceResult::InvalidConfiguration);
}

TEST(EdgeCaseTest, LargeTensor) {
    // Create a very large tensor
    Tensor tensor("large", DataType::Float32, TensorShape{1000, 1000});
    EXPECT_EQ(tensor.element_count(), 1000000);
    EXPECT_EQ(tensor.size_in_bytes(), 4000000);  // 4 MB
}

TEST(EdgeCaseTest, ZeroSizeTensor) {
    Tensor tensor("empty", DataType::Float32, TensorShape{0});
    EXPECT_EQ(tensor.element_count(), 0);
    EXPECT_EQ(tensor.size_in_bytes(), 0);
}

// ============================================================================
// Performance Tests
// ============================================================================

TEST(PerformanceTest, TensorShapeElementCountPerformance) {
    TensorShape shape{1, 3, 224, 224};

    auto start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < 1000000; i++) {
        [[maybe_unused]] auto count = shape.element_count();
    }
    auto end = std::chrono::high_resolution_clock::now();

    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    // Should be very fast (< 10ms for 1M calls)
    EXPECT_LT(duration.count(), 10000);
}

TEST(PerformanceTest, TensorDataAccessPerformance) {
    Tensor tensor("data", DataType::Float32, TensorShape{1000});
    float* data = tensor.data_as<float>();
    ASSERT_NE(data, nullptr);

    auto start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < 1000; i++) {
        data[i] = static_cast<float>(i);
    }
    auto end = std::chrono::high_resolution_clock::now();

    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    // Should be very fast (< 100us for 1K writes)
    EXPECT_LT(duration.count(), 100);
}
