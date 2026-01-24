#pragma once
/**
 * @file inference_session.h
 * @brief ONNX Runtime wrapper for ML model inference
 *
 * This file provides a high-level interface for running machine learning
 * model inference using ONNX Runtime. Supports multiple execution providers
 * (CPU, CUDA, TensorRT, CoreML, etc.) with comprehensive configuration options.
 *
 * Key features:
 * - Multiple execution providers (CPU, GPU, specialized accelerators)
 * - Synchronous and asynchronous inference
 * - Model loading from file or memory
 * - Automatic tensor shape validation
 * - Performance statistics and profiling
 * - Type-safe tensor data access
 * - Warmup support for consistent performance
 * - Provider availability checking
 */

#include "jaguar/core/types.h"
#include <vector>
#include <string>
#include <memory>
#include <functional>
#include <chrono>
#include <cstring>

namespace jaguar::ml {

// ============================================================================
// Forward Declarations
// ============================================================================

class IInferenceSession;
class InferenceSession;

// ============================================================================
// Inference Result Enum
// ============================================================================

/**
 * @brief Result codes for inference operations
 */
enum class InferenceResult : UInt8 {
    Success = 0,

    // Configuration errors
    InvalidConfiguration,
    InvalidModel,
    InvalidInput,
    InvalidOutput,

    // Runtime errors
    RuntimeError,
    ExecutionFailed,
    TimeoutError,

    // State errors
    NotInitialized,
    AlreadyInitialized,
    ModelNotLoaded,

    // Resource errors
    OutOfMemory,
    DeviceUnavailable,
    ProviderNotSupported
};

/**
 * @brief Convert InferenceResult to string
 */
inline const char* inference_result_to_string(InferenceResult result) {
    switch (result) {
        case InferenceResult::Success: return "Success";
        case InferenceResult::InvalidConfiguration: return "InvalidConfiguration";
        case InferenceResult::InvalidModel: return "InvalidModel";
        case InferenceResult::InvalidInput: return "InvalidInput";
        case InferenceResult::InvalidOutput: return "InvalidOutput";
        case InferenceResult::RuntimeError: return "RuntimeError";
        case InferenceResult::ExecutionFailed: return "ExecutionFailed";
        case InferenceResult::TimeoutError: return "TimeoutError";
        case InferenceResult::NotInitialized: return "NotInitialized";
        case InferenceResult::AlreadyInitialized: return "AlreadyInitialized";
        case InferenceResult::ModelNotLoaded: return "ModelNotLoaded";
        case InferenceResult::OutOfMemory: return "OutOfMemory";
        case InferenceResult::DeviceUnavailable: return "DeviceUnavailable";
        case InferenceResult::ProviderNotSupported: return "ProviderNotSupported";
        default: return "Unknown";
    }
}

// ============================================================================
// Execution Provider Enum
// ============================================================================

/**
 * @brief Execution providers for inference acceleration
 */
enum class ExecutionProvider : UInt8 {
    /// Default CPU execution
    CPU = 0,

    /// NVIDIA GPU acceleration via CUDA
    CUDA,

    /// NVIDIA TensorRT optimization engine
    TensorRT,

    /// Apple CoreML for macOS/iOS devices
    CoreML,

    /// Windows DirectML for DirectX 12 GPUs
    DirectML,

    /// Intel OpenVINO for Intel hardware
    OpenVINO
};

/**
 * @brief Convert ExecutionProvider to string
 */
inline const char* execution_provider_to_string(ExecutionProvider provider) {
    switch (provider) {
        case ExecutionProvider::CPU: return "CPU";
        case ExecutionProvider::CUDA: return "CUDA";
        case ExecutionProvider::TensorRT: return "TensorRT";
        case ExecutionProvider::CoreML: return "CoreML";
        case ExecutionProvider::DirectML: return "DirectML";
        case ExecutionProvider::OpenVINO: return "OpenVINO";
        default: return "Unknown";
    }
}

// ============================================================================
// Data Type Enum
// ============================================================================

/**
 * @brief Tensor data types supported by inference engine
 */
enum class DataType : UInt8 {
    /// 32-bit floating point
    Float32 = 0,

    /// 64-bit floating point
    Float64,

    /// 32-bit signed integer
    Int32,

    /// 64-bit signed integer
    Int64,

    /// 8-bit unsigned integer
    UInt8,

    /// Boolean (1 byte)
    Bool
};

/**
 * @brief Convert DataType to string
 */
inline const char* data_type_to_string(DataType type) {
    switch (type) {
        case DataType::Float32: return "Float32";
        case DataType::Float64: return "Float64";
        case DataType::Int32: return "Int32";
        case DataType::Int64: return "Int64";
        case DataType::UInt8: return "UInt8";
        case DataType::Bool: return "Bool";
        default: return "Unknown";
    }
}

/**
 * @brief Get size in bytes of a data type
 */
inline UInt64 data_type_size(DataType type) {
    switch (type) {
        case DataType::Float32: return 4;
        case DataType::Float64: return 8;
        case DataType::Int32: return 4;
        case DataType::Int64: return 8;
        case DataType::UInt8: return 1;
        case DataType::Bool: return 1;
        default: return 0;
    }
}

// ============================================================================
// Tensor Shape
// ============================================================================

/**
 * @brief Shape descriptor for tensors
 */
struct TensorShape {
    /// Dimensions of the tensor (e.g., [1, 3, 224, 224] for batch of images)
    std::vector<Int64> dimensions;

    TensorShape() = default;

    /**
     * @brief Construct from initializer list
     */
    TensorShape(std::initializer_list<Int64> dims)
        : dimensions(dims) {}

    /**
     * @brief Construct from vector
     */
    explicit TensorShape(std::vector<Int64> dims)
        : dimensions(std::move(dims)) {}

    /**
     * @brief Calculate total number of elements
     * @return Product of all dimensions, or 0 if dynamic
     */
    Int64 element_count() const noexcept {
        if (dimensions.empty()) {
            return 0;
        }

        Int64 count = 1;
        for (auto dim : dimensions) {
            // Negative dimension means dynamic shape
            if (dim < 0) {
                return 0;
            }
            count *= dim;
        }
        return count;
    }

    /**
     * @brief Check if shape has dynamic dimensions
     * @return True if any dimension is -1 (dynamic)
     */
    bool is_dynamic() const noexcept {
        for (auto dim : dimensions) {
            if (dim < 0) {
                return true;
            }
        }
        return false;
    }

    /**
     * @brief Get number of dimensions (rank)
     */
    UInt64 rank() const noexcept {
        return dimensions.size();
    }

    /**
     * @brief Equality comparison
     */
    bool operator==(const TensorShape& other) const noexcept {
        return dimensions == other.dimensions;
    }

    bool operator!=(const TensorShape& other) const noexcept {
        return !(*this == other);
    }
};

// ============================================================================
// Tensor Info
// ============================================================================

/**
 * @brief Metadata about a model input or output tensor
 */
struct TensorInfo {
    /// Name of the tensor (e.g., "input", "output", "logits")
    std::string name;

    /// Data type of the tensor elements
    DataType data_type{DataType::Float32};

    /// Shape of the tensor
    TensorShape shape;

    /// Whether this is an input tensor (vs output)
    bool is_input{true};

    TensorInfo() = default;

    TensorInfo(std::string name_, DataType type, TensorShape shape_, bool input = true)
        : name(std::move(name_))
        , data_type(type)
        , shape(std::move(shape_))
        , is_input(input) {}

    /**
     * @brief Calculate size in bytes needed for this tensor
     */
    UInt64 size_in_bytes() const noexcept {
        return static_cast<UInt64>(shape.element_count()) * data_type_size(data_type);
    }
};

// ============================================================================
// Tensor
// ============================================================================

/**
 * @brief Container for tensor data used in inference
 */
struct Tensor {
    /// Name of the tensor
    std::string name;

    /// Data type of tensor elements
    DataType data_type{DataType::Float32};

    /// Shape of the tensor
    TensorShape shape;

    /// Raw tensor data as bytes
    std::vector<UInt8> data;

    Tensor() = default;

    Tensor(std::string name_, DataType type, TensorShape shape_)
        : name(std::move(name_))
        , data_type(type)
        , shape(std::move(shape_)) {
        // Pre-allocate data buffer
        UInt64 size = static_cast<UInt64>(shape.element_count()) * data_type_size(data_type);
        data.resize(size);
    }

    /**
     * @brief Get typed view of tensor data
     * @tparam T Element type (must match data_type)
     * @return Pointer to typed data, or nullptr if type mismatch
     */
    template<typename T>
    T* data_as() noexcept {
        if (!validate_type<T>()) {
            return nullptr;
        }
        return reinterpret_cast<T*>(data.data());
    }

    template<typename T>
    const T* data_as() const noexcept {
        if (!validate_type<T>()) {
            return nullptr;
        }
        return reinterpret_cast<const T*>(data.data());
    }

    /**
     * @brief Set tensor data from typed vector
     * @tparam T Element type (must match data_type)
     * @param values Vector of values to copy
     * @return True if successful, false if type mismatch or size mismatch
     */
    template<typename T>
    bool set_data(const std::vector<T>& values) {
        if (!validate_type<T>()) {
            return false;
        }

        Int64 expected_count = shape.element_count();
        if (expected_count > 0 && static_cast<Int64>(values.size()) != expected_count) {
            return false;
        }

        UInt64 byte_size = values.size() * sizeof(T);
        data.resize(byte_size);
        std::memcpy(data.data(), values.data(), byte_size);
        return true;
    }

    /**
     * @brief Get number of elements in the tensor
     */
    Int64 element_count() const noexcept {
        return shape.element_count();
    }

    /**
     * @brief Get size in bytes
     */
    UInt64 size_in_bytes() const noexcept {
        return data.size();
    }

private:
    template<typename T>
    bool validate_type() const noexcept {
        if constexpr (std::is_same_v<T, float>) {
            return data_type == DataType::Float32;
        } else if constexpr (std::is_same_v<T, double>) {
            return data_type == DataType::Float64;
        } else if constexpr (std::is_same_v<T, Int32>) {
            return data_type == DataType::Int32;
        } else if constexpr (std::is_same_v<T, Int64>) {
            return data_type == DataType::Int64;
        } else if constexpr (std::is_same_v<T, UInt8>) {
            return data_type == DataType::UInt8;
        } else if constexpr (std::is_same_v<T, bool>) {
            return data_type == DataType::Bool;
        }
        return false;
    }
};

// ============================================================================
// Inference Configuration
// ============================================================================

/**
 * @brief Configuration for inference session
 */
struct InferenceConfig {
    /// Execution provider to use for inference
    ExecutionProvider provider{ExecutionProvider::CPU};

    /// Device ID for GPU providers (0 = first GPU)
    Int32 device_id{0};

    /// Number of threads for intra-operator parallelism (0 = auto)
    UInt32 intra_op_threads{0};

    /// Number of threads for inter-operator parallelism (0 = auto)
    UInt32 inter_op_threads{0};

    /// Enable memory arena for faster allocation
    bool enable_memory_arena{true};

    /// Enable performance profiling
    bool enable_profiling{false};

    /// Timeout for inference operations
    std::chrono::milliseconds timeout{1000};

    /// Factory methods for common configurations
    static InferenceConfig default_config() noexcept {
        return InferenceConfig{};
    }

    /**
     * @brief CPU-only configuration
     * @param threads Number of threads (0 = auto)
     */
    static InferenceConfig cpu(UInt32 threads = 0) noexcept {
        InferenceConfig config;
        config.provider = ExecutionProvider::CPU;
        config.intra_op_threads = threads;
        config.inter_op_threads = threads;
        return config;
    }

    /**
     * @brief CUDA GPU configuration
     * @param device GPU device ID
     */
    static InferenceConfig cuda(Int32 device = 0) noexcept {
        InferenceConfig config;
        config.provider = ExecutionProvider::CUDA;
        config.device_id = device;
        return config;
    }

    /**
     * @brief TensorRT configuration
     * @param device GPU device ID
     */
    static InferenceConfig tensorrt(Int32 device = 0) noexcept {
        InferenceConfig config;
        config.provider = ExecutionProvider::TensorRT;
        config.device_id = device;
        // TensorRT benefits from longer timeout for engine building
        config.timeout = std::chrono::milliseconds(5000);
        return config;
    }

    /**
     * @brief CoreML configuration for Apple devices
     */
    static InferenceConfig coreml() noexcept {
        InferenceConfig config;
        config.provider = ExecutionProvider::CoreML;
        return config;
    }

    /**
     * @brief DirectML configuration for Windows
     * @param device GPU device ID
     */
    static InferenceConfig directml(Int32 device = 0) noexcept {
        InferenceConfig config;
        config.provider = ExecutionProvider::DirectML;
        config.device_id = device;
        return config;
    }

    /**
     * @brief OpenVINO configuration for Intel hardware
     */
    static InferenceConfig openvino() noexcept {
        InferenceConfig config;
        config.provider = ExecutionProvider::OpenVINO;
        return config;
    }

    /**
     * @brief High-performance CPU configuration
     */
    static InferenceConfig cpu_fast() noexcept {
        InferenceConfig config;
        config.provider = ExecutionProvider::CPU;
        config.intra_op_threads = 0; // Use all cores
        config.inter_op_threads = 0;
        config.enable_memory_arena = true;
        return config;
    }

    /**
     * @brief Low-latency configuration
     */
    static InferenceConfig low_latency() noexcept {
        InferenceConfig config;
        config.provider = ExecutionProvider::CPU;
        config.intra_op_threads = 1; // Minimize thread overhead
        config.inter_op_threads = 1;
        config.enable_memory_arena = true;
        config.timeout = std::chrono::milliseconds(100);
        return config;
    }

    /**
     * @brief Profiling configuration for performance analysis
     */
    static InferenceConfig profiling() noexcept {
        InferenceConfig config;
        config.enable_profiling = true;
        config.timeout = std::chrono::milliseconds(10000); // Longer timeout
        return config;
    }
};

// ============================================================================
// Inference Statistics
// ============================================================================

/**
 * @brief Performance statistics for inference operations
 */
struct InferenceStats {
    /// Total number of inference calls
    UInt64 total_inferences{0};

    /// Number of successful inferences
    UInt64 successful_inferences{0};

    /// Number of failed inferences
    UInt64 failed_inferences{0};

    /// Total time spent in inference
    std::chrono::nanoseconds total_inference_time{0};

    /// Minimum inference time observed
    std::chrono::nanoseconds min_inference_time{std::chrono::nanoseconds::max()};

    /// Maximum inference time observed
    std::chrono::nanoseconds max_inference_time{0};

    /**
     * @brief Calculate average inference time
     * @return Average time per inference, or 0 if no inferences
     */
    std::chrono::nanoseconds average_inference_time() const noexcept {
        if (successful_inferences == 0) {
            return std::chrono::nanoseconds{0};
        }
        return total_inference_time / successful_inferences;
    }

    /**
     * @brief Calculate success rate
     * @return Ratio of successful to total inferences (0.0 to 1.0)
     */
    Real success_rate() const noexcept {
        if (total_inferences == 0) {
            return 0.0;
        }
        return static_cast<Real>(successful_inferences) / static_cast<Real>(total_inferences);
    }

    /**
     * @brief Calculate average throughput in inferences per second
     */
    Real throughput() const noexcept {
        if (total_inference_time.count() == 0) {
            return 0.0;
        }
        Real seconds = std::chrono::duration<Real>(total_inference_time).count();
        return static_cast<Real>(successful_inferences) / seconds;
    }

    /**
     * @brief Reset all statistics
     */
    void reset() noexcept {
        *this = InferenceStats{};
    }
};

// ============================================================================
// Inference Session Interface
// ============================================================================

/**
 * @brief Interface for ML inference sessions
 *
 * Provides abstract interface for running ML model inference.
 * Implementations can use different backends (ONNX Runtime, TensorRT, etc.)
 */
class IInferenceSession {
public:
    virtual ~IInferenceSession() = default;

    // ========================================================================
    // Lifecycle
    // ========================================================================

    /**
     * @brief Initialize the inference session
     * @return Success or error code
     */
    virtual InferenceResult initialize() = 0;

    /**
     * @brief Shutdown and cleanup resources
     * @return Success or error code
     */
    virtual InferenceResult shutdown() = 0;

    /**
     * @brief Check if session is initialized
     */
    virtual bool is_initialized() const = 0;

    // ========================================================================
    // Model Loading
    // ========================================================================

    /**
     * @brief Load model from file
     * @param path Path to model file (e.g., .onnx file)
     * @return Success or error code
     */
    virtual InferenceResult load_model(const std::string& path) = 0;

    /**
     * @brief Load model from memory
     * @param data Model data as bytes
     * @return Success or error code
     */
    virtual InferenceResult load_model_from_memory(const std::vector<UInt8>& data) = 0;

    /**
     * @brief Check if model is loaded
     */
    virtual bool is_model_loaded() const = 0;

    // ========================================================================
    // Model Introspection
    // ========================================================================

    /**
     * @brief Get information about model inputs
     * @return Vector of input tensor metadata
     */
    virtual std::vector<TensorInfo> get_input_info() const = 0;

    /**
     * @brief Get information about model outputs
     * @return Vector of output tensor metadata
     */
    virtual std::vector<TensorInfo> get_output_info() const = 0;

    // ========================================================================
    // Inference
    // ========================================================================

    /**
     * @brief Run synchronous inference
     * @param inputs Input tensors (must match model inputs)
     * @param outputs Output tensors (will be populated)
     * @return Success or error code
     */
    virtual InferenceResult run(const std::vector<Tensor>& inputs,
                                 std::vector<Tensor>& outputs) = 0;

    /**
     * @brief Run asynchronous inference
     * @param inputs Input tensors (must match model inputs)
     * @param callback Callback invoked with result and outputs
     * @return Success if started, error code otherwise
     */
    virtual InferenceResult run_async(
        const std::vector<Tensor>& inputs,
        std::function<void(InferenceResult, std::vector<Tensor>)> callback) = 0;

    // ========================================================================
    // Statistics
    // ========================================================================

    /**
     * @brief Get inference statistics
     */
    virtual InferenceStats get_stats() const = 0;
};

// ============================================================================
// Inference Session Implementation
// ============================================================================

/**
 * @brief Main ONNX Runtime-based inference session
 *
 * Provides complete inference functionality using ONNX Runtime as the backend.
 * Supports multiple execution providers, async execution, and comprehensive
 * performance tracking.
 */
class InferenceSession : public IInferenceSession {
public:
    /**
     * @brief Construct inference session with configuration
     * @param config Inference configuration
     */
    explicit InferenceSession(const InferenceConfig& config = InferenceConfig::default_config());

    ~InferenceSession();

    // Non-copyable, movable
    InferenceSession(const InferenceSession&) = delete;
    InferenceSession& operator=(const InferenceSession&) = delete;
    InferenceSession(InferenceSession&&) noexcept;
    InferenceSession& operator=(InferenceSession&&) noexcept;

    // ========================================================================
    // IInferenceSession Implementation
    // ========================================================================

    InferenceResult initialize() override;
    InferenceResult shutdown() override;
    bool is_initialized() const override;

    InferenceResult load_model(const std::string& path) override;
    InferenceResult load_model_from_memory(const std::vector<UInt8>& data) override;
    bool is_model_loaded() const override;

    std::vector<TensorInfo> get_input_info() const override;
    std::vector<TensorInfo> get_output_info() const override;

    InferenceResult run(const std::vector<Tensor>& inputs,
                        std::vector<Tensor>& outputs) override;

    InferenceResult run_async(
        const std::vector<Tensor>& inputs,
        std::function<void(InferenceResult, std::vector<Tensor>)> callback) override;

    InferenceStats get_stats() const override;

    // ========================================================================
    // Additional Methods
    // ========================================================================

    /**
     * @brief Get current configuration
     */
    const InferenceConfig& get_config() const noexcept;

    /**
     * @brief Warm up the model by running dummy inferences
     * @param iterations Number of warmup iterations
     * @return Success or error code
     *
     * Warmup helps stabilize performance by pre-allocating buffers and
     * compiling optimizations. Recommended before benchmarking.
     */
    InferenceResult warmup(UInt32 iterations = 3);

    /**
     * @brief Change execution provider (requires reloading model)
     * @param provider New execution provider
     * @return Success or error code
     */
    InferenceResult set_provider(ExecutionProvider provider);

    /**
     * @brief Get current execution provider
     */
    ExecutionProvider get_provider() const noexcept;

    /**
     * @brief Reset statistics counters
     */
    void reset_stats();

    /**
     * @brief Get detailed profiling information (if enabled)
     * @return Profiling data as string, or empty if profiling disabled
     */
    std::string get_profiling_data() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

// ============================================================================
// Factory Functions
// ============================================================================

/**
 * @brief Create inference session with configuration
 * @param config Inference configuration
 * @return Unique pointer to inference session
 */
inline std::unique_ptr<InferenceSession> create_inference_session(
    const InferenceConfig& config = InferenceConfig::default_config()) {
    return std::make_unique<InferenceSession>(config);
}

/**
 * @brief Check if an execution provider is available
 * @param provider Provider to check
 * @return True if provider is available on this system
 *
 * Use this to check provider availability before configuring the session.
 * For example, CUDA provider requires NVIDIA GPU and CUDA libraries.
 */
bool is_provider_available(ExecutionProvider provider);

/**
 * @brief Get list of all available execution providers
 * @return Vector of available providers
 */
std::vector<ExecutionProvider> get_available_providers();

// ============================================================================
// Helper Functions
// ============================================================================

/**
 * @brief Create tensor from typed data
 * @tparam T Element type
 * @param name Tensor name
 * @param shape Tensor shape
 * @param data Vector of values
 * @return Constructed tensor, or empty tensor if type mismatch
 */
template<typename T>
inline Tensor create_tensor(const std::string& name,
                            const TensorShape& shape,
                            const std::vector<T>& data) {
    DataType dtype;
    if constexpr (std::is_same_v<T, float>) {
        dtype = DataType::Float32;
    } else if constexpr (std::is_same_v<T, double>) {
        dtype = DataType::Float64;
    } else if constexpr (std::is_same_v<T, Int32>) {
        dtype = DataType::Int32;
    } else if constexpr (std::is_same_v<T, Int64>) {
        dtype = DataType::Int64;
    } else if constexpr (std::is_same_v<T, UInt8>) {
        dtype = DataType::UInt8;
    } else if constexpr (std::is_same_v<T, bool>) {
        dtype = DataType::Bool;
    } else {
        // Unsupported type
        return Tensor{};
    }

    Tensor tensor(name, dtype, shape);
    tensor.set_data(data);
    return tensor;
}

/**
 * @brief Validate that tensors match expected info
 * @param tensors Tensors to validate
 * @param expected Expected tensor metadata
 * @param check_data Whether to check that data is populated
 * @return True if all tensors valid, false otherwise
 */
inline bool validate_tensors(const std::vector<Tensor>& tensors,
                             const std::vector<TensorInfo>& expected,
                             bool check_data = true) {
    if (tensors.size() != expected.size()) {
        return false;
    }

    for (UInt64 i = 0; i < tensors.size(); ++i) {
        const auto& tensor = tensors[i];
        const auto& info = expected[i];

        // Check name match
        if (tensor.name != info.name) {
            return false;
        }

        // Check data type match
        if (tensor.data_type != info.data_type) {
            return false;
        }

        // Check shape compatibility (allow dynamic shapes)
        if (!info.shape.is_dynamic()) {
            if (tensor.shape != info.shape) {
                return false;
            }
        }

        // Check data populated if required
        if (check_data && tensor.data.empty()) {
            return false;
        }
    }

    return true;
}

/**
 * @brief Calculate total memory used by tensors
 * @param tensors Tensors to measure
 * @return Total size in bytes
 */
inline UInt64 calculate_tensor_memory(const std::vector<Tensor>& tensors) {
    UInt64 total = 0;
    for (const auto& tensor : tensors) {
        total += tensor.size_in_bytes();
    }
    return total;
}

/**
 * @brief Format tensor shape as string
 * @param shape Tensor shape
 * @return String representation (e.g., "[1, 3, 224, 224]")
 */
inline std::string format_shape(const TensorShape& shape) {
    std::string result = "[";
    for (UInt64 i = 0; i < shape.dimensions.size(); ++i) {
        if (i > 0) {
            result += ", ";
        }
        if (shape.dimensions[i] < 0) {
            result += "?";
        } else {
            result += std::to_string(shape.dimensions[i]);
        }
    }
    result += "]";
    return result;
}

/**
 * @brief Format tensor info as string for logging
 * @param info Tensor information
 * @return Human-readable string
 */
inline std::string format_tensor_info(const TensorInfo& info) {
    std::string result = info.name;
    result += ": ";
    result += data_type_to_string(info.data_type);
    result += " ";
    result += format_shape(info.shape);
    return result;
}

/**
 * @brief Get recommended batch size based on available memory
 * @param model_input_size Size of one input in bytes
 * @param available_memory Available memory in bytes
 * @return Recommended batch size
 */
inline UInt32 recommend_batch_size(UInt64 model_input_size,
                                    UInt64 available_memory) {
    if (model_input_size == 0) {
        return 1;
    }

    // Use 80% of available memory to leave headroom
    UInt64 usable_memory = (available_memory * 80) / 100;
    UInt64 batch_size = usable_memory / model_input_size;

    // Clamp to reasonable range
    if (batch_size < 1) {
        return 1;
    }
    if (batch_size > 1024) {
        return 1024;
    }

    return static_cast<UInt32>(batch_size);
}

} // namespace jaguar::ml
