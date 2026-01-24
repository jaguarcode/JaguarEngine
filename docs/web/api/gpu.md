# GPU Compute API Reference

GPU Compute module (`jaguar_gpu`) provides hardware-accelerated computation through a unified abstraction layer supporting CUDA, OpenCL, Metal, and Vulkan backends.

## Overview

The GPU module enables:
- **Backend Abstraction**: Single API for multiple GPU backends
- **Kernel Management**: Launch and manage compute kernels
- **Memory Management**: Efficient buffer allocation and transfer
- **Device Queries**: Capability detection and device selection

---

## Compute Backend

### Header

```cpp
#include <jaguar/gpu/compute_backend.h>
```

### Backend Types

```cpp
namespace jaguar::gpu {

// Supported compute backends
enum class BackendType {
    CPU,            // Fallback CPU implementation
    CUDA,           // NVIDIA CUDA
    OpenCL,         // OpenCL (cross-platform)
    Metal,          // Apple Metal
    Vulkan          // Vulkan Compute
};

// Device types
enum class DeviceType {
    Unknown,
    CPU,
    GPU,
    Accelerator,
    Custom
};

// Memory types
enum class MemoryType {
    DeviceLocal,    // GPU memory only
    HostVisible,    // CPU-accessible GPU memory
    HostCached,     // Cached host memory
    Shared          // Unified memory (CPU + GPU)
};

// Memory access patterns
enum class MemoryAccess {
    ReadOnly,
    WriteOnly,
    ReadWrite
};

// Kernel argument types
enum class ArgType {
    Buffer,
    Scalar,
    LocalMemory,
    Image,
    Sampler
};

}  // namespace jaguar::gpu
```

### Handle Types

```cpp
namespace jaguar::gpu {

// Buffer handle (opaque reference to GPU memory)
struct BufferHandle {
    uint64_t id = 0;
    bool is_valid() const { return id != 0; }
    bool operator==(const BufferHandle& other) const { return id == other.id; }
};

// Stream/queue handle for async operations
struct StreamHandle {
    uint64_t id = 0;
    bool is_valid() const { return id != 0; }
};

// Event handle for synchronization
struct EventHandle {
    uint64_t id = 0;
    bool is_valid() const { return id != 0; }
};

// Kernel handle
struct KernelHandle {
    uint64_t id = 0;
    bool is_valid() const { return id != 0; }
};

}  // namespace jaguar::gpu
```

### Result Types

```cpp
namespace jaguar::gpu {

// Backend operation result
struct BackendResult {
    bool success;
    std::string error_message;
    int error_code = 0;
};

// Device capabilities
struct DeviceCapabilities {
    std::string device_name;
    std::string vendor;
    std::string driver_version;
    DeviceType device_type;

    // Compute capabilities
    uint32_t compute_units;
    uint32_t max_work_group_size;
    std::array<uint32_t, 3> max_work_item_sizes;
    uint32_t max_work_item_dimensions;
    uint32_t warp_size;                     // CUDA warp / AMD wavefront

    // Memory capabilities
    uint64_t global_memory_size;
    uint64_t local_memory_size;             // Shared memory per block
    uint64_t constant_memory_size;
    uint64_t max_buffer_size;
    uint32_t memory_bus_width;
    uint64_t memory_bandwidth;              // bytes/sec

    // Features
    bool supports_double_precision;
    bool supports_half_precision;
    bool supports_atomics;
    bool supports_images;
    bool supports_unified_memory;

    // Clock speeds (MHz)
    uint32_t clock_frequency;
    uint32_t memory_clock_frequency;
};

// Launch configuration
struct LaunchConfig {
    std::array<uint32_t, 3> global_size = {1, 1, 1};    // Total work items
    std::array<uint32_t, 3> local_size = {1, 1, 1};     // Work group size
    uint32_t shared_memory_size = 0;                     // Dynamic shared memory
    StreamHandle stream;                                 // Execution stream

    // Helper constructors
    static LaunchConfig linear(uint32_t total_items, uint32_t group_size = 256);
    static LaunchConfig grid_2d(uint32_t width, uint32_t height,
                               uint32_t block_x = 16, uint32_t block_y = 16);
    static LaunchConfig grid_3d(uint32_t x, uint32_t y, uint32_t z,
                               uint32_t block_x = 8, uint32_t block_y = 8,
                               uint32_t block_z = 8);
};

// Kernel argument
struct KernelArg {
    ArgType type;
    BufferHandle buffer;            // For buffer arguments
    const void* scalar_ptr;         // For scalar arguments
    size_t scalar_size;
    size_t local_memory_size;       // For local memory arguments

    // Factory methods
    static KernelArg from_buffer(BufferHandle buffer);
    static KernelArg from_scalar(const void* ptr, size_t size);
    template<typename T>
    static KernelArg from_value(const T& value);
    static KernelArg local_memory(size_t size);
};

}  // namespace jaguar::gpu
```

### IComputeBackend Interface

```cpp
namespace jaguar::gpu {

class IComputeBackend {
public:
    virtual ~IComputeBackend() = default;

    // Lifecycle
    virtual BackendResult initialize() = 0;
    virtual void shutdown() = 0;
    virtual BackendType get_type() const = 0;

    // Device management
    virtual size_t get_device_count() const = 0;
    virtual DeviceCapabilities get_device_capabilities(size_t device_index = 0) const = 0;
    virtual BackendResult select_device(size_t device_index) = 0;
    virtual size_t get_selected_device() const = 0;

    // Buffer management
    virtual BufferHandle create_buffer(size_t size, MemoryType type = MemoryType::DeviceLocal) = 0;
    virtual BufferHandle create_buffer(size_t size, const void* initial_data,
                                       MemoryType type = MemoryType::DeviceLocal) = 0;
    virtual void destroy_buffer(BufferHandle buffer) = 0;
    virtual size_t get_buffer_size(BufferHandle buffer) const = 0;

    // Memory transfers
    virtual BackendResult copy_to_device(BufferHandle dst, const void* src, size_t size) = 0;
    virtual BackendResult copy_to_device_async(BufferHandle dst, const void* src, size_t size,
                                               StreamHandle stream) = 0;
    virtual BackendResult copy_from_device(void* dst, BufferHandle src, size_t size) = 0;
    virtual BackendResult copy_from_device_async(void* dst, BufferHandle src, size_t size,
                                                 StreamHandle stream) = 0;
    virtual BackendResult copy_device_to_device(BufferHandle dst, BufferHandle src, size_t size) = 0;

    // Memory mapping (for HostVisible buffers)
    virtual void* map_buffer(BufferHandle buffer, MemoryAccess access) = 0;
    virtual void unmap_buffer(BufferHandle buffer) = 0;

    // Kernel management
    virtual KernelHandle load_kernel(const std::string& source,
                                     const std::string& kernel_name,
                                     const std::string& compile_options = "") = 0;
    virtual KernelHandle load_kernel_from_binary(const std::vector<uint8_t>& binary,
                                                 const std::string& kernel_name) = 0;
    virtual void destroy_kernel(KernelHandle kernel) = 0;

    // Kernel execution
    virtual BackendResult launch_kernel(KernelHandle kernel,
                                        const LaunchConfig& config,
                                        const std::vector<KernelArg>& args) = 0;

    // Stream management
    virtual StreamHandle create_stream() = 0;
    virtual void destroy_stream(StreamHandle stream) = 0;
    virtual BackendResult synchronize_stream(StreamHandle stream) = 0;
    virtual BackendResult synchronize_device() = 0;

    // Event management
    virtual EventHandle create_event() = 0;
    virtual void destroy_event(EventHandle event) = 0;
    virtual BackendResult record_event(EventHandle event, StreamHandle stream) = 0;
    virtual BackendResult wait_for_event(EventHandle event, StreamHandle stream) = 0;
    virtual bool is_event_complete(EventHandle event) = 0;
    virtual float get_elapsed_time(EventHandle start, EventHandle end) = 0;  // milliseconds

    // Utilities
    virtual size_t get_available_memory() const = 0;
    virtual size_t get_total_memory() const = 0;
};

}  // namespace jaguar::gpu
```

### IKernel Interface

```cpp
namespace jaguar::gpu {

// Higher-level kernel wrapper
class IKernel {
public:
    virtual ~IKernel() = default;

    // Get kernel name
    virtual std::string get_name() const = 0;

    // Get argument information
    virtual size_t get_num_arguments() const = 0;
    virtual ArgType get_argument_type(size_t index) const = 0;

    // Set arguments
    virtual void set_argument(size_t index, BufferHandle buffer) = 0;
    virtual void set_argument(size_t index, const void* data, size_t size) = 0;
    template<typename T>
    void set_argument(size_t index, const T& value) {
        set_argument(index, &value, sizeof(T));
    }
    virtual void set_local_memory_argument(size_t index, size_t size) = 0;

    // Launch
    virtual BackendResult launch(const LaunchConfig& config) = 0;

    // Get preferred work group size
    virtual size_t get_preferred_work_group_size() const = 0;

    // Get local memory usage
    virtual size_t get_local_memory_usage() const = 0;
};

}  // namespace jaguar::gpu
```

### Backend Factory

```cpp
namespace jaguar::gpu {

class BackendFactory {
public:
    // Create specific backend
    static std::unique_ptr<IComputeBackend> create(BackendType type);

    // Create best available backend (prioritizes GPU over CPU)
    static std::unique_ptr<IComputeBackend> create_best_available();

    // Query available backends
    static std::vector<BackendType> get_available_backends();

    // Check if specific backend is available
    static bool is_backend_available(BackendType type);
};

}  // namespace jaguar::gpu
```

---

## Usage Examples

### Basic Buffer Operations

```cpp
#include <jaguar/gpu/compute_backend.h>

using namespace jaguar::gpu;

// Create best available backend
auto backend = BackendFactory::create_best_available();
backend->initialize();

// Print device info
auto caps = backend->get_device_capabilities();
std::cout << "Device: " << caps.device_name << "\n";
std::cout << "Memory: " << caps.global_memory_size / (1024*1024) << " MB\n";
std::cout << "Compute units: " << caps.compute_units << "\n";

// Create buffers
const size_t N = 1024 * 1024;
std::vector<float> host_input(N, 1.0f);
std::vector<float> host_output(N);

auto input_buffer = backend->create_buffer(N * sizeof(float), host_input.data());
auto output_buffer = backend->create_buffer(N * sizeof(float));

// ... compute ...

// Read results
backend->copy_from_device(host_output.data(), output_buffer, N * sizeof(float));

// Cleanup
backend->destroy_buffer(input_buffer);
backend->destroy_buffer(output_buffer);
backend->shutdown();
```

### Kernel Compilation and Launch

```cpp
#include <jaguar/gpu/compute_backend.h>

using namespace jaguar::gpu;

auto backend = BackendFactory::create(BackendType::CUDA);
backend->initialize();

// Kernel source (CUDA)
const char* kernel_source = R"(
extern "C" __global__ void vector_add(
    const float* a,
    const float* b,
    float* c,
    int n
) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx < n) {
        c[idx] = a[idx] + b[idx];
    }
}
)";

// Compile kernel
auto kernel = backend->load_kernel(kernel_source, "vector_add", "-O3");

// Create buffers
const int N = 1000000;
std::vector<float> a(N, 1.0f), b(N, 2.0f), c(N);

auto buf_a = backend->create_buffer(N * sizeof(float), a.data());
auto buf_b = backend->create_buffer(N * sizeof(float), b.data());
auto buf_c = backend->create_buffer(N * sizeof(float));

// Set kernel arguments
std::vector<KernelArg> args = {
    KernelArg::from_buffer(buf_a),
    KernelArg::from_buffer(buf_b),
    KernelArg::from_buffer(buf_c),
    KernelArg::from_value(N)
};

// Launch configuration
auto config = LaunchConfig::linear(N, 256);

// Launch kernel
auto result = backend->launch_kernel(kernel, config, args);
if (!result.success) {
    std::cerr << "Kernel launch failed: " << result.error_message << "\n";
}

// Synchronize and read results
backend->synchronize_device();
backend->copy_from_device(c.data(), buf_c, N * sizeof(float));

// Verify
for (int i = 0; i < 10; ++i) {
    std::cout << "c[" << i << "] = " << c[i] << "\n";  // Should be 3.0
}

// Cleanup
backend->destroy_buffer(buf_a);
backend->destroy_buffer(buf_b);
backend->destroy_buffer(buf_c);
backend->destroy_kernel(kernel);
backend->shutdown();
```

### Async Operations with Streams

```cpp
#include <jaguar/gpu/compute_backend.h>

using namespace jaguar::gpu;

auto backend = BackendFactory::create_best_available();
backend->initialize();

// Create multiple streams for concurrent operations
auto stream1 = backend->create_stream();
auto stream2 = backend->create_stream();

// Create events for timing
auto start_event = backend->create_event();
auto end_event = backend->create_event();

const size_t N = 10000000;
std::vector<float> host_a(N), host_b(N);

auto buf_a = backend->create_buffer(N * sizeof(float), MemoryType::DeviceLocal);
auto buf_b = backend->create_buffer(N * sizeof(float), MemoryType::DeviceLocal);

// Record start
backend->record_event(start_event, stream1);

// Async copy on stream1
backend->copy_to_device_async(buf_a, host_a.data(), N * sizeof(float), stream1);

// Async copy on stream2 (runs concurrently)
backend->copy_to_device_async(buf_b, host_b.data(), N * sizeof(float), stream2);

// Launch kernels (after respective copies complete)
// ... kernel launches ...

// Record end
backend->record_event(end_event, stream1);

// Synchronize
backend->synchronize_stream(stream1);
backend->synchronize_stream(stream2);

// Get elapsed time
float elapsed_ms = backend->get_elapsed_time(start_event, end_event);
std::cout << "Elapsed time: " << elapsed_ms << " ms\n";

// Cleanup
backend->destroy_event(start_event);
backend->destroy_event(end_event);
backend->destroy_stream(stream1);
backend->destroy_stream(stream2);
backend->destroy_buffer(buf_a);
backend->destroy_buffer(buf_b);
backend->shutdown();
```

### Cross-Platform Kernel

```cpp
#include <jaguar/gpu/compute_backend.h>

using namespace jaguar::gpu;

// OpenCL kernel (portable)
const char* opencl_kernel = R"(
__kernel void physics_update(
    __global float4* positions,
    __global float4* velocities,
    __global const float4* forces,
    __global const float* masses,
    float dt,
    int num_entities
) {
    int gid = get_global_id(0);
    if (gid >= num_entities) return;

    float mass = masses[gid];
    float4 acceleration = forces[gid] / mass;

    velocities[gid] += acceleration * dt;
    positions[gid] += velocities[gid] * dt;
}
)";

// CUDA kernel (NVIDIA-optimized)
const char* cuda_kernel = R"(
extern "C" __global__ void physics_update(
    float4* positions,
    float4* velocities,
    const float4* forces,
    const float* masses,
    float dt,
    int num_entities
) {
    int gid = blockIdx.x * blockDim.x + threadIdx.x;
    if (gid >= num_entities) return;

    float mass = masses[gid];
    float4 acceleration = make_float4(
        forces[gid].x / mass,
        forces[gid].y / mass,
        forces[gid].z / mass,
        0.0f
    );

    velocities[gid].x += acceleration.x * dt;
    velocities[gid].y += acceleration.y * dt;
    velocities[gid].z += acceleration.z * dt;

    positions[gid].x += velocities[gid].x * dt;
    positions[gid].y += velocities[gid].y * dt;
    positions[gid].z += velocities[gid].z * dt;
}
)";

// Select appropriate kernel based on backend
auto backend = BackendFactory::create_best_available();
backend->initialize();

const char* kernel_source = nullptr;
switch (backend->get_type()) {
    case BackendType::CUDA:
        kernel_source = cuda_kernel;
        break;
    case BackendType::OpenCL:
    case BackendType::CPU:
    default:
        kernel_source = opencl_kernel;
        break;
}

auto kernel = backend->load_kernel(kernel_source, "physics_update");

// Use kernel for physics simulation
// ...
```

### Memory-Mapped I/O

```cpp
#include <jaguar/gpu/compute_backend.h>

using namespace jaguar::gpu;

auto backend = BackendFactory::create_best_available();
backend->initialize();

// Create host-visible buffer
const size_t N = 1024;
auto buffer = backend->create_buffer(
    N * sizeof(float),
    MemoryType::HostVisible
);

// Map buffer for CPU access
float* mapped = static_cast<float*>(
    backend->map_buffer(buffer, MemoryAccess::WriteOnly)
);

// Write directly to GPU memory
for (size_t i = 0; i < N; ++i) {
    mapped[i] = static_cast<float>(i);
}

// Unmap before GPU use
backend->unmap_buffer(buffer);

// ... use buffer in GPU kernel ...

// Map again to read results
mapped = static_cast<float*>(
    backend->map_buffer(buffer, MemoryAccess::ReadOnly)
);

for (size_t i = 0; i < 10; ++i) {
    std::cout << "buffer[" << i << "] = " << mapped[i] << "\n";
}

backend->unmap_buffer(buffer);
backend->destroy_buffer(buffer);
backend->shutdown();
```

### Physics Simulation on GPU

```cpp
#include <jaguar/gpu/compute_backend.h>
#include <jaguar/jaguar.h>

using namespace jaguar;
using namespace jaguar::gpu;

class GPUPhysicsEngine {
    std::unique_ptr<IComputeBackend> backend_;
    KernelHandle integration_kernel_;
    KernelHandle force_kernel_;

    BufferHandle positions_;
    BufferHandle velocities_;
    BufferHandle forces_;
    BufferHandle masses_;

    size_t num_entities_ = 0;

public:
    void initialize(size_t max_entities) {
        backend_ = BackendFactory::create_best_available();
        backend_->initialize();

        // Allocate buffers for max entities
        size_t vec4_size = max_entities * 4 * sizeof(float);
        size_t scalar_size = max_entities * sizeof(float);

        positions_ = backend_->create_buffer(vec4_size);
        velocities_ = backend_->create_buffer(vec4_size);
        forces_ = backend_->create_buffer(vec4_size);
        masses_ = backend_->create_buffer(scalar_size);

        // Load kernels
        integration_kernel_ = backend_->load_kernel(
            integration_kernel_source, "integrate");
        force_kernel_ = backend_->load_kernel(
            force_kernel_source, "compute_forces");
    }

    void update_entities(const std::vector<physics::EntityState>& states) {
        num_entities_ = states.size();

        // Pack data
        std::vector<float> pos_data(num_entities_ * 4);
        std::vector<float> vel_data(num_entities_ * 4);
        std::vector<float> mass_data(num_entities_);

        for (size_t i = 0; i < num_entities_; ++i) {
            pos_data[i*4 + 0] = states[i].position.x;
            pos_data[i*4 + 1] = states[i].position.y;
            pos_data[i*4 + 2] = states[i].position.z;
            pos_data[i*4 + 3] = 0.0f;

            vel_data[i*4 + 0] = states[i].velocity.x;
            vel_data[i*4 + 1] = states[i].velocity.y;
            vel_data[i*4 + 2] = states[i].velocity.z;
            vel_data[i*4 + 3] = 0.0f;

            mass_data[i] = states[i].mass;
        }

        // Upload to GPU
        backend_->copy_to_device(positions_, pos_data.data(),
                                pos_data.size() * sizeof(float));
        backend_->copy_to_device(velocities_, vel_data.data(),
                                vel_data.size() * sizeof(float));
        backend_->copy_to_device(masses_, mass_data.data(),
                                mass_data.size() * sizeof(float));
    }

    void step(float dt) {
        auto config = LaunchConfig::linear(num_entities_, 256);

        // Compute forces
        std::vector<KernelArg> force_args = {
            KernelArg::from_buffer(positions_),
            KernelArg::from_buffer(velocities_),
            KernelArg::from_buffer(forces_),
            KernelArg::from_buffer(masses_),
            KernelArg::from_value(static_cast<int>(num_entities_))
        };
        backend_->launch_kernel(force_kernel_, config, force_args);

        // Integrate
        std::vector<KernelArg> int_args = {
            KernelArg::from_buffer(positions_),
            KernelArg::from_buffer(velocities_),
            KernelArg::from_buffer(forces_),
            KernelArg::from_buffer(masses_),
            KernelArg::from_value(dt),
            KernelArg::from_value(static_cast<int>(num_entities_))
        };
        backend_->launch_kernel(integration_kernel_, config, int_args);

        backend_->synchronize_device();
    }

    void read_results(std::vector<physics::EntityState>& states) {
        std::vector<float> pos_data(num_entities_ * 4);
        std::vector<float> vel_data(num_entities_ * 4);

        backend_->copy_from_device(pos_data.data(), positions_,
                                  pos_data.size() * sizeof(float));
        backend_->copy_from_device(vel_data.data(), velocities_,
                                  vel_data.size() * sizeof(float));

        for (size_t i = 0; i < num_entities_; ++i) {
            states[i].position.x = pos_data[i*4 + 0];
            states[i].position.y = pos_data[i*4 + 1];
            states[i].position.z = pos_data[i*4 + 2];

            states[i].velocity.x = vel_data[i*4 + 0];
            states[i].velocity.y = vel_data[i*4 + 1];
            states[i].velocity.z = vel_data[i*4 + 2];
        }
    }

    void shutdown() {
        backend_->destroy_kernel(integration_kernel_);
        backend_->destroy_kernel(force_kernel_);
        backend_->destroy_buffer(positions_);
        backend_->destroy_buffer(velocities_);
        backend_->destroy_buffer(forces_);
        backend_->destroy_buffer(masses_);
        backend_->shutdown();
    }
};
```

---

## See Also

- [Architecture](../advanced/architecture.md) - System architecture overview
- [Machine Learning API](ml.md) - GPU-accelerated ML inference
- [Configuration](configuration.md) - Engine configuration
