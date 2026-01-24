# GPU Compute Module Documentation

The GPU module provides a unified abstraction layer for GPU-accelerated physics computation across multiple backends (CUDA, OpenCL, Metal, Vulkan). It enables high-performance batch processing of physics workloads with automatic CPU/GPU workload distribution.

## Headers

| Header | Purpose |
|--------|---------|
| `jaguar/gpu/compute_backend.h` | Backend abstraction, device management, kernel dispatch |
| `jaguar/gpu/physics_kernels.h` | GPU-accelerated physics computation kernels |
| `jaguar/gpu/hybrid_physics.h` | Automatic CPU/GPU workload distribution |
| `jaguar/gpu/memory_pool.h` | GPU memory pool management |

## Backend Types (`compute_backend.h`)

### Supported Backends

```cpp
enum class BackendType : UInt8 {
    CPU     = 0,    // CPU reference implementation (always available)
    CUDA    = 1,    // NVIDIA CUDA
    OpenCL  = 2,    // OpenCL (cross-platform)
    Metal   = 3,    // Apple Metal
    Vulkan  = 4,    // Vulkan Compute
    Auto    = 255   // Automatic selection (best available)
};
```

### Memory Types

```cpp
enum class MemoryType : UInt8 {
    DeviceLocal     = 0,    // GPU-only memory (fastest for compute)
    HostVisible     = 1,    // CPU-accessible GPU memory
    HostCached      = 2,    // CPU-cached memory (for frequent reads)
    Shared          = 3,    // Unified memory (CPU/GPU shared)
    Staging         = 4     // Transfer buffer
};
```

## Backend Factory

```cpp
// Create specific backend
auto backend = BackendFactory::create(BackendType::CUDA);

// Create best available backend (CUDA > Metal > OpenCL > CPU)
auto backend = BackendFactory::create_best_available();

// Check available backends
std::vector<BackendType> available = BackendFactory::available_backends();
bool cuda_available = BackendFactory::is_available(BackendType::CUDA);
```

## IComputeBackend Interface

### Lifecycle

```cpp
class IComputeBackend {
public:
    // Initialize backend with options
    virtual BackendResult initialize(const BackendOptions& options = {}) = 0;
    virtual void shutdown() = 0;
    virtual bool is_initialized() const = 0;
    virtual BackendType type() const = 0;
};
```

### Memory Management

```cpp
// Allocate GPU buffer
BufferHandle positions = backend->allocate(
    sizeof(Vec3) * entity_count,
    MemoryType::DeviceLocal,
    MemoryAccess::ReadWrite
);

// Upload data to GPU
backend->upload(positions, host_positions.data(), sizeof(Vec3) * count);

// Download data from GPU
backend->download(positions, host_positions.data(), sizeof(Vec3) * count);

// Copy between buffers
backend->copy(src_buffer, dst_buffer, size);

// Map buffer for direct access
void* ptr = backend->map(positions, MemoryAccess::ReadWrite);
// ... modify data ...
backend->unmap(positions);

// Free buffer
backend->free(positions);
```

### Kernel Management

```cpp
// Create kernel from source
auto kernel = backend->create_kernel(
    "integrate_positions",
    kernel_source_code,
    "-DDEBUG"  // Compiler options
);

// Set kernel arguments
kernel->set_arg(0, KernelArg::Buffer(positions));
kernel->set_arg(1, KernelArg::Buffer(velocities));
kernel->set_arg(2, KernelArg::Float(static_cast<float>(dt)));
kernel->set_arg(3, KernelArg::UInt(entity_count));

// Dispatch kernel
LaunchConfig config = LaunchConfig::Linear(entity_count, 256);
backend->dispatch(kernel.get(), config);

// Synchronize
backend->synchronize();
```

### Launch Configuration

```cpp
// 1D linear dispatch
LaunchConfig config1d = LaunchConfig::Linear(total_threads, block_size);

// 2D grid dispatch
LaunchConfig config2d = LaunchConfig::Grid2D(width, height, 16, 16);

// 3D grid dispatch
LaunchConfig config3d = LaunchConfig::Grid3D(x, y, z, 8, 8, 8);
```

## Physics Kernels (`physics_kernels.h`)

### PhysicsKernelManager

```cpp
auto backend = BackendFactory::create_best_available();
PhysicsKernelManager kernels(backend.get());

PhysicsKernelConfig config;
config.use_symplectic = true;
config.gravity_z = -9.81;
kernels.initialize(config);
```

### Integration Kernels

```cpp
// Allocate buffers
BufferHandle positions = kernels.allocate_vec3_buffer(entity_count);
BufferHandle velocities = kernels.allocate_vec3_buffer(entity_count);
BufferHandle forces = kernels.allocate_vec3_buffer(entity_count);
BufferHandle masses = kernels.allocate_scalar_buffer(entity_count);

// Upload initial state
backend->upload(positions, host_positions.data(), sizeof(Vec3) * count);
backend->upload(velocities, host_velocities.data(), sizeof(Vec3) * count);

// Integrate positions: positions += velocities * dt
kernels.integrate_positions(positions, velocities, dt, entity_count);

// Integrate velocities: velocities += (forces / masses + gravity) * dt
kernels.integrate_velocities(velocities, forces, masses, dt, entity_count);

// Integrate orientations (quaternion integration)
kernels.integrate_orientations(orientations, angular_velocities, dt, entity_count);

// Full symplectic step
RigidBodyStateBuffers state;
state.positions = positions;
state.velocities = velocities;
state.forces = forces;
state.masses = masses;
state.count = entity_count;
kernels.integrate_symplectic(state, dt);
```

### Collision Detection

```cpp
// Allocate collision buffers
BufferHandle aabbs = kernels.allocate_aabb_buffer(entity_count);
BufferHandle pairs = kernels.allocate_collision_pair_buffer(max_pairs);
BufferHandle extents = kernels.allocate_vec3_buffer(entity_count);

// Update AABBs from positions and extents
kernels.update_aabbs(positions, extents, aabbs, entity_count);

// Broad-phase collision detection
SizeT pair_count = 0;
kernels.collision_broad_phase(aabbs, entity_count, pairs, &pair_count);

// Sort AABBs for sweep-and-prune
kernels.sort_aabbs(aabbs, entity_count, 0);  // Sort along X axis
```

### Aerodynamics

```cpp
BufferHandle aero_inputs = backend->allocate(sizeof(AeroInput) * aircraft_count);
BufferHandle aero_outputs = backend->allocate(sizeof(AeroOutput) * aircraft_count);

// Set aerodynamic coefficient tables
kernels.set_aero_model(model_id, alpha_table, cl_table, cd_table, cm_table);

// Compute aerodynamic forces
kernels.compute_aero_forces(aero_inputs, aero_outputs, aircraft_count);
```

### Terrain Sampling

```cpp
// Set terrain heightmap
kernels.set_terrain_heightmap(
    heightmap_buffer,
    width, height,
    origin_x, origin_y,
    scale
);

// Batch sample terrain
BufferHandle requests = backend->allocate(sizeof(TerrainSampleRequest) * count);
BufferHandle results = backend->allocate(sizeof(TerrainSampleResult) * count);
kernels.sample_terrain_batch(requests, results, count);
```

### Ocean Waves

```cpp
BufferHandle spectrum = backend->allocate(sizeof(float) * 2 * resolution * resolution);

// Compute wave spectrum
kernels.compute_wave_spectrum(
    spectrum,
    resolution,       // Power of 2 (e.g., 256)
    wind_speed,       // m/s
    wind_direction,   // radians
    simulation_time
);

// Sample wave heights at positions
kernels.sample_wave_heights(
    positions,        // float2 * count (x, z)
    heights,          // Output heights
    normals,          // Output normals (optional)
    spectrum,
    resolution,
    patch_size,
    sample_count
);
```

### Atmosphere

```cpp
// Compute atmospheric properties at altitudes
kernels.compute_atmosphere_density(
    altitudes,        // Input altitudes
    densities,        // Output densities
    temperatures,     // Output temperatures (optional)
    pressures,        // Output pressures (optional)
    count
);

// Batch compute for aircraft
kernels.compute_atmosphere_batch(
    positions,        // float3 * count
    air_density,      // Output
    speed_of_sound,   // Output
    temperature,      // Output
    count
);
```

## Hybrid Physics System (`hybrid_physics.h`)

### Automatic Workload Distribution

```cpp
HybridPhysicsConfig config;
config.strategy = RoutingStrategy::Adaptive;
config.gpu_threshold = 200;  // Use GPU when > 200 entities
config.transfer_mode = TransferMode::Asynchronous;
config.enable_dynamic_adjustment = true;

HybridPhysicsSystem hybrid;
hybrid.initialize(config, &entity_storage);

// Each frame - automatically routes to CPU or GPU
hybrid.step(dt);

// Force specific path
hybrid.force_next_path(true);  // Force GPU
hybrid.force_next_path(false); // Force CPU
```

### Routing Strategies

```cpp
enum class RoutingStrategy : UInt8 {
    ThresholdBased,     // Route based on entity count
    PerformanceBased,   // Route based on measured performance
    ForceCPU,           // Always use CPU
    ForceGPU,           // Always use GPU
    Adaptive            // Dynamic adjustment based on metrics
};
```

### Configuration Presets

```cpp
// Physics-heavy simulations
auto config = HybridPhysicsConfig::physics_optimized();

// Low-latency requirements (120 FPS)
auto config = HybridPhysicsConfig::low_latency();

// Maximum throughput
auto config = HybridPhysicsConfig::high_throughput();
```

### Performance Monitoring

```cpp
const HybridPhysicsStats& stats = hybrid.stats();

// Frame statistics
Real avg_frame_time = stats.avg_frame_time_ms;
UInt64 gpu_frames = stats.gpu_only_frames;

// Workload statistics
for (const auto& ws : stats.workload_stats) {
    Real cpu_throughput = ws.cpu_throughput_per_us();
    Real gpu_throughput = ws.gpu_throughput_per_us();
}

// GPU efficiency
Real efficiency = stats.gpu_efficiency();

// Last routing decision
const RoutingDecision& decision = hybrid.last_decision();
if (decision.use_gpu) {
    // GPU path was chosen
}
```

## Device Capabilities

```cpp
DeviceCapabilities caps = backend->get_device_capabilities(0);

// Device info
std::string name = caps.name;           // e.g., "NVIDIA RTX 4090"
std::string vendor = caps.vendor;       // e.g., "NVIDIA Corporation"
DeviceType type = caps.device_type;     // DiscreteGPU, IntegratedGPU, etc.

// Memory
SizeT global_mem = caps.global_memory;          // Total GPU memory
SizeT local_mem = caps.local_memory;            // Per-workgroup memory
SizeT bandwidth = caps.memory_bandwidth;        // Bytes/second

// Compute units
UInt32 compute_units = caps.compute_units;      // SMs for NVIDIA
UInt32 warp_size = caps.warp_size;              // 32 for NVIDIA

// Features
bool has_double = caps.supports_double;
bool has_atomics = caps.supports_atomics;
bool has_unified = caps.supports_unified_memory;

// Check requirements
bool meets_reqs = caps.meets_requirements(
    1024 * 1024 * 1024,  // 1GB minimum memory
    8                     // 8 compute units minimum
);
```

## Best Practices

### Memory Efficiency

```cpp
// Use SoA layout for GPU efficiency
std::vector<float> positions_x(count);
std::vector<float> positions_y(count);
std::vector<float> positions_z(count);

// Prefer DeviceLocal for compute-only buffers
BufferHandle compute_buffer = backend->allocate(
    size, MemoryType::DeviceLocal
);

// Use Shared memory for frequent CPU-GPU access
BufferHandle shared_buffer = backend->allocate(
    size, MemoryType::Shared
);
```

### Async Execution

```cpp
// Create separate streams for overlapping operations
StreamHandle upload_stream = backend->create_stream();
StreamHandle compute_stream = backend->create_stream();
StreamHandle download_stream = backend->create_stream();

// Upload on one stream
LaunchConfig upload_config;
upload_config.stream = upload_stream;
backend->upload(buffer, data, size, 0, upload_stream);

// Record event after upload
EventHandle upload_done = backend->record_event(upload_stream);

// Make compute stream wait for upload
backend->stream_wait_event(compute_stream, upload_done);

// Dispatch compute
LaunchConfig compute_config;
compute_config.stream = compute_stream;
backend->dispatch(kernel, compute_config);

// Wait for completion
backend->synchronize_stream(compute_stream);
```

### Error Handling

```cpp
BackendResult result = backend->dispatch(kernel, config);

if (result != BackendResult::Success) {
    std::string error = backend->last_error();
    std::cerr << "GPU error: " << result_to_string(result)
              << " - " << error << std::endl;

    // Fallback to CPU
    execute_cpu_fallback();
}
```

## CPU Fallback

All GPU kernels have CPU implementations for graceful fallback:

```cpp
namespace cpu_kernels {
    void integrate_positions_cpu(float* positions, const float* velocities,
                                 float dt, SizeT count);

    void integrate_velocities_cpu(float* velocities, const float* forces,
                                  const float* masses, float gx, float gy, float gz,
                                  float dt, SizeT count);

    SizeT collision_broad_phase_cpu(const AABB* aabbs, SizeT count,
                                    CollisionPair* pairs, SizeT max_pairs,
                                    const SpatialHashConfig& config);
}
```

## Build Configuration

Enable GPU backends in CMake:

```cmake
option(JAGUAR_ENABLE_CUDA "Enable CUDA GPU acceleration" OFF)
option(JAGUAR_ENABLE_METAL "Enable Metal GPU acceleration (macOS)" OFF)
option(JAGUAR_ENABLE_VULKAN "Enable Vulkan compute" OFF)
```
