# JaguarEngine API Reference

Complete API reference for JaguarEngine multi-domain physics simulation platform.

## Table of Contents

1. [Core API](#core-api)
2. [Physics API](#physics-api)
3. [Threading API](#threading-api)
4. [Terrain API](#terrain-api)
5. [Entity Loader API](#entity-loader-api)
6. [Federation API](#federation-api)
7. [Air Domain API](#air-domain-api)
8. [Land Domain API](#land-domain-api)
9. [Sea Domain API](#sea-domain-api)
10. [Space Domain API](#space-domain-api)
11. [Environment API](#environment-api)
12. [Configuration API](#configuration-api)

---

## Core API

**Header:** `jaguar/core/types.h`

### Basic Types

```cpp
namespace jaguar {
    using Real = double;         // Primary floating-point type
    using SizeT = std::size_t;   // Size type
    using UInt8 = uint8_t;       // 8-bit unsigned
    using UInt32 = uint32_t;     // 32-bit unsigned
    using UInt64 = uint64_t;     // 64-bit unsigned
    using Int32 = int32_t;       // 32-bit signed
    using Int64 = int64_t;       // 64-bit signed
}
```

### Vector Types

```cpp
struct Vec3 {
    Real x, y, z;

    Real norm() const;
    Vec3 normalized() const;
    Real dot(const Vec3& other) const;
    Vec3 cross(const Vec3& other) const;

    Vec3 operator+(const Vec3& other) const;
    Vec3 operator-(const Vec3& other) const;
    Vec3 operator*(Real scalar) const;
};

struct Vec4 {
    Real x, y, z, w;
};
```

### Quaternion

```cpp
struct Quaternion {
    Real w, x, y, z;

    static Quaternion identity();
    static Quaternion from_euler(Real roll, Real pitch, Real yaw);
    static Quaternion from_axis_angle(const Vec3& axis, Real angle);

    Quaternion normalized() const;
    Quaternion conjugate() const;
    Vec3 rotate(const Vec3& v) const;
    Mat3x3 to_rotation_matrix() const;
    void to_euler(Real& roll, Real& pitch, Real& yaw) const;

    Quaternion operator*(const Quaternion& other) const;
};
```

### Matrix Types

```cpp
struct Mat3x3 {
    Real data[3][3];

    static Mat3x3 identity();
    static Mat3x3 from_euler(Real roll, Real pitch, Real yaw);

    Vec3 operator*(const Vec3& v) const;
    Mat3x3 operator*(const Mat3x3& other) const;
    Mat3x3 transpose() const;
};
```

### Enumerations

```cpp
enum class Domain : UInt8 {
    Generic = 0,
    Air = 1,
    Land = 2,
    Sea = 3,
    Space = 4
};

enum class CoordinateFrame : UInt8 {
    ECEF = 0,   // Earth-Centered Earth-Fixed
    NED = 1,    // North-East-Down
    ENU = 2,    // East-North-Up
    ECI = 3     // Earth-Centered Inertial
};

using ComponentMask = UInt64;
using EntityId = UInt64;
constexpr EntityId INVALID_ENTITY_ID = 0;
```

### Constants

```cpp
namespace constants {
    constexpr Real PI = 3.14159265358979323846;
    constexpr Real TWO_PI = 6.28318530717958647692;
    constexpr Real DEG_TO_RAD = PI / 180.0;
    constexpr Real RAD_TO_DEG = 180.0 / PI;
    constexpr Real G0 = 9.80665;              // Standard gravity (m/s²)
    constexpr Real EARTH_RADIUS = 6378137.0;  // WGS84 equatorial radius (m)
    constexpr Real EARTH_MU = 3.986004418e14; // Earth gravitational parameter (m³/s²)
}
```

---

## Physics API

**Header:** `jaguar/physics/entity.h`, `jaguar/physics/solver.h`, `jaguar/physics/force.h`

### EntityState

```cpp
struct EntityState {
    Vec3 position{0, 0, 0};           // Position (m)
    Vec3 velocity{0, 0, 0};           // Velocity (m/s)
    Quaternion orientation;            // Attitude quaternion
    Vec3 angular_velocity{0, 0, 0};   // Angular velocity (rad/s)
    Real mass{1.0};                   // Mass (kg)
    Mat3x3 inertia;                   // Inertia tensor (kg·m²)

    // Derived quantities
    Real get_roll() const;
    Real get_pitch() const;
    Real get_yaw() const;
    Vec3 get_euler_angles() const;
    Mat3x3 get_rotation_matrix() const;
};
```

### EntityForces

```cpp
struct EntityForces {
    Vec3 force{0, 0, 0};    // Total force (N)
    Vec3 torque{0, 0, 0};   // Total torque (N·m)

    void clear();
    void add_force(const Vec3& f);
    void add_torque(const Vec3& t);
    void add_force_at_point(const Vec3& f, const Vec3& point, const Vec3& cg);
};
```

### EntityManager

```cpp
class EntityManager {
public:
    EntityId create_entity(const std::string& name, Domain domain);
    void destroy_entity(EntityId id);
    bool entity_exists(EntityId id) const;

    EntityState get_state(EntityId id) const;
    void set_state(EntityId id, const EntityState& state);

    Domain get_domain(EntityId id) const;
    std::string get_name(EntityId id) const;

    SizeT entity_count() const;
    std::vector<EntityId> get_all_entities() const;
    std::vector<EntityId> get_entities_by_domain(Domain domain) const;
};
```

### Integration

```cpp
// Integrator interface
class IStatePropagator {
public:
    virtual void propagate(EntityState& state,
                           const EntityForces& forces,
                           Real dt) = 0;
};

// RK4 Integrator
class RK4Integrator : public IStatePropagator {
public:
    void propagate(EntityState& state,
                   const EntityForces& forces,
                   Real dt) override;
};

// ABM4 Integrator
class ABM4Integrator : public IStatePropagator {
public:
    void propagate(EntityState& state,
                   const EntityForces& forces,
                   Real dt) override;
};
```

### Force Generator Interfaces

```cpp
/// Base interface for all force-generating components
class IForceGenerator {
public:
    virtual ~IForceGenerator() = default;

    /**
     * Compute forces and torques for an entity
     * @param state Current entity state
     * @param env Environment conditions
     * @param dt Time step (seconds)
     * @param out_forces Output forces and torques (accumulated)
     */
    virtual void compute_forces(const EntityState& state,
                                const environment::Environment& env,
                                Real dt,
                                EntityForces& out_forces) = 0;

    /// Initialize the force generator
    virtual bool initialize() { return true; }

    /// Get the name of this force generator
    virtual const std::string& name() const = 0;

    /// Get the domain this generator applies to
    virtual Domain domain() const = 0;

    /// Check if this generator is enabled
    virtual bool is_enabled() const;

    /// Enable/disable this generator
    virtual void set_enabled(bool enabled);
};

/// Force generator for aerodynamic forces (lift, drag, moments)
class IAerodynamicsModel : public IForceGenerator {
public:
    Domain domain() const override { return Domain::Air; }

    virtual Real get_cl() const = 0;         ///< Lift coefficient
    virtual Real get_cd() const = 0;         ///< Drag coefficient
    virtual Real get_cm() const = 0;         ///< Pitching moment coefficient
    virtual Real get_alpha() const = 0;      ///< Angle of attack (rad)
    virtual Real get_beta() const = 0;       ///< Sideslip angle (rad)
    virtual Real get_mach() const = 0;       ///< Mach number
    virtual Real get_qbar() const = 0;       ///< Dynamic pressure (Pa)
};

/// Force generator for propulsion (engines, rockets)
class IPropulsionModel : public IForceGenerator {
public:
    Domain domain() const override { return Domain::Air; }

    virtual Real get_thrust() const = 0;         ///< Current thrust (N)
    virtual Real get_fuel_flow() const = 0;      ///< Fuel flow rate (kg/s)
    virtual Real get_fuel_remaining() const = 0; ///< Remaining fuel (kg)
    virtual void set_throttle(Real throttle) = 0; ///< Set throttle (0.0-1.0)
    virtual bool is_running() const = 0;         ///< Engine running?
};

/// Force generator for terrain interaction (wheels, tracks)
class ITerramechanicsModel : public IForceGenerator {
public:
    Domain domain() const override { return Domain::Land; }

    virtual Real get_sinkage() const = 0;           ///< Sinkage depth (m)
    virtual Real get_motion_resistance() const = 0; ///< Resistance force (N)
    virtual Real get_traction() const = 0;          ///< Traction force (N)
    virtual Real get_slip_ratio() const = 0;        ///< Slip ratio
};

/// Force generator for hydrodynamic forces (hull, rudder)
class IHydrodynamicsModel : public IForceGenerator {
public:
    Domain domain() const override { return Domain::Sea; }

    virtual Real get_buoyancy() const = 0;  ///< Buoyancy force (N)
    virtual Real get_draft() const = 0;     ///< Draft (m)
    virtual Real get_heel() const = 0;      ///< Heel angle (rad)
    virtual Real get_trim() const = 0;      ///< Trim angle (rad)
};

/// Force generator for gravitational forces
class IGravityModel : public IForceGenerator {
public:
    Domain domain() const override { return Domain::Generic; }

    virtual Vec3 get_gravity_acceleration() const = 0;
    virtual void set_fidelity(int level) = 0;  ///< 0=point mass, 1=J2, 2=J4, 3=EGM96
};
```

### Force Generator Registry

Central registry for managing force generators per entity:

```cpp
class ForceGeneratorRegistry {
public:
    ForceGeneratorRegistry() = default;

    /**
     * Register a force generator
     */
    void register_generator(std::unique_ptr<IForceGenerator> generator);

    /**
     * Get all generators for a specific domain
     */
    std::vector<IForceGenerator*> get_generators(Domain domain);

    /**
     * Get all enabled generators
     */
    std::vector<IForceGenerator*> get_enabled_generators();

    /**
     * Find generator by name
     */
    IForceGenerator* find(const std::string& name);

    /**
     * Clear all generators
     */
    void clear();

    /**
     * Get total number of registered generators
     */
    SizeT size() const;
};
```

**Usage:**

```cpp
ForceGeneratorRegistry registry;

// Register generators for an aircraft
registry.register_generator(create_simple_gravity());
registry.register_generator(create_wgs84_gravity());
registry.register_generator(create_simple_aerodynamics());
registry.register_generator(create_drag_model(0.25, 30.0));

// Get all enabled generators
auto active = registry.get_enabled_generators();

// Find specific generator
auto aero = registry.find("aerodynamics");
if (aero) {
    aero->set_enabled(true);
}

// Query by domain
auto air_gens = registry.get_generators(Domain::Air);
```

---

## Threading API

**Header:** `jaguar/core/threading/thread_pool.h`

### Work-Stealing Thread Pool

High-performance thread pool using Chase-Lev work-stealing algorithm for parallel force computation and physics simulation:

```cpp
class ThreadPool {
public:
    /**
     * Construct a thread pool
     * @param num_threads Number of worker threads (0 = hardware concurrency)
     */
    explicit ThreadPool(SizeT num_threads = 0);

    ~ThreadPool();

    /**
     * Submit a task and get a future for the result
     * @tparam F Callable type
     * @tparam Args Argument types
     * @param f Function to execute
     * @param args Arguments to pass
     * @return Future containing the result
     */
    template<typename F, typename... Args>
    auto submit(F&& f, Args&&... args)
        -> std::future<std::invoke_result_t<F, Args...>>;

    /**
     * Execute a parallel for loop
     * @param start Start index (inclusive)
     * @param end End index (exclusive)
     * @param body Function to execute for each index
     * @param grain_size Minimum iterations per task (0 = auto)
     */
    void parallel_for(SizeT start, SizeT end,
                      const std::function<void(SizeT)>& body,
                      SizeT grain_size = 0);

    /**
     * Execute a parallel for loop with range-based body
     * @param start Start index (inclusive)
     * @param end End index (exclusive)
     * @param body Function receiving (start, end) range
     * @param grain_size Minimum iterations per task (0 = auto)
     */
    void parallel_for_range(SizeT start, SizeT end,
                           const std::function<void(SizeT, SizeT)>& body,
                           SizeT grain_size = 0);

    /// Get number of worker threads
    SizeT num_threads() const;

    /// Get current thread index (0 to num_threads-1, or num_threads if not in pool)
    SizeT current_thread_index() const;

    /// Shutdown pool and wait for all tasks
    void shutdown();

    /// Check if pool is shut down
    bool is_shutdown() const;

    /// Wait for all pending tasks to complete
    void wait_all();

    /**
     * Get the global thread pool instance
     * @param num_threads Number of threads (only used on first call)
     */
    static ThreadPool& get_global(SizeT num_threads = 0);
};
```

**Features:**
- Per-thread work queues for cache efficiency
- Work stealing for automatic load balancing
- Template-based submit() for type-safe task submission
- Parallel iteration with automatic grain size tuning
- Global singleton pool for easy access

**Usage:**
```cpp
// Use global thread pool
auto& pool = ThreadPool::get_global();

// Submit parallel force computations
std::vector<std::future<EntityForces>> tasks;
for (auto& entity : entities) {
    auto task = pool.submit([&entity, &env, dt]() {
        EntityForces forces;
        compute_forces_for_entity(entity, env, dt, forces);
        return forces;
    });
    tasks.push_back(std::move(task));
}

// Wait for all to complete
pool.wait_all();

// Collect results
for (auto& task : tasks) {
    auto forces = task.get();
    // Process...
}

// Parallel for loops
pool.parallel_for(0, entities.size(), [&](SizeT i) {
    process_entity(entities[i]);
});

// Range-based parallel for (better cache locality)
pool.parallel_for_range(0, entities.size(),
    [&](SizeT start, SizeT end) {
        for (SizeT i = start; i < end; ++i) {
            process_entity(entities[i]);
        }
    }, 64);  // Process 64 entities per task
```

**Parallel Algorithms:**

```cpp
// Parallel reduce
Real total_mass = parallel_reduce<decltype(entities)::iterator, Real>(
    pool, entities.begin(), entities.end(), 0.0,
    [](Real a, const Entity& e) { return a + e.mass; });

// Parallel transform
std::vector<Vec3> positions(entities.size());
parallel_transform(pool,
    entities.begin(), entities.end(), positions.begin(),
    [](const Entity& e) { return e.position; });
```

---

## Terrain API

**Header:** `jaguar/environment/terrain.h`

### GDAL Terrain Loader

Load GIS terrain data from GeoTIFF and other GDAL-supported formats with bilinear interpolation:

```cpp
class GDALTerrainLoader {
public:
    GDALTerrainLoader();
    ~GDALTerrainLoader();

    /**
     * Check if GDAL is available at compile time
     */
    static bool is_available();

    /**
     * Open terrain dataset from file
     * @param path Path to GeoTIFF or other GDAL-supported raster
     * @return true if successfully opened
     */
    bool open(const std::string& path);

    /**
     * Close current dataset and release resources
     */
    void close();

    /**
     * Get dataset metadata
     */
    TerrainDataset get_dataset_info() const;

    /**
     * Get elevation at WGS84 position with bilinear interpolation
     * @param lat Latitude (radians)
     * @param lon Longitude (radians)
     * @return Elevation in meters, or NaN if unavailable
     */
    Real get_elevation(Real lat, Real lon) const;

    /**
     * Get surface normal at position using elevation gradient
     * @param lat Latitude (radians)
     * @param lon Longitude (radians)
     * @return Surface normal in NED frame, or {0,0,1} if unavailable
     */
    Vec3 get_normal(Real lat, Real lon) const;

    /**
     * Complete terrain query
     * @param lat Latitude (radians)
     * @param lon Longitude (radians)
     * @return Terrain query with elevation and normal
     */
    TerrainQuery query(Real lat, Real lon) const;

    /**
     * Get dataset geographic bounds (WGS84)
     * @param min_lat, max_lat, min_lon, max_lon Bounds in radians
     */
    void get_bounds(Real& min_lat, Real& max_lat, Real& min_lon, Real& max_lon) const;

    /**
     * Get spatial resolution in degrees
     */
    Real get_resolution() const;

    /**
     * Check if a dataset is currently loaded
     */
    bool is_loaded() const;
};

/// Dataset metadata structure
struct TerrainDataset {
    Real min_lat, max_lat;       ///< Latitude bounds (rad)
    Real min_lon, max_lon;       ///< Longitude bounds (rad)
    Real resolution{0.0};        ///< Spatial resolution (degrees)
    int width{0};                ///< Raster width (pixels)
    int height{0};               ///< Raster height (pixels)
    std::string projection;      ///< Projection/CRS information
    bool valid{false};
};
```

**Features:**
- Bilinear interpolation for smooth elevation queries
- Surface normal computation from elevation gradient
- Support for GeoTIFF, DTED, and all GDAL-supported rasters
- Graceful fallback when GDAL unavailable
- Thread-safe read operations

**Usage:**
```cpp
GDALTerrainLoader loader;
if (loader.open("/data/n37e122.tif")) {
    Real elev = loader.get_elevation(lat, lon);
    Vec3 normal = loader.get_normal(lat, lon);
    TerrainQuery query = loader.query(lat, lon);

    // Check bounds
    Real res = loader.get_resolution();
    auto info = loader.get_dataset_info();
}
```

### Terrain Manager

Central terrain management with caching and LOD support:

```cpp
class TerrainManager {
public:
    TerrainManager();
    ~TerrainManager();

    /**
     * Add data source path (DTED, GeoTIFF directory)
     */
    void add_data_path(const std::string& path);

    /**
     * Load terrain from a file
     * @param path Path to GeoTIFF or other GDAL-supported raster
     * @return true if successfully loaded
     */
    bool load_terrain(const std::string& path);

    /**
     * Check if terrain data is loaded
     */
    bool is_terrain_loaded() const;

    /**
     * Set cache size in MB
     */
    void set_cache_size(SizeT mb);

    /**
     * Initialize terrain system
     */
    bool initialize();

    /**
     * Shutdown and release resources
     */
    void shutdown();

    /**
     * Get elevation at geodetic position
     * @param lat Latitude (rad)
     * @param lon Longitude (rad)
     * @return Elevation in meters, or NaN if unavailable
     */
    Real get_elevation(Real lat, Real lon) const;

    /**
     * Get elevation at ECEF position
     */
    Real get_elevation_ecef(const Vec3& ecef) const;

    /**
     * Get surface normal at position
     */
    Vec3 get_surface_normal(Real lat, Real lon) const;

    /**
     * Get slope angle at position
     */
    Real get_slope_angle(Real lat, Real lon) const;

    /**
     * Get material at position
     */
    TerrainMaterial get_material(Real lat, Real lon) const;

    /**
     * Complete terrain query
     */
    TerrainQuery query(Real lat, Real lon) const;

    /**
     * Query terrain at ECEF position
     */
    TerrainQuery query_ecef(const Vec3& ecef) const;

    /**
     * Set focus point for LOD management
     */
    void set_focus_point(const Vec3& ecef);

    /**
     * Set detail radius around focus
     */
    void set_detail_radius(Real radius_m);

    /**
     * Update terrain paging (call each frame)
     */
    void update();

    /**
     * Get number of loaded tiles
     */
    SizeT loaded_tile_count() const;

    /**
     * Get cache memory usage
     */
    SizeT cache_usage_bytes() const;
};

/// Terrain query result
struct TerrainQuery {
    Real elevation{0.0};        ///< Height above WGS84 (m)
    Vec3 normal{0, 0, 1};       ///< Surface normal (NED)
    Real slope_angle{0.0};      ///< Slope angle (rad)
    TerrainMaterial material;
    bool valid{false};
};

/// Terrain material properties
struct TerrainMaterial {
    SurfaceType type{SurfaceType::Unknown};
    Real friction_coefficient{0.7};
    Real rolling_resistance{0.01};
    Real k_c{0.0};              ///< Cohesive modulus (Bekker-Wong)
    Real k_phi{0.0};            ///< Frictional modulus
    Real n{1.0};                ///< Deformation exponent
    Real max_slope_deg{30.0};
    Real bearing_capacity{100000.0};  ///< Pa
};
```

**Features:**
- Thread-safe read operations (queries)
- Asynchronous tile loading with cache management
- LOD management with focus point
- Multiple raster format support
- Bilinear interpolation for smooth queries

**Usage:**
```cpp
TerrainManager terrain;
terrain.add_data_path("/data/dted");
terrain.load_terrain("/data/dted/n37/e122/n37e122.tif");
terrain.set_cache_size(512);  // 512MB cache
terrain.set_detail_radius(10000);  // 10km detail radius

// In main loop, update LOD
terrain.set_focus_point(entity_ecef_position);
terrain.update();

// Query in physics loop (thread-safe)
Real elevation = terrain.get_elevation(lat, lon);
Vec3 normal = terrain.get_surface_normal(lat, lon);
TerrainQuery query = terrain.query(lat, lon);
```

---

## Entity Loader API

**Header:** `jaguar/interface/xml_entity_loader.h`

### XmlEntityLoader

Load entity definitions from JSBSim-style XML configuration:

```cpp
class XmlEntityLoader {
public:
    XmlEntityLoader() = default;

    /**
     * Load entity definition from XML file
     * @param path Path to XML file
     * @return Parsed entity definition
     * @throws std::runtime_error on parse failure
     */
    EntityDefinition load(const std::string& path);

    /**
     * Load entity definition from XML string
     * @param xml XML content
     * @return Parsed entity definition
     */
    EntityDefinition load_from_string(const std::string& xml);

    /**
     * Get last error message
     */
    const std::string& get_error() const;
};
```

### EntityDefinition Structure

Complete entity definition containing all properties and component data:

```cpp
struct EntityDefinition {
    std::string name;                               ///< Entity name
    std::string type;                               ///< "aircraft", "vehicle", "vessel", etc.
    Domain domain{Domain::Generic};

    EntityMetrics metrics;                          ///< Geometric properties
    MassBalance mass_balance;                       ///< Mass and inertia

    std::optional<AerodynamicsData> aerodynamics;
    std::optional<PropulsionData> propulsion;

    ComponentMask component_mask{0};                ///< Bitmask of available components
};

struct EntityMetrics {
    Real wingspan{0.0};         ///< Wingspan (m)
    Real length{0.0};           ///< Length (m)
    Real height{0.0};           ///< Height (m)
    Real wing_area{0.0};        ///< Wing planform area (m²)
    Real reference_area{0.0};   ///< Reference area (m²)
};

struct MassBalance {
    Real empty_mass{0.0};       ///< Empty weight (kg)
    Real fuel_mass{0.0};        ///< Fuel capacity (kg)
    Vec3 cg_location{};         ///< Center of gravity (m, body frame)
    Mat3x3 inertia{};           ///< Inertia tensor (kg·m²)
};

struct AeroCoefficient {
    std::string name;           ///< Coefficient name (e.g., "CL", "CD")
    std::vector<Real> breakpoints;  ///< Alpha breakpoints
    std::vector<Real> values;   ///< Coefficient values at breakpoints
};

struct AerodynamicsData {
    std::vector<AeroCoefficient> lift_tables;
    std::vector<AeroCoefficient> drag_tables;
    std::vector<AeroCoefficient> moment_tables;
};

struct PropulsionData {
    std::string engine_type;        ///< Engine type identifier
    Real max_thrust{0.0};           ///< Maximum thrust (N)
    Real afterburner_thrust{0.0};   ///< Afterburner thrust (N)
    Vec3 thrust_location{};         ///< Thrust vector location (m, body frame)
};
```

**XML Format:**

```xml
<?xml version="1.0"?>
<entity name="F-16C" type="aircraft">
    <domain>air</domain>

    <metrics>
        <wingspan>9.45</wingspan>
        <length>15.45</length>
        <height>4.88</height>
        <wing_area>27.87</wing_area>
        <reference_area>27.87</reference_area>
    </metrics>

    <mass_balance>
        <empty_mass>7700</empty_mass>
        <fuel_mass>2728</fuel_mass>
        <cg_location x="0.0" y="0.0" z="-1.5"/>
        <inertia ixx="23500" iyy="142000" izz="158000" ixz="0"/>
    </mass_balance>

    <aerodynamics>
        <cl_table name="CL" alpha_ref="0.0">
            <breakpoints>0, 5, 10, 15, 20</breakpoints>
            <values>0.0, 0.3, 0.8, 1.2, 1.1</values>
        </cl_table>
        <cd_table name="CD">
            <breakpoints>0, 5, 10, 15, 20</breakpoints>
            <values>0.02, 0.025, 0.05, 0.12, 0.18</values>
        </cd_table>
    </aerodynamics>

    <propulsion>
        <engine_type>turbofan</engine_type>
        <max_thrust>120000</max_thrust>
        <afterburner_thrust>200000</afterburner_thrust>
        <thrust_location x="0.0" y="0.0" z="0.0"/>
    </propulsion>
</entity>
```

**Usage:**

```cpp
XmlEntityLoader loader;

// Load from file
EntityDefinition def = loader.load("f16.xml");
if (def.name.empty()) {
    std::cerr << "Error: " << loader.get_error() << std::endl;
}

// Load from string
std::string xml_str = R"(<?xml version="1.0"?>...)";
def = loader.load_from_string(xml_str);

// Create entity from definition
if (!def.name.empty()) {
    EntityId id = engine.create_entity(def.name, def.domain);

    // Set up entity state from definition
    EntityState state;
    state.mass = def.mass_balance.empty_mass;
    state.inertia = def.mass_balance.inertia;
    engine.set_entity_state(id, state);

    // Check what components are available
    if (def.aerodynamics) {
        // Register aerodynamics model...
    }
    if (def.propulsion) {
        // Register propulsion model...
    }
}
```

---

## Federation API

**Header:** `jaguar/federation/dis_protocol.h`, `jaguar/federation/hla_rti.h`

### DIS Protocol (IEEE 1278.1-2012)

Distributed Interactive Simulation for real-time distributed military simulation:

#### Entity State PDU

Primary PDU for entity position and state:

```cpp
struct EntityStatePDU {
    PDUHeader header;

    EntityIdentifier entity_id;     ///< Unique entity identifier
    ForceId force_id;               ///< Friendly, Opposing, Neutral, etc.

    EntityType entity_type;         ///< SISO-REF-010 entity type hierarchy
    Vec3 entity_linear_velocity;    ///< Velocity (m/s) in ECEF
    Vec3 entity_location;           ///< Position (m) in ECEF

    EulerAngles entity_orientation; ///< Euler angles (yaw, pitch, roll)
    EntityAppearance appearance;    ///< Visual appearance (damage, smoke, etc.)

    DeadReckoningParameters dead_reckoning_params;
    EntityMarking entity_marking;   ///< Entity name/callsign
    EntityCapabilities capabilities;

    std::vector<ArticulationParameter> articulation_params;
};

/// Dead Reckoning Models (DRM)
enum class DeadReckoningAlgorithm : UInt8 {
    DRM_Static = 0,      ///< No movement
    DRM_FPW = 1,         ///< Fixed Position, World
    DRM_RPW = 2,         ///< Rotating Position, World
    DRM_RVW = 3,         ///< Rotating Velocity, World
    DRM_FVW = 4,         ///< Fixed Velocity, World
    DRM_FPB = 5,         ///< Fixed Position, Body
    DRM_RPB = 6,         ///< Rotating Position, Body
    DRM_RVB = 7,         ///< Rotating Velocity, Body
    DRM_FVB = 8,         ///< Fixed Velocity, Body
    DRM_RVW_HighRes = 9  ///< Rotating Velocity, World (high-res)
};

/// Unique entity identifier (Site, Application, Entity)
struct EntityIdentifier {
    UInt16 site{0};
    UInt16 application{0};
    UInt16 entity{0};

    bool is_valid() const;
    bool operator==(const EntityIdentifier& other) const;
};
```

#### Fire and Detonation PDUs

Warfare simulation events:

```cpp
struct FirePDU {
    PDUHeader header;

    EntityIdentifier firing_entity_id;
    EntityIdentifier target_entity_id;
    EntityIdentifier munition_id;
    EventIdentifier event_id;

    UInt32 fire_mission_index{0};
    Vec3 location_in_world;         ///< Fire location (ECEF meters)
    BurstDescriptor burst_descriptor;
    Vec3 velocity;                  ///< Initial munition velocity (m/s)
    Real range{0.0};                ///< Range to target (meters)
};

struct DetonationPDU {
    PDUHeader header;

    EntityIdentifier firing_entity_id;
    EntityIdentifier target_entity_id;
    EntityIdentifier munition_id;
    EventIdentifier event_id;

    Vec3 velocity;                  ///< Velocity at detonation (m/s)
    Vec3 location_in_world;         ///< Detonation location (ECEF meters)
    BurstDescriptor burst_descriptor;
    Vec3 location_in_entity;        ///< Impact location relative to target
    DetonationResult detonation_result;

    std::vector<ArticulationParameter> articulation_params;
};

enum class DetonationResult : UInt8 {
    Other = 0,
    EntityImpact = 1,
    EntityProximateDetonation = 2,
    GroundImpact = 3,
    GroundProximateDetonation = 4,
    Detonation = 5,
    // ... additional values
};

struct BurstDescriptor {
    EntityType munition_type;
    UInt16 warhead{0};
    UInt16 fuse{0};
    UInt16 quantity{1};     ///< Number of rounds
    UInt16 rate{0};         ///< Rate of fire (rounds/min)
};
```

#### Dead Reckoning Calculator

Position extrapolation for network bandwidth reduction:

```cpp
class DeadReckoningCalculator {
public:
    /**
     * Extrapolate entity position using dead reckoning
     * @param state Current entity state (at last update time)
     * @param elapsed_time Time since last update (seconds)
     * @return Extrapolated state
     */
    EntityStatePDU extrapolate_position(const EntityStatePDU& state,
                                        Real elapsed_time) const noexcept;

    /**
     * Check if entity state should be updated
     * @param current Current actual state
     * @param predicted Predicted state from dead reckoning
     * @param position_threshold Position error threshold (meters)
     * @param orientation_threshold Orientation error threshold (radians)
     * @return True if update should be sent
     */
    bool should_send_update(const EntityStatePDU& current,
                           const EntityStatePDU& predicted,
                           Real position_threshold = DIS_DR_THRESHOLD_POSITION,
                           Real orientation_threshold = DIS_DR_THRESHOLD_ORIENTATION) const noexcept;

    /// Calculate position error between two states (meters)
    Real calculate_position_error(const EntityStatePDU& a,
                                  const EntityStatePDU& b) const noexcept;

    /// Calculate orientation error between two states (radians)
    Real calculate_orientation_error(const EntityStatePDU& a,
                                     const EntityStatePDU& b) const noexcept;
};
```

#### DIS Network Interface

Send/receive DIS PDUs over network:

```cpp
class IDISNetwork {
public:
    /**
     * Initialize network (bind socket, join multicast group)
     * @param multicast_address Multicast group address (e.g., "239.1.2.3")
     * @param port Port number (default 3000)
     * @param interface_address Local interface to bind (empty = any)
     */
    virtual bool initialize(const std::string& multicast_address,
                           UInt16 port = 3000,
                           const std::string& interface_address = "") = 0;

    virtual void shutdown() = 0;

    /// Send PDU
    virtual bool send_pdu(const EntityStatePDU& pdu) = 0;
    virtual bool send_pdu(const FirePDU& pdu) = 0;
    virtual bool send_pdu(const DetonationPDU& pdu) = 0;

    /**
     * Receive next PDU (non-blocking)
     */
    virtual SizeT receive_pdu(UInt8* buffer, SizeT buffer_size) = 0;

    /// Join DIS exercise
    virtual void join_exercise(UInt16 site_id, UInt16 application_id) = 0;

    /// Leave exercise (send Remove Entity PDUs)
    virtual void leave_exercise() = 0;

    virtual UInt16 get_site_id() const = 0;
    virtual UInt16 get_application_id() const = 0;
    virtual UInt16 allocate_entity_id() = 0;
};

/// Create standard DIS codec (big-endian)
std::unique_ptr<IDISCodec> create_dis_codec();

/// Create DIS network handler (UDP multicast)
std::unique_ptr<IDISNetwork> create_dis_network();
```

**DIS Constants:**
```cpp
constexpr UInt8 DIS_VERSION = 7;                           // Protocol version
constexpr SizeT DIS_MAX_PDU_SIZE = 8192;                   // Max PDU size
constexpr Real DIS_HEARTBEAT_INTERVAL = 5.0;              // Seconds
constexpr Real DIS_DR_THRESHOLD_POSITION = 1.0;           // Meters
constexpr Real DIS_DR_THRESHOLD_ORIENTATION = 3.0;        // Degrees
```

### HLA (IEEE 1516-2010)

High Level Architecture federation management:

#### RTI Ambassador Interface

```cpp
class IRTIAmbassador {
public:
    // ========================================================================
    // Federation Management
    // ========================================================================

    /**
     * Create a new federation execution
     * @param federation_name Name of the federation
     * @param fom_modules List of FOM module paths
     */
    virtual HLAResult create_federation_execution(
        std::string_view federation_name,
        const std::vector<std::string>& fom_modules) = 0;

    /**
     * Destroy a federation execution
     */
    virtual HLAResult destroy_federation_execution(
        std::string_view federation_name) = 0;

    /**
     * Join a federation execution
     * @param federate_name Name of this federate
     * @param federate_type Type of this federate
     * @param federation_name Name of the federation to join
     */
    virtual HLAResult join_federation_execution(
        std::string_view federate_name,
        std::string_view federate_type,
        std::string_view federation_name) = 0;

    /**
     * Resign from federation execution
     */
    virtual HLAResult resign_federation_execution(
        ResignAction action) = 0;

    // ========================================================================
    // Declaration Management
    // ========================================================================

    /**
     * Publish object class attributes
     */
    virtual HLAResult publish_object_class_attributes(
        ObjectClassHandle object_class,
        const std::vector<AttributeHandle>& attributes) = 0;

    /**
     * Subscribe to object class attributes
     */
    virtual HLAResult subscribe_object_class_attributes(
        ObjectClassHandle object_class,
        const std::vector<AttributeHandle>& attributes,
        bool passive = false) = 0;

    /**
     * Publish interaction class
     */
    virtual HLAResult publish_interaction_class(
        InteractionClassHandle interaction_class) = 0;

    /**
     * Subscribe to interaction class
     */
    virtual HLAResult subscribe_interaction_class(
        InteractionClassHandle interaction_class,
        bool passive = false) = 0;

    // ========================================================================
    // Object Management
    // ========================================================================

    /**
     * Register object instance
     */
    virtual HLAResult register_object_instance(
        ObjectClassHandle object_class,
        ObjectInstanceHandle& handle,
        std::string_view instance_name = "") = 0;

    /**
     * Update attribute values for an object instance
     */
    virtual HLAResult update_attribute_values(
        ObjectInstanceHandle object,
        const AttributeValueSet& attributes,
        const std::vector<UInt8>& tag = {},
        const std::optional<LogicalTime>& time = std::nullopt) = 0;

    /**
     * Delete object instance
     */
    virtual HLAResult delete_object_instance(
        ObjectInstanceHandle object,
        const std::vector<UInt8>& tag = {},
        const std::optional<LogicalTime>& time = std::nullopt) = 0;

    // ========================================================================
    // Time Management
    // ========================================================================

    /**
     * Enable time regulation
     */
    virtual HLAResult enable_time_regulation(
        const LogicalTimeInterval& lookahead) = 0;

    /**
     * Enable time constrained
     */
    virtual HLAResult enable_time_constrained() = 0;

    /**
     * Request time advance
     */
    virtual HLAResult time_advance_request(
        const LogicalTime& time) = 0;

    /**
     * Query GALT (Greatest Available Logical Time)
     */
    virtual HLAResult query_galt(LogicalTime& time) = 0;

    // ========================================================================
    // Processing
    // ========================================================================

    /**
     * Process RTI callbacks (tick)
     * @param min_wait Minimum wait time (milliseconds)
     * @param max_wait Maximum wait time (milliseconds)
     */
    virtual HLAResult evo_callback(
        std::chrono::milliseconds min_wait,
        std::chrono::milliseconds max_wait) = 0;
};

/// Create an RTI Ambassador instance
std::unique_ptr<IRTIAmbassador> create_rti_ambassador(
    const HLAConfiguration& config);
```

#### HLA Configuration

```cpp
struct HLAConfiguration {
    // Federation identity
    std::string federation_name = "JaguarFederation";
    std::string federate_name = "JaguarEngine";
    std::string federate_type = "SimulationEngine";

    // FOM modules
    std::vector<std::string> fom_module_paths;

    // RTI connection
    struct RTIConnectionSettings {
        std::string rti_host = "localhost";
        UInt16 rti_port = 8989;
        std::string rti_type = "HLA_EVOLVER";
        bool use_tls = false;
    } connection;

    // Time management
    struct TimeManagementSettings {
        bool enable_time_regulation = true;
        bool enable_time_constrained = true;
        LogicalTimeInterval lookahead{0.1};
    } time_management;

    // Optional settings
    bool auto_provide_update = true;
    std::chrono::milliseconds heartbeat_interval{5000};

    static HLAConfiguration default_config();
};
```

**Usage Example:**

```cpp
// Create RTI ambassador
HLAConfiguration config = HLAConfiguration::default_config();
auto rti = create_rti_ambassador(config);

// Create federation
rti->create_federation_execution("MyFederation", {"RPR_FOM_v2.xml"});

// Join federation
ObjectInstanceHandle entity_handle;
rti->join_federation_execution("JaguarEngine", "SimEngine", "MyFederation");

// Publish attributes
auto obj_class = get_object_class_handle("BaseEntity");
std::vector<AttributeHandle> attrs = {get_attribute_handle(obj_class, "Position"),
                                       get_attribute_handle(obj_class, "Velocity")};
rti->publish_object_class_attributes(obj_class, attrs);

// Register object
rti->register_object_instance(obj_class, entity_handle, "Fighter-1");

// Update attributes in main loop
AttributeValueSet update;
update.set_attribute(position_handle, encode_position(entity.position));
update.set_attribute(velocity_handle, encode_velocity(entity.velocity));
rti->update_attribute_values(entity_handle, update);

// Time management
rti->enable_time_regulation(LogicalTimeInterval{0.1});
rti->enable_time_constrained();
rti->time_advance_request(LogicalTime{sim_time});

// Process callbacks
rti->evo_callback(std::chrono::milliseconds{10},
                  std::chrono::milliseconds{100});
```

---

## Air Domain API

**Header:** `jaguar/domain/air.h`

### AeroTable

```cpp
class AeroTable {
public:
    void set_breakpoints(int dimension, const std::vector<Real>& values);
    void set_data(const std::vector<Real>& data);
    Real lookup(const std::vector<Real>& inputs) const;
};
```

### AerodynamicsModel

```cpp
class AerodynamicsModel : public physics::IAerodynamicsModel {
public:
    // Configuration
    void set_reference_area(Real area);    // m²
    void set_reference_chord(Real chord);  // m
    void set_reference_span(Real span);    // m

    // Coefficient tables (optional)
    void set_cl_table(std::unique_ptr<AeroTable> table);
    void set_cd_table(std::unique_ptr<AeroTable> table);
    void set_cm_table(std::unique_ptr<AeroTable> table);

    // IForceGenerator
    void compute_forces(const physics::EntityState& state,
                        const environment::Environment& env,
                        Real dt,
                        physics::EntityForces& forces) override;

    // IAerodynamicsModel
    Real get_cl() const override;
    Real get_cd() const override;
    Real get_cm() const override;
    Real get_alpha() const override;  // rad
    Real get_beta() const override;   // rad
    Real get_mach() const override;
    Real get_qbar() const override;   // Pa
};
```

### PropulsionModel

```cpp
class PropulsionModel : public physics::IPropulsionModel {
public:
    // Configuration
    void set_max_thrust(Real thrust);               // N
    void set_fuel_capacity(Real fuel);              // kg
    void set_specific_fuel_consumption(Real sfc);   // kg/N/s

    // Control
    void set_throttle(Real throttle);  // 0.0 to 1.0
    void start();
    void stop();

    // IForceGenerator
    void compute_forces(const physics::EntityState& state,
                        const environment::Environment& env,
                        Real dt,
                        physics::EntityForces& forces) override;

    // IPropulsionModel
    Real get_thrust() const override;           // N
    Real get_fuel_flow() const override;        // kg/s
    Real get_fuel_remaining() const override;   // kg
    bool is_running() const override;
};
```

### FlightControlSystem

```cpp
class FlightControlSystem {
public:
    struct ControlInputs {
        Real pitch_cmd{0.0};     // -1 to +1
        Real roll_cmd{0.0};      // -1 to +1
        Real yaw_cmd{0.0};       // -1 to +1
        Real throttle_cmd{0.0};  // 0 to 1
    };

    struct ControlOutputs {
        Real elevator_deg{0.0};
        Real aileron_deg{0.0};
        Real rudder_deg{0.0};
    };

    ControlOutputs process(const ControlInputs& inputs, Real dt);

    void set_elevator_range(Real min_deg, Real max_deg);
    void set_aileron_range(Real min_deg, Real max_deg);
    void set_rudder_range(Real min_deg, Real max_deg);
};
```

---

## Land Domain API

**Header:** `jaguar/domain/land.h`

### SoilProperties

```cpp
struct SoilProperties {
    Real k_c{0.0};      // Cohesive modulus (kN/m^(n+1))
    Real k_phi{0.0};    // Frictional modulus (kN/m^(n+2))
    Real n{1.0};        // Deformation exponent
    Real c{0.0};        // Cohesion (kPa)
    Real phi{0.0};      // Internal friction angle (rad)

    // Presets
    static SoilProperties DrySand();
    static SoilProperties WetSand();
    static SoilProperties Clay();
    static SoilProperties Snow();
    static SoilProperties Asphalt();
};
```

### TerramechanicsModel

```cpp
class TerramechanicsModel : public physics::ITerramechanicsModel {
public:
    // Configuration
    void set_contact_area(Real width, Real length);  // m
    void set_vehicle_weight(Real weight_n);          // N

    // IForceGenerator
    void compute_forces(const physics::EntityState& state,
                        const environment::Environment& env,
                        Real dt,
                        physics::EntityForces& forces) override;

    // ITerramechanicsModel
    Real get_sinkage() const override;            // m
    Real get_motion_resistance() const override;  // N
    Real get_traction() const override;           // N
    Real get_slip_ratio() const override;
};
```

### SuspensionUnit

```cpp
struct SuspensionUnit {
    Real spring_k{50000.0};      // Spring stiffness (N/m)
    Real damper_c{5000.0};       // Damping coefficient (N·s/m)
    Real preload{0.0};           // Preload force (N)
    Real travel_max{0.3};        // Maximum travel (m)
    Real travel_min{0.0};        // Minimum travel (m)
    Real current_position{0.0};  // Current compression (m)
    Real current_velocity{0.0};  // Compression rate (m/s)

    Real calculate_force() const;
};
```

### SuspensionModel

```cpp
class SuspensionModel {
public:
    void add_unit(const Vec3& position, const SuspensionUnit& unit);
    void update(const physics::EntityState& state, Real dt);

    Vec3 get_total_force() const;
    Vec3 get_total_torque() const;
    SizeT unit_count() const;
};
```

### TrackedVehicleModel

```cpp
class TrackedVehicleModel {
public:
    struct TrackState {
        Real tension{10000.0};  // Track tension (N)
        Real velocity{0.0};     // Track linear velocity (m/s)
        Real slip{0.0};         // Track slip ratio
    };

    void set_sprocket(Real radius, Real max_torque);
    void update(Real drive_torque, Real load,
                const SoilProperties& soil, Real dt);

    const TrackState& get_left_track() const;
    const TrackState& get_right_track() const;
    Real get_propulsive_force() const;
};
```

---

## Sea Domain API

**Header:** `jaguar/domain/sea.h`

### SeaState

```cpp
enum class WaveSpectrum { PiersonMoskowitz, JONSWAP, Bretschneider };

struct SeaState {
    Real significant_height{1.0};  // H_s (m)
    Real peak_period{6.0};         // T_p (s)
    Real direction{0.0};           // Primary wave direction (rad from N)
    WaveSpectrum spectrum{WaveSpectrum::PiersonMoskowitz};

    static SeaState FromNATOSeaState(int sea_state);  // 0-8
};
```

### WaveModel

```cpp
class WaveModel {
public:
    void set_sea_state(const SeaState& state);

    Real get_elevation(Real x, Real y, Real time) const;          // m
    Vec3 get_particle_velocity(Real x, Real y, Real z, Real time) const;  // m/s
    Vec3 get_slope(Real x, Real y, Real time) const;
};
```

### BuoyancyModel

```cpp
class BuoyancyModel : public physics::IHydrodynamicsModel {
public:
    void set_displaced_volume(Real volume);       // m³
    void set_metacentric_height(Real gm);         // m
    void set_center_of_buoyancy(const Vec3& cb);  // relative to CG

    void compute_forces(const physics::EntityState& state,
                        const environment::Environment& env,
                        Real dt,
                        physics::EntityForces& forces) override;

    Real get_buoyancy() const override;  // N
    Real get_draft() const override;     // m
    Real get_heel() const override;      // rad
    Real get_trim() const override;      // rad
};
```

### RAOModel

```cpp
class RAOModel {
public:
    // DOF: 0=surge, 1=sway, 2=heave, 3=roll, 4=pitch, 5=yaw
    void set_rao(int dof,
                 const std::vector<Real>& frequencies,  // rad/s
                 const std::vector<Real>& amplitudes,
                 const std::vector<Real>& phases);      // rad

    void get_response(Real omega, Real wave_amp,
                      Real& out_amplitude, Real& out_phase, int dof) const;

    void calculate_response(const WaveModel& waves, Real time,
                            Vec3& out_disp, Vec3& out_rot);
};
```

### HydrodynamicsModel

```cpp
class HydrodynamicsModel : public physics::IHydrodynamicsModel {
public:
    void set_hull_coefficients(Real x_vv, Real x_rr,
                               Real y_v, Real y_r,
                               Real n_v, Real n_r);
    void set_rudder_parameters(Real area, Real aspect_ratio);
    void set_propeller_parameters(Real diameter, Real pitch_ratio);

    void set_rudder_angle(Real angle_rad);
    void set_propeller_rpm(Real rpm);

    void compute_forces(const physics::EntityState& state,
                        const environment::Environment& env,
                        Real dt,
                        physics::EntityForces& forces) override;

    Real get_buoyancy() const override;
    Real get_draft() const override;
    Real get_heel() const override;
    Real get_trim() const override;
};
```

---

## Space Domain API

**Header:** `jaguar/domain/space.h`

### OrbitalElements

```cpp
struct OrbitalElements {
    Real semi_major_axis{0.0};  // a (m)
    Real eccentricity{0.0};     // e
    Real inclination{0.0};      // i (rad)
    Real raan{0.0};             // Ω (rad) - Right Ascension of Ascending Node
    Real arg_periapsis{0.0};    // ω (rad) - Argument of Periapsis
    Real mean_anomaly{0.0};     // M (rad)
    Real epoch{0.0};            // Epoch time (seconds from reference)

    static OrbitalElements from_cartesian(const Vec3& position,
                                          const Vec3& velocity,
                                          Real mu = constants::EARTH_MU);
    void to_cartesian(Vec3& position, Vec3& velocity,
                      Real mu = constants::EARTH_MU) const;

    Real period() const;          // Orbital period (s)
    Real apoapsis() const;        // Apoapsis radius (m)
    Real periapsis() const;       // Periapsis radius (m)
    Real mean_motion() const;     // Mean motion (rad/s)
    OrbitType classify() const;
};

enum class OrbitType { LEO, MEO, GEO, HEO, Escape, Invalid };
```

### TLE

```cpp
struct TLE {
    std::string name;
    int catalog_number{0};
    int epoch_year{0};
    Real epoch_day{0.0};
    Real mean_motion_dot{0.0};
    Real mean_motion_ddot{0.0};
    Real bstar{0.0};
    Real inclination{0.0};
    Real raan{0.0};
    Real eccentricity{0.0};
    Real arg_perigee{0.0};
    Real mean_anomaly{0.0};
    Real mean_motion{0.0};
    int revolution_number{0};

    bool parse(const std::string& line1, const std::string& line2);
    OrbitalElements to_elements() const;
};
```

### SGP4Propagator

```cpp
class SGP4Propagator {
public:
    bool initialize(const TLE& tle);
    void propagate(Real minutes_since_epoch,
                   Vec3& position_eci,
                   Vec3& velocity_eci) const;
    Real get_epoch_jd() const;
};
```

### GravityModel

```cpp
enum class GravityFidelity { PointMass, J2, J4, Full };

class GravityModel {
public:
    void set_fidelity(GravityFidelity fidelity);
    Vec3 compute_acceleration(const Vec3& position_ecef) const;
    void compute_forces(const physics::EntityState& state,
                        const environment::Environment& env,
                        Real dt,
                        physics::EntityForces& forces);
};
```

### AtmosphericDragModel

```cpp
class AtmosphericDragModel {
public:
    void set_drag_coefficient(Real cd);
    void set_area(Real area);  // m²

    void compute_forces(const physics::EntityState& state,
                        const environment::Environment& env,
                        Real dt,
                        physics::EntityForces& forces);

    Real get_drag_force() const;
};
```

### JB08AtmosphereModel

```cpp
class JB08AtmosphereModel {
public:
    void set_space_weather(Real f107, Real f107_avg, Real ap);
    Real get_density(const Vec3& position_ecef, Real jd) const;  // kg/m³
};
```

### Coordinate Transforms

```cpp
namespace transforms {
    Vec3 ecef_to_eci(const Vec3& ecef, Real jd);
    Vec3 eci_to_ecef(const Vec3& eci, Real jd);
    Vec3 ecef_to_geodetic(const Vec3& ecef);  // Returns {lat, lon, alt}
    Vec3 geodetic_to_ecef(Real lat, Real lon, Real alt);
    Real greenwich_sidereal_time(Real jd);
}
```

---

## Environment API

**Header:** `jaguar/environment/environment.h`, `jaguar/environment/atmosphere.h`, `jaguar/environment/terrain.h`, `jaguar/environment/ocean.h`

### Environment

```cpp
struct Environment {
    Real latitude{0.0};            // rad
    Real longitude{0.0};           // rad
    Real altitude{0.0};            // m above WGS84
    Vec3 position_ecef{0, 0, 0};

    AtmosphereState atmosphere;
    TerrainQuery terrain;
    Real terrain_elevation{0.0};
    OceanState ocean;
    bool over_water{false};
    Vec3 gravity{0, 0, 9.80665};
};
```

### EnvironmentService

```cpp
class EnvironmentService {
public:
    bool initialize();
    void shutdown();
    void update(Real dt);

    Environment query(const Vec3& ecef, Real time) const;
    Environment query_geodetic(Real lat, Real lon, Real alt, Real time) const;

    TerrainManager& terrain();
    AtmosphereManager& atmosphere();
    OceanManager& ocean();

    void set_time(Real time);
};
```

### AtmosphereState

```cpp
struct AtmosphereState {
    Real temperature{288.15};      // K
    Real pressure{101325.0};       // Pa
    Real density{1.225};           // kg/m³
    Real speed_of_sound{340.29};   // m/s
    Real viscosity{1.789e-5};      // Pa·s
    Vec3 wind{0, 0, 0};            // m/s (NED)
    Real humidity{0.0};            // 0-1
    Real visibility{10000.0};      // m
};
```

### StandardAtmosphere

```cpp
class StandardAtmosphere {
public:
    AtmosphereState get_state(Real altitude) const;
    Real get_temperature(Real altitude) const;
    Real get_pressure(Real altitude) const;
    Real get_density(Real altitude) const;
    Real get_speed_of_sound(Real altitude) const;

    static Real geometric_to_geopotential(Real h);
    static Real geopotential_to_geometric(Real H);
};
```

### WeatherModel

```cpp
class WeatherModel {
public:
    void set_wind_layer(Real altitude, Real direction, Real speed);
    Vec3 get_wind(Real lat, Real lon, Real alt) const;

    void set_rain_rate(Real rate);  // mm/hour
    Real get_rain_attenuation(Real frequency_ghz) const;  // dB/km

    void set_fog_visibility(Real visibility_m);
    Real get_visibility(Real lat, Real lon, Real alt) const;
};
```

### AtmosphereManager

```cpp
class AtmosphereManager {
public:
    AtmosphereState get_state(Real lat, Real lon, Real alt) const;
    const StandardAtmosphere& standard() const;
    WeatherModel& weather();
    void set_weather_enabled(bool enabled);
};
```

### TerrainQuery

```cpp
struct TerrainQuery {
    Real elevation{0.0};         // m above WGS84
    Vec3 normal{0, 0, 1};        // Surface normal (NED)
    Real slope_angle{0.0};       // rad
    TerrainMaterial material;
    bool valid{false};
};
```

### TerrainManager

```cpp
class TerrainManager {
public:
    void add_data_path(const std::string& path);
    void set_cache_size(SizeT mb);
    bool initialize();
    void shutdown();

    Real get_elevation(Real lat, Real lon) const;
    Vec3 get_surface_normal(Real lat, Real lon) const;
    Real get_slope_angle(Real lat, Real lon) const;
    TerrainMaterial get_material(Real lat, Real lon) const;
    TerrainQuery query(Real lat, Real lon) const;

    void set_focus_point(const Vec3& ecef);
    void set_detail_radius(Real radius_m);
    void update();

    SizeT loaded_tile_count() const;
    SizeT cache_usage_bytes() const;
};
```

### OceanState

```cpp
struct OceanState {
    Real water_depth{1000.0};      // m
    Real surface_elevation{0.0};   // m
    Vec3 current{0, 0, 0};         // m/s
    Real temperature{15.0};        // °C
    Real salinity{35.0};           // ppt
    Real density{1025.0};          // kg/m³
};
```

### OceanManager

```cpp
class OceanManager {
public:
    void set_sea_state(const domain::sea::SeaState& state);
    const domain::sea::SeaState& get_sea_state() const;
    OceanState get_state(Real lat, Real lon, Real time) const;

    Real get_wave_elevation(Real x, Real y, Real time) const;
    Vec3 get_wave_slope(Real x, Real y, Real time) const;
    Vec3 get_particle_velocity(Real x, Real y, Real z, Real time) const;

    void set_current(const Vec3& current);
    Vec3 get_current(Real lat, Real lon, Real depth) const;
    Real get_density(Real lat, Real lon, Real depth) const;

    void update(Real dt);
};
```

---

## Configuration API

**Header:** `jaguar/interface/config.h`

### EngineConfig

```cpp
struct EngineConfig {
    Real time_step{0.01};
    SizeT max_entities{10000};
    CoordinateFrame coordinate_frame{CoordinateFrame::ECEF};

    int physics_threads{-1};
    int io_threads{2};
    bool work_stealing{true};

    bool terrain_enabled{true};
    SizeT terrain_cache_mb{2048};
    int terrain_tile_size{256};
    std::vector<std::string> terrain_data_paths;

    std::string atmosphere_model{"us_standard_1976"};
    bool weather_enabled{true};

    bool dis_enabled{false};
    int dis_port{3000};
    int dis_site_id{1};
    int dis_app_id{1};

    bool hla_enabled{false};
    std::string hla_federation;
    std::string hla_federate;

    static EngineConfig load(const std::string& path);
    static EngineConfig defaults();
    bool save(const std::string& path) const;
};
```

### EntityConfig

```cpp
struct EntityConfig {
    std::string name;
    Domain domain{Domain::Generic};

    Real empty_mass{0.0};
    Real fuel_capacity{0.0};
    Vec3 cg_location{0, 0, 0};
    Mat3x3 inertia{};

    Real wingspan{0.0};
    Real length{0.0};
    Real height{0.0};
    Real reference_area{0.0};
    Real reference_chord{0.0};

    ComponentMask components{0};

    static EntityConfig load(const std::string& path);
};
```

### ConfigLoader

```cpp
class ConfigLoader {
public:
    EngineConfig load_engine_config(const std::string& path);
    EntityConfig load_entity_config(const std::string& path);
    void add_search_path(const std::string& path);
    std::string find_file(const std::string& filename) const;
};
```

---

## Constants API

**Header:** `jaguar/core/constants.h`

### Physics and Simulation Constants

```cpp
namespace jaguar::constants {

// ============================================================================
// Simulation Time Constants
// ============================================================================

/// Default physics time step (100 Hz)
constexpr Real DEFAULT_TIME_STEP = 0.01;

/// Minimum allowed time step
constexpr Real TIME_STEP_MIN = 1e-6;

/// Maximum allowed time step
constexpr Real TIME_STEP_MAX = 0.1;

// ============================================================================
// Physics Thresholds
// ============================================================================

/// Mass threshold below which entities are considered massless
constexpr Real MASS_EPSILON = 1e-10;

/// Velocity threshold for considering entity stationary
constexpr Real VELOCITY_EPSILON = 1e-8;

/// Angular velocity threshold
constexpr Real ANGULAR_VELOCITY_EPSILON = 1e-8;

/// Inertia component threshold
constexpr Real INERTIA_EPSILON = 1e-10;

/// Position epsilon for numerical comparisons (meters)
constexpr Real POSITION_EPSILON = 1e-9;

// ============================================================================
// Constraint Solver Constants
// ============================================================================

/// Default Baumgarte stabilization factor (0.0-1.0)
constexpr Real DEFAULT_BAUMGARTE_FACTOR = 0.2;

/// Default constraint slop (penetration allowed, meters)
constexpr Real DEFAULT_SLOP = 0.01;

/// Default warm start factor (previous impulse scaling)
constexpr Real DEFAULT_WARM_START_FACTOR = 0.8;

/// Default convergence threshold for iterative solver
constexpr Real CONVERGENCE_THRESHOLD = 1e-8;

/// Maximum warm start impulse multiplier
constexpr Real MAX_WARM_START_MULTIPLIER = 10.0;

// ============================================================================
// Terrain and Environment
// ============================================================================

/// Default sea level altitude (m)
constexpr Real DEFAULT_SEA_LEVEL = 0.0;

/// Default ground level when terrain unavailable (m)
constexpr Real DEFAULT_GROUND_LEVEL = 0.0;

/// Minimum altitude above terrain (m)
constexpr Real TERRAIN_MIN_CLEARANCE = 0.01;

// ============================================================================
// Adaptive Integration
// ============================================================================

/// Default error tolerance for adaptive integrators
constexpr Real ADAPTIVE_TOLERANCE = 1e-6;

/// Minimum step size for adaptive integrators
constexpr Real ADAPTIVE_MIN_DT = 1e-9;

/// Maximum step size for adaptive integrators
constexpr Real ADAPTIVE_MAX_DT = 0.1;

/// Safety factor for step size adjustment
constexpr Real ADAPTIVE_SAFETY_FACTOR = 0.9;

/// Maximum step growth factor
constexpr Real ADAPTIVE_GROWTH_LIMIT = 2.0;

/// Minimum step shrink factor
constexpr Real ADAPTIVE_SHRINK_LIMIT = 0.1;

// ============================================================================
// Entity and Solver Limits
// ============================================================================

/// Maximum number of entities per simulation
constexpr SizeT MAX_ENTITIES = 100000;

/// Maximum constraint rows per constraint
constexpr int MAX_CONSTRAINT_ROWS = 6;

/// Default velocity iterations for constraint solver
constexpr int DEFAULT_VELOCITY_ITERATIONS = 10;

/// Default position iterations for constraint solver
constexpr int DEFAULT_POSITION_ITERATIONS = 3;

} // namespace jaguar::constants
```

### Using Constants

```cpp
#include "jaguar/core/constants.h"

// Configure physics engine
EngineConfig cfg;
cfg.time_step = constants::DEFAULT_TIME_STEP;
cfg.physics_threads = std::thread::hardware_concurrency();

// Configure adaptive integrator
AdaptiveConfig adaptive;
adaptive.tolerance = constants::ADAPTIVE_TOLERANCE;
adaptive.min_dt = constants::ADAPTIVE_MIN_DT;
adaptive.max_dt = constants::ADAPTIVE_MAX_DT;

// Constraint solver tuning
solver.set_baumgarte_factor(constants::DEFAULT_BAUMGARTE_FACTOR);
solver.set_slop(constants::DEFAULT_SLOP);
solver.set_warm_start_factor(constants::DEFAULT_WARM_START_FACTOR);
solver.set_iterations(constants::DEFAULT_VELOCITY_ITERATIONS,
                      constants::DEFAULT_POSITION_ITERATIONS);
```

---

## Engine API

**Header:** `jaguar/jaguar.h`

### Engine

```cpp
class Engine {
public:
    bool initialize();
    bool initialize(const config::EngineConfig& cfg);
    void shutdown();

    EntityId create_entity(const std::string& name, Domain domain);
    EntityId create_entity_from_config(const config::EntityConfig& cfg);
    void destroy_entity(EntityId id);

    physics::EntityState get_entity_state(EntityId id) const;
    void set_entity_state(EntityId id, const physics::EntityState& state);
    void apply_forces(EntityId id, const physics::EntityForces& forces);

    environment::Environment get_environment(EntityId id) const;
    environment::Environment get_environment_at(const Vec3& ecef) const;

    void step(Real dt);
    void run_for(Real duration);

    Real get_time() const;
    void set_time(Real time);

    SizeT entity_count() const;
};

// Version info
std::string GetVersionString();
int GetVersionMajor();
int GetVersionMinor();
int GetVersionPatch();
```

---

## See Also

- [Examples Guide](EXAMPLES.md) - Code examples
- [Configuration Guide](CONFIGURATION.md) - XML configuration
- [Module Documentation](modules/) - Detailed module docs
- [Contributing Guide](CONTRIBUTING.md) - Development guidelines
