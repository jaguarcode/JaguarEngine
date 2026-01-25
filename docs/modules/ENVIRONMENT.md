# Environment Documentation

The Environment module provides atmospheric, terrain, and ocean environment models for physics calculations across all domains.

## Headers

| Header | Purpose |
|--------|---------|
| `jaguar/environment/environment.h` | Combined environment state and service |
| `jaguar/environment/atmosphere.h` | US Standard Atmosphere 1976, weather |
| `jaguar/environment/terrain.h` | Terrain queries, surface materials |
| `jaguar/environment/ocean.h` | Ocean state, waves, currents |

## Components

### Environment

Combined environment state passed to force generators:

```cpp
struct Environment {
    // Position context
    Real latitude{0.0};           // rad
    Real longitude{0.0};          // rad
    Real altitude{0.0};           // m above WGS84
    Vec3 position_ecef{0, 0, 0};  // ECEF position

    // Atmosphere
    AtmosphereState atmosphere;

    // Terrain (if applicable)
    TerrainQuery terrain;
    Real terrain_elevation{0.0};  // Terrain height below entity

    // Ocean (if applicable)
    OceanState ocean;
    bool over_water{false};

    // Gravity
    Vec3 gravity{0, 0, 9.80665};  // Gravity vector in local frame
};
```

### EnvironmentService

Central service providing environment data to all physics systems:

```cpp
class EnvironmentService {
public:
    bool initialize();
    void shutdown();
    void update(Real dt);

    // Query environment at ECEF position
    Environment query(const Vec3& ecef, Real time) const;

    // Query at geodetic position
    Environment query_geodetic(Real lat, Real lon, Real alt, Real time) const;

    // Terrain integration (v0.6.0+)
    bool load_terrain(const std::string& path);
    TerrainQuery query_terrain(const Vec3& position_ecef) const;
    Real get_terrain_elevation(Real lat, Real lon) const;

    // Subsystem access
    TerrainManager& terrain();
    AtmosphereManager& atmosphere();
    OceanManager& ocean();

    void set_time(Real time);
};
```

**Usage:**
```cpp
EnvironmentService env_service;
env_service.initialize();

// Query environment at entity position
Vec3 ecef_pos = entity.get_position_ecef();
Real sim_time = simulation.get_time();

Environment env = env_service.query(ecef_pos, sim_time);

// Use in force calculations
aero_model.compute_forces(state, env, dt, forces);
```

## Atmosphere Subsystem

### AtmosphereState

Atmospheric conditions at a point:

```cpp
struct AtmosphereState {
    Real temperature{288.15};     // K
    Real pressure{101325.0};      // Pa
    Real density{1.225};          // kg/m³
    Real speed_of_sound{340.29};  // m/s
    Real viscosity{1.789e-5};     // Pa·s
    Vec3 wind{0, 0, 0};           // Wind velocity in NED (m/s)
    Real humidity{0.0};           // Relative humidity (0-1)
    Real visibility{10000.0};     // Visibility (m)
};
```

### StandardAtmosphere

US Standard Atmosphere 1976 model for altitudes 0-86 km:

```cpp
class StandardAtmosphere {
public:
    AtmosphereState get_state(Real altitude) const;
    Real get_temperature(Real altitude) const;
    Real get_pressure(Real altitude) const;
    Real get_density(Real altitude) const;
    Real get_speed_of_sound(Real altitude) const;

    // Altitude conversions
    static Real geometric_to_geopotential(Real h);
    static Real geopotential_to_geometric(Real H);
};
```

**Atmospheric Layers (US Standard 1976):**

| Layer | Altitude (km) | Name | T (K) | Lapse Rate (K/m) |
|-------|---------------|------|-------|------------------|
| 0 | 0-11 | Troposphere | 288.15 | -0.0065 |
| 1 | 11-20 | Tropopause | 216.65 | 0.0 (isothermal) |
| 2 | 20-32 | Stratosphere 1 | 216.65 | +0.001 |
| 3 | 32-47 | Stratosphere 2 | 228.65 | +0.0028 |
| 4 | 47-51 | Stratopause | 270.65 | 0.0 (isothermal) |
| 5 | 51-71 | Mesosphere 1 | 270.65 | -0.0028 |
| 6 | 71-85 | Mesosphere 2 | 214.65 | -0.002 |

**Physical Constants:**
```cpp
R_STAR = 8.31432       // Universal gas constant (J/(mol·K))
M0 = 28.9644e-3        // Mean molecular weight (kg/mol)
R_AIR = 287.053        // Gas constant for air (J/(kg·K))
GAMMA = 1.4            // Ratio of specific heats
G0 = 9.80665           // Standard gravity (m/s²)
R_EARTH = 6356766.0    // Effective Earth radius (m)
```

**Pressure Equations:**

Gradient layer (L ≠ 0):
```
P = P_base × (T / T_base)^(-g₀M₀ / R*L)
```

Isothermal layer (L = 0):
```
P = P_base × exp(-g₀M₀ΔH / R*T_base)
```

**Density (Ideal Gas Law):**
```
ρ = P / (R_AIR × T)
```

**Speed of Sound:**
```
a = √(γ × R_AIR × T)
```

**Dynamic Viscosity (Sutherland's Law):**
```
μ = μ_ref × (T/T_ref)^(3/2) × (T_ref + S) / (T + S)
```
Where: μ_ref = 1.716×10⁻⁵ Pa·s, T_ref = 273.15 K, S = 110.4 K

**Reference Values at Sea Level:**

| Property | Value | Unit |
|----------|-------|------|
| Temperature | 288.15 | K (15°C) |
| Pressure | 101325 | Pa |
| Density | 1.225 | kg/m³ |
| Speed of Sound | 340.29 | m/s |
| Viscosity | 1.789×10⁻⁵ | Pa·s |

**Usage:**
```cpp
StandardAtmosphere atmo;

// Get complete state at 10 km altitude
AtmosphereState state = atmo.get_state(10000.0);

// Individual queries
Real T = atmo.get_temperature(10000.0);  // ~223 K
Real P = atmo.get_pressure(10000.0);     // ~26,500 Pa
Real rho = atmo.get_density(10000.0);    // ~0.414 kg/m³
Real a = atmo.get_speed_of_sound(10000.0); // ~299 m/s
```

### WeatherModel

Weather effects overlaid on standard atmosphere:

```cpp
class WeatherModel {
public:
    // Wind layers
    void set_wind_layer(Real altitude, Real direction, Real speed);
    Vec3 get_wind(Real lat, Real lon, Real alt) const;

    // Rain effects
    void set_rain_rate(Real rate);  // mm/hour
    Real get_rain_attenuation(Real frequency_ghz) const;  // dB/km

    // Fog/visibility
    void set_fog_visibility(Real visibility_m);
    Real get_visibility(Real lat, Real lon, Real alt) const;
};
```

**Wind Layer Interpolation:**
- Wind layers stored by altitude
- Linear interpolation between layers
- Wind direction is "from" direction (meteorological convention)
- Converted to NED velocity vector

**Rain Attenuation (ITU-R P.838-3 Simplified):**
```
γ_R = k × R^α  (dB/km)
```

| Frequency (GHz) | k | α |
|-----------------|---|---|
| ≤1 | 3.87×10⁻⁵ | 0.912 |
| 1-10 | 0.0101×f^1.276 | 1.0 + 0.03f |
| >10 | 0.0367×f^0.5 | 1.13 |

**Usage:**
```cpp
WeatherModel weather;

// Set wind profile
weather.set_wind_layer(0.0, 270.0 * DEG_TO_RAD, 5.0);     // Surface: 5 m/s from W
weather.set_wind_layer(3000.0, 280.0 * DEG_TO_RAD, 15.0); // 3 km: 15 m/s from WNW
weather.set_wind_layer(9000.0, 300.0 * DEG_TO_RAD, 30.0); // 9 km: 30 m/s from NW

// Set precipitation
weather.set_rain_rate(10.0);  // 10 mm/hour (moderate rain)

// Query attenuation for radar
Real atten = weather.get_rain_attenuation(10.0);  // ~0.4 dB/km at 10 GHz
```

### AtmosphereManager

Combined atmosphere management:

```cpp
class AtmosphereManager {
public:
    AtmosphereState get_state(Real lat, Real lon, Real alt) const;

    const StandardAtmosphere& standard() const;
    WeatherModel& weather();

    void set_weather_enabled(bool enabled);
};
```

## Terrain Subsystem

### SurfaceType

Terrain surface material classification:

```cpp
enum class SurfaceType : UInt8 {
    Unknown,
    Asphalt,
    Concrete,
    Gravel,
    DrySand,
    WetSand,
    Clay,
    Mud,
    Grass,
    Snow,
    Ice,
    Water
};
```

### TerrainMaterial

Terrain surface properties:

```cpp
struct TerrainMaterial {
    SurfaceType type{SurfaceType::Unknown};
    Real friction_coefficient{0.7};
    Real rolling_resistance{0.01};

    // Bekker-Wong parameters
    Real k_c{0.0};      // Cohesive modulus (kN/m^(n+1))
    Real k_phi{0.0};    // Frictional modulus (kN/m^(n+2))
    Real n{1.0};        // Deformation exponent

    // Trafficability
    Real max_slope_deg{30.0};
    Real bearing_capacity{100000.0};  // Pa
};
```

**Default Material Properties:**

| Surface | μ (friction) | k_c | k_phi | n | Bearing (kPa) |
|---------|-------------|-----|-------|---|---------------|
| Asphalt | 0.85 | 10⁶ | 10⁶ | 0 | 10,000 |
| Concrete | 0.80 | 10⁶ | 10⁶ | 0 | 15,000 |
| Gravel | 0.60 | 50 | 2000 | 0.8 | 500 |
| Dry Sand | 0.40 | 0.99 | 1528 | 1.1 | 100 |
| Clay | 0.50 | 13.19 | 692 | 0.5 | 200 |
| Grass | 0.55 | 20 | 1000 | 0.9 | 150 |
| Snow | 0.20 | 4.37 | 196 | 1.6 | 50 |
| Ice | 0.10 | 10⁶ | 10⁶ | 0 | 500 |

### TerrainQuery

Result of terrain query at a position:

```cpp
struct TerrainQuery {
    Real elevation{0.0};        // Height above WGS84 ellipsoid (m)
    Vec3 normal{0, 0, 1};       // Surface normal (NED)
    Real slope_angle{0.0};      // Slope angle (rad)
    TerrainMaterial material;
    bool valid{false};
};
```

### TerrainManager

Central terrain data manager:

```cpp
class TerrainManager {
public:
    // Configuration
    void add_data_path(const std::string& path);
    void set_cache_size(SizeT mb);
    bool initialize();
    void shutdown();

    // Queries
    Real get_elevation(Real lat, Real lon) const;
    Real get_elevation_ecef(const Vec3& ecef) const;
    Vec3 get_surface_normal(Real lat, Real lon) const;
    Real get_slope_angle(Real lat, Real lon) const;
    TerrainMaterial get_material(Real lat, Real lon) const;
    TerrainQuery query(Real lat, Real lon) const;
    TerrainQuery query_ecef(const Vec3& ecef) const;

    // LOD management
    void set_focus_point(const Vec3& ecef);
    void set_detail_radius(Real radius_m);
    void update();

    // Statistics
    SizeT loaded_tile_count() const;
    SizeT cache_usage_bytes() const;
};
```

**Supported Data Formats:**
- DTED (Digital Terrain Elevation Data) Level 0, 1, 2
- GeoTIFF elevation rasters
- SRTM (Shuttle Radar Topography Mission) data

**Usage:**
```cpp
TerrainManager terrain;

// Configure data sources
terrain.add_data_path("/data/dted/");
terrain.add_data_path("/data/srtm/");
terrain.set_cache_size(512);  // 512 MB cache

terrain.initialize();

// Query terrain
Real lat = 37.5 * DEG_TO_RAD;
Real lon = -122.0 * DEG_TO_RAD;

TerrainQuery result = terrain.query(lat, lon);
if (result.valid) {
    Real elevation = result.elevation;
    Vec3 normal = result.normal;
    Real slope_deg = result.slope_angle * RAD_TO_DEG;

    // Check vehicle mobility
    if (slope_deg < result.material.max_slope_deg) {
        // Terrain passable
    }
}

// LOD management for moving entities
terrain.set_focus_point(entity_ecef);
terrain.set_detail_radius(50000.0);  // 50 km high-detail radius
terrain.update();  // Call each frame
```

### GDALTerrainLoader

GDAL-based terrain data loader for GeoTIFF and other formats:

```cpp
class GDALTerrainLoader {
public:
    static bool is_available();

    bool open(const std::string& path);
    void close();

    Real get_elevation(Real lat, Real lon) const;
    Vec3 get_normal(Real lat, Real lon) const;
    TerrainQuery query(Real lat, Real lon) const;
    void get_bounds(Real& min_lat, Real& max_lat,
                    Real& min_lon, Real& max_lon) const;
    Real get_resolution() const;
    bool is_loaded() const;
};
```

### Terrain Integration (v0.6.0+)

The Environment module now provides seamless terrain integration through the EnvironmentService, enabling direct loading and querying of GIS data without manual TerrainManager configuration.

#### EnvironmentService Terrain Methods

**Load Terrain Data**

```cpp
bool EnvironmentService::load_terrain(const std::string& path);
```

Loads terrain data from a GeoTIFF or other GDAL-supported raster format. This is a convenience method that handles dataset opening and integration with the internal TerrainManager.

- **Parameters:**
  - `path`: Absolute or relative path to terrain dataset file
- **Returns:** `true` if successfully loaded, `false` on error
- **Supports:** GeoTIFF, SRTM, DTED, and other GDAL-compatible formats
- **Thread-safe:** Safe to call from multiple threads after initialization
- **Error handling:** Logs errors via standard engine logging; returns false on failure

**Query Terrain at ECEF Position**

```cpp
TerrainQuery EnvironmentService::query_terrain(const Vec3& position_ecef) const;
```

Performs a complete terrain query at an ECEF coordinate, returning elevation, surface normal, slope angle, and material properties.

- **Parameters:**
  - `position_ecef`: Position in Earth-Centered Earth-Fixed coordinates (meters)
- **Returns:** `TerrainQuery` structure with all terrain properties
- **Elevation:** Height above WGS84 ellipsoid in meters
- **Normal:** Surface normal in NED frame (for slope calculations and contact normals)
- **Slope:** Angle in radians between 0 and π/2
- **Material:** Surface properties including friction, bearing capacity, and terramechanics parameters
- **Validity:** Check `TerrainQuery::valid` flag before using results

**Get Terrain Elevation at Geodetic Position**

```cpp
Real EnvironmentService::get_terrain_elevation(Real lat, Real lon) const;
```

Convenience method for querying only elevation at a geodetic position. Useful when full terrain properties are not needed.

- **Parameters:**
  - `lat`: Latitude in radians (-π/2 to π/2)
  - `lon`: Longitude in radians (-π to π)
- **Returns:** Elevation in meters above WGS84 ellipsoid, or NaN if unavailable
- **Performance:** Faster than full terrain query for elevation-only use cases

#### Terrain Integration Examples

**Basic Terrain Loading and Querying**

```cpp
EnvironmentService env_service;
env_service.initialize();

// Load GeoTIFF elevation dataset
if (!env_service.load_terrain("/data/elevation_model.tif")) {
    std::cerr << "Failed to load terrain data\n";
    return false;
}

// Query terrain at entity position
Vec3 entity_ecef = entity.get_position_ecef();
TerrainQuery terrain = env_service.query_terrain(entity_ecef);

if (terrain.valid) {
    Real elevation = terrain.elevation;
    Real slope = terrain.slope_angle * RAD_TO_DEG;

    // Use for physics calculations
    if (slope < terrain.material.max_slope_deg) {
        // Terrain passable for ground vehicle
    }
}
```

**Terrain-Aware Physics Simulation**

```cpp
class GroundVehicle {
public:
    void update(const EnvironmentService& env, Real dt) {
        // Get terrain at vehicle position
        TerrainQuery terrain = env.query_terrain(position_ecef_);

        if (!terrain.valid) {
            return;  // No terrain data available
        }

        // Update height above ground (AGL)
        Real altitude_agl = altitude_msl_ - terrain.elevation;

        // Check slope passability
        Real slope_deg = terrain.slope_angle * RAD_TO_DEG;
        if (slope_deg > terrain.material.max_slope_deg) {
            // Stop movement - slope too steep
            velocity_ned_ = Vec3{0, 0, 0};
            return;
        }

        // Update terramechanics with soil properties
        terramechanics_.set_soil_params(
            terrain.material.k_c,
            terrain.material.k_phi,
            terrain.material.n
        );

        // Compute terrain interaction forces
        Vec3 normal = terrain.normal;
        Real friction = terrain.material.friction_coefficient;

        // Continue normal physics calculation...
    }

private:
    Vec3 position_ecef_;
    Real altitude_msl_;
    Vec3 velocity_ned_;
    TerramechanicsModel terramechanics_;
};
```

**Environment Query with Terrain Data**

```cpp
// Query complete environment including terrain
Environment env = env_service.query(entity_ecef, sim_time);

// Terrain is included in environment structure
if (env.terrain.valid) {
    // Atmospheric conditions
    Real air_density = env.atmosphere.density;
    Vec3 wind = env.atmosphere.wind;

    // Terrain data
    Real elevation = env.terrain.elevation;
    TerrainMaterial mat = env.terrain.material;

    // Ocean data (if applicable)
    if (env.over_water) {
        Real wave_height = env.ocean.surface_elevation;
    } else {
        // Ground contact - use terrain normal and friction
        Vec3 ground_normal = env.terrain.normal;
        Real friction_coeff = mat.friction_coefficient;
    }
}
```

**Multi-Source Terrain Loading**

```cpp
EnvironmentService env_service;
env_service.initialize();

// Load primary elevation model
env_service.load_terrain("/data/srtm_dem.tif");

// Add coarser fallback data
env_service.terrain().add_data_path("/data/srtm_level0/");

// Set up caching for performance
env_service.terrain().set_cache_size(512);  // 512 MB cache

// Update LOD around focus point (e.g., player position)
env_service.terrain().set_focus_point(player_ecef);
env_service.terrain().set_detail_radius(50000.0);  // 50 km radius

// Frame update
void update_frame(Real dt) {
    env_service.update(dt);
    env_service.terrain().update();  // Update LOD paging
}
```

#### Thread Safety

All terrain query methods are thread-safe after initialization:
- **TerrainManager**: Read-only queries are thread-safe
- **GDALTerrainLoader**: Read queries thread-safe after `open()` completes
- **EnvironmentService**: All const methods thread-safe for concurrent reads

Configuration methods (loading, cache size) are NOT thread-safe and should only be called during initialization.

#### Terrain Integration with Environment Queries

When using `EnvironmentService::query()` or `query_geodetic()`, the returned `Environment` structure includes complete terrain data:

```cpp
struct Environment {
    // Position context
    Real latitude{0.0};           // rad
    Real longitude{0.0};          // rad
    Real altitude{0.0};           // m above WGS84
    Vec3 position_ecef{0, 0, 0};  // ECEF position

    // Atmosphere
    AtmosphereState atmosphere;

    // Terrain - automatically populated if data loaded
    TerrainQuery terrain;
    Real terrain_elevation{0.0};  // Terrain height below entity

    // Ocean
    OceanState ocean;
    bool over_water{false};

    // Gravity
    Vec3 gravity{0, 0, 9.80665};  // Gravity vector in local frame
};
```

The `terrain` field contains:
- **elevation**: Height above WGS84 ellipsoid (meters)
- **normal**: Surface normal in NED frame (for contact normals)
- **slope_angle**: Slope magnitude in radians
- **material**: Surface type and terramechanics parameters
- **valid**: Flag indicating whether terrain data is available

#### Material Properties for Surface Interactions

Terrain materials include properties for both kinematic and dynamic friction modeling:

```cpp
struct TerrainMaterial {
    SurfaceType type{SurfaceType::Unknown};
    Real friction_coefficient{0.7};      // Kinetic friction (Coulomb)
    Real rolling_resistance{0.01};       // Rolling resistance coefficient

    // Bekker-Wong terramechanics
    Real k_c{0.0};      // Cohesive modulus (kN/m^(n+1))
    Real k_phi{0.0};    // Frictional modulus (kN/m^(n+2))
    Real n{1.0};        // Deformation exponent

    // Trafficability
    Real max_slope_deg{30.0};           // Maximum traversable slope (degrees)
    Real bearing_capacity{100000.0};    // Bearing capacity (Pa)
};
```

These properties enable:
- **Tire/Track Friction**: `friction_coefficient` for normal friction calculations
- **Sinkage Estimation**: Bekker parameters for deformation predictions
- **Bearing Failure Detection**: Checking `bearing_capacity` against vehicle weight
- **Slope Trafficability**: Comparing actual slope against `max_slope_deg`
- **Rolling Resistance**: Energy loss for wheeled vehicles using `rolling_resistance`

**Example: Terramechanics Integration**

```cpp
TerrainQuery terrain = env.query_terrain(vehicle_ecef);

// Calculate sinkage using Bekker model
Real contact_pressure = vehicle_weight / contact_area;
if (contact_pressure > terrain.material.bearing_capacity) {
    // Vehicle will sink or get stuck
    mobility_status = MobilityStatus::BOGGED;
}

// Adjust tire rolling resistance
Real rr_force = vehicle_weight * terrain.material.rolling_resistance;

// Calculate maximum steering friction
Real max_friction_force = normal_force * terrain.material.friction_coefficient;
```

## Ocean Subsystem

### OceanState

Ocean conditions at a point:

```cpp
struct OceanState {
    Real water_depth{1000.0};     // Water depth (m)
    Real surface_elevation{0.0};  // Wave surface elevation (m)
    Vec3 current{0, 0, 0};        // Ocean current (m/s)
    Real temperature{15.0};       // Water temperature (°C)
    Real salinity{35.0};          // Salinity (ppt)
    Real density{1025.0};         // Water density (kg/m³)
};
```

### OceanManager

Ocean environment management:

```cpp
class OceanManager {
public:
    // Sea state
    void set_sea_state(const domain::sea::SeaState& state);
    const domain::sea::SeaState& get_sea_state() const;
    OceanState get_state(Real lat, Real lon, Real time) const;

    // Wave queries
    Real get_wave_elevation(Real x, Real y, Real time) const;
    Vec3 get_wave_slope(Real x, Real y, Real time) const;
    Vec3 get_particle_velocity(Real x, Real y, Real z, Real time) const;

    // Ocean current
    void set_current(const Vec3& current);
    Vec3 get_current(Real lat, Real lon, Real depth) const;

    // Water properties
    Real get_density(Real lat, Real lon, Real depth) const;

    void update(Real dt);
};
```

**Water Density Model:**

UNESCO equation of state (simplified):
```
ρ = ρ₀ + ΔρT + Δρs + Δρp
```

Where:
- ρ₀ = 999.842 kg/m³ (pure water at 4°C)
- ΔρT = temperature effect
- Δρs = salinity effect
- Δρp = pressure (depth) effect

**Typical Values:**

| Parameter | Surface | 1000m Depth |
|-----------|---------|-------------|
| Temperature | 15-25°C | 4-5°C |
| Salinity | 35 ppt | 34.7 ppt |
| Density | 1025 kg/m³ | 1028 kg/m³ |

**Usage:**
```cpp
OceanManager ocean;

// Set sea conditions
domain::sea::SeaState sea = domain::sea::SeaState::FromNATOSeaState(5);
ocean.set_sea_state(sea);

// Set current
ocean.set_current(Vec3{0.5, 0.2, 0.0});  // 0.5 m/s N, 0.2 m/s E

// Query ocean at ship position
Real time = simulation.get_time();
OceanState state = ocean.get_state(ship_lat, ship_lon, time);

// Wave surface at ship position
Real wave_height = ocean.get_wave_elevation(ship_x, ship_y, time);

// Underwater current at periscope depth
Vec3 current = ocean.get_current(sub_lat, sub_lon, -20.0);
```

## Complete Example

```cpp
#include <jaguar/jaguar.h>

class SimulationWorld {
public:
    SimulationWorld() {
        // Initialize environment service
        env_service_.initialize();

        // Load terrain data (v0.6.0+)
        if (!env_service_.load_terrain("/data/elevation_models/srtm_dem.tif")) {
            std::cerr << "Warning: Failed to load terrain data\n";
        }

        // Configure terrain caching and LOD
        env_service_.terrain().add_data_path("/data/terrain/dted/");
        env_service_.terrain().set_cache_size(512);

        // Configure weather
        auto& weather = env_service_.atmosphere().weather();
        weather.set_wind_layer(0.0, 270.0 * DEG_TO_RAD, 5.0);
        weather.set_wind_layer(5000.0, 280.0 * DEG_TO_RAD, 20.0);

        // Configure ocean
        domain::sea::SeaState sea;
        sea.significant_height = 2.0;
        sea.peak_period = 8.0;
        sea.spectrum = domain::sea::WaveSpectrum::JONSWAP;
        env_service_.ocean().set_sea_state(sea);
    }

    void update(Real dt) {
        env_service_.update(dt);
        env_service_.set_time(sim_time_);

        // Update terrain LOD around player
        env_service_.terrain().set_focus_point(player_ecef_);
        env_service_.terrain().update();

        sim_time_ += dt;
    }

    Environment get_environment_at(const Vec3& ecef) const {
        return env_service_.query(ecef, sim_time_);
    }

    // Utility: Check if position is over water
    bool is_over_water(Real lat, Real lon) const {
        TerrainQuery terrain = env_service_.terrain().query(lat, lon);
        if (!terrain.valid) return true;  // Assume water if no data

        // Check surface type
        return terrain.material.type == SurfaceType::Water;
    }

    // Utility: Direct terrain query (v0.6.0+)
    TerrainQuery get_terrain_at(const Vec3& ecef) const {
        return env_service_.query_terrain(ecef);
    }

    // Utility: Get elevation at position (v0.6.0+)
    Real get_elevation_at(Real lat, Real lon) const {
        return env_service_.get_terrain_elevation(lat, lon);
    }

    // Utility: Get wind at altitude
    Vec3 get_wind(Real lat, Real lon, Real alt) const {
        AtmosphereState atmo = env_service_.atmosphere().get_state(lat, lon, alt);
        return atmo.wind;
    }

private:
    EnvironmentService env_service_;
    Real sim_time_{0.0};
    Vec3 player_ecef_{0, 0, 0};
};

// Entity using environment
class Aircraft {
public:
    void update(const SimulationWorld& world, Real dt) {
        // Get environment at aircraft position
        Environment env = world.get_environment_at(position_ecef_);

        // Aerodynamic calculations use:
        // - env.atmosphere.density for lift/drag
        // - env.atmosphere.speed_of_sound for Mach number
        // - env.atmosphere.wind for relative airspeed

        Real rho = env.atmosphere.density;
        Real a = env.atmosphere.speed_of_sound;
        Vec3 wind = env.atmosphere.wind;

        // Compute relative velocity (airspeed)
        Vec3 V_air = velocity_ned_ - wind;
        Real airspeed = V_air.norm();
        Real mach = airspeed / a;

        // Dynamic pressure
        Real qbar = 0.5 * rho * airspeed * airspeed;

        // ... compute forces ...
    }

private:
    Vec3 position_ecef_;
    Vec3 velocity_ned_;
};

// Ground vehicle using terrain
class Tank {
public:
    void update(const SimulationWorld& world, Real dt) {
        Environment env = world.get_environment_at(position_ecef_);

        // Check terrain below vehicle
        if (env.terrain.valid) {
            // Terrain-vehicle interaction
            Real terrain_height = env.terrain.elevation;
            Vec3 normal = env.terrain.normal;
            Real slope = env.terrain.slope_angle;

            // Get soil properties for terramechanics
            TerrainMaterial& mat = env.terrain.material;
            terra_.set_soil(mat.k_c, mat.k_phi, mat.n);

            // Check mobility
            if (slope > mat.max_slope_deg * DEG_TO_RAD) {
                // Slope too steep
            }

            // Calculate bearing capacity margin
            Real contact_pressure = weight_ / contact_area_;
            if (contact_pressure > mat.bearing_capacity) {
                // Vehicle sinking into soft terrain
                mobility_status_ = MobilityStatus::Bogged;
            }

            // Friction-based force limiting
            Real max_traction = normal_force_ * mat.friction_coefficient;
            if (desired_traction_force_ > max_traction) {
                // Wheels slipping - reduce acceleration
                traction_force_ = max_traction;
            }
        }
    }

private:
    Vec3 position_ecef_;
    Real weight_;
    Real contact_area_;
    Real normal_force_;
    Real desired_traction_force_;
    Real traction_force_;
    MobilityStatus mobility_status_;
    TerramechanicsModel terra_;
};

// Ship using ocean
class Ship {
public:
    void update(const SimulationWorld& world, Real dt) {
        Environment env = world.get_environment_at(position_ecef_);

        if (env.over_water) {
            // Wave surface at ship position
            Real wave_elev = env.ocean.surface_elevation;

            // Ocean current
            Vec3 current = env.ocean.current;

            // Water density for buoyancy
            Real rho_water = env.ocean.density;

            // Compute hydrodynamic forces
            // ...
        }
    }

private:
    Vec3 position_ecef_;
};
```

## Coordinate Conventions

### Position Representations

| System | Components | Usage |
|--------|------------|-------|
| ECEF | X, Y, Z (m) | Internal calculations, physics |
| Geodetic | lat, lon (rad), alt (m) | User interface, environment queries |
| NED | N, E, D (m) | Local navigation, forces |

### Altitude Reference

- **Geometric altitude**: Height above WGS84 ellipsoid (used in atmosphere model)
- **Geopotential altitude**: Used internally in US Standard Atmosphere 1976
- **Terrain elevation**: Height above WGS84 at terrain surface
- **Height AGL**: Altitude - Terrain elevation

### Conversion:
```cpp
// Geometric to geopotential
H = (r × h) / (r + h)

// Geopotential to geometric
h = (r × H) / (r - H)

// Where r = 6356766 m (effective Earth radius)
```

## Performance Considerations

### Terrain System
- Tile-based paging with LRU cache
- LOD management based on distance from focus
- Async loading for smooth performance
- Typical cache size: 256-512 MB

### Atmosphere
- Standard atmosphere: O(1) layer lookup
- Weather interpolation: O(n) where n = wind layers
- Pre-computed layer pressures for efficiency

### Ocean
- Wave superposition: O(n) where n = wave components (typically 30)
- Current interpolation: O(1) for uniform current

### Update Frequency

| Subsystem | Typical Rate | Notes |
|-----------|-------------|-------|
| Terrain | 1-10 Hz | LOD updates only |
| Atmosphere | 100 Hz | Per-entity, with physics |
| Ocean | 50-100 Hz | Wave animation |

## References

- **US Standard Atmosphere 1976**: NOAA/NASA/USAF Technical Report
- **Sutherland's Law**: White - "Viscous Fluid Flow"
- **ITU-R P.838-3**: Rain attenuation model for radio propagation
- **Bekker-Wong Terramechanics**: Wong - "Theory of Ground Vehicles"
- **UNESCO Equation of State**: UNESCO Technical Papers in Marine Science No. 44
