# Environment API Reference

Atmosphere, terrain, ocean, and environmental systems.

**Headers:** `jaguar/environment/environment.h`, `jaguar/environment/atmosphere.h`, `jaguar/environment/terrain.h`, `jaguar/environment/ocean.h`

## Environment

Complete environmental state at a location.

```cpp
struct Environment {
    // Position
    Real latitude{0.0};            // rad
    Real longitude{0.0};           // rad
    Real altitude{0.0};            // m above WGS84 ellipsoid
    Vec3 position_ecef{0, 0, 0};

    // Atmosphere
    AtmosphereState atmosphere;

    // Terrain
    TerrainQuery terrain;
    Real terrain_elevation{0.0};   // m above WGS84

    // Ocean
    OceanState ocean;
    bool over_water{false};

    // Gravity
    Vec3 gravity{0, 0, 9.80665};   // Local gravity vector (NED)

    // Time
    Real time{0.0};                // Simulation time
    Real julian_date{0.0};         // For sun position
};
```

### Example

```cpp
// Get environment at entity location
Environment env = engine.get_environment(entity_id);

// Or at arbitrary position
Vec3 ecef = transforms::geodetic_to_ecef(lat, lon, alt);
Environment env = engine.get_environment_at(ecef);

// Use environment data
Real temperature = env.atmosphere.temperature;
Real terrain_height = env.terrain.elevation;
bool is_water = env.over_water;
```

## EnvironmentService

Central environment management service.

```cpp
class EnvironmentService {
public:
    // Lifecycle
    bool initialize();
    void shutdown();
    void update(Real dt);

    // Query environment
    Environment query(const Vec3& ecef, Real time) const;
    Environment query_geodetic(Real lat, Real lon, Real alt, Real time) const;

    // Access managers
    TerrainManager& terrain();
    AtmosphereManager& atmosphere();
    OceanManager& ocean();

    // Time management
    void set_time(Real time);
    Real get_time() const;
    void set_julian_date(Real jd);
    Real get_julian_date() const;

    // Sun position
    Vec3 get_sun_direction() const;  // Unit vector toward sun (ECEF)
    Real get_solar_flux() const;     // W/m² at Earth distance
};
```

## AtmosphereState

Atmospheric properties at a point.

```cpp
struct AtmosphereState {
    Real temperature{288.15};      // K
    Real pressure{101325.0};       // Pa
    Real density{1.225};           // kg/m³
    Real speed_of_sound{340.29};   // m/s
    Real viscosity{1.789e-5};      // Pa·s (dynamic)
    Vec3 wind{0, 0, 0};            // m/s (NED frame)
    Real humidity{0.0};            // 0-1 relative humidity
    Real visibility{10000.0};      // m
};
```

## StandardAtmosphere

ISA (International Standard Atmosphere) model.

```cpp
class StandardAtmosphere {
public:
    // Query at altitude
    AtmosphereState get_state(Real altitude) const;

    // Individual properties
    Real get_temperature(Real altitude) const;    // K
    Real get_pressure(Real altitude) const;       // Pa
    Real get_density(Real altitude) const;        // kg/m³
    Real get_speed_of_sound(Real altitude) const; // m/s

    // Altitude conversions
    static Real geometric_to_geopotential(Real h);
    static Real geopotential_to_geometric(Real H);

    // Pressure altitude
    static Real pressure_altitude(Real pressure);
    static Real density_altitude(Real density);
};
```

### Example

```cpp
StandardAtmosphere isa;

// Get properties at 10 km
Real alt = 10000.0;
auto state = isa.get_state(alt);

std::cout << "Temperature: " << state.temperature << " K\n";
std::cout << "Pressure: " << state.pressure << " Pa\n";
std::cout << "Density: " << state.density << " kg/m³\n";
std::cout << "Speed of sound: " << state.speed_of_sound << " m/s\n";
```

## WeatherModel

Dynamic weather effects.

```cpp
class WeatherModel {
public:
    // Wind layers
    void set_wind_layer(Real altitude, Real direction, Real speed);
    void add_wind_layer(Real altitude, Real direction, Real speed);
    void clear_wind_layers();
    Vec3 get_wind(Real lat, Real lon, Real alt) const;

    // Turbulence
    void set_turbulence_intensity(Real intensity);  // 0-1
    Vec3 get_turbulence(Real lat, Real lon, Real alt, Real time) const;

    // Precipitation
    void set_rain_rate(Real rate);  // mm/hour
    Real get_rain_attenuation(Real frequency_ghz) const;  // dB/km

    // Visibility
    void set_fog_visibility(Real visibility_m);
    Real get_visibility(Real lat, Real lon, Real alt) const;

    // Temperature deviation from ISA
    void set_isa_deviation(Real delta_t);  // K
};
```

### Example

```cpp
auto& weather = engine.get_environment_service().atmosphere().weather();

// Set wind layers
weather.set_wind_layer(0, 270.0 * DEG_TO_RAD, 5.0);     // Surface: 5 m/s from W
weather.set_wind_layer(1000, 280.0 * DEG_TO_RAD, 15.0); // 1 km: 15 m/s
weather.set_wind_layer(5000, 290.0 * DEG_TO_RAD, 30.0); // 5 km: 30 m/s

// Get interpolated wind
Vec3 wind = weather.get_wind(lat, lon, 2500.0);

// Enable turbulence
weather.set_turbulence_intensity(0.3);
```

## AtmosphereManager

Complete atmosphere management.

```cpp
class AtmosphereManager {
public:
    // Query
    AtmosphereState get_state(Real lat, Real lon, Real alt) const;

    // Access components
    const StandardAtmosphere& standard() const;
    WeatherModel& weather();

    // Enable/disable features
    void set_weather_enabled(bool enabled);
    bool is_weather_enabled() const;

    // Upper atmosphere (for space domain)
    Real get_density_jb08(Real lat, Real lon, Real alt, Real jd) const;
};
```

## TerrainQuery

Terrain query result.

```cpp
struct TerrainQuery {
    Real elevation{0.0};         // m above WGS84
    Vec3 normal{0, 0, 1};        // Surface normal (NED)
    Real slope_angle{0.0};       // rad
    TerrainMaterial material;
    bool valid{false};           // True if terrain data available
};

struct TerrainMaterial {
    enum class Type {
        Unknown, Rock, Sand, Clay, Snow, Water, Forest, Urban, Road
    };

    Type type{Type::Unknown};
    std::string name;
    Real friction{0.5};
    Real vegetation_density{0.0};
};
```

## TerrainManager

Terrain data management and queries.

```cpp
class TerrainManager {
public:
    // Configuration
    void add_data_path(const std::string& path);
    void set_cache_size(SizeT mb);

    // Lifecycle
    bool initialize();
    void shutdown();

    // Queries
    Real get_elevation(Real lat, Real lon) const;
    Vec3 get_surface_normal(Real lat, Real lon) const;
    Real get_slope_angle(Real lat, Real lon) const;
    TerrainMaterial get_material(Real lat, Real lon) const;
    TerrainQuery query(Real lat, Real lon) const;

    // Line of sight
    bool has_line_of_sight(const Vec3& from_ecef,
                           const Vec3& to_ecef) const;

    // LOD management
    void set_focus_point(const Vec3& ecef);
    void set_detail_radius(Real radius_m);
    void update();

    // Status
    SizeT loaded_tile_count() const;
    SizeT cache_usage_bytes() const;
    bool has_coverage(Real lat, Real lon) const;
};
```

### Example

```cpp
auto& terrain = engine.get_environment_service().terrain();

// Add data paths
terrain.add_data_path("/data/dted/");
terrain.add_data_path("/data/srtm/");
terrain.set_cache_size(4096);  // 4 GB cache

// Initialize
terrain.initialize();

// Query elevation
Real lat = 37.0 * DEG_TO_RAD;
Real lon = -122.0 * DEG_TO_RAD;
Real elevation = terrain.get_elevation(lat, lon);

// Full query
TerrainQuery query = terrain.query(lat, lon);
if (query.valid) {
    std::cout << "Elevation: " << query.elevation << " m\n";
    std::cout << "Slope: " << query.slope_angle * RAD_TO_DEG << " deg\n";
    std::cout << "Material: " << query.material.name << "\n";
}

// Set LOD focus
terrain.set_focus_point(player_position);
terrain.set_detail_radius(50000.0);  // 50 km high detail

// Check line of sight
bool los = terrain.has_line_of_sight(observer_ecef, target_ecef);
```

## OceanState

Ocean properties at a point.

```cpp
struct OceanState {
    Real water_depth{1000.0};      // m
    Real surface_elevation{0.0};   // m (wave height)
    Vec3 current{0, 0, 0};         // m/s
    Real temperature{15.0};        // °C
    Real salinity{35.0};           // ppt
    Real density{1025.0};          // kg/m³
    Real sound_speed{1500.0};      // m/s
};
```

## OceanManager

Ocean environment management.

```cpp
class OceanManager {
public:
    // Sea state
    void set_sea_state(const domain::sea::SeaState& state);
    const domain::sea::SeaState& get_sea_state() const;

    // Query
    OceanState get_state(Real lat, Real lon, Real time) const;

    // Wave surface
    Real get_wave_elevation(Real x, Real y, Real time) const;
    Vec3 get_wave_slope(Real x, Real y, Real time) const;
    Vec3 get_particle_velocity(Real x, Real y, Real z, Real time) const;

    // Currents
    void set_current(const Vec3& current);
    Vec3 get_current(Real lat, Real lon, Real depth) const;

    // Properties
    Real get_density(Real lat, Real lon, Real depth) const;
    Real get_sound_speed(Real lat, Real lon, Real depth) const;

    // Update
    void update(Real dt);
};
```

### Example

```cpp
auto& ocean = engine.get_environment_service().ocean();

// Set sea state
auto sea = domain::sea::SeaState::FromNATOSeaState(4);
ocean.set_sea_state(sea);

// Set current
ocean.set_current(Vec3{0.5, 0.0, 0.0});  // 0.5 m/s eastward

// Query wave height
Real wave_height = ocean.get_wave_elevation(ship_x, ship_y, sim_time);

// Get water properties for sonar
Real sound_speed = ocean.get_sound_speed(lat, lon, -50.0);  // 50m depth
```

## See Also

- [Core API](core.md) - Coordinate transforms
- [Air Domain API](air.md) - Atmosphere integration
- [Land Domain API](land.md) - Terrain integration
- [Sea Domain API](sea.md) - Ocean integration
- [Concepts: Environment](../concepts/environment.md) - Environment system overview

