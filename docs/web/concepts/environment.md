# Environment

The Environment system provides atmospheric, terrain, and ocean data for physics calculations. This guide covers environmental services and queries.

## Overview

JaguarEngine's environment system includes:

- **Atmosphere**: Temperature, pressure, density, wind
- **Terrain**: Elevation, surface normals, soil types
- **Ocean**: Waves, currents, sea state

## Environment Structure

```cpp
struct Environment {
    Real latitude{0.0};            // rad
    Real longitude{0.0};           // rad
    Real altitude{0.0};            // m above WGS84

    AtmosphereState atmosphere;
    TerrainQuery terrain;
    Real terrain_elevation{0.0};
    OceanState ocean;
    bool over_water{false};
    Vec3 gravity{0, 0, 9.80665};
};
```

## Atmosphere

### Atmosphere State

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

### Standard Atmosphere (US Std 1976)

| Altitude (km) | Temperature (K) | Pressure (Pa) | Density (kg/m³) |
|---------------|-----------------|---------------|-----------------|
| 0 | 288.15 | 101325 | 1.225 |
| 11 | 216.65 | 22632 | 0.3639 |
| 20 | 216.65 | 5474.9 | 0.0880 |
| 32 | 228.65 | 868.02 | 0.0132 |

### Using Atmosphere Data

```cpp
// Get environment at entity location
auto env = engine.get_environment(aircraft);

// Access atmosphere
Real density = env.atmosphere.density;
Real mach = airspeed / env.atmosphere.speed_of_sound;
Real dynamic_pressure = 0.5 * density * airspeed * airspeed;

// Wind components (NED)
Vec3 wind = env.atmosphere.wind;
```

### Weather Effects

```cpp
// Access weather model
auto& atm_mgr = engine.get_environment_service().atmosphere();

// Set wind layer
atm_mgr.weather().set_wind_layer(
    5000.0,                    // altitude (m)
    45.0 * DEG_TO_RAD,        // direction (from)
    15.0                       // speed (m/s)
);

// Set rain/fog
atm_mgr.weather().set_rain_rate(10.0);        // mm/hour
atm_mgr.weather().set_fog_visibility(1000.0); // m
```

## Terrain

### Terrain Query

```cpp
struct TerrainQuery {
    Real elevation{0.0};         // m above WGS84
    Vec3 normal{0, 0, 1};        // Surface normal (NED)
    Real slope_angle{0.0};       // rad
    TerrainMaterial material;
    bool valid{false};
};
```

### Querying Terrain

```cpp
auto& terrain_mgr = engine.get_environment_service().terrain();

// Get elevation at position
Real elev = terrain_mgr.get_elevation(latitude, longitude);

// Get surface normal
Vec3 normal = terrain_mgr.get_surface_normal(latitude, longitude);

// Get slope angle
Real slope = terrain_mgr.get_slope_angle(latitude, longitude);

// Full query
TerrainQuery query = terrain_mgr.query(latitude, longitude);
if (query.valid) {
    // Use terrain data
}
```

### Terrain Data Sources

| Format | Description | Resolution |
|--------|-------------|------------|
| DTED Level 0 | 30 arc-sec | ~900m |
| DTED Level 1 | 3 arc-sec | ~90m |
| DTED Level 2 | 1 arc-sec | ~30m |
| GeoTIFF | Custom DEM | Variable |

### Terrain Configuration

```xml
<terrain enabled="true">
    <cache_mb>2048</cache_mb>
    <tile_size>256</tile_size>
    <data_path>/data/dted/</data_path>
    <data_path>/data/srtm/</data_path>
</terrain>
```

### Terrain Materials

Soil properties for terramechanics:

```cpp
// Get material at position
TerrainMaterial mat = terrain_mgr.get_material(lat, lon);

// Access soil properties
SoilProperties soil = mat.get_soil_properties();
Real friction = soil.get_friction_coefficient();
```

## Ocean

### Ocean State

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

### Sea State Configuration

```cpp
// Set sea conditions
auto& ocean_mgr = engine.get_environment_service().ocean();

domain::sea::SeaState sea_state;
sea_state.significant_height = 3.0;  // m
sea_state.peak_period = 9.0;         // s
sea_state.direction = 45.0 * DEG_TO_RAD;  // from NE
sea_state.spectrum = WaveSpectrum::JONSWAP;

ocean_mgr.set_sea_state(sea_state);
```

### Wave Queries

```cpp
// Get wave elevation
Real elevation = ocean_mgr.get_wave_elevation(x, y, time);

// Get wave slope
Vec3 slope = ocean_mgr.get_wave_slope(x, y, time);

// Get particle velocity (for submerged objects)
Vec3 vel = ocean_mgr.get_particle_velocity(x, y, z, time);
```

### Ocean Currents

```cpp
// Set uniform current
ocean_mgr.set_current(Vec3{0.5, 0.0, 0.0});  // 0.5 m/s east

// Query current at depth
Vec3 current = ocean_mgr.get_current(lat, lon, depth);
```

## Environment Service

### Initialization

```cpp
// Access environment service
auto& env_service = engine.get_environment_service();

// Initialize (called automatically by engine)
env_service.initialize();

// Update environment state
env_service.update(dt);
```

### Querying by Position

```cpp
// Query at ECEF position
Vec3 ecef_pos{6378137.0, 0.0, 0.0};
Environment env = env_service.query(ecef_pos, time);

// Query at geodetic position
Environment env = env_service.query_geodetic(
    lat, lon, alt, time
);
```

### Performance Optimization

```cpp
// Set terrain focus for LOD
auto& terrain = env_service.terrain();
terrain.set_focus_point(camera_ecef);
terrain.set_detail_radius(50000.0);  // 50 km high detail

// Monitor cache usage
SizeT tiles = terrain.loaded_tile_count();
SizeT bytes = terrain.cache_usage_bytes();
```

## Configuration

### Engine Configuration

```xml
<engine_config>
    <terrain enabled="true">
        <cache_mb>2048</cache_mb>
        <data_path>/data/terrain/</data_path>
    </terrain>

    <atmosphere>
        <model>us_standard_1976</model>
        <weather_enabled>true</weather_enabled>
    </atmosphere>

    <ocean enabled="true">
        <default_sea_state>4</default_sea_state>
    </ocean>
</engine_config>
```

## Best Practices

### Environment Queries

1. **Cache results**: Don't query every frame if position hasn't changed
2. **Use entity queries**: `get_environment(entity_id)` is optimized
3. **Batch queries**: Group nearby position queries

### Terrain Data

1. **Set appropriate cache size**: Larger cache = fewer disk reads
2. **Use focus point**: Prioritize detail near areas of interest
3. **Match resolution to need**: Use coarser data for distant entities

### Weather Effects

1. **Update gradually**: Sudden weather changes can cause instabilities
2. **Consider performance**: Complex weather adds computational cost
3. **Test edge cases**: High winds, extreme temperatures

## See Also

- [Entities](entities.md) - Entity management
- [Force Generators](force-generators.md) - Physics models
- [Configuration](../api/configuration.md) - Configuration options
- [API Reference](../api/environment.md) - Environment API
