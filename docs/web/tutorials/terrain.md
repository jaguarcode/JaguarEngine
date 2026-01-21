# Terrain Tutorial

This tutorial demonstrates how to work with terrain data in JaguarEngine.

## Overview

JaguarEngine's terrain system supports:

- **DTED**: Digital Terrain Elevation Data (Levels 0-2)
- **SRTM**: Shuttle Radar Topography Mission
- **GeoTIFF**: General raster elevation data
- **Quadtree LOD**: Level-of-detail for performance

## Terrain Configuration

### Engine Setup

```xml
<engine_config>
    <terrain enabled="true">
        <cache_mb>2048</cache_mb>
        <tile_size>256</tile_size>
        <data_path>/data/terrain/dted/</data_path>
        <data_path>/data/terrain/srtm/</data_path>
    </terrain>
</engine_config>
```

### Programmatic Configuration

```cpp
config::EngineConfig cfg;
cfg.terrain_enabled = true;
cfg.terrain_cache_mb = 4096;  // 4 GB cache
cfg.terrain_data_paths.push_back("/data/dted/");
cfg.terrain_data_paths.push_back("/data/srtm/");

Engine engine;
engine.initialize(cfg);
```

## Basic Terrain Queries

### Elevation Query

```cpp
auto& terrain = engine.get_environment_service().terrain();

// Query elevation at position
Real lat = 37.0 * DEG_TO_RAD;
Real lon = -122.0 * DEG_TO_RAD;
Real elevation = terrain.get_elevation(lat, lon);

std::cout << "Elevation: " << elevation << " m\n";
```

### Surface Normal

```cpp
// Get surface normal (for vehicle orientation)
Vec3 normal = terrain.get_surface_normal(lat, lon);

// Compute slope
Real slope_angle = terrain.get_slope_angle(lat, lon);
std::cout << "Slope: " << slope_angle * RAD_TO_DEG << " degrees\n";
```

### Full Query

```cpp
TerrainQuery query = terrain.query(lat, lon);
if (query.valid) {
    std::cout << "Elevation: " << query.elevation << " m\n";
    std::cout << "Normal: " << query.normal << "\n";
    std::cout << "Slope: " << query.slope_angle * RAD_TO_DEG << " deg\n";
    std::cout << "Material: " << query.material.name << "\n";
}
```

## Working with Materials

### Soil Types

```cpp
// Get terrain material
TerrainMaterial material = terrain.get_material(lat, lon);

// Get soil properties for terramechanics
domain::land::SoilProperties soil;

switch (material.type) {
    case MaterialType::Sand:
        soil = domain::land::SoilProperties::DrySand();
        break;
    case MaterialType::Clay:
        soil = domain::land::SoilProperties::Clay();
        break;
    case MaterialType::Rock:
        soil = domain::land::SoilProperties::Asphalt();
        break;
    case MaterialType::Water:
        // Handle water body
        break;
    default:
        soil = domain::land::SoilProperties::DrySand();
}
```

### Material Database

```xml
<!-- materials/terrain_materials.xml -->
<material_database>
    <material id="1" name="sand">
        <friction>0.6</friction>
        <soil_type>dry_sand</soil_type>
    </material>

    <material id="2" name="forest">
        <friction>0.7</friction>
        <soil_type>clay</soil_type>
        <vegetation_density>0.8</vegetation_density>
    </material>

    <material id="3" name="urban">
        <friction>0.85</friction>
        <soil_type>asphalt</soil_type>
    </material>
</material_database>
```

## LOD and Performance

### Focus Point

```cpp
// Set terrain detail center (e.g., player position)
Vec3 focus_ecef = current_entity_position;
terrain.set_focus_point(focus_ecef);

// Set radius of high detail
terrain.set_detail_radius(50000.0);  // 50 km high detail
```

### Cache Management

```cpp
// Monitor cache usage
SizeT loaded_tiles = terrain.loaded_tile_count();
SizeT cache_bytes = terrain.cache_usage_bytes();

std::cout << "Loaded tiles: " << loaded_tiles << "\n";
std::cout << "Cache usage: " << cache_bytes / 1024 / 1024 << " MB\n";

// Force cache update
terrain.update();
```

## Ground Vehicle Integration

### Height Above Terrain

```cpp
void update_vehicle_on_terrain(Engine& engine, EntityId vehicle) {
    auto state = engine.get_entity_state(vehicle);
    auto env = engine.get_environment(vehicle);

    // Height above terrain
    Real hat = env.altitude - env.terrain_elevation;

    if (hat < 0.1) {
        // Vehicle on ground - apply terrain normal
        Vec3 up = env.terrain.normal;

        // Project velocity onto terrain plane
        Vec3 vel = state.velocity;
        Real normal_vel = vel.dot(up);
        if (normal_vel < 0) {
            // Remove downward velocity component
            vel -= up * normal_vel;
            state.velocity = vel;
        }

        // Align vehicle to terrain
        // ... orientation adjustment ...
    }
}
```

### Terrain Following

```cpp
class TerrainFollower {
public:
    void set_target_agl(Real agl) { target_agl_ = agl; }

    void update(Engine& engine, EntityId aircraft, Real dt) {
        auto state = engine.get_entity_state(aircraft);
        auto env = engine.get_environment(aircraft);

        // Current height above terrain
        Real current_agl = env.altitude - env.terrain_elevation;

        // Look-ahead terrain query
        Vec3 ahead = state.velocity.normalized() * lookahead_dist_;
        Real lat_ahead, lon_ahead, alt_ahead;
        transforms::ecef_to_geodetic(
            state.position + ahead, lat_ahead, lon_ahead, alt_ahead);

        auto& terrain = engine.get_environment_service().terrain();
        Real terrain_ahead = terrain.get_elevation(lat_ahead, lon_ahead);

        // Required altitude
        Real required_alt = terrain_ahead + target_agl_;

        // Compute climb/dive command
        Real alt_error = required_alt - env.altitude;
        pitch_cmd_ = kp_ * alt_error;
        pitch_cmd_ = std::clamp(pitch_cmd_, -max_pitch_, max_pitch_);
    }

    Real get_pitch_command() const { return pitch_cmd_; }

private:
    Real target_agl_{100.0};
    Real lookahead_dist_{1000.0};
    Real kp_{0.01};
    Real max_pitch_{0.5};
    Real pitch_cmd_{0.0};
};
```

## Terrain Data Sources

### DTED Levels

| Level | Resolution | Coverage | Use Case |
|-------|-----------|----------|----------|
| 0 | ~900 m | Global | Strategic planning |
| 1 | ~90 m | Most land | Tactical simulation |
| 2 | ~30 m | Select areas | High-fidelity |

### Data Organization

```
/data/terrain/
├── dted/
│   ├── w123/
│   │   ├── n36.dt1
│   │   ├── n37.dt1
│   │   └── ...
│   └── w122/
│       └── ...
├── srtm/
│   ├── n36_w123_1arc_v3.tif
│   └── ...
└── custom/
    └── local_dem.tif
```

## Advanced Topics

### Custom Terrain Provider

```cpp
class CustomTerrainProvider : public ITerrainProvider {
public:
    Real get_elevation(Real lat, Real lon) override {
        // Custom elevation source
        return lookup_custom_data(lat, lon);
    }

    Vec3 get_normal(Real lat, Real lon) override {
        // Compute normal from neighboring elevations
        Real h0 = get_elevation(lat, lon);
        Real hx = get_elevation(lat, lon + dlat);
        Real hy = get_elevation(lat + dlat, lon);

        Vec3 dx{dlat * R_EARTH, 0, hx - h0};
        Vec3 dy{0, dlat * R_EARTH, hy - h0};
        return dx.cross(dy).normalized();
    }

private:
    Real lookup_custom_data(Real lat, Real lon);
};
```

### Procedural Terrain

```cpp
Real procedural_elevation(Real lat, Real lon) {
    // Perlin noise-based terrain
    Real scale = 0.0001;
    Real x = lon / scale;
    Real y = lat / scale;

    Real h = 0.0;
    Real amplitude = 1000.0;  // Base amplitude in meters

    // Multiple octaves
    for (int octave = 0; octave < 4; ++octave) {
        h += amplitude * perlin_noise(x, y);
        amplitude *= 0.5;
        x *= 2.0;
        y *= 2.0;
    }

    return h;
}
```

## Best Practices

### Performance

1. **Cache appropriately**: Set cache size based on area of operation
2. **Use focus point**: Prioritize detail where needed
3. **Batch queries**: Group terrain queries when possible

### Data Management

1. **Organize by region**: Structure data directories logically
2. **Use appropriate resolution**: Higher isn't always better
3. **Verify coverage**: Ensure data covers simulation area

### Common Issues

**Missing terrain data**: Verify data paths and file formats

**Performance spikes**: Increase cache size or reduce detail radius

**Discontinuities**: Check for data gaps between tiles

## See Also

- [Environment](../concepts/environment.md) - Environment system
- [Land Domain](land-domain.md) - Ground vehicle simulation
- [Configuration](../api/configuration.md) - Configuration options
