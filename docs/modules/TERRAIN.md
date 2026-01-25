# Terrain Module - GIS Data Integration and Elevation Queries

## Overview

The Terrain module integrates GDAL (Geospatial Data Abstraction Library) for loading and querying raster elevation data. It provides bilinear interpolation for smooth elevation surfaces, surface normal computation for terrain-aware physics, and efficient caching for large terrain datasets.

## Key Features

- **GDAL Integration**: Support for GeoTIFF, DTED, and other GDAL-compatible formats
- **Bilinear Interpolation**: Smooth elevation queries between grid points
- **Surface Normals**: Computed from height gradients for realistic physics
- **Caching**: LRU cache for efficient repeated queries
- **Thread-Safe Queries**: Read operations safe from multiple physics threads
- **Material Properties**: Surface friction, bearing capacity, trafficability
- **Asynchronous Loading**: Non-blocking terrain data loading
- **Coordinate System Support**: WGS84, local projections

## Architecture

```
┌──────────────────────────────────────────────────────────┐
│                  Terrain Manager                         │
├──────────────────────────────────────────────────────────┤
│  ┌────────────────────────────────────────────────────┐  │
│  │          GDAL Terrain Loader                       │  │
│  │  - GeoTIFF, DTED, GIS rasters                      │  │
│  │  - Coordinate transformations                       │  │
│  │  - Metadata parsing                                 │  │
│  └────────────────────────────────────────────────────┘  │
│  ┌────────────────────────────────────────────────────┐  │
│  │          Elevation Query Engine                    │  │
│  │  - Bilinear interpolation                          │  │
│  │  - Height gradient computation                     │  │
│  │  - Surface normal calculation                      │  │
│  └────────────────────────────────────────────────────┘  │
│  ┌────────────────────────────────────────────────────┐  │
│  │          LRU Cache Layer                           │  │
│  │  - Recent query results                            │  │
│  │  - Thread-safe read access                         │  │
│  │  - Configurable size (MB)                          │  │
│  └────────────────────────────────────────────────────┘  │
│  ┌────────────────────────────────────────────────────┐  │
│  │          Material Database                         │  │
│  │  - Surface type mapping                            │  │
│  │  - Friction coefficients                           │  │
│  │  - Bearing capacity                                │  │
│  └────────────────────────────────────────────────────┘  │
└──────────────────────────────────────────────────────────┘
```

## Building with Terrain Support

### GDAL Dependency

The terrain module has **optional GDAL support**. You have two options:

**Option 1: With GDAL (Full functionality)**
```bash
# Install GDAL development libraries
# macOS
brew install gdal

# Ubuntu/Debian
sudo apt-get install libgdal-dev

# Then configure JaguarEngine
cmake -DCMAKE_BUILD_TYPE=Release ..
cmake --build .
```

The build system automatically detects GDAL via `find_package(GDAL)`. If found:
- Compilation flag `JAGUAR_HAS_GDAL` is set
- Full GDAL implementation is compiled
- Support for GeoTIFF, DTED, and all GDAL formats

**Option 2: Without GDAL (Stub implementation)**
```bash
# Build without GDAL (no special setup needed)
cmake -DCMAKE_BUILD_TYPE=Release ..
cmake --build .
```

If GDAL is not found:
- Compilation flag `JAGUAR_HAS_GDAL` is NOT set
- Stub implementation is compiled
- `GDALTerrainLoader::is_available()` returns false
- Queries return NaN (indicating no data)
- Code still compiles and links successfully

### Runtime Detection

Always check if GDAL is available:

```cpp
// Check at runtime
if (GDALTerrainLoader::is_available()) {
    // Full GDAL available, can load terrain
    terrain.load_terrain("/data/elevation.tif");
} else {
    // Running with stub - no terrain data available
    // Queries will use default fallback values
}
```

## API Reference

### TerrainQuery Structure

Result of elevation and surface query:

```cpp
struct TerrainQuery {
    Real elevation{0.0};        // Height above WGS84 ellipsoid (m)
    Vec3 normal{0, 0, 1};       // Surface normal (NED frame)
    Real slope_angle{0.0};      // Slope angle (rad)
    TerrainMaterial material;   // Surface type and properties
    bool valid{false};          // Query succeeded
};
```

**Fields explained:**

- **elevation**: Height in meters above the WGS84 ellipsoid reference surface
- **normal**: Unit surface normal vector in NED (North-East-Down) frame. Used for contact force calculation
- **slope_angle**: Slope angle in radians, computed as angle between surface normal and downward direction
- **material**: Surface properties including friction and bearing capacity
- **valid**: True if query succeeded within data bounds, false if out of bounds or no data available

### TerrainMaterial Structure

Surface properties for physics simulation:

```cpp
struct TerrainMaterial {
    SurfaceType type{SurfaceType::Unknown};
    Real friction_coefficient{0.7};    // Bekker friction
    Real rolling_resistance{0.01};     // Energy loss per meter

    // Bekker-Wong terramechanics parameters
    Real k_c{0.0};              ///< Cohesive modulus (kN/m^(n+1))
    Real k_phi{0.0};            ///< Frictional modulus (kN/m^(n+2))
    Real n{1.0};                ///< Deformation exponent

    // Trafficability limits
    Real max_slope_deg{30.0};   // Maximum passable slope
    Real bearing_capacity{100000.0};  // Pa - maximum load
};

enum class SurfaceType : UInt8 {
    Unknown, Asphalt, Concrete, Gravel, DrySand, WetSand,
    Clay, Mud, Grass, Snow, Ice, Water
};
```

### GDAL Terrain Loader

Low-level GDAL raster loading with automatic coordinate transformation:

```cpp
class GDALTerrainLoader {
public:
    GDALTerrainLoader();
    ~GDALTerrainLoader();

    // Non-copyable
    GDALTerrainLoader(const GDALTerrainLoader&) = delete;
    GDALTerrainLoader& operator=(const GDALTerrainLoader&) = delete;

    // ====================================================================
    // Dataset Management
    // ====================================================================

    /**
     * Check if GDAL is available at compile time
     * @return true if compiled with GDAL support
     */
    static bool is_available();

    /**
     * Open terrain dataset from file
     * @param path Path to GeoTIFF, DTED, or other GDAL-supported raster
     * @return true if successfully opened
     */
    bool open(const std::string& path);

    /**
     * Close current dataset and release resources
     */
    void close();

    /**
     * Get dataset metadata (bounds, resolution, CRS)
     * @return TerrainDataset with bounds and resolution in WGS84
     */
    TerrainDataset get_dataset_info() const;

    // ====================================================================
    // Elevation Queries
    // ====================================================================

    /**
     * Get elevation at WGS84 position with bilinear interpolation
     * @param lat Latitude (radians)
     * @param lon Longitude (radians)
     * @return Elevation in meters, or NaN if out of bounds
     */
    Real get_elevation(Real lat, Real lon) const;

    /**
     * Get surface normal at position using elevation gradient
     * Computed from finite differences of elevation
     * @param lat Latitude (radians)
     * @param lon Longitude (radians)
     * @return Surface normal in NED frame, or {0,0,1} if unavailable
     */
    Vec3 get_normal(Real lat, Real lon) const;

    /**
     * Complete terrain query (elevation + normal + slope)
     * @param lat Latitude (radians)
     * @param lon Longitude (radians)
     * @return Terrain query with elevation, normal, and slope_angle
     */
    TerrainQuery query(Real lat, Real lon) const;

    // ====================================================================
    // Dataset Properties
    // ====================================================================

    /**
     * Get dataset geographic bounds (WGS84)
     * @param min_lat Minimum latitude (radians)
     * @param max_lat Maximum latitude (radians)
     * @param min_lon Minimum longitude (radians)
     * @param max_lon Maximum longitude (radians)
     */
    void get_bounds(Real& min_lat, Real& max_lat, Real& min_lon, Real& max_lon) const;

    /**
     * Get spatial resolution in degrees
     * @return Resolution in decimal degrees
     */
    Real get_resolution() const;

    /**
     * Check if a dataset is currently loaded
     * @return true if a file is successfully opened
     */
    bool is_loaded() const;
};
```

**Thread Safety:**

The GDAL loader is thread-safe for read operations after the dataset is loaded:
- `get_elevation()`, `get_normal()`, `query()` are thread-safe
- `open()` and `close()` are NOT thread-safe (call from single thread)

**Bilinear Interpolation Algorithm:**

For position (lat, lon) that maps to pixel coordinates (px, py) within grid:

```
1. Compute fractional pixel coordinates:
   x0 = floor(px), x1 = x0 + 1
   y0 = floor(py), y1 = y0 + 1
   u = px - x0 (fractional part, 0-1)
   v = py - y0 (fractional part, 0-1)

2. Read 2x2 neighborhood from raster:
   z00 = elevation[x0, y0]
   z10 = elevation[x1, y0]
   z01 = elevation[x0, y1]
   z11 = elevation[x1, y1]

3. Bilinear interpolation:
   z = (1-u)(1-v)*z00 + u(1-v)*z10
       + (1-u)*v*z01 + u*v*z11
```

**Advantages:**
- Smooth elevation field (no blocky artifacts)
- Sub-pixel accuracy
- Numerically stable

**Coordinate Transformation:**

When dataset CRS differs from WGS84:
1. Receive input in WGS84 (latitude, longitude in radians)
2. Use OGRCoordinateTransformation to convert to dataset CRS
3. Apply geotransform matrix to get pixel coordinates
4. Perform bilinear interpolation

```cpp
// Example: Querying GeoTIFF in local projection
Real elev = loader.get_elevation(
    37.7749 * M_PI / 180.0,   // San Francisco latitude (rad)
    -122.4194 * M_PI / 180.0  // San Francisco longitude (rad)
);
// Internally transforms to dataset CRS and gets elevation
```

**Surface Normal Computation:**

Computed from elevation gradient using finite differences:

```
1. Sample elevation at neighboring points:
   z_north = get_elevation(lat + δlat, lon)
   z_south = get_elevation(lat - δlat, lon)
   z_east = get_elevation(lat, lon + δlon)
   z_west = get_elevation(lat, lon - δlon)

2. Compute gradients in NED frame:
   dz/dx = (z_north - z_south) / (2 * δlat)
   dz/dy = (z_east - z_west) / (2 * δlon)

3. Normalize to unit normal:
   magnitude = sqrt(1 + (dz/dx)² + (dz/dy)²)
   nx = -(dz/dx) / magnitude
   ny = -(dz/dy) / magnitude
   nz = 1.0 / magnitude
   normal = [nx, ny, nz] in NED frame
```

**No-Data Handling:**

GDAL datasets may have no-data values indicating missing elevation:
- No-data value is detected automatically during open()
- Queries in no-data regions return NaN
- Manager falls back to default terrain gracefully

### Terrain Manager

High-level terrain interface with ECEF support and thread-safe queries:

```cpp
class TerrainManager {
public:
    TerrainManager();
    ~TerrainManager();

    // Non-copyable
    TerrainManager(const TerrainManager&) = delete;
    TerrainManager& operator=(const TerrainManager&) = delete;

    // ====================================================================
    // Lifecycle Management
    // ====================================================================

    /**
     * Add data source path (searched in order during initialize)
     * @param path Directory containing terrain files or single file path
     */
    void add_data_path(const std::string& path);

    /**
     * Load terrain from a file
     * @param path Path to GeoTIFF or other GDAL-supported raster
     * @return true if successfully loaded
     */
    bool load_terrain(const std::string& path);

    /**
     * Initialize terrain system (loads from data paths)
     * @return true if successful
     */
    bool initialize();

    /**
     * Shutdown and release resources
     */
    void shutdown();

    /**
     * Check if terrain data is loaded
     * @return true if data is available for queries
     */
    bool is_terrain_loaded() const;

    // ====================================================================
    // Elevation Queries (Geodetic Coordinates)
    // ====================================================================

    /**
     * Get elevation at WGS84 position
     * @param lat Latitude (radians)
     * @param lon Longitude (radians)
     * @return Elevation in meters above WGS84 ellipsoid, or NaN if unavailable
     */
    Real get_elevation(Real lat, Real lon) const;

    /**
     * Get surface normal at position (NED frame)
     * @param lat Latitude (radians)
     * @param lon Longitude (radians)
     * @return Unit normal vector, or {0,0,1} if unavailable
     */
    Vec3 get_surface_normal(Real lat, Real lon) const;

    /**
     * Get slope angle at position
     * @param lat Latitude (radians)
     * @param lon Longitude (radians)
     * @return Slope angle in radians (0 = flat, π/2 = vertical)
     */
    Real get_slope_angle(Real lat, Real lon) const;

    /**
     * Get material properties at position
     * @param lat Latitude (radians)
     * @param lon Longitude (radians)
     * @return Material with friction, bearing capacity, etc.
     */
    TerrainMaterial get_material(Real lat, Real lon) const;

    /**
     * Complete terrain query (all fields)
     * @param lat Latitude (radians)
     * @param lon Longitude (radians)
     * @return TerrainQuery with elevation, normal, slope, material, valid flag
     */
    TerrainQuery query(Real lat, Real lon) const;

    // ====================================================================
    // ECEF Coordinate Queries
    // ====================================================================

    /**
     * Get elevation at ECEF position
     * Automatically converts ECEF to geodetic coordinates
     * @param ecef Position in ECEF (meters)
     * @return Elevation in meters
     */
    Real get_elevation_ecef(const Vec3& ecef) const;

    /**
     * Complete terrain query at ECEF position
     * @param ecef Position in ECEF (meters)
     * @return TerrainQuery with full terrain information
     */
    TerrainQuery query_ecef(const Vec3& ecef) const;

    // ====================================================================
    // LOD/Caching Management
    // ====================================================================

    /**
     * Set cache size in MB (for tiled terrain systems)
     * @param mb Cache size in megabytes
     */
    void set_cache_size(SizeT mb);

    /**
     * Set focus point for LOD management
     * Hint to loader which region should be preloaded
     * @param ecef Focus point in ECEF coordinates
     */
    void set_focus_point(const Vec3& ecef);

    /**
     * Set detail radius around focus point
     * @param radius_m Radius in meters
     */
    void set_detail_radius(Real radius_m);

    /**
     * Update terrain paging (call once per frame from main thread)
     * Loads/unloads tiles around focus point
     */
    void update();

    // ====================================================================
    // Statistics and Metadata
    // ====================================================================

    /**
     * Get number of loaded terrain tiles
     * @return Tile count (0 for single-file terrain)
     */
    SizeT loaded_tile_count() const;

    /**
     * Get cache memory usage
     * @return Bytes of loaded terrain data
     */
    SizeT cache_usage_bytes() const;
};
```

**Manager Behavior:**

- **Fallback to defaults**: If no terrain loaded, returns flat ground (elevation=0) with grass material
- **No-data handling**: Out-of-bounds queries handled gracefully with valid=false
- **Lifetime**: Initialize once, use across multiple frames, shutdown when done

**Usage Patterns:**

Basic terrain loading and querying:

```cpp
// Create manager
TerrainManager terrain;

// Add data paths (searched in order)
terrain.add_data_path("/data/terrain");

// Load specific file
if (!terrain.load_terrain("/data/terrain/bay_area.tif")) {
    std::cerr << "Failed to load terrain\n";
}

// Configure cache for tiled terrain
terrain.set_cache_size(512);  // 512MB cache

// Geodetic query (latitude/longitude in radians)
double lat = 37.7749 * M_PI / 180.0;  // San Francisco
double lon = -122.4194 * M_PI / 180.0;
Real elevation = terrain.get_elevation(lat, lon);

// Get surface normal for physics
Vec3 ground_normal = terrain.get_surface_normal(lat, lon);

// Complete terrain query
TerrainQuery result = terrain.query(lat, lon);
if (result.valid) {
    std::cout << "Elevation: " << result.elevation << " m\n";
    std::cout << "Slope: " << (result.slope_angle * 180 / M_PI) << " degrees\n";
    std::cout << "Friction: " << result.material.friction_coefficient << "\n";
}
```

ECEF coordinate queries (useful for physics simulation):

```cpp
// Get vehicle position in ECEF
Vec3 vehicle_ecef = entity.position;

// Query terrain at ECEF position (handles conversion internally)
TerrainQuery terrain_at_vehicle = terrain.query_ecef(vehicle_ecef);

// Also available:
Real elev = terrain.get_elevation_ecef(vehicle_ecef);

// Graceful fallback: if no terrain loaded, returns defaults
// valid = true, elevation = 0, normal = {0,0,1}, material = grass
```

Integration with focus area for LOD:

```cpp
// Set focus point around player for pre-loading
terrain.set_focus_point(player_position_ecef);
terrain.set_detail_radius(50000);  // 50km detail area

// Update cache once per frame (from main thread only)
void main_loop() {
    while (running) {
        update_physics();

        // Update which terrain tiles are loaded
        terrain.set_focus_point(get_player_position());
        terrain.update();  // Non-blocking, handles paging

        render();
    }
}
```

## Thread Safety

### Safe Operations

**Thread-Safe for Reading (multiple concurrent threads):**
- `get_elevation()`
- `get_surface_normal()`
- `get_slope_angle()`
- `get_material()`
- `query()`
- `query_batch()`
- Cache read access is lock-free

### Unsafe Operations

**NOT Thread-Safe (single writer required):**
- `load_terrain()`
- `add_data_path()`
- `set_cache_size()`
- `clear_cache()`
- `update()`

**Recommended Usage Pattern:**

```cpp
// Main thread setup
void initialize_terrain(TerrainManager& terrain) {
    terrain.add_data_path("/data/terrain");
    terrain.load_terrain("/data/terrain/region.tif");
    terrain.set_cache_size(256);
}

// Physics threads - read-only
void physics_update(const TerrainManager& terrain) {
    for (auto& entity : entities) {
        TerrainQuery query = terrain.query(entity.lat, entity.lon);
        apply_ground_forces(entity, query);
    }
}

// Main thread - periodic updates
void main_loop() {
    while (running) {
        update_physics(dt);
        terrain.set_focus_point(player_position);
        terrain.update();  // Called from single thread
    }
}
```

## Performance Considerations

### Query Performance

| Operation | Time (μs) | Notes |
|-----------|-----------|-------|
| Elevation query (cached in memory) | 0.1-0.5 | Fast path, no I/O |
| Elevation query (L3 cache miss) | 1-5 | Memory access, no I/O |
| Elevation query (disk I/O) | 100-1000 | GDAL reads from disk |
| Surface normal query | 0.2-0.5 | From cached elevation |
| Slope angle computation | 0.05-0.1 | From normal |
| Material lookup | 0.01-0.05 | Hash table lookup |
| Full TerrainQuery | 0.3-1.0 | Elevation + normal + slope |
| query_ecef (ECEF conversion) | 1-2 | + coordinate transformation |

**Performance notes:**
- All timings assume thread-safe shared_mutex lock acquisition (~0.1 μs per operation)
- First query in a region may trigger disk I/O if not cached
- Subsequent queries in same region are cache-hit fast
- Normal computation uses finite differences (3-5 elevation samples)

### Memory Usage Analysis

```cpp
// Example: 256MB LRU cache with 1-degree tiles

struct TerrainMemory {
    SizeT cache_mb = 256;                // Configured cache size
    SizeT per_tile_mb = 0.5;             // 1° × 1° GeoTIFF tile
    SizeT tiles_in_cache = cache_mb / per_tile_mb;  // ~512 tiles

    // Coverage estimate (at WGS84)
    SizeT degree_coverage = tiles_in_cache;  // 512 tiles
    SizeT sq_km_per_tile = 111 * 111;  // ~12,000 km² per degree at equator
    SizeT total_coverage_sq_km = degree_coverage * sq_km_per_tile;  // ~6.1M km²

    // Query rate
    SizeT entities = 10000;             // 10k vehicles
    SizeT queries_per_frame = entities; // 1 query per entity
    Real frame_rate = 60.0;             // 60 Hz

    // Time to load one tile at 100MB/s disk speed
    Real load_time_ms = (per_tile_mb * 1000) / 100;  // ~5ms per tile
};

// Practical example: 256MB cache
// - Holds ~512 tiles at 1° × 1° resolution
// - Covers ~6 million km² (entire continental US + more)
// - 10k entities querying at 60 Hz = 600k queries/sec
// - With caching: almost all cache hits after warmup
```

### Optimization Strategies

**1. Pre-load focus area:**
```cpp
// Before simulation starts, pre-populate cache
terrain.set_focus_point(simulation_center_ecef);
for (int i = 0; i < 10; ++i) {
    terrain.update();  // Load tiles incrementally
}
```

**2. Spatial locality clustering:**
```cpp
// Good performance: entities in same region (share cache lines)
// Poor performance: random global positions (each requires new tile)

// Solution: Update focus point to busiest region
Vec3 center_of_mass = compute_entity_center();
terrain.set_focus_point(center_of_mass);
terrain.update();  // 1x per frame
```

**3. ECEF vs geodetic queries:**
```cpp
// Prefer query_ecef() for physics (saves coordinate conversion)
TerrainQuery result = terrain.query_ecef(entity.position_ecef);  // 1-2 μs
// vs.
GeodeticPosition lla = ecef_to_lla(entity.position_ecef);
TerrainQuery result = terrain.query(lla.latitude, lla.longitude);  // 2-3 μs
```

**4. Batch queries (if processing multiple entities at once):**
```cpp
// Process entities in spatial clusters
std::vector<Vec3> entity_positions;
std::vector<TerrainQuery> results;

for (auto& entity : entities_in_region) {
    entity_positions.push_back(entity.position_ecef);
}

// All queries benefit from shared cache lines
for (size_t i = 0; i < entity_positions.size(); ++i) {
    results[i] = terrain.query_ecef(entity_positions[i]);
}
```

### Thread-Safe Query Performance

Queries are protected by `std::shared_mutex`:

```cpp
// Multiple physics threads reading (shared lock)
std::shared_lock lock(terrain_mutex);  // ~0.05-0.1 μs
Real elevation = loader.get_elevation(lat, lon);  // ~0.1-0.5 μs
```

**Contention analysis:**
- Read-only contention: Minimal (many readers can hold shared lock)
- Write operations: Exclusive lock, serializes all access
- Practical impact: Negligible for typical frame rates (<1% overhead)

### Cache Strategy Recommendations

| Scenario | Cache Size | Update Frequency | Notes |
|----------|-----------|------------------|-------|
| Single vehicle | 64-128 MB | Every frame | Minimal memory, follow one entity |
| 10-100 vehicles | 256-512 MB | Every frame | Cluster entities spatially |
| 1000+ vehicles | 1-2 GB | Every 2-3 frames | Large cache, update focus area |
| Streaming terrain | 4-8 GB | Every frame | Large tiles, aggressive paging |

**Cache eviction policy:**
- Default: LRU (Least Recently Used) eviction
- Good for: Entities moving through terrain (working set stays relevant)
- Poor for: Random access patterns (churns cache)

### Bilinear Interpolation vs. Nearest Neighbor

**Bilinear (implemented):**
- Smooth elevation field
- Normals have continuous variation
- Slightly slower (~0.1 μs more per query)
- Better physics stability

**Nearest neighbor (not implemented):**
- Blocky elevation field
- Sharp normal discontinuities at tile edges
- Faster but worse physics behavior

## Coordinate Systems

### Coordinate Frame Overview

The terrain system uses multiple coordinate frames:

| Frame | Purpose | Note |
|-------|---------|------|
| **WGS84 Geodetic** | Terrain queries (input) | Latitude/longitude/altitude |
| **ECEF** | Physics simulation | Earth-Centered-Earth-Fixed |
| **NED** | Physics output (normals) | North-East-Down local frame |
| **Body** | Vehicle physics | Vehicle-relative frame |
| **Raster** | Internal GDAL processing | Pixel coordinates in dataset CRS |

### Geodetic Coordinates (Query Input)

All terrain queries accept **WGS84 geodetic coordinates**:

- **Latitude**: [-π/2, π/2] radians (North positive, positive = northern hemisphere)
- **Longitude**: [-π, π] radians (East positive, positive = eastern hemisphere)
- **Altitude**: meters above WGS84 ellipsoid (only for context, not used in queries)

```cpp
// San Francisco Bay Area
double lat = 37.7749 * M_PI / 180.0;   // 37.7749° N in radians
double lon = -122.4194 * M_PI / 180.0; // 122.4194° W in radians

// Query terrain at this location
Real elevation = terrain.get_elevation(lat, lon);  // meters above WGS84
Vec3 normal = terrain.get_surface_normal(lat, lon);  // NED frame

// Common locations reference
// New York:     40.7128° N,  74.0060° W
// Tokyo:        35.6762° N, 139.6503° E
// Sydney:      -33.8688° S, 151.2093° E
// Cape Town:   -33.9249° S,  18.4241° E
```

### WGS84 Ellipsoid

The WGS84 reference ellipsoid defines the zero elevation surface:

```cpp
// WGS84 ellipsoid parameters
struct WGS84Ellipsoid {
    Real semi_major_axis = 6378137.0;     // a (meters) - equatorial radius
    Real semi_minor_axis = 6356752.314245; // b (meters) - polar radius
    Real flattening = (a - b) / a;         // f = 1/298.257223563
};

// Earth surface elevation varies:
// - Deepest ocean trench: ~-11,000 m (Mariana Trench)
// - Highest mountain: ~+8,849 m (Mount Everest)
// - Most land: between -500 m and +3,000 m
```

### ECEF Coordinates (Physics Frame)

Earth-Centered-Earth-Fixed (ECEF) coordinates used by physics engine:

```cpp
// ECEF is Cartesian coordinates centered at Earth's center
struct ECEFPosition {
    Real x;  // Direction to prime meridian (0° longitude, equator)
    Real y;  // Direction to 90° E longitude, equator
    Real z;  // Direction to North Pole
    // Distances in meters from Earth center (~6.37M meters)
};

// Example: San Francisco
// Geodetic: 37.7749° N, 122.4194° W, altitude 52m
// ECEF approx: (-2706520, -4276220, 3890950) meters

// Terrain module provides direct ECEF query
Vec3 entity_ecef = entity.position;  // From physics
TerrainQuery result = terrain.query_ecef(entity_ecef);  // Converts internally
```

### NED Frame (Output for Physics)

Surface normals output in **NED (North-East-Down)** frame:

```cpp
Vec3 ned_normal = terrain.get_surface_normal(lat, lon);
// ned_normal = [nx, ny, nz] unit vector where:
//   nx > 0: slope faces north
//   ny > 0: slope faces east
//   nz > 0: slope faces downward (convex)
//   nz < 0: slope faces upward (concave - rare)

// Example: 30° slope facing northeast
Vec3 example = {0.5, 0.5, 0.71};  // normalized

// Flat terrain
Vec3 flat = {0, 0, 1};  // faces directly down

// Vertical cliff
Vec3 cliff = {1, 0, 0};  // faces north (only nx non-zero)
```

### Coordinate Transformations

Use utility functions for conversions (from coordinates module):

```cpp
// Geodetic to ECEF
Vec3 position_ecef = geodetic_to_ecef(lat, lon, altitude);

// ECEF to Geodetic
GeodeticPosition lla = ecef_to_lla(ecef_position);
Real lat = lla.latitude;      // radians
Real lon = lla.longitude;     // radians
Real alt = lla.altitude;      // meters

// Construct terrain query
TerrainQuery tq = terrain.query(lla.latitude, lla.longitude);
```

### Dataset Projection (Internal)

GDAL datasets may use various projections. Transformations handled automatically:

```cpp
// GeoTIFF might use:
// - WGS84 geographic (lat/lon) - no transformation needed
// - UTM projection - transformed to WGS84 for query
// - Local projection - transformed to WGS84 for query

// User code remains unchanged - always use geodetic
Real elev = terrain.get_elevation(lat, lon);  // Works for any dataset CRS
```

### Practical Example: Vehicles in Physics Simulation

```cpp
// Physics simulation uses ECEF for entities
class Vehicle {
    Vec3 position_ecef;      // ECEF (x, y, z) meters
    Quaternion orientation;  // Body frame orientation
    Vec3 velocity_ecef;      // ECEF velocity (m/s)
};

// Terrain interaction loop
for (auto& vehicle : vehicles) {
    // Query terrain at vehicle location
    // Internal conversion: ECEF → LLA → GDAL coordinates → elevation
    TerrainQuery ground = terrain.query_ecef(vehicle.position_ecef);

    if (ground.valid) {
        // Compute ground altitude in ECEF
        Real ground_altitude_z = geodetic_to_ecef(
            ecef_to_lla(vehicle.position_ecef).latitude,
            ecef_to_lla(vehicle.position_ecef).longitude,
            ground.elevation
        ).z;

        // Penetration (negative = airborne, positive = penetrating)
        Real penetration = ground_altitude_z - vehicle.position_ecef.z;

        // Apply contact forces using terrain normal
        if (penetration > 0) {
            // Convert NED normal to ECEF body frame
            Vec3 contact_force = compute_contact_force(
                penetration,
                ground.normal,  // NED frame
                vehicle.orientation
            );
            vehicle.add_force(contact_force);
        }
    }
}
```

## Terrain-Aware Physics

### EnvironmentService Integration

Terrain queries are performed through the central EnvironmentService:

```cpp
// From environment/environment.h
class EnvironmentService {
public:
    // Load terrain from file
    bool load_terrain(const std::string& path);

    // Query terrain at ECEF position
    TerrainQuery query_terrain(const Vec3& position_ecef) const;

    // Get elevation at geodetic position
    Real get_terrain_elevation(Real lat, Real lon) const;

    // Get underlying terrain manager
    TerrainManager& terrain() { return terrain_; }
    const TerrainManager& terrain() const { return terrain_; }

    // Combined environment query (terrain + atmosphere + ocean)
    Environment query(const Vec3& ecef, Real time) const;
};
```

Usage:

```cpp
// Initialize environment service
EnvironmentService env_service;
env_service.initialize();
env_service.load_terrain("/data/elevation.tif");

// Query at ECEF position
Vec3 vehicle_position_ecef = get_vehicle_position();
TerrainQuery terrain_info = env_service.query_terrain(vehicle_position_ecef);

// Combined environment (terrain + weather + ocean)
Environment full_env = env_service.query(vehicle_position_ecef, simulation_time);
```

### Ground Contact Forces

Computing contact forces based on terrain:

```cpp
// Query terrain at entity position
TerrainQuery terrain = terrain_manager.query_ecef(entity.position_ecef);

if (!terrain.valid) {
    // No terrain data - skip ground contact
    return;
}

// Convert ECEF entity position to geodetic for clarity
GeodeticPosition lla = ecef_to_lla(entity.position_ecef);

// Compute ground surface altitude in ECEF
Real ground_altitude_ecef = geodetic_to_ecef(
    lla.latitude,
    lla.longitude,
    terrain.elevation
).z;

Real vehicle_altitude_ecef = entity.position_ecef.z;
Real penetration = ground_altitude_ecef - vehicle_altitude_ecef;

if (penetration > 0) {
    // Vehicle is below ground - apply contact force
    Real contact_force = penetration * spring_stiffness;

    // Convert terrain normal to vehicle body frame
    Vec3 normal_ned = terrain.normal;  // NED frame
    Vec3 normal_body = ned_to_body_frame(normal_ned, entity.orientation);

    // Apply spring force
    Vec3 spring_force = normal_body * contact_force;
    entity.add_force(spring_force);

    // Apply damping
    Vec3 velocity_body = to_body_frame(entity.velocity, entity.orientation);
    Real velocity_normal = dot(velocity_body, normal_body);
    Vec3 damping_force = -damping_coefficient * velocity_normal * normal_body;
    entity.add_force(damping_force);

    // Apply friction (material-dependent)
    Real friction_coefficient = terrain.material.friction_coefficient;
    Vec3 friction = -friction_coefficient * cross(normal_body, cross(entity.gravity, velocity_body));
    entity.add_force(friction);
}
```

### Wheel Suspension Model

The terrain system integrates with WheelSuspension for multi-wheel vehicles:

```cpp
namespace jaguar::domain::land {

class WheelSuspension {
public:
    /**
     * Create wheel suspension at body-relative position
     * @param position Position in body frame (m)
     * @param wheel_radius Wheel radius (m)
     * @param unit Suspension unit (spring, damper)
     */
    WheelSuspension(const Vec3& position, Real wheel_radius, const SuspensionUnit& unit);

    /**
     * Compute suspension forces (integrates with terrain)
     * @param state Entity state (position, orientation, velocity)
     * @param env_service EnvironmentService for terrain queries
     * @param dt Time step (s)
     * @return Force in world frame
     */
    Vec3 compute_forces(const physics::EntityState& state,
                       const environment::EnvironmentService* env_service,
                       Real dt);

    /**
     * Get torque contribution about body center
     */
    Vec3 compute_torque(const physics::EntityState& state) const;

    /**
     * Check if wheel is grounded
     */
    bool is_grounded() const { return is_grounded_; }

    /**
     * Get friction coefficient from last terrain query
     */
    Real get_friction_coefficient() const { return friction_coefficient_; }
};

} // namespace jaguar::domain::land
```

Wheel suspension automatically:
1. Queries terrain at wheel position via EnvironmentService
2. Computes penetration against terrain surface
3. Applies spring/damper forces along terrain normal
4. Uses material-specific friction coefficients

### Multi-Wheel Vehicle Suspension

```cpp
class SuspensionModel {
public:
    // Add wheels at body-relative positions
    void add_wheel(const Vec3& position, Real wheel_radius, const SuspensionUnit& unit);

    // Update all wheels and compute total forces/torques
    void update(const physics::EntityState& state,
               const environment::EnvironmentService* env_service,
               Real dt);

    // Get total forces and torques
    Vec3 get_total_force() const;
    Vec3 get_total_torque() const;

    // Check ground contact
    SizeT grounded_wheel_count() const;
};
```

Example vehicle setup:

```cpp
// Create suspension for 4-wheel vehicle
SuspensionModel suspension;

// Front left (body frame)
SuspensionUnit unit{.spring_k = 50000, .damper_c = 5000};
suspension.add_wheel(Vec3{-2.0, 0.8, -0.8}, 0.35, unit);  // Front left
suspension.add_wheel(Vec3{-2.0, -0.8, -0.8}, 0.35, unit); // Front right
suspension.add_wheel(Vec3{2.0, 0.8, -0.8}, 0.35, unit);   // Rear left
suspension.add_wheel(Vec3{2.0, -0.8, -0.8}, 0.35, unit);  // Rear right

// Update each frame
suspension.update(entity_state, &env_service, dt);

// Add suspension forces to total vehicle forces
Vec3 total_suspension_force = suspension.get_total_force();
Vec3 total_suspension_torque = suspension.get_total_torque();
entity.add_force(total_suspension_force);
entity.add_torque(total_suspension_torque);
```

## Data Formats Supported

### DTED (Digital Terrain Elevation Data)

**File format:** Binary, 1/1 arc-second typical
**Extension:** `.dt0`, `.dt1`, `.dt2`
**Grid:** Latitude-longitude aligned
**Type:** Post (pixel at grid point)
**Access:** Via GDAL DTED driver

```bash
# DTED directory structure
/dted/n37/e122/n37e122.dt1   # 1x1 degree tiles
/dted/n38/e122/n38e122.dt1
```

### GeoTIFF

**File format:** GIS raster with geo-referencing
**Extension:** `.tif`, `.tiff`
**Grid:** Any projection with GCPs or geotransform
**Type:** Flexible (DEM, DSM, etc.)
**Access:** Via GDAL GeoTIFF driver

```bash
# Single large GeoTIFF
/geotiff/california.tif          # May be 50GB+
/geotiff/california_cloud_optimized.cog  # Cloud-optimized
```

### Other GDAL Formats

Any GDAL-supported raster:
- HDF5
- NetCDF
- GeoJP2
- ArcGrid (.asc)
- ENVI binary

```cpp
// GDAL auto-detects format
loader.load("/path/to/dem.hdf5");
loader.load("/path/to/dem.nc");
```

## Complete Examples

### Example 1: Simple Elevation Query

```cpp
#include "jaguar/environment/terrain.h"
#include "jaguar/core/coordinates.h"
#include <iostream>
#include <cmath>

using namespace jaguar::environment;
using namespace jaguar::core;

int main() {
    // Create manager
    TerrainManager terrain;

    // Load elevation data
    if (!terrain.load_terrain("/data/elevation/california.tif")) {
        std::cerr << "Failed to load terrain\n";
        return 1;
    }

    // San Francisco location
    double lat = 37.7749 * M_PI / 180.0;   // radians
    double lon = -122.4194 * M_PI / 180.0; // radians

    // Query elevation
    Real elevation = terrain.get_elevation(lat, lon);
    if (!std::isnan(elevation)) {
        std::cout << "San Francisco elevation: " << elevation << " m\n";
    }

    // Query surface properties
    TerrainQuery result = terrain.query(lat, lon);
    if (result.valid) {
        std::cout << "Elevation: " << result.elevation << " m\n";
        std::cout << "Slope: " << (result.slope_angle * 180 / M_PI) << " degrees\n";
        std::cout << "Normal: [" << result.normal.x << ", "
                  << result.normal.y << ", " << result.normal.z << "]\n";
    }

    return 0;
}
```

### Example 2: ECEF-Based Vehicle Query

```cpp
#include "jaguar/environment/terrain.h"
#include "jaguar/environment/environment.h"
#include "jaguar/core/coordinates.h"
#include <iostream>

using namespace jaguar::environment;
using namespace jaguar::core::coordinates;

int main() {
    // Initialize environment service
    EnvironmentService env_service;
    if (!env_service.initialize()) {
        std::cerr << "Failed to initialize environment\n";
        return 1;
    }

    // Load terrain
    if (!env_service.load_terrain("/data/elevation/region.tif")) {
        std::cerr << "Warning: No terrain data loaded, using defaults\n";
    }

    // Vehicle position in ECEF (example: some location)
    Vec3 vehicle_ecef{-2706520.0, -4276220.0, 3890950.0};

    // Query terrain (automatic ECEF → geodetic conversion)
    TerrainQuery terrain_at_vehicle = env_service.query_terrain(vehicle_ecef);

    if (terrain_at_vehicle.valid) {
        std::cout << "Vehicle elevation: " << terrain_at_vehicle.elevation << " m\n";
        std::cout << "Ground friction: " << terrain_at_vehicle.material.friction_coefficient << "\n";

        // Compute ground surface altitude
        GeodeticPosition lla = ecef_to_lla(vehicle_ecef);
        Vec3 ground_ecef = geodetic_to_ecef(
            lla.latitude,
            lla.longitude,
            terrain_at_vehicle.elevation
        );

        Real penetration = vehicle_ecef.z - ground_ecef.z;
        std::cout << "Penetration depth: " << penetration << " m\n";
    } else {
        std::cout << "Vehicle out of terrain bounds\n";
    }

    return 0;
}
```

### Example 3: Parallel Physics with Terrain Integration

```cpp
#include "jaguar/environment/terrain.h"
#include "jaguar/physics/entity.h"
#include "jaguar/core/threading/thread_pool.h"
#include <vector>
#include <future>

using namespace jaguar::environment;
using namespace jaguar::physics;
using namespace jaguar::core::threading;

struct Vehicle {
    EntityId id;
    Vec3 position_ecef;
    Vec3 velocity_ecef;
    Real mass{1000.0};  // kg
};

// Compute forces on vehicle using terrain
Vec3 compute_vehicle_forces(
    const Vehicle& vehicle,
    const TerrainManager& terrain,
    Real dt)
{
    Vec3 forces{0, 0, 0};

    // Gravity
    Vec3 gravity_force{0, 0, vehicle.mass * 9.81};
    forces += gravity_force;

    // Terrain interaction
    TerrainQuery ground = terrain.query_ecef(vehicle.position_ecef);
    if (ground.valid) {
        // Compute ground altitude
        GeodeticPosition lla = ecef_to_lla(vehicle.position_ecef);
        Vec3 ground_point = geodetic_to_ecef(
            lla.latitude,
            lla.longitude,
            ground.elevation
        );

        Real penetration = ground_point.z - vehicle.position_ecef.z;
        if (penetration > 0) {
            // Contact force: spring + damping
            Real spring_stiffness = 50000.0;  // N/m
            Real damping_coeff = 5000.0;      // N·s/m

            Vec3 normal_ned = ground.normal;
            // Convert NED to ECEF (simplified - actual conversion more complex)
            Vec3 normal_ecef = normal_ned;  // Approximation

            Real contact_force = penetration * spring_stiffness;
            Real vel_component = dot(vehicle.velocity_ecef, normal_ecef);
            Real damping_force = damping_coeff * vel_component;

            forces += normal_ecef * (contact_force + damping_force);

            // Friction
            Real friction_coeff = ground.material.friction_coefficient;
            Vec3 friction_force = -friction_coeff * vehicle.velocity_ecef;
            forces += friction_force;
        }
    }

    return forces;
}

int main() {
    // Setup
    TerrainManager terrain;
    terrain.load_terrain("/data/elevation/region.tif");
    terrain.set_cache_size(512);

    WorkStealingThreadPool pool;

    // Create vehicles
    std::vector<Vehicle> vehicles(10000);
    for (size_t i = 0; i < vehicles.size(); ++i) {
        vehicles[i].id = i;
        // Initialize positions (example: random)
        vehicles[i].position_ecef = Vec3{-2700000 + (i % 1000) * 100, -4270000, 3890000};
    }

    // Simulation loop
    Real dt = 0.016;  // 60 Hz
    for (int step = 0; step < 1000; ++step) {
        // Update terrain focus
        Vec3 center = vehicles[0].position_ecef;
        terrain.set_focus_point(center);
        terrain.update();

        // Compute forces in parallel
        std::vector<std::future<Vec3>> force_futures;
        for (auto& vehicle : vehicles) {
            auto future = pool.enqueue(
                [&vehicle, &terrain, dt]() {
                    return compute_vehicle_forces(vehicle, terrain, dt);
                }
            );
            force_futures.push_back(std::move(future));
        }

        // Integrate (sequential)
        for (size_t i = 0; i < vehicles.size(); ++i) {
            Vec3 forces = force_futures[i].get();

            // Simple Euler integration
            Vec3 acceleration = forces / vehicles[i].mass;
            vehicles[i].velocity_ecef += acceleration * dt;
            vehicles[i].position_ecef += vehicles[i].velocity_ecef * dt;
        }

        if (step % 60 == 0) {
            std::cout << "Step " << step << "\n";
        }
    }

    return 0;
}
```

### Example 4: Multi-Wheel Vehicle with Suspension

```cpp
#include "jaguar/domain/land/suspension.h"
#include "jaguar/environment/environment.h"
#include "jaguar/physics/entity.h"
#include <iostream>
#include <vector>

using namespace jaguar::domain::land;
using namespace jaguar::environment;
using namespace jaguar::physics;

int main() {
    // Initialize environment and terrain
    EnvironmentService env_service;
    env_service.initialize();
    env_service.load_terrain("/data/elevation/terrain.tif");

    // Create suspension system for 4-wheel vehicle
    SuspensionModel suspension;

    // Spring-damper unit (same for all wheels)
    SuspensionUnit unit;
    unit.spring_k = 50000.0;   // N/m
    unit.damper_c = 5000.0;    // N·s/m
    unit.travel_max = 0.3;     // 30 cm max compression
    unit.travel_min = 0.0;

    // Add wheels at body-relative positions (in meters)
    // Typical 4-wheel vehicle wheelbase ~2.5m, track ~1.6m
    Real wheelbase = 2.5;
    Real track = 1.6 / 2;
    Real wheel_radius = 0.35;

    suspension.add_wheel(Vec3{-wheelbase/2, track, -0.8}, wheel_radius, unit);   // Front left
    suspension.add_wheel(Vec3{-wheelbase/2, -track, -0.8}, wheel_radius, unit);  // Front right
    suspension.add_wheel(Vec3{wheelbase/2, track, -0.8}, wheel_radius, unit);    // Rear left
    suspension.add_wheel(Vec3{wheelbase/2, -track, -0.8}, wheel_radius, unit);   // Rear right

    // Vehicle state
    EntityState vehicle_state;
    vehicle_state.position_ecef = Vec3{-2700000, -4270000, 3890000};
    vehicle_state.velocity_ecef = Vec3{10, 0, 0};  // 10 m/s forward
    vehicle_state.orientation = Quaternion::identity();

    // Update suspension and get forces/torques
    Real dt = 0.016;  // 60 Hz
    suspension.update(vehicle_state, &env_service, dt);

    Vec3 total_force = suspension.get_total_force();
    Vec3 total_torque = suspension.get_total_torque();
    SizeT grounded_wheels = suspension.grounded_wheel_count();

    std::cout << "Total suspension force: [" << total_force.x << ", "
              << total_force.y << ", " << total_force.z << "] N\n";
    std::cout << "Grounded wheels: " << grounded_wheels << "/4\n";

    // Check individual wheel status
    for (SizeT i = 0; i < suspension.wheel_count(); ++i) {
        const WheelSuspension& wheel = suspension.wheel(i);
        std::cout << "Wheel " << i << ": ";
        std::cout << (wheel.is_grounded() ? "grounded" : "airborne");
        std::cout << ", friction=" << wheel.get_friction_coefficient() << "\n";
    }

    return 0;
}
```

### Example 5: Conditional Compilation (GDAL Check)

```cpp
#include "jaguar/environment/terrain.h"
#include <iostream>

using namespace jaguar::environment;

int main() {
    // Check if GDAL is available
    bool gdal_available = GDALTerrainLoader::is_available();
    std::cout << "GDAL support: " << (gdal_available ? "YES" : "NO") << "\n";

    TerrainManager terrain;

    if (gdal_available) {
        // GDAL is available - can load real terrain data
        if (terrain.load_terrain("/data/elevation/dem.tif")) {
            std::cout << "Terrain loaded successfully\n";
            Real elev = terrain.get_elevation(37.7749 * 3.14159 / 180, -122.4194 * 3.14159 / 180);
            std::cout << "Elevation: " << elev << " m\n";
        } else {
            std::cout << "Failed to load terrain file\n";
        }
    } else {
        // Stub implementation - queries return defaults
        std::cout << "GDAL not available, using flat ground\n";
        Real elev = terrain.get_elevation(37.7749 * 3.14159 / 180, -122.4194 * 3.14159 / 180);
        std::cout << "Default elevation: " << elev << " m\n";
    }

    return 0;
}
```

## Troubleshooting

### GDAL Not Available

**Problem:** `GDALTerrainLoader::is_available()` returns false

**Causes:**
1. GDAL development libraries not installed
2. CMake failed to find GDAL during build

**Solution:**
```bash
# Install GDAL
# macOS
brew install gdal

# Ubuntu/Debian
sudo apt-get install libgdal-dev

# Reconfigure and rebuild
rm -rf build/
cmake -DCMAKE_BUILD_TYPE=Release ..
cmake --build .
```

### "Elevation is NaN"

**Problem:** `get_elevation()` returns NaN (std::numeric_limits<Real>::quiet_NaN())

**Causes:**
1. No terrain data loaded
2. Query position is outside dataset bounds
3. Dataset has no-data value at that location

**Debugging:**
```cpp
TerrainManager terrain;
if (!terrain.is_terrain_loaded()) {
    std::cerr << "No terrain loaded\n";
    return;
}

// Check bounds
double lat = 37.7749 * M_PI / 180.0;
double lon = -122.4194 * M_PI / 180.0;

// Query with validity check
TerrainQuery result = terrain.query(lat, lon);
if (!result.valid) {
    std::cerr << "Query out of bounds or no data\n";
    return;
}

// result.elevation is now safe to use
```

### Contention in Multi-Threaded Physics

**Problem:** Physics threads are slow despite parallel queries

**Causes:**
1. Many threads querying same region (shared_mutex contention)
2. Cache misses (working set too large)
3. Query latency (disk I/O not complete)

**Solutions:**
```cpp
// 1. Ensure terrain is pre-loaded
terrain.set_focus_point(player_position);
for (int i = 0; i < 5; ++i) {
    terrain.update();  // Populate cache
}

// 2. Increase cache size
terrain.set_cache_size(512);  // More memory, fewer misses

// 3. Profile queries
#ifdef PROFILE_TERRAIN
auto t0 = clock::now();
TerrainQuery result = terrain.query(lat, lon);
auto elapsed = clock::now() - t0;
if (elapsed > 1000us) {
    std::cout << "Slow query: " << elapsed.count() << " μs\n";
}
#endif
```

### Incorrect Surface Normals

**Problem:** Physics behaves incorrectly (wrong contact forces)

**Causes:**
1. NED normal misinterpreted in body/ECEF frame
2. Coordinate transformation error
3. Slope computation from normal incorrect

**Verification:**
```cpp
Vec3 ned_normal = terrain.get_surface_normal(lat, lon);

// Verify it's unit magnitude
Real magnitude = std::sqrt(
    ned_normal.x * ned_normal.x +
    ned_normal.y * ned_normal.y +
    ned_normal.z * ned_normal.z
);
assert(std::abs(magnitude - 1.0) < 0.01);

// Verify slope computation
Real slope_angle = terrain.get_slope_angle(lat, lon);
Real expected_slope = std::acos(ned_normal.z);
assert(std::abs(slope_angle - expected_slope) < 0.01);

// Verify reasonable values
// Flat terrain: normal ≈ [0, 0, 1], slope ≈ 0
// 30° slope: normal ≈ [?, ?, 0.866], slope ≈ 0.524 rad (30°)
// 60° slope: normal ≈ [?, ?, 0.5], slope ≈ 1.047 rad (60°)
```

### Performance Issues

**Problem:** Terrain queries are slow or causing frame drops

| Issue | Cause | Solution |
|-------|-------|----------|
| First query slow | Cold start, GDAL opening file | Pre-warm cache: `terrain.update()` × 5 |
| Random queries slow | Cache misses | Use focus point + update |
| Consistent latency | Disk I/O unavoidable | Increase cache size |
| Contention visible | Too many threads | Reduce physics threads or batch queries |

### Build Errors with GDAL

**Error:** `undefined reference to 'GDALAllRegister'`

**Cause:** GDAL library not linked

**Solution:**
```bash
# Check CMakeLists.txt includes:
# find_package(GDAL REQUIRED)
# target_link_libraries(jaguar PUBLIC ${GDAL_LIBRARIES})

# Verify GDAL installation
gdal-config --version
gdal-config --libs

# Manual fix if needed
export GDAL_DIR=/usr/local/opt/gdal  # or your install path
cmake -DCMAKE_BUILD_TYPE=Release ..
cmake --build .
```

### Terrain Data Issues

**Problem:** Loaded terrain but elevations seem wrong

**Checklist:**
1. Verify file path correct: `ls -la /path/to/dem.tif`
2. Check file is valid GeoTIFF: `gdalinfo /path/to/dem.tif`
3. Verify coordinate system is WGS84 or transformable
4. Check for no-data values: `gdalinfo -mm /path/to/dem.tif`
5. Compare with known reference elevation

```bash
# Diagnose file
gdalinfo /path/to/dem.tif | grep -E "Driver|Size|Corner|NoData|Band Type"

# Expected output:
# Driver: GTiff/GeoTIFF
# Size is 3601, 3601
# Upper Left  (-122.0042, 37.0083)
# Lower Right (-119.0000, 34.0042)
# NoData Value=-9999
# Band 1 Block=3601x1 Type=Int16
```

## See Also

- [Physics Module](PHYSICS.md) - Terrain-aware physics integration, suspension systems
- [Atmosphere Module](ATMOSPHERE.md) - Environmental conditions at location
- [Environment Module](ENVIRONMENT.md) - Combined environment queries (terrain + atmosphere + ocean)
- [Threading Module](THREADING.md) - Parallel force computation patterns
- [Coordinates Module](COORDINATES.md) - ECEF, geodetic, and local frame conversions
- [GDAL Documentation](https://gdal.org/index.html) - Supported raster formats and drivers
- [WGS84 Specification](https://en.wikipedia.org/wiki/World_Geodetic_System) - Geodetic reference system
