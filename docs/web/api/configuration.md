# Configuration API Reference

XML configuration loading and engine/entity configuration.

**Header:** `jaguar/interface/config.h`

## EngineConfig

Engine-wide configuration settings.

```cpp
struct EngineConfig {
    // Simulation
    Real time_step{0.01};                           // Integration step (s)
    SizeT max_entities{10000};                      // Maximum entities
    CoordinateFrame coordinate_frame{CoordinateFrame::ECEF};

    // Threading
    int physics_threads{-1};                        // -1 = auto-detect
    int io_threads{2};
    bool work_stealing{true};

    // Terrain
    bool terrain_enabled{true};
    SizeT terrain_cache_mb{2048};
    int terrain_tile_size{256};
    std::vector<std::string> terrain_data_paths;

    // Atmosphere
    std::string atmosphere_model{"us_standard_1976"};
    bool weather_enabled{true};

    // Network (DIS)
    bool dis_enabled{false};
    int dis_port{3000};
    int dis_site_id{1};
    int dis_app_id{1};
    std::string dis_address{"239.1.2.3"};

    // Network (HLA)
    bool hla_enabled{false};
    std::string hla_federation;
    std::string hla_federate;
    std::string hla_fom_path;

    // Static methods
    static EngineConfig load(const std::string& path);
    static EngineConfig defaults();
    bool save(const std::string& path) const;
    bool validate() const;
};
```

### XML Format

```xml
<?xml version="1.0" encoding="UTF-8"?>
<engine_config version="1.0">
    <simulation>
        <time_step>0.01</time_step>
        <max_entities>10000</max_entities>
        <coordinate_frame>ECEF</coordinate_frame>
    </simulation>

    <threading>
        <physics_threads>-1</physics_threads>
        <io_threads>2</io_threads>
        <work_stealing>true</work_stealing>
    </threading>

    <terrain enabled="true">
        <cache_mb>2048</cache_mb>
        <tile_size>256</tile_size>
        <data_path>/data/dted/</data_path>
        <data_path>/data/srtm/</data_path>
    </terrain>

    <atmosphere>
        <model>us_standard_1976</model>
        <weather_enabled>true</weather_enabled>
    </atmosphere>

    <network>
        <dis enabled="false">
            <port>3000</port>
            <site_id>1</site_id>
            <app_id>1</app_id>
        </dis>
        <hla enabled="false">
            <federation>MyFederation</federation>
            <federate>JaguarEngine</federate>
        </hla>
    </network>
</engine_config>
```

### Example

```cpp
// Load from file
auto cfg = config::EngineConfig::load("config/engine.xml");

// Or create programmatically
config::EngineConfig cfg;
cfg.time_step = 0.005;           // 200 Hz
cfg.physics_threads = 8;
cfg.terrain_enabled = true;
cfg.terrain_cache_mb = 4096;
cfg.terrain_data_paths.push_back("/data/terrain/");

// Validate
if (!cfg.validate()) {
    std::cerr << "Invalid configuration\n";
}

// Initialize engine
Engine engine;
engine.initialize(cfg);

// Save for later
cfg.save("config/my_config.xml");
```

## EntityConfig

Entity definition configuration.

```cpp
struct EntityConfig {
    std::string name;
    Domain domain{Domain::Generic};

    // Mass properties
    Real empty_mass{0.0};           // kg
    Real fuel_capacity{0.0};        // kg
    Vec3 cg_location{0, 0, 0};      // m
    Mat3x3 inertia{};               // kg·m²

    // Geometry
    Real wingspan{0.0};             // m
    Real length{0.0};               // m
    Real height{0.0};               // m
    Real reference_area{0.0};       // m²
    Real reference_chord{0.0};      // m

    // Component flags
    ComponentMask components{0};

    // Path to aerodynamics data (if any)
    std::string aero_data_path;

    // Path to propulsion data (if any)
    std::string propulsion_data_path;

    // Static methods
    static EntityConfig load(const std::string& path);
};
```

### Aircraft XML

```xml
<?xml version="1.0" encoding="UTF-8"?>
<entity type="aircraft" name="F-16C">
    <description>General Dynamics F-16C</description>

    <metrics>
        <wingspan unit="ft">32.8</wingspan>
        <length unit="ft">49.5</length>
        <height unit="ft">16.7</height>
        <wing_area unit="ft2">300.0</wing_area>
        <wing_chord unit="ft">11.32</wing_chord>
    </metrics>

    <mass_balance>
        <empty_weight unit="lbs">19700</empty_weight>
        <fuel_capacity unit="lbs">6972</fuel_capacity>

        <center_of_gravity unit="in">
            <x>180.0</x>
            <y>0.0</y>
            <z>0.0</z>
        </center_of_gravity>

        <inertia unit="slug*ft2">
            <ixx>9496</ixx>
            <iyy>55814</iyy>
            <izz>63100</izz>
            <ixz>982</ixz>
        </inertia>
    </mass_balance>

    <aerodynamics>
        <!-- Aerodynamic tables -->
    </aerodynamics>

    <propulsion>
        <engine type="turbofan" name="F110-GE-129">
            <max_thrust unit="lbf">17000</max_thrust>
            <afterburner_thrust unit="lbf">29000</afterburner_thrust>
        </engine>
    </propulsion>

    <flight_control>
        <!-- FCS definition -->
    </flight_control>
</entity>
```

### Ground Vehicle XML

```xml
<?xml version="1.0" encoding="UTF-8"?>
<entity type="ground_vehicle" name="M1A2">
    <metrics>
        <length unit="m">9.77</length>
        <width unit="m">3.66</width>
        <height unit="m">2.44</height>
    </metrics>

    <mass_balance>
        <combat_weight unit="kg">62000</combat_weight>
        <fuel_capacity unit="L">1900</fuel_capacity>

        <inertia unit="kg*m2">
            <ixx>50000</ixx>
            <iyy>200000</iyy>
            <izz>220000</izz>
        </inertia>
    </mass_balance>

    <tracks>
        <track_width unit="m">0.63</track_width>
        <track_length unit="m">4.6</track_length>
        <sprocket_radius unit="m">0.33</sprocket_radius>
    </tracks>

    <suspension type="torsion_bar">
        <road_wheels_per_side>7</road_wheels_per_side>
        <spring_rate unit="N/m">300000</spring_rate>
        <damping_rate unit="N*s/m">30000</damping_rate>
    </suspension>
</entity>
```

### Ship XML

```xml
<?xml version="1.0" encoding="UTF-8"?>
<entity type="surface_ship" name="DDG-51">
    <metrics>
        <length unit="m">154.0</length>
        <beam unit="m">20.0</beam>
        <draft unit="m">9.4</draft>
    </metrics>

    <mass_balance>
        <displacement unit="tonnes">8600</displacement>
        <metacentric_height unit="m">2.5</metacentric_height>
    </mass_balance>

    <hydrodynamics>
        <hull_coefficients>
            <x_vv>-0.04</x_vv>
            <y_v>-0.4</y_v>
            <n_v>-0.1</n_v>
        </hull_coefficients>

        <rudder>
            <area unit="m2">18.0</area>
            <max_angle unit="deg">35</max_angle>
        </rudder>

        <propeller>
            <diameter unit="m">5.2</diameter>
            <max_rpm>180</max_rpm>
        </propeller>
    </hydrodynamics>
</entity>
```

### Spacecraft XML

```xml
<?xml version="1.0" encoding="UTF-8"?>
<entity type="spacecraft" name="GPS-IIR">
    <mass_balance>
        <dry_mass unit="kg">1080</dry_mass>
        <fuel_mass unit="kg">350</fuel_mass>
    </mass_balance>

    <aerodynamics>
        <drag_coefficient>2.2</drag_coefficient>
        <cross_section unit="m2">10.0</cross_section>
    </aerodynamics>

    <srp>
        <reflectivity>0.8</reflectivity>
        <area unit="m2">15.0</area>
    </srp>

    <orbit>
        <semi_major_axis unit="km">26560</semi_major_axis>
        <eccentricity>0.0</eccentricity>
        <inclination unit="deg">55.0</inclination>
    </orbit>
</entity>
```

## ConfigLoader

Configuration file loading and path management.

```cpp
class ConfigLoader {
public:
    // Add search paths
    void add_search_path(const std::string& path);
    void clear_search_paths();

    // Find file in search paths
    std::string find_file(const std::string& filename) const;
    bool file_exists(const std::string& filename) const;

    // Load configurations
    EngineConfig load_engine_config(const std::string& path);
    EntityConfig load_entity_config(const std::string& path);

    // Load aerodynamic tables
    std::unique_ptr<domain::air::AeroTable> load_aero_table(
        const std::string& path);

    // Load material database
    std::vector<domain::land::SoilProperties> load_soil_database(
        const std::string& path);
};
```

### Example

```cpp
config::ConfigLoader loader;

// Add search paths
loader.add_search_path("/opt/jaguar/data/");
loader.add_search_path("./data/");
loader.add_search_path(std::getenv("JAGUAR_DATA"));

// Load engine config
auto engine_cfg = loader.load_engine_config("config/engine.xml");

// Load entity configs
auto f16_cfg = loader.load_entity_config("aircraft/f16.xml");
auto tank_cfg = loader.load_entity_config("vehicles/m1a2.xml");

// Create entities from config
EntityId f16 = engine.create_entity_from_config(f16_cfg);
EntityId tank = engine.create_entity_from_config(tank_cfg);
```

## Unit System

Configuration files support multiple units with automatic conversion.

| Quantity | Supported Units |
|----------|-----------------|
| Length | m, ft, in, cm, mm, km, nm (nautical mile) |
| Mass | kg, lbs, slug, g |
| Force | N, lbf, kN |
| Angle | rad, deg |
| Area | m2, ft2, in2 |
| Velocity | m/s, ft/s, kts, km/h, mph |
| Inertia | kg*m2, slug*ft2 |
| Pressure | Pa, psi, atm, bar, hPa |
| Temperature | K, C, F |
| Time | s, ms, min, hr |

### Example

```xml
<!-- All internally converted to SI -->
<wingspan unit="ft">32.8</wingspan>     <!-- → 10.0 m -->
<mass unit="lbs">19700</mass>           <!-- → 8936 kg -->
<angle unit="deg">45.0</angle>          <!-- → 0.785 rad -->
<velocity unit="kts">250</velocity>     <!-- → 128.6 m/s -->
```

## See Also

- [API Overview](overview.md) - Complete API index
- [Getting Started](../getting-started/quickstart.md) - Quick start guide
- [Engine Setup](../getting-started/first-simulation.md) - First simulation

