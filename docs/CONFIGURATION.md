# Configuration Guide

This guide covers JaguarEngine's XML configuration system for engine settings, entity definitions, and data files.

## Table of Contents

1. [Configuration Overview](#configuration-overview)
2. [Engine Configuration](#engine-configuration)
3. [Entity Configuration](#entity-configuration)
4. [Aerodynamics Data](#aerodynamics-data)
5. [Material Databases](#material-databases)
6. [Configuration API](#configuration-api)

## Configuration Overview

JaguarEngine uses XML files for configuration, inspired by JSBSim's flight dynamics model format. Configuration files are organized as:

```
data/
├── config/
│   └── engine.xml          # Engine settings
├── aircraft/
│   ├── f16.xml             # Aircraft definitions
│   └── ...
├── vehicles/
│   ├── m1a2.xml            # Ground vehicles
│   └── ...
├── ships/
│   ├── ddg51.xml           # Naval vessels
│   └── ...
├── spacecraft/
│   ├── gps_iir.xml         # Space vehicles
│   └── ...
└── materials/
    └── soil_types.xml      # Terrain materials
```

### Unit System

Configuration files support multiple unit systems. Specify units with the `unit` attribute:

| Quantity | Supported Units |
|----------|-----------------|
| Length | m, ft, in, cm, mm |
| Mass | kg, lbs, slug |
| Force | N, lbf, kN |
| Angle | rad, deg |
| Area | m2, ft2, in2 |
| Velocity | m/s, ft/s, kts, km/h |
| Inertia | kg*m2, slug*ft2 |
| Pressure | Pa, psi, atm, bar |
| Temperature | K, C, F |

**Example:**
```xml
<wingspan unit="ft">32.8</wingspan>    <!-- Internally converted to meters -->
<mass unit="lbs">19700</mass>          <!-- Internally converted to kg -->
<angle unit="deg">45.0</angle>         <!-- Internally converted to radians -->
```

## Engine Configuration

### Basic Engine Configuration

```xml
<?xml version="1.0" encoding="UTF-8"?>
<engine_config version="1.0">
    <!-- Simulation Settings -->
    <simulation>
        <time_step>0.01</time_step>           <!-- Seconds -->
        <max_entities>10000</max_entities>
        <coordinate_frame>ECEF</coordinate_frame>
    </simulation>

    <!-- Threading -->
    <threading>
        <physics_threads>-1</physics_threads> <!-- -1 = auto detect -->
        <io_threads>2</io_threads>
        <work_stealing>true</work_stealing>
    </threading>

    <!-- Terrain -->
    <terrain enabled="true">
        <cache_mb>2048</cache_mb>
        <tile_size>256</tile_size>
        <data_path>/data/dted/</data_path>
        <data_path>/data/srtm/</data_path>
    </terrain>

    <!-- Atmosphere -->
    <atmosphere>
        <model>us_standard_1976</model>
        <weather_enabled>true</weather_enabled>
    </atmosphere>

    <!-- Network Interfaces -->
    <network>
        <dis enabled="false">
            <port>3000</port>
            <site_id>1</site_id>
            <app_id>1</app_id>
        </dis>

        <hla enabled="false">
            <federation>ExampleFederation</federation>
            <federate>JaguarEngine</federate>
        </hla>
    </network>
</engine_config>
```

### Configuration Parameters

#### Simulation Section

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `time_step` | Real | 0.01 | Integration time step (seconds) |
| `max_entities` | Integer | 10000 | Maximum concurrent entities |
| `coordinate_frame` | String | ECEF | Primary coordinate frame: `ECEF`, `NED`, `ENU` |

#### Threading Section

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `physics_threads` | Integer | -1 | Physics worker threads (-1 = auto) |
| `io_threads` | Integer | 2 | I/O worker threads |
| `work_stealing` | Boolean | true | Enable work-stealing scheduler |

#### Terrain Section

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `enabled` | Boolean | true | Enable terrain system |
| `cache_mb` | Integer | 2048 | Terrain cache size (MB) |
| `tile_size` | Integer | 256 | Terrain tile resolution |
| `data_path` | String[] | - | Paths to terrain data |

#### Atmosphere Section

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `model` | String | us_standard_1976 | Atmosphere model |
| `weather_enabled` | Boolean | true | Enable weather effects |

## Entity Configuration

### Aircraft Entity

```xml
<?xml version="1.0" encoding="UTF-8"?>
<entity type="aircraft" name="F-16C">
    <description>
        General Dynamics F-16C Fighting Falcon
        Single-engine multirole fighter aircraft
    </description>

    <!-- Reference Geometry -->
    <metrics>
        <wingspan unit="ft">32.8</wingspan>
        <length unit="ft">49.5</length>
        <height unit="ft">16.7</height>
        <wing_area unit="ft2">300.0</wing_area>
        <wing_chord unit="ft">11.32</wing_chord>
    </metrics>

    <!-- Mass Properties -->
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
            <ixy>0</ixy>
            <ixz>982</ixz>
            <iyz>0</iyz>
        </inertia>
    </mass_balance>

    <!-- Aerodynamics (see Aerodynamics Data section) -->
    <aerodynamics>
        <!-- ... -->
    </aerodynamics>

    <!-- Propulsion -->
    <propulsion>
        <engine type="turbofan" name="F110-GE-129">
            <location unit="in">
                <x>400.0</x>
                <y>0.0</y>
                <z>0.0</z>
            </location>

            <max_thrust unit="lbf">17000</max_thrust>
            <afterburner_thrust unit="lbf">29000</afterburner_thrust>
            <tsfc>0.76</tsfc>           <!-- lb/hr/lbf -->
            <tsfc_ab>2.05</tsfc_ab>     <!-- Afterburner TSFC -->

            <!-- Thrust table (Mach vs Altitude) -->
            <thrust_table>
                <independentVar lookup="row">velocities/mach</independentVar>
                <independentVar lookup="column">position/altitude-ft</independentVar>
                <tableData>
                            0       10000   20000   30000   40000
                    0.0     1.00    0.85    0.70    0.55    0.40
                    0.5     1.05    0.90    0.75    0.60    0.45
                    0.9     1.10    0.95    0.80    0.65    0.50
                    1.2     1.00    0.85    0.70    0.55    0.40
                    1.5     0.85    0.72    0.60    0.47    0.34
                </tableData>
            </thrust_table>
        </engine>
    </propulsion>

    <!-- Flight Control System -->
    <flight_control>
        <channel name="pitch">
            <summer>
                <input>fcs/pitch-cmd</input>
                <input>fcs/pitch-trim</input>
                <clipto>
                    <min>-1.0</min>
                    <max>1.0</max>
                </clipto>
            </summer>
            <aerosurface_scale>
                <input>fcs/pitch-sum</input>
                <range>
                    <min>-25.0</min>  <!-- degrees -->
                    <max>25.0</max>
                </range>
                <output>fcs/elevator-deg</output>
            </aerosurface_scale>
        </channel>

        <channel name="roll">
            <summer>
                <input>fcs/roll-cmd</input>
                <input>fcs/roll-trim</input>
            </summer>
            <aerosurface_scale>
                <input>fcs/roll-sum</input>
                <range>
                    <min>-20.0</min>
                    <max>20.0</max>
                </range>
                <output>fcs/aileron-deg</output>
            </aerosurface_scale>
        </channel>

        <channel name="yaw">
            <summer>
                <input>fcs/yaw-cmd</input>
                <input>fcs/yaw-trim</input>
            </summer>
            <aerosurface_scale>
                <input>fcs/yaw-sum</input>
                <range>
                    <min>-30.0</min>
                    <max>30.0</max>
                </range>
                <output>fcs/rudder-deg</output>
            </aerosurface_scale>
        </channel>
    </flight_control>

    <!-- Landing Gear -->
    <ground_reactions>
        <gear name="nose_gear">
            <location unit="in">
                <x>80.0</x>
                <y>0.0</y>
                <z>-80.0</z>
            </location>
            <spring_coeff unit="lbs/ft">5000</spring_coeff>
            <damping_coeff unit="lbs/ft/sec">1500</damping_coeff>
            <max_steer unit="deg">75</max_steer>
        </gear>

        <gear name="left_main">
            <location unit="in">
                <x>200.0</x>
                <y>-60.0</y>
                <z>-80.0</z>
            </location>
            <spring_coeff unit="lbs/ft">15000</spring_coeff>
            <damping_coeff unit="lbs/ft/sec">4500</damping_coeff>
        </gear>

        <gear name="right_main">
            <location unit="in">
                <x>200.0</x>
                <y>60.0</y>
                <z>-80.0</z>
            </location>
            <spring_coeff unit="lbs/ft">15000</spring_coeff>
            <damping_coeff unit="lbs/ft/sec">4500</damping_coeff>
        </gear>
    </ground_reactions>
</entity>
```

### Ground Vehicle Entity

```xml
<?xml version="1.0" encoding="UTF-8"?>
<entity type="ground_vehicle" name="M1A2">
    <description>M1A2 Abrams Main Battle Tank</description>

    <metrics>
        <length unit="m">9.77</length>
        <width unit="m">3.66</width>
        <height unit="m">2.44</height>
    </metrics>

    <mass_balance>
        <combat_weight unit="kg">62000</combat_weight>
        <fuel_capacity unit="L">1900</fuel_capacity>

        <center_of_gravity unit="m">
            <x>0.0</x>
            <y>0.0</y>
            <z>0.5</z>
        </center_of_gravity>

        <inertia unit="kg*m2">
            <ixx>50000</ixx>
            <iyy>200000</iyy>
            <izz>220000</izz>
        </inertia>
    </mass_balance>

    <!-- Track System -->
    <tracks>
        <track_width unit="m">0.63</track_width>
        <track_length unit="m">4.6</track_length>
        <sprocket_radius unit="m">0.33</sprocket_radius>
        <max_torque unit="N*m">100000</max_torque>
    </tracks>

    <!-- Suspension -->
    <suspension type="torsion_bar">
        <road_wheels_per_side>7</road_wheels_per_side>
        <spring_rate unit="N/m">300000</spring_rate>
        <damping_rate unit="N*s/m">30000</damping_rate>
        <travel unit="m">0.40</travel>
    </suspension>

    <!-- Powerpack -->
    <engine type="gas_turbine" name="AGT1500">
        <max_power unit="hp">1500</max_power>
        <fuel_consumption unit="L/km">4.0</fuel_consumption>
    </engine>
</entity>
```

### Naval Vessel Entity

```xml
<?xml version="1.0" encoding="UTF-8"?>
<entity type="surface_ship" name="DDG-51">
    <description>Arleigh Burke-class Destroyer</description>

    <metrics>
        <length unit="m">154.0</length>
        <beam unit="m">20.0</beam>
        <draft unit="m">9.4</draft>
    </metrics>

    <mass_balance>
        <displacement unit="tonnes">8600</displacement>
        <fuel_capacity unit="tonnes">500</fuel_capacity>

        <metacentric_height unit="m">2.5</metacentric_height>
        <center_of_buoyancy unit="m">
            <x>0.0</x>
            <y>0.0</y>
            <z>-4.7</z>
        </center_of_buoyancy>
    </mass_balance>

    <!-- Hull Hydrodynamics (MMG coefficients) -->
    <hydrodynamics>
        <hull_coefficients>
            <x_vv>-0.04</x_vv>
            <x_rr>-0.01</x_rr>
            <y_v>-0.4</y_v>
            <y_r>0.05</y_r>
            <n_v>-0.1</n_v>
            <n_r>-0.05</n_r>
        </hull_coefficients>

        <rudder>
            <area unit="m2">18.0</area>
            <aspect_ratio>1.6</aspect_ratio>
            <max_angle unit="deg">35.0</max_angle>
        </rudder>

        <propeller>
            <diameter unit="m">5.2</diameter>
            <pitch_ratio>1.0</pitch_ratio>
            <max_rpm>180</max_rpm>
        </propeller>
    </hydrodynamics>

    <!-- Seakeeping RAO -->
    <seakeeping>
        <rao_heave>
            <frequencies>0.3, 0.5, 0.7, 1.0, 1.5</frequencies>
            <amplitudes>0.9, 1.0, 0.95, 0.7, 0.4</amplitudes>
            <phases>0.0, 0.1, 0.2, 0.4, 0.6</phases>
        </rao_heave>
        <rao_roll>
            <frequencies>0.3, 0.5, 0.7, 1.0, 1.5</frequencies>
            <amplitudes>0.02, 0.05, 0.08, 0.04, 0.02</amplitudes>
            <phases>0.0, 0.2, 0.5, 0.8, 1.0</phases>
        </rao_roll>
    </seakeeping>
</entity>
```

### Spacecraft Entity

```xml
<?xml version="1.0" encoding="UTF-8"?>
<entity type="spacecraft" name="GPS-IIR">
    <description>GPS Block IIR Satellite</description>

    <mass_balance>
        <dry_mass unit="kg">1080</dry_mass>
        <fuel_mass unit="kg">350</fuel_mass>

        <inertia unit="kg*m2">
            <ixx>500</ixx>
            <iyy>600</iyy>
            <izz>400</izz>
        </inertia>
    </mass_balance>

    <!-- Drag Properties -->
    <aerodynamics>
        <drag_coefficient>2.2</drag_coefficient>
        <cross_section unit="m2">10.0</cross_section>
    </aerodynamics>

    <!-- Solar Radiation Pressure -->
    <srp>
        <reflectivity>0.8</reflectivity>
        <area unit="m2">15.0</area>
    </srp>

    <!-- Initial Orbit (optional) -->
    <orbit>
        <semi_major_axis unit="km">26560</semi_major_axis>
        <eccentricity>0.0</eccentricity>
        <inclination unit="deg">55.0</inclination>
        <raan unit="deg">0.0</raan>
        <arg_periapsis unit="deg">0.0</arg_periapsis>
        <mean_anomaly unit="deg">0.0</mean_anomaly>
    </orbit>
</entity>
```

## Aerodynamics Data

### Table-Based Coefficients

```xml
<aerodynamics>
    <!-- Lift Coefficient Table -->
    <axis name="LIFT">
        <function name="aero/coefficient/CL">
            <description>Lift coefficient vs alpha and Mach</description>
            <table>
                <independentVar lookup="row">aero/alpha-rad</independentVar>
                <independentVar lookup="column">velocities/mach</independentVar>
                <tableData>
                            0.0     0.5     0.9     1.2
                    -0.35   -0.80   -0.75   -0.70   -0.60
                    -0.17   -0.40   -0.38   -0.35   -0.30
                    0.00    0.20    0.22    0.25    0.20
                    0.17    0.80    0.85    0.90    0.75
                    0.35    1.20    1.25    1.15    0.90
                    0.52    0.90    0.85    0.70    0.50
                </tableData>
            </table>
        </function>
    </axis>

    <!-- Drag Coefficient -->
    <axis name="DRAG">
        <function name="aero/coefficient/CD">
            <table>
                <independentVar lookup="row">aero/alpha-rad</independentVar>
                <independentVar lookup="column">velocities/mach</independentVar>
                <tableData>
                            0.0     0.5     0.9     1.2
                    -0.35   0.080   0.085   0.110   0.150
                    -0.17   0.030   0.032   0.045   0.070
                    0.00    0.020   0.022   0.035   0.055
                    0.17    0.035   0.038   0.055   0.085
                    0.35    0.100   0.110   0.140   0.200
                    0.52    0.200   0.220   0.280   0.350
                </tableData>
            </table>
        </function>
    </axis>

    <!-- Pitching Moment -->
    <axis name="PITCH">
        <function name="aero/coefficient/Cm">
            <table>
                <independentVar>aero/alpha-rad</independentVar>
                <tableData>
                    -0.35    0.10
                    -0.17    0.05
                    0.00     0.00
                    0.17    -0.10
                    0.35    -0.25
                    0.52    -0.40
                </tableData>
            </table>
        </function>
    </axis>

    <!-- Control Surface Effects -->
    <axis name="PITCH">
        <function name="aero/coefficient/Cm_elevator">
            <description>Pitching moment due to elevator</description>
            <product>
                <value>-0.05</value>  <!-- dCm/d_elevator -->
                <property>fcs/elevator-deg</property>
            </product>
        </function>
    </axis>

    <!-- Damping Derivatives -->
    <axis name="ROLL">
        <function name="aero/coefficient/Cl_p">
            <description>Roll damping</description>
            <product>
                <value>-0.4</value>  <!-- Cl_p -->
                <property>aero/p-aero-rad_sec</property>
                <property>aero/qbar-psf</property>
                <property>metrics/bw-ft</property>
                <value>0.5</value>
            </product>
        </function>
    </axis>
</aerodynamics>
```

### Available Properties

| Property | Description | Units |
|----------|-------------|-------|
| `aero/alpha-rad` | Angle of attack | rad |
| `aero/beta-rad` | Sideslip angle | rad |
| `velocities/mach` | Mach number | - |
| `aero/qbar-psf` | Dynamic pressure | lb/ft² |
| `aero/p-aero-rad_sec` | Roll rate | rad/s |
| `aero/q-aero-rad_sec` | Pitch rate | rad/s |
| `aero/r-aero-rad_sec` | Yaw rate | rad/s |
| `position/altitude-ft` | Altitude | ft |
| `fcs/elevator-deg` | Elevator deflection | deg |
| `fcs/aileron-deg` | Aileron deflection | deg |
| `fcs/rudder-deg` | Rudder deflection | deg |
| `metrics/bw-ft` | Wing span | ft |
| `metrics/cbar-ft` | Mean chord | ft |
| `metrics/Sw-ft2` | Wing area | ft² |

## Material Databases

### Soil Types Database

```xml
<?xml version="1.0" encoding="UTF-8"?>
<soil_database version="1.0">
    <soil name="dry_sand" id="1">
        <description>Dry sandy soil</description>
        <bekker_parameters>
            <k_c unit="kN/m^(n+1)">0.99</k_c>
            <k_phi unit="kN/m^(n+2)">1528</k_phi>
            <n>1.10</n>
            <cohesion unit="kPa">1.04</cohesion>
            <friction_angle unit="deg">28</friction_angle>
        </bekker_parameters>
        <surface_properties>
            <friction_coefficient>0.6</friction_coefficient>
            <rolling_resistance>0.04</rolling_resistance>
        </surface_properties>
    </soil>

    <soil name="wet_sand" id="2">
        <description>Wet sandy soil (saturated)</description>
        <bekker_parameters>
            <k_c unit="kN/m^(n+1)">5.27</k_c>
            <k_phi unit="kN/m^(n+2)">1515</k_phi>
            <n>0.73</n>
            <cohesion unit="kPa">1.72</cohesion>
            <friction_angle unit="deg">29</friction_angle>
        </bekker_parameters>
        <surface_properties>
            <friction_coefficient>0.5</friction_coefficient>
            <rolling_resistance>0.08</rolling_resistance>
        </surface_properties>
    </soil>

    <soil name="clay" id="3">
        <description>Clay soil</description>
        <bekker_parameters>
            <k_c unit="kN/m^(n+1)">13.19</k_c>
            <k_phi unit="kN/m^(n+2)">692</k_phi>
            <n>0.50</n>
            <cohesion unit="kPa">4.14</cohesion>
            <friction_angle unit="deg">13</friction_angle>
        </bekker_parameters>
        <surface_properties>
            <friction_coefficient>0.4</friction_coefficient>
            <rolling_resistance>0.10</rolling_resistance>
        </surface_properties>
    </soil>

    <soil name="snow" id="4">
        <description>Compacted snow</description>
        <bekker_parameters>
            <k_c unit="kN/m^(n+1)">4.37</k_c>
            <k_phi unit="kN/m^(n+2)">196</k_phi>
            <n>1.60</n>
            <cohesion unit="kPa">1.03</cohesion>
            <friction_angle unit="deg">19.7</friction_angle>
        </bekker_parameters>
        <surface_properties>
            <friction_coefficient>0.2</friction_coefficient>
            <rolling_resistance>0.15</rolling_resistance>
        </surface_properties>
    </soil>

    <soil name="asphalt" id="5">
        <description>Paved asphalt road</description>
        <bekker_parameters>
            <!-- Essentially rigid - high values -->
            <k_c unit="kN/m^(n+1)">1000000</k_c>
            <k_phi unit="kN/m^(n+2)">1000000</k_phi>
            <n>0.0</n>
            <cohesion unit="kPa">1000</cohesion>
            <friction_angle unit="deg">45</friction_angle>
        </bekker_parameters>
        <surface_properties>
            <friction_coefficient>0.8</friction_coefficient>
            <rolling_resistance>0.01</rolling_resistance>
        </surface_properties>
    </soil>

    <soil name="mud" id="7">
        <description>Wet muddy terrain</description>
        <bekker_parameters>
            <k_c unit="kN/m^(n+1)">2.0</k_c>
            <k_phi unit="kN/m^(n+2)">100</k_phi>
            <n>0.80</n>
            <cohesion unit="kPa">2.0</cohesion>
            <friction_angle unit="deg">5</friction_angle>
        </bekker_parameters>
        <surface_properties>
            <friction_coefficient>0.3</friction_coefficient>
            <rolling_resistance>0.25</rolling_resistance>
        </surface_properties>
    </soil>
</soil_database>
```

### Bekker-Wong Parameters Reference

| Soil Type | k_c | k_phi | n | c (kPa) | φ (deg) | μ |
|-----------|-----|-------|---|---------|---------|---|
| Dry Sand | 0.99 | 1528 | 1.10 | 1.04 | 28 | 0.6 |
| Wet Sand | 5.27 | 1515 | 0.73 | 1.72 | 29 | 0.5 |
| Clay | 13.19 | 692 | 0.50 | 4.14 | 13 | 0.4 |
| Snow | 4.37 | 196 | 1.60 | 1.03 | 19.7 | 0.2 |
| Asphalt | 10⁶ | 10⁶ | 0.0 | 1000 | 45 | 0.8 |
| Mud | 2.0 | 100 | 0.80 | 2.0 | 5 | 0.3 |
| Gravel | 3.0 | 3000 | 0.90 | 0.5 | 35 | 0.7 |

## Configuration API

### Loading Configuration

```cpp
#include "jaguar/interface/config.h"

// Load engine configuration
jaguar::config::ConfigLoader loader;
loader.add_search_path("/data/config/");
loader.add_search_path("./config/");

auto engine_cfg = loader.load_engine_config("engine.xml");

// Initialize engine with configuration
jaguar::Engine engine;
engine.initialize(engine_cfg);

// Load entity configuration
auto f16_cfg = loader.load_entity_config("aircraft/f16.xml");
auto entity_id = engine.create_entity_from_config(f16_cfg);
```

### EngineConfig Structure

```cpp
namespace jaguar::config {

struct EngineConfig {
    // Simulation settings
    Real time_step{0.01};
    SizeT max_entities{10000};
    CoordinateFrame coordinate_frame{CoordinateFrame::ECEF};

    // Threading
    int physics_threads{-1};  // -1 = auto
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

    // Network
    bool dis_enabled{false};
    int dis_port{3000};
    int dis_site_id{1};
    int dis_app_id{1};

    bool hla_enabled{false};
    std::string hla_federation;
    std::string hla_federate;

    // Static methods
    static EngineConfig load(const std::string& path);
    static EngineConfig defaults();
    bool save(const std::string& path) const;
};

}
```

### EntityConfig Structure

```cpp
namespace jaguar::config {

struct EntityConfig {
    std::string name;
    Domain domain{Domain::Generic};

    // Mass properties
    Real empty_mass{0.0};
    Real fuel_capacity{0.0};
    Vec3 cg_location{0, 0, 0};
    Mat3x3 inertia{};

    // Geometry
    Real wingspan{0.0};
    Real length{0.0};
    Real height{0.0};
    Real reference_area{0.0};
    Real reference_chord{0.0};

    // Component flags
    ComponentMask components{0};

    static EntityConfig load(const std::string& path);
};

}
```

### Programmatic Configuration

```cpp
// Create configuration programmatically
jaguar::config::EngineConfig cfg;
cfg.time_step = 0.005;           // 200 Hz
cfg.max_entities = 5000;
cfg.physics_threads = 8;
cfg.terrain_enabled = true;
cfg.terrain_cache_mb = 4096;
cfg.terrain_data_paths.push_back("/data/dted/");
cfg.atmosphere_model = "us_standard_1976";
cfg.weather_enabled = true;

// Save for later use
cfg.save("my_config.xml");

// Initialize with config
jaguar::Engine engine;
engine.initialize(cfg);
```

### Configuration Validation

```cpp
// Validate configuration before use
auto cfg = jaguar::config::EngineConfig::load("engine.xml");

if (cfg.time_step <= 0.0 || cfg.time_step > 1.0) {
    throw std::runtime_error("Invalid time step");
}

if (cfg.max_entities == 0) {
    throw std::runtime_error("max_entities must be > 0");
}

if (cfg.terrain_enabled && cfg.terrain_data_paths.empty()) {
    std::cerr << "Warning: terrain enabled but no data paths specified\n";
}
```

## Best Practices

### Configuration File Organization

```
project/
├── config/
│   ├── engine.xml           # Main engine settings
│   ├── engine_debug.xml     # Debug/development settings
│   └── engine_release.xml   # Production settings
├── data/
│   ├── aircraft/            # Aircraft definitions
│   ├── vehicles/            # Ground vehicles
│   ├── ships/               # Naval vessels
│   ├── spacecraft/          # Space vehicles
│   └── materials/           # Material databases
└── terrain/                 # Terrain data (DTED, SRTM)
```

### Version Control

- Track all XML configuration files in version control
- Use descriptive comments in configuration files
- Keep separate configurations for development and production
- Document custom entity configurations thoroughly

### Performance Tuning

```xml
<!-- High-performance configuration -->
<engine_config>
    <simulation>
        <time_step>0.02</time_step>  <!-- Lower rate for performance -->
        <max_entities>1000</max_entities>
    </simulation>
    <threading>
        <physics_threads>-1</physics_threads>  <!-- Auto-detect -->
        <work_stealing>true</work_stealing>
    </threading>
    <terrain>
        <cache_mb>4096</cache_mb>  <!-- Large cache -->
    </terrain>
</engine_config>

<!-- Accuracy-focused configuration -->
<engine_config>
    <simulation>
        <time_step>0.001</time_step>  <!-- High rate for accuracy -->
        <max_entities>100</max_entities>
    </simulation>
</engine_config>
```

## See Also

- [Examples Guide](EXAMPLES.md) - Usage examples
- [API Reference](API_REFERENCE.md) - Complete API documentation
- [Domain Documentation](modules/) - Domain-specific physics models
