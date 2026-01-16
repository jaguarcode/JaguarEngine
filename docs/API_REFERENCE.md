# JaguarEngine API Reference

Complete API reference for JaguarEngine multi-domain physics simulation platform.

## Table of Contents

1. [Core API](#core-api)
2. [Physics API](#physics-api)
3. [Air Domain API](#air-domain-api)
4. [Land Domain API](#land-domain-api)
5. [Sea Domain API](#sea-domain-api)
6. [Space Domain API](#space-domain-api)
7. [Environment API](#environment-api)
8. [Configuration API](#configuration-api)

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
// Base interface
class IForceGenerator {
public:
    virtual void compute_forces(const EntityState& state,
                                const environment::Environment& env,
                                Real dt,
                                EntityForces& forces) = 0;
};

// Domain-specific interfaces
class IAerodynamicsModel : public IForceGenerator {
public:
    virtual Real get_cl() const = 0;
    virtual Real get_cd() const = 0;
    virtual Real get_cm() const = 0;
    virtual Real get_alpha() const = 0;
    virtual Real get_beta() const = 0;
    virtual Real get_mach() const = 0;
    virtual Real get_qbar() const = 0;
};

class IPropulsionModel : public IForceGenerator {
public:
    virtual Real get_thrust() const = 0;
    virtual Real get_fuel_flow() const = 0;
    virtual Real get_fuel_remaining() const = 0;
    virtual bool is_running() const = 0;
};

class ITerramechanicsModel : public IForceGenerator {
public:
    virtual Real get_sinkage() const = 0;
    virtual Real get_motion_resistance() const = 0;
    virtual Real get_traction() const = 0;
    virtual Real get_slip_ratio() const = 0;
};

class IHydrodynamicsModel : public IForceGenerator {
public:
    virtual Real get_buoyancy() const = 0;
    virtual Real get_draft() const = 0;
    virtual Real get_heel() const = 0;
    virtual Real get_trim() const = 0;
};
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
