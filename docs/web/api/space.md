# Space Domain API Reference

Orbital mechanics, propagators, gravity models, and spacecraft APIs.

**Header:** `jaguar/domain/space.h`

## OrbitalElements

Classical Keplerian orbital elements.

```cpp
struct OrbitalElements {
    Real semi_major_axis{0.0};  // a (m)
    Real eccentricity{0.0};     // e (0 <= e < 1 for ellipse)
    Real inclination{0.0};      // i (rad)
    Real raan{0.0};             // Ω - Right Ascension of Ascending Node (rad)
    Real arg_periapsis{0.0};    // ω - Argument of Periapsis (rad)
    Real mean_anomaly{0.0};     // M (rad)
    Real epoch{0.0};            // Epoch time (Julian date)

    // Conversion methods
    static OrbitalElements from_cartesian(const Vec3& position,
                                          const Vec3& velocity,
                                          Real mu = constants::EARTH_MU);
    void to_cartesian(Vec3& position, Vec3& velocity,
                      Real mu = constants::EARTH_MU) const;

    // Derived quantities
    Real period() const;              // Orbital period (s)
    Real apoapsis() const;            // Apoapsis radius (m)
    Real periapsis() const;           // Periapsis radius (m)
    Real apoapsis_altitude() const;   // Above Earth surface (m)
    Real periapsis_altitude() const;  // Above Earth surface (m)
    Real mean_motion() const;         // n (rad/s)
    Real specific_energy() const;     // J/kg
    Real specific_angular_momentum() const;  // m²/s

    // Anomaly conversions
    Real eccentric_anomaly() const;   // E (rad)
    Real true_anomaly() const;        // ν (rad)

    // Orbit classification
    OrbitType classify() const;
};

enum class OrbitType {
    LEO,      // Low Earth Orbit (< 2000 km)
    MEO,      // Medium Earth Orbit (2000-35786 km)
    GEO,      // Geostationary (35786 km)
    HEO,      // Highly Elliptical Orbit
    Escape,   // e >= 1
    Invalid
};
```

### Example

```cpp
// Create ISS-like orbit
domain::space::OrbitalElements orbit;
orbit.semi_major_axis = constants::EARTH_RADIUS + 420000.0;  // 420 km
orbit.eccentricity = 0.0001;
orbit.inclination = 51.6 * DEG_TO_RAD;
orbit.raan = 0.0;
orbit.arg_periapsis = 0.0;
orbit.mean_anomaly = 0.0;

// Get orbital characteristics
std::cout << "Period: " << orbit.period() / 60.0 << " minutes\n";
std::cout << "Type: " << (orbit.classify() == OrbitType::LEO ? "LEO" : "other") << "\n";

// Convert to state vectors
Vec3 position, velocity;
orbit.to_cartesian(position, velocity);

// Convert from state vectors
auto elements = OrbitalElements::from_cartesian(position, velocity);
```

## TLE

Two-Line Element set parsing.

```cpp
struct TLE {
    std::string name;
    int catalog_number{0};
    char classification{'U'};
    std::string intl_designator;
    int epoch_year{0};
    Real epoch_day{0.0};
    Real mean_motion_dot{0.0};      // rev/day²
    Real mean_motion_ddot{0.0};     // rev/day³
    Real bstar{0.0};                // drag term
    Real inclination{0.0};          // rad
    Real raan{0.0};                 // rad
    Real eccentricity{0.0};
    Real arg_perigee{0.0};          // rad
    Real mean_anomaly{0.0};         // rad
    Real mean_motion{0.0};          // rev/day
    int revolution_number{0};

    // Parse TLE lines
    bool parse(const std::string& line1, const std::string& line2);
    bool parse(const std::string& name,
               const std::string& line1, const std::string& line2);

    // Convert to classical elements
    OrbitalElements to_elements() const;

    // Get epoch as Julian date
    Real epoch_jd() const;
};
```

### Example

```cpp
// Parse ISS TLE
std::string line1 = "1 25544U 98067A   21275.52422453  .00001234  00000-0  28888-4 0  9990";
std::string line2 = "2 25544  51.6442  27.4345 0003542 211.7212 148.3510 15.48919914305213";

domain::space::TLE tle;
if (tle.parse("ISS (ZARYA)", line1, line2)) {
    std::cout << "Satellite: " << tle.name << "\n";
    std::cout << "Inclination: " << tle.inclination * RAD_TO_DEG << " deg\n";
    std::cout << "Period: " << 1440.0 / tle.mean_motion << " minutes\n";
}
```

## SGP4Propagator

Simplified General Perturbations propagator.

```cpp
class SGP4Propagator {
public:
    // Initialize from TLE
    bool initialize(const TLE& tle);

    // Propagate to time since epoch
    void propagate(Real minutes_since_epoch,
                   Vec3& position_km,    // TEME frame
                   Vec3& velocity_km_s) const;

    // Propagate to Julian date
    void propagate_to_jd(Real jd,
                         Vec3& position_km,
                         Vec3& velocity_km_s) const;

    // Get epoch
    Real get_epoch_jd() const;

    // Error status
    int get_error_code() const;
    std::string get_error_message() const;
};
```

### Example

```cpp
domain::space::SGP4Propagator sgp4;

// Initialize
if (!sgp4.initialize(tle)) {
    std::cerr << "SGP4 error: " << sgp4.get_error_message() << "\n";
    return;
}

// Propagate for 24 hours
for (Real minutes = 0; minutes < 24 * 60; minutes += 1.0) {
    Vec3 pos_km, vel_kms;
    sgp4.propagate(minutes, pos_km, vel_kms);

    // Convert to SI
    Vec3 pos_m = pos_km * 1000.0;
    Vec3 vel_ms = vel_kms * 1000.0;

    // Convert TEME to ECEF
    Real jd = sgp4.get_epoch_jd() + minutes / 1440.0;
    Vec3 pos_ecef = transforms::teme_to_ecef(pos_m, jd);

    // Get geodetic position
    Real lat, lon, alt;
    transforms::ecef_to_geodetic(pos_ecef, lat, lon, alt);

    std::cout << "Lat: " << lat * RAD_TO_DEG
              << ", Lon: " << lon * RAD_TO_DEG
              << ", Alt: " << alt / 1000.0 << " km\n";
}
```

## GravityModel

Earth gravity with perturbations.

```cpp
enum class GravityFidelity {
    PointMass,  // Simple 1/r² gravity
    J2,         // Include J2 (oblateness)
    J4,         // Include J2 through J4
    Full        // Full geopotential (EGM96)
};

class GravityModel {
public:
    // Configuration
    void set_fidelity(GravityFidelity fidelity);
    void set_fidelity(int degree);  // For full model
    void set_central_body(Real mu, Real radius);

    // Load gravity field
    bool load_egm96();
    bool load_from_file(const std::string& path);

    // Compute acceleration
    Vec3 compute_acceleration(const Vec3& position_ecef) const;

    // IForceGenerator interface
    void compute_forces(const physics::EntityState& state,
                        const environment::Environment& env,
                        Real dt,
                        physics::EntityForces& forces);

    // Get potential
    Real get_potential(const Vec3& position_ecef) const;
};
```

### Example

```cpp
domain::space::GravityModel gravity;

// Simple J2 model
gravity.set_fidelity(GravityFidelity::J2);

// Or higher fidelity
gravity.set_fidelity(GravityFidelity::Full);
gravity.load_egm96();

// Compute acceleration
Vec3 position_eci = /* satellite position */;
Vec3 accel = gravity.compute_acceleration(position_eci);

// Apply to satellite
physics::EntityForces forces;
forces.add_force(accel * satellite_mass);
```

## AtmosphericDragModel

Atmospheric drag for LEO satellites.

```cpp
class AtmosphericDragModel {
public:
    // Configuration
    void set_drag_coefficient(Real cd);  // Typically 2.0-2.5
    void set_area(Real area);            // Cross-section (m²)
    void set_mass(Real mass);            // For A/m ratio

    // Compute forces
    void compute_forces(const physics::EntityState& state,
                        const environment::Environment& env,
                        Real dt,
                        physics::EntityForces& forces);

    // Queries
    Real get_drag_force() const;         // N
    Real get_density() const;            // kg/m³ (last computed)
    Real get_ballistic_coefficient() const;  // kg/m²
};
```

## JB08AtmosphereModel

Jacchia-Bowman 2008 upper atmosphere model.

```cpp
class JB08AtmosphereModel {
public:
    // Space weather inputs
    void set_space_weather(Real f107,      // 10.7 cm flux
                           Real f107_avg,   // 81-day average
                           Real ap);        // Geomagnetic index

    // Load space weather from file
    bool load_space_weather(const std::string& path);

    // Query density
    Real get_density(const Vec3& position_ecef, Real jd) const;

    // Query temperature
    Real get_temperature(const Vec3& position_ecef, Real jd) const;
};
```

### Example

```cpp
domain::space::JB08AtmosphereModel atmosphere;

// Set space weather (moderate activity)
atmosphere.set_space_weather(150.0,   // F10.7
                              145.0,   // F10.7 avg
                              15.0);   // Ap

// Get density at satellite
Real rho = atmosphere.get_density(pos_ecef, jd);

// Configure drag model
domain::space::AtmosphericDragModel drag;
drag.set_drag_coefficient(2.2);
drag.set_area(10.0);  // m²
drag.set_mass(1000.0);  // kg

// Compute drag
physics::EntityForces forces;
drag.compute_forces(state, env, dt, forces);
```

## SolarRadiationPressure

Solar radiation pressure model.

```cpp
class SRPModel {
public:
    // Configuration
    void set_area(Real area);           // m²
    void set_reflectivity(Real cr);     // 0=absorb, 1=reflect
    void set_mass(Real mass);           // kg

    // Shadow calculation
    enum class ShadowModel { None, Cylindrical, Conical };
    void set_shadow_model(ShadowModel model);

    // Compute forces
    void compute_forces(const physics::EntityState& state,
                        const environment::Environment& env,
                        Real dt,
                        physics::EntityForces& forces);

    // Queries
    bool in_shadow() const;
    Real get_shadow_factor() const;  // 0=full shadow, 1=full sun
};
```

## Coordinate Transforms (Space)

```cpp
namespace transforms {
    // TEME <-> ECI (J2000)
    Vec3 teme_to_eci(const Vec3& teme, Real jd);
    Vec3 eci_to_teme(const Vec3& eci, Real jd);

    // TEME <-> ECEF
    Vec3 teme_to_ecef(const Vec3& teme, Real jd);
    Vec3 ecef_to_teme(const Vec3& ecef, Real jd);

    // Time systems
    Real utc_to_tt(Real jd_utc);
    Real tt_to_utc(Real jd_tt);
    Real greenwich_sidereal_time(Real jd_ut1);

    // Julian date
    Real julian_date(int year, int month, int day,
                     int hour, int min, Real sec);
    void calendar_date(Real jd, int& year, int& month, int& day,
                       int& hour, int& min, Real& sec);
}
```

## See Also

- [Physics API](physics.md) - Force generator interfaces
- [Core API](core.md) - Coordinate transforms
- [Space Domain Concepts](../domains/space.md) - Domain overview
- [Space Domain Tutorial](../tutorials/space-domain.md) - Step-by-step guide

