#pragma once
/**
 * @file space.h
 * @brief Space domain physics models (orbital mechanics, gravity)
 */

#include "jaguar/core/types.h"
#include "jaguar/physics/force.h"

namespace jaguar::domain::space {

// ============================================================================
// Orbital Elements
// ============================================================================

/**
 * @brief Classical Keplerian orbital elements
 */
struct OrbitalElements {
    Real semi_major_axis{7000000.0};  ///< a (m)
    Real eccentricity{0.0};           ///< e
    Real inclination{0.0};            ///< i (rad)
    Real raan{0.0};                   ///< Right Ascension of Ascending Node (rad)
    Real arg_of_perigee{0.0};         ///< Argument of perigee (rad)
    Real true_anomaly{0.0};           ///< True anomaly (rad)

    /**
     * @brief Convert to ECI position and velocity
     */
    void to_state_vector(Vec3& pos_eci, Vec3& vel_eci) const;

    /**
     * @brief Convert from ECI position and velocity
     */
    static OrbitalElements from_state_vector(const Vec3& pos_eci, const Vec3& vel_eci);

    /**
     * @brief Calculate orbital period (seconds)
     */
    Real period() const;

    /**
     * @brief Calculate orbital altitude at current position (m)
     */
    Real altitude() const;
};

/**
 * @brief Two-Line Element (TLE) data
 */
struct TLE {
    std::string name;
    std::string line1;
    std::string line2;

    // Parsed values
    int satellite_number{0};
    Real epoch_year{0.0};
    Real epoch_day{0.0};
    Real bstar{0.0};           ///< Drag term
    Real inclination{0.0};     ///< rad
    Real raan{0.0};            ///< rad
    Real eccentricity{0.0};
    Real arg_of_perigee{0.0};  ///< rad
    Real mean_anomaly{0.0};    ///< rad
    Real mean_motion{0.0};     ///< rev/day

    /**
     * @brief Parse TLE from two lines
     */
    static TLE parse(const std::string& line1, const std::string& line2);
};

// ============================================================================
// SGP4 Propagator
// ============================================================================

/**
 * @brief SGP4/SDP4 orbital propagator
 */
class SGP4Propagator {
public:
    SGP4Propagator();
    ~SGP4Propagator();

    /**
     * @brief Initialize from TLE
     */
    bool initialize(const TLE& tle);

    /**
     * @brief Propagate to specified time
     * @param minutes_since_epoch Minutes from TLE epoch
     * @param pos_eci Output position in ECI (km)
     * @param vel_eci Output velocity in ECI (km/s)
     * @return true if propagation successful
     */
    bool propagate(Real minutes_since_epoch, Vec3& pos_eci, Vec3& vel_eci);

    /**
     * @brief Get orbital elements at current time
     */
    OrbitalElements get_elements() const;

    /**
     * @brief Check if using deep space (SDP4) mode
     */
    bool is_deep_space() const { return deep_space_; }

private:
    struct SGP4Data;
    std::unique_ptr<SGP4Data> data_;
    bool deep_space_{false};
    bool initialized_{false};
};

// ============================================================================
// Gravity Model
// ============================================================================

/**
 * @brief Gravitational force model
 */
class GravityModel : public physics::IGravityModel {
public:
    GravityModel();
    ~GravityModel() override;

    void compute_forces(
        const physics::EntityState& state,
        const environment::Environment& env,
        Real dt,
        physics::EntityForces& out_forces) override;

    const std::string& name() const override { return name_; }

    Vec3 get_gravity_acceleration() const override { return gravity_accel_; }
    void set_fidelity(int level) override { fidelity_ = level; }

    /**
     * @brief Fidelity levels:
     * 0 = Point mass (μ/r²)
     * 1 = J2 perturbation
     * 2 = J2-J4 perturbations
     * 3 = Full EGM96/EGM2008
     */
    int get_fidelity() const { return fidelity_; }

private:
    std::string name_{"Gravity"};
    int fidelity_{1};
    Vec3 gravity_accel_{0, 0, 0};

    // Zonal harmonics
    static constexpr Real J2 = 1.08263e-3;
    static constexpr Real J3 = -2.532e-6;
    static constexpr Real J4 = -1.6109e-6;

    Vec3 compute_point_mass(const Vec3& pos_ecef) const;
    Vec3 compute_j2(const Vec3& pos_ecef) const;
    Vec3 compute_j2_j4(const Vec3& pos_ecef) const;
};

// ============================================================================
// Atmospheric Drag (Space)
// ============================================================================

/**
 * @brief Jacchia-Bowman 2008 atmospheric density model
 */
class JB08AtmosphereModel {
public:
    JB08AtmosphereModel();
    ~JB08AtmosphereModel();

    /**
     * @brief Get atmospheric density at altitude
     * @param altitude Geodetic altitude (m)
     * @param latitude Geodetic latitude (rad)
     * @param longitude Longitude (rad)
     * @param time Simulation time
     * @return Density (kg/m³)
     */
    Real get_density(Real altitude, Real latitude, Real longitude, Real time) const;

    /**
     * @brief Set space weather indices
     * @param f107 10.7 cm solar flux
     * @param f107_avg 81-day average F10.7
     * @param ap Geomagnetic index
     */
    void set_space_weather(Real f107, Real f107_avg, Real ap);

private:
    Real f107_{150.0};
    Real f107_avg_{150.0};
    Real ap_{15.0};
};

/**
 * @brief Atmospheric drag force for satellites
 */
class AtmosphericDragModel : public physics::IForceGenerator {
public:
    AtmosphericDragModel();
    ~AtmosphericDragModel() override;

    void compute_forces(
        const physics::EntityState& state,
        const environment::Environment& env,
        Real dt,
        physics::EntityForces& out_forces) override;

    const std::string& name() const override { return name_; }
    Domain domain() const override { return Domain::Space; }

    /**
     * @brief Set ballistic coefficient (m²/kg)
     */
    void set_ballistic_coefficient(Real bc) { ballistic_coeff_ = bc; }

    /**
     * @brief Set cross-sectional area (m²)
     */
    void set_area(Real area) { area_ = area; }

    /**
     * @brief Set drag coefficient
     */
    void set_cd(Real cd) { cd_ = cd; }

private:
    std::string name_{"AtmosphericDrag"};
    JB08AtmosphereModel atmosphere_;

    Real ballistic_coeff_{0.01};  // m²/kg
    Real area_{1.0};              // m²
    Real cd_{2.2};                // Typical satellite Cd
};

// ============================================================================
// Coordinate Transformations
// ============================================================================

namespace transforms {

/**
 * @brief Convert ECEF to ECI
 */
Vec3 ecef_to_eci(const Vec3& ecef, Real time);

/**
 * @brief Convert ECI to ECEF
 */
Vec3 eci_to_ecef(const Vec3& eci, Real time);

/**
 * @brief Convert geodetic (lat, lon, alt) to ECEF
 */
Vec3 geodetic_to_ecef(Real lat_rad, Real lon_rad, Real alt_m);

/**
 * @brief Convert ECEF to geodetic
 */
void ecef_to_geodetic(const Vec3& ecef, Real& lat_rad, Real& lon_rad, Real& alt_m);

/**
 * @brief Get Greenwich Mean Sidereal Time
 */
Real gmst(Real julian_date);

// ============================================================================
// LVLH (Local Vertical Local Horizontal) Frame Transforms
// ============================================================================

/**
 * @brief LVLH frame definition (also called Hill frame or RSW frame)
 *
 * LVLH is a local orbital frame centered on a spacecraft:
 *   - R (Radial): Points from Earth center through spacecraft (outward)
 *   - S (Along-track): In orbital plane, perpendicular to R, in direction of motion
 *   - W (Cross-track): Completes right-handed system (normal to orbital plane)
 *
 * Alternative naming conventions:
 *   - RSW: Radial, Along-track (S), Cross-track (W)
 *   - RIC: Radial, In-track, Cross-track
 *   - VNC: Velocity, Normal, Co-normal (different orientation)
 */

/**
 * @brief Get rotation matrix from ECI to LVLH frame
 *
 * @param pos_eci Spacecraft position in ECI (m)
 * @param vel_eci Spacecraft velocity in ECI (m/s)
 * @return 3x3 rotation matrix (ECI to LVLH)
 */
Mat3x3 eci_to_lvlh_matrix(const Vec3& pos_eci, const Vec3& vel_eci);

/**
 * @brief Get rotation matrix from LVLH to ECI frame
 *
 * @param pos_eci Spacecraft position in ECI (m)
 * @param vel_eci Spacecraft velocity in ECI (m/s)
 * @return 3x3 rotation matrix (LVLH to ECI)
 */
Mat3x3 lvlh_to_eci_matrix(const Vec3& pos_eci, const Vec3& vel_eci);

/**
 * @brief Transform position vector from ECI to LVLH frame
 *
 * @param vec_eci Vector in ECI frame
 * @param ref_pos_eci Reference spacecraft position in ECI
 * @param ref_vel_eci Reference spacecraft velocity in ECI
 * @return Vector in LVLH frame
 */
Vec3 eci_to_lvlh(const Vec3& vec_eci, const Vec3& ref_pos_eci, const Vec3& ref_vel_eci);

/**
 * @brief Transform position vector from LVLH to ECI frame
 *
 * @param vec_lvlh Vector in LVLH frame
 * @param ref_pos_eci Reference spacecraft position in ECI
 * @param ref_vel_eci Reference spacecraft velocity in ECI
 * @return Vector in ECI frame
 */
Vec3 lvlh_to_eci(const Vec3& vec_lvlh, const Vec3& ref_pos_eci, const Vec3& ref_vel_eci);

/**
 * @brief Calculate relative position in LVLH frame
 *
 * Given target and reference spacecraft states, compute relative position
 * in the reference spacecraft's LVLH frame.
 *
 * @param target_pos_eci Target spacecraft position in ECI
 * @param ref_pos_eci Reference spacecraft position in ECI
 * @param ref_vel_eci Reference spacecraft velocity in ECI
 * @return Relative position in LVLH frame (R, S, W components)
 */
Vec3 relative_position_lvlh(const Vec3& target_pos_eci,
                             const Vec3& ref_pos_eci,
                             const Vec3& ref_vel_eci);

/**
 * @brief Calculate relative velocity in LVLH frame
 *
 * @param target_pos_eci Target spacecraft position in ECI
 * @param target_vel_eci Target spacecraft velocity in ECI
 * @param ref_pos_eci Reference spacecraft position in ECI
 * @param ref_vel_eci Reference spacecraft velocity in ECI
 * @return Relative velocity in LVLH frame
 */
Vec3 relative_velocity_lvlh(const Vec3& target_pos_eci,
                             const Vec3& target_vel_eci,
                             const Vec3& ref_pos_eci,
                             const Vec3& ref_vel_eci);

/**
 * @brief Get angular velocity of LVLH frame relative to ECI
 *
 * The LVLH frame rotates as the spacecraft orbits. This returns
 * the angular velocity vector (in LVLH coordinates) representing
 * this rotation.
 *
 * @param pos_eci Spacecraft position in ECI
 * @param vel_eci Spacecraft velocity in ECI
 * @return Angular velocity of LVLH frame (rad/s)
 */
Vec3 lvlh_angular_velocity(const Vec3& pos_eci, const Vec3& vel_eci);

} // namespace transforms

} // namespace jaguar::domain::space
