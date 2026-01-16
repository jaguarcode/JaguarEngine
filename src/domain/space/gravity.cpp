/**
 * @file gravity.cpp
 * @brief Gravity model implementation with J2-J4 zonal harmonics
 *
 * Implements gravitational acceleration models from point mass
 * through full J2-J4 perturbations for accurate orbital mechanics.
 */

#include "jaguar/domain/space.h"
#include "jaguar/environment/environment.h"
#include <cmath>

namespace jaguar::domain::space {

// ============================================================================
// Constants
// ============================================================================

namespace {

// WGS-84 Earth constants
constexpr Real MU_EARTH = 3.986004418e14;       // m³/s²
constexpr Real EARTH_RADIUS = 6378137.0;        // m (equatorial radius)
constexpr Real EARTH_FLATTENING = 1.0/298.257223563;

// Zonal harmonic coefficients (normalized)
constexpr Real J2_COEFF = 1.08262668355e-3;     // J2
constexpr Real J3_COEFF = -2.53265648533e-6;    // J3
constexpr Real J4_COEFF = -1.61962159137e-6;    // J4

constexpr Real PI = 3.14159265358979323846;

} // anonymous namespace

// ============================================================================
// GravityModel Implementation
// ============================================================================

GravityModel::GravityModel() = default;
GravityModel::~GravityModel() = default;

void GravityModel::compute_forces(
    const physics::EntityState& state,
    const environment::Environment& /*env*/,
    Real /*dt*/,
    physics::EntityForces& out_forces)
{
    if (!enabled_) return;

    // Position in ECEF (assumed input is ECEF for space entities)
    Vec3 pos = state.position;

    // Compute gravitational acceleration based on fidelity level
    switch (fidelity_) {
        case 0:
            gravity_accel_ = compute_point_mass(pos);
            break;
        case 1:
            gravity_accel_ = compute_j2(pos);
            break;
        case 2:
        default:
            gravity_accel_ = compute_j2_j4(pos);
            break;
        // Level 3 (full EGM96/EGM2008) would require spherical harmonic tables
    }

    // Apply force: F = m * a
    out_forces.force.x += gravity_accel_.x * state.mass;
    out_forces.force.y += gravity_accel_.y * state.mass;
    out_forces.force.z += gravity_accel_.z * state.mass;
}

Vec3 GravityModel::compute_point_mass(const Vec3& pos_ecef) const
{
    // Simple two-body gravitational acceleration
    // a = -μ * r / |r|³

    Real r2 = pos_ecef.x * pos_ecef.x +
              pos_ecef.y * pos_ecef.y +
              pos_ecef.z * pos_ecef.z;
    Real r = std::sqrt(r2);

    if (r < 1.0) {
        return Vec3{0, 0, 0};  // Avoid singularity
    }

    Real r3 = r * r2;
    Real coeff = -MU_EARTH / r3;

    return Vec3{
        coeff * pos_ecef.x,
        coeff * pos_ecef.y,
        coeff * pos_ecef.z
    };
}

Vec3 GravityModel::compute_j2(const Vec3& pos_ecef) const
{
    // J2 gravity model (oblate Earth perturbation)
    //
    // This adds the primary perturbation due to Earth's equatorial bulge.
    // The J2 term is ~1000x larger than higher-order terms.

    Real x = pos_ecef.x;
    Real y = pos_ecef.y;
    Real z = pos_ecef.z;

    Real r2 = x*x + y*y + z*z;
    Real r = std::sqrt(r2);

    if (r < 1.0) {
        return Vec3{0, 0, 0};
    }

    Real r_inv = 1.0 / r;
    Real r2_inv = r_inv * r_inv;
    Real r3_inv = r2_inv * r_inv;

    Real z2 = z * z;
    Real re2 = EARTH_RADIUS * EARTH_RADIUS;

    // Point mass term
    Real pm_coeff = -MU_EARTH * r3_inv;

    // J2 term
    // dU/dr = -μ/r² * [1 - (3/2)J2(Re/r)²(3sin²φ - 1)]
    // where sin²φ = z²/r²
    Real j2_factor = 1.5 * J2_COEFF * re2 * r2_inv;
    Real sin2_phi = z2 * r2_inv;

    // Acceleration components with J2
    // ax = -μx/r³ * [1 + j2_factor * (1 - 5*sin²φ)]
    // ay = -μy/r³ * [1 + j2_factor * (1 - 5*sin²φ)]
    // az = -μz/r³ * [1 + j2_factor * (3 - 5*sin²φ)]

    Real j2_xy = 1.0 + j2_factor * (1.0 - 5.0 * sin2_phi);
    Real j2_z = 1.0 + j2_factor * (3.0 - 5.0 * sin2_phi);

    return Vec3{
        pm_coeff * x * j2_xy,
        pm_coeff * y * j2_xy,
        pm_coeff * z * j2_z
    };
}

Vec3 GravityModel::compute_j2_j4(const Vec3& pos_ecef) const
{
    // Full J2-J4 gravity model
    //
    // Includes:
    // - J2: Primary oblateness (equatorial bulge)
    // - J3: North-South asymmetry (pear shape)
    // - J4: Higher-order oblateness

    Real x = pos_ecef.x;
    Real y = pos_ecef.y;
    Real z = pos_ecef.z;

    Real r2 = x*x + y*y + z*z;
    Real r = std::sqrt(r2);

    if (r < 1.0) {
        return Vec3{0, 0, 0};
    }

    Real r_inv = 1.0 / r;
    Real r2_inv = r_inv * r_inv;
    Real r3_inv = r2_inv * r_inv;
    Real r4_inv = r2_inv * r2_inv;

    Real z2 = z * z;

    Real re = EARTH_RADIUS;
    Real re2 = re * re;
    Real re3 = re2 * re;
    Real re4 = re2 * re2;

    // sin²φ and sin⁴φ where φ is geocentric latitude
    Real sin2_phi = z2 * r2_inv;
    Real sin4_phi = sin2_phi * sin2_phi;

    // Point mass contribution
    Real mu_r3 = MU_EARTH * r3_inv;

    // J2 contribution
    Real j2_coeff = 1.5 * J2_COEFF * re2 * r2_inv;
    Real j2_xy = j2_coeff * (1.0 - 5.0 * sin2_phi);
    Real j2_z = j2_coeff * (3.0 - 5.0 * sin2_phi);

    // J3 contribution (odd harmonic - affects z asymmetrically)
    Real j3_coeff = 0.5 * J3_COEFF * re3 * r3_inv;
    Real j3_xy = j3_coeff * z * r_inv * (10.0 - 35.0 * sin2_phi / 3.0);
    Real j3_z_term = j3_coeff * (3.0 - 30.0 * sin2_phi + 35.0 * sin4_phi / 3.0);

    // J4 contribution
    Real j4_coeff = -0.625 * J4_COEFF * re4 * r4_inv;
    Real j4_xy = j4_coeff * (3.0 - 42.0 * sin2_phi + 63.0 * sin4_phi);
    Real j4_z = j4_coeff * (15.0 - 70.0 * sin2_phi + 63.0 * sin4_phi);

    // Combine all terms
    Real ax = -mu_r3 * x * (1.0 + j2_xy + j3_xy + j4_xy);
    Real ay = -mu_r3 * y * (1.0 + j2_xy + j3_xy + j4_xy);
    Real az = -mu_r3 * z * (1.0 + j2_z) - mu_r3 * j3_z_term - mu_r3 * z * j4_z;

    return Vec3{ax, ay, az};
}

// ============================================================================
// Coordinate Transforms Implementation
// ============================================================================

namespace transforms {

Vec3 ecef_to_eci(const Vec3& ecef, Real time)
{
    // Convert ECEF to ECI using Greenwich Sidereal Time rotation
    // ECI = Rz(-GMST) * ECEF

    Real gmst_rad = gmst(time);
    Real cos_gmst = std::cos(gmst_rad);
    Real sin_gmst = std::sin(gmst_rad);

    return Vec3{
        cos_gmst * ecef.x - sin_gmst * ecef.y,
        sin_gmst * ecef.x + cos_gmst * ecef.y,
        ecef.z
    };
}

Vec3 eci_to_ecef(const Vec3& eci, Real time)
{
    // Convert ECI to ECEF using Greenwich Sidereal Time rotation
    // ECEF = Rz(GMST) * ECI

    Real gmst_rad = gmst(time);
    Real cos_gmst = std::cos(gmst_rad);
    Real sin_gmst = std::sin(gmst_rad);

    return Vec3{
        cos_gmst * eci.x + sin_gmst * eci.y,
        -sin_gmst * eci.x + cos_gmst * eci.y,
        eci.z
    };
}

Vec3 geodetic_to_ecef(Real lat_rad, Real lon_rad, Real alt_m)
{
    // WGS-84 geodetic to ECEF conversion
    Real sin_lat = std::sin(lat_rad);
    Real cos_lat = std::cos(lat_rad);
    Real sin_lon = std::sin(lon_rad);
    Real cos_lon = std::cos(lon_rad);

    // Prime vertical radius of curvature
    Real e2 = 2.0 * EARTH_FLATTENING - EARTH_FLATTENING * EARTH_FLATTENING;
    Real N = EARTH_RADIUS / std::sqrt(1.0 - e2 * sin_lat * sin_lat);

    return Vec3{
        (N + alt_m) * cos_lat * cos_lon,
        (N + alt_m) * cos_lat * sin_lon,
        (N * (1.0 - e2) + alt_m) * sin_lat
    };
}

void ecef_to_geodetic(const Vec3& ecef, Real& lat_rad, Real& lon_rad, Real& alt_m)
{
    // Iterative ECEF to geodetic conversion (Bowring's method)
    Real x = ecef.x;
    Real y = ecef.y;
    Real z = ecef.z;

    // Longitude is straightforward
    lon_rad = std::atan2(y, x);

    // Distance from Z-axis
    Real p = std::sqrt(x*x + y*y);

    // First approximation of latitude
    Real e2 = 2.0 * EARTH_FLATTENING - EARTH_FLATTENING * EARTH_FLATTENING;
    Real lat = std::atan2(z, p * (1.0 - e2));

    // Iterate to convergence
    for (int i = 0; i < 10; ++i) {
        Real sin_lat = std::sin(lat);
        Real N = EARTH_RADIUS / std::sqrt(1.0 - e2 * sin_lat * sin_lat);
        Real lat_new = std::atan2(z + e2 * N * sin_lat, p);

        if (std::abs(lat_new - lat) < 1e-12) {
            break;
        }
        lat = lat_new;
    }

    lat_rad = lat;

    // Altitude
    Real sin_lat = std::sin(lat);
    Real cos_lat = std::cos(lat);
    Real N = EARTH_RADIUS / std::sqrt(1.0 - e2 * sin_lat * sin_lat);

    if (std::abs(cos_lat) > 1e-10) {
        alt_m = p / cos_lat - N;
    } else {
        alt_m = std::abs(z) / std::abs(sin_lat) - N * (1.0 - e2);
    }
}

Real gmst(Real julian_date)
{
    // Greenwich Mean Sidereal Time from Julian Date
    //
    // Reference: IAU SOFA (Standards of Fundamental Astronomy)

    // Julian centuries from J2000.0
    Real T = (julian_date - 2451545.0) / 36525.0;

    // GMST in seconds at 0h UT1
    Real gmst_sec = 67310.54841 +
                    (876600.0 * 3600.0 + 8640184.812866) * T +
                    0.093104 * T * T -
                    6.2e-6 * T * T * T;

    // Convert to radians and normalize to [0, 2π]
    Real gmst_rad = std::fmod(gmst_sec, 86400.0) * PI / 43200.0;
    if (gmst_rad < 0.0) {
        gmst_rad += 2.0 * PI;
    }

    return gmst_rad;
}

// ============================================================================
// LVLH (Local Vertical Local Horizontal) Frame Transforms
// ============================================================================

/**
 * LVLH Frame Construction:
 *
 * Given spacecraft position r and velocity v in ECI:
 *
 *   R_hat = r / |r|                    (radial, points outward from Earth)
 *   W_hat = (r × v) / |r × v|          (cross-track, normal to orbital plane)
 *   S_hat = W_hat × R_hat              (along-track, in orbital plane, ~velocity direction)
 *
 * The rotation matrix from ECI to LVLH has these unit vectors as rows:
 *   R_eci_to_lvlh = [R_hat; S_hat; W_hat]
 *
 * Reference: Vallado, "Fundamentals of Astrodynamics and Applications"
 */

namespace {

// Helper to compute vector magnitude
inline Real vec_magnitude(const Vec3& v) {
    return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

// Helper for cross product
inline Vec3 vec_cross(const Vec3& a, const Vec3& b) {
    return Vec3{
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    };
}

// Helper to normalize vector
inline Vec3 vec_normalize(const Vec3& v) {
    Real mag = vec_magnitude(v);
    if (mag < 1e-12) {
        return Vec3{0.0, 0.0, 0.0};
    }
    return Vec3{v.x / mag, v.y / mag, v.z / mag};
}

} // anonymous namespace

Mat3x3 eci_to_lvlh_matrix(const Vec3& pos_eci, const Vec3& vel_eci)
{
    // Compute LVLH basis vectors in ECI coordinates

    // R_hat: radial direction (outward from Earth center)
    Vec3 R_hat = vec_normalize(pos_eci);

    // h = r × v: angular momentum vector (normal to orbital plane)
    Vec3 h = vec_cross(pos_eci, vel_eci);

    // W_hat: cross-track direction (normal to orbital plane)
    Vec3 W_hat = vec_normalize(h);

    // S_hat: along-track direction (in orbital plane, roughly velocity direction)
    // S_hat = W_hat × R_hat
    Vec3 S_hat = vec_cross(W_hat, R_hat);

    // Build rotation matrix: rows are the LVLH basis vectors expressed in ECI
    Mat3x3 R;
    R(0, 0) = R_hat.x;  R(0, 1) = R_hat.y;  R(0, 2) = R_hat.z;
    R(1, 0) = S_hat.x;  R(1, 1) = S_hat.y;  R(1, 2) = S_hat.z;
    R(2, 0) = W_hat.x;  R(2, 1) = W_hat.y;  R(2, 2) = W_hat.z;

    return R;
}

Mat3x3 lvlh_to_eci_matrix(const Vec3& pos_eci, const Vec3& vel_eci)
{
    // LVLH to ECI is the transpose of ECI to LVLH (rotation matrices are orthogonal)
    return eci_to_lvlh_matrix(pos_eci, vel_eci).transpose();
}

Vec3 eci_to_lvlh(const Vec3& vec_eci, const Vec3& ref_pos_eci, const Vec3& ref_vel_eci)
{
    Mat3x3 R = eci_to_lvlh_matrix(ref_pos_eci, ref_vel_eci);
    return R * vec_eci;
}

Vec3 lvlh_to_eci(const Vec3& vec_lvlh, const Vec3& ref_pos_eci, const Vec3& ref_vel_eci)
{
    Mat3x3 R = lvlh_to_eci_matrix(ref_pos_eci, ref_vel_eci);
    return R * vec_lvlh;
}

Vec3 relative_position_lvlh(const Vec3& target_pos_eci,
                             const Vec3& ref_pos_eci,
                             const Vec3& ref_vel_eci)
{
    // Compute relative position in ECI
    Vec3 rel_pos_eci{
        target_pos_eci.x - ref_pos_eci.x,
        target_pos_eci.y - ref_pos_eci.y,
        target_pos_eci.z - ref_pos_eci.z
    };

    // Transform to LVLH frame
    return eci_to_lvlh(rel_pos_eci, ref_pos_eci, ref_vel_eci);
}

Vec3 relative_velocity_lvlh(const Vec3& target_pos_eci,
                             const Vec3& target_vel_eci,
                             const Vec3& ref_pos_eci,
                             const Vec3& ref_vel_eci)
{
    // Compute relative velocity in ECI
    Vec3 rel_vel_eci{
        target_vel_eci.x - ref_vel_eci.x,
        target_vel_eci.y - ref_vel_eci.y,
        target_vel_eci.z - ref_vel_eci.z
    };

    // Get LVLH frame angular velocity
    Vec3 omega_lvlh = lvlh_angular_velocity(ref_pos_eci, ref_vel_eci);

    // Get relative position in LVLH
    Vec3 rel_pos_lvlh = relative_position_lvlh(target_pos_eci, ref_pos_eci, ref_vel_eci);

    // Transform relative velocity to LVLH
    Vec3 rel_vel_lvlh = eci_to_lvlh(rel_vel_eci, ref_pos_eci, ref_vel_eci);

    // Account for rotation of LVLH frame: v_rel_lvlh = v_rel_eci_in_lvlh - omega × r_rel_lvlh
    Vec3 omega_cross_r = vec_cross(omega_lvlh, rel_pos_lvlh);

    return Vec3{
        rel_vel_lvlh.x - omega_cross_r.x,
        rel_vel_lvlh.y - omega_cross_r.y,
        rel_vel_lvlh.z - omega_cross_r.z
    };
}

Vec3 lvlh_angular_velocity(const Vec3& pos_eci, const Vec3& vel_eci)
{
    // Angular velocity of LVLH frame relative to ECI
    //
    // For a Keplerian orbit, the LVLH frame rotates at the orbital rate.
    // ω = h / r² in the W (cross-track) direction
    //
    // More precisely, in LVLH coordinates:
    //   ω_lvlh = [0, 0, |h| / r²]
    //
    // where h = r × v is the specific angular momentum

    Real r = vec_magnitude(pos_eci);
    if (r < 1.0) {
        return Vec3{0.0, 0.0, 0.0};  // Avoid singularity
    }

    // Specific angular momentum magnitude
    Vec3 h = vec_cross(pos_eci, vel_eci);
    Real h_mag = vec_magnitude(h);

    // Angular velocity magnitude: n = h / r²
    Real omega = h_mag / (r * r);

    // In LVLH coordinates, ω is purely in the W direction
    // (The frame rotates about the orbit normal)
    return Vec3{0.0, 0.0, omega};
}

} // namespace transforms

// ============================================================================
// JB08 Atmospheric Model Implementation
// ============================================================================

JB08AtmosphereModel::JB08AtmosphereModel() = default;
JB08AtmosphereModel::~JB08AtmosphereModel() = default;

Real JB08AtmosphereModel::get_density(Real altitude, Real /*latitude*/,
                                       Real /*longitude*/, Real /*time*/) const
{
    // Simplified JB08 model - exponential atmosphere with space weather modulation
    //
    // Full JB08 would include:
    // - Solar EUV heating
    // - Geomagnetic heating
    // - Semiannual density variation
    // - Local time variation
    // - Latitude variation

    if (altitude < 100000.0) {
        // Below 100 km, use US Standard Atmosphere
        return 0.0;  // Not in space domain
    }

    if (altitude > 1000000.0) {
        // Above 1000 km, negligible density
        return 1e-18;
    }

    // Base density model (exponential atmosphere)
    // Reference density at 400 km altitude
    constexpr Real rho_400 = 3.0e-12;  // kg/m³
    constexpr Real H_400 = 58000.0;     // Scale height at 400 km (m)

    Real h = altitude - 400000.0;  // Height above 400 km

    // Scale height varies with altitude
    Real H = H_400 * (1.0 + 0.002 * (altitude - 400000.0) / 100000.0);

    // Base density
    Real rho_base = rho_400 * std::exp(-h / H);

    // Solar activity modulation
    // F10.7 of 70 is solar minimum, 250 is solar maximum
    Real f107_factor = 1.0 + 0.3 * (f107_ - 150.0) / 100.0;

    // Geomagnetic activity modulation
    Real ap_factor = 1.0 + 0.05 * (ap_ - 15.0);

    return rho_base * f107_factor * ap_factor;
}

void JB08AtmosphereModel::set_space_weather(Real f107, Real f107_avg, Real ap)
{
    f107_ = f107;
    f107_avg_ = f107_avg;
    ap_ = ap;
}

// ============================================================================
// Atmospheric Drag Model Implementation
// ============================================================================

AtmosphericDragModel::AtmosphericDragModel() = default;
AtmosphericDragModel::~AtmosphericDragModel() = default;

void AtmosphericDragModel::compute_forces(
    const physics::EntityState& state,
    const environment::Environment& /*env*/,
    Real /*dt*/,
    physics::EntityForces& out_forces)
{
    if (!enabled_) return;

    // Get altitude from ECEF position
    Real lat, lon, alt;
    transforms::ecef_to_geodetic(state.position, lat, lon, alt);

    // Get atmospheric density
    Real rho = atmosphere_.get_density(alt, lat, lon, 0.0);

    if (rho < 1e-20) {
        return;  // Negligible density
    }

    // Velocity relative to atmosphere (assumes rotating atmosphere)
    // For high accuracy, would need to account for atmospheric rotation
    Vec3 vel = state.velocity;
    Real v = std::sqrt(vel.x * vel.x + vel.y * vel.y + vel.z * vel.z);

    if (v < 1.0) {
        return;  // Negligible velocity
    }

    // Drag force: F_drag = -0.5 * ρ * Cd * A * v² * (v_hat)
    Real drag_mag = 0.5 * rho * cd_ * area_ * v * v;

    // Apply in opposite direction to velocity
    Real v_inv = 1.0 / v;
    out_forces.force.x -= drag_mag * vel.x * v_inv;
    out_forces.force.y -= drag_mag * vel.y * v_inv;
    out_forces.force.z -= drag_mag * vel.z * v_inv;
}

} // namespace jaguar::domain::space
