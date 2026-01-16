#pragma once
/**
 * @file coordinates.h
 * @brief Coordinate transformation utilities
 *
 * Provides conversions between different coordinate reference frames:
 * - ECEF (Earth-Centered, Earth-Fixed)
 * - LLA (Latitude, Longitude, Altitude - geodetic)
 * - NED (North-East-Down) local tangent plane
 * - ENU (East-North-Up) local tangent plane
 * - Body frame transformations
 *
 * All angles are in radians unless otherwise specified.
 * WGS84 ellipsoid model is used for geodetic calculations.
 */

#include "jaguar/core/types.h"

namespace jaguar {

// ============================================================================
// WGS84 Ellipsoid Parameters
// ============================================================================

namespace wgs84 {
    /// Semi-major axis (equatorial radius) in meters
    constexpr Real a = 6378137.0;

    /// Semi-minor axis (polar radius) in meters
    constexpr Real b = 6356752.314245;

    /// Flattening
    constexpr Real f = 1.0 / 298.257223563;

    /// First eccentricity squared
    constexpr Real e2 = 2.0 * f - f * f;  // ≈ 0.00669437999014

    /// Second eccentricity squared
    constexpr Real ep2 = (a * a - b * b) / (b * b);
}

// ============================================================================
// Geodetic Position (Latitude, Longitude, Altitude)
// ============================================================================

/**
 * @brief Geodetic position in WGS84 coordinates
 */
struct GeodeticPosition {
    Real latitude{0.0};   ///< Geodetic latitude (radians, -PI/2 to PI/2)
    Real longitude{0.0};  ///< Longitude (radians, -PI to PI)
    Real altitude{0.0};   ///< Height above WGS84 ellipsoid (meters)

    constexpr GeodeticPosition() noexcept = default;
    constexpr GeodeticPosition(Real lat, Real lon, Real alt) noexcept
        : latitude(lat), longitude(lon), altitude(alt) {}

    /// Create from degrees
    static GeodeticPosition from_degrees(Real lat_deg, Real lon_deg, Real alt_m) noexcept;

    /// Convert to degrees
    void to_degrees(Real& lat_deg, Real& lon_deg, Real& alt_m) const noexcept;
};

// ============================================================================
// Coordinate Transformation Functions
// ============================================================================

namespace coord {

// ----------------------------------------------------------------------------
// ECEF <-> Geodetic (LLA) Conversions
// ----------------------------------------------------------------------------

/**
 * @brief Convert geodetic coordinates to ECEF
 *
 * @param lla Geodetic position (lat, lon, alt in radians/meters)
 * @return Position in ECEF coordinates (meters)
 */
Vec3 lla_to_ecef(const GeodeticPosition& lla) noexcept;

/**
 * @brief Convert ECEF coordinates to geodetic
 *
 * Uses Bowring's iterative method for accuracy.
 *
 * @param ecef Position in ECEF coordinates (meters)
 * @return Geodetic position (lat, lon, alt)
 */
GeodeticPosition ecef_to_lla(const Vec3& ecef) noexcept;

// ----------------------------------------------------------------------------
// ECEF <-> NED Conversions
// ----------------------------------------------------------------------------

/**
 * @brief Get rotation matrix from ECEF to NED frame at given position
 *
 * @param lla Reference geodetic position for NED origin
 * @return 3x3 rotation matrix (ECEF to NED)
 */
Mat3x3 ecef_to_ned_matrix(const GeodeticPosition& lla) noexcept;

/**
 * @brief Get rotation matrix from NED to ECEF frame at given position
 *
 * @param lla Reference geodetic position for NED origin
 * @return 3x3 rotation matrix (NED to ECEF)
 */
Mat3x3 ned_to_ecef_matrix(const GeodeticPosition& lla) noexcept;

/**
 * @brief Transform vector from ECEF to NED frame
 *
 * @param v Vector in ECEF frame
 * @param lla Reference position for NED frame
 * @return Vector in NED frame
 */
Vec3 ecef_to_ned(const Vec3& v, const GeodeticPosition& lla) noexcept;

/**
 * @brief Transform vector from NED to ECEF frame
 *
 * @param v Vector in NED frame
 * @param lla Reference position for NED frame
 * @return Vector in ECEF frame
 */
Vec3 ned_to_ecef(const Vec3& v, const GeodeticPosition& lla) noexcept;

// ----------------------------------------------------------------------------
// ECEF <-> ENU Conversions
// ----------------------------------------------------------------------------

/**
 * @brief Get rotation matrix from ECEF to ENU frame at given position
 *
 * @param lla Reference geodetic position for ENU origin
 * @return 3x3 rotation matrix (ECEF to ENU)
 */
Mat3x3 ecef_to_enu_matrix(const GeodeticPosition& lla) noexcept;

/**
 * @brief Get rotation matrix from ENU to ECEF frame at given position
 *
 * @param lla Reference geodetic position for ENU origin
 * @return 3x3 rotation matrix (ENU to ECEF)
 */
Mat3x3 enu_to_ecef_matrix(const GeodeticPosition& lla) noexcept;

/**
 * @brief Transform vector from ECEF to ENU frame
 *
 * @param v Vector in ECEF frame
 * @param lla Reference position for ENU frame
 * @return Vector in ENU frame
 */
Vec3 ecef_to_enu(const Vec3& v, const GeodeticPosition& lla) noexcept;

/**
 * @brief Transform vector from ENU to ECEF frame
 *
 * @param v Vector in ENU frame
 * @param lla Reference position for ENU frame
 * @return Vector in ECEF frame
 */
Vec3 enu_to_ecef(const Vec3& v, const GeodeticPosition& lla) noexcept;

// ----------------------------------------------------------------------------
// NED <-> ENU Conversions
// ----------------------------------------------------------------------------

/**
 * @brief Convert NED vector to ENU
 *
 * @param ned Vector in NED frame (North, East, Down)
 * @return Vector in ENU frame (East, North, Up)
 */
constexpr Vec3 ned_to_enu(const Vec3& ned) noexcept {
    return {ned.y, ned.x, -ned.z};
}

/**
 * @brief Convert ENU vector to NED
 *
 * @param enu Vector in ENU frame (East, North, Up)
 * @return Vector in NED frame (North, East, Down)
 */
constexpr Vec3 enu_to_ned(const Vec3& enu) noexcept {
    return {enu.y, enu.x, -enu.z};
}

// ----------------------------------------------------------------------------
// Body Frame Conversions
// ----------------------------------------------------------------------------

/**
 * @brief Transform vector from NED to body frame
 *
 * @param v Vector in NED frame
 * @param orientation Body orientation quaternion (body to NED)
 * @return Vector in body frame
 */
Vec3 ned_to_body(const Vec3& v, const Quat& orientation) noexcept;

/**
 * @brief Transform vector from body to NED frame
 *
 * @param v Vector in body frame
 * @param orientation Body orientation quaternion (body to NED)
 * @return Vector in NED frame
 */
Vec3 body_to_ned(const Vec3& v, const Quat& orientation) noexcept;

// ----------------------------------------------------------------------------
// Geodetic Utilities
// ----------------------------------------------------------------------------

/**
 * @brief Calculate the radius of curvature in the prime vertical
 *
 * @param lat Geodetic latitude (radians)
 * @return Radius of curvature N (meters)
 */
Real radius_of_curvature_n(Real lat) noexcept;

/**
 * @brief Calculate the radius of curvature in the meridian
 *
 * @param lat Geodetic latitude (radians)
 * @return Radius of curvature M (meters)
 */
Real radius_of_curvature_m(Real lat) noexcept;

/**
 * @brief Calculate great circle distance between two geodetic positions
 *
 * Uses the Haversine formula.
 *
 * @param lla1 First geodetic position
 * @param lla2 Second geodetic position
 * @return Great circle distance (meters)
 */
Real great_circle_distance(const GeodeticPosition& lla1, const GeodeticPosition& lla2) noexcept;

/**
 * @brief Calculate initial bearing between two geodetic positions
 *
 * @param lla1 Starting geodetic position
 * @param lla2 Ending geodetic position
 * @return Initial bearing (radians, clockwise from north)
 */
Real initial_bearing(const GeodeticPosition& lla1, const GeodeticPosition& lla2) noexcept;

/**
 * @brief Calculate destination point given start, bearing, and distance
 *
 * @param start Starting geodetic position
 * @param bearing Initial bearing (radians, clockwise from north)
 * @param distance Distance to travel (meters)
 * @return Destination geodetic position
 */
GeodeticPosition destination_point(const GeodeticPosition& start, Real bearing, Real distance) noexcept;

// ----------------------------------------------------------------------------
// Quaternion Frame Conversions
// ----------------------------------------------------------------------------

/**
 * @brief Convert NED orientation quaternion to ECEF orientation quaternion
 *
 * @param q_ned Orientation quaternion in NED frame (body to NED)
 * @param lla Reference geodetic position
 * @return Orientation quaternion in ECEF frame (body to ECEF)
 */
Quat ned_quat_to_ecef_quat(const Quat& q_ned, const GeodeticPosition& lla) noexcept;

/**
 * @brief Convert ECEF orientation quaternion to NED orientation quaternion
 *
 * @param q_ecef Orientation quaternion in ECEF frame (body to ECEF)
 * @param lla Reference geodetic position
 * @return Orientation quaternion in NED frame (body to NED)
 */
Quat ecef_quat_to_ned_quat(const Quat& q_ecef, const GeodeticPosition& lla) noexcept;

/**
 * @brief Get quaternion representing rotation from ECEF to NED
 *
 * @param lla Reference geodetic position
 * @return Quaternion (ECEF to NED rotation)
 */
Quat ecef_to_ned_quat(const GeodeticPosition& lla) noexcept;

} // namespace coord

// ============================================================================
// Math Utilities for Coordinate Transforms
// ============================================================================

namespace math {

/**
 * @brief Normalize angle to range [-PI, PI]
 */
Real normalize_angle(Real angle) noexcept;

/**
 * @brief Normalize angle to range [0, 2*PI]
 */
Real normalize_angle_positive(Real angle) noexcept;

/**
 * @brief Convert degrees to radians
 */
constexpr Real deg_to_rad(Real deg) noexcept {
    return deg * constants::DEG_TO_RAD;
}

/**
 * @brief Convert radians to degrees
 */
constexpr Real rad_to_deg(Real rad) noexcept {
    return rad * constants::RAD_TO_DEG;
}

} // namespace math

} // namespace jaguar
