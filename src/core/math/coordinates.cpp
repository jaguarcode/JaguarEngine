/**
 * @file coordinates.cpp
 * @brief Coordinate transformation implementations
 */

#include "jaguar/core/coordinates.h"
#include <cmath>

namespace jaguar {

// ============================================================================
// GeodeticPosition Implementation
// ============================================================================

GeodeticPosition GeodeticPosition::from_degrees(Real lat_deg, Real lon_deg, Real alt_m) noexcept {
    return {
        lat_deg * constants::DEG_TO_RAD,
        lon_deg * constants::DEG_TO_RAD,
        alt_m
    };
}

void GeodeticPosition::to_degrees(Real& lat_deg, Real& lon_deg, Real& alt_m) const noexcept {
    lat_deg = latitude * constants::RAD_TO_DEG;
    lon_deg = longitude * constants::RAD_TO_DEG;
    alt_m = altitude;
}

// ============================================================================
// ECEF <-> Geodetic Conversions
// ============================================================================

namespace coord {

Vec3 lla_to_ecef(const GeodeticPosition& lla) noexcept {
    Real sin_lat = std::sin(lla.latitude);
    Real cos_lat = std::cos(lla.latitude);
    Real sin_lon = std::sin(lla.longitude);
    Real cos_lon = std::cos(lla.longitude);

    // Radius of curvature in prime vertical
    Real N = wgs84::a / std::sqrt(1.0 - wgs84::e2 * sin_lat * sin_lat);

    // ECEF coordinates
    Real x = (N + lla.altitude) * cos_lat * cos_lon;
    Real y = (N + lla.altitude) * cos_lat * sin_lon;
    Real z = (N * (1.0 - wgs84::e2) + lla.altitude) * sin_lat;

    return {x, y, z};
}

GeodeticPosition ecef_to_lla(const Vec3& ecef) noexcept {
    // Bowring's iterative method
    Real x = ecef.x;
    Real y = ecef.y;
    Real z = ecef.z;

    // Calculate longitude
    Real lon = std::atan2(y, x);

    // Distance from Z-axis
    Real p = std::sqrt(x * x + y * y);

    // Handle pole case
    if (p < 1e-10) {
        Real lat = (z >= 0.0) ? constants::PI / 2.0 : -constants::PI / 2.0;
        Real alt = std::abs(z) - wgs84::b;
        return {lat, lon, alt};
    }

    // Initial latitude estimate using Bowring's formula
    Real theta = std::atan2(z * wgs84::a, p * wgs84::b);
    Real sin_theta = std::sin(theta);
    Real cos_theta = std::cos(theta);

    Real lat = std::atan2(
        z + wgs84::ep2 * wgs84::b * sin_theta * sin_theta * sin_theta,
        p - wgs84::e2 * wgs84::a * cos_theta * cos_theta * cos_theta
    );

    // Iterate for accuracy (usually converges in 2-3 iterations)
    for (int i = 0; i < 5; ++i) {
        Real sin_lat = std::sin(lat);
        Real N = wgs84::a / std::sqrt(1.0 - wgs84::e2 * sin_lat * sin_lat);

        Real lat_new = std::atan2(
            z + wgs84::e2 * N * sin_lat,
            p
        );

        if (std::abs(lat_new - lat) < 1e-12) {
            lat = lat_new;
            break;
        }
        lat = lat_new;
    }

    // Calculate altitude
    Real sin_lat = std::sin(lat);
    Real cos_lat = std::cos(lat);
    Real N = wgs84::a / std::sqrt(1.0 - wgs84::e2 * sin_lat * sin_lat);
    Real alt;

    if (std::abs(cos_lat) > 1e-10) {
        alt = p / cos_lat - N;
    } else {
        alt = std::abs(z) / std::abs(sin_lat) - N * (1.0 - wgs84::e2);
    }

    return {lat, lon, alt};
}

// ============================================================================
// ECEF <-> NED Conversions
// ============================================================================

Mat3x3 ecef_to_ned_matrix(const GeodeticPosition& lla) noexcept {
    Real sin_lat = std::sin(lla.latitude);
    Real cos_lat = std::cos(lla.latitude);
    Real sin_lon = std::sin(lla.longitude);
    Real cos_lon = std::cos(lla.longitude);

    // ECEF to NED rotation matrix
    // Row 0: North direction in ECEF
    // Row 1: East direction in ECEF
    // Row 2: Down direction in ECEF
    Mat3x3 R;
    R(0, 0) = -sin_lat * cos_lon;  R(0, 1) = -sin_lat * sin_lon;  R(0, 2) = cos_lat;
    R(1, 0) = -sin_lon;            R(1, 1) = cos_lon;             R(1, 2) = 0.0;
    R(2, 0) = -cos_lat * cos_lon;  R(2, 1) = -cos_lat * sin_lon;  R(2, 2) = -sin_lat;

    return R;
}

Mat3x3 ned_to_ecef_matrix(const GeodeticPosition& lla) noexcept {
    return ecef_to_ned_matrix(lla).transpose();
}

Vec3 ecef_to_ned(const Vec3& v, const GeodeticPosition& lla) noexcept {
    return ecef_to_ned_matrix(lla) * v;
}

Vec3 ned_to_ecef(const Vec3& v, const GeodeticPosition& lla) noexcept {
    return ned_to_ecef_matrix(lla) * v;
}

// ============================================================================
// ECEF <-> ENU Conversions
// ============================================================================

Mat3x3 ecef_to_enu_matrix(const GeodeticPosition& lla) noexcept {
    Real sin_lat = std::sin(lla.latitude);
    Real cos_lat = std::cos(lla.latitude);
    Real sin_lon = std::sin(lla.longitude);
    Real cos_lon = std::cos(lla.longitude);

    // ECEF to ENU rotation matrix
    Mat3x3 R;
    R(0, 0) = -sin_lon;            R(0, 1) = cos_lon;             R(0, 2) = 0.0;
    R(1, 0) = -sin_lat * cos_lon;  R(1, 1) = -sin_lat * sin_lon;  R(1, 2) = cos_lat;
    R(2, 0) = cos_lat * cos_lon;   R(2, 1) = cos_lat * sin_lon;   R(2, 2) = sin_lat;

    return R;
}

Mat3x3 enu_to_ecef_matrix(const GeodeticPosition& lla) noexcept {
    return ecef_to_enu_matrix(lla).transpose();
}

Vec3 ecef_to_enu(const Vec3& v, const GeodeticPosition& lla) noexcept {
    return ecef_to_enu_matrix(lla) * v;
}

Vec3 enu_to_ecef(const Vec3& v, const GeodeticPosition& lla) noexcept {
    return enu_to_ecef_matrix(lla) * v;
}

// ============================================================================
// Body Frame Conversions
// ============================================================================

Vec3 ned_to_body(const Vec3& v, const Quat& orientation) noexcept {
    // orientation is body to NED, so we need inverse (NED to body)
    return orientation.conjugate().rotate(v);
}

Vec3 body_to_ned(const Vec3& v, const Quat& orientation) noexcept {
    return orientation.rotate(v);
}

// ============================================================================
// Geodetic Utilities
// ============================================================================

Real radius_of_curvature_n(Real lat) noexcept {
    Real sin_lat = std::sin(lat);
    return wgs84::a / std::sqrt(1.0 - wgs84::e2 * sin_lat * sin_lat);
}

Real radius_of_curvature_m(Real lat) noexcept {
    Real sin_lat = std::sin(lat);
    Real denom = 1.0 - wgs84::e2 * sin_lat * sin_lat;
    return wgs84::a * (1.0 - wgs84::e2) / (denom * std::sqrt(denom));
}

Real great_circle_distance(const GeodeticPosition& lla1, const GeodeticPosition& lla2) noexcept {
    // Haversine formula
    Real dlat = lla2.latitude - lla1.latitude;
    Real dlon = lla2.longitude - lla1.longitude;

    Real sin_dlat_2 = std::sin(dlat / 2.0);
    Real sin_dlon_2 = std::sin(dlon / 2.0);

    Real a = sin_dlat_2 * sin_dlat_2 +
             std::cos(lla1.latitude) * std::cos(lla2.latitude) *
             sin_dlon_2 * sin_dlon_2;

    Real c = 2.0 * std::atan2(std::sqrt(a), std::sqrt(1.0 - a));

    // Use mean Earth radius for simplicity
    constexpr Real R_MEAN = (wgs84::a + wgs84::b) / 2.0;
    return R_MEAN * c;
}

Real initial_bearing(const GeodeticPosition& lla1, const GeodeticPosition& lla2) noexcept {
    Real dlon = lla2.longitude - lla1.longitude;

    Real x = std::cos(lla2.latitude) * std::sin(dlon);
    Real y = std::cos(lla1.latitude) * std::sin(lla2.latitude) -
             std::sin(lla1.latitude) * std::cos(lla2.latitude) * std::cos(dlon);

    return std::atan2(x, y);
}

GeodeticPosition destination_point(const GeodeticPosition& start, Real bearing, Real distance) noexcept {
    // Use mean Earth radius
    constexpr Real R_MEAN = (wgs84::a + wgs84::b) / 2.0;
    Real angular_distance = distance / R_MEAN;

    Real sin_lat1 = std::sin(start.latitude);
    Real cos_lat1 = std::cos(start.latitude);
    Real sin_d = std::sin(angular_distance);
    Real cos_d = std::cos(angular_distance);
    Real sin_brng = std::sin(bearing);
    Real cos_brng = std::cos(bearing);

    Real lat2 = std::asin(sin_lat1 * cos_d + cos_lat1 * sin_d * cos_brng);
    Real lon2 = start.longitude + std::atan2(
        sin_brng * sin_d * cos_lat1,
        cos_d - sin_lat1 * std::sin(lat2)
    );

    // Normalize longitude to [-PI, PI]
    lon2 = math::normalize_angle(lon2);

    return {lat2, lon2, start.altitude};
}

// ============================================================================
// Quaternion Frame Conversions
// ============================================================================

Quat ecef_to_ned_quat(const GeodeticPosition& lla) noexcept {
    // Convert rotation matrix to quaternion
    Mat3x3 R = ecef_to_ned_matrix(lla);

    // Extract quaternion from rotation matrix
    Real trace = R(0, 0) + R(1, 1) + R(2, 2);
    Quat q;

    if (trace > 0.0) {
        Real s = 0.5 / std::sqrt(trace + 1.0);
        q.w = 0.25 / s;
        q.x = (R(2, 1) - R(1, 2)) * s;
        q.y = (R(0, 2) - R(2, 0)) * s;
        q.z = (R(1, 0) - R(0, 1)) * s;
    } else if (R(0, 0) > R(1, 1) && R(0, 0) > R(2, 2)) {
        Real s = 2.0 * std::sqrt(1.0 + R(0, 0) - R(1, 1) - R(2, 2));
        q.w = (R(2, 1) - R(1, 2)) / s;
        q.x = 0.25 * s;
        q.y = (R(0, 1) + R(1, 0)) / s;
        q.z = (R(0, 2) + R(2, 0)) / s;
    } else if (R(1, 1) > R(2, 2)) {
        Real s = 2.0 * std::sqrt(1.0 + R(1, 1) - R(0, 0) - R(2, 2));
        q.w = (R(0, 2) - R(2, 0)) / s;
        q.x = (R(0, 1) + R(1, 0)) / s;
        q.y = 0.25 * s;
        q.z = (R(1, 2) + R(2, 1)) / s;
    } else {
        Real s = 2.0 * std::sqrt(1.0 + R(2, 2) - R(0, 0) - R(1, 1));
        q.w = (R(1, 0) - R(0, 1)) / s;
        q.x = (R(0, 2) + R(2, 0)) / s;
        q.y = (R(1, 2) + R(2, 1)) / s;
        q.z = 0.25 * s;
    }

    return q.normalized();
}

Quat ned_quat_to_ecef_quat(const Quat& q_ned, const GeodeticPosition& lla) noexcept {
    // q_ecef = q_ned_to_ecef * q_ned
    Quat q_ned_to_ecef = ecef_to_ned_quat(lla).conjugate();
    return q_ned_to_ecef * q_ned;
}

Quat ecef_quat_to_ned_quat(const Quat& q_ecef, const GeodeticPosition& lla) noexcept {
    // q_ned = q_ecef_to_ned * q_ecef
    Quat q_ecef_to_ned = ecef_to_ned_quat(lla);
    return q_ecef_to_ned * q_ecef;
}

} // namespace coord

// ============================================================================
// Math Utilities
// ============================================================================

namespace math {

Real normalize_angle(Real angle) noexcept {
    while (angle > constants::PI) {
        angle -= 2.0 * constants::PI;
    }
    while (angle < -constants::PI) {
        angle += 2.0 * constants::PI;
    }
    return angle;
}

Real normalize_angle_positive(Real angle) noexcept {
    while (angle >= 2.0 * constants::PI) {
        angle -= 2.0 * constants::PI;
    }
    while (angle < 0.0) {
        angle += 2.0 * constants::PI;
    }
    return angle;
}

} // namespace math

} // namespace jaguar
