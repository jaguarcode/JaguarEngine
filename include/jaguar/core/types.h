#pragma once
/**
 * @file types.h
 * @brief Core type definitions for JaguarEngine
 *
 * This file defines fundamental types used throughout the engine,
 * including numeric types, coordinate frames, and domain identifiers.
 */

#include <cstdint>
#include <cstddef>
#include <limits>

namespace jaguar {

// ============================================================================
// Numeric Types
// ============================================================================

/**
 * @brief Primary floating-point type for physics calculations
 *
 * Using double precision (64-bit) for geodetic accuracy.
 * Single precision may introduce unacceptable errors for global coordinates.
 */
using Real = double;

/**
 * @brief Single precision floating-point (for rendering, less critical data)
 */
using Float32 = float;

/**
 * @brief Double precision floating-point
 */
using Float64 = double;

// Integer types
using Int8   = std::int8_t;
using Int16  = std::int16_t;
using Int32  = std::int32_t;
using Int64  = std::int64_t;
using UInt8  = std::uint8_t;
using UInt16 = std::uint16_t;
using UInt32 = std::uint32_t;
using UInt64 = std::uint64_t;
using SizeT  = std::size_t;

// ============================================================================
// Entity & Component Identifiers
// ============================================================================

/**
 * @brief Unique identifier for simulation entities
 */
using EntityId = UInt32;

/**
 * @brief Invalid entity ID constant
 */
constexpr EntityId INVALID_ENTITY_ID = std::numeric_limits<EntityId>::max();

/**
 * @brief Bitmask for component attachment tracking
 */
using ComponentMask = UInt64;

/**
 * @brief Property ID for fast property access
 */
using PropertyId = UInt32;

/**
 * @brief Invalid property ID constant
 */
constexpr PropertyId INVALID_PROPERTY_ID = std::numeric_limits<PropertyId>::max();

// ============================================================================
// Math Structures (Lightweight - for POD data storage)
// ============================================================================

// Forward declarations
struct Vec3;
struct Quat;
struct Mat3x3;

/**
 * @brief 3D vector (position, velocity, force, etc.)
 */
struct Vec3 {
    Real x{0.0};
    Real y{0.0};
    Real z{0.0};

    constexpr Vec3() noexcept = default;
    constexpr Vec3(Real x_, Real y_, Real z_) noexcept : x(x_), y(y_), z(z_) {}

    // Basic operations
    constexpr Vec3 operator+(const Vec3& other) const noexcept {
        return {x + other.x, y + other.y, z + other.z};
    }
    constexpr Vec3 operator-(const Vec3& other) const noexcept {
        return {x - other.x, y - other.y, z - other.z};
    }
    constexpr Vec3 operator*(Real scalar) const noexcept {
        return {x * scalar, y * scalar, z * scalar};
    }
    constexpr Vec3& operator+=(const Vec3& other) noexcept {
        x += other.x; y += other.y; z += other.z;
        return *this;
    }

    // Dot product
    constexpr Real dot(const Vec3& other) const noexcept {
        return x * other.x + y * other.y + z * other.z;
    }

    // Cross product
    constexpr Vec3 cross(const Vec3& other) const noexcept {
        return {
            y * other.z - z * other.y,
            z * other.x - x * other.z,
            x * other.y - y * other.x
        };
    }

    // Magnitude squared (avoid sqrt when possible)
    constexpr Real length_squared() const noexcept {
        return x*x + y*y + z*z;
    }

    // Scalar multiplication (scalar on left)
    friend constexpr Vec3 operator*(Real scalar, const Vec3& v) noexcept {
        return {v.x * scalar, v.y * scalar, v.z * scalar};
    }

    // Division by scalar
    constexpr Vec3 operator/(Real scalar) const noexcept {
        return {x / scalar, y / scalar, z / scalar};
    }

    // Unary minus
    constexpr Vec3 operator-() const noexcept {
        return {-x, -y, -z};
    }

    // Compound assignment operators
    constexpr Vec3& operator-=(const Vec3& other) noexcept {
        x -= other.x; y -= other.y; z -= other.z;
        return *this;
    }
    constexpr Vec3& operator*=(Real scalar) noexcept {
        x *= scalar; y *= scalar; z *= scalar;
        return *this;
    }
    constexpr Vec3& operator/=(Real scalar) noexcept {
        x /= scalar; y /= scalar; z /= scalar;
        return *this;
    }

    // Length (magnitude) - requires sqrt, defined in cpp
    Real length() const noexcept;

    // Normalized vector - defined in cpp
    Vec3 normalized() const noexcept;

    // Static zero vector
    static constexpr Vec3 Zero() noexcept { return {0.0, 0.0, 0.0}; }

    // Static unit vectors
    static constexpr Vec3 UnitX() noexcept { return {1.0, 0.0, 0.0}; }
    static constexpr Vec3 UnitY() noexcept { return {0.0, 1.0, 0.0}; }
    static constexpr Vec3 UnitZ() noexcept { return {0.0, 0.0, 1.0}; }
};

/**
 * @brief Quaternion for rotation representation (w, x, y, z)
 *
 * Using Hamilton convention: q = w + xi + yj + zk
 * Represents rotation from body frame to reference frame.
 */
struct Quat {
    Real w{1.0};  // Scalar part
    Real x{0.0};  // i component
    Real y{0.0};  // j component
    Real z{0.0};  // k component

    constexpr Quat() noexcept = default;
    constexpr Quat(Real w_, Real x_, Real y_, Real z_) noexcept
        : w(w_), x(x_), y(y_), z(z_) {}

    // Identity quaternion
    static constexpr Quat Identity() noexcept {
        return {1.0, 0.0, 0.0, 0.0};
    }

    // Quaternion multiplication (Hamilton product)
    constexpr Quat operator*(const Quat& q) const noexcept {
        return {
            w*q.w - x*q.x - y*q.y - z*q.z,
            w*q.x + x*q.w + y*q.z - z*q.y,
            w*q.y - x*q.z + y*q.w + z*q.x,
            w*q.z + x*q.y - y*q.x + z*q.w
        };
    }

    // Conjugate (inverse for unit quaternion)
    constexpr Quat conjugate() const noexcept {
        return {w, -x, -y, -z};
    }

    // Rotate a vector by this quaternion
    Vec3 rotate(const Vec3& v) const noexcept;

    // Scalar multiplication
    constexpr Quat operator*(Real scalar) const noexcept {
        return {w * scalar, x * scalar, y * scalar, z * scalar};
    }

    // Addition
    constexpr Quat operator+(const Quat& q) const noexcept {
        return {w + q.w, x + q.x, y + q.y, z + q.z};
    }

    // Magnitude squared
    constexpr Real norm_squared() const noexcept {
        return w*w + x*x + y*y + z*z;
    }

    // Magnitude - defined in cpp
    Real norm() const noexcept;

    // Normalized quaternion - defined in cpp
    Quat normalized() const noexcept;

    // Inverse (for unit quaternion, same as conjugate)
    Quat inverse() const noexcept;

    // Create from axis-angle representation
    static Quat from_axis_angle(const Vec3& axis, Real angle) noexcept;

    // Create from Euler angles (roll, pitch, yaw in radians)
    static Quat from_euler(Real roll, Real pitch, Real yaw) noexcept;

    // Convert to Euler angles
    void to_euler(Real& roll, Real& pitch, Real& yaw) const noexcept;

    // Convert to rotation matrix
    Mat3x3 to_rotation_matrix() const noexcept;
};

/**
 * @brief 3x3 matrix (inertia tensor, rotation matrix, etc.)
 *
 * Row-major storage: m[row][col] or m[row*3 + col]
 */
struct Mat3x3 {
    Real m[9]{1, 0, 0, 0, 1, 0, 0, 0, 1};  // Identity by default

    constexpr Mat3x3() noexcept = default;

    // Access by row, column
    constexpr Real& operator()(SizeT row, SizeT col) noexcept {
        return m[row * 3 + col];
    }
    constexpr Real operator()(SizeT row, SizeT col) const noexcept {
        return m[row * 3 + col];
    }

    // Identity matrix
    static constexpr Mat3x3 Identity() noexcept {
        Mat3x3 result;
        return result;
    }

    // Zero matrix
    static constexpr Mat3x3 Zero() noexcept {
        Mat3x3 result;
        for (auto& v : result.m) v = 0.0;
        return result;
    }

    // Matrix-vector multiplication
    constexpr Vec3 operator*(const Vec3& v) const noexcept {
        return {
            m[0]*v.x + m[1]*v.y + m[2]*v.z,
            m[3]*v.x + m[4]*v.y + m[5]*v.z,
            m[6]*v.x + m[7]*v.y + m[8]*v.z
        };
    }

    // Matrix-matrix multiplication
    constexpr Mat3x3 operator*(const Mat3x3& other) const noexcept {
        Mat3x3 result = Zero();
        for (SizeT i = 0; i < 3; ++i) {
            for (SizeT j = 0; j < 3; ++j) {
                for (SizeT k = 0; k < 3; ++k) {
                    result(i, j) += (*this)(i, k) * other(k, j);
                }
            }
        }
        return result;
    }

    // Transpose
    constexpr Mat3x3 transpose() const noexcept {
        Mat3x3 result;
        for (SizeT i = 0; i < 3; ++i) {
            for (SizeT j = 0; j < 3; ++j) {
                result(i, j) = (*this)(j, i);
            }
        }
        return result;
    }

    // Determinant
    constexpr Real determinant() const noexcept {
        return m[0] * (m[4]*m[8] - m[5]*m[7])
             - m[1] * (m[3]*m[8] - m[5]*m[6])
             + m[2] * (m[3]*m[7] - m[4]*m[6]);
    }

    // Inverse (returns identity if not invertible)
    Mat3x3 inverse() const noexcept;

    // Scalar multiplication
    constexpr Mat3x3 operator*(Real scalar) const noexcept {
        Mat3x3 result;
        for (SizeT i = 0; i < 9; ++i) {
            result.m[i] = m[i] * scalar;
        }
        return result;
    }

    // Addition
    constexpr Mat3x3 operator+(const Mat3x3& other) const noexcept {
        Mat3x3 result;
        for (SizeT i = 0; i < 9; ++i) {
            result.m[i] = m[i] + other.m[i];
        }
        return result;
    }

    // Create diagonal matrix
    static constexpr Mat3x3 Diagonal(Real a, Real b, Real c) noexcept {
        Mat3x3 result = Zero();
        result(0, 0) = a;
        result(1, 1) = b;
        result(2, 2) = c;
        return result;
    }

    // Create from inertia values (Ixx, Iyy, Izz, Ixy, Ixz, Iyz)
    static constexpr Mat3x3 Inertia(Real ixx, Real iyy, Real izz,
                                     Real ixy = 0, Real ixz = 0, Real iyz = 0) noexcept {
        Mat3x3 result;
        result(0, 0) = ixx;  result(0, 1) = -ixy; result(0, 2) = -ixz;
        result(1, 0) = -ixy; result(1, 1) = iyy;  result(1, 2) = -iyz;
        result(2, 0) = -ixz; result(2, 1) = -iyz; result(2, 2) = izz;
        return result;
    }
};

// ============================================================================
// Domain & Coordinate Frame Enumerations
// ============================================================================

/**
 * @brief Simulation domain classification
 */
enum class Domain : UInt8 {
    Air,      ///< Aircraft, missiles, projectiles
    Land,     ///< Ground vehicles, tracked vehicles
    Sea,      ///< Surface vessels, submarines
    Space,    ///< Satellites, re-entry vehicles
    Generic   ///< Generic rigid body
};

/**
 * @brief Coordinate reference frames
 */
enum class CoordinateFrame : UInt8 {
    ECEF,     ///< Earth-Centered, Earth-Fixed
    ECI,      ///< Earth-Centered Inertial (J2000)
    NED,      ///< North-East-Down (local tangent plane)
    ENU,      ///< East-North-Up (local tangent plane)
    Body      ///< Entity body-fixed frame
};

/**
 * @brief Simulation execution state
 */
enum class SimulationState : UInt8 {
    Uninitialized,
    Initialized,
    Running,
    Paused,
    Stopped,
    Error
};

// ============================================================================
// Physical Constants
// ============================================================================

namespace constants {

/// Gravitational constant (m³/kg/s²)
constexpr Real G = 6.67430e-11;

/// Earth's gravitational parameter (m³/s²)
constexpr Real MU_EARTH = 3.986004418e14;

/// Earth's equatorial radius (m)
constexpr Real R_EARTH_EQUATOR = 6378137.0;

/// Earth's polar radius (m)
constexpr Real R_EARTH_POLAR = 6356752.3142;

/// Earth's angular velocity (rad/s)
constexpr Real OMEGA_EARTH = 7.2921159e-5;

/// Standard gravity (m/s²)
constexpr Real G0 = 9.80665;

/// Speed of light (m/s)
constexpr Real C = 299792458.0;

/// Sea level standard atmospheric pressure (Pa)
constexpr Real P0 = 101325.0;

/// Sea level standard temperature (K)
constexpr Real T0 = 288.15;

/// Sea level standard air density (kg/m³)
constexpr Real RHO0 = 1.225;

/// Universal gas constant (J/mol/K)
constexpr Real R_GAS = 8.31446261815324;

/// Air specific gas constant (J/kg/K)
constexpr Real R_AIR = 287.05287;

/// Ratio of specific heats for air
constexpr Real GAMMA_AIR = 1.4;

/// Standard water density (kg/m³)
constexpr Real RHO_WATER = 1025.0;  // Seawater

/// Pi
constexpr Real PI = 3.14159265358979323846;

/// Degrees to radians conversion
constexpr Real DEG_TO_RAD = PI / 180.0;

/// Radians to degrees conversion
constexpr Real RAD_TO_DEG = 180.0 / PI;

/// Feet to meters conversion
constexpr Real FT_TO_M = 0.3048;

/// Meters to feet conversion
constexpr Real M_TO_FT = 1.0 / FT_TO_M;

/// Knots to m/s conversion
constexpr Real KT_TO_MS = 0.514444;

/// Pounds-force to Newtons conversion
constexpr Real LBF_TO_N = 4.44822;

/// Slugs to kilograms conversion
constexpr Real SLUG_TO_KG = 14.593903;

} // namespace constants

} // namespace jaguar
