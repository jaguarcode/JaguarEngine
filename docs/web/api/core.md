# Core API Reference

Core types, vectors, quaternions, and fundamental data structures.

**Header:** `jaguar/core/types.h`

## Basic Types

```cpp
namespace jaguar {
    using Real = double;         // Primary floating-point type
    using SizeT = std::size_t;   // Size type
    using UInt8 = uint8_t;       // 8-bit unsigned
    using UInt32 = uint32_t;     // 32-bit unsigned
    using UInt64 = uint64_t;     // 64-bit unsigned
    using Int32 = int32_t;       // 32-bit signed
    using Int64 = int64_t;       // 64-bit signed

    using ComponentMask = UInt64;
    using EntityId = UInt64;
    constexpr EntityId INVALID_ENTITY_ID = 0;
}
```

## Vec3

3D vector type for positions, velocities, and forces.

```cpp
struct Vec3 {
    Real x, y, z;

    // Constructors
    Vec3();
    Vec3(Real x, Real y, Real z);

    // Operations
    Real norm() const;                    // Vector magnitude
    Vec3 normalized() const;              // Unit vector
    Real dot(const Vec3& other) const;    // Dot product
    Vec3 cross(const Vec3& other) const;  // Cross product

    // Operators
    Vec3 operator+(const Vec3& other) const;
    Vec3 operator-(const Vec3& other) const;
    Vec3 operator*(Real scalar) const;
    Vec3 operator/(Real scalar) const;
    Vec3& operator+=(const Vec3& other);
    Vec3& operator-=(const Vec3& other);
    Vec3& operator*=(Real scalar);
};
```

### Example

```cpp
Vec3 position{100.0, 200.0, -500.0};
Vec3 velocity{50.0, 0.0, 0.0};

// Vector operations
Real speed = velocity.norm();           // 50.0
Vec3 direction = velocity.normalized(); // {1, 0, 0}

// Update position
position += velocity * dt;

// Cross product for torque
Vec3 force{0.0, 0.0, -9.81 * mass};
Vec3 arm{1.0, 0.0, 0.0};
Vec3 torque = arm.cross(force);
```

## Vec4

4D vector type.

```cpp
struct Vec4 {
    Real x, y, z, w;

    Vec4();
    Vec4(Real x, Real y, Real z, Real w);
};
```

## Quaternion

Rotation representation using quaternions.

```cpp
struct Quaternion {
    Real w, x, y, z;

    // Static constructors
    static Quaternion identity();
    static Quaternion from_euler(Real roll, Real pitch, Real yaw);
    static Quaternion from_axis_angle(const Vec3& axis, Real angle);

    // Operations
    Quaternion normalized() const;
    Quaternion conjugate() const;
    Vec3 rotate(const Vec3& v) const;
    Mat3x3 to_rotation_matrix() const;
    void to_euler(Real& roll, Real& pitch, Real& yaw) const;

    // Operators
    Quaternion operator*(const Quaternion& other) const;
};
```

### Example

```cpp
// Create from Euler angles (radians)
Real roll = 0.0;
Real pitch = 10.0 * DEG_TO_RAD;
Real yaw = 45.0 * DEG_TO_RAD;
Quaternion orientation = Quaternion::from_euler(roll, pitch, yaw);

// Rotate a vector
Vec3 body_force{1000.0, 0.0, 0.0};
Vec3 world_force = orientation.rotate(body_force);

// Combine rotations
Quaternion rotation = Quaternion::from_axis_angle(Vec3{0, 0, 1}, 0.1);
orientation = orientation * rotation;

// Convert back to Euler
orientation.to_euler(roll, pitch, yaw);
```

## Mat3x3

3x3 matrix for rotations and inertia tensors.

```cpp
struct Mat3x3 {
    Real data[3][3];

    // Static constructors
    static Mat3x3 identity();
    static Mat3x3 from_euler(Real roll, Real pitch, Real yaw);

    // Operations
    Vec3 operator*(const Vec3& v) const;
    Mat3x3 operator*(const Mat3x3& other) const;
    Mat3x3 transpose() const;
    Mat3x3 inverse() const;
    Real determinant() const;
};
```

### Example

```cpp
// Create inertia tensor
Mat3x3 inertia = Mat3x3::identity();
inertia.data[0][0] = 10000.0;  // Ixx
inertia.data[1][1] = 50000.0;  // Iyy
inertia.data[2][2] = 55000.0;  // Izz

// Transform vector
Vec3 angular_momentum = inertia * angular_velocity;

// Rotation matrix
Mat3x3 rotation = Mat3x3::from_euler(roll, pitch, yaw);
Vec3 world_vec = rotation * body_vec;
```

## Enumerations

### Domain

```cpp
enum class Domain : UInt8 {
    Generic = 0,  // Generic entity
    Air = 1,      // Aircraft, missiles
    Land = 2,     // Ground vehicles
    Sea = 3,      // Ships, submarines
    Space = 4     // Satellites, spacecraft
};
```

### CoordinateFrame

```cpp
enum class CoordinateFrame : UInt8 {
    ECEF = 0,   // Earth-Centered Earth-Fixed
    NED = 1,    // North-East-Down (local)
    ENU = 2,    // East-North-Up (local)
    ECI = 3     // Earth-Centered Inertial
};
```

## Constants

Physical and mathematical constants.

```cpp
namespace constants {
    // Mathematical
    constexpr Real PI = 3.14159265358979323846;
    constexpr Real TWO_PI = 6.28318530717958647692;
    constexpr Real HALF_PI = 1.57079632679489661923;

    // Conversion
    constexpr Real DEG_TO_RAD = PI / 180.0;
    constexpr Real RAD_TO_DEG = 180.0 / PI;

    // Physical
    constexpr Real G0 = 9.80665;              // Standard gravity (m/s²)
    constexpr Real EARTH_RADIUS = 6378137.0;  // WGS84 equatorial (m)
    constexpr Real EARTH_POLAR = 6356752.3;   // WGS84 polar (m)
    constexpr Real EARTH_MU = 3.986004418e14; // Gravitational param (m³/s²)
    constexpr Real EARTH_J2 = 1.08263e-3;     // J2 perturbation
    constexpr Real EARTH_OMEGA = 7.2921159e-5; // Rotation rate (rad/s)

    // Atmosphere
    constexpr Real SEA_LEVEL_PRESSURE = 101325.0;    // Pa
    constexpr Real SEA_LEVEL_TEMPERATURE = 288.15;   // K
    constexpr Real SEA_LEVEL_DENSITY = 1.225;        // kg/m³
    constexpr Real SPEED_OF_SOUND_SL = 340.29;       // m/s

    // Water
    constexpr Real WATER_DENSITY = 1025.0;   // Seawater (kg/m³)
}
```

### Example

```cpp
using namespace jaguar::constants;

// Convert angles
Real heading_deg = 45.0;
Real heading_rad = heading_deg * DEG_TO_RAD;

// Calculate orbital velocity
Real altitude = 400000.0;  // 400 km
Real radius = EARTH_RADIUS + altitude;
Real orbital_velocity = std::sqrt(EARTH_MU / radius);

// Gravity at altitude
Real g = G0 * std::pow(EARTH_RADIUS / radius, 2);
```

## Coordinate Transforms

Coordinate system conversions.

```cpp
namespace transforms {
    // ECEF <-> Geodetic
    Vec3 geodetic_to_ecef(Real lat, Real lon, Real alt);
    void ecef_to_geodetic(const Vec3& ecef, Real& lat, Real& lon, Real& alt);

    // ECEF <-> ECI
    Vec3 ecef_to_eci(const Vec3& ecef, Real jd);
    Vec3 eci_to_ecef(const Vec3& eci, Real jd);

    // Local frames
    Mat3x3 ecef_to_ned_rotation(Real lat, Real lon);
    Mat3x3 ecef_to_enu_rotation(Real lat, Real lon);

    // Time
    Real greenwich_sidereal_time(Real jd);
    Real julian_date(int year, int month, int day,
                     int hour, int min, Real sec);
}
```

### Example

```cpp
using namespace jaguar::transforms;

// Convert geodetic to ECEF
Real lat = 37.0 * DEG_TO_RAD;
Real lon = -122.0 * DEG_TO_RAD;
Real alt = 1000.0;  // 1 km altitude
Vec3 ecef = geodetic_to_ecef(lat, lon, alt);

// Convert ECEF to ECI for orbital calculations
Real jd = julian_date(2024, 1, 1, 12, 0, 0.0);
Vec3 eci = ecef_to_eci(ecef, jd);

// Get local NED frame
Mat3x3 ned_rotation = ecef_to_ned_rotation(lat, lon);
Vec3 velocity_ned = ned_rotation * velocity_ecef;
```

## See Also

- [Physics API](physics.md) - Entity state and forces
- [Concepts: Coordinates](../concepts/coordinates.md) - Coordinate systems explained
- [API Overview](overview.md) - Complete API index

