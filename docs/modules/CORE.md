# Core Module Documentation

The Core module provides fundamental types, mathematical structures, memory management, and the property system that underpin all other JaguarEngine components.

## Headers

| Header | Purpose |
|--------|---------|
| `jaguar/core/types.h` | Fundamental types, math structures, constants |
| `jaguar/core/memory.h` | Pool allocators, aligned containers |
| `jaguar/core/property.h` | Hierarchical property system |
| `jaguar/core/simd.h` | SIMD operations and intrinsics |
| `jaguar/core/coordinates.h` | Coordinate frame conversions |

## Types (`types.h`)

### Numeric Types

```cpp
namespace jaguar {
    using Real = double;         // Primary floating-point type (64-bit)
    using Float32 = float;       // Single precision
    using Float64 = double;      // Double precision

    using Int8 = std::int8_t;
    using Int16 = std::int16_t;
    using Int32 = std::int32_t;
    using Int64 = std::int64_t;
    using UInt8 = std::uint8_t;
    using UInt16 = std::uint16_t;
    using UInt32 = std::uint32_t;
    using UInt64 = std::uint64_t;
    using SizeT = std::size_t;
}
```

JaguarEngine uses `double` precision for all physics calculations to maintain geodetic accuracy over large distances.

### Entity Identifiers

```cpp
using EntityId = UInt32;
constexpr EntityId INVALID_ENTITY_ID = std::numeric_limits<EntityId>::max();

using ComponentMask = UInt64;    // Bitmask for component attachment
using PropertyId = UInt32;       // Fast property access
```

### Math Structures

#### Vec3 - 3D Vector

```cpp
struct Vec3 {
    Real x{0.0}, y{0.0}, z{0.0};

    // Construction
    constexpr Vec3() noexcept = default;
    constexpr Vec3(Real x_, Real y_, Real z_) noexcept;

    // Arithmetic
    constexpr Vec3 operator+(const Vec3& other) const noexcept;
    constexpr Vec3 operator-(const Vec3& other) const noexcept;
    constexpr Vec3 operator*(Real scalar) const noexcept;
    constexpr Vec3 operator/(Real scalar) const noexcept;
    constexpr Vec3 operator-() const noexcept;

    // Compound assignment
    constexpr Vec3& operator+=(const Vec3& other) noexcept;
    constexpr Vec3& operator-=(const Vec3& other) noexcept;
    constexpr Vec3& operator*=(Real scalar) noexcept;
    constexpr Vec3& operator/=(Real scalar) noexcept;

    // Products
    constexpr Real dot(const Vec3& other) const noexcept;
    constexpr Vec3 cross(const Vec3& other) const noexcept;

    // Magnitude
    constexpr Real length_squared() const noexcept;
    Real length() const noexcept;
    Vec3 normalized() const noexcept;

    // Static factories
    static constexpr Vec3 Zero() noexcept;
    static constexpr Vec3 UnitX() noexcept;
    static constexpr Vec3 UnitY() noexcept;
    static constexpr Vec3 UnitZ() noexcept;
};
```

**Usage:**
```cpp
Vec3 pos{100.0, 200.0, -1000.0};
Vec3 vel{50.0, 0.0, -5.0};

Vec3 new_pos = pos + vel * 0.01;           // Position update
Real speed = vel.length();                  // Magnitude
Vec3 direction = vel.normalized();          // Unit vector
Real component = vel.dot(Vec3::UnitX());   // Dot product
Vec3 torque = pos.cross(force);            // Cross product
```

#### Quat - Quaternion

```cpp
struct Quat {
    Real w{1.0}, x{0.0}, y{0.0}, z{0.0};  // Hamilton convention

    // Construction
    constexpr Quat() noexcept = default;
    constexpr Quat(Real w_, Real x_, Real y_, Real z_) noexcept;
    static constexpr Quat Identity() noexcept;
    static Quat from_axis_angle(const Vec3& axis, Real angle) noexcept;
    static Quat from_euler(Real roll, Real pitch, Real yaw) noexcept;

    // Operations
    constexpr Quat operator*(const Quat& q) const noexcept;
    constexpr Quat conjugate() const noexcept;
    Quat inverse() const noexcept;
    Quat normalized() const noexcept;

    // Vector rotation
    Vec3 rotate(const Vec3& v) const noexcept;

    // Conversion
    void to_euler(Real& roll, Real& pitch, Real& yaw) const noexcept;
    Mat3x3 to_rotation_matrix() const noexcept;

    // Magnitude
    constexpr Real norm_squared() const noexcept;
    Real norm() const noexcept;
};
```

**Usage:**
```cpp
// Create rotation from Euler angles (roll=0, pitch=5°, yaw=45°)
Quat orientation = Quat::from_euler(0.0, 5.0 * DEG_TO_RAD, 45.0 * DEG_TO_RAD);

// Rotate a vector
Vec3 body_vel{100.0, 0.0, 0.0};
Vec3 world_vel = orientation.rotate(body_vel);

// Inverse rotation (world to body)
Vec3 back_to_body = orientation.conjugate().rotate(world_vel);

// Extract Euler angles
Real roll, pitch, yaw;
orientation.to_euler(roll, pitch, yaw);
```

#### Mat3x3 - 3x3 Matrix

```cpp
struct Mat3x3 {
    Real m[9];  // Row-major storage

    // Access
    constexpr Real& operator()(SizeT row, SizeT col) noexcept;
    constexpr Real operator()(SizeT row, SizeT col) const noexcept;

    // Static factories
    static constexpr Mat3x3 Identity() noexcept;
    static constexpr Mat3x3 Zero() noexcept;
    static constexpr Mat3x3 Diagonal(Real a, Real b, Real c) noexcept;
    static constexpr Mat3x3 Inertia(Real ixx, Real iyy, Real izz,
                                    Real ixy = 0, Real ixz = 0, Real iyz = 0) noexcept;

    // Operations
    constexpr Vec3 operator*(const Vec3& v) const noexcept;
    constexpr Mat3x3 operator*(const Mat3x3& other) const noexcept;
    constexpr Mat3x3 operator*(Real scalar) const noexcept;
    constexpr Mat3x3 operator+(const Mat3x3& other) const noexcept;
    constexpr Mat3x3 transpose() const noexcept;
    constexpr Real determinant() const noexcept;
    Mat3x3 inverse() const noexcept;
};
```

**Usage:**
```cpp
// Create inertia tensor
Mat3x3 inertia = Mat3x3::Inertia(1000.0, 5000.0, 5500.0);  // kg·m²

// Matrix-vector multiplication
Vec3 angular_momentum = inertia * angular_velocity;

// Rotation matrix from quaternion
Mat3x3 R = orientation.to_rotation_matrix();
Vec3 rotated = R * vector;
```

### Enumerations

```cpp
enum class Domain : UInt8 {
    Air,      // Aircraft, missiles, projectiles
    Land,     // Ground vehicles, tracked vehicles
    Sea,      // Surface vessels, submarines
    Space,    // Satellites, re-entry vehicles
    Generic   // Generic rigid body
};

enum class CoordinateFrame : UInt8 {
    ECEF,     // Earth-Centered, Earth-Fixed
    ECI,      // Earth-Centered Inertial (J2000)
    NED,      // North-East-Down (local tangent plane)
    ENU,      // East-North-Up (local tangent plane)
    Body      // Entity body-fixed frame
};

enum class SimulationState : UInt8 {
    Uninitialized,
    Initialized,
    Running,
    Paused,
    Stopped,
    Error
};
```

### Physical Constants

```cpp
namespace jaguar::constants {
    constexpr Real G = 6.67430e-11;              // Gravitational constant (m³/kg/s²)
    constexpr Real MU_EARTH = 3.986004418e14;   // Earth GM (m³/s²)
    constexpr Real R_EARTH_EQUATOR = 6378137.0; // Equatorial radius (m)
    constexpr Real R_EARTH_POLAR = 6356752.3142;// Polar radius (m)
    constexpr Real OMEGA_EARTH = 7.2921159e-5;  // Earth angular velocity (rad/s)
    constexpr Real G0 = 9.80665;                // Standard gravity (m/s²)
    constexpr Real C = 299792458.0;             // Speed of light (m/s)

    // Atmospheric
    constexpr Real P0 = 101325.0;               // Sea level pressure (Pa)
    constexpr Real T0 = 288.15;                 // Sea level temperature (K)
    constexpr Real RHO0 = 1.225;                // Sea level density (kg/m³)
    constexpr Real R_AIR = 287.05287;           // Air gas constant (J/kg/K)
    constexpr Real GAMMA_AIR = 1.4;             // Ratio of specific heats

    // Ocean
    constexpr Real RHO_WATER = 1025.0;          // Seawater density (kg/m³)

    // Mathematical
    constexpr Real PI = 3.14159265358979323846;
    constexpr Real DEG_TO_RAD = PI / 180.0;
    constexpr Real RAD_TO_DEG = 180.0 / PI;

    // Unit conversions
    constexpr Real FT_TO_M = 0.3048;
    constexpr Real M_TO_FT = 1.0 / FT_TO_M;
    constexpr Real KT_TO_MS = 0.514444;
    constexpr Real LBF_TO_N = 4.44822;
    constexpr Real SLUG_TO_KG = 14.593903;
}
```

## Coordinates (`coordinates.h`)

### Coordinate Conversions

```cpp
namespace jaguar::coordinates {

    // Geodetic to ECEF
    Vec3 geodetic_to_ecef(Real latitude, Real longitude, Real altitude);

    // ECEF to Geodetic
    void ecef_to_geodetic(const Vec3& ecef, Real& lat, Real& lon, Real& alt);

    // NED to ECEF rotation matrix at position
    Mat3x3 ned_to_ecef_rotation(Real latitude, Real longitude);

    // ECEF to NED rotation matrix
    Mat3x3 ecef_to_ned_rotation(Real latitude, Real longitude);

    // ECI to ECEF (requires time for Earth rotation)
    Vec3 eci_to_ecef(const Vec3& eci, Real julian_date);
    Vec3 ecef_to_eci(const Vec3& ecef, Real julian_date);

    // LVLH (Local Vertical Local Horizontal) for orbital mechanics
    Mat3x3 ecef_to_lvlh_rotation(const Vec3& position, const Vec3& velocity);
}
```

**Usage:**
```cpp
using namespace jaguar::coordinates;

// Convert lat/lon/alt to ECEF
Real lat = 37.0 * DEG_TO_RAD;
Real lon = -122.0 * DEG_TO_RAD;
Real alt = 1000.0;  // meters
Vec3 ecef = geodetic_to_ecef(lat, lon, alt);

// Get NED rotation at this position
Mat3x3 R_ned = ned_to_ecef_rotation(lat, lon);

// Convert NED velocity to ECEF velocity
Vec3 vel_ned{100.0, 0.0, -5.0};  // North, East, Down
Vec3 vel_ecef = R_ned * vel_ned;
```

## Memory Management (`memory.h`)

### Pool Allocator

```cpp
namespace jaguar::memory {

    class PoolAllocator {
    public:
        PoolAllocator(SizeT block_size, SizeT block_count);
        ~PoolAllocator();

        void* allocate();
        void deallocate(void* ptr);

        SizeT available_blocks() const;
        SizeT total_blocks() const;
        SizeT block_size() const;
    };
}
```

### Aligned Containers

```cpp
template<typename T, SizeT Alignment = 64>
class AlignedVector {
public:
    void push_back(const T& value);
    T& operator[](SizeT index);
    T* data();
    SizeT size() const;
    void reserve(SizeT capacity);
    void clear();
};
```

## Property System (`property.h`)

The property system provides JSBSim-style hierarchical access to simulation parameters.

### Property Manager

```cpp
namespace jaguar::property {

    class PropertyManager {
    public:
        // Registration
        PropertyId register_property(const std::string& path, Real* value_ptr);
        PropertyId register_property(const std::string& path, Real initial_value);

        // Access by path
        Real get(const std::string& path) const;
        void set(const std::string& path, Real value);

        // Access by ID (fast)
        Real get(PropertyId id) const;
        void set(PropertyId id, Real value);

        // Query
        bool exists(const std::string& path) const;
        PropertyId find(const std::string& path) const;

        // Iteration
        void for_each(std::function<void(const std::string&, Real)> func) const;
    };
}
```

**Usage:**
```cpp
PropertyManager props;

// Register properties
props.register_property("position/x-m", 0.0);
props.register_property("position/y-m", 0.0);
props.register_property("position/z-m", -1000.0);
props.register_property("velocities/u-fps", 500.0);
props.register_property("aero/alpha-rad", 0.05);

// Read/write by path
Real alpha = props.get("aero/alpha-rad");
props.set("position/x-m", 100.0);

// Fast access by ID
PropertyId alpha_id = props.find("aero/alpha-rad");
Real alpha_fast = props.get(alpha_id);
```

### Property Paths

Property paths follow a hierarchical naming convention:

```
category/subcategory/name-unit

Examples:
  position/x-m              Position X in meters
  position/lat-rad          Latitude in radians
  velocities/u-fps          Body X velocity in feet/second
  velocities/v-fps          Body Y velocity
  velocities/w-fps          Body Z velocity
  attitude/phi-rad          Roll angle
  attitude/theta-rad        Pitch angle
  attitude/psi-rad          Yaw angle
  aero/alpha-rad            Angle of attack
  aero/beta-rad             Sideslip angle
  aero/qbar-psf             Dynamic pressure
  propulsion/thrust-lbs     Engine thrust
  fcs/elevator-cmd          Elevator command (-1 to 1)
```

## Best Practices

### Memory Efficiency

```cpp
// Use SoA (Structure of Arrays) for batch processing
std::vector<Vec3> positions;
std::vector<Vec3> velocities;
std::vector<Quat> orientations;

// Instead of AoS (Array of Structures)
// std::vector<EntityState> entities;  // Less cache-friendly
```

### Coordinate Frame Consistency

Always document which coordinate frame vectors are in:

```cpp
Vec3 vel_body{100.0, 0.0, 0.0};  // Body frame: forward, right, down
Vec3 vel_ned = orientation.rotate(vel_body);  // NED frame: north, east, down
Vec3 vel_ecef = R_ned * vel_ned;  // ECEF frame
```

### Unit Conventions

JaguarEngine internally uses SI units:
- **Distance**: meters (m)
- **Mass**: kilograms (kg)
- **Time**: seconds (s)
- **Angle**: radians (rad)
- **Force**: newtons (N)
- **Temperature**: kelvin (K)

Use constants for conversion:
```cpp
Real altitude_ft = 30000.0;
Real altitude_m = altitude_ft * constants::FT_TO_M;

Real heading_deg = 45.0;
Real heading_rad = heading_deg * constants::DEG_TO_RAD;
```
