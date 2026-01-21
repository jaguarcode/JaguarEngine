# Coordinate Systems

JaguarEngine supports multiple coordinate frames for different applications. This guide covers coordinate systems, transformations, and conventions.

## Supported Frames

| Frame | Origin | Axes | Use Case |
|-------|--------|------|----------|
| ECEF | Earth center | Earth-fixed | Global positioning |
| ECI J2000 | Earth center | Inertial | Orbital mechanics |
| NED | Local origin | North-East-Down | Aircraft, vehicles |
| ENU | Local origin | East-North-Up | Visualization |
| Body | Entity CG | Entity-specific | Local calculations |

## ECEF (Earth-Centered Earth-Fixed)

### Definition

- **Origin**: Earth's center of mass
- **X-axis**: Through prime meridian at equator
- **Y-axis**: Through 90°E longitude at equator
- **Z-axis**: Through North Pole

### Usage

```cpp
// ECEF is the default frame in JaguarEngine
Vec3 position_ecef{6378137.0, 0.0, 0.0};  // On equator at prime meridian

// Entity positions are stored in ECEF
auto state = engine.get_entity_state(entity);
Vec3 ecef_pos = state.position;  // ECEF meters
```

### Advantages

- Global reference frame
- No singularities
- Direct geodetic conversion

## ECI (Earth-Centered Inertial)

### Definition

- **Origin**: Earth's center of mass
- **X-axis**: Toward vernal equinox (J2000)
- **Y-axis**: Completes right-hand system
- **Z-axis**: Toward North celestial pole

### Usage

```cpp
using namespace jaguar::transforms;

// Convert ECI to ECEF
Vec3 pos_eci{6878000.0, 0.0, 0.0};
Real julian_date = 2451545.0;  // J2000 epoch
Vec3 pos_ecef = eci_to_ecef(pos_eci, julian_date);

// Convert ECEF to ECI
Vec3 pos_eci_back = ecef_to_eci(pos_ecef, julian_date);
```

### Space Domain

```cpp
// SGP4 outputs ECI coordinates
SGP4Propagator sgp4;
sgp4.initialize(tle);

Vec3 pos_eci, vel_eci;
sgp4.propagate(minutes, pos_eci, vel_eci);
```

## NED (North-East-Down)

### Definition

- **Origin**: Local reference point
- **X-axis**: True North
- **Y-axis**: True East
- **Z-axis**: Down (toward Earth center)

### Usage

```cpp
// Aircraft state often expressed in NED
// Position: relative to local origin
// Velocity: NED components

// NED velocity example
Vec3 vel_ned{200.0, 0.0, -10.0};  // 200 m/s North, 10 m/s descent
```

### Air Domain Conventions

In the Air domain, positions are often NED:
- Positive X = North
- Positive Y = East
- Positive Z = Down (altitude is -Z)

```cpp
// Altitude from NED position
Real altitude = -state.position.z;
```

## Body Frame

### Definition

- **Origin**: Entity center of gravity (CG)
- **X-axis**: Forward
- **Y-axis**: Right
- **Z-axis**: Down

### Domain Conventions

| Domain | X (Forward) | Y (Right) | Z (Down) |
|--------|-------------|-----------|----------|
| Air | Nose | Starboard wing | Belly |
| Land | Vehicle front | Passenger side | Ground |
| Sea | Bow | Starboard | Keel |
| Space | Primary thrust | Secondary | Nadir |

### Body-to-ECEF Transformation

```cpp
// Orientation quaternion transforms body to ECEF
Quaternion q = state.orientation;

// Transform body vector to ECEF
Vec3 body_vec{1.0, 0.0, 0.0};  // Forward direction
Vec3 ecef_vec = q.rotate(body_vec);

// Get rotation matrix
Mat3x3 dcm = q.to_rotation_matrix();
```

## Geodetic Coordinates

### Definition

- **Latitude**: Angle from equatorial plane (-90° to +90°)
- **Longitude**: Angle from prime meridian (-180° to +180°)
- **Altitude**: Height above WGS84 ellipsoid

### Conversions

```cpp
using namespace jaguar::transforms;

// ECEF to Geodetic
Vec3 ecef{6378137.0, 0.0, 0.0};
Real lat, lon, alt;
ecef_to_geodetic(ecef, lat, lon, alt);
// lat = 0, lon = 0, alt = 0

// Geodetic to ECEF
Real lat_rad = 37.0 * DEG_TO_RAD;
Real lon_rad = -122.0 * DEG_TO_RAD;
Real alt_m = 100.0;
Vec3 ecef_out = geodetic_to_ecef(lat_rad, lon_rad, alt_m);
```

## Euler Angles

### Convention

JaguarEngine uses aerospace Euler angles (3-2-1 rotation order):

1. **Yaw** (ψ): Rotation about Z-axis
2. **Pitch** (θ): Rotation about Y-axis
3. **Roll** (φ): Rotation about X-axis

### Usage

```cpp
// Create quaternion from Euler angles
Real roll = 10.0 * DEG_TO_RAD;
Real pitch = 5.0 * DEG_TO_RAD;
Real yaw = 45.0 * DEG_TO_RAD;
Quaternion q = Quaternion::from_euler(roll, pitch, yaw);

// Extract Euler angles from quaternion
Real roll_out, pitch_out, yaw_out;
q.to_euler(roll_out, pitch_out, yaw_out);

// From entity state
Real aircraft_roll = state.get_roll();
Real aircraft_pitch = state.get_pitch();
Real aircraft_yaw = state.get_yaw();
```

## Angular Rates

### Body Frame Rates

| Symbol | Axis | Name |
|--------|------|------|
| p | X | Roll rate |
| q | Y | Pitch rate |
| r | Z | Yaw rate |

```cpp
// Angular velocity is in body frame
Vec3 omega = state.angular_velocity;
Real p = omega.x;  // Roll rate (rad/s)
Real q = omega.y;  // Pitch rate (rad/s)
Real r = omega.z;  // Yaw rate (rad/s)
```

## Quaternions

### Operations

```cpp
// Create identity quaternion
Quaternion q = Quaternion::identity();

// From axis-angle
Vec3 axis{0.0, 0.0, 1.0};  // Z-axis
Real angle = 45.0 * DEG_TO_RAD;
Quaternion q = Quaternion::from_axis_angle(axis, angle);

// Quaternion multiplication (composition)
Quaternion q3 = q1 * q2;  // Apply q2, then q1

// Rotate a vector
Vec3 v_rotated = q.rotate(v);

// Inverse rotation
Vec3 v_original = q.conjugate().rotate(v_rotated);

// Normalize (maintain unit quaternion)
q = q.normalized();
```

### Integration

```cpp
// Quaternion derivative
// q_dot = 0.5 * q * omega_quat
Quaternion omega_quat{0, omega.x, omega.y, omega.z};
Quaternion q_dot = q * omega_quat * 0.5;

// Integration step
q = (q + q_dot * dt).normalized();
```

## Time Systems

### Julian Date

```cpp
// Greenwich Mean Sidereal Time
Real jd = 2451545.0;  // J2000 epoch
Real gmst = transforms::greenwich_sidereal_time(jd);
```

### Simulation Time

```cpp
// Get current simulation time
Real sim_time = engine.get_time();

// Set simulation time
engine.set_time(0.0);
```

## Best Practices

### Coordinate Consistency

1. **Document frame**: Always specify which frame data is in
2. **Transform early**: Convert to working frame at input
3. **Transform late**: Convert to output frame only when needed

### Numerical Precision

1. **Use double precision**: Essential for geodetic calculations
2. **Normalize quaternions**: Prevent drift in orientation
3. **Avoid singularities**: Use quaternions instead of Euler angles for integration

### Common Pitfalls

- **Forgetting frame**: Mixing ECEF and NED positions
- **Sign conventions**: Down is positive Z in NED
- **Angular units**: Radians vs degrees
- **Rotation order**: Euler angle convention matters

## See Also

- [Entities](entities.md) - Entity state management
- [API Reference](../api/core.md) - Core types and math
- [Space Domain](../domains/space.md) - Orbital coordinate frames
