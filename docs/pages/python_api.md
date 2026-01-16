# Python API Reference {#python_api}

JaguarEngine provides Python bindings through the `pyjaguar` module, enabling rapid prototyping and seamless integration with Python's scientific ecosystem.

## Installation

Build JaguarEngine with Python support:

```bash
cmake -DJAGUAR_BUILD_PYTHON=ON ..
cmake --build .
```

The `pyjaguar` module will be built in the `build/` directory.

## Module Overview

```python
import pyjaguar as jag

# Version information
print(f"JaguarEngine version: {jag.__version__}")
```

## Classes

### Vec3

3D vector class for positions, velocities, and forces.

```python
# Construction
v1 = jag.Vec3()              # Zero vector
v2 = jag.Vec3(1.0, 2.0, 3.0) # From components

# Properties
print(v1.x, v1.y, v1.z)

# Operations
v3 = v1 + v2                 # Addition
v4 = v1 - v2                 # Subtraction
v5 = v1 * 2.0                # Scalar multiplication
v6 = v1 / 2.0                # Scalar division

# Methods
dot_product = v1.dot(v2)
cross_product = v1.cross(v2)
length = v1.length()
length_sq = v1.length_squared()
normalized = v1.normalized()

# NumPy interop
import numpy as np
arr = v1.to_numpy()                          # Vec3 -> ndarray
v_from_np = jag.Vec3.from_numpy(np.array([1,2,3]))  # ndarray -> Vec3
```

### Quat

Quaternion class for orientations and rotations.

```python
# Construction
q1 = jag.Quat.identity()     # Identity quaternion
q2 = jag.Quat.from_euler(roll, pitch, yaw)  # From Euler angles (rad)
q3 = jag.Quat.from_axis_angle(axis, angle)  # From axis-angle

# Properties
print(q1.w, q1.x, q1.y, q1.z)

# Operations
q4 = q1 * q2                 # Quaternion multiplication
rotated_vec = q1.rotate(v)   # Rotate a vector

# Methods
q_conj = q1.conjugate()
q_inv = q1.inverse()
roll, pitch, yaw = q1.to_euler()  # To Euler angles
```

### Mat3x3

3x3 matrix class for inertia tensors and rotations.

```python
# Construction
m1 = jag.Mat3x3.identity()
m2 = jag.Mat3x3.zero()
m3 = jag.Mat3x3.inertia(Ixx, Iyy, Izz)  # Diagonal inertia

# Element access
val = m1[0, 0]               # Get element

# Operations
m4 = m1 * m2                 # Matrix multiplication
v = m1 * vec                 # Matrix-vector multiplication

# Methods
m_t = m1.transpose()
det = m1.determinant()
m_inv = m1.inverse()
tr = m1.trace()
```

### Domain

Enumeration for simulation domains.

```python
jag.Domain.Air               # Aircraft, missiles
jag.Domain.Land              # Ground vehicles
jag.Domain.Sea               # Ships, submarines
jag.Domain.Space             # Satellites, spacecraft
jag.Domain.Unknown           # Unspecified
```

### CoordinateFrame

Enumeration for coordinate frames.

```python
jag.CoordinateFrame.ECEF     # Earth-Centered Earth-Fixed
jag.CoordinateFrame.ECI      # Earth-Centered Inertial
jag.CoordinateFrame.NED      # North-East-Down
jag.CoordinateFrame.ENU      # East-North-Up
jag.CoordinateFrame.Body     # Body-fixed frame
jag.CoordinateFrame.Wind     # Wind-aligned frame
jag.CoordinateFrame.Stability  # Stability axis frame
```

### SimulationState

Enumeration for simulation states.

```python
jag.SimulationState.Uninitialized
jag.SimulationState.Initialized
jag.SimulationState.Running
jag.SimulationState.Paused
jag.SimulationState.Stopped
```

### EntityState

Structure containing entity kinematic state.

```python
state = jag.EntityState()

# Position and velocity (Vec3)
state.position = jag.Vec3(x, y, z)
state.velocity = jag.Vec3(vx, vy, vz)
state.acceleration = jag.Vec3(ax, ay, az)

# Orientation and angular rates
state.orientation = jag.Quat.from_euler(r, p, y)
state.angular_velocity = jag.Vec3(p, q, r)
state.angular_acceleration = jag.Vec3(p_dot, q_dot, r_dot)

# Mass properties
state.mass = 12000.0  # kg
state.inertia = jag.Mat3x3.inertia(Ixx, Iyy, Izz)
state.cg_offset = jag.Vec3(0, 0, 0)

# Metadata
state.coordinate_frame = jag.CoordinateFrame.NED
state.time = 0.0
```

### Engine

Main simulation engine class.

```python
# Create engine
engine = jag.Engine()

# Lifecycle
success = engine.initialize()           # Initialize with defaults
success = engine.initialize("config.xml")  # Initialize with config
engine.shutdown()                        # Release resources

# Simulation control
engine.step(dt)                          # Advance by dt seconds
engine.run()                             # Run continuously
engine.pause()                           # Pause simulation
engine.resume()                          # Resume simulation
engine.stop()                            # Stop simulation

# State queries
state = engine.get_state()               # Get SimulationState
time = engine.get_time()                 # Get simulation time
dt = engine.get_dt()                     # Get time step

# Entity management
entity_id = engine.create_entity("name", jag.Domain.Air)
engine.remove_entity(entity_id)
entity_state = engine.get_entity_state(entity_id)
engine.set_entity_state(entity_id, state)
forces = engine.get_entity_forces(entity_id)
count = engine.entity_count()
entities = engine.get_all_entities()
```

## Constants Module

Physical constants are available in the `jag.constants` submodule:

```python
from pyjaguar import constants

# Universal constants
constants.G          # Gravitational constant (m³/kg/s²)
constants.C          # Speed of light (m/s)
constants.PI         # Pi
constants.TWO_PI     # 2*Pi
constants.HALF_PI    # Pi/2
constants.DEG_TO_RAD # Degrees to radians conversion
constants.RAD_TO_DEG # Radians to degrees conversion

# Earth parameters
constants.MU_EARTH        # Earth gravitational parameter (m³/s²)
constants.R_EARTH_EQUATOR # Earth equatorial radius (m)
constants.R_EARTH_POLAR   # Earth polar radius (m)
constants.OMEGA_EARTH     # Earth rotation rate (rad/s)
constants.J2              # Earth J2 coefficient
constants.FLATTENING      # Earth flattening

# Atmosphere
constants.G0        # Standard gravity (m/s²)
constants.RHO0      # Sea level density (kg/m³)
constants.P0        # Sea level pressure (Pa)
constants.T0        # Sea level temperature (K)
constants.GAMMA_AIR # Air specific heat ratio
constants.R_AIR     # Air gas constant (J/kg/K)
```

## Unit Conversion Functions

```python
import pyjaguar as jag

# Length conversions
meters = jag.ft_to_m(feet)
feet = jag.m_to_ft(meters)
meters = jag.nm_to_m(nautical_miles)

# Speed conversions
ms = jag.kt_to_ms(knots)
knots = jag.ms_to_kt(ms)

# Angle conversions
radians = jag.deg_to_rad(degrees)
degrees = jag.rad_to_deg(radians)
```

## Example: Complete Simulation

```python
import pyjaguar as jag
import numpy as np
import matplotlib.pyplot as plt

def run_simulation():
    # Initialize
    engine = jag.Engine()
    engine.initialize()

    # Create aircraft
    aircraft = engine.create_entity("f16", jag.Domain.Air)

    # Set initial state
    state = engine.get_entity_state(aircraft)
    state.position = jag.Vec3(0, 0, -1000)  # 1km altitude
    state.velocity = jag.Vec3(200, 0, 0)    # 200 m/s
    state.mass = 12000
    state.inertia = jag.Mat3x3.inertia(9496, 55814, 63100)
    engine.set_entity_state(aircraft, state)

    # Run simulation and collect data
    dt = 0.01
    times = []
    positions = []
    velocities = []

    while engine.get_time() < 10.0:
        engine.step(dt)

        state = engine.get_entity_state(aircraft)
        times.append(engine.get_time())
        positions.append(state.position.to_numpy())
        velocities.append(state.velocity.to_numpy())

    engine.shutdown()

    # Convert to numpy arrays
    positions = np.array(positions)
    velocities = np.array(velocities)

    # Plot results
    fig, axes = plt.subplots(2, 1, figsize=(10, 8))

    axes[0].plot(times, positions[:, 0], label='North')
    axes[0].plot(times, positions[:, 1], label='East')
    axes[0].plot(times, -positions[:, 2], label='Altitude')
    axes[0].set_xlabel('Time (s)')
    axes[0].set_ylabel('Position (m)')
    axes[0].legend()
    axes[0].grid(True)

    axes[1].plot(times, np.linalg.norm(velocities, axis=1))
    axes[1].set_xlabel('Time (s)')
    axes[1].set_ylabel('Speed (m/s)')
    axes[1].grid(True)

    plt.tight_layout()
    plt.savefig('simulation_results.png')
    plt.show()

if __name__ == "__main__":
    run_simulation()
```
