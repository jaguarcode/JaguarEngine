# Python API Reference

Python bindings for JaguarEngine.

## Installation

```bash
pip install jaguar-engine
```

Or build from source:

```bash
cd JaguarEngine
pip install .
```

## Quick Start

```python
import jaguar

# Create and initialize engine
engine = jaguar.Engine()
engine.initialize()

# Create an aircraft
aircraft = engine.create_entity("F-16", jaguar.Domain.Air)

# Set initial state
state = jaguar.EntityState()
state.position = jaguar.Vec3(0, 0, -5000)  # 5 km altitude
state.velocity = jaguar.Vec3(200, 0, 0)    # 200 m/s
state.mass = 12000
engine.set_entity_state(aircraft, state)

# Simulation loop
dt = 0.01
for t in range(10000):  # 100 seconds
    engine.step(dt)

    # Get current state
    state = engine.get_entity_state(aircraft)
    if t % 100 == 0:
        print(f"Time: {t*dt:.1f}s, Alt: {-state.position.z:.0f}m")

engine.shutdown()
```

## Core Types

### Vec3

```python
# Construction
v = jaguar.Vec3()           # (0, 0, 0)
v = jaguar.Vec3(1, 2, 3)    # (1, 2, 3)
v = jaguar.Vec3([1, 2, 3])  # From list

# Properties
x, y, z = v.x, v.y, v.z

# Methods
length = v.norm()
unit = v.normalized()
dot = v.dot(other)
cross = v.cross(other)

# Operators
v3 = v1 + v2
v3 = v1 - v2
v3 = v * 2.0
v3 = v / 2.0

# NumPy interop
import numpy as np
arr = np.array(v)       # To NumPy
v = jaguar.Vec3(arr)    # From NumPy
```

### Quaternion

```python
# Construction
q = jaguar.Quaternion()                    # Identity
q = jaguar.Quaternion(w, x, y, z)
q = jaguar.Quaternion.from_euler(roll, pitch, yaw)
q = jaguar.Quaternion.from_axis_angle(axis, angle)

# Methods
q_norm = q.normalized()
q_conj = q.conjugate()
v_rot = q.rotate(v)
roll, pitch, yaw = q.to_euler()

# Operators
q3 = q1 * q2  # Rotation composition
```

### Mat3x3

```python
# Construction
m = jaguar.Mat3x3()                     # Identity
m = jaguar.Mat3x3.from_euler(r, p, y)

# Access
value = m[i, j]
m[i, j] = value

# Methods
v_out = m * v_in
m_out = m1 * m2
m_t = m.transpose()

# NumPy interop
import numpy as np
arr = np.array(m)        # 3x3 NumPy array
m = jaguar.Mat3x3(arr)   # From NumPy
```

## Engine

```python
class Engine:
    def initialize(self, config: EngineConfig = None) -> bool
    def shutdown(self) -> None

    # Entity management
    def create_entity(self, name: str, domain: Domain) -> int
    def destroy_entity(self, entity_id: int) -> None
    def entity_exists(self, entity_id: int) -> bool

    # State access
    def get_entity_state(self, entity_id: int) -> EntityState
    def set_entity_state(self, entity_id: int, state: EntityState) -> None
    def apply_forces(self, entity_id: int, forces: EntityForces) -> None

    # Environment
    def get_environment(self, entity_id: int) -> Environment
    def get_environment_at(self, position: Vec3) -> Environment

    # Simulation
    def step(self, dt: float) -> None
    def run_for(self, duration: float) -> None

    # Time
    def get_time(self) -> float
    def set_time(self, time: float) -> None

    # Properties
    def entity_count(self) -> int
```

### Example

```python
import jaguar

# Configure engine
config = jaguar.EngineConfig()
config.time_step = 0.01
config.terrain_enabled = True
config.terrain_data_paths = ["/data/dted/"]

# Initialize
engine = jaguar.Engine()
if not engine.initialize(config):
    raise RuntimeError("Failed to initialize engine")

try:
    # Create entities
    aircraft = engine.create_entity("Aircraft", jaguar.Domain.Air)

    # Run simulation
    while engine.get_time() < 60.0:
        engine.step(0.01)

finally:
    engine.shutdown()
```

## EntityState

```python
class EntityState:
    position: Vec3          # m (ECEF)
    velocity: Vec3          # m/s
    orientation: Quaternion
    angular_velocity: Vec3  # rad/s
    mass: float            # kg
    inertia: Mat3x3        # kg·m²

    # Derived properties
    def get_roll(self) -> float    # rad
    def get_pitch(self) -> float   # rad
    def get_yaw(self) -> float     # rad
    def get_euler(self) -> tuple   # (roll, pitch, yaw)
```

### Example

```python
state = jaguar.EntityState()

# Set position (geodetic to ECEF)
lat, lon, alt = 37.0, -122.0, 10000.0
state.position = jaguar.transforms.geodetic_to_ecef(
    lat * jaguar.DEG_TO_RAD,
    lon * jaguar.DEG_TO_RAD,
    alt
)

# Set velocity
state.velocity = jaguar.Vec3(200, 0, 0)

# Set orientation
state.orientation = jaguar.Quaternion.from_euler(
    0.0,                    # roll
    5.0 * jaguar.DEG_TO_RAD, # pitch
    90.0 * jaguar.DEG_TO_RAD # heading (east)
)

# Set mass properties
state.mass = 15000.0
state.inertia = jaguar.Mat3x3()
state.inertia[0, 0] = 20000   # Ixx
state.inertia[1, 1] = 100000  # Iyy
state.inertia[2, 2] = 110000  # Izz

engine.set_entity_state(entity_id, state)
```

## EntityForces

```python
class EntityForces:
    force: Vec3    # N
    torque: Vec3   # N·m

    def clear(self) -> None
    def add_force(self, f: Vec3) -> None
    def add_torque(self, t: Vec3) -> None
    def add_force_at_point(self, f: Vec3, point: Vec3, cg: Vec3) -> None
```

### Example

```python
forces = jaguar.EntityForces()
forces.clear()

# Gravity
gravity = jaguar.Vec3(0, 0, state.mass * jaguar.G0)
forces.add_force(gravity)

# Thrust (in body frame, convert to world)
thrust_body = jaguar.Vec3(50000, 0, 0)
thrust_world = state.orientation.rotate(thrust_body)
forces.add_force(thrust_world)

engine.apply_forces(entity_id, forces)
```

## Environment

```python
class Environment:
    latitude: float        # rad
    longitude: float       # rad
    altitude: float        # m
    position_ecef: Vec3

    atmosphere: AtmosphereState
    terrain: TerrainQuery
    terrain_elevation: float
    ocean: OceanState
    over_water: bool
    gravity: Vec3

class AtmosphereState:
    temperature: float     # K
    pressure: float        # Pa
    density: float         # kg/m³
    speed_of_sound: float  # m/s
    wind: Vec3             # m/s (NED)

class TerrainQuery:
    elevation: float       # m
    normal: Vec3
    slope_angle: float     # rad
    valid: bool
```

### Example

```python
env = engine.get_environment(aircraft)

print(f"Altitude: {env.altitude:.0f} m")
print(f"Temperature: {env.atmosphere.temperature:.1f} K")
print(f"Density: {env.atmosphere.density:.4f} kg/m³")
print(f"Terrain elevation: {env.terrain_elevation:.0f} m")

# Height above terrain
hat = env.altitude - env.terrain_elevation
print(f"Height above terrain: {hat:.0f} m")
```

## Domain Models

### Air Domain

```python
from jaguar.domain import air

# Aerodynamics
aero = air.AerodynamicsModel()
aero.set_reference_area(28.0)
aero.set_reference_chord(3.5)
aero.set_reference_span(10.0)

aero.set_elevator(elevator_deg)
aero.compute_forces(state, env, dt, forces)

print(f"CL: {aero.get_cl():.3f}")
print(f"Alpha: {aero.get_alpha() * jaguar.RAD_TO_DEG:.1f} deg")

# Propulsion
engine_model = air.PropulsionModel()
engine_model.set_max_thrust(75000)
engine_model.set_fuel_capacity(3000)
engine_model.set_throttle(0.8)
engine_model.compute_forces(state, env, dt, forces)
```

### Land Domain

```python
from jaguar.domain import land

# Soil
soil = land.SoilProperties.DrySand()

# Terramechanics
terra = land.TerramechanicsModel()
terra.set_contact_area(0.63, 4.6)
terra.set_vehicle_weight(62000 * jaguar.G0)
terra.set_soil(soil)
terra.compute_forces(state, env, dt, forces)

print(f"Sinkage: {terra.get_sinkage():.3f} m")
```

### Sea Domain

```python
from jaguar.domain import sea

# Sea state
sea_state = sea.SeaState.FromNATOSeaState(4)

# Buoyancy
buoyancy = sea.BuoyancyModel()
buoyancy.set_displaced_volume(8390)
buoyancy.set_metacentric_height(2.5)
buoyancy.compute_forces(state, env, dt, forces)

print(f"Draft: {buoyancy.get_draft():.2f} m")
print(f"Heel: {buoyancy.get_heel() * jaguar.RAD_TO_DEG:.1f} deg")
```

### Space Domain

```python
from jaguar.domain import space

# Orbital elements
orbit = space.OrbitalElements()
orbit.semi_major_axis = jaguar.EARTH_RADIUS + 420000
orbit.eccentricity = 0.0001
orbit.inclination = 51.6 * jaguar.DEG_TO_RAD

pos, vel = orbit.to_cartesian()
print(f"Period: {orbit.period() / 60:.1f} minutes")

# SGP4 propagator
tle = space.TLE()
tle.parse(line1, line2)

sgp4 = space.SGP4Propagator()
sgp4.initialize(tle)

for minutes in range(0, 24*60, 10):
    pos_km, vel_kms = sgp4.propagate(minutes)
    pos_m = pos_km * 1000
```

## Coordinate Transforms

```python
from jaguar import transforms

# Geodetic <-> ECEF
ecef = transforms.geodetic_to_ecef(lat_rad, lon_rad, alt_m)
lat, lon, alt = transforms.ecef_to_geodetic(ecef)

# ECEF <-> ECI
eci = transforms.ecef_to_eci(ecef, julian_date)
ecef = transforms.eci_to_ecef(eci, julian_date)

# Julian date
jd = transforms.julian_date(2024, 1, 1, 12, 0, 0.0)
```

## Constants

```python
import jaguar

# Mathematical
jaguar.PI
jaguar.TWO_PI
jaguar.DEG_TO_RAD
jaguar.RAD_TO_DEG

# Physical
jaguar.G0              # Standard gravity (9.80665 m/s²)
jaguar.EARTH_RADIUS    # WGS84 equatorial radius
jaguar.EARTH_MU        # Earth gravitational parameter

# Enumerations
jaguar.Domain.Air
jaguar.Domain.Land
jaguar.Domain.Sea
jaguar.Domain.Space
```

## See Also

- [API Overview](overview.md) - Complete API index
- [Lua API](lua.md) - Lua bindings
- [Getting Started](../getting-started/quickstart.md) - Quick start guide

