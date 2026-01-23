# Python Scripting Tutorial

This tutorial demonstrates how to use JaguarEngine's Python bindings for simulation scripting, scenario automation, and rapid prototyping.

## Prerequisites

- JaguarEngine built with Python bindings (`-DJAGUAR_BUILD_PYTHON=ON`)
- Python 3.8+
- NumPy installed (`pip install numpy`)

## Installation

### Build from Source

```bash
cd JaguarEngine

# Configure with Python bindings enabled
cmake -B build -DJAGUAR_BUILD_PYTHON=ON

# Build
cmake --build build --parallel

# Install the module
pip install -e .
```

### Verify Installation

```python
import jaguar
print(f"JaguarEngine version: {jaguar.__version__}")
print(f"Domains: {list(jaguar.Domain.__members__)}")
```

---

## Tutorial 1: Hello World - Minimal Simulation

The simplest possible JaguarEngine Python program:

```python
import jaguar

# Create and initialize engine
engine = jaguar.Engine()
engine.initialize()

# Create an entity
entity = engine.create_entity("Test", jaguar.Domain.Air)

# Set initial state
state = jaguar.EntityState()
state.position = jaguar.Vec3(0, 0, -1000)  # 1 km altitude
state.velocity = jaguar.Vec3(100, 0, 0)    # 100 m/s forward
state.mass = 1000
engine.set_entity_state(entity, state)

# Run simulation
for _ in range(1000):  # 10 seconds at 100 Hz
    engine.step(0.01)

# Get final state
final = engine.get_entity_state(entity)
print(f"Final position: ({final.position.x:.1f}, {final.position.y:.1f}, {final.position.z:.1f})")

engine.shutdown()
```

---

## Tutorial 2: Aircraft Simulation

A complete aircraft simulation with aerodynamics and propulsion:

```python
import jaguar
from jaguar.domain import air

def main():
    print("F-16 Flight Simulation (Python)\n")

    # Initialize engine
    engine = jaguar.Engine()
    engine.initialize()

    # Create aircraft entity
    aircraft = engine.create_entity("F-16", jaguar.Domain.Air)

    # Configure aerodynamics
    aero = air.AerodynamicsModel()
    aero.set_reference_area(27.87)    # m²
    aero.set_reference_chord(3.45)    # m
    aero.set_reference_span(9.45)     # m

    # Configure propulsion
    propulsion = air.PropulsionModel()
    propulsion.set_max_thrust(131000.0)  # N
    propulsion.set_fuel_capacity(3200.0)  # kg
    propulsion.set_specific_fuel_consumption(2.5e-5)
    propulsion.start()
    propulsion.set_throttle(0.85)  # 85% power

    # Set initial state (10 km altitude, 250 m/s)
    state = jaguar.EntityState()
    state.position = jaguar.Vec3(0, 0, -10000)
    state.velocity = jaguar.Vec3(250, 0, 0)
    state.orientation = jaguar.Quaternion.from_euler(0, 0, 0)
    state.mass = 12000
    engine.set_entity_state(aircraft, state)

    # Print header
    print(f"{'Time(s)':>7} | {'Alt(m)':>9} | {'Speed(m/s)':>10} | {'Mach':>5} | {'Fuel(kg)':>8}")
    print("-" * 55)

    # Simulation loop
    dt = 0.01  # 100 Hz
    forces = jaguar.EntityForces()

    for i in range(6000):  # 60 seconds
        t = i * dt

        # Get current state and environment
        s = engine.get_entity_state(aircraft)
        env = engine.get_environment(aircraft)

        # Compute forces
        forces.clear()
        aero.compute_forces(s, env, dt, forces)
        propulsion.compute_forces(s, env, dt, forces)

        # Add gravity
        forces.add_force(jaguar.Vec3(0, 0, s.mass * jaguar.G0))

        # Apply and step
        engine.apply_forces(aircraft, forces)
        engine.step(dt)

        # Print telemetry every 5 seconds
        if i % 500 == 0:
            print(f"{t:7.1f} | {-s.position.z:9.0f} | {s.velocity.norm():10.1f} | "
                  f"{aero.get_mach():5.2f} | {propulsion.get_fuel_remaining():8.1f}")

    engine.shutdown()

if __name__ == "__main__":
    main()
```

---

## Tutorial 3: NumPy Integration

Leverage NumPy for efficient batch operations and data analysis:

```python
import jaguar
import numpy as np
import matplotlib.pyplot as plt

def run_trajectory_simulation():
    """Simulate a ballistic trajectory and plot results."""

    engine = jaguar.Engine()
    engine.initialize()

    projectile = engine.create_entity("Projectile", jaguar.Domain.Air)

    # Initial conditions: 45° launch at 500 m/s
    v0 = 500.0
    angle = 45.0 * jaguar.DEG_TO_RAD

    state = jaguar.EntityState()
    state.position = jaguar.Vec3(0, 0, 0)
    state.velocity = jaguar.Vec3(v0 * np.cos(angle), 0, -v0 * np.sin(angle))
    state.mass = 100.0
    engine.set_entity_state(projectile, state)

    # Data collection arrays
    times = []
    positions = []
    velocities = []

    dt = 0.01
    t = 0.0
    forces = jaguar.EntityForces()

    while True:
        s = engine.get_entity_state(projectile)
        env = engine.get_environment(projectile)

        # Store data
        times.append(t)
        positions.append(s.position.to_numpy())
        velocities.append(s.velocity.to_numpy())

        # Check if hit ground
        if s.position.z > 0 and t > 0.1:
            break

        # Apply gravity only
        forces.clear()
        forces.add_force(jaguar.Vec3(0, 0, s.mass * jaguar.G0))

        engine.apply_forces(projectile, forces)
        engine.step(dt)
        t += dt

        if t > 120:  # Safety limit
            break

    engine.shutdown()

    # Convert to NumPy arrays
    times = np.array(times)
    positions = np.array(positions)
    velocities = np.array(velocities)

    # Analysis
    max_height = -np.min(positions[:, 2])
    range_dist = positions[-1, 0]
    flight_time = times[-1]

    print(f"Trajectory Analysis:")
    print(f"  Max height: {max_height:.1f} m")
    print(f"  Range: {range_dist:.1f} m")
    print(f"  Flight time: {flight_time:.2f} s")

    # Plot trajectory
    plt.figure(figsize=(12, 5))

    plt.subplot(1, 2, 1)
    plt.plot(positions[:, 0], -positions[:, 2])
    plt.xlabel('Distance (m)')
    plt.ylabel('Height (m)')
    plt.title('Ballistic Trajectory')
    plt.grid(True)

    plt.subplot(1, 2, 2)
    speed = np.linalg.norm(velocities, axis=1)
    plt.plot(times, speed)
    plt.xlabel('Time (s)')
    plt.ylabel('Speed (m/s)')
    plt.title('Speed vs Time')
    plt.grid(True)

    plt.tight_layout()
    plt.savefig('trajectory.png')
    plt.show()

if __name__ == "__main__":
    run_trajectory_simulation()
```

---

## Tutorial 4: Multi-Entity Scenario

Simulate multiple entities interacting in the same environment:

```python
import jaguar
import numpy as np

def create_formation(engine, base_name, domain, count, spacing):
    """Create a formation of entities."""
    entities = []
    for i in range(count):
        name = f"{base_name}-{i+1}"
        entity = engine.create_entity(name, domain)
        entities.append(entity)
    return entities

def main():
    print("Multi-Domain Scenario Simulation\n")

    engine = jaguar.Engine()
    engine.initialize()

    # Create blue force (4 aircraft)
    blue_aircraft = create_formation(engine, "Blue", jaguar.Domain.Air, 4, 500)

    # Create red force (4 aircraft)
    red_aircraft = create_formation(engine, "Red", jaguar.Domain.Air, 4, 500)

    # Set blue force initial states (from west)
    for i, aircraft in enumerate(blue_aircraft):
        state = jaguar.EntityState()
        state.position = jaguar.Vec3(0, i * 500, -8000)
        state.velocity = jaguar.Vec3(250, 0, 0)  # Heading east
        state.mass = 12000
        engine.set_entity_state(aircraft, state)

    # Set red force initial states (from east)
    for i, aircraft in enumerate(red_aircraft):
        state = jaguar.EntityState()
        state.position = jaguar.Vec3(50000, i * 500, -8000)
        state.velocity = jaguar.Vec3(-250, 0, 0)  # Heading west
        state.mass = 12000
        engine.set_entity_state(aircraft, state)

    print(f"Created {len(blue_aircraft)} Blue aircraft and {len(red_aircraft)} Red aircraft")
    print("Starting simulation...\n")

    # Simulation loop
    dt = 0.01
    duration = 120.0  # 2 minutes

    for t in np.arange(0, duration, dt):
        engine.step(dt)

        # Log every 10 seconds
        if abs(t % 10.0) < dt:
            print(f"T = {t:.0f}s")

            # Calculate closest approach
            min_dist = float('inf')
            for blue in blue_aircraft:
                blue_state = engine.get_entity_state(blue)
                for red in red_aircraft:
                    red_state = engine.get_entity_state(red)
                    dist = (blue_state.position - red_state.position).norm()
                    min_dist = min(min_dist, dist)

            print(f"  Closest blue-red distance: {min_dist:.0f} m")

    print("\nSimulation complete!")
    engine.shutdown()

if __name__ == "__main__":
    main()
```

---

## Tutorial 5: Custom Scenario Script

Create a reusable scenario framework:

```python
import jaguar
from dataclasses import dataclass
from typing import List, Dict, Optional
import json

@dataclass
class EntityConfig:
    name: str
    domain: str
    position: List[float]
    velocity: List[float]
    mass: float

@dataclass
class ScenarioConfig:
    name: str
    duration: float
    time_step: float
    entities: List[EntityConfig]

def load_scenario(filepath: str) -> ScenarioConfig:
    """Load scenario from JSON file."""
    with open(filepath, 'r') as f:
        data = json.load(f)

    entities = [EntityConfig(**e) for e in data['entities']]
    return ScenarioConfig(
        name=data['name'],
        duration=data['duration'],
        time_step=data['time_step'],
        entities=entities
    )

def domain_from_string(domain_str: str) -> jaguar.Domain:
    """Convert string to Domain enum."""
    domains = {
        'air': jaguar.Domain.Air,
        'land': jaguar.Domain.Land,
        'sea': jaguar.Domain.Sea,
        'space': jaguar.Domain.Space,
    }
    return domains.get(domain_str.lower(), jaguar.Domain.Generic)

class Scenario:
    def __init__(self, config: ScenarioConfig):
        self.config = config
        self.engine = jaguar.Engine()
        self.entities: Dict[str, int] = {}
        self.time = 0.0
        self.data: Dict[str, List] = {}

    def setup(self):
        """Initialize scenario."""
        self.engine.initialize()

        for entity_cfg in self.config.entities:
            # Create entity
            domain = domain_from_string(entity_cfg.domain)
            entity_id = self.engine.create_entity(entity_cfg.name, domain)
            self.entities[entity_cfg.name] = entity_id

            # Set initial state
            state = jaguar.EntityState()
            state.position = jaguar.Vec3(*entity_cfg.position)
            state.velocity = jaguar.Vec3(*entity_cfg.velocity)
            state.mass = entity_cfg.mass
            self.engine.set_entity_state(entity_id, state)

            # Initialize data collection
            self.data[entity_cfg.name] = []

        print(f"Scenario '{self.config.name}' initialized with {len(self.entities)} entities")

    def step(self):
        """Advance simulation by one time step."""
        self.engine.step(self.config.time_step)
        self.time += self.config.time_step

        # Collect data
        for name, entity_id in self.entities.items():
            state = self.engine.get_entity_state(entity_id)
            self.data[name].append({
                'time': self.time,
                'position': state.position.to_numpy().tolist(),
                'velocity': state.velocity.to_numpy().tolist(),
            })

    def run(self, callback=None):
        """Run complete scenario."""
        steps = int(self.config.duration / self.config.time_step)

        for i in range(steps):
            self.step()

            if callback:
                callback(self, i, steps)

        print(f"Scenario completed at T = {self.time:.2f}s")

    def cleanup(self):
        """Shutdown engine."""
        self.engine.shutdown()

    def export_results(self, filepath: str):
        """Export collected data to JSON."""
        results = {
            'scenario': self.config.name,
            'duration': self.config.duration,
            'time_step': self.config.time_step,
            'entities': self.data
        }

        with open(filepath, 'w') as f:
            json.dump(results, f, indent=2)

        print(f"Results exported to {filepath}")

# Example usage
if __name__ == "__main__":
    # Create scenario configuration programmatically
    config = ScenarioConfig(
        name="Intercept Scenario",
        duration=60.0,
        time_step=0.01,
        entities=[
            EntityConfig("Target", "air", [0, 0, -8000], [200, 0, 0], 10000),
            EntityConfig("Interceptor", "air", [30000, 5000, -10000], [-300, -50, 20], 15000),
        ]
    )

    # Run scenario
    scenario = Scenario(config)
    scenario.setup()

    def progress_callback(sc, step, total):
        if step % 1000 == 0:
            print(f"Progress: {100 * step / total:.0f}%")

    scenario.run(callback=progress_callback)
    scenario.export_results("intercept_results.json")
    scenario.cleanup()
```

---

## Tutorial 6: Coordinate Transforms

Working with geodetic coordinates and reference frames:

```python
import jaguar
from jaguar import transforms
import numpy as np

def geodetic_to_simulation():
    """Convert geodetic (lat/lon/alt) to simulation coordinates."""

    # Example locations
    locations = [
        ("New York", 40.7128, -74.0060, 0),
        ("London", 51.5074, -0.1278, 0),
        ("Tokyo", 35.6762, 139.6503, 0),
    ]

    print("Geodetic to ECEF Conversion:")
    print(f"{'Location':<12} | {'Lat':>10} | {'Lon':>10} | {'ECEF X (km)':>12} | {'ECEF Y (km)':>12} | {'ECEF Z (km)':>12}")
    print("-" * 80)

    for name, lat, lon, alt in locations:
        lat_rad = lat * jaguar.DEG_TO_RAD
        lon_rad = lon * jaguar.DEG_TO_RAD

        ecef = transforms.geodetic_to_ecef(lat_rad, lon_rad, alt)

        print(f"{name:<12} | {lat:>10.4f} | {lon:>10.4f} | {ecef.x/1000:>12.1f} | {ecef.y/1000:>12.1f} | {ecef.z/1000:>12.1f}")

def aircraft_at_location():
    """Simulate aircraft at a specific geodetic location."""

    engine = jaguar.Engine()
    engine.initialize()

    aircraft = engine.create_entity("Aircraft", jaguar.Domain.Air)

    # Aircraft over San Francisco at 10 km altitude
    lat = 37.7749 * jaguar.DEG_TO_RAD
    lon = -122.4194 * jaguar.DEG_TO_RAD
    alt = 10000  # meters

    # Convert to ECEF
    position = transforms.geodetic_to_ecef(lat, lon, alt)

    # Velocity: 250 m/s heading east (need to compute local east direction)
    # Simplified: assume local east is roughly +Y in ECEF at this longitude
    speed = 250.0
    velocity = jaguar.Vec3(
        -speed * np.sin(lon),  # East component
        speed * np.cos(lon),
        0
    )

    state = jaguar.EntityState()
    state.position = position
    state.velocity = velocity
    state.mass = 12000
    engine.set_entity_state(aircraft, state)

    print(f"\nAircraft initialized over San Francisco:")
    print(f"  Position (ECEF): ({position.x:.0f}, {position.y:.0f}, {position.z:.0f}) m")
    print(f"  Altitude: {alt} m")

    # Simulate for 60 seconds
    dt = 0.01
    for _ in range(6000):
        engine.step(dt)

    # Get final position and convert back to geodetic
    final_state = engine.get_entity_state(aircraft)
    final_lat, final_lon, final_alt = transforms.ecef_to_geodetic(final_state.position)

    print(f"\nAfter 60 seconds:")
    print(f"  Latitude: {final_lat * jaguar.RAD_TO_DEG:.4f}°")
    print(f"  Longitude: {final_lon * jaguar.RAD_TO_DEG:.4f}°")
    print(f"  Altitude: {final_alt:.0f} m")

    engine.shutdown()

if __name__ == "__main__":
    geodetic_to_simulation()
    aircraft_at_location()
```

---

## Common Patterns

### Pattern: Property Access

```python
# Get/set engine properties
time_scale = engine.get_property("time.scale")
engine.set_property("time.scale", 2.0)  # 2x speed

# Entity-specific properties
throttle = engine.get_property(aircraft, "throttle")
engine.set_property(aircraft, "throttle", 0.9)
```

### Pattern: Event Handling

```python
# Query simulation state
if engine.get_state() == jaguar.SimulationState.Running:
    engine.pause()

# Check entity existence
if engine.entity_exists(entity_id):
    state = engine.get_entity_state(entity_id)
```

### Pattern: Error Handling

```python
try:
    engine = jaguar.Engine()
    if not engine.initialize():
        raise RuntimeError("Engine initialization failed")

    # ... simulation code ...

except Exception as e:
    print(f"Simulation error: {e}")
finally:
    engine.shutdown()
```

---

## Debugging Tips

### Check Entity State

```python
def debug_entity(engine, entity_id, name="Entity"):
    """Print detailed entity state for debugging."""
    state = engine.get_entity_state(entity_id)
    env = engine.get_environment(entity_id)

    print(f"\n=== {name} Debug Info ===")
    print(f"Position: ({state.position.x:.2f}, {state.position.y:.2f}, {state.position.z:.2f})")
    print(f"Velocity: ({state.velocity.x:.2f}, {state.velocity.y:.2f}, {state.velocity.z:.2f})")
    print(f"Speed: {state.velocity.norm():.2f} m/s")
    print(f"Mass: {state.mass:.1f} kg")
    print(f"Altitude: {env.altitude:.1f} m")
    print(f"Atmosphere: T={env.atmosphere.temperature:.1f}K, ρ={env.atmosphere.density:.4f} kg/m³")
```

### Performance Profiling

```python
import time

def profile_simulation(engine, steps, dt):
    """Measure simulation performance."""
    start = time.perf_counter()

    for _ in range(steps):
        engine.step(dt)

    elapsed = time.perf_counter() - start
    sim_time = steps * dt

    print(f"Simulated {sim_time:.1f}s in {elapsed:.3f}s real time")
    print(f"Real-time factor: {sim_time / elapsed:.1f}x")
    print(f"Steps/second: {steps / elapsed:.0f}")
```

---

## Next Steps

- [Lua Scripting Tutorial](lua-scripting.md) - Lua scripting guide
- [Examples Guide](examples.md) - More example code
- [Python API Reference](../api/python.md) - Complete API documentation
- [Air Domain Tutorial](air-domain.md) - Aircraft simulation deep dive
