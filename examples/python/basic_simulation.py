#!/usr/bin/env python3
"""
JaguarEngine Basic Simulation Example

Demonstrates:
- Engine initialization
- Entity creation
- State manipulation
- Simulation stepping
- Property access
"""

import pyjaguar as jag
import numpy as np

def main():
    print("JaguarEngine Python Example")
    print("=" * 40)
    print(f"Version: {jag.__version__}")
    print()

    # Create and initialize the engine
    engine = jag.Engine()
    if not engine.initialize():
        print("Failed to initialize engine")
        return

    print("Engine initialized successfully")
    print()

    # ==========================================================================
    # Math Types Demo
    # ==========================================================================
    print("Math Types Demo")
    print("-" * 40)

    # Vec3 operations
    v1 = jag.Vec3(1.0, 2.0, 3.0)
    v2 = jag.Vec3(4.0, 5.0, 6.0)

    print(f"v1 = {v1}")
    print(f"v2 = {v2}")
    print(f"v1 + v2 = {v1 + v2}")
    print(f"v1 dot v2 = {v1.dot(v2)}")
    print(f"v1 cross v2 = {v1.cross(v2)}")
    print(f"|v1| = {v1.length():.4f}")
    print()

    # Quaternion operations
    q1 = jag.Quat.from_euler(0.0, 0.0, jag.deg_to_rad(45))  # 45 deg yaw
    print(f"Quaternion (45° yaw): {q1}")

    rotated = q1.rotate(jag.Vec3(1, 0, 0))
    print(f"Unit X rotated 45° yaw: ({rotated.x:.4f}, {rotated.y:.4f}, {rotated.z:.4f})")
    print()

    # Numpy interop
    v_numpy = v1.to_numpy()
    print(f"Vec3 as numpy array: {v_numpy}")
    print(f"Vec3 from numpy: {jag.Vec3.from_numpy(np.array([7.0, 8.0, 9.0]))}")
    print()

    # ==========================================================================
    # Entity Creation
    # ==========================================================================
    print("Entity Creation")
    print("-" * 40)

    # Create entities in different domains
    aircraft = engine.create_entity("f16", jag.Domain.Air)
    vehicle = engine.create_entity("m1a2", jag.Domain.Land)
    ship = engine.create_entity("destroyer", jag.Domain.Sea)
    satellite = engine.create_entity("gps01", jag.Domain.Space)

    print(f"Created aircraft: ID={aircraft}")
    print(f"Created vehicle: ID={vehicle}")
    print(f"Created ship: ID={ship}")
    print(f"Created satellite: ID={satellite}")
    print()

    # ==========================================================================
    # State Manipulation
    # ==========================================================================
    print("State Manipulation")
    print("-" * 40)

    # Get and modify aircraft state
    state = engine.get_entity_state(aircraft)
    print(f"Initial aircraft state: {state}")

    # Set initial position (1000m altitude)
    state.position = jag.Vec3(0.0, 0.0, -1000.0)  # NED: negative Z = up
    state.velocity = jag.Vec3(200.0, 0.0, 0.0)     # 200 m/s forward
    state.mass = 12000.0  # 12,000 kg
    state.inertia = jag.Mat3x3.inertia(9496, 55814, 63100)

    engine.set_entity_state(aircraft, state)
    print(f"Modified aircraft state: {engine.get_entity_state(aircraft)}")
    print()

    # ==========================================================================
    # Simulation Loop
    # ==========================================================================
    print("Running Simulation")
    print("-" * 40)

    dt = 0.01  # 100 Hz
    total_time = 1.0  # 1 second
    steps = int(total_time / dt)

    print(f"Time step: {dt} s")
    print(f"Total simulation time: {total_time} s")
    print(f"Number of steps: {steps}")
    print()

    for i in range(steps):
        engine.step(dt)

        # Print progress every 10 steps
        if i % 10 == 0:
            state = engine.get_entity_state(aircraft)
            print(f"t={engine.get_time():6.2f}s | "
                  f"pos=({state.position.x:8.1f}, {state.position.y:8.1f}, {state.position.z:8.1f})")

    print()
    print(f"Final simulation time: {engine.get_time():.3f} s")
    print()

    # ==========================================================================
    # Physical Constants
    # ==========================================================================
    print("Physical Constants")
    print("-" * 40)
    print(f"Gravitational constant G = {jag.constants.G:.4e} m³/kg/s²")
    print(f"Earth radius = {jag.constants.R_EARTH_EQUATOR:.0f} m")
    print(f"Standard gravity = {jag.constants.G0} m/s²")
    print(f"Sea level density = {jag.constants.RHO0} kg/m³")
    print(f"Pi = {jag.constants.PI}")
    print()

    # ==========================================================================
    # Unit Conversions
    # ==========================================================================
    print("Unit Conversions")
    print("-" * 40)
    print(f"100 ft = {jag.ft_to_m(100):.2f} m")
    print(f"100 m = {jag.m_to_ft(100):.2f} ft")
    print(f"100 knots = {jag.kt_to_ms(100):.2f} m/s")
    print(f"45 degrees = {jag.deg_to_rad(45):.4f} rad")
    print()

    # ==========================================================================
    # Cleanup
    # ==========================================================================
    engine.shutdown()
    print("Engine shutdown complete")


if __name__ == "__main__":
    main()
