#!/usr/bin/env python3
"""
Flight Simulation Example using JaguarEngine Python Bindings

This example demonstrates:
1. Creating an engine and initializing simulation
2. Creating aircraft entities with different domains
3. Setting initial states (position, velocity, orientation)
4. Running the simulation and monitoring state changes
5. Using the event system for collision and threshold monitoring
6. NumPy integration for efficient data handling

Requirements:
    pip install numpy matplotlib

Build JaguarEngine with Python bindings:
    cmake -B build -DJAGUAR_BUILD_PYTHON=ON
    cmake --build build
"""

import sys
import math
from pathlib import Path

# Add build directory to path for development
build_path = Path(__file__).parent.parent.parent / "build"
if build_path.exists():
    sys.path.insert(0, str(build_path))

try:
    import pyjaguar as jag
    import numpy as np
    HAS_MATPLOTLIB = False
    try:
        import matplotlib.pyplot as plt
        HAS_MATPLOTLIB = True
    except ImportError:
        print("matplotlib not available, skipping plots")
except ImportError as e:
    print(f"Failed to import pyjaguar: {e}")
    print("Build with: cmake -B build -DJAGUAR_BUILD_PYTHON=ON && cmake --build build")
    sys.exit(1)


def main():
    """Run a basic flight simulation demonstration."""
    print("=" * 60)
    print("JaguarEngine Python Flight Simulation Example")
    print(f"Version: {jag.__version__}")
    print("=" * 60)

    # ========================================================================
    # 1. Create and Initialize Engine
    # ========================================================================
    print("\n[1] Creating and initializing engine...")

    engine = jag.Engine()
    if not engine.initialize():
        print("Failed to initialize engine!")
        return

    print(f"    Engine state: {engine.get_state()}")
    print(f"    Initial time: {engine.get_time():.3f} s")

    # ========================================================================
    # 2. Create Aircraft Entity
    # ========================================================================
    print("\n[2] Creating aircraft entity...")

    aircraft_id = engine.create_entity("F16_Alpha", jag.Domain.Air)
    if aircraft_id == jag.INVALID_ENTITY_ID:
        print("Failed to create aircraft!")
        return

    print(f"    Created aircraft with ID: {aircraft_id}")

    # ========================================================================
    # 3. Set Initial State
    # ========================================================================
    print("\n[3] Setting initial state...")

    initial_state = jag.EntityState()

    # Position: 10km altitude, at origin
    initial_state.position = jag.Vec3(0.0, 0.0, -10000.0)  # NED: down is negative

    # Velocity: 200 m/s forward (approximately Mach 0.6)
    initial_state.velocity = jag.Vec3(200.0, 0.0, 0.0)

    # Orientation: Level flight, heading north
    initial_state.orientation = jag.Quat.identity()

    # Mass: typical F-16 empty weight
    initial_state.mass = 8500.0  # kg

    # Inertia tensor (simplified)
    initial_state.inertia = jag.Mat3x3.inertia(
        ixx=12000.0,  # Roll
        iyy=75000.0,  # Pitch
        izz=85000.0   # Yaw
    )

    engine.set_entity_state(aircraft_id, initial_state)

    state = engine.get_entity_state(aircraft_id)
    print(f"    Position: ({state.position.x:.1f}, {state.position.y:.1f}, {state.position.z:.1f}) m")
    print(f"    Velocity: ({state.velocity.x:.1f}, {state.velocity.y:.1f}, {state.velocity.z:.1f}) m/s")
    print(f"    Mass: {state.mass:.1f} kg")

    # ========================================================================
    # 4. Set Up Event Monitoring (Optional)
    # ========================================================================
    print("\n[4] Setting up event monitoring...")

    # Event callback function
    event_log = []

    def on_event(event_data):
        """Handle all events."""
        event_log.append({
            'time': event_data.timestamp,
            'type': event_data.type,
            'source': event_data.source_entity
        })
        return True  # Continue propagation

    # Create event dispatcher (if available in engine)
    try:
        dispatcher = jag.EventDispatcher()
        handler_id = dispatcher.subscribe_all(on_event, "flight_monitor")
        print(f"    Event handler registered: {handler_id}")
    except Exception as e:
        print(f"    Event system not available: {e}")
        dispatcher = None

    # ========================================================================
    # 5. Run Simulation
    # ========================================================================
    print("\n[5] Running simulation...")

    dt = 0.01  # 10ms time step (100 Hz)
    sim_duration = 10.0  # 10 seconds
    num_steps = int(sim_duration / dt)

    # Data recording
    times = []
    positions_x = []
    positions_z = []
    velocities = []

    print(f"    Time step: {dt * 1000:.1f} ms")
    print(f"    Duration: {sim_duration:.1f} s")
    print(f"    Total steps: {num_steps}")
    print()

    for i in range(num_steps):
        # Step simulation
        engine.step(dt)

        # Record state every 100 steps (1 second intervals)
        if i % 100 == 0:
            state = engine.get_entity_state(aircraft_id)
            t = engine.get_time()

            times.append(t)
            positions_x.append(state.position.x)
            positions_z.append(-state.position.z)  # Convert to altitude
            velocities.append(state.velocity.length())

            # Print progress
            print(f"    t={t:6.2f}s | "
                  f"x={state.position.x:10.1f}m | "
                  f"alt={-state.position.z:8.1f}m | "
                  f"v={state.velocity.length():6.1f}m/s")

    # ========================================================================
    # 6. Display Results
    # ========================================================================
    print("\n[6] Final state:")

    final_state = engine.get_entity_state(aircraft_id)
    print(f"    Position: ({final_state.position.x:.1f}, {final_state.position.y:.1f}, {final_state.position.z:.1f}) m")
    print(f"    Velocity: ({final_state.velocity.x:.1f}, {final_state.velocity.y:.1f}, {final_state.velocity.z:.1f}) m/s")
    print(f"    Distance traveled: {final_state.position.x:.1f} m")
    print(f"    Simulation time: {engine.get_time():.3f} s")

    if event_log:
        print(f"\n    Events logged: {len(event_log)}")

    # ========================================================================
    # 7. Plot Results (if matplotlib available)
    # ========================================================================
    if HAS_MATPLOTLIB and len(times) > 1:
        print("\n[7] Generating plots...")

        fig, axes = plt.subplots(2, 2, figsize=(12, 8))
        fig.suptitle("JaguarEngine Flight Simulation Results", fontsize=14)

        # Position over time
        axes[0, 0].plot(times, np.array(positions_x) / 1000, 'b-')
        axes[0, 0].set_xlabel('Time (s)')
        axes[0, 0].set_ylabel('Distance (km)')
        axes[0, 0].set_title('Horizontal Distance')
        axes[0, 0].grid(True)

        # Altitude over time
        axes[0, 1].plot(times, np.array(positions_z) / 1000, 'g-')
        axes[0, 1].set_xlabel('Time (s)')
        axes[0, 1].set_ylabel('Altitude (km)')
        axes[0, 1].set_title('Altitude')
        axes[0, 1].grid(True)

        # Velocity over time
        axes[1, 0].plot(times, velocities, 'r-')
        axes[1, 0].set_xlabel('Time (s)')
        axes[1, 0].set_ylabel('Speed (m/s)')
        axes[1, 0].set_title('Speed')
        axes[1, 0].grid(True)

        # Trajectory (x vs altitude)
        axes[1, 1].plot(np.array(positions_x) / 1000, np.array(positions_z) / 1000, 'k-')
        axes[1, 1].set_xlabel('Distance (km)')
        axes[1, 1].set_ylabel('Altitude (km)')
        axes[1, 1].set_title('Trajectory')
        axes[1, 1].grid(True)

        plt.tight_layout()
        plt.savefig('flight_simulation_results.png', dpi=150)
        print("    Saved: flight_simulation_results.png")
        plt.show()

    # ========================================================================
    # 8. Cleanup
    # ========================================================================
    print("\n[8] Shutting down...")

    engine.destroy_entity(aircraft_id)
    engine.shutdown()

    print("    Done!")
    print("=" * 60)


def demo_numpy_integration():
    """Demonstrate NumPy integration for batch operations."""
    print("\n" + "=" * 60)
    print("NumPy Integration Demo")
    print("=" * 60)

    # Vec3 <-> NumPy conversion
    v = jag.Vec3(1.0, 2.0, 3.0)
    arr = v.to_numpy()
    print(f"\nVec3 to NumPy: {v} -> {arr}")

    # NumPy operations
    arr_normalized = arr / np.linalg.norm(arr)
    v_normalized = jag.Vec3.from_numpy(arr_normalized)
    print(f"Normalized: {v_normalized}")

    # Mat3x3 <-> NumPy conversion
    m = jag.Mat3x3.diagonal(1.0, 2.0, 3.0)
    m_arr = m.to_numpy()
    print(f"\nMat3x3 to NumPy:\n{m_arr}")

    # NumPy matrix operations
    m_inv = np.linalg.inv(m_arr)
    print(f"Inverse:\n{m_inv}")

    # Quat <-> NumPy conversion
    q = jag.Quat.from_euler(0.1, 0.2, 0.3)
    q_arr = q.to_numpy()
    print(f"\nQuat to NumPy (w,x,y,z): {q_arr}")


def demo_threshold_monitoring():
    """Demonstrate threshold monitoring for parameter tracking."""
    print("\n" + "=" * 60)
    print("Threshold Monitoring Demo")
    print("=" * 60)

    # Create threshold monitor
    monitor = jag.ThresholdMonitor()

    # Configure speed threshold
    speed_config = jag.speed_threshold(max_speed=340.0, hysteresis=10.0)  # Mach 1
    monitor.add_threshold("speed", speed_config)

    # Configure altitude threshold
    alt_config = jag.altitude_threshold(max_altitude=15000.0, min_altitude=500.0)
    monitor.add_threshold("altitude", alt_config)

    # Configure fuel threshold
    fuel_config = jag.fuel_threshold(low_warning=20.0, critical_low=5.0)
    monitor.add_threshold("fuel", fuel_config)

    print(f"\nConfigured {monitor.threshold_count()} thresholds")

    # Simulate parameter changes
    test_values = [
        ("speed", 300.0, "Normal speed"),
        ("speed", 350.0, "Overspeed!"),
        ("speed", 320.0, "Still overspeed (hysteresis)"),
        ("speed", 325.0, "Recovered to normal"),
        ("altitude", 10000.0, "Normal altitude"),
        ("altitude", 16000.0, "Altitude exceeded!"),
        ("fuel", 50.0, "Fuel OK"),
        ("fuel", 15.0, "Fuel low warning!"),
        ("fuel", 3.0, "Fuel critical!"),
    ]

    print("\nSimulating parameter changes:")
    for param, value, description in test_values:
        monitor.update(param, value)
        state = monitor.get_state(param)
        print(f"  {param}={value:6.1f} -> {state.name:15s} | {description}")

    # Check exceeded thresholds
    exceeded = monitor.get_exceeded_thresholds()
    if exceeded:
        print(f"\nExceeded thresholds: {exceeded}")

    print()


if __name__ == "__main__":
    main()
    demo_numpy_integration()
    demo_threshold_monitoring()
