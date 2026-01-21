# Integration

JaguarEngine uses numerical integration to propagate entity states forward in time. This guide covers integrators, time stepping, and state propagation.

## Overview

The integration process:

1. Compute forces and moments on entity
2. Calculate accelerations from Newton's laws
3. Integrate accelerations to get new velocity
4. Integrate velocity to get new position
5. Integrate angular rates to get new orientation

## Integrator Interface

```cpp
class IStatePropagator {
public:
    virtual void propagate(EntityState& state,
                           const EntityForces& forces,
                           Real dt) = 0;
};
```

## Available Integrators

### RK4 (Runge-Kutta 4th Order)

The default integrator for most applications:

```cpp
class RK4Integrator : public IStatePropagator {
public:
    void propagate(EntityState& state,
                   const EntityForces& forces,
                   Real dt) override;
};
```

**Characteristics:**
- 4th order accuracy: Error ~ O(dt⁵)
- Good balance of accuracy and performance
- Stable for typical simulation rates
- Recommended for general use

### ABM4 (Adams-Bashforth-Moulton 4th Order)

Multi-step integrator for smooth dynamics:

```cpp
class ABM4Integrator : public IStatePropagator {
public:
    void propagate(EntityState& state,
                   const EntityForces& forces,
                   Real dt) override;
};
```

**Characteristics:**
- 4th order accuracy
- More efficient for smooth systems
- Requires history (4 previous steps)
- Uses RK4 for startup

## Equations of Motion

### Translational Dynamics

```
m · a = F_total
a = F_total / m
```

**Integration:**
```cpp
Vec3 accel = forces.force / state.mass;
state.velocity += accel * dt;
state.position += state.velocity * dt;
```

### Rotational Dynamics

```
I · ω̇ = M - ω × (I · ω)
```

**Integration:**
```cpp
Vec3 omega = state.angular_velocity;
Vec3 angular_accel = state.inertia.inverse() *
    (forces.torque - omega.cross(state.inertia * omega));
state.angular_velocity += angular_accel * dt;
```

### Quaternion Integration

```
q̇ = 0.5 · q ⊗ ω_body
```

**Integration:**
```cpp
Quaternion omega_quat{0, omega.x, omega.y, omega.z};
Quaternion q_dot = state.orientation * omega_quat * 0.5;
state.orientation = (state.orientation + q_dot * dt).normalized();
```

## Time Step Selection

### Recommended Time Steps by Domain

| Domain | Typical dt | Reason |
|--------|-----------|--------|
| Air (aircraft) | 0.01 s (100 Hz) | Flight dynamics |
| Air (missile) | 0.001-0.005 s | High-g maneuvers |
| Land | 0.02 s (50 Hz) | Ground contact |
| Sea | 0.02-0.05 s | Ship dynamics |
| Space (SGP4) | Analytical | Pre-integrated |
| Space (numerical) | 10-60 s | Orbital periods |

### Time Step Guidelines

**Too Large:**
- Numerical instability
- Missed dynamics
- Energy gain/loss

**Too Small:**
- Wasted computation
- Round-off error accumulation
- Slower than real-time

### Adaptive Time Stepping

For variable-complexity scenarios:

```cpp
Real compute_adaptive_dt(const EntityState& state,
                         const EntityForces& forces,
                         Real dt_max) {
    // Estimate required time step based on dynamics
    Real accel_mag = (forces.force / state.mass).norm();
    Real omega_mag = state.angular_velocity.norm();

    // Scale based on acceleration
    Real dt_accel = (accel_mag > 0) ? 0.1 / accel_mag : dt_max;

    // Scale based on rotation rate
    Real dt_omega = (omega_mag > 0) ? 0.05 / omega_mag : dt_max;

    return std::min({dt_accel, dt_omega, dt_max});
}
```

## Using Integrators

### Basic Usage

```cpp
// Simulation loop
Real dt = 0.01;  // 100 Hz
Real end_time = 100.0;

for (Real t = 0; t < end_time; t += dt) {
    // Compute forces
    physics::EntityForces forces;
    forces.clear();
    compute_forces(state, env, forces);

    // Integrate
    engine.step(dt);
}
```

### Custom Integrator

```cpp
class SemiImplicitEuler : public IStatePropagator {
public:
    void propagate(EntityState& state,
                   const EntityForces& forces,
                   Real dt) override {
        // Update velocity first (implicit in position)
        Vec3 accel = forces.force / state.mass;
        state.velocity += accel * dt;

        // Then update position with new velocity
        state.position += state.velocity * dt;

        // Angular dynamics
        Vec3 omega = state.angular_velocity;
        Vec3 angular_accel = state.inertia.inverse() *
            (forces.torque - omega.cross(state.inertia * omega));
        state.angular_velocity += angular_accel * dt;

        // Quaternion update
        Quaternion omega_quat{0, omega.x, omega.y, omega.z};
        Quaternion q_dot = state.orientation * omega_quat * 0.5;
        state.orientation = (state.orientation + q_dot * dt).normalized();
    }
};
```

## Stability Analysis

### Energy Conservation

For conservative systems, total energy should be constant:

```cpp
Real compute_energy(const EntityState& state) {
    // Kinetic energy
    Real KE_trans = 0.5 * state.mass * state.velocity.dot(state.velocity);
    Real KE_rot = 0.5 * state.angular_velocity.dot(
        state.inertia * state.angular_velocity);

    // Potential energy (gravity)
    Real PE = state.mass * G0 * (-state.position.z);

    return KE_trans + KE_rot + PE;
}

// Monitor energy drift
Real E0 = compute_energy(initial_state);
Real E = compute_energy(current_state);
Real drift = (E - E0) / E0;
if (std::abs(drift) > 0.01) {
    // Energy drift > 1%, may need smaller time step
}
```

### CFL Condition

For wave propagation (speed of sound, etc.):

```
dt < dx / c
```

Where dx is characteristic length and c is wave speed.

## Special Cases

### Constrained Systems

For systems with constraints (e.g., ground contact):

```cpp
void apply_ground_constraint(EntityState& state, Real ground_z) {
    if (state.position.z > ground_z) {
        // Below ground - apply constraint
        state.position.z = ground_z;

        // Remove downward velocity
        if (state.velocity.z > 0) {
            state.velocity.z = 0;
        }
    }
}
```

### Discontinuities

For sudden events (impacts, staging):

```cpp
void handle_impact(EntityState& state, Real restitution) {
    // Reflect velocity
    state.velocity.z = -restitution * state.velocity.z;

    // Apply impulse
    physics::EntityForces impulse;
    // ... compute impulse ...
}
```

## Best Practices

### Initialization

1. Set valid initial state before integration
2. Ensure quaternion is normalized
3. Verify inertia tensor is positive definite

### Numerical Hygiene

1. Normalize quaternions every step
2. Check for NaN/Inf after integration
3. Monitor energy conservation

### Performance

1. Profile integration vs force computation
2. Consider multi-rate integration for multi-scale systems
3. Use appropriate precision (float vs double)

## See Also

- [Force Generators](force-generators.md) - Computing forces
- [Entities](entities.md) - Entity state management
- [API Reference](../api/physics.md) - Physics API
