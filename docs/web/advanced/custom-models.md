# Custom Physics Models

This guide explains how to create custom physics models and extend JaguarEngine's capabilities.

## Overview

JaguarEngine's component-based architecture allows you to create custom force generators, integrators, and domain-specific models by implementing standard interfaces.

## Force Generator Interface

All physics models implement the `IForceGenerator` interface:

```cpp
class IForceGenerator {
public:
    virtual void compute_forces(const EntityState& state,
                                const environment::Environment& env,
                                Real dt,
                                EntityForces& forces) = 0;
};
```

## Creating a Custom Aerodynamics Model

### Step 1: Implement the Interface

```cpp
#include <jaguar/physics/force.h>

class CustomAeroModel : public physics::IAerodynamicsModel {
public:
    // Configuration
    void set_reference_area(Real area) { ref_area_ = area; }
    void set_reference_chord(Real chord) { ref_chord_ = chord; }
    void set_reference_span(Real span) { ref_span_ = span; }

    // IForceGenerator implementation
    void compute_forces(const physics::EntityState& state,
                        const environment::Environment& env,
                        Real dt,
                        physics::EntityForces& forces) override {
        // Compute aerodynamic angles
        Vec3 vel_body = transform_to_body(state.velocity, state.orientation);
        Real V = vel_body.norm();

        if (V < 1.0) return;  // Minimum airspeed threshold

        alpha_ = std::atan2(vel_body.z, vel_body.x);
        beta_ = std::asin(vel_body.y / V);
        mach_ = V / env.atmosphere.speed_of_sound;
        qbar_ = 0.5 * env.atmosphere.density * V * V;

        // Compute coefficients
        compute_coefficients();

        // Apply forces
        Real L = qbar_ * ref_area_ * cl_;
        Real D = qbar_ * ref_area_ * cd_;
        Real M = qbar_ * ref_area_ * ref_chord_ * cm_;

        // Transform to body frame and add to forces
        forces.add_force(transform_aero_forces(L, D, alpha_));
        forces.add_torque(Vec3{0, M, 0});
    }

    // IAerodynamicsModel queries
    Real get_cl() const override { return cl_; }
    Real get_cd() const override { return cd_; }
    Real get_cm() const override { return cm_; }
    Real get_alpha() const override { return alpha_; }
    Real get_beta() const override { return beta_; }
    Real get_mach() const override { return mach_; }
    Real get_qbar() const override { return qbar_; }

private:
    void compute_coefficients() {
        // Your custom coefficient model here
        cl_ = 5.5 * alpha_;  // Example: linear lift curve
        cd_ = 0.02 + cl_ * cl_ / (3.14159 * 8.0 * 0.85);  // Drag polar
        cm_ = -0.5 * alpha_;  // Pitching moment
    }

    Real ref_area_{1.0}, ref_chord_{1.0}, ref_span_{1.0};
    Real alpha_{0}, beta_{0}, mach_{0}, qbar_{0};
    Real cl_{0}, cd_{0}, cm_{0};
};
```

### Step 2: Register and Use

```cpp
// Create entity
EntityId aircraft = engine.create_entity("CustomAircraft", Domain::Air);

// Create and configure custom model
auto aero = std::make_unique<CustomAeroModel>();
aero->set_reference_area(27.87);
aero->set_reference_chord(3.45);
aero->set_reference_span(9.45);

// Use in simulation loop
physics::EntityForces forces;
auto state = engine.get_entity_state(aircraft);
auto env = engine.get_environment(aircraft);

forces.clear();
aero->compute_forces(state, env, dt, forces);
engine.apply_forces(aircraft, forces);
```

## Creating Custom Terramechanics

```cpp
class CustomTerraModel : public physics::ITerramechanicsModel {
public:
    void set_contact_dimensions(Real width, Real length) {
        contact_width_ = width;
        contact_length_ = length;
    }

    void compute_forces(const physics::EntityState& state,
                        const environment::Environment& env,
                        Real dt,
                        physics::EntityForces& forces) override {
        // Check if on ground
        Real height = env.altitude - env.terrain_elevation;
        if (height > 0.1) return;

        // Get soil properties
        auto soil = env.terrain.material.get_soil_properties();

        // Bekker-Wong pressure-sinkage
        Real pressure = state.mass * constants::G0 /
                       (contact_width_ * contact_length_);
        Real sinkage = compute_sinkage(pressure, soil);

        // Motion resistance
        Real resistance = compute_resistance(sinkage, soil);

        // Traction
        Real traction = compute_traction(soil, pressure);

        // Apply forces
        forces.add_force(Vec3{traction - resistance, 0, 0});

        sinkage_ = sinkage;
        resistance_ = resistance;
        traction_ = traction;
    }

    Real get_sinkage() const override { return sinkage_; }
    Real get_motion_resistance() const override { return resistance_; }
    Real get_traction() const override { return traction_; }
    Real get_slip_ratio() const override { return slip_; }

private:
    Real compute_sinkage(Real pressure, const SoilProperties& soil) {
        // p = (k_c/b + k_phi) * z^n
        // z = (p / (k_c/b + k_phi))^(1/n)
        Real k = soil.k_c / contact_width_ + soil.k_phi;
        return std::pow(pressure / k, 1.0 / soil.n);
    }

    Real compute_resistance(Real sinkage, const SoilProperties& soil) {
        Real k = soil.k_c / contact_width_ + soil.k_phi;
        return contact_width_ * contact_length_ * k *
               std::pow(sinkage, soil.n + 1) / (soil.n + 1);
    }

    Real compute_traction(const SoilProperties& soil, Real pressure) {
        // Mohr-Coulomb: tau_max = c + sigma * tan(phi)
        Real tau_max = soil.c + pressure * std::tan(soil.phi);
        return tau_max * contact_width_ * contact_length_;
    }

    Real contact_width_{0.5}, contact_length_{4.0};
    Real sinkage_{0}, resistance_{0}, traction_{0}, slip_{0};
};
```

## XML Configuration for Custom Models

You can load custom model parameters from XML:

```xml
<entity type="aircraft" name="CustomAircraft">
    <aerodynamics model="custom">
        <reference_area unit="m2">27.87</reference_area>
        <reference_chord unit="m">3.45</reference_chord>
        <reference_span unit="m">9.45</reference_span>

        <custom_parameters>
            <cl_alpha>5.5</cl_alpha>
            <cd_0>0.02</cd_0>
            <oswald_efficiency>0.85</oswald_efficiency>
        </custom_parameters>
    </aerodynamics>
</entity>
```

## Creating Custom Integrators

```cpp
class CustomIntegrator : public IStatePropagator {
public:
    void propagate(EntityState& state,
                   const EntityForces& forces,
                   Real dt) override {
        // Your custom integration scheme
        // Example: Semi-implicit Euler
        Vec3 accel = forces.force / state.mass;
        state.velocity += accel * dt;
        state.position += state.velocity * dt;

        // Angular dynamics
        Vec3 angular_accel = state.inertia.inverse() *
                            (forces.torque - state.angular_velocity.cross(
                             state.inertia * state.angular_velocity));
        state.angular_velocity += angular_accel * dt;

        // Quaternion integration
        Quaternion omega_quat{0,
            state.angular_velocity.x,
            state.angular_velocity.y,
            state.angular_velocity.z};
        state.orientation = (state.orientation +
            state.orientation * omega_quat * 0.5 * dt).normalized();
    }
};
```

## Best Practices

### Performance

1. **Cache intermediate results**: Store computed values that are used multiple times
2. **Use SIMD operations**: Leverage vectorized math where possible
3. **Minimize allocations**: Pre-allocate buffers in constructors

### Numerical Stability

1. **Set minimum thresholds**: Avoid division by zero with velocity checks
2. **Clamp outputs**: Limit coefficients to physical ranges
3. **Use appropriate precision**: Double precision for geodetic calculations

### Testing

```cpp
TEST(CustomAeroModel, LiftCoefficientLinear) {
    CustomAeroModel model;
    model.set_reference_area(10.0);

    physics::EntityState state;
    state.velocity = Vec3{100.0, 0.0, 5.0};  // Small alpha

    environment::Environment env;
    env.atmosphere.density = 1.225;
    env.atmosphere.speed_of_sound = 340.0;

    physics::EntityForces forces;
    model.compute_forces(state, env, 0.01, forces);

    EXPECT_NEAR(model.get_alpha(), 0.05, 0.001);
    EXPECT_GT(model.get_cl(), 0.0);
}
```

## See Also

- [Architecture](architecture.md) - System architecture overview
- [API Reference](../api/core.md) - Core API documentation
- [Examples](../tutorials/examples.md) - Complete code examples
