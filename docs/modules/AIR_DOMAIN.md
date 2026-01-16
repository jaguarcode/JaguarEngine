# Air Domain Documentation

The Air domain module implements aerodynamics, propulsion, and flight control systems for aircraft, missiles, and other airborne entities.

## Headers

| Header | Purpose |
|--------|---------|
| `jaguar/domain/air.h` | Aerodynamics, propulsion, flight control |

## Components

### AeroTable

Multi-dimensional lookup table for aerodynamic coefficients with linear interpolation:

```cpp
class AeroTable {
public:
    // Set independent variable breakpoints (alpha, mach, beta, etc.)
    void set_breakpoints(int dimension, const std::vector<Real>& values);

    // Set table data (row-major order)
    void set_data(const std::vector<Real>& data);

    // Lookup with N-dimensional interpolation
    Real lookup(const std::vector<Real>& inputs) const;
};
```

**Features:**
- Up to 6 dimensions (alpha, beta, Mach, altitude, control surfaces)
- Binary search for efficient index location
- N-dimensional linear interpolation
- Extrapolation at boundaries (holds boundary values)

**Usage:**
```cpp
AeroTable cl_table;

// 2D table: CL vs alpha (dim 0) and Mach (dim 1)
cl_table.set_breakpoints(0, {-10.0, -5.0, 0.0, 5.0, 10.0, 15.0});  // alpha (deg)
cl_table.set_breakpoints(1, {0.0, 0.4, 0.8, 1.0, 1.2});            // Mach

// Data: 6 alpha points × 5 Mach points = 30 values (row-major)
cl_table.set_data({
    -0.5, -0.5, -0.45, -0.4, -0.35,   // alpha = -10
    -0.25, -0.25, -0.22, -0.2, -0.17, // alpha = -5
    0.0, 0.0, 0.0, 0.0, 0.0,          // alpha = 0
    // ... etc
});

// Interpolate at alpha=7.5 deg, Mach=0.6
Real cl = cl_table.lookup({7.5, 0.6});
```

### AerodynamicsModel

Complete 6-DOF aerodynamics model implementing `IAerodynamicsModel`:

```cpp
class AerodynamicsModel : public physics::IAerodynamicsModel {
public:
    // Configuration
    void set_reference_area(Real area);   // Wing area (m²)
    void set_reference_chord(Real chord); // Mean aerodynamic chord (m)
    void set_reference_span(Real span);   // Wing span (m)

    // Coefficient tables (optional - defaults used if not set)
    void set_cl_table(std::unique_ptr<AeroTable> table);
    void set_cd_table(std::unique_ptr<AeroTable> table);
    void set_cm_table(std::unique_ptr<AeroTable> table);

    // State queries (from IAerodynamicsModel)
    Real get_cl() const;    // Lift coefficient
    Real get_cd() const;    // Drag coefficient
    Real get_cm() const;    // Pitching moment coefficient
    Real get_alpha() const; // Angle of attack (rad)
    Real get_beta() const;  // Sideslip angle (rad)
    Real get_mach() const;  // Mach number
    Real get_qbar() const;  // Dynamic pressure (Pa)
};
```

**Aerodynamic Computations:**

1. **Velocity Transformation**: ECEF → Body frame (accounting for wind)
2. **Angle Computation**:
   - α = atan2(w, u) (angle of attack)
   - β = asin(v / V) (sideslip)
3. **Dynamic Pressure**: q = 0.5 × ρ × V²
4. **Coefficient Lookup**: From tables or default curves
5. **Damping Derivatives**: Rate effects (p, q, r)
6. **Force Transformation**: Wind frame → Body frame

**Default Coefficient Models:**

```cpp
// Lift: CL = CL_α × α (CL_α = 5.5/rad typical)
CL = 5.5 * alpha;  // Limited to ±1.5

// Drag: Parabolic polar CD = CD_0 + K × CL²
CD = 0.02 + CL² / (π × AR × e);  // AR=8, e=0.85

// Pitching moment: CM = CM_α × α (negative = stable)
CM = -0.5 * alpha;

// Lateral-directional:
CY = -0.5 * beta;     // Side force
Cl = -0.1 * beta;     // Roll (dihedral effect)
Cn = 0.1 * beta;      // Yaw (weathercock stability)
```

**Damping Derivatives:**
```cpp
// Non-dimensional rates
p̂ = p × b / (2V)   // Roll rate
q̂ = q × c / (2V)   // Pitch rate
r̂ = r × b / (2V)   // Yaw rate

// Contributions
CL += 5.0 × q̂      // Pitch rate lift
CM += -15.0 × q̂   // Pitch damping
Cl += -0.4 × p̂    // Roll damping
Cn += -0.15 × r̂   // Yaw damping
```

**Force Transformation (Wind → Body):**
```cpp
X_body = -D×cos(α)×cos(β) - Y×cos(α)×sin(β) + L×sin(α)
Y_body = -D×sin(β) + Y×cos(β)
Z_body = -D×sin(α)×cos(β) - Y×sin(α)×sin(β) - L×cos(α)
```

**Usage:**
```cpp
AerodynamicsModel aero;

// Configure reference geometry (F-16 example)
aero.set_reference_area(27.87);    // m²
aero.set_reference_chord(3.45);    // m
aero.set_reference_span(9.45);     // m

// Optional: Set custom coefficient tables
auto cl_table = std::make_unique<AeroTable>();
// ... configure table ...
aero.set_cl_table(std::move(cl_table));

// In simulation loop
physics::EntityForces forces;
forces.clear();
aero.compute_forces(state, env, dt, forces);

// Query computed values
Real alpha_deg = aero.get_alpha() * RAD_TO_DEG;
Real cl = aero.get_cl();
Real qbar = aero.get_qbar();
```

### PropulsionModel

Turbofan/turbojet engine model implementing `IPropulsionModel`:

```cpp
class PropulsionModel : public physics::IPropulsionModel {
public:
    // Configuration
    void set_max_thrust(Real thrust);             // Sea-level static thrust (N)
    void set_fuel_capacity(Real fuel);            // Maximum fuel (kg)
    void set_specific_fuel_consumption(Real sfc); // SFC (kg/N/s)

    // Control
    void set_throttle(Real throttle);  // 0.0 to 1.0
    void start();
    void stop();

    // State queries
    Real get_thrust() const;          // Current thrust (N)
    Real get_fuel_flow() const;       // Fuel consumption (kg/s)
    Real get_fuel_remaining() const;  // Remaining fuel (kg)
    bool is_running() const;
};
```

**Thrust Modeling:**

1. **Altitude Effect** (density ratio):
   ```cpp
   σ = ρ / ρ_SL
   altitude_factor = σ^0.7  // Typical turbofan
   ```

2. **Ram Effect** (Mach correction):
   ```cpp
   if (M < 0.8)  ram = 1.0 + 0.15×M
   if (M < 1.2)  ram = 1.12 - 0.1×(M - 0.8)
   if (M ≥ 1.2)  ram = max(0.5, 1.08 - 0.3×(M - 1.2))
   ```

3. **Net Thrust**:
   ```cpp
   T = T_max × throttle × altitude_factor × ram_factor
   ```

**Fuel Consumption:**

```cpp
// SFC variation with operating conditions
sfc_factor = f(throttle, altitude, mach)

// Low throttle: less efficient
if (throttle < 0.3) sfc_factor *= 1 + 0.5×(0.3 - throttle)

// High throttle: afterburner region
if (throttle > 0.9) sfc_factor *= 1 + 0.3×(throttle - 0.9)/0.1

// Fuel flow
fuel_flow = sfc × thrust × sfc_factor
```

**Usage:**
```cpp
PropulsionModel engine;

// Configure F-16 engine (F110-GE-129)
engine.set_max_thrust(131000.0);   // N (with afterburner)
engine.set_fuel_capacity(3200.0); // kg
engine.set_specific_fuel_consumption(2.5e-5); // kg/N/s

// Start and set throttle
engine.start();
engine.set_throttle(0.85);

// In simulation loop
physics::EntityForces forces;
forces.clear();
engine.compute_forces(state, env, dt, forces);

// Check status
if (engine.get_fuel_remaining() < 500.0) {
    // Low fuel warning
}
```

### FlightControlSystem

Flight control system for mapping pilot inputs to control surface deflections:

```cpp
class FlightControlSystem {
public:
    struct ControlInputs {
        Real pitch_cmd{0.0};    // -1 to +1
        Real roll_cmd{0.0};     // -1 to +1
        Real yaw_cmd{0.0};      // -1 to +1
        Real throttle_cmd{0.0}; // 0 to 1
    };

    struct ControlOutputs {
        Real elevator_deg{0.0};
        Real aileron_deg{0.0};
        Real rudder_deg{0.0};
    };

    // Process inputs to surface deflections
    ControlOutputs process(const ControlInputs& inputs, Real dt);

    // Configuration
    void set_elevator_range(Real min_deg, Real max_deg);  // Default: ±25°
    void set_aileron_range(Real min_deg, Real max_deg);   // Default: ±20°
    void set_rudder_range(Real min_deg, Real max_deg);    // Default: ±30°
};
```

**Usage:**
```cpp
FlightControlSystem fcs;

// Configure control surface limits
fcs.set_elevator_range(-25.0, 25.0);
fcs.set_aileron_range(-20.0, 20.0);
fcs.set_rudder_range(-30.0, 30.0);

// Process pilot inputs
FlightControlSystem::ControlInputs inputs;
inputs.pitch_cmd = -0.3;   // Pull back slightly
inputs.roll_cmd = 0.5;     // Roll right
inputs.yaw_cmd = 0.0;
inputs.throttle_cmd = 0.8;

auto outputs = fcs.process(inputs, dt);

// Use outputs to update aerodynamic model
// (Control surface effects on coefficients)
```

## Complete Aircraft Example

```cpp
#include <jaguar/jaguar.h>

class Aircraft {
public:
    Aircraft(const std::string& name) {
        // Create entity
        entity_id_ = entities_.create_entity(name, Domain::Air);

        // Configure aerodynamics
        aero_.set_reference_area(27.87);   // F-16 wing area
        aero_.set_reference_chord(3.45);
        aero_.set_reference_span(9.45);

        // Configure propulsion
        engine_.set_max_thrust(131000.0);
        engine_.set_fuel_capacity(3200.0);
        engine_.set_specific_fuel_consumption(2.5e-5);
        engine_.start();

        // Configure FCS
        fcs_.set_elevator_range(-25.0, 25.0);
        fcs_.set_aileron_range(-20.0, 20.0);
    }

    void set_throttle(Real throttle) {
        engine_.set_throttle(throttle);
    }

    void set_controls(Real pitch, Real roll, Real yaw) {
        pilot_inputs_.pitch_cmd = pitch;
        pilot_inputs_.roll_cmd = roll;
        pilot_inputs_.yaw_cmd = yaw;
    }

    void update(Real dt) {
        // Get current state
        auto state = entities_.get_state(entity_id_);

        // Update environment
        environment::Environment env;
        // ... populate environment ...

        // Clear forces
        forces_.clear();

        // Compute aerodynamic forces
        aero_.compute_forces(state, env, dt, forces_);

        // Compute propulsion forces
        engine_.compute_forces(state, env, dt, forces_);

        // Add gravity (simplified)
        Vec3 gravity{0.0, 0.0, state.mass * 9.81};
        forces_.add_force(gravity);

        // Process FCS
        auto surface_deflections = fcs_.process(pilot_inputs_, dt);

        // Integrate state
        // ... use physics system ...
    }

    // Telemetry
    Real get_altitude() const;
    Real get_airspeed() const;
    Real get_mach() const { return aero_.get_mach(); }
    Real get_alpha_deg() const { return aero_.get_alpha() * constants::RAD_TO_DEG; }
    Real get_fuel_remaining() const { return engine_.get_fuel_remaining(); }

private:
    physics::EntityManager entities_;
    EntityId entity_id_;

    AerodynamicsModel aero_;
    PropulsionModel engine_;
    FlightControlSystem fcs_;

    FlightControlSystem::ControlInputs pilot_inputs_;
    physics::EntityForces forces_;
};
```

## Aerodynamic Conventions

### Body-Axis System
- **X**: Forward (out the nose)
- **Y**: Right (out the right wing)
- **Z**: Down

### Angular Rates
- **p**: Roll rate (about X)
- **q**: Pitch rate (about Y)
- **r**: Yaw rate (about Z)

### Moments
- **L**: Rolling moment (positive = right wing down)
- **M**: Pitching moment (positive = nose up)
- **N**: Yawing moment (positive = nose right)

### Sign Conventions
| Parameter | Positive Direction |
|-----------|-------------------|
| α (alpha) | Nose up relative to velocity |
| β (beta) | Nose left relative to velocity |
| Elevator | Trailing edge up (nose up) |
| Aileron | Right aileron trailing edge down |
| Rudder | Trailing edge left (nose left) |

## Performance Considerations

### Coefficient Table Design
- Use sufficient breakpoints near nonlinear regions (stall, transonic)
- Minimum 5-7 alpha breakpoints for basic accuracy
- Include Mach breakpoints at 0.0, 0.4, 0.8, 0.9, 1.0, 1.2, 1.5
- Control surface effects can add dimensions (up to 6 total)

### Numerical Stability
- Minimum airspeed threshold (1 m/s) prevents division by zero
- Alpha/beta clamped to physical limits
- Coefficient limits prevent extreme forces

### Integration Considerations
- 100 Hz typical for aircraft (dt = 0.01s)
- Higher rates for high-performance aircraft or missiles
- RK4 provides good stability for aircraft dynamics

## References

- **Flight Dynamics**: Stevens, Lewis & Johnson - "Aircraft Control and Simulation"
- **Aerodynamics**: Anderson - "Fundamentals of Aerodynamics"
- **Propulsion**: Mattingly - "Elements of Propulsion: Gas Turbines and Rockets"
- **Flight Control**: Blakelock - "Automatic Control of Aircraft and Missiles"
