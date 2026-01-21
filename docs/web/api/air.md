# Air Domain API Reference

Aerodynamics, propulsion, and flight control system APIs.

**Header:** `jaguar/domain/air.h`

## AeroTable

Multi-dimensional lookup table for aerodynamic coefficients.

```cpp
class AeroTable {
public:
    // Configuration
    void set_breakpoints(int dimension, const std::vector<Real>& values);
    void set_data(const std::vector<Real>& data);

    // Lookup
    Real lookup(const std::vector<Real>& inputs) const;
    Real lookup(Real input) const;  // 1D convenience
    Real lookup(Real input1, Real input2) const;  // 2D convenience
};
```

### Example

```cpp
// Create 2D table: CL vs alpha and Mach
AeroTable cl_table;

// Alpha breakpoints (radians)
cl_table.set_breakpoints(0, {-0.35, -0.17, 0.0, 0.17, 0.35, 0.52});

// Mach breakpoints
cl_table.set_breakpoints(1, {0.0, 0.5, 0.9, 1.2});

// CL data (row-major: alpha varies fastest)
cl_table.set_data({
    -0.80, -0.75, -0.70, -0.60,  // alpha = -0.35
    -0.40, -0.38, -0.35, -0.30,  // alpha = -0.17
     0.20,  0.22,  0.25,  0.20,  // alpha = 0.0
     0.80,  0.85,  0.90,  0.75,  // alpha = 0.17
     1.20,  1.25,  1.15,  0.90,  // alpha = 0.35
     0.90,  0.85,  0.70,  0.50   // alpha = 0.52
});

// Lookup
Real alpha = 0.1;  // rad
Real mach = 0.7;
Real cl = cl_table.lookup(alpha, mach);
```

## AerodynamicsModel

Complete aerodynamic force and moment model.

```cpp
class AerodynamicsModel : public physics::IAerodynamicsModel {
public:
    // Reference geometry
    void set_reference_area(Real area);    // m²
    void set_reference_chord(Real chord);  // m (for pitching moment)
    void set_reference_span(Real span);    // m (for rolling/yawing)

    // Coefficient tables
    void set_cl_table(std::unique_ptr<AeroTable> table);
    void set_cd_table(std::unique_ptr<AeroTable> table);
    void set_cm_table(std::unique_ptr<AeroTable> table);
    void set_cy_table(std::unique_ptr<AeroTable> table);  // Side force
    void set_cn_table(std::unique_ptr<AeroTable> table);  // Yawing moment
    void set_cll_table(std::unique_ptr<AeroTable> table); // Rolling moment

    // Control effectiveness
    void set_cl_elevator(Real dcl_de);   // dCL/d_elevator
    void set_cm_elevator(Real dcm_de);   // dCm/d_elevator
    void set_cy_rudder(Real dcy_dr);     // dCY/d_rudder
    void set_cn_rudder(Real dcn_dr);     // dCn/d_rudder
    void set_cll_aileron(Real dcll_da);  // dCll/d_aileron

    // Damping derivatives
    void set_cl_alpha_dot(Real cl_ad);   // CL_alpha_dot
    void set_cm_alpha_dot(Real cm_ad);   // Cm_alpha_dot
    void set_cm_q(Real cmq);             // Pitch damping
    void set_cn_r(Real cnr);             // Yaw damping
    void set_cll_p(Real cllp);           // Roll damping

    // Control surface positions
    void set_elevator(Real deg);
    void set_aileron(Real deg);
    void set_rudder(Real deg);

    // IForceGenerator
    void compute_forces(const physics::EntityState& state,
                        const environment::Environment& env,
                        Real dt,
                        physics::EntityForces& forces) override;

    // IAerodynamicsModel queries
    Real get_cl() const override;
    Real get_cd() const override;
    Real get_cm() const override;
    Real get_alpha() const override;  // rad
    Real get_beta() const override;   // rad
    Real get_mach() const override;
    Real get_qbar() const override;   // Pa
};
```

### Example

```cpp
domain::air::AerodynamicsModel aero;

// Set reference geometry
aero.set_reference_area(27.87);   // m²
aero.set_reference_chord(3.45);   // m
aero.set_reference_span(10.0);    // m

// Set coefficient tables
aero.set_cl_table(std::move(cl_table));
aero.set_cd_table(std::move(cd_table));
aero.set_cm_table(std::move(cm_table));

// Set control effectiveness
aero.set_cm_elevator(-0.05);  // Elevator authority

// Set damping
aero.set_cm_q(-15.0);  // Pitch damping
aero.set_cll_p(-0.4);  // Roll damping

// In simulation loop
aero.set_elevator(elevator_cmd);
aero.set_aileron(aileron_cmd);

physics::EntityForces forces;
aero.compute_forces(state, env, dt, forces);

// Query aerodynamic state
std::cout << "Alpha: " << aero.get_alpha() * RAD_TO_DEG << " deg\n";
std::cout << "Mach: " << aero.get_mach() << "\n";
std::cout << "CL: " << aero.get_cl() << "\n";
```

## PropulsionModel

Engine thrust and fuel consumption model.

```cpp
class PropulsionModel : public physics::IPropulsionModel {
public:
    // Configuration
    void set_max_thrust(Real thrust);               // N
    void set_afterburner_thrust(Real thrust);       // N (optional)
    void set_fuel_capacity(Real fuel);              // kg
    void set_specific_fuel_consumption(Real sfc);   // kg/N/s
    void set_afterburner_sfc(Real sfc);             // kg/N/s

    // Thrust table (optional, for altitude/Mach variation)
    void set_thrust_table(std::unique_ptr<AeroTable> table);

    // Engine position (for thrust line offset)
    void set_position(const Vec3& pos);
    void set_thrust_axis(const Vec3& axis);

    // Control
    void set_throttle(Real throttle);  // 0.0 to 1.0
    void set_afterburner(bool enabled);
    void start();
    void stop();

    // Fuel management
    void set_fuel(Real fuel);  // kg
    void add_fuel(Real fuel);  // kg

    // IForceGenerator
    void compute_forces(const physics::EntityState& state,
                        const environment::Environment& env,
                        Real dt,
                        physics::EntityForces& forces) override;

    // IPropulsionModel queries
    Real get_thrust() const override;           // N
    Real get_fuel_flow() const override;        // kg/s
    Real get_fuel_remaining() const override;   // kg
    bool is_running() const override;
};
```

### Example

```cpp
domain::air::PropulsionModel engine;

// Configure engine
engine.set_max_thrust(76000.0);        // 76 kN military thrust
engine.set_afterburner_thrust(127000.0); // 127 kN with AB
engine.set_fuel_capacity(3000.0);       // 3000 kg internal fuel
engine.set_specific_fuel_consumption(0.0225);  // SFC
engine.set_afterburner_sfc(0.058);

// Position (behind CG)
engine.set_position(Vec3{-5.0, 0.0, 0.0});
engine.set_thrust_axis(Vec3{1.0, 0.0, 0.0});

// Start and set throttle
engine.start();
engine.set_throttle(0.8);  // 80% throttle

// In simulation loop
physics::EntityForces forces;
engine.compute_forces(state, env, dt, forces);

// Check fuel
if (engine.get_fuel_remaining() < 500.0) {
    std::cout << "Bingo fuel!\n";
}
```

## FlightControlSystem

Flight control surface management.

```cpp
class FlightControlSystem {
public:
    struct ControlInputs {
        Real pitch_cmd{0.0};     // -1 to +1
        Real roll_cmd{0.0};      // -1 to +1
        Real yaw_cmd{0.0};       // -1 to +1
        Real throttle_cmd{0.0};  // 0 to 1
    };

    struct ControlOutputs {
        Real elevator_deg{0.0};
        Real aileron_deg{0.0};
        Real rudder_deg{0.0};
    };

    // Configuration
    void set_elevator_range(Real min_deg, Real max_deg);
    void set_aileron_range(Real min_deg, Real max_deg);
    void set_rudder_range(Real min_deg, Real max_deg);

    // Rate limits
    void set_elevator_rate_limit(Real deg_per_sec);
    void set_aileron_rate_limit(Real deg_per_sec);
    void set_rudder_rate_limit(Real deg_per_sec);

    // Trim
    void set_pitch_trim(Real trim);  // -1 to +1
    void set_roll_trim(Real trim);
    void set_yaw_trim(Real trim);

    // Process inputs
    ControlOutputs process(const ControlInputs& inputs, Real dt);

    // Query current positions
    Real get_elevator() const;
    Real get_aileron() const;
    Real get_rudder() const;
};
```

### Example

```cpp
domain::air::FlightControlSystem fcs;

// Configure control ranges
fcs.set_elevator_range(-25.0, 25.0);
fcs.set_aileron_range(-21.5, 21.5);
fcs.set_rudder_range(-30.0, 30.0);

// Set rate limits
fcs.set_elevator_rate_limit(60.0);  // 60 deg/s

// Trim for level flight
fcs.set_pitch_trim(0.05);

// Process pilot inputs
FlightControlSystem::ControlInputs inputs;
inputs.pitch_cmd = -0.2;  // Nose up
inputs.roll_cmd = 0.5;    // Roll right
inputs.throttle_cmd = 0.8;

auto outputs = fcs.process(inputs, dt);

// Apply to aerodynamics model
aero.set_elevator(outputs.elevator_deg);
aero.set_aileron(outputs.aileron_deg);
aero.set_rudder(outputs.rudder_deg);
```

## Atmosphere Integration

The air domain models integrate with the atmosphere service.

```cpp
// Get environment at entity position
auto env = engine.get_environment(aircraft_id);

// Atmosphere properties available
Real temperature = env.atmosphere.temperature;    // K
Real pressure = env.atmosphere.pressure;          // Pa
Real density = env.atmosphere.density;            // kg/m³
Real speed_of_sound = env.atmosphere.speed_of_sound;  // m/s
Vec3 wind = env.atmosphere.wind;                  // m/s (NED)

// Aerodynamics automatically uses these
aero.compute_forces(state, env, dt, forces);
```

## See Also

- [Physics API](physics.md) - Force generator interfaces
- [Environment API](environment.md) - Atmosphere system
- [Air Domain Concepts](../domains/air.md) - Domain overview
- [Air Domain Tutorial](../tutorials/air-domain.md) - Step-by-step guide

