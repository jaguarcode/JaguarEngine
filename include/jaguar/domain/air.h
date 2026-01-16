#pragma once
/**
 * @file air.h
 * @brief Air domain physics models (aerodynamics, propulsion)
 */

#include "jaguar/core/types.h"
#include "jaguar/physics/force.h"

namespace jaguar::domain::air {

// ============================================================================
// Aerodynamic Coefficient Table
// ============================================================================

/**
 * @brief Multi-dimensional lookup table for aerodynamic coefficients
 *
 * Supports interpolation across multiple independent variables
 * (alpha, beta, Mach, control surface deflections, etc.)
 */
class AeroTable {
public:
    AeroTable();
    ~AeroTable();

    // Move operations
    AeroTable(AeroTable&&) noexcept = default;
    AeroTable& operator=(AeroTable&&) noexcept = default;

    // Non-copyable (pimpl with unique_ptr)
    AeroTable(const AeroTable&) = delete;
    AeroTable& operator=(const AeroTable&) = delete;

    /**
     * @brief Set independent variable breakpoints
     */
    void set_breakpoints(int dimension, const std::vector<Real>& values);

    /**
     * @brief Set table data
     */
    void set_data(const std::vector<Real>& data);

    /**
     * @brief Lookup value with interpolation
     */
    Real lookup(const std::vector<Real>& inputs) const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

// ============================================================================
// Aerodynamics Model
// ============================================================================

/**
 * @brief Complete aerodynamics model with coefficient tables
 */
class AerodynamicsModel : public physics::IAerodynamicsModel {
public:
    AerodynamicsModel();
    ~AerodynamicsModel() override;

    // IForceGenerator interface
    void compute_forces(
        const physics::EntityState& state,
        const environment::Environment& env,
        Real dt,
        physics::EntityForces& out_forces) override;

    bool initialize() override;
    const std::string& name() const override { return name_; }

    // IAerodynamicsModel interface
    Real get_cl() const override { return cl_; }
    Real get_cd() const override { return cd_; }
    Real get_cm() const override { return cm_; }
    Real get_alpha() const override { return alpha_; }
    Real get_beta() const override { return beta_; }
    Real get_mach() const override { return mach_; }
    Real get_qbar() const override { return qbar_; }

    // Configuration
    void set_reference_area(Real area) { s_ref_ = area; }
    void set_reference_chord(Real chord) { c_ref_ = chord; }
    void set_reference_span(Real span) { b_ref_ = span; }

    void set_cl_table(std::unique_ptr<AeroTable> table);
    void set_cd_table(std::unique_ptr<AeroTable> table);
    void set_cm_table(std::unique_ptr<AeroTable> table);

private:
    std::string name_{"Aerodynamics"};

    // Reference geometry
    Real s_ref_{1.0};  // Reference area (m²)
    Real c_ref_{1.0};  // Reference chord (m)
    Real b_ref_{1.0};  // Reference span (m)

    // Current state
    Real cl_{0.0}, cd_{0.0}, cm_{0.0};
    Real cy_{0.0}, cl_roll_{0.0}, cn_{0.0};
    Real alpha_{0.0}, beta_{0.0}, mach_{0.0}, qbar_{0.0};

    // Coefficient tables
    std::unique_ptr<AeroTable> cl_table_;
    std::unique_ptr<AeroTable> cd_table_;
    std::unique_ptr<AeroTable> cm_table_;
};

// ============================================================================
// Propulsion Model
// ============================================================================

/**
 * @brief Engine/motor model
 */
class PropulsionModel : public physics::IPropulsionModel {
public:
    PropulsionModel();
    ~PropulsionModel() override;

    void compute_forces(
        const physics::EntityState& state,
        const environment::Environment& env,
        Real dt,
        physics::EntityForces& out_forces) override;

    const std::string& name() const override { return name_; }

    Real get_thrust() const override { return thrust_; }
    Real get_fuel_flow() const override { return fuel_flow_; }
    Real get_fuel_remaining() const override { return fuel_remaining_; }
    void set_throttle(Real throttle) override { throttle_ = throttle; }
    bool is_running() const override { return running_; }

    // Configuration
    void set_max_thrust(Real thrust) { max_thrust_ = thrust; }
    void set_fuel_capacity(Real fuel) { fuel_capacity_ = fuel; fuel_remaining_ = fuel; }
    void set_specific_fuel_consumption(Real sfc) { sfc_ = sfc; }

    void start();
    void stop();

private:
    std::string name_{"Propulsion"};

    Real max_thrust_{10000.0};      // N
    Real fuel_capacity_{1000.0};    // kg
    Real fuel_remaining_{1000.0};   // kg
    Real sfc_{0.0001};              // kg/N/s
    Real throttle_{0.0};
    Real thrust_{0.0};
    Real fuel_flow_{0.0};
    bool running_{false};
};

// ============================================================================
// Flight Control System
// ============================================================================

/**
 * @brief Flight control system / autopilot
 */
class FlightControlSystem {
public:
    FlightControlSystem() = default;

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

    /**
     * @brief Process control inputs to surface deflections
     */
    ControlOutputs process(const ControlInputs& inputs, Real dt);

    // Configuration
    void set_elevator_range(Real min_deg, Real max_deg);
    void set_aileron_range(Real min_deg, Real max_deg);
    void set_rudder_range(Real min_deg, Real max_deg);

private:
    Real elev_min_{-25.0}, elev_max_{25.0};
    Real ail_min_{-20.0}, ail_max_{20.0};
    Real rud_min_{-30.0}, rud_max_{30.0};
};

} // namespace jaguar::domain::air
