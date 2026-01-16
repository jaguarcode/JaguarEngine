#pragma once
/**
 * @file land.h
 * @brief Land domain physics models (terramechanics, suspension)
 */

#include "jaguar/core/types.h"
#include "jaguar/physics/force.h"

namespace jaguar::domain::land {

// ============================================================================
// Soil Properties
// ============================================================================

/**
 * @brief Bekker-Wong soil parameters
 */
struct SoilProperties {
    Real k_c{0.0};      ///< Cohesive modulus (kN/m^(n+1))
    Real k_phi{0.0};    ///< Frictional modulus (kN/m^(n+2))
    Real n{1.0};        ///< Deformation exponent
    Real c{0.0};        ///< Cohesion (kPa)
    Real phi{0.0};      ///< Internal friction angle (rad)

    // Common soil types
    static SoilProperties DrySand();
    static SoilProperties WetSand();
    static SoilProperties Clay();
    static SoilProperties Snow();
    static SoilProperties Asphalt();  // Rigid surface
};

// ============================================================================
// Terramechanics Model (Bekker-Wong)
// ============================================================================

/**
 * @brief Terramechanics force generator using Bekker-Wong theory
 */
class TerramechanicsModel : public physics::ITerramechanicsModel {
public:
    TerramechanicsModel();
    ~TerramechanicsModel() override;

    void compute_forces(
        const physics::EntityState& state,
        const environment::Environment& env,
        Real dt,
        physics::EntityForces& out_forces) override;

    const std::string& name() const override { return name_; }

    Real get_sinkage() const override { return sinkage_; }
    Real get_motion_resistance() const override { return motion_resistance_; }
    Real get_traction() const override { return traction_; }
    Real get_slip_ratio() const override { return slip_ratio_; }

    // Configuration
    void set_contact_area(Real width, Real length);
    void set_vehicle_weight(Real weight_n);

private:
    std::string name_{"Terramechanics"};

    Real contact_width_{0.5};   // m
    Real contact_length_{1.0};  // m
    Real vehicle_weight_{10000.0}; // N

    // Computed values
    Real sinkage_{0.0};
    Real motion_resistance_{0.0};
    Real traction_{0.0};
    Real slip_ratio_{0.0};

    /**
     * @brief Calculate pressure-sinkage using Bekker equation
     * p = (k_c/b + k_phi) * z^n
     */
    Real calculate_sinkage(const SoilProperties& soil, Real pressure) const;
};

// ============================================================================
// Suspension Model
// ============================================================================

/**
 * @brief Single suspension unit (spring-damper)
 */
struct SuspensionUnit {
    Real spring_k{50000.0};     ///< Spring stiffness (N/m)
    Real damper_c{5000.0};      ///< Damping coefficient (N·s/m)
    Real preload{0.0};          ///< Preload force (N)
    Real travel_max{0.3};       ///< Maximum travel (m)
    Real travel_min{0.0};       ///< Minimum travel (m)

    // State
    Real current_position{0.0}; ///< Current compression (m)
    Real current_velocity{0.0}; ///< Compression rate (m/s)

    /**
     * @brief Calculate suspension force
     */
    Real calculate_force() const;
};

/**
 * @brief Vehicle suspension system
 */
class SuspensionModel {
public:
    SuspensionModel();
    ~SuspensionModel();

    /**
     * @brief Add a suspension unit at specified position
     */
    void add_unit(const Vec3& position, const SuspensionUnit& unit);

    /**
     * @brief Update all suspension units
     */
    void update(const physics::EntityState& state, Real dt);

    /**
     * @brief Get total force from all suspension units
     */
    Vec3 get_total_force() const;

    /**
     * @brief Get total torque from all suspension units
     */
    Vec3 get_total_torque() const;

    /**
     * @brief Get number of suspension units
     */
    SizeT unit_count() const { return units_.size(); }

private:
    struct UnitData {
        Vec3 position;
        SuspensionUnit unit;
    };
    std::vector<UnitData> units_;
};

// ============================================================================
// Tracked Vehicle Model
// ============================================================================

/**
 * @brief Simplified tracked vehicle dynamics
 */
class TrackedVehicleModel {
public:
    TrackedVehicleModel();
    ~TrackedVehicleModel();

    struct TrackState {
        Real tension{10000.0};      ///< Track tension (N)
        Real velocity{0.0};         ///< Track linear velocity (m/s)
        Real slip{0.0};             ///< Track slip ratio
    };

    /**
     * @brief Set sprocket parameters
     */
    void set_sprocket(Real radius, Real max_torque);

    /**
     * @brief Update track dynamics
     */
    void update(Real drive_torque, Real load, const SoilProperties& soil, Real dt);

    /**
     * @brief Get current track state
     */
    const TrackState& get_left_track() const { return left_track_; }
    const TrackState& get_right_track() const { return right_track_; }

    /**
     * @brief Get propulsive force
     */
    Real get_propulsive_force() const;

private:
    Real sprocket_radius_{0.3};
    Real max_torque_{50000.0};

    TrackState left_track_;
    TrackState right_track_;
};

} // namespace jaguar::domain::land
