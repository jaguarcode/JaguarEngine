#pragma once
/**
 * @file sea.h
 * @brief Sea domain physics models (hydrodynamics, buoyancy)
 */

#include "jaguar/core/types.h"
#include "jaguar/physics/force.h"

namespace jaguar::domain::sea {

// ============================================================================
// Buoyancy Model
// ============================================================================

/**
 * @brief Buoyancy force calculator using displaced volume
 */
class BuoyancyModel : public physics::IHydrodynamicsModel {
public:
    BuoyancyModel();
    ~BuoyancyModel() override;

    void compute_forces(
        const physics::EntityState& state,
        const environment::Environment& env,
        Real dt,
        physics::EntityForces& out_forces) override;

    const std::string& name() const override { return name_; }

    Real get_buoyancy() const override { return buoyancy_force_; }
    Real get_draft() const override { return draft_; }
    Real get_heel() const override { return heel_; }
    Real get_trim() const override { return trim_; }

    // Configuration
    void set_displaced_volume(Real volume) { displaced_volume_ = volume; }
    void set_metacentric_height(Real gm) { metacentric_height_ = gm; }
    void set_center_of_buoyancy(const Vec3& cb) { center_of_buoyancy_ = cb; }

private:
    std::string name_{"Buoyancy"};

    Real displaced_volume_{100.0};     // m³
    Real metacentric_height_{2.0};     // m (GM)
    Vec3 center_of_buoyancy_{0, 0, -1}; // Relative to CG

    // Computed values
    Real buoyancy_force_{0.0};
    Real draft_{0.0};
    Real heel_{0.0};
    Real trim_{0.0};
};

// ============================================================================
// Wave Model
// ============================================================================

/**
 * @brief Wave spectrum types
 */
enum class WaveSpectrum {
    PiersonMoskowitz,   ///< Fully developed sea
    JONSWAP,            ///< Fetch-limited
    Bretschneider       ///< Parameterized
};

/**
 * @brief Sea state parameters
 */
struct SeaState {
    Real significant_height{1.0};   ///< H_s (m)
    Real peak_period{6.0};          ///< T_p (s)
    Real direction{0.0};            ///< Primary wave direction (rad from N)
    WaveSpectrum spectrum{WaveSpectrum::PiersonMoskowitz};

    // NATO sea state presets (0-8)
    static SeaState FromNATOSeaState(int sea_state);
};

/**
 * @brief Wave surface model for wave loading calculation
 */
class WaveModel {
public:
    WaveModel();
    ~WaveModel();

    /**
     * @brief Set sea state
     */
    void set_sea_state(const SeaState& state);

    /**
     * @brief Get wave elevation at position and time
     */
    Real get_elevation(Real x, Real y, Real time) const;

    /**
     * @brief Get wave particle velocity at position
     */
    Vec3 get_particle_velocity(Real x, Real y, Real z, Real time) const;

    /**
     * @brief Get wave slope at position
     */
    Vec3 get_slope(Real x, Real y, Real time) const;

    /**
     * @brief Wave component for spectral superposition
     */
    struct WaveComponent {
        Real amplitude;
        Real frequency;
        Real direction;
        Real phase;
    };

    /**
     * @brief Access wave components for RAO calculations
     */
    const std::vector<WaveComponent>& get_components() const { return components_; }

private:
    SeaState sea_state_;
    std::vector<WaveComponent> components_;

    void generate_components();

    /**
     * @brief Compute wave spectrum value at given frequency
     * @param omega Angular frequency (rad/s)
     * @return Spectral density (m²·s)
     */
    Real compute_spectrum(Real omega) const;
};

// ============================================================================
// RAO (Response Amplitude Operator) Model
// ============================================================================

/**
 * @brief RAO table for ship motion response
 */
class RAOModel {
public:
    RAOModel();
    ~RAOModel();

    /**
     * @brief Set RAO data for a specific DOF
     * @param dof 0=surge, 1=sway, 2=heave, 3=roll, 4=pitch, 5=yaw
     * @param frequencies Wave frequencies (rad/s)
     * @param amplitudes RAO amplitudes
     * @param phases RAO phases (rad)
     */
    void set_rao(int dof,
                 const std::vector<Real>& frequencies,
                 const std::vector<Real>& amplitudes,
                 const std::vector<Real>& phases);

    /**
     * @brief Get motion response for given wave frequency
     */
    void get_response(Real omega, Real wave_amp, Real& out_amplitude, Real& out_phase, int dof) const;

    /**
     * @brief Calculate 6DOF motion response to sea state
     */
    void calculate_response(const WaveModel& waves, Real time, Vec3& out_disp, Vec3& out_rot);

private:
    struct RAOData {
        std::vector<Real> frequencies;
        std::vector<Real> amplitudes;
        std::vector<Real> phases;
    };
    RAOData rao_data_[6];  // 6 DOF

    /**
     * @brief Initialize with default RAO data for generic vessel
     */
    void initialize_default_rao();
};

// ============================================================================
// Hydrodynamics Model (MMG)
// ============================================================================

/**
 * @brief Maneuvering Mathematical Group (MMG) model
 */
class HydrodynamicsModel : public physics::IHydrodynamicsModel {
public:
    HydrodynamicsModel();
    ~HydrodynamicsModel() override;

    void compute_forces(
        const physics::EntityState& state,
        const environment::Environment& env,
        Real dt,
        physics::EntityForces& out_forces) override;

    const std::string& name() const override { return name_; }

    Real get_buoyancy() const override;
    Real get_draft() const override { return draft_; }
    Real get_heel() const override { return heel_; }
    Real get_trim() const override { return trim_; }

    // Configuration
    void set_hull_coefficients(Real x_vv, Real x_rr, Real y_v, Real y_r, Real n_v, Real n_r);
    void set_rudder_parameters(Real area, Real aspect_ratio);
    void set_propeller_parameters(Real diameter, Real pitch_ratio);

    // Control
    void set_rudder_angle(Real angle_rad) { rudder_angle_ = angle_rad; }
    void set_propeller_rpm(Real rpm) { propeller_rpm_ = rpm; }

private:
    std::string name_{"Hydrodynamics"};

    // Hull coefficients
    Real x_vv_{0.0}, x_rr_{0.0};
    Real y_v_{0.0}, y_r_{0.0};
    Real n_v_{0.0}, n_r_{0.0};

    // Rudder parameters
    Real rudder_area_{10.0};
    Real rudder_aspect_ratio_{1.5};
    Real rudder_angle_{0.0};

    // Propeller parameters
    Real propeller_diameter_{3.0};
    Real propeller_pitch_ratio_{1.0};
    Real propeller_rpm_{0.0};

    // State
    Real draft_{0.0};
    Real heel_{0.0};
    Real trim_{0.0};
};

} // namespace jaguar::domain::sea
