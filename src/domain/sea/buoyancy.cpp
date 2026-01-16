/**
 * @file buoyancy.cpp
 * @brief Buoyancy model implementation
 *
 * Implements Archimedes' principle and static stability calculations
 * for floating bodies.
 *
 * Key concepts:
 * - Buoyancy force: F_b = rho * g * V_displaced
 * - Righting moment: M = W * GM * sin(heel)
 * - Metacentric height (GM): Measure of initial stability
 *
 * Reference: Ship Hydrostatics and Stability (Biran)
 */

#include "jaguar/domain/sea.h"
#include "jaguar/environment/environment.h"
#include <cmath>
#include <algorithm>

namespace jaguar::domain::sea {

// ============================================================================
// BuoyancyModel Implementation
// ============================================================================

BuoyancyModel::BuoyancyModel() = default;
BuoyancyModel::~BuoyancyModel() = default;

void BuoyancyModel::compute_forces(
    const physics::EntityState& state,
    const environment::Environment& env,
    [[maybe_unused]] Real dt,
    physics::EntityForces& out_forces)
{
    if (!enabled_) {
        buoyancy_force_ = 0.0;
        draft_ = 0.0;
        heel_ = 0.0;
        trim_ = 0.0;
        return;
    }

    // Check if we're in water
    if (!env.over_water) {
        buoyancy_force_ = 0.0;
        draft_ = 0.0;
        return;
    }

    // Get water density (default seawater)
    Real rho_water = constants::RHO_WATER;  // 1025 kg/m³

    // Calculate draft (depth below water surface)
    // Altitude is height above WGS84, ocean state has surface elevation
    Real water_surface = env.ocean.surface_elevation;
    draft_ = water_surface - env.altitude;

    // If above water, no buoyancy
    if (draft_ <= 0.0) {
        buoyancy_force_ = 0.0;
        heel_ = 0.0;
        trim_ = 0.0;
        return;
    }

    // Calculate submerged volume (simplified - assume box-like hull)
    // A proper implementation would use hull geometry
    // For now, use a linear approximation: V_submerged proportional to draft
    Real max_draft = 5.0;  // Assumed maximum draft when fully submerged
    Real submersion_ratio = std::min(draft_ / max_draft, 1.0);
    Real submerged_volume = displaced_volume_ * submersion_ratio;

    // Buoyancy force (Archimedes' principle)
    // F_b = rho * g * V
    buoyancy_force_ = rho_water * constants::G0 * submerged_volume;

    // Get orientation as Euler angles for heel and trim
    Real roll, pitch, yaw;
    state.orientation.to_euler(roll, pitch, yaw);
    heel_ = roll;   // Roll angle = heel
    trim_ = pitch;  // Pitch angle = trim

    // Calculate forces and moments
    Vec3 force{0.0, 0.0, 0.0};
    Vec3 torque{0.0, 0.0, 0.0};

    // Buoyancy force acts upward in world frame
    // In body frame, this depends on orientation
    // For NED frame, up is -Z, so buoyancy is negative Z force in world
    // Transform to body frame
    Vec3 buoyancy_world{0.0, 0.0, -buoyancy_force_};  // Upward in NED
    Vec3 buoyancy_body = state.orientation.conjugate().rotate(buoyancy_world);

    force = buoyancy_body;

    // Calculate righting moment from heel (transverse stability)
    // M_roll = W * GM_T * sin(heel)
    // W = displaced weight (approximated by buoyancy at equilibrium)
    Real weight = state.mass * constants::G0;
    Real righting_moment_roll = weight * metacentric_height_ * std::sin(heel_);

    // Righting moment acts to restore upright position
    // Negative roll (port down) -> positive moment (to starboard)
    torque.x = -righting_moment_roll;

    // Calculate righting moment from trim (longitudinal stability)
    // M_pitch = W * GM_L * sin(trim)
    // Assume longitudinal GM is larger than transverse (typical for ships)
    Real gm_longitudinal = metacentric_height_ * 3.0;  // Rule of thumb
    Real righting_moment_pitch = weight * gm_longitudinal * std::sin(trim_);
    torque.y = -righting_moment_pitch;

    // Add moment from center of buoyancy offset
    // Torque = r_CB × F_buoyancy
    Vec3 cb_torque = center_of_buoyancy_.cross(buoyancy_body);
    torque += cb_torque;

    // Damping for roll (important for stability)
    // Simple linear damping based on angular velocity
    constexpr Real ROLL_DAMPING = 10000.0;  // N·m/(rad/s)
    constexpr Real PITCH_DAMPING = 50000.0;
    constexpr Real YAW_DAMPING = 20000.0;

    torque.x -= ROLL_DAMPING * state.angular_velocity.x;
    torque.y -= PITCH_DAMPING * state.angular_velocity.y;
    torque.z -= YAW_DAMPING * state.angular_velocity.z;

    out_forces.add_force(force);
    out_forces.add_torque(torque);
}

// ============================================================================
// SeaState Implementation
// ============================================================================

SeaState SeaState::FromNATOSeaState(int sea_state) {
    // NATO Sea State codes (approximate values)
    // Reference: World Meteorological Organization
    SeaState result;

    switch (sea_state) {
        case 0:  // Calm (glassy)
            result.significant_height = 0.0;
            result.peak_period = 0.0;
            break;
        case 1:  // Calm (rippled)
            result.significant_height = 0.05;
            result.peak_period = 2.0;
            break;
        case 2:  // Smooth (wavelets)
            result.significant_height = 0.2;
            result.peak_period = 3.0;
            break;
        case 3:  // Slight
            result.significant_height = 0.75;
            result.peak_period = 5.0;
            break;
        case 4:  // Moderate
            result.significant_height = 1.5;
            result.peak_period = 6.5;
            break;
        case 5:  // Rough
            result.significant_height = 2.75;
            result.peak_period = 8.0;
            break;
        case 6:  // Very rough
            result.significant_height = 5.0;
            result.peak_period = 10.0;
            break;
        case 7:  // High
            result.significant_height = 8.0;
            result.peak_period = 12.0;
            break;
        case 8:  // Very high
        default:
            result.significant_height = 12.0;
            result.peak_period = 15.0;
            break;
    }

    result.direction = 0.0;  // Default: from north
    result.spectrum = WaveSpectrum::PiersonMoskowitz;

    return result;
}

// ============================================================================
// WaveModel Implementation
// ============================================================================

WaveModel::WaveModel() {
    set_sea_state(SeaState{});
}

WaveModel::~WaveModel() = default;

void WaveModel::set_sea_state(const SeaState& state) {
    sea_state_ = state;
    generate_components();
}

void WaveModel::generate_components() {
    components_.clear();

    // Skip if calm seas
    if (sea_state_.significant_height < 0.01) {
        return;
    }

    // Generate wave components from spectrum
    // Using superposition of sinusoidal waves
    constexpr int NUM_COMPONENTS = 30;
    constexpr Real OMEGA_MIN = 0.3;   // rad/s
    constexpr Real OMEGA_MAX = 2.5;   // rad/s

    Real Hs = sea_state_.significant_height;
    Real Tp = sea_state_.peak_period;
    Real omega_p = (Tp > 0.0) ? (2.0 * constants::PI / Tp) : 1.0;

    // Pierson-Moskowitz or JONSWAP spectrum parameters
    Real alpha = 0.0;
    Real gamma = 1.0;  // PM uses gamma=1

    if (sea_state_.spectrum == WaveSpectrum::PiersonMoskowitz) {
        // PM spectrum: S(omega) = (alpha * g^2 / omega^5) * exp(-beta * (omega_p/omega)^4)
        alpha = 0.0081;  // Phillips constant
        gamma = 1.0;
    } else if (sea_state_.spectrum == WaveSpectrum::JONSWAP) {
        // JONSWAP: PM * gamma^r, where r depends on omega vs omega_p
        alpha = 0.0081;
        gamma = 3.3;  // Average JONSWAP peak enhancement
    }

    Real d_omega = (OMEGA_MAX - OMEGA_MIN) / NUM_COMPONENTS;

    for (int i = 0; i < NUM_COMPONENTS; ++i) {
        Real omega = OMEGA_MIN + (static_cast<Real>(i) + 0.5) * d_omega;

        // Calculate spectral density
        Real S = 0.0;

        if (sea_state_.spectrum == WaveSpectrum::PiersonMoskowitz ||
            sea_state_.spectrum == WaveSpectrum::JONSWAP) {
            // PM spectrum formula
            Real g = constants::G0;
            Real omega_4 = omega * omega * omega * omega;
            Real omega_5 = omega_4 * omega;
            Real pm_part = (alpha * g * g / omega_5) *
                          std::exp(-1.25 * std::pow(omega_p / omega, 4.0));

            // JONSWAP peak enhancement
            Real sigma = (omega <= omega_p) ? 0.07 : 0.09;
            Real r = std::exp(-std::pow(omega - omega_p, 2.0) /
                             (2.0 * sigma * sigma * omega_p * omega_p));
            Real jonswap_factor = std::pow(gamma, r);

            S = pm_part * jonswap_factor;
        } else {
            // Bretschneider parameterized spectrum
            Real A = 173.0 * Hs * Hs / (Tp * Tp * Tp * Tp);
            Real B = 691.0 / (Tp * Tp * Tp * Tp);
            Real omega_4 = omega * omega * omega * omega;
            Real omega_5 = omega_4 * omega;
            S = A / omega_5 * std::exp(-B / omega_4);
        }

        // Wave amplitude from spectral density
        // a_i = sqrt(2 * S(omega_i) * d_omega)
        Real amplitude = std::sqrt(2.0 * S * d_omega);

        // Random phase (using simple deterministic pseudo-random for reproducibility)
        Real phase = std::fmod(omega * 12345.6789, 2.0 * constants::PI);

        // Wave direction (add some spreading around primary direction)
        Real spread = 0.3 * std::sin(omega * 7.89);  // ±17 degrees
        Real direction = sea_state_.direction + spread;

        components_.push_back({amplitude, omega, direction, phase});
    }
}

Real WaveModel::get_elevation(Real x, Real y, Real time) const {
    Real elevation = 0.0;

    for (const auto& wave : components_) {
        // Wave number from dispersion relation: omega^2 = g * k (deep water)
        Real k = wave.frequency * wave.frequency / constants::G0;

        // Wave direction components
        Real kx = k * std::cos(wave.direction);
        Real ky = k * std::sin(wave.direction);

        // Elevation: eta = a * cos(kx*x + ky*y - omega*t + phase)
        Real theta = kx * x + ky * y - wave.frequency * time + wave.phase;
        elevation += wave.amplitude * std::cos(theta);
    }

    return elevation;
}

Vec3 WaveModel::get_particle_velocity(Real x, Real y, Real z, Real time) const {
    Vec3 velocity{0.0, 0.0, 0.0};

    for (const auto& wave : components_) {
        Real omega = wave.frequency;
        Real k = omega * omega / constants::G0;
        Real a = wave.amplitude;

        Real kx = k * std::cos(wave.direction);
        Real ky = k * std::sin(wave.direction);

        Real theta = kx * x + ky * y - omega * time + wave.phase;

        // Deep water particle velocities (z measured from surface, negative down)
        Real decay = std::exp(k * z);  // z < 0, so this decays with depth

        // Horizontal velocities in wave direction
        Real u_wave = a * omega * decay * std::cos(theta);
        Real w_wave = a * omega * decay * std::sin(theta);

        // Decompose into x, y components
        velocity.x += u_wave * std::cos(wave.direction);
        velocity.y += u_wave * std::sin(wave.direction);
        velocity.z += w_wave;
    }

    return velocity;
}

Vec3 WaveModel::get_slope(Real x, Real y, Real time) const {
    Vec3 slope{0.0, 0.0, 0.0};

    for (const auto& wave : components_) {
        Real k = wave.frequency * wave.frequency / constants::G0;
        Real a = wave.amplitude;

        Real kx = k * std::cos(wave.direction);
        Real ky = k * std::sin(wave.direction);

        Real theta = kx * x + ky * y - wave.frequency * time + wave.phase;

        // Slope = d(eta)/dx, d(eta)/dy
        // d(eta)/dx = -a * kx * sin(theta)
        slope.x += -a * kx * std::sin(theta);
        slope.y += -a * ky * std::sin(theta);
    }

    return slope;
}

// ============================================================================
// RAOModel Implementation
// ============================================================================

RAOModel::RAOModel() = default;
RAOModel::~RAOModel() = default;

void RAOModel::set_rao(int dof,
                       const std::vector<Real>& frequencies,
                       const std::vector<Real>& amplitudes,
                       const std::vector<Real>& phases)
{
    if (dof < 0 || dof >= 6) return;

    rao_data_[dof].frequencies = frequencies;
    rao_data_[dof].amplitudes = amplitudes;
    rao_data_[dof].phases = phases;
}

void RAOModel::get_response(Real omega, Real wave_amp,
                            Real& out_amplitude, Real& out_phase, int dof) const
{
    if (dof < 0 || dof >= 6) {
        out_amplitude = 0.0;
        out_phase = 0.0;
        return;
    }

    const auto& data = rao_data_[dof];

    if (data.frequencies.empty()) {
        // No RAO data - assume unity response
        out_amplitude = wave_amp;
        out_phase = 0.0;
        return;
    }

    // Linear interpolation in RAO table
    // Find bracketing frequencies
    SizeT n = data.frequencies.size();
    if (omega <= data.frequencies[0]) {
        out_amplitude = wave_amp * data.amplitudes[0];
        out_phase = data.phases[0];
        return;
    }
    if (omega >= data.frequencies[n-1]) {
        out_amplitude = wave_amp * data.amplitudes[n-1];
        out_phase = data.phases[n-1];
        return;
    }

    // Find interval
    for (SizeT i = 0; i < n - 1; ++i) {
        if (omega >= data.frequencies[i] && omega < data.frequencies[i+1]) {
            Real t = (omega - data.frequencies[i]) /
                    (data.frequencies[i+1] - data.frequencies[i]);
            Real amp_interp = data.amplitudes[i] + t * (data.amplitudes[i+1] - data.amplitudes[i]);
            Real phase_interp = data.phases[i] + t * (data.phases[i+1] - data.phases[i]);

            out_amplitude = wave_amp * amp_interp;
            out_phase = phase_interp;
            return;
        }
    }

    out_amplitude = wave_amp;
    out_phase = 0.0;
}

void RAOModel::calculate_response(const WaveModel& waves, Real time,
                                  Vec3& out_disp, Vec3& out_rot)
{
    out_disp = Vec3{0.0, 0.0, 0.0};
    out_rot = Vec3{0.0, 0.0, 0.0};

    // Get wave components from the model
    const auto& components = waves.get_components();

    if (components.empty()) {
        return;
    }

    // Response accumulator arrays for 6 DOF
    // DOF: 0=surge, 1=sway, 2=heave, 3=roll, 4=pitch, 5=yaw
    Real response[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    // Sum response from each wave component using superposition
    for (const auto& wave : components) {
        Real omega = wave.frequency;
        Real wave_amp = wave.amplitude;
        Real wave_phase = wave.phase - omega * time;  // Time-dependent phase

        // Wave encounter angle (direction wave is coming FROM relative to ship heading)
        // Assuming ship heading is 0 (north), wave direction is measured from north
        Real encounter_angle = wave.direction;

        // For each DOF, get RAO response and accumulate
        for (int dof = 0; dof < 6; ++dof) {
            Real rao_amp, rao_phase;
            get_response(omega, 1.0, rao_amp, rao_phase, dof);

            // Total response amplitude at this frequency
            Real resp_amp = wave_amp * rao_amp;

            // Encounter frequency modification for translational DOFs
            // This accounts for how wave direction affects different motions
            Real direction_factor = 1.0;
            switch (dof) {
                case 0:  // Surge - longitudinal motion
                    direction_factor = std::cos(encounter_angle);
                    break;
                case 1:  // Sway - lateral motion
                    direction_factor = std::sin(encounter_angle);
                    break;
                case 2:  // Heave - vertical motion
                    direction_factor = 1.0;  // Heave is largely omnidirectional
                    break;
                case 3:  // Roll - affected by beam seas
                    direction_factor = std::abs(std::sin(encounter_angle));
                    break;
                case 4:  // Pitch - affected by head/following seas
                    direction_factor = std::abs(std::cos(encounter_angle));
                    break;
                case 5:  // Yaw - affected by quartering seas
                    direction_factor = std::abs(std::sin(2.0 * encounter_angle)) * 0.5;
                    break;
            }

            // Accumulate sinusoidal response
            Real total_phase = wave_phase + rao_phase;
            response[dof] += resp_amp * direction_factor * std::cos(total_phase);
        }
    }

    // Extract translation and rotation responses
    out_disp.x = response[0];  // Surge
    out_disp.y = response[1];  // Sway
    out_disp.z = response[2];  // Heave

    out_rot.x = response[3];   // Roll
    out_rot.y = response[4];   // Pitch
    out_rot.z = response[5];   // Yaw
}

} // namespace jaguar::domain::sea
