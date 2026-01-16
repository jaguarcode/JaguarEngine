/**
 * @file wave_spectrum.cpp
 * @brief Ocean manager and wave spectrum models implementation
 *
 * Implements:
 * - OceanManager: Central ocean environment service
 * - WaveModel: Surface wave simulation using spectral methods
 * - SeaState: NATO sea state presets
 */

#include "jaguar/environment/ocean.h"
#include "jaguar/domain/sea.h"
#include <cmath>
#include <random>
#include <algorithm>

namespace jaguar::environment {

// ============================================================================
// OceanManager Implementation
// ============================================================================

struct OceanManager::Impl {
    domain::sea::SeaState sea_state;
    domain::sea::WaveModel wave_model;
    Vec3 current{0.0, 0.0, 0.0};
    Real base_temperature{15.0};
    Real base_salinity{35.0};

    Impl()
    {
        // Initialize with calm sea
        sea_state.significant_height = 0.5;
        sea_state.peak_period = 5.0;
        sea_state.direction = 0.0;
        wave_model.set_sea_state(sea_state);
    }
};

OceanManager::OceanManager()
    : impl_(std::make_unique<Impl>()) {}

OceanManager::~OceanManager() = default;

void OceanManager::set_sea_state(const domain::sea::SeaState& state)
{
    impl_->sea_state = state;
    impl_->wave_model.set_sea_state(state);
}

const domain::sea::SeaState& OceanManager::get_sea_state() const
{
    return impl_->sea_state;
}

OceanState OceanManager::get_state(Real lat, Real lon, Real time) const
{
    OceanState state;

    // Get wave elevation at this position
    // Convert lat/lon to local x/y coordinates (simplified)
    constexpr Real EARTH_RADIUS = 6371000.0;
    Real x = lon * EARTH_RADIUS * std::cos(lat);
    Real y = lat * EARTH_RADIUS;

    state.surface_elevation = impl_->wave_model.get_elevation(x, y, time);

    // Basic ocean properties (could be enhanced with bathymetry data)
    state.water_depth = 1000.0;  // Default deep water
    state.current = impl_->current;
    state.temperature = impl_->base_temperature;
    state.salinity = impl_->base_salinity;

    // Calculate density from temperature and salinity (UNESCO equation simplified)
    state.density = compute_seawater_density(state.temperature, state.salinity);

    return state;
}

Real OceanManager::get_wave_elevation(Real x, Real y, Real time) const
{
    return impl_->wave_model.get_elevation(x, y, time);
}

Vec3 OceanManager::get_wave_slope(Real x, Real y, Real time) const
{
    return impl_->wave_model.get_slope(x, y, time);
}

Vec3 OceanManager::get_particle_velocity(Real x, Real y, Real z, Real time) const
{
    return impl_->wave_model.get_particle_velocity(x, y, z, time);
}

void OceanManager::set_current(const Vec3& current)
{
    impl_->current = current;
}

Vec3 OceanManager::get_current([[maybe_unused]] Real lat,
                               [[maybe_unused]] Real lon,
                               [[maybe_unused]] Real depth) const
{
    // Simple model: uniform current (could be enhanced with depth profile)
    return impl_->current;
}

Real OceanManager::get_density([[maybe_unused]] Real lat,
                               [[maybe_unused]] Real lon,
                               [[maybe_unused]] Real depth) const
{
    // Simple model: uniform density (could be enhanced with depth profile)
    return compute_seawater_density(impl_->base_temperature, impl_->base_salinity);
}

void OceanManager::update([[maybe_unused]] Real dt)
{
    // Wave model uses continuous time, no update needed
    // Could be used for slowly-varying sea state changes
}

Real OceanManager::compute_seawater_density(Real T, Real S) const
{
    // UNESCO equation of state for seawater (simplified)
    // T: temperature in Celsius, S: salinity in ppt
    // Valid for 0-40°C and 0.5-43 ppt

    // Pure water density at temperature T
    Real rho_w = 999.842594 + 6.793952e-2 * T
               - 9.095290e-3 * T * T
               + 1.001685e-4 * T * T * T
               - 1.120083e-6 * T * T * T * T
               + 6.536332e-9 * T * T * T * T * T;

    // Salinity correction
    Real A = 8.24493e-1 - 4.0899e-3 * T + 7.6438e-5 * T * T
           - 8.2467e-7 * T * T * T + 5.3875e-9 * T * T * T * T;
    Real B = -5.72466e-3 + 1.0227e-4 * T - 1.6546e-6 * T * T;
    Real C = 4.8314e-4;

    return rho_w + A * S + B * S * std::sqrt(S) + C * S * S;
}

} // namespace jaguar::environment

// ============================================================================
// Sea Domain Implementation (Wave Model, Sea State)
// ============================================================================

namespace jaguar::domain::sea {

// ============================================================================
// SeaState Presets
// ============================================================================

SeaState SeaState::FromNATOSeaState(int sea_state)
{
    // NATO sea state codes (based on Douglas Sea Scale)
    SeaState state;
    state.spectrum = WaveSpectrum::PiersonMoskowitz;
    state.direction = 0.0;

    switch (sea_state) {
        case 0:  // Calm (glassy)
            state.significant_height = 0.0;
            state.peak_period = 0.0;
            break;
        case 1:  // Calm (rippled)
            state.significant_height = 0.05;
            state.peak_period = 2.0;
            break;
        case 2:  // Smooth (wavelets)
            state.significant_height = 0.15;
            state.peak_period = 3.0;
            break;
        case 3:  // Slight
            state.significant_height = 0.6;
            state.peak_period = 5.0;
            break;
        case 4:  // Moderate
            state.significant_height = 1.5;
            state.peak_period = 6.5;
            break;
        case 5:  // Rough
            state.significant_height = 2.75;
            state.peak_period = 8.0;
            break;
        case 6:  // Very rough
            state.significant_height = 5.0;
            state.peak_period = 10.0;
            break;
        case 7:  // High
            state.significant_height = 8.5;
            state.peak_period = 13.0;
            break;
        case 8:  // Very high
        default:
            state.significant_height = 12.0;
            state.peak_period = 16.0;
            break;
    }

    return state;
}

// ============================================================================
// WaveModel Implementation
// ============================================================================

WaveModel::WaveModel()
{
    // Initialize with calm sea
    sea_state_.significant_height = 0.5;
    sea_state_.peak_period = 5.0;
    sea_state_.direction = 0.0;
    generate_components();
}

WaveModel::~WaveModel() = default;

void WaveModel::set_sea_state(const SeaState& state)
{
    sea_state_ = state;
    generate_components();
}

void WaveModel::generate_components()
{
    components_.clear();

    if (sea_state_.significant_height < 0.01) {
        return;  // Calm sea, no waves
    }

    // Number of wave components for spectral representation
    constexpr int NUM_COMPONENTS = 25;

    // Frequency range
    Real omega_p = 2.0 * constants::PI / sea_state_.peak_period;
    Real omega_min = 0.4 * omega_p;
    Real omega_max = 2.5 * omega_p;
    Real d_omega = (omega_max - omega_min) / static_cast<Real>(NUM_COMPONENTS);

    // Random phase generator
    std::mt19937 rng(42);  // Fixed seed for reproducibility
    std::uniform_real_distribution<Real> phase_dist(0.0, 2.0 * constants::PI);

    // Generate wave components
    for (int i = 0; i < NUM_COMPONENTS; ++i) {
        Real omega = omega_min + (static_cast<Real>(i) + 0.5) * d_omega;

        // Calculate spectrum value
        Real S = compute_spectrum(omega);

        // Convert to amplitude: a = sqrt(2 * S * d_omega)
        Real amplitude = std::sqrt(2.0 * S * d_omega);

        if (amplitude < 1e-4) continue;  // Skip negligible components

        WaveComponent comp;
        comp.amplitude = amplitude;
        comp.frequency = omega;
        comp.direction = sea_state_.direction;
        comp.phase = phase_dist(rng);

        components_.push_back(comp);
    }
}

Real WaveModel::compute_spectrum(Real omega) const
{
    // Pierson-Moskowitz spectrum (fully developed sea)
    // S(omega) = (alpha * g^2 / omega^5) * exp(-beta * (omega_p / omega)^4)

    constexpr Real g = 9.80665;
    constexpr Real beta = 0.74;

    Real omega_p = 2.0 * constants::PI / sea_state_.peak_period;
    Real H_s = sea_state_.significant_height;

    // Modified for given significant wave height
    // For P-M spectrum: H_s = 4 * sqrt(m0), where m0 is zeroth moment
    // This gives: alpha = 5 * H_s^2 * omega_p^4 / (16 * g^2)
    Real alpha_scaled = 5.0 * H_s * H_s * std::pow(omega_p, 4) / (16.0 * g * g);

    if (omega < 0.01) return 0.0;

    Real omega_ratio = omega_p / omega;
    Real S = (alpha_scaled * g * g / std::pow(omega, 5)) *
             std::exp(-beta * std::pow(omega_ratio, 4));

    // Apply JONSWAP peak enhancement if using JONSWAP spectrum
    if (sea_state_.spectrum == WaveSpectrum::JONSWAP) {
        constexpr Real gamma = 3.3;  // Peak enhancement factor
        Real sigma = (omega <= omega_p) ? 0.07 : 0.09;
        Real r = std::exp(-std::pow(omega - omega_p, 2) /
                         (2.0 * sigma * sigma * omega_p * omega_p));
        S *= std::pow(gamma, r);
    }

    return S;
}

Real WaveModel::get_elevation(Real x, Real y, Real time) const
{
    if (components_.empty()) return 0.0;

    Real elevation = 0.0;

    for (const auto& comp : components_) {
        // Wave number from dispersion relation (deep water)
        constexpr Real g = 9.80665;
        Real k = comp.frequency * comp.frequency / g;

        // Wave propagation direction
        Real kx = k * std::cos(comp.direction);
        Real ky = k * std::sin(comp.direction);

        // Superposition of wave components
        // eta = sum( a * cos(kx*x + ky*y - omega*t + phase) )
        Real phase = kx * x + ky * y - comp.frequency * time + comp.phase;
        elevation += comp.amplitude * std::cos(phase);
    }

    return elevation;
}

Vec3 WaveModel::get_slope(Real x, Real y, Real time) const
{
    if (components_.empty()) return Vec3{0.0, 0.0, 0.0};

    Real deta_dx = 0.0;
    Real deta_dy = 0.0;

    for (const auto& comp : components_) {
        constexpr Real g = 9.80665;
        Real k = comp.frequency * comp.frequency / g;

        Real kx = k * std::cos(comp.direction);
        Real ky = k * std::sin(comp.direction);

        Real phase = kx * x + ky * y - comp.frequency * time + comp.phase;
        Real sin_phase = std::sin(phase);

        // d(eta)/dx = -sum( a * kx * sin(phase) )
        deta_dx -= comp.amplitude * kx * sin_phase;
        deta_dy -= comp.amplitude * ky * sin_phase;
    }

    return Vec3{deta_dx, deta_dy, 0.0};
}

Vec3 WaveModel::get_particle_velocity(Real x, Real y, Real z, Real time) const
{
    if (components_.empty()) return Vec3{0.0, 0.0, 0.0};

    Real u = 0.0, v = 0.0, w = 0.0;

    for (const auto& comp : components_) {
        constexpr Real g = 9.80665;
        Real k = comp.frequency * comp.frequency / g;
        Real omega = comp.frequency;

        Real kx = k * std::cos(comp.direction);
        Real ky = k * std::sin(comp.direction);

        Real phase = kx * x + ky * y - omega * time + comp.phase;

        // Deep water particle velocities (z <= 0)
        // u = a * omega * exp(k*z) * cos(direction) * cos(phase)
        // v = a * omega * exp(k*z) * sin(direction) * cos(phase)
        // w = a * omega * exp(k*z) * sin(phase)

        Real decay = std::exp(k * std::min(z, 0.0));  // Decay with depth
        Real cos_phase = std::cos(phase);
        Real sin_phase = std::sin(phase);

        Real vel_mag = comp.amplitude * omega * decay;
        u += vel_mag * std::cos(comp.direction) * cos_phase;
        v += vel_mag * std::sin(comp.direction) * cos_phase;
        w += vel_mag * sin_phase;
    }

    return Vec3{u, v, w};
}

// ============================================================================
// RAOModel Implementation
// ============================================================================

/**
 * RAO (Response Amplitude Operator) represents the transfer function between
 * wave excitation and vessel motion response for each degree of freedom.
 *
 * For a given wave frequency ω, the vessel response in DOF i is:
 *   response_i(t) = Σ[ a_wave * RAO_i(ω) * cos(ω*t + phase_wave + φ_i(ω)) ]
 *
 * where:
 *   - a_wave is the wave amplitude
 *   - RAO_i(ω) is the amplitude transfer function
 *   - φ_i(ω) is the phase transfer function
 *
 * The 6 DOFs are:
 *   0: Surge (longitudinal translation)
 *   1: Sway (lateral translation)
 *   2: Heave (vertical translation)
 *   3: Roll (rotation about longitudinal axis)
 *   4: Pitch (rotation about lateral axis)
 *   5: Yaw (rotation about vertical axis)
 *
 * Reference: Principles of Naval Architecture (SNAME), Seakeeping chapter
 */

RAOModel::RAOModel()
{
    // Initialize with default RAO data for a generic frigate-sized vessel
    // These are approximate values for educational/testing purposes
    initialize_default_rao();
}

RAOModel::~RAOModel() = default;

void RAOModel::initialize_default_rao()
{
    // Default frequency range (rad/s)
    std::vector<Real> default_freqs = {
        0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.2, 1.5, 2.0
    };

    // Heave RAO (DOF 2) - typically near 1.0 at low frequency, resonance peak, then decay
    std::vector<Real> heave_amp = {
        0.95, 0.98, 1.02, 1.15, 1.35, 1.20, 0.85, 0.55, 0.35, 0.18, 0.08, 0.03
    };
    std::vector<Real> heave_phase = {
        0.0, -0.05, -0.15, -0.35, -0.70, -1.20, -1.70, -2.10, -2.40, -2.70, -2.90, -3.05
    };

    // Roll RAO (DOF 3) - sharp resonance, very sensitive to wave direction
    std::vector<Real> roll_amp = {
        0.10, 0.15, 0.25, 0.50, 1.80, 3.50, 2.20, 1.10, 0.60, 0.25, 0.10, 0.04
    };
    std::vector<Real> roll_phase = {
        0.0, -0.10, -0.25, -0.50, -1.00, -1.57, -2.20, -2.60, -2.85, -3.00, -3.10, -3.14
    };

    // Pitch RAO (DOF 4) - similar to heave but phase-shifted
    std::vector<Real> pitch_amp = {
        0.08, 0.12, 0.18, 0.28, 0.42, 0.55, 0.48, 0.35, 0.22, 0.12, 0.05, 0.02
    };
    std::vector<Real> pitch_phase = {
        -1.57, -1.65, -1.80, -2.00, -2.30, -2.60, -2.85, -3.00, -3.10, -3.14, -3.14, -3.14
    };

    // Surge RAO (DOF 0) - coupled with pitch
    std::vector<Real> surge_amp = {
        0.02, 0.04, 0.08, 0.15, 0.22, 0.25, 0.20, 0.14, 0.08, 0.04, 0.02, 0.01
    };
    std::vector<Real> surge_phase = {
        0.0, -0.10, -0.25, -0.45, -0.75, -1.10, -1.50, -1.90, -2.30, -2.70, -2.95, -3.10
    };

    // Sway RAO (DOF 1) - coupled with roll
    std::vector<Real> sway_amp = {
        0.05, 0.08, 0.15, 0.30, 0.55, 0.75, 0.60, 0.40, 0.25, 0.12, 0.05, 0.02
    };
    std::vector<Real> sway_phase = {
        -1.57, -1.60, -1.70, -1.85, -2.10, -2.50, -2.80, -3.00, -3.10, -3.14, -3.14, -3.14
    };

    // Yaw RAO (DOF 5) - small in head seas, larger in quartering seas
    std::vector<Real> yaw_amp = {
        0.01, 0.02, 0.03, 0.05, 0.08, 0.10, 0.08, 0.05, 0.03, 0.02, 0.01, 0.005
    };
    std::vector<Real> yaw_phase = {
        0.0, -0.15, -0.35, -0.60, -1.00, -1.50, -2.00, -2.40, -2.70, -2.95, -3.10, -3.14
    };

    // Set RAO data for each DOF
    set_rao(0, default_freqs, surge_amp, surge_phase);
    set_rao(1, default_freqs, sway_amp, sway_phase);
    set_rao(2, default_freqs, heave_amp, heave_phase);
    set_rao(3, default_freqs, roll_amp, roll_phase);
    set_rao(4, default_freqs, pitch_amp, pitch_phase);
    set_rao(5, default_freqs, yaw_amp, yaw_phase);
}

void RAOModel::set_rao(int dof,
                       const std::vector<Real>& frequencies,
                       const std::vector<Real>& amplitudes,
                       const std::vector<Real>& phases)
{
    if (dof >= 0 && dof < 6) {
        rao_data_[dof].frequencies = frequencies;
        rao_data_[dof].amplitudes = amplitudes;
        rao_data_[dof].phases = phases;
    }
}

void RAOModel::get_response(Real omega, [[maybe_unused]] Real wave_amp,
                            Real& out_amplitude, Real& out_phase, int dof) const
{
    if (dof < 0 || dof >= 6 || rao_data_[dof].frequencies.empty()) {
        out_amplitude = 0.0;
        out_phase = 0.0;
        return;
    }

    const auto& data = rao_data_[dof];

    // Clamp to frequency range
    if (omega <= data.frequencies.front()) {
        out_amplitude = data.amplitudes.front();
        out_phase = data.phases.front();
        return;
    }
    if (omega >= data.frequencies.back()) {
        out_amplitude = data.amplitudes.back();
        out_phase = data.phases.back();
        return;
    }

    // Linear interpolation in RAO table
    auto it = std::lower_bound(data.frequencies.begin(), data.frequencies.end(), omega);
    size_t idx = static_cast<size_t>(std::distance(data.frequencies.begin(), it));

    Real t = (omega - data.frequencies[idx - 1]) /
             (data.frequencies[idx] - data.frequencies[idx - 1]);

    out_amplitude = data.amplitudes[idx - 1] + t * (data.amplitudes[idx] - data.amplitudes[idx - 1]);
    out_phase = data.phases[idx - 1] + t * (data.phases[idx] - data.phases[idx - 1]);
}

void RAOModel::calculate_response(const WaveModel& waves, Real time, Vec3& out_disp, Vec3& out_rot)
{
    /**
     * Calculate 6DOF vessel response to irregular waves using spectral superposition.
     *
     * For each wave component i and each DOF j:
     *   response_j += a_i * RAO_j(ω_i) * cos(ω_i*t + φ_i + ε_j(ω_i))
     *
     * where:
     *   - a_i is the wave component amplitude
     *   - ω_i is the wave component frequency
     *   - φ_i is the wave component phase
     *   - RAO_j(ω_i) is the response amplitude operator
     *   - ε_j(ω_i) is the RAO phase
     */

    out_disp = Vec3{0.0, 0.0, 0.0};
    out_rot = Vec3{0.0, 0.0, 0.0};

    const auto& components = waves.get_components();
    if (components.empty()) {
        return;
    }

    // Arrays for 6DOF response
    Real response[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    // Sum contributions from each wave component
    for (const auto& comp : components) {
        Real omega = comp.frequency;
        Real wave_amp = comp.amplitude;
        Real wave_phase = comp.phase;

        // Calculate response for each DOF
        for (int dof = 0; dof < 6; ++dof) {
            Real rao_amp, rao_phase;
            get_response(omega, wave_amp, rao_amp, rao_phase, dof);

            // Response contribution from this wave component
            // Phase includes: wave phase from spectral decomposition + RAO phase shift
            Real total_phase = -omega * time + wave_phase + rao_phase;
            response[dof] += wave_amp * rao_amp * std::cos(total_phase);
        }
    }

    // Pack into output vectors
    // Translations: surge (x), sway (y), heave (z)
    out_disp.x = response[0];  // Surge
    out_disp.y = response[1];  // Sway
    out_disp.z = response[2];  // Heave

    // Rotations: roll (x), pitch (y), yaw (z)
    out_rot.x = response[3];   // Roll
    out_rot.y = response[4];   // Pitch
    out_rot.z = response[5];   // Yaw
}

} // namespace jaguar::domain::sea
