/**
 * @file aerodynamics.cpp
 * @brief Aerodynamics model implementation
 *
 * Implements a full 6-DOF aerodynamics model with:
 * - Multi-dimensional coefficient tables with interpolation
 * - Body-axis force and moment computations
 * - Support for control surface deflections
 * - Stability and control derivatives
 *
 * Reference: Aircraft flight dynamics texts (Etkin, Stevens & Lewis)
 */

#include "jaguar/domain/air.h"
#include "jaguar/environment/environment.h"
#include "jaguar/core/coordinates.h"
#include <cmath>
#include <algorithm>
#include <vector>
#include <array>

namespace jaguar::domain::air {

// ============================================================================
// AeroTable Implementation
// ============================================================================

/**
 * @brief Multi-dimensional lookup table with linear interpolation
 *
 * Supports up to 6 dimensions (alpha, beta, Mach, altitude, control surfaces)
 * Uses n-dimensional linear interpolation for smooth coefficient transitions.
 */
struct AeroTable::Impl {
    static constexpr int MAX_DIMENSIONS = 6;

    std::array<std::vector<Real>, MAX_DIMENSIONS> breakpoints;
    std::vector<Real> data;
    int num_dimensions{0};

    /**
     * @brief Find the lower bound index and interpolation fraction
     * @param values Breakpoint vector
     * @param x Input value
     * @param idx Output: lower bound index
     * @param frac Output: interpolation fraction (0-1)
     */
    static void find_index(const std::vector<Real>& values, Real x,
                          SizeT& idx, Real& frac) {
        if (values.empty()) {
            idx = 0;
            frac = 0.0;
            return;
        }

        if (x <= values.front()) {
            idx = 0;
            frac = 0.0;
            return;
        }

        if (x >= values.back()) {
            idx = values.size() > 1 ? values.size() - 2 : 0;
            frac = 1.0;
            return;
        }

        // Binary search for lower bound
        auto it = std::lower_bound(values.begin(), values.end(), x);
        if (it == values.begin()) {
            idx = 0;
            frac = 0.0;
        } else {
            idx = static_cast<SizeT>(std::distance(values.begin(), it)) - 1;
            Real x0 = values[idx];
            Real x1 = values[idx + 1];
            frac = (x1 - x0) > 1e-10 ? (x - x0) / (x1 - x0) : 0.0;
        }
    }

    /**
     * @brief Calculate flat array index from multi-dimensional indices
     */
    SizeT calc_flat_index(const std::vector<SizeT>& indices) const {
        SizeT flat_idx = 0;
        SizeT stride = 1;

        for (int i = num_dimensions - 1; i >= 0; --i) {
            flat_idx += indices[static_cast<SizeT>(i)] * stride;
            stride *= breakpoints[static_cast<SizeT>(i)].size();
        }
        return flat_idx;
    }

    /**
     * @brief N-dimensional linear interpolation
     */
    Real interpolate(const std::vector<Real>& inputs) const {
        if (data.empty() || num_dimensions == 0) {
            return 0.0;
        }

        // Find indices and fractions for each dimension
        std::vector<SizeT> lower_indices(static_cast<SizeT>(num_dimensions));
        std::vector<Real> fractions(static_cast<SizeT>(num_dimensions));

        for (int d = 0; d < num_dimensions; ++d) {
            Real input_val = d < static_cast<int>(inputs.size()) ? inputs[static_cast<SizeT>(d)] : 0.0;
            find_index(breakpoints[static_cast<SizeT>(d)], input_val,
                      lower_indices[static_cast<SizeT>(d)], fractions[static_cast<SizeT>(d)]);
        }

        // Multilinear interpolation: 2^n corners
        SizeT num_corners = 1ULL << static_cast<unsigned>(num_dimensions);
        Real result = 0.0;

        for (SizeT corner = 0; corner < num_corners; ++corner) {
            std::vector<SizeT> corner_indices(static_cast<SizeT>(num_dimensions));
            Real weight = 1.0;

            for (int d = 0; d < num_dimensions; ++d) {
                bool use_upper = (corner >> static_cast<unsigned>(d)) & 1ULL;
                SizeT bp_size = breakpoints[static_cast<SizeT>(d)].size();
                SizeT max_idx = bp_size > 0 ? bp_size - 1 : 0;

                if (use_upper) {
                    corner_indices[static_cast<SizeT>(d)] = std::min(lower_indices[static_cast<SizeT>(d)] + 1, max_idx);
                    weight *= fractions[static_cast<SizeT>(d)];
                } else {
                    corner_indices[static_cast<SizeT>(d)] = lower_indices[static_cast<SizeT>(d)];
                    weight *= (1.0 - fractions[static_cast<SizeT>(d)]);
                }
            }

            SizeT flat_idx = calc_flat_index(corner_indices);
            if (flat_idx < data.size()) {
                result += weight * data[flat_idx];
            }
        }

        return result;
    }
};

AeroTable::AeroTable() : impl_(std::make_unique<Impl>()) {}

// Need destructor in .cpp since Impl is incomplete in header
AeroTable::~AeroTable() = default;

void AeroTable::set_breakpoints(int dimension, const std::vector<Real>& values) {
    if (dimension >= 0 && dimension < Impl::MAX_DIMENSIONS) {
        impl_->breakpoints[static_cast<SizeT>(dimension)] = values;
        impl_->num_dimensions = std::max(impl_->num_dimensions, dimension + 1);
    }
}

void AeroTable::set_data(const std::vector<Real>& data) {
    impl_->data = data;
}

Real AeroTable::lookup(const std::vector<Real>& inputs) const {
    return impl_->interpolate(inputs);
}

// ============================================================================
// AerodynamicsModel Implementation
// ============================================================================

namespace {

/**
 * @brief Compute angle of attack from body-axis velocities
 * @param u Forward velocity (body X)
 * @param w Downward velocity (body Z)
 * @return Angle of attack in radians
 */
Real compute_alpha(Real u, Real w) {
    // Prevent division by zero at very low speeds
    if (std::abs(u) < 1.0) {
        return std::atan2(w, std::max(std::abs(u), 1.0));
    }
    return std::atan2(w, u);
}

/**
 * @brief Compute sideslip angle from body-axis velocities
 * @param u Forward velocity (body X)
 * @param v Lateral velocity (body Y)
 * @param V_total Total velocity magnitude
 * @return Sideslip angle in radians
 */
Real compute_beta(Real /*u*/, Real v, Real V_total) {
    if (V_total < 1.0) {
        return 0.0;
    }
    return std::asin(std::clamp(v / V_total, -1.0, 1.0));
}

/**
 * @brief Default CL vs alpha curve (subsonic airfoil)
 * CL = CL_alpha * alpha + CL_0
 */
Real default_cl(Real alpha, Real /*mach*/) {
    constexpr Real CL_ALPHA = 5.5;  // rad^-1 (typical for subsonic)
    constexpr Real CL_0 = 0.0;      // Symmetric airfoil
    constexpr Real CL_MAX = 1.5;
    constexpr Real CL_MIN = -1.5;

    Real cl = CL_0 + CL_ALPHA * alpha;
    return std::clamp(cl, CL_MIN, CL_MAX);
}

/**
 * @brief Default CD vs CL curve (parabolic drag polar)
 * CD = CD_0 + K * CL^2
 */
Real default_cd(Real cl, Real /*mach*/) {
    constexpr Real CD_0 = 0.02;     // Parasitic drag
    constexpr Real AR = 8.0;        // Aspect ratio
    constexpr Real E = 0.85;        // Oswald efficiency
    Real K = 1.0 / (constants::PI * AR * E);

    return CD_0 + K * cl * cl;
}

/**
 * @brief Default CM vs alpha curve (static stability)
 * CM = CM_0 + CM_alpha * alpha
 */
Real default_cm(Real alpha, Real /*mach*/) {
    constexpr Real CM_0 = 0.0;
    constexpr Real CM_ALPHA = -0.5;  // Negative = stable

    return CM_0 + CM_ALPHA * alpha;
}

/**
 * @brief Default CY vs beta curve (sideslip)
 */
Real default_cy(Real beta) {
    constexpr Real CY_BETA = -0.5;  // rad^-1
    return CY_BETA * beta;
}

/**
 * @brief Default Cl (roll moment) vs beta curve
 */
Real default_cl_roll(Real beta) {
    constexpr Real CL_BETA = -0.1;  // Dihedral effect
    return CL_BETA * beta;
}

/**
 * @brief Default Cn (yaw moment) vs beta curve
 */
Real default_cn(Real beta) {
    constexpr Real CN_BETA = 0.1;   // Weathercock stability
    return CN_BETA * beta;
}

} // anonymous namespace

AerodynamicsModel::AerodynamicsModel() = default;
AerodynamicsModel::~AerodynamicsModel() = default;

bool AerodynamicsModel::initialize() {
    // Tables are optional - defaults will be used if not set
    return true;
}

void AerodynamicsModel::compute_forces(
    const physics::EntityState& state,
    const environment::Environment& env,
    Real /*dt*/,
    physics::EntityForces& out_forces)
{
    if (!enabled_) {
        return;
    }

    // Transform velocity from ECEF to body frame
    Vec3 v_ecef = state.velocity;

    // Account for wind
    Vec3 wind_ecef = coord::ned_to_ecef(env.atmosphere.wind,
        GeodeticPosition{env.latitude, env.longitude, env.altitude});
    Vec3 v_air_ecef = v_ecef - wind_ecef;

    // Transform to body frame
    Vec3 v_body = state.orientation.conjugate().rotate(v_air_ecef);

    Real u = v_body.x;  // Forward
    Real v = v_body.y;  // Right
    Real w = v_body.z;  // Down

    // Compute airspeed
    Real V_total = v_body.length();

    // Minimum speed for aerodynamics
    constexpr Real V_MIN = 1.0;  // m/s
    if (V_total < V_MIN) {
        // Store zero coefficients
        cl_ = cd_ = cm_ = 0.0;
        cy_ = cl_roll_ = cn_ = 0.0;
        alpha_ = beta_ = 0.0;
        mach_ = 0.0;
        qbar_ = 0.0;
        return;
    }

    // Compute aerodynamic angles
    alpha_ = compute_alpha(u, w);
    beta_ = compute_beta(u, v, V_total);

    // Compute Mach number
    Real speed_of_sound = env.atmosphere.speed_of_sound;
    mach_ = (speed_of_sound > 0.0) ? V_total / speed_of_sound : 0.0;

    // Compute dynamic pressure: q = 0.5 * rho * V^2
    Real rho = env.atmosphere.density;
    qbar_ = 0.5 * rho * V_total * V_total;

    // Get aerodynamic coefficients
    if (cl_table_) {
        cl_ = cl_table_->lookup({alpha_, mach_});
    } else {
        cl_ = default_cl(alpha_, mach_);
    }

    if (cd_table_) {
        cd_ = cd_table_->lookup({alpha_, mach_});
    } else {
        cd_ = default_cd(cl_, mach_);
    }

    if (cm_table_) {
        cm_ = cm_table_->lookup({alpha_, mach_});
    } else {
        cm_ = default_cm(alpha_, mach_);
    }

    // Lateral-directional coefficients (simplified - use defaults)
    cy_ = default_cy(beta_);
    cl_roll_ = default_cl_roll(beta_);
    cn_ = default_cn(beta_);

    // Add damping derivatives (rate effects)
    Real p = state.angular_velocity.x;  // Roll rate
    Real q = state.angular_velocity.y;  // Pitch rate
    Real r = state.angular_velocity.z;  // Yaw rate

    // Non-dimensional rates
    Real p_hat = (V_total > V_MIN) ? p * b_ref_ / (2.0 * V_total) : 0.0;
    Real q_hat = (V_total > V_MIN) ? q * c_ref_ / (2.0 * V_total) : 0.0;
    Real r_hat = (V_total > V_MIN) ? r * b_ref_ / (2.0 * V_total) : 0.0;

    // Damping contributions (typical values)
    constexpr Real CL_Q = 5.0;     // Pitch rate effect on lift
    constexpr Real CM_Q = -15.0;   // Pitch damping
    constexpr Real CY_R = 0.3;     // Yaw rate effect on side force
    constexpr Real CL_P = -0.4;    // Roll damping
    constexpr Real CN_R = -0.15;   // Yaw damping

    cl_ += CL_Q * q_hat;
    cm_ += CM_Q * q_hat;
    cy_ += CY_R * r_hat;
    cl_roll_ += CL_P * p_hat;
    cn_ += CN_R * r_hat;

    // Convert coefficients to forces and moments
    // All forces in body frame

    // Aerodynamic forces in wind frame
    Real L = qbar_ * s_ref_ * cl_;   // Lift (perpendicular to V)
    Real D = qbar_ * s_ref_ * cd_;   // Drag (opposite to V)
    Real Y = qbar_ * s_ref_ * cy_;   // Side force

    // Transform from wind to body frame
    Real cos_alpha = std::cos(alpha_);
    Real sin_alpha = std::sin(alpha_);
    Real cos_beta = std::cos(beta_);
    Real sin_beta = std::sin(beta_);

    // X-body force (axial force)
    Real X_body = -D * cos_alpha * cos_beta - Y * cos_alpha * sin_beta + L * sin_alpha;

    // Y-body force (side force)
    Real Y_body = -D * sin_beta + Y * cos_beta;

    // Z-body force (normal force)
    Real Z_body = -D * sin_alpha * cos_beta - Y * sin_alpha * sin_beta - L * cos_alpha;

    // Moments in body frame
    Real L_moment = qbar_ * s_ref_ * b_ref_ * cl_roll_;  // Rolling moment
    Real M_moment = qbar_ * s_ref_ * c_ref_ * cm_;       // Pitching moment
    Real N_moment = qbar_ * s_ref_ * b_ref_ * cn_;       // Yawing moment

    // Add to output forces (body frame)
    out_forces.add_force(Vec3{X_body, Y_body, Z_body});
    out_forces.add_torque(Vec3{L_moment, M_moment, N_moment});
}

void AerodynamicsModel::set_cl_table(std::unique_ptr<AeroTable> table) {
    cl_table_ = std::move(table);
}

void AerodynamicsModel::set_cd_table(std::unique_ptr<AeroTable> table) {
    cd_table_ = std::move(table);
}

void AerodynamicsModel::set_cm_table(std::unique_ptr<AeroTable> table) {
    cm_table_ = std::move(table);
}

} // namespace jaguar::domain::air
