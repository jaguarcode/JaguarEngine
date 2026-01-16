/**
 * @file force_generators.cpp
 * @brief Force generator implementations
 *
 * Implements concrete force generators for gravity, basic aerodynamics,
 * and the force generator registry.
 */

#include "jaguar/physics/force.h"
#include "jaguar/environment/environment.h"
#include <cmath>
#include <algorithm>

namespace jaguar::physics {

// ============================================================================
// ForceGeneratorRegistry Implementation
// ============================================================================

void ForceGeneratorRegistry::register_generator(std::unique_ptr<IForceGenerator> generator) {
    if (generator) {
        generators_.push_back(std::move(generator));
    }
}

std::vector<IForceGenerator*> ForceGeneratorRegistry::get_generators(Domain domain) {
    std::vector<IForceGenerator*> result;
    for (auto& gen : generators_) {
        if (gen->domain() == domain || domain == Domain::Generic) {
            result.push_back(gen.get());
        }
    }
    return result;
}

std::vector<IForceGenerator*> ForceGeneratorRegistry::get_enabled_generators() {
    std::vector<IForceGenerator*> result;
    for (auto& gen : generators_) {
        if (gen->is_enabled()) {
            result.push_back(gen.get());
        }
    }
    return result;
}

IForceGenerator* ForceGeneratorRegistry::find(const std::string& name) {
    for (auto& gen : generators_) {
        if (gen->name() == name) {
            return gen.get();
        }
    }
    return nullptr;
}

void ForceGeneratorRegistry::clear() {
    generators_.clear();
}

// ============================================================================
// Simple Gravity Model
// ============================================================================

/**
 * @brief Simple constant gravity model
 *
 * Provides constant gravitational acceleration. For higher fidelity,
 * use the WGS84GravityModel or EGM96GravityModel.
 */
class SimpleGravityModel : public IGravityModel {
public:
    explicit SimpleGravityModel(Real g = constants::G0)
        : gravity_magnitude_(g) {}

    void compute_forces(
        const EntityState& state,
        const environment::Environment& env,
        [[maybe_unused]] Real dt,
        EntityForces& out_forces) override
    {
        // Use environment gravity if available, otherwise use configured value
        Vec3 gravity = env.gravity;
        if (gravity.length_squared() < 1e-10) {
            gravity = Vec3{0.0, 0.0, -gravity_magnitude_};
        }

        // F = m * g (in world/ECEF frame)
        out_forces.add_force(gravity * state.mass);

        // Store current gravity for queries
        current_gravity_ = gravity;
    }

    const std::string& name() const override {
        static std::string n = "SimpleGravity";
        return n;
    }

    Domain domain() const override { return Domain::Generic; }

    Vec3 get_gravity_acceleration() const override {
        return current_gravity_;
    }

    void set_fidelity([[maybe_unused]] int level) override {
        // Simple model ignores fidelity level
    }

    void set_gravity_magnitude(Real g) {
        gravity_magnitude_ = g;
    }

private:
    Real gravity_magnitude_;
    Vec3 current_gravity_{0.0, 0.0, -constants::G0};
    std::string name_{"SimpleGravity"};
};

// ============================================================================
// WGS84 Gravity Model (J2 perturbation)
// ============================================================================

/**
 * @brief WGS84 gravity model with J2 perturbation
 *
 * Computes gravitational acceleration including Earth oblateness effects
 * using the J2 zonal harmonic coefficient.
 */
class WGS84GravityModel : public IGravityModel {
public:
    WGS84GravityModel() = default;

    void compute_forces(
        const EntityState& state,
        [[maybe_unused]] const environment::Environment& env,
        [[maybe_unused]] Real dt,
        EntityForces& out_forces) override
    {
        // Compute gravity based on position
        current_gravity_ = compute_gravity(state.position);

        // F = m * g
        out_forces.add_force(current_gravity_ * state.mass);
    }

    const std::string& name() const override {
        static std::string n = "WGS84Gravity";
        return n;
    }

    Domain domain() const override { return Domain::Generic; }

    Vec3 get_gravity_acceleration() const override {
        return current_gravity_;
    }

    void set_fidelity(int level) override {
        fidelity_level_ = std::clamp(level, 0, 2);
    }

private:
    Vec3 compute_gravity(const Vec3& pos_ecef) const {
        Real r = pos_ecef.length();
        if (r < 1e-3) {
            return Vec3{0.0, 0.0, -constants::G0};
        }

        Real r2 = r * r;
        Real r3 = r2 * r;

        // Point mass term: -GM/r² * r_hat
        Real gm_r3 = constants::MU_EARTH / r3;
        Vec3 g_point = pos_ecef * (-gm_r3);

        if (fidelity_level_ == 0) {
            return g_point;
        }

        // J2 perturbation
        Real z = pos_ecef.z;
        Real z2 = z * z;
        Real re2 = constants::R_EARTH_EQUATOR * constants::R_EARTH_EQUATOR;

        // J2 coefficient (Earth oblateness)
        constexpr Real J2 = 1.08263e-3;

        Real factor = 1.5 * J2 * re2 / r2;
        Real z2_r2 = z2 / r2;

        Vec3 g_j2;
        g_j2.x = gm_r3 * pos_ecef.x * factor * (5.0 * z2_r2 - 1.0);
        g_j2.y = gm_r3 * pos_ecef.y * factor * (5.0 * z2_r2 - 1.0);
        g_j2.z = gm_r3 * pos_ecef.z * factor * (5.0 * z2_r2 - 3.0);

        return g_point - g_j2;
    }

    Vec3 current_gravity_{0.0, 0.0, -constants::G0};
    int fidelity_level_{1};  // Default to J2
};

// ============================================================================
// Simple Aerodynamics Model
// ============================================================================

/**
 * @brief Simple aerodynamics model using coefficients
 *
 * Computes lift, drag, and pitching moment from aerodynamic coefficients.
 * Suitable for missiles, simple aircraft, or as a placeholder.
 */
class SimpleAerodynamicsModel : public IAerodynamicsModel {
public:
    struct AeroCoefficients {
        Real cl0{0.0};      ///< Zero-lift coefficient
        Real cl_alpha{5.7}; ///< Lift curve slope (per rad)
        Real cd0{0.02};     ///< Zero-lift drag coefficient
        Real k{0.05};       ///< Induced drag factor (CD = CD0 + k*CL²)
        Real cm0{0.0};      ///< Zero-lift pitching moment
        Real cm_alpha{-0.5}; ///< Pitch stiffness (per rad)
        Real cm_q{-10.0};   ///< Pitch damping (per rad/s normalized)
    };

    struct GeometryParams {
        Real reference_area{1.0};    ///< Reference area (m²)
        Real reference_chord{1.0};   ///< Reference chord (m)
        Real reference_span{1.0};    ///< Reference span (m)
    };

    SimpleAerodynamicsModel() = default;

    void set_coefficients(const AeroCoefficients& coef) {
        coefficients_ = coef;
    }

    void set_geometry(const GeometryParams& geom) {
        geometry_ = geom;
    }

    void compute_forces(
        const EntityState& state,
        const environment::Environment& env,
        [[maybe_unused]] Real dt,
        EntityForces& out_forces) override
    {
        // Get airspeed relative to wind
        Vec3 wind = env.atmosphere.wind;
        Vec3 airspeed_body = compute_airspeed_body(state, wind);

        Real V = airspeed_body.length();
        if (V < 1.0) {
            // Too slow for meaningful aerodynamics
            clear_cached_values();
            return;
        }

        // Compute aerodynamic angles
        alpha_ = std::atan2(-airspeed_body.z, airspeed_body.x);
        beta_ = std::asin(airspeed_body.y / V);

        // Dynamic pressure: q = 0.5 * rho * V²
        Real rho = env.atmosphere.density;
        qbar_ = 0.5 * rho * V * V;

        // Mach number
        mach_ = V / env.atmosphere.speed_of_sound;

        // Compute coefficients
        cl_ = coefficients_.cl0 + coefficients_.cl_alpha * alpha_;
        cd_ = coefficients_.cd0 + coefficients_.k * cl_ * cl_;
        cm_ = coefficients_.cm0 + coefficients_.cm_alpha * alpha_;

        // Add pitch damping
        Real q_normalized = state.angular_velocity.y * geometry_.reference_chord / (2.0 * V);
        cm_ += coefficients_.cm_q * q_normalized;

        // Compute forces in stability frame
        Real S = geometry_.reference_area;
        Real L = qbar_ * S * cl_;  // Lift
        Real D = qbar_ * S * cd_;  // Drag

        // Transform to body frame
        // In stability frame: X is along airspeed, Z is perpendicular
        Real cos_alpha = std::cos(alpha_);
        Real sin_alpha = std::sin(alpha_);

        Vec3 force_body;
        force_body.x = -D * cos_alpha + L * sin_alpha;  // Axial force
        force_body.y = 0.0;  // Side force (simplified)
        force_body.z = -D * sin_alpha - L * cos_alpha;  // Normal force

        // Pitching moment
        Real M = qbar_ * S * geometry_.reference_chord * cm_;

        out_forces.add_force(force_body);
        out_forces.add_torque(Vec3{0.0, M, 0.0});
    }

    const std::string& name() const override {
        static std::string n = "SimpleAerodynamics";
        return n;
    }

    Domain domain() const override { return Domain::Air; }

    Real get_cl() const override { return cl_; }
    Real get_cd() const override { return cd_; }
    Real get_cm() const override { return cm_; }
    Real get_alpha() const override { return alpha_; }
    Real get_beta() const override { return beta_; }
    Real get_mach() const override { return mach_; }
    Real get_qbar() const override { return qbar_; }

private:
    Vec3 compute_airspeed_body(const EntityState& state, const Vec3& wind_ned) const {
        // Transform wind from NED to body frame using quaternion
        // For now, assume body frame aligned with NED (simplified)
        // TODO: Proper coordinate transformation

        // Velocity in inertial frame minus wind gives airspeed
        Vec3 airspeed_inertial = state.velocity - wind_ned;

        // Transform to body frame using inverse rotation
        return state.orientation.conjugate().rotate(airspeed_inertial);
    }

    void clear_cached_values() {
        alpha_ = 0.0;
        beta_ = 0.0;
        mach_ = 0.0;
        qbar_ = 0.0;
        cl_ = 0.0;
        cd_ = 0.0;
        cm_ = 0.0;
    }

    AeroCoefficients coefficients_;
    GeometryParams geometry_;

    // Cached values
    Real alpha_{0.0};
    Real beta_{0.0};
    Real mach_{0.0};
    Real qbar_{0.0};
    Real cl_{0.0};
    Real cd_{0.0};
    Real cm_{0.0};
};

// ============================================================================
// Drag-Only Aerodynamics (for simple projectiles)
// ============================================================================

/**
 * @brief Simple drag-only aerodynamics model
 *
 * Computes only drag force opposing velocity. Suitable for
 * simple projectiles, parachutes, or atmospheric entry.
 */
class DragModel : public IForceGenerator {
public:
    DragModel(Real drag_coefficient = 0.5, Real reference_area = 1.0)
        : cd_(drag_coefficient), ref_area_(reference_area) {}

    void compute_forces(
        const EntityState& state,
        const environment::Environment& env,
        [[maybe_unused]] Real dt,
        EntityForces& out_forces) override
    {
        Real V = state.velocity.length();
        if (V < 0.01) return;  // No drag at very low speeds

        // Dynamic pressure
        Real qbar = 0.5 * env.atmosphere.density * V * V;

        // Drag force magnitude
        Real drag_magnitude = qbar * ref_area_ * cd_;

        // Drag opposes velocity
        Vec3 velocity_dir = state.velocity * (1.0 / V);
        Vec3 drag_force = velocity_dir * (-drag_magnitude);

        out_forces.add_force(drag_force);
    }

    const std::string& name() const override {
        static std::string n = "DragModel";
        return n;
    }

    Domain domain() const override { return Domain::Air; }

    void set_drag_coefficient(Real cd) { cd_ = cd; }
    void set_reference_area(Real area) { ref_area_ = area; }

    Real get_drag_coefficient() const { return cd_; }
    Real get_reference_area() const { return ref_area_; }

private:
    Real cd_;
    Real ref_area_;
};

// ============================================================================
// Factory Functions
// ============================================================================

namespace force {

std::unique_ptr<IGravityModel> create_simple_gravity(Real g) {
    return std::make_unique<SimpleGravityModel>(g);
}

std::unique_ptr<IGravityModel> create_wgs84_gravity() {
    return std::make_unique<WGS84GravityModel>();
}

std::unique_ptr<IAerodynamicsModel> create_simple_aerodynamics() {
    return std::make_unique<SimpleAerodynamicsModel>();
}

std::unique_ptr<IForceGenerator> create_drag_model(Real cd, Real area) {
    return std::make_unique<DragModel>(cd, area);
}

} // namespace force

} // namespace jaguar::physics
