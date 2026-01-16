#pragma once
/**
 * @file force.h
 * @brief Abstract force generator interfaces
 *
 * Defines the base interfaces for all force-generating components
 * in the physics simulation. Following JSBSim's component model.
 */

#include "jaguar/core/types.h"
#include "jaguar/physics/entity.h"
#include <memory>
#include <string>
#include <vector>

namespace jaguar {

// Forward declarations
namespace environment {
    struct Environment;
}

namespace physics {

// ============================================================================
// Force Generator Interface
// ============================================================================

/**
 * @brief Abstract interface for force-generating components
 *
 * All physics models that produce forces/torques (aerodynamics,
 * propulsion, gravity, etc.) implement this interface.
 */
class IForceGenerator {
public:
    virtual ~IForceGenerator() = default;

    /**
     * @brief Compute forces and torques for an entity
     *
     * @param state Current entity state
     * @param env Environment conditions
     * @param dt Time step (seconds)
     * @param out_forces Output forces and torques (accumulated)
     */
    virtual void compute_forces(
        const EntityState& state,
        const environment::Environment& env,
        Real dt,
        EntityForces& out_forces) = 0;

    /**
     * @brief Initialize the force generator
     * @return true if initialization successful
     */
    virtual bool initialize() { return true; }

    /**
     * @brief Get the name of this force generator
     */
    virtual const std::string& name() const = 0;

    /**
     * @brief Get the domain this generator applies to
     */
    virtual Domain domain() const = 0;

    /**
     * @brief Check if this generator is enabled
     */
    virtual bool is_enabled() const { return enabled_; }

    /**
     * @brief Enable/disable this generator
     */
    virtual void set_enabled(bool enabled) { enabled_ = enabled; }

protected:
    bool enabled_{true};
};

// ============================================================================
// Specialized Force Generator Types
// ============================================================================

/**
 * @brief Force generator for aerodynamic forces (lift, drag, moments)
 */
class IAerodynamicsModel : public IForceGenerator {
public:
    Domain domain() const override { return Domain::Air; }

    /**
     * @brief Get current lift coefficient
     */
    virtual Real get_cl() const = 0;

    /**
     * @brief Get current drag coefficient
     */
    virtual Real get_cd() const = 0;

    /**
     * @brief Get current pitching moment coefficient
     */
    virtual Real get_cm() const = 0;

    /**
     * @brief Get current angle of attack (radians)
     */
    virtual Real get_alpha() const = 0;

    /**
     * @brief Get current sideslip angle (radians)
     */
    virtual Real get_beta() const = 0;

    /**
     * @brief Get current Mach number
     */
    virtual Real get_mach() const = 0;

    /**
     * @brief Get current dynamic pressure (Pa)
     */
    virtual Real get_qbar() const = 0;
};

/**
 * @brief Force generator for propulsion (engines, rockets)
 */
class IPropulsionModel : public IForceGenerator {
public:
    Domain domain() const override { return Domain::Air; }  // Can be overridden

    /**
     * @brief Get current thrust (N)
     */
    virtual Real get_thrust() const = 0;

    /**
     * @brief Get fuel flow rate (kg/s)
     */
    virtual Real get_fuel_flow() const = 0;

    /**
     * @brief Get remaining fuel mass (kg)
     */
    virtual Real get_fuel_remaining() const = 0;

    /**
     * @brief Set throttle position (0.0 - 1.0)
     */
    virtual void set_throttle(Real throttle) = 0;

    /**
     * @brief Check if engine is running
     */
    virtual bool is_running() const = 0;
};

/**
 * @brief Force generator for terrain interaction (wheels, tracks)
 */
class ITerramechanicsModel : public IForceGenerator {
public:
    Domain domain() const override { return Domain::Land; }

    /**
     * @brief Get current sinkage depth (m)
     */
    virtual Real get_sinkage() const = 0;

    /**
     * @brief Get current motion resistance (N)
     */
    virtual Real get_motion_resistance() const = 0;

    /**
     * @brief Get current traction force (N)
     */
    virtual Real get_traction() const = 0;

    /**
     * @brief Get slip ratio
     */
    virtual Real get_slip_ratio() const = 0;
};

/**
 * @brief Force generator for hydrodynamic forces (hull, rudder)
 */
class IHydrodynamicsModel : public IForceGenerator {
public:
    Domain domain() const override { return Domain::Sea; }

    /**
     * @brief Get current buoyancy force (N)
     */
    virtual Real get_buoyancy() const = 0;

    /**
     * @brief Get current draft (m)
     */
    virtual Real get_draft() const = 0;

    /**
     * @brief Get current heel angle (rad)
     */
    virtual Real get_heel() const = 0;

    /**
     * @brief Get current trim angle (rad)
     */
    virtual Real get_trim() const = 0;
};

/**
 * @brief Force generator for gravitational forces
 */
class IGravityModel : public IForceGenerator {
public:
    Domain domain() const override { return Domain::Generic; }

    /**
     * @brief Get gravitational acceleration at current position (m/s²)
     */
    virtual Vec3 get_gravity_acceleration() const = 0;

    /**
     * @brief Set gravity model fidelity level
     * @param level 0=point mass, 1=J2, 2=J2-J4, 3=full EGM96
     */
    virtual void set_fidelity(int level) = 0;
};

// ============================================================================
// Force Generator Registry
// ============================================================================

/**
 * @brief Registry for managing force generators
 */
class ForceGeneratorRegistry {
public:
    ForceGeneratorRegistry() = default;
    ~ForceGeneratorRegistry() = default;

    /**
     * @brief Register a force generator
     */
    void register_generator(std::unique_ptr<IForceGenerator> generator);

    /**
     * @brief Get all generators for a specific domain
     */
    std::vector<IForceGenerator*> get_generators(Domain domain);

    /**
     * @brief Get all enabled generators
     */
    std::vector<IForceGenerator*> get_enabled_generators();

    /**
     * @brief Find generator by name
     */
    IForceGenerator* find(const std::string& name);

    /**
     * @brief Clear all generators
     */
    void clear();

    /**
     * @brief Get total number of registered generators
     */
    SizeT size() const { return generators_.size(); }

private:
    std::vector<std::unique_ptr<IForceGenerator>> generators_;
};

// ============================================================================
// Force Generator Factory Functions
// ============================================================================

namespace force {

/**
 * @brief Create a simple constant gravity model
 * @param g Gravitational acceleration magnitude (default: 9.80665 m/s²)
 */
std::unique_ptr<IGravityModel> create_simple_gravity(Real g = 9.80665);

/**
 * @brief Create WGS84 gravity model with J2 perturbation
 */
std::unique_ptr<IGravityModel> create_wgs84_gravity();

/**
 * @brief Create simple aerodynamics model
 */
std::unique_ptr<IAerodynamicsModel> create_simple_aerodynamics();

/**
 * @brief Create drag-only model for simple projectiles
 * @param cd Drag coefficient
 * @param area Reference area (m²)
 */
std::unique_ptr<IForceGenerator> create_drag_model(Real cd = 0.5, Real area = 1.0);

} // namespace force

} // namespace physics
} // namespace jaguar
