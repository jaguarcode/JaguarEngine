#pragma once
/**
 * @file constraint.h
 * @brief Constraint system types and interfaces
 *
 * Defines the constraint framework for multi-body joint systems in JaguarEngine.
 * Constraints represent physical connections between entities that restrict
 * their relative motion (joints, contacts, limits).
 *
 * The constraint solver uses an iterative impulse-based approach (Sequential
 * Impulse / Projected Gauss-Seidel) to satisfy constraints while maintaining
 * stability and performance.
 */

#include "jaguar/core/types.h"
#include "jaguar/physics/entity.h"
#include <functional>
#include <memory>
#include <string>

namespace jaguar::physics {

// Forward declarations
struct Entity;
struct EntityState;

/**
 * @brief Enumeration of constraint types
 *
 * Each type represents a different kind of physical connection
 * with specific degrees of freedom (DOF) restrictions.
 */
enum class ConstraintType : UInt8 {
    // Basic constraints
    Distance,       ///< Fixed distance between two points (1 DOF restricted)
    BallSocket,     ///< Ball-and-socket joint (3 DOF restricted, rotation free)
    Hinge,          ///< Hinge/revolute joint (5 DOF restricted, 1 rotation axis)
    Slider,         ///< Prismatic joint (5 DOF restricted, 1 translation axis)
    Fixed,          ///< Fixed/weld joint (6 DOF restricted)

    // Contact constraints
    Contact,        ///< Non-penetration constraint (normal direction)
    Friction,       ///< Friction constraint (tangent direction)

    // Limit constraints
    DistanceLimit,  ///< Distance within min/max range
    AngleLimit,     ///< Angle within min/max range

    // Advanced joints
    Universal,      ///< Universal joint (4 DOF restricted, 2 rotation axes)
    Cone,           ///< Cone limit (swing limits for ball socket)
    Motor,          ///< Motor constraint (applies torque/force to reach target)

    // Custom
    Custom          ///< User-defined constraint
};

/**
 * @brief Constraint state flags
 */
enum class ConstraintFlags : UInt32 {
    None            = 0,
    Enabled         = 1 << 0,   ///< Constraint is active
    Broken          = 1 << 1,   ///< Constraint has broken (exceeded limits)
    WarmStarted     = 1 << 2,   ///< Using warm starting from previous frame
    Sleeping        = 1 << 3,   ///< Constraint is sleeping (no motion)
    CollisionPair   = 1 << 4,   ///< This is a collision-generated constraint
};

inline ConstraintFlags operator|(ConstraintFlags a, ConstraintFlags b) {
    return static_cast<ConstraintFlags>(static_cast<UInt32>(a) | static_cast<UInt32>(b));
}

inline ConstraintFlags operator&(ConstraintFlags a, ConstraintFlags b) {
    return static_cast<ConstraintFlags>(static_cast<UInt32>(a) & static_cast<UInt32>(b));
}

inline bool has_flag(ConstraintFlags flags, ConstraintFlags flag) {
    return (static_cast<UInt32>(flags) & static_cast<UInt32>(flag)) != 0;
}

/**
 * @brief Unique identifier for constraints
 */
using ConstraintId = UInt32;
constexpr ConstraintId INVALID_CONSTRAINT_ID = std::numeric_limits<ConstraintId>::max();

/**
 * @brief Body reference in a constraint
 *
 * Can reference either a dynamic entity or a static anchor point.
 */
struct ConstraintBody {
    EntityId entity_id{INVALID_ENTITY_ID};  ///< Entity ID (INVALID for static anchor)
    Vec3 local_anchor{Vec3::Zero()};        ///< Attachment point in body local frame
    Vec3 local_axis{Vec3::UnitX()};         ///< Reference axis in body local frame

    /**
     * @brief Check if this is a static anchor (world-fixed point)
     */
    bool is_static() const { return entity_id == INVALID_ENTITY_ID; }

    /**
     * @brief Create a static anchor at world position
     */
    static ConstraintBody Static(const Vec3& world_pos) {
        ConstraintBody body;
        body.entity_id = INVALID_ENTITY_ID;
        body.local_anchor = world_pos;  // For static, local_anchor IS world position
        return body;
    }

    /**
     * @brief Create a dynamic body reference
     */
    static ConstraintBody Dynamic(EntityId id, const Vec3& local_anchor,
                                   const Vec3& local_axis = Vec3::UnitX()) {
        ConstraintBody body;
        body.entity_id = id;
        body.local_anchor = local_anchor;
        body.local_axis = local_axis;
        return body;
    }
};

/**
 * @brief Constraint parameters common to most constraint types
 */
struct ConstraintParams {
    Real stiffness{1.0};           ///< Constraint stiffness (0-1, 1=rigid)
    Real damping{0.0};             ///< Constraint damping coefficient
    Real restitution{0.0};         ///< Bounce coefficient for limits
    Real break_force{0.0};         ///< Force threshold to break (0=unbreakable)
    Real break_torque{0.0};        ///< Torque threshold to break (0=unbreakable)
    int solver_iterations{4};      ///< Iterations for this constraint
    Real slop{0.01};               ///< Allowed penetration/error before correction
};

/**
 * @brief Result of constraint evaluation
 *
 * Contains the Jacobian information needed by the solver to compute
 * corrective impulses.
 */
struct ConstraintRow {
    // Jacobian entries (J = [J_lin_A, J_ang_A, J_lin_B, J_ang_B])
    Vec3 J_linear_A{Vec3::Zero()};     ///< Linear Jacobian for body A
    Vec3 J_angular_A{Vec3::Zero()};    ///< Angular Jacobian for body A
    Vec3 J_linear_B{Vec3::Zero()};     ///< Linear Jacobian for body B
    Vec3 J_angular_B{Vec3::Zero()};    ///< Angular Jacobian for body B

    Real rhs{0.0};                      ///< Right-hand side (bias / error term)
    Real cfm{0.0};                      ///< Constraint force mixing (softness)
    Real erp{0.2};                      ///< Error reduction parameter

    Real lower_limit{-std::numeric_limits<Real>::infinity()};  ///< Min impulse
    Real upper_limit{std::numeric_limits<Real>::infinity()};   ///< Max impulse

    Real accumulated_impulse{0.0};      ///< Accumulated impulse (for warm starting)
    Real effective_mass{0.0};           ///< Computed effective mass (cached)

    /**
     * @brief Compute effective mass for this row
     *
     * M_eff = 1 / (J * M^-1 * J^T + CFM)
     */
    void compute_effective_mass(Real inv_mass_A, const Mat3x3& inv_inertia_A,
                                 Real inv_mass_B, const Mat3x3& inv_inertia_B);
};

/**
 * @brief Base interface for all constraint types
 *
 * A constraint defines a relationship between two bodies that must be
 * satisfied during simulation. Each constraint type implements specific
 * joint mechanics by providing Jacobian computation.
 */
class IConstraint {
public:
    virtual ~IConstraint() = default;

    /**
     * @brief Get constraint type identifier
     */
    virtual ConstraintType type() const = 0;

    /**
     * @brief Get constraint name (for debugging)
     */
    virtual const std::string& name() const = 0;

    /**
     * @brief Get number of constraint rows (DOFs constrained)
     */
    virtual int num_rows() const = 0;

    /**
     * @brief Build constraint rows for the solver
     *
     * Computes the Jacobian matrices and right-hand side (error/bias)
     * for each constraint row based on current body states.
     *
     * @param state_A State of body A
     * @param state_B State of body B
     * @param dt Time step
     * @param[out] rows Output constraint rows
     */
    virtual void build_rows(const EntityState* state_A, const EntityState* state_B,
                            Real dt, ConstraintRow* rows) = 0;

    /**
     * @brief Apply impulse to bodies
     *
     * After solver computes impulses, this applies velocity changes.
     *
     * @param rows Constraint rows with computed impulses
     * @param[in,out] state_A State of body A (modified)
     * @param[in,out] state_B State of body B (modified)
     */
    virtual void apply_impulse(const ConstraintRow* rows,
                               EntityState* state_A, EntityState* state_B);

    /**
     * @brief Get unique constraint ID
     */
    ConstraintId id() const { return id_; }

    /**
     * @brief Set constraint ID
     */
    void set_id(ConstraintId id) { id_ = id; }

    /**
     * @brief Get body A reference
     */
    const ConstraintBody& body_A() const { return body_A_; }

    /**
     * @brief Get body B reference
     */
    const ConstraintBody& body_B() const { return body_B_; }

    /**
     * @brief Get constraint flags
     */
    ConstraintFlags flags() const { return flags_; }

    /**
     * @brief Set constraint flags
     */
    void set_flags(ConstraintFlags flags) { flags_ = flags; }

    /**
     * @brief Check if constraint is enabled
     */
    bool is_enabled() const { return has_flag(flags_, ConstraintFlags::Enabled); }

    /**
     * @brief Enable or disable constraint
     */
    void set_enabled(bool enabled);

    /**
     * @brief Check if constraint is broken
     */
    bool is_broken() const { return has_flag(flags_, ConstraintFlags::Broken); }

    /**
     * @brief Get constraint parameters
     */
    const ConstraintParams& params() const { return params_; }

    /**
     * @brief Set constraint parameters
     */
    void set_params(const ConstraintParams& params) { params_ = params; }

    /**
     * @brief Get accumulated force magnitude (for breakage detection)
     */
    Real get_force_magnitude() const { return force_magnitude_; }

    /**
     * @brief Get accumulated torque magnitude (for breakage detection)
     */
    Real get_torque_magnitude() const { return torque_magnitude_; }

protected:
    IConstraint(const ConstraintBody& body_A, const ConstraintBody& body_B)
        : body_A_(body_A), body_B_(body_B),
          flags_(ConstraintFlags::Enabled) {}

    ConstraintId id_{INVALID_CONSTRAINT_ID};
    ConstraintBody body_A_;
    ConstraintBody body_B_;
    ConstraintFlags flags_;
    ConstraintParams params_;
    Real force_magnitude_{0.0};
    Real torque_magnitude_{0.0};

    /**
     * @brief Helper to compute world anchor positions
     */
    Vec3 get_world_anchor_A(const EntityState* state) const;
    Vec3 get_world_anchor_B(const EntityState* state) const;

    /**
     * @brief Helper to compute world axis
     */
    Vec3 get_world_axis_A(const EntityState* state) const;
    Vec3 get_world_axis_B(const EntityState* state) const;
};

/**
 * @brief Constraint creation callback type
 */
using ConstraintCreateFunc = std::function<std::unique_ptr<IConstraint>(
    const ConstraintBody&, const ConstraintBody&, const ConstraintParams&)>;

} // namespace jaguar::physics
