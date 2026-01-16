/**
 * @file physics_system.cpp
 * @brief Physics system implementation
 */

#include "jaguar/physics/solver.h"
#include <cmath>

namespace jaguar::physics {

// ============================================================================
// Quaternion Utilities Implementation
// ============================================================================

namespace quaternion_utils {

Quat integrate(const Quat& q, const Vec3& omega, Real dt) {
    // First-order quaternion integration
    // q_dot = 0.5 * q * omega (omega as pure quaternion)
    Quat omega_q{0.0, omega.x, omega.y, omega.z};
    Quat q_dot = q * omega_q;

    Quat result{
        q.w + 0.5 * q_dot.w * dt,
        q.x + 0.5 * q_dot.x * dt,
        q.y + 0.5 * q_dot.y * dt,
        q.z + 0.5 * q_dot.z * dt
    };

    return result.normalized();
}

Quat integrate_rk4(const Quat& q, const Vec3& omega, Real dt) {
    // Fourth-order Runge-Kutta quaternion integration
    auto quat_derivative = [](const Quat& q, const Vec3& w) -> Quat {
        Quat omega_q{0.0, w.x, w.y, w.z};
        Quat q_dot = q * omega_q;
        return {q_dot.w * 0.5, q_dot.x * 0.5, q_dot.y * 0.5, q_dot.z * 0.5};
    };

    Quat k1 = quat_derivative(q, omega);

    Quat q2{q.w + k1.w * dt * 0.5, q.x + k1.x * dt * 0.5,
            q.y + k1.y * dt * 0.5, q.z + k1.z * dt * 0.5};
    Quat k2 = quat_derivative(q2, omega);

    Quat q3{q.w + k2.w * dt * 0.5, q.x + k2.x * dt * 0.5,
            q.y + k2.y * dt * 0.5, q.z + k2.z * dt * 0.5};
    Quat k3 = quat_derivative(q3, omega);

    Quat q4{q.w + k3.w * dt, q.x + k3.x * dt,
            q.y + k3.y * dt, q.z + k3.z * dt};
    Quat k4 = quat_derivative(q4, omega);

    Quat result{
        q.w + (k1.w + 2.0*k2.w + 2.0*k3.w + k4.w) * dt / 6.0,
        q.x + (k1.x + 2.0*k2.x + 2.0*k3.x + k4.x) * dt / 6.0,
        q.y + (k1.y + 2.0*k2.y + 2.0*k3.y + k4.y) * dt / 6.0,
        q.z + (k1.z + 2.0*k2.z + 2.0*k3.z + k4.z) * dt / 6.0
    };

    return result.normalized();
}

Quat normalize(const Quat& q) {
    return q.normalized();
}

Quat from_euler(Real roll, Real pitch, Real yaw) {
    return Quat::from_euler(roll, pitch, yaw);
}

void to_euler(const Quat& q, Real& roll, Real& pitch, Real& yaw) {
    q.to_euler(roll, pitch, yaw);
}

Mat3x3 to_rotation_matrix(const Quat& q) {
    return q.to_rotation_matrix();
}

} // namespace quaternion_utils

// ============================================================================
// PhysicsSystem Implementation
// ============================================================================

PhysicsSystem::PhysicsSystem() {
    // Default to RK4 integrator
    propagator_ = std::make_unique<RK4Integrator>();
}

PhysicsSystem::~PhysicsSystem() = default;

void PhysicsSystem::set_propagator(std::unique_ptr<IStatePropagator> propagator) {
    propagator_ = std::move(propagator);
}

void PhysicsSystem::update(EntityManager& entity_manager, Real dt) {
    auto& storage = entity_manager.get_state_storage();

    // NOTE: Forces should already be computed and accumulated by the engine's
    // force calculation phase BEFORE calling this method. Do NOT clear forces here.
    //
    // The physics step flow is:
    // 1. Engine clears forces (storage.clear_forces())
    // 2. Engine computes all forces via ForceGeneratorRegistry
    // 3. Engine calls PhysicsSystem::update() to integrate

    // Integrate all active entities
    entity_manager.for_each([&](Entity& entity) {
        if (entity.active) {
            update_entity(entity, storage, dt);
        }
    });
}

void PhysicsSystem::update_entity(Entity& entity, EntityStateStorage& storage, Real dt) {
    if (!propagator_) return;

    EntityState state = storage.get_state(entity.state_index);
    EntityForces forces = storage.forces(entity.state_index);

    propagator_->integrate(state, forces, dt);

    storage.set_state(entity.state_index, state);
}

} // namespace jaguar::physics
