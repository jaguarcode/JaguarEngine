/**
 * @file sequential_impulse_solver.cpp
 * @brief Sequential Impulse constraint solver implementation
 *
 * Implements the Sequential Impulse (SI) / Projected Gauss-Seidel (PGS)
 * constraint solver with warm starting and Baumgarte stabilization.
 *
 * Reference: Erin Catto, "Iterative Dynamics with Temporal Coherence"
 */

#include "jaguar/physics/constraints/constraint_solver.h"
#include "jaguar/physics/entity.h"
#include <cmath>
#include <algorithm>
#include <chrono>

namespace jaguar::physics {

// ============================================================================
// SequentialImpulseSolver Implementation
// ============================================================================

SequentialImpulseSolver::SequentialImpulseSolver() = default;

void SequentialImpulseSolver::clear() {
    constraints_.clear();
    warm_start_cache_.clear();
}

void SequentialImpulseSolver::begin_frame(Real dt) {
    dt_ = dt;
    constraints_.clear();
}

void SequentialImpulseSolver::add_constraint(
    IConstraint* constraint,
    EntityState* state_A,
    EntityState* state_B) {

    if (!constraint || !constraint->is_enabled() || constraint->is_broken()) {
        return;
    }

    ConstraintData data;
    data.constraint = constraint;
    data.state_A = state_A;
    data.state_B = state_B;
    data.rows.resize(static_cast<size_t>(constraint->num_rows()));

    // Compute inverse mass and inertia
    compute_inverse_mass(state_A, data.inv_mass_A, data.inv_inertia_A);
    compute_inverse_mass(state_B, data.inv_mass_B, data.inv_inertia_B);

    constraints_.push_back(std::move(data));
}

void SequentialImpulseSolver::compute_inverse_mass(
    const EntityState* state,
    Real& inv_mass,
    Mat3x3& inv_inertia) const {

    if (!state || state->mass < 1e-10) {
        // Static or infinite mass body
        inv_mass = 0.0;
        inv_inertia = Mat3x3::Zero();
        return;
    }

    inv_mass = 1.0 / state->mass;

    // Compute inverse inertia
    Real det = state->inertia.determinant();
    if (std::abs(det) < 1e-10) {
        // Fallback to diagonal inverse
        inv_inertia = Mat3x3::Identity();
        Real Ixx = state->inertia(0, 0);
        Real Iyy = state->inertia(1, 1);
        Real Izz = state->inertia(2, 2);
        if (Ixx > 1e-10) inv_inertia(0, 0) = 1.0 / Ixx;
        else inv_inertia(0, 0) = 0.0;
        if (Iyy > 1e-10) inv_inertia(1, 1) = 1.0 / Iyy;
        else inv_inertia(1, 1) = 0.0;
        if (Izz > 1e-10) inv_inertia(2, 2) = 1.0 / Izz;
        else inv_inertia(2, 2) = 0.0;
    } else {
        inv_inertia = state->inertia.inverse();
    }
}

void SequentialImpulseSolver::build_constraints() {
    for (auto& data : constraints_) {
        // Build constraint rows (Jacobians, RHS, limits)
        data.constraint->build_rows(
            data.state_A, data.state_B, dt_, data.rows.data());

        // Compute effective mass for each row
        for (auto& row : data.rows) {
            row.compute_effective_mass(
                data.inv_mass_A, data.inv_inertia_A,
                data.inv_mass_B, data.inv_inertia_B);
        }
    }
}

void SequentialImpulseSolver::apply_warm_start() {
    if (!config_.warm_starting) return;

    for (auto& data : constraints_) {
        ConstraintId id = data.constraint->id();
        auto it = warm_start_cache_.find(id);

        if (it != warm_start_cache_.end()) {
            const auto& cached_impulses = it->second;
            size_t n = std::min(cached_impulses.size(), data.rows.size());

            for (size_t i = 0; i < n; ++i) {
                // Apply scaled warm start impulse
                Real warm_impulse = cached_impulses[i] * config_.warm_start_factor;
                data.rows[i].accumulated_impulse = warm_impulse;

                // Apply the impulse to velocities
                if (std::abs(warm_impulse) > 1e-10) {
                    const auto& row = data.rows[i];

                    // Body A
                    if (data.state_A && data.inv_mass_A > 0) {
                        data.state_A->velocity = data.state_A->velocity -
                            row.J_linear_A * (warm_impulse * data.inv_mass_A);

                        Vec3 ang_impulse = data.inv_inertia_A * row.J_angular_A * warm_impulse;
                        data.state_A->angular_velocity = data.state_A->angular_velocity - ang_impulse;
                    }

                    // Body B
                    if (data.state_B && data.inv_mass_B > 0) {
                        data.state_B->velocity = data.state_B->velocity +
                            row.J_linear_B * (warm_impulse * data.inv_mass_B);

                        Vec3 ang_impulse = data.inv_inertia_B * row.J_angular_B * warm_impulse;
                        data.state_B->angular_velocity = data.state_B->angular_velocity + ang_impulse;
                    }
                }
            }
        }
    }
}

Real SequentialImpulseSolver::solve_row(ConstraintData& data, int row_index) {
    ConstraintRow& row = data.rows[static_cast<size_t>(row_index)];

    if (row.effective_mass < 1e-10) {
        return 0.0;
    }

    // Compute relative velocity: Jv = J · v
    Real Jv = 0.0;

    // Body A contribution
    if (data.state_A) {
        Jv -= row.J_linear_A.dot(data.state_A->velocity);
        Jv -= row.J_angular_A.dot(data.state_A->angular_velocity);
    }

    // Body B contribution
    if (data.state_B) {
        Jv += row.J_linear_B.dot(data.state_B->velocity);
        Jv += row.J_angular_B.dot(data.state_B->angular_velocity);
    }

    // Compute impulse: λ = M_eff * (rhs - Jv - cfm * λ_accumulated)
    Real lambda = row.effective_mass * (row.rhs - Jv - row.cfm * row.accumulated_impulse);

    // Clamp impulse to limits
    Real old_impulse = row.accumulated_impulse;
    row.accumulated_impulse = std::clamp(
        old_impulse + lambda,
        row.lower_limit,
        row.upper_limit);
    lambda = row.accumulated_impulse - old_impulse;

    // Apply impulse to velocities
    if (std::abs(lambda) > 1e-15) {
        // Body A (negative Jacobian direction)
        if (data.state_A && data.inv_mass_A > 0) {
            data.state_A->velocity = data.state_A->velocity -
                row.J_linear_A * (lambda * data.inv_mass_A);

            Vec3 ang_impulse = data.inv_inertia_A * row.J_angular_A * lambda;
            data.state_A->angular_velocity = data.state_A->angular_velocity - ang_impulse;
        }

        // Body B (positive Jacobian direction)
        if (data.state_B && data.inv_mass_B > 0) {
            data.state_B->velocity = data.state_B->velocity +
                row.J_linear_B * (lambda * data.inv_mass_B);

            Vec3 ang_impulse = data.inv_inertia_B * row.J_angular_B * lambda;
            data.state_B->angular_velocity = data.state_B->angular_velocity + ang_impulse;
        }
    }

    return std::abs(lambda);
}

void SequentialImpulseSolver::solve_velocity_constraints(int iterations) {
    for (int iter = 0; iter < iterations; ++iter) {
        for (auto& data : constraints_) {
            int n = data.constraint->num_rows();
            for (int i = 0; i < n; ++i) {
                solve_row(data, i);
            }
        }
    }
}

void SequentialImpulseSolver::solve_position_constraints(int iterations) {
    // Position correction using Baumgarte stabilization
    // This is already handled in the bias term (rhs) during build_rows
    // If split impulse is enabled, we would separate position correction here

    if (!config_.split_impulse) {
        return;  // Position correction integrated in velocity solve
    }

    // Split impulse: separate position correction pass
    // (More stable for stacking, but more expensive)
    for (int iter = 0; iter < iterations; ++iter) {
        for (auto& data : constraints_) {
            // Rebuild rows with position-only bias
            // This is a simplified version - full implementation would
            // maintain separate pseudo-velocity for position correction
            (void)data;  // TODO: Implement split impulse position correction
        }
    }
}

void SequentialImpulseSolver::store_warm_start() {
    warm_start_cache_.clear();

    for (const auto& data : constraints_) {
        ConstraintId id = data.constraint->id();
        std::vector<Real> impulses;
        impulses.reserve(data.rows.size());

        for (const auto& row : data.rows) {
            impulses.push_back(row.accumulated_impulse);
        }

        warm_start_cache_[id] = std::move(impulses);
    }
}

ConstraintSolverResult SequentialImpulseSolver::solve() {
    auto start_time = std::chrono::high_resolution_clock::now();

    ConstraintSolverResult result;
    result.total_constraints = static_cast<int>(constraints_.size());

    if (constraints_.empty()) {
        return result;
    }

    // Step 1: Build constraint rows
    build_constraints();

    // Step 2: Apply warm starting
    apply_warm_start();

    // Step 3: Velocity iterations
    solve_velocity_constraints(config_.velocity_iterations);

    // Step 4: Position iterations (if split impulse enabled)
    solve_position_constraints(config_.position_iterations);

    // Compute statistics
    result.active_constraints = result.total_constraints;
    result.iterations_used = config_.velocity_iterations;

    Real total_impulse = 0.0;
    Real max_error = 0.0;
    int broken_count = 0;

    for (const auto& data : constraints_) {
        for (const auto& row : data.rows) {
            total_impulse += std::abs(row.accumulated_impulse);
        }

        // Check for broken constraints
        if (data.constraint->is_broken()) {
            broken_count++;
        }
    }

    result.total_impulse = total_impulse;
    result.max_error = max_error;
    result.broken_count = broken_count;

    auto end_time = std::chrono::high_resolution_clock::now();
    result.solve_time_ms = std::chrono::duration<Real, std::milli>(
        end_time - start_time).count();

    return result;
}

void SequentialImpulseSolver::end_frame() {
    // Store warm starting data for next frame
    store_warm_start();

    // Apply final impulses to constraints for force output
    for (auto& data : constraints_) {
        data.constraint->apply_impulse(
            data.rows.data(), data.state_A, data.state_B);
    }
}

// ============================================================================
// ConstraintManager Implementation
// ============================================================================

ConstraintManager::ConstraintManager()
    : solver_(std::make_unique<SequentialImpulseSolver>()) {
}

ConstraintManager::~ConstraintManager() = default;

void ConstraintManager::set_solver(std::unique_ptr<IConstraintSolver> solver) {
    if (solver) {
        solver_ = std::move(solver);
    }
}

ConstraintId ConstraintManager::add_constraint(std::unique_ptr<IConstraint> constraint) {
    if (!constraint) {
        return INVALID_CONSTRAINT_ID;
    }

    ConstraintId id = next_id_++;
    constraint->set_id(id);

    // Track entity associations
    EntityId entity_A = constraint->body_A().entity_id;
    EntityId entity_B = constraint->body_B().entity_id;

    if (entity_A != INVALID_ENTITY_ID) {
        entity_constraints_[entity_A].push_back(id);
    }
    if (entity_B != INVALID_ENTITY_ID) {
        entity_constraints_[entity_B].push_back(id);
    }

    constraints_[id] = std::move(constraint);
    return id;
}

void ConstraintManager::remove_constraint(ConstraintId id) {
    auto it = constraints_.find(id);
    if (it == constraints_.end()) {
        return;
    }

    IConstraint* constraint = it->second.get();

    // Remove from entity tracking
    EntityId entity_A = constraint->body_A().entity_id;
    EntityId entity_B = constraint->body_B().entity_id;

    auto remove_from_entity = [&](EntityId entity_id) {
        if (entity_id == INVALID_ENTITY_ID) return;
        auto eit = entity_constraints_.find(entity_id);
        if (eit != entity_constraints_.end()) {
            auto& vec = eit->second;
            vec.erase(std::remove(vec.begin(), vec.end(), id), vec.end());
            if (vec.empty()) {
                entity_constraints_.erase(eit);
            }
        }
    };

    remove_from_entity(entity_A);
    remove_from_entity(entity_B);

    constraints_.erase(it);
}

IConstraint* ConstraintManager::get_constraint(ConstraintId id) {
    auto it = constraints_.find(id);
    return (it != constraints_.end()) ? it->second.get() : nullptr;
}

std::vector<IConstraint*> ConstraintManager::get_entity_constraints(EntityId entity_id) {
    std::vector<IConstraint*> result;

    auto it = entity_constraints_.find(entity_id);
    if (it != entity_constraints_.end()) {
        for (ConstraintId id : it->second) {
            if (IConstraint* c = get_constraint(id)) {
                result.push_back(c);
            }
        }
    }

    return result;
}

void ConstraintManager::remove_entity_constraints(EntityId entity_id) {
    auto it = entity_constraints_.find(entity_id);
    if (it == entity_constraints_.end()) {
        return;
    }

    // Copy IDs to avoid iterator invalidation
    std::vector<ConstraintId> to_remove = it->second;
    for (ConstraintId id : to_remove) {
        remove_constraint(id);
    }
}

ConstraintSolverResult ConstraintManager::solve(EntityManager& entity_manager, Real dt) {
    if (!solver_) {
        return {};
    }

    solver_->begin_frame(dt);

    // Add all enabled constraints to solver
    for (auto& [id, constraint] : constraints_) {
        if (!constraint->is_enabled() || constraint->is_broken()) {
            continue;
        }

        EntityState* state_A = nullptr;
        EntityState* state_B = nullptr;

        // Get entity states
        EntityId entity_A = constraint->body_A().entity_id;
        EntityId entity_B = constraint->body_B().entity_id;

        if (entity_A != INVALID_ENTITY_ID) {
            state_A = entity_manager.get_entity_state(entity_A);
        }
        if (entity_B != INVALID_ENTITY_ID) {
            state_B = entity_manager.get_entity_state(entity_B);
        }

        solver_->add_constraint(constraint.get(), state_A, state_B);
    }

    // Solve
    ConstraintSolverResult result = solver_->solve();

    solver_->end_frame();

    // Sync modified states back to SoA storage
    entity_manager.sync_entity_states();

    return result;
}

void ConstraintManager::clear() {
    constraints_.clear();
    entity_constraints_.clear();
    if (solver_) {
        solver_->clear();
    }
}

const ConstraintSolverConfig& ConstraintManager::config() const {
    static ConstraintSolverConfig default_config;
    return solver_ ? solver_->config() : default_config;
}

void ConstraintManager::set_config(const ConstraintSolverConfig& config) {
    if (solver_) {
        solver_->set_config(config);
    }
}

} // namespace jaguar::physics
