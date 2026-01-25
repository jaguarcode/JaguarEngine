#pragma once
/**
 * @file constants.h
 * @brief Engine configuration constants
 *
 * Centralizes simulation-specific configuration constants.
 * Physical constants (G, MU_EARTH, etc.) remain in types.h.
 */

#include "jaguar/core/types.h"

namespace jaguar::constants {

// ============================================================================
// Simulation Time Constants
// ============================================================================

/// Default physics time step (100 Hz)
constexpr Real DEFAULT_TIME_STEP = 0.01;

/// Minimum allowed time step
constexpr Real TIME_STEP_MIN = 1e-6;

/// Maximum allowed time step
constexpr Real TIME_STEP_MAX = 0.1;

// ============================================================================
// Physics Thresholds
// ============================================================================

/// Mass threshold below which entities are considered massless
constexpr Real MASS_EPSILON = 1e-10;

/// Velocity threshold for considering entity stationary
constexpr Real VELOCITY_EPSILON = 1e-8;

/// Angular velocity threshold
constexpr Real ANGULAR_VELOCITY_EPSILON = 1e-8;

/// Inertia component threshold
constexpr Real INERTIA_EPSILON = 1e-10;

/// Position epsilon for numerical comparisons (meters)
constexpr Real POSITION_EPSILON = 1e-9;

// ============================================================================
// Constraint Solver Constants
// ============================================================================

/// Default Baumgarte stabilization factor
constexpr Real DEFAULT_BAUMGARTE_FACTOR = 0.2;

/// Default constraint slop (penetration allowed)
constexpr Real DEFAULT_SLOP = 0.01;

/// Default warm start factor
constexpr Real DEFAULT_WARM_START_FACTOR = 0.8;

/// Default convergence threshold for iterative solver
constexpr Real CONVERGENCE_THRESHOLD = 1e-8;

/// Maximum warm start impulse multiplier
constexpr Real MAX_WARM_START_MULTIPLIER = 10.0;

// ============================================================================
// Terrain and Environment
// ============================================================================

/// Default sea level altitude (m)
constexpr Real DEFAULT_SEA_LEVEL = 0.0;

/// Default ground level when terrain unavailable (m)
constexpr Real DEFAULT_GROUND_LEVEL = 0.0;

/// Minimum altitude above terrain (m)
constexpr Real TERRAIN_MIN_CLEARANCE = 0.01;

// ============================================================================
// Adaptive Integration
// ============================================================================

/// Default error tolerance for adaptive integrators
constexpr Real ADAPTIVE_TOLERANCE = 1e-6;

/// Minimum step size for adaptive integrators
constexpr Real ADAPTIVE_MIN_DT = 1e-9;

/// Maximum step size for adaptive integrators
constexpr Real ADAPTIVE_MAX_DT = 0.1;

/// Safety factor for step size adjustment
constexpr Real ADAPTIVE_SAFETY_FACTOR = 0.9;

/// Maximum step growth factor
constexpr Real ADAPTIVE_GROWTH_LIMIT = 2.0;

/// Minimum step shrink factor
constexpr Real ADAPTIVE_SHRINK_LIMIT = 0.1;

// ============================================================================
// Entity Limits
// ============================================================================

/// Maximum number of entities per simulation
constexpr SizeT MAX_ENTITIES = 100000;

/// Maximum constraint rows per constraint
constexpr int MAX_CONSTRAINT_ROWS = 6;

/// Default velocity iterations for constraint solver
constexpr int DEFAULT_VELOCITY_ITERATIONS = 10;

/// Default position iterations for constraint solver
constexpr int DEFAULT_POSITION_ITERATIONS = 3;

} // namespace jaguar::constants
