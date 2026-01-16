#pragma once
/**
 * @file time.h
 * @brief Time management for physics simulation
 *
 * Provides precise time tracking, frame counting, and time scaling
 * for deterministic physics simulation.
 */

#include "jaguar/core/types.h"

namespace jaguar::core {

/**
 * @brief Time management for simulation
 *
 * Handles simulation time tracking with support for:
 * - Fixed and variable time steps
 * - Time scaling (faster/slower than real-time)
 * - Frame counting and statistics
 * - Pause/resume functionality
 */
class TimeManager {
public:
    TimeManager();
    ~TimeManager();

    // ========================================================================
    // Lifecycle
    // ========================================================================

    /**
     * @brief Initialize time manager
     * @param fixed_dt Fixed time step (0 for variable)
     * @return true if successful
     */
    bool initialize(Real fixed_dt = 0.0);

    /**
     * @brief Reset time to initial state
     */
    void reset();

    // ========================================================================
    // Time Control
    // ========================================================================

    /**
     * @brief Advance simulation time by dt
     * @param dt Time step in seconds
     */
    void advance(Real dt);

    /**
     * @brief Pause time advancement
     */
    void pause();

    /**
     * @brief Resume time advancement
     */
    void resume();

    /**
     * @brief Check if time is paused
     */
    bool is_paused() const { return paused_; }

    // ========================================================================
    // Time Access
    // ========================================================================

    /**
     * @brief Get current simulation time (seconds)
     */
    Real get_time() const { return sim_time_; }

    /**
     * @brief Get time elapsed since last frame
     */
    Real get_delta_time() const { return delta_time_; }

    /**
     * @brief Get fixed time step (0 if variable)
     */
    Real get_fixed_dt() const { return fixed_dt_; }

    /**
     * @brief Get actual time step used
     *
     * Returns fixed_dt if set, otherwise returns delta_time
     */
    Real get_effective_dt() const;

    // ========================================================================
    // Time Scaling
    // ========================================================================

    /**
     * @brief Get time scale factor (1.0 = real-time)
     */
    Real get_time_scale() const { return time_scale_; }

    /**
     * @brief Set time scale factor
     * @param scale Scale factor (0.0 to 100.0)
     */
    void set_time_scale(Real scale);

    /**
     * @brief Get scaled delta time (delta_time * time_scale)
     */
    Real get_scaled_dt() const;

    // ========================================================================
    // Frame Statistics
    // ========================================================================

    /**
     * @brief Get current frame number
     */
    UInt64 get_frame_count() const { return frame_count_; }

    /**
     * @brief Get average frame rate (Hz)
     */
    Real get_frame_rate() const;

    /**
     * @brief Get minimum frame time in last second
     */
    Real get_min_frame_time() const { return min_frame_time_; }

    /**
     * @brief Get maximum frame time in last second
     */
    Real get_max_frame_time() const { return max_frame_time_; }

    // ========================================================================
    // Fixed Timestep Sub-stepping
    // ========================================================================

    /**
     * @brief Set fixed time step
     * @param dt Fixed time step (0 for variable)
     */
    void set_fixed_dt(Real dt);

    /**
     * @brief Get accumulated time for sub-stepping
     *
     * When using fixed timestep, this is the remaining time
     * that hasn't been consumed by fixed steps.
     */
    Real get_accumulator() const { return accumulator_; }

    /**
     * @brief Check if another fixed step is available
     */
    bool has_fixed_step() const;

    /**
     * @brief Consume one fixed time step
     * @return true if step was consumed, false if accumulator empty
     */
    bool consume_fixed_step();

    /**
     * @brief Get interpolation factor for rendering
     *
     * Returns value in [0,1] representing progress between
     * the last and next physics update for smooth rendering.
     */
    Real get_interpolation_factor() const;

private:
    // Time state
    Real sim_time_{0.0};        ///< Total simulation time
    Real delta_time_{0.0};      ///< Time since last frame
    Real time_scale_{1.0};      ///< Time scaling factor
    Real fixed_dt_{0.0};        ///< Fixed timestep (0 = variable)
    Real accumulator_{0.0};     ///< Accumulated time for sub-stepping

    // Frame statistics
    UInt64 frame_count_{0};     ///< Total frames processed
    Real frame_time_sum_{0.0};  ///< Sum of frame times for averaging
    int frame_time_count_{0};   ///< Count for averaging
    Real min_frame_time_{1e9};  ///< Minimum frame time
    Real max_frame_time_{0.0};  ///< Maximum frame time
    Real last_stat_time_{0.0};  ///< Last time stats were reset

    // State
    bool paused_{false};        ///< Is time paused
    bool initialized_{false};   ///< Is manager initialized
};

// ============================================================================
// Utility Functions
// ============================================================================

/**
 * @brief Convert Julian date to simulation time
 */
Real julian_to_sim_time(Real julian_date);

/**
 * @brief Convert simulation time to Julian date
 */
Real sim_time_to_julian(Real sim_time, Real epoch_julian);

/**
 * @brief Get modified Julian date
 */
Real get_modified_julian_date(int year, int month, int day,
                              int hour, int minute, Real second);

} // namespace jaguar::core
