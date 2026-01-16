/**
 * @file time_manager.cpp
 * @brief Time management implementation
 */

#include "jaguar/core/time.h"
#include <algorithm>

namespace jaguar::core {

// ============================================================================
// TimeManager Implementation
// ============================================================================

TimeManager::TimeManager() = default;
TimeManager::~TimeManager() = default;

bool TimeManager::initialize(Real fixed_dt)
{
    reset();
    fixed_dt_ = std::max(0.0, fixed_dt);
    initialized_ = true;
    return true;
}

void TimeManager::reset()
{
    sim_time_ = 0.0;
    delta_time_ = 0.0;
    accumulator_ = 0.0;
    frame_count_ = 0;
    frame_time_sum_ = 0.0;
    frame_time_count_ = 0;
    min_frame_time_ = 1e9;
    max_frame_time_ = 0.0;
    last_stat_time_ = 0.0;
    paused_ = false;
}

void TimeManager::advance(Real dt)
{
    if (!initialized_ || paused_) {
        delta_time_ = 0.0;
        return;
    }

    // Apply time scaling
    dt *= time_scale_;

    // Clamp to prevent spiral of death
    constexpr Real MAX_DT = 0.25;  // Max 250ms per frame
    dt = std::min(dt, MAX_DT);

    delta_time_ = dt;
    sim_time_ += dt;
    ++frame_count_;

    // Update frame statistics
    frame_time_sum_ += dt;
    ++frame_time_count_;
    min_frame_time_ = std::min(min_frame_time_, dt);
    max_frame_time_ = std::max(max_frame_time_, dt);

    // Reset statistics every second
    if (sim_time_ - last_stat_time_ >= 1.0) {
        // Keep the current min/max for one more second
        if (frame_time_count_ > 0) {
            // Reset for next averaging period
            frame_time_sum_ = dt;
            frame_time_count_ = 1;
        }
        min_frame_time_ = dt;
        max_frame_time_ = dt;
        last_stat_time_ = sim_time_;
    }

    // Accumulate time for fixed timestep sub-stepping
    if (fixed_dt_ > 0.0) {
        accumulator_ += dt;
    }
}

void TimeManager::pause()
{
    paused_ = true;
}

void TimeManager::resume()
{
    paused_ = false;
}

Real TimeManager::get_effective_dt() const
{
    if (fixed_dt_ > 0.0) {
        return fixed_dt_;
    }
    return delta_time_;
}

void TimeManager::set_time_scale(Real scale)
{
    // Clamp to reasonable range
    time_scale_ = std::clamp(scale, 0.0, 100.0);
}

Real TimeManager::get_scaled_dt() const
{
    return delta_time_;  // Already scaled in advance()
}

Real TimeManager::get_frame_rate() const
{
    if (frame_time_count_ <= 0 || frame_time_sum_ <= 0.0) {
        return 0.0;
    }
    Real avg_frame_time = frame_time_sum_ / static_cast<Real>(frame_time_count_);
    return avg_frame_time > 0.0 ? 1.0 / avg_frame_time : 0.0;
}

void TimeManager::set_fixed_dt(Real dt)
{
    fixed_dt_ = std::max(0.0, dt);
    accumulator_ = 0.0;  // Reset accumulator when changing fixed dt
}

bool TimeManager::has_fixed_step() const
{
    return fixed_dt_ > 0.0 && accumulator_ >= fixed_dt_;
}

bool TimeManager::consume_fixed_step()
{
    if (!has_fixed_step()) {
        return false;
    }
    accumulator_ -= fixed_dt_;
    return true;
}

Real TimeManager::get_interpolation_factor() const
{
    if (fixed_dt_ <= 0.0) {
        return 1.0;  // No interpolation needed for variable timestep
    }
    return accumulator_ / fixed_dt_;
}

// ============================================================================
// Utility Functions
// ============================================================================

Real julian_to_sim_time(Real julian_date)
{
    // Convert Julian date to seconds since J2000 epoch (2451545.0)
    constexpr Real J2000_EPOCH = 2451545.0;
    constexpr Real SECONDS_PER_DAY = 86400.0;
    return (julian_date - J2000_EPOCH) * SECONDS_PER_DAY;
}

Real sim_time_to_julian(Real sim_time, Real epoch_julian)
{
    constexpr Real SECONDS_PER_DAY = 86400.0;
    return epoch_julian + sim_time / SECONDS_PER_DAY;
}

Real get_modified_julian_date(int year, int month, int day,
                              int hour, int minute, Real second)
{
    // Algorithm from "Astronomical Algorithms" by Jean Meeus
    int a = (14 - month) / 12;
    int y = year + 4800 - a;
    int m = month + 12 * a - 3;

    // Julian Day Number
    int jdn = day + (153 * m + 2) / 5 + 365 * y + y / 4 - y / 100 + y / 400 - 32045;

    // Add fractional day
    Real fraction = (static_cast<Real>(hour) - 12.0) / 24.0 +
                    static_cast<Real>(minute) / 1440.0 +
                    second / 86400.0;

    Real jd = static_cast<Real>(jdn) + fraction;

    // Convert to Modified Julian Date (MJD = JD - 2400000.5)
    return jd - 2400000.5;
}

} // namespace jaguar::core
