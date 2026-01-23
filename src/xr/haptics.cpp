/**
 * @file haptics.cpp
 * @brief Haptic feedback system implementation
 *
 * Implements haptic feedback for controllers, vests, motion platforms,
 * engine vibration, and G-force simulation.
 */

#include "jaguar/xr/haptics.h"
#include <cmath>
#include <algorithm>
#include <unordered_map>
#include <mutex>

namespace jaguar::haptics {

// ============================================================================
// Result String Conversion
// ============================================================================

const char* haptic_result_to_string(HapticResult result) {
    switch (result) {
        case HapticResult::Success: return "Success";
        case HapticResult::NotInitialized: return "Not Initialized";
        case HapticResult::DeviceNotConnected: return "Device Not Connected";
        case HapticResult::DeviceNotSupported: return "Device Not Supported";
        case HapticResult::InvalidParameter: return "Invalid Parameter";
        case HapticResult::EffectNotSupported: return "Effect Not Supported";
        case HapticResult::EffectQueueFull: return "Effect Queue Full";
        case HapticResult::BufferOverflow: return "Buffer Overflow";
        case HapticResult::CalibrationRequired: return "Calibration Required";
        case HapticResult::SafetyLimitExceeded: return "Safety Limit Exceeded";
        case HapticResult::CommunicationError: return "Communication Error";
        case HapticResult::PermissionDenied: return "Permission Denied";
        case HapticResult::OutOfMemory: return "Out of Memory";
        case HapticResult::InternalError: return "Internal Error";
        default: return "Unknown Error";
    }
}

// ============================================================================
// Vest Haptic Pattern Implementation
// ============================================================================

VestHapticPattern VestHapticPattern::ImpactFromDirection(const Vec3& direction, Real intensity) {
    VestHapticPattern pattern;
    pattern.base_effect = HapticEffect::Impact(intensity);
    pattern.duration = 0.1;

    // Normalize direction
    Vec3 dir = direction;
    Real len = std::sqrt(dir.x * dir.x + dir.y * dir.y + dir.z * dir.z);
    if (len > 0.001) {
        dir = dir / len;
    }

    // Map direction to zones
    // Positive X = right, Negative X = left
    // Positive Y = up, Negative Y = down (not much we can do with vest)
    // Positive Z = front, Negative Z = back

    auto& intensities = pattern.intensities;

    // Front vs back
    if (dir.z > 0.3) {
        // Impact from front - activate front zones
        Real front_intensity = intensity * std::abs(dir.z);
        intensities[static_cast<size_t>(VestZone::ChestLeft)] = front_intensity;
        intensities[static_cast<size_t>(VestZone::ChestCenter)] = front_intensity;
        intensities[static_cast<size_t>(VestZone::ChestRight)] = front_intensity;
        intensities[static_cast<size_t>(VestZone::AbdomenLeft)] = front_intensity * 0.7;
        intensities[static_cast<size_t>(VestZone::AbdomenCenter)] = front_intensity * 0.7;
        intensities[static_cast<size_t>(VestZone::AbdomenRight)] = front_intensity * 0.7;
    } else if (dir.z < -0.3) {
        // Impact from back - activate back zones
        Real back_intensity = intensity * std::abs(dir.z);
        intensities[static_cast<size_t>(VestZone::UpperBackLeft)] = back_intensity;
        intensities[static_cast<size_t>(VestZone::UpperBackCenter)] = back_intensity;
        intensities[static_cast<size_t>(VestZone::UpperBackRight)] = back_intensity;
        intensities[static_cast<size_t>(VestZone::LowerBackLeft)] = back_intensity * 0.7;
        intensities[static_cast<size_t>(VestZone::LowerBackCenter)] = back_intensity * 0.7;
        intensities[static_cast<size_t>(VestZone::LowerBackRight)] = back_intensity * 0.7;
    }

    // Left vs right
    if (dir.x > 0.3) {
        // Impact from right
        Real right_intensity = intensity * std::abs(dir.x);
        intensities[static_cast<size_t>(VestZone::ChestRight)] += right_intensity * 0.5;
        intensities[static_cast<size_t>(VestZone::AbdomenRight)] += right_intensity * 0.5;
        intensities[static_cast<size_t>(VestZone::UpperBackRight)] += right_intensity * 0.5;
        intensities[static_cast<size_t>(VestZone::RightSide)] = right_intensity;
        intensities[static_cast<size_t>(VestZone::RightShoulder)] = right_intensity * 0.8;
    } else if (dir.x < -0.3) {
        // Impact from left
        Real left_intensity = intensity * std::abs(dir.x);
        intensities[static_cast<size_t>(VestZone::ChestLeft)] += left_intensity * 0.5;
        intensities[static_cast<size_t>(VestZone::AbdomenLeft)] += left_intensity * 0.5;
        intensities[static_cast<size_t>(VestZone::UpperBackLeft)] += left_intensity * 0.5;
        intensities[static_cast<size_t>(VestZone::LeftSide)] = left_intensity;
        intensities[static_cast<size_t>(VestZone::LeftShoulder)] = left_intensity * 0.8;
    }

    // Clamp all values to 0-1
    for (auto& val : intensities) {
        val = std::clamp(val, 0.0, 1.0);
    }

    return pattern;
}

VestHapticPattern VestHapticPattern::Heartbeat(Real bpm) {
    VestHapticPattern pattern;

    // Heartbeat = two quick pulses
    Real beat_duration = 60.0 / bpm;  // Duration of one beat cycle
    Real pulse_duration = 0.08;

    pattern.base_effect = HapticEffect::Pulse(0.6, pulse_duration, pulse_duration, 2);
    pattern.duration = beat_duration;

    // Focus on chest center
    pattern.intensities[static_cast<size_t>(VestZone::ChestCenter)] = 0.8;
    pattern.intensities[static_cast<size_t>(VestZone::ChestLeft)] = 0.6;
    pattern.intensities[static_cast<size_t>(VestZone::ChestRight)] = 0.4;

    return pattern;
}

VestHapticPattern VestHapticPattern::Breathing(Real rate) {
    VestHapticPattern pattern;

    // Breathing cycle: inhale + exhale
    Real cycle_duration = 60.0 / rate;

    pattern.base_effect.waveform = HapticWaveform::Sine;
    pattern.base_effect.frequency = 1.0 / cycle_duration;
    pattern.base_effect.amplitude = 0.4;
    pattern.base_effect.duration = cycle_duration;
    pattern.base_effect.envelope = HapticEnvelope::ADSR(
        cycle_duration * 0.4,   // Inhale (attack)
        0.0,                     // No decay
        1.0,                     // Full sustain
        cycle_duration * 0.4    // Exhale (release)
    );

    // Chest and abdomen expansion
    pattern.intensities[static_cast<size_t>(VestZone::ChestCenter)] = 0.7;
    pattern.intensities[static_cast<size_t>(VestZone::ChestLeft)] = 0.6;
    pattern.intensities[static_cast<size_t>(VestZone::ChestRight)] = 0.6;
    pattern.intensities[static_cast<size_t>(VestZone::AbdomenCenter)] = 0.5;
    pattern.intensities[static_cast<size_t>(VestZone::AbdomenLeft)] = 0.4;
    pattern.intensities[static_cast<size_t>(VestZone::AbdomenRight)] = 0.4;

    return pattern;
}

// ============================================================================
// Motion Cueing Parameters Implementation
// ============================================================================

MotionCueingParams MotionCueingParams::AircraftDefault() {
    MotionCueingParams params;
    params.algorithm = WashoutAlgorithm::Classical;

    // High-pass filters - aircraft typical values
    params.hp_cutoff_freq = {0.5, 0.5, 0.5, 0.3, 0.3, 0.3};
    params.hp_damping = {0.707, 0.707, 0.707, 0.707, 0.707, 0.707};

    // Low-pass filters for tilt coordination
    params.lp_cutoff_freq = {1.0, 1.0, 2.5};
    params.lp_damping = {0.707, 0.707, 0.707};

    // Scaling
    params.scaling = {0.4, 0.4, 0.6, 0.6, 0.6, 0.4};

    // Tilt coordination
    params.tilt_coordination_gain = 0.6;
    params.max_tilt_rate = 0.5;

    return params;
}

MotionCueingParams MotionCueingParams::GroundVehicleDefault() {
    MotionCueingParams params;
    params.algorithm = WashoutAlgorithm::Classical;

    // Higher cutoff for faster washout (more dynamic)
    params.hp_cutoff_freq = {0.8, 0.8, 1.0, 0.5, 0.5, 0.5};
    params.hp_damping = {0.707, 0.707, 0.707, 0.707, 0.707, 0.707};

    // Tilt coordination
    params.lp_cutoff_freq = {1.5, 1.5, 3.0};
    params.lp_damping = {0.707, 0.707, 0.707};

    // Higher scaling for responsive feel
    params.scaling = {0.5, 0.6, 0.4, 0.7, 0.7, 0.5};

    params.tilt_coordination_gain = 0.7;
    params.max_tilt_rate = 0.8;

    return params;
}

MotionCueingParams MotionCueingParams::ShipDefault() {
    MotionCueingParams params;
    params.algorithm = WashoutAlgorithm::Classical;

    // Lower cutoff for slower, larger motions
    params.hp_cutoff_freq = {0.2, 0.2, 0.3, 0.15, 0.15, 0.2};
    params.hp_damping = {0.707, 0.707, 0.707, 0.707, 0.707, 0.707};

    // Low-pass for slow tilt coordination
    params.lp_cutoff_freq = {0.5, 0.5, 1.0};
    params.lp_damping = {0.707, 0.707, 0.707};

    // Heave emphasis for wave motion
    params.scaling = {0.3, 0.3, 0.8, 0.8, 0.8, 0.4};

    params.tilt_coordination_gain = 0.4;
    params.max_tilt_rate = 0.3;

    return params;
}

// ============================================================================
// Mock Haptic Controller Implementation
// ============================================================================

class MockHapticController : public IHapticController {
public:
    MockHapticController() {
        capabilities_.device_type = HapticDeviceType::Controller;
        capabilities_.actuator_type = ActuatorType::LRA;
        capabilities_.actuator_count = 2;
        capabilities_.frequency_min = 80.0;
        capabilities_.frequency_max = 500.0;
        capabilities_.amplitude_resolution = 256;
        capabilities_.supports_custom_waveforms = true;
        capabilities_.device_name = "Mock XR Controller";
        capabilities_.manufacturer = "JaguarEngine";
    }

    HapticDeviceType get_type() const override { return HapticDeviceType::Controller; }

    HapticDeviceCapabilities get_capabilities() const override { return capabilities_; }

    HapticConnectionState get_connection_state() const override { return connection_state_; }

    HapticResult connect() override {
        connection_state_ = HapticConnectionState::Connected;
        return HapticResult::Success;
    }

    HapticResult disconnect() override {
        connection_state_ = HapticConnectionState::Disconnected;
        active_effects_.clear();
        return HapticResult::Success;
    }

    HapticResult calibrate() override {
        connection_state_ = HapticConnectionState::Ready;
        return HapticResult::Success;
    }

    bool is_ready() const override {
        return connection_state_ == HapticConnectionState::Ready ||
               connection_state_ == HapticConnectionState::Connected;
    }

    HapticResult emergency_stop() override {
        active_effects_.clear();
        return HapticResult::Success;
    }

    std::string get_last_error() const override { return last_error_; }

    HapticEffectId play_effect(xr::XRHand hand, const HapticEffect& effect) override {
        if (!is_ready()) return INVALID_HAPTIC_EFFECT_ID;

        HapticEffectId id = ++next_effect_id_;
        HapticEffectState state;
        state.id = id;
        state.is_playing = true;
        active_effects_[id] = {effect, state, hand};

        return id;
    }

    HapticResult play_vibration(const ControllerVibrationConfig& config) override {
        if (!is_ready()) return HapticResult::DeviceNotConnected;

        // Store vibration state
        vibration_states_[static_cast<size_t>(config.hand)] = config;
        return HapticResult::Success;
    }

    HapticResult stop_effect(HapticEffectId effect_id) override {
        auto it = active_effects_.find(effect_id);
        if (it != active_effects_.end()) {
            it->second.state.is_playing = false;
            active_effects_.erase(it);
        }
        return HapticResult::Success;
    }

    HapticResult stop_all(xr::XRHand hand) override {
        for (auto it = active_effects_.begin(); it != active_effects_.end();) {
            if (it->second.hand == hand) {
                it = active_effects_.erase(it);
            } else {
                ++it;
            }
        }
        return HapticResult::Success;
    }

    HapticResult set_adaptive_trigger(xr::XRHand hand,
                                      bool is_left_trigger,
                                      const AdaptiveTriggerConfig& config) override {
        (void)hand;
        (void)is_left_trigger;
        trigger_configs_[static_cast<size_t>(hand) * 2 + (is_left_trigger ? 0 : 1)] = config;
        return HapticResult::Success;
    }

    std::optional<HapticEffectState> get_effect_state(HapticEffectId effect_id) const override {
        auto it = active_effects_.find(effect_id);
        if (it != active_effects_.end()) {
            return it->second.state;
        }
        return std::nullopt;
    }

    // Test helper: get active effect count
    size_t get_active_effect_count() const { return active_effects_.size(); }

private:
    struct ActiveEffect {
        HapticEffect effect;
        HapticEffectState state;
        xr::XRHand hand;
    };

    HapticDeviceCapabilities capabilities_;
    HapticConnectionState connection_state_{HapticConnectionState::Disconnected};
    std::string last_error_;
    HapticEffectId next_effect_id_{0};
    std::unordered_map<HapticEffectId, ActiveEffect> active_effects_;
    std::array<ControllerVibrationConfig, 2> vibration_states_;
    std::array<AdaptiveTriggerConfig, 4> trigger_configs_;
};

// ============================================================================
// Mock Haptic Vest Implementation
// ============================================================================

class MockHapticVest : public IHapticVest {
public:
    MockHapticVest() {
        capabilities_.device_type = HapticDeviceType::Vest;
        capabilities_.actuator_type = ActuatorType::ERM;
        capabilities_.actuator_count = 16;
        capabilities_.frequency_min = 50.0;
        capabilities_.frequency_max = 300.0;
        capabilities_.device_name = "Mock Haptic Vest";
        capabilities_.manufacturer = "JaguarEngine";

        config_.zone_count = 16;
        config_.supports_thermal = false;
        config_.max_intensity = 1.0;
        config_.response_time = 0.01;
    }

    HapticDeviceType get_type() const override { return HapticDeviceType::Vest; }

    HapticDeviceCapabilities get_capabilities() const override { return capabilities_; }

    HapticConnectionState get_connection_state() const override { return connection_state_; }

    HapticResult connect() override {
        connection_state_ = HapticConnectionState::Connected;
        return HapticResult::Success;
    }

    HapticResult disconnect() override {
        connection_state_ = HapticConnectionState::Disconnected;
        zone_states_.fill(0.0);
        return HapticResult::Success;
    }

    HapticResult calibrate() override {
        connection_state_ = HapticConnectionState::Ready;
        return HapticResult::Success;
    }

    bool is_ready() const override {
        return connection_state_ == HapticConnectionState::Ready ||
               connection_state_ == HapticConnectionState::Connected;
    }

    HapticResult emergency_stop() override {
        zone_states_.fill(0.0);
        active_patterns_.clear();
        return HapticResult::Success;
    }

    std::string get_last_error() const override { return last_error_; }

    VestConfig get_config() const override { return config_; }

    HapticEffectId play_pattern(const VestHapticPattern& pattern) override {
        if (!is_ready()) return INVALID_HAPTIC_EFFECT_ID;

        HapticEffectId id = ++next_effect_id_;
        active_patterns_[id] = {pattern, 0.0};

        // Apply pattern intensities
        for (size_t i = 0; i < zone_states_.size(); ++i) {
            zone_states_[i] = std::max(zone_states_[i], pattern.intensities[i]);
        }

        return id;
    }

    HapticResult set_zone_intensity(VestZone zone, Real intensity) override {
        if (!is_ready()) return HapticResult::DeviceNotConnected;

        size_t idx = static_cast<size_t>(zone);
        if (idx >= zone_states_.size()) return HapticResult::InvalidParameter;

        zone_states_[idx] = std::clamp(intensity, 0.0, 1.0);
        return HapticResult::Success;
    }

    HapticResult set_zone_intensities(const std::array<Real, 16>& intensities) override {
        if (!is_ready()) return HapticResult::DeviceNotConnected;

        for (size_t i = 0; i < zone_states_.size(); ++i) {
            zone_states_[i] = std::clamp(intensities[i], 0.0, 1.0);
        }
        return HapticResult::Success;
    }

    HapticResult set_thermal(VestZone zone, Real temperature_delta) override {
        (void)zone;
        (void)temperature_delta;
        if (!config_.supports_thermal) {
            return HapticResult::EffectNotSupported;
        }
        return HapticResult::Success;
    }

    HapticResult stop_all() override {
        zone_states_.fill(0.0);
        active_patterns_.clear();
        return HapticResult::Success;
    }

    std::array<Real, 16> get_zone_states() const override {
        return zone_states_;
    }

private:
    struct ActivePattern {
        VestHapticPattern pattern;
        Real elapsed_time;
    };

    HapticDeviceCapabilities capabilities_;
    VestConfig config_;
    HapticConnectionState connection_state_{HapticConnectionState::Disconnected};
    std::string last_error_;
    std::array<Real, 16> zone_states_{};
    HapticEffectId next_effect_id_{0};
    std::unordered_map<HapticEffectId, ActivePattern> active_patterns_;
};

// ============================================================================
// Mock Motion Platform Implementation
// ============================================================================

class MockMotionPlatform : public IMotionPlatform {
public:
    MockMotionPlatform() {
        capabilities_.device_type = HapticDeviceType::MotionPlatform;
        capabilities_.actuator_type = ActuatorType::Electric;
        capabilities_.actuator_count = 6;
        capabilities_.max_force = 5000.0;
        capabilities_.max_displacement = 0.5;
        capabilities_.device_name = "Mock 6-DOF Motion Platform";
        capabilities_.manufacturer = "JaguarEngine";

        platform_caps_.supported_dof = {true, true, true, true, true, true};
        platform_caps_.max_displacement = {0.3, 0.3, 0.25, 0.35, 0.35, 0.35};  // m or rad
        platform_caps_.max_velocity = {0.5, 0.5, 0.4, 0.8, 0.8, 0.8};
        platform_caps_.max_acceleration = {5.0, 5.0, 4.0, 8.0, 8.0, 8.0};
        platform_caps_.payload_capacity = 300.0;
        platform_caps_.update_rate = 1000.0;

        cueing_params_ = MotionCueingParams::AircraftDefault();
    }

    HapticDeviceType get_type() const override { return HapticDeviceType::MotionPlatform; }

    HapticDeviceCapabilities get_capabilities() const override { return capabilities_; }

    HapticConnectionState get_connection_state() const override { return connection_state_; }

    HapticResult connect() override {
        connection_state_ = HapticConnectionState::Connected;
        return HapticResult::Success;
    }

    HapticResult disconnect() override {
        connection_state_ = HapticConnectionState::Disconnected;
        return HapticResult::Success;
    }

    HapticResult calibrate() override {
        connection_state_ = HapticConnectionState::Calibrating;
        // Simulate calibration
        connection_state_ = HapticConnectionState::Ready;
        return HapticResult::Success;
    }

    bool is_ready() const override {
        return connection_state_ == HapticConnectionState::Ready && state_.is_enabled;
    }

    HapticResult emergency_stop() override {
        state_.fault_active = true;
        state_.fault_message = "Emergency stop activated";
        state_.is_enabled = false;
        // Return to neutral
        state_.position.fill(0.0);
        state_.velocity.fill(0.0);
        state_.acceleration.fill(0.0);
        return HapticResult::Success;
    }

    std::string get_last_error() const override { return state_.fault_message; }

    MotionPlatformCapabilities get_platform_capabilities() const override {
        return platform_caps_;
    }

    MotionPlatformState get_state() const override { return state_; }

    HapticResult home() override {
        if (connection_state_ != HapticConnectionState::Connected &&
            connection_state_ != HapticConnectionState::Ready) {
            return HapticResult::DeviceNotConnected;
        }

        state_.position.fill(0.0);
        state_.velocity.fill(0.0);
        state_.acceleration.fill(0.0);
        state_.is_homed = true;
        state_.fault_active = false;
        state_.fault_message.clear();

        return HapticResult::Success;
    }

    HapticResult enable() override {
        if (!state_.is_homed) {
            return HapticResult::CalibrationRequired;
        }

        state_.is_enabled = true;
        connection_state_ = HapticConnectionState::Ready;
        return HapticResult::Success;
    }

    HapticResult disable() override {
        state_.is_enabled = false;
        return HapticResult::Success;
    }

    HapticResult send_command(const MotionCommand& command) override {
        if (!is_ready()) return HapticResult::DeviceNotConnected;

        if (command.emergency_stop) {
            return emergency_stop();
        }

        // Clamp commands to platform limits
        for (size_t i = 0; i < 6; ++i) {
            state_.position[i] = std::clamp(command.position[i],
                                            -platform_caps_.max_displacement[i],
                                            platform_caps_.max_displacement[i]);
            state_.velocity[i] = std::clamp(command.velocity[i],
                                            -platform_caps_.max_velocity[i],
                                            platform_caps_.max_velocity[i]);
            state_.acceleration[i] = std::clamp(command.acceleration[i],
                                                -platform_caps_.max_acceleration[i],
                                                platform_caps_.max_acceleration[i]);
        }

        return HapticResult::Success;
    }

    HapticResult set_cueing_params(const MotionCueingParams& params) override {
        cueing_params_ = params;
        return HapticResult::Success;
    }

    MotionCueingParams get_cueing_params() const override {
        return cueing_params_;
    }

    HapticResult apply_vehicle_state(const Vec3& acceleration,
                                     const Vec3& angular_velocity,
                                     const Vec3& angular_acceleration) override {
        if (!is_ready()) return HapticResult::DeviceNotConnected;

        // Simple motion cueing: scale and limit vehicle motion
        // In a real implementation, this would use washout filters

        MotionCommand cmd;

        // Translational: direct mapping with scaling and high-pass (simulated)
        cmd.acceleration[static_cast<size_t>(MotionDOF::Surge)] =
            acceleration.x * cueing_params_.scaling[0];
        cmd.acceleration[static_cast<size_t>(MotionDOF::Sway)] =
            acceleration.y * cueing_params_.scaling[1];
        cmd.acceleration[static_cast<size_t>(MotionDOF::Heave)] =
            acceleration.z * cueing_params_.scaling[2];

        // Rotational: angular velocity to position with scaling
        cmd.velocity[static_cast<size_t>(MotionDOF::Roll)] =
            angular_velocity.x * cueing_params_.scaling[3];
        cmd.velocity[static_cast<size_t>(MotionDOF::Pitch)] =
            angular_velocity.y * cueing_params_.scaling[4];
        cmd.velocity[static_cast<size_t>(MotionDOF::Yaw)] =
            angular_velocity.z * cueing_params_.scaling[5];

        // Tilt coordination: sustained G converted to tilt
        if (cueing_params_.tilt_coordination_gain > 0) {
            Real tilt_pitch = std::atan2(acceleration.x, 9.81) *
                              cueing_params_.tilt_coordination_gain;
            Real tilt_roll = std::atan2(-acceleration.y, 9.81) *
                             cueing_params_.tilt_coordination_gain;

            // Rate limit tilt changes
            tilt_pitch = std::clamp(tilt_pitch,
                                    -cueing_params_.max_tilt_rate,
                                    cueing_params_.max_tilt_rate);
            tilt_roll = std::clamp(tilt_roll,
                                   -cueing_params_.max_tilt_rate,
                                   cueing_params_.max_tilt_rate);

            cmd.position[static_cast<size_t>(MotionDOF::Pitch)] += tilt_pitch;
            cmd.position[static_cast<size_t>(MotionDOF::Roll)] += tilt_roll;
        }

        // Add angular acceleration feed-forward
        cmd.acceleration[static_cast<size_t>(MotionDOF::Roll)] =
            angular_acceleration.x * cueing_params_.scaling[3] * 0.1;
        cmd.acceleration[static_cast<size_t>(MotionDOF::Pitch)] =
            angular_acceleration.y * cueing_params_.scaling[4] * 0.1;
        cmd.acceleration[static_cast<size_t>(MotionDOF::Yaw)] =
            angular_acceleration.z * cueing_params_.scaling[5] * 0.1;

        return send_command(cmd);
    }

    HapticResult add_vibration(const HapticEffect& effect, Real amplitude) override {
        (void)effect;
        (void)amplitude;
        // Store vibration overlay
        return HapticResult::Success;
    }

    HapticResult park() override {
        MotionCommand cmd;
        cmd.position.fill(0.0);
        cmd.velocity.fill(0.0);
        cmd.acceleration.fill(0.0);
        return send_command(cmd);
    }

private:
    HapticDeviceCapabilities capabilities_;
    MotionPlatformCapabilities platform_caps_;
    MotionPlatformState state_;
    MotionCueingParams cueing_params_;
    HapticConnectionState connection_state_{HapticConnectionState::Disconnected};
};

// ============================================================================
// Mock Haptic Renderer Implementation
// ============================================================================

class MockHapticRenderer : public IHapticRenderer {
public:
    MockHapticRenderer() {
        controller_ = std::make_shared<MockHapticController>();
        vest_ = std::make_shared<MockHapticVest>();
        motion_platform_ = std::make_shared<MockMotionPlatform>();
    }

    HapticResult initialize(const HapticRendererConfig& config) override {
        config_ = config;
        is_initialized_ = true;

        if (config.auto_connect_devices) {
            if (config.enable_controllers) {
                controller_->connect();
                controller_->calibrate();
            }
            if (config.enable_vest) {
                vest_->connect();
                vest_->calibrate();
            }
            if (config.enable_motion_platform) {
                motion_platform_->connect();
                motion_platform_->calibrate();
                motion_platform_->home();
                motion_platform_->enable();
                motion_platform_->set_cueing_params(config.motion_cueing_params);
            }
        }

        return HapticResult::Success;
    }

    HapticResult shutdown() override {
        stop_all();

        if (controller_) controller_->disconnect();
        if (vest_) vest_->disconnect();
        if (motion_platform_) {
            motion_platform_->park();
            motion_platform_->disable();
            motion_platform_->disconnect();
        }

        is_initialized_ = false;
        return HapticResult::Success;
    }

    HapticResult update(Real delta_time) override {
        if (!is_initialized_) return HapticResult::NotInitialized;

        // Update effect playback times
        for (auto& [id, effect_data] : active_effects_) {
            effect_data.elapsed_time += delta_time;

            // Check for effect completion
            if (effect_data.effect.duration > 0 &&
                effect_data.elapsed_time >= effect_data.effect.duration) {
                effect_data.completed = true;
            }
        }

        // Remove completed effects
        for (auto it = active_effects_.begin(); it != active_effects_.end();) {
            if (it->second.completed) {
                it = active_effects_.erase(it);
            } else {
                ++it;
            }
        }

        stats_.frames_rendered++;
        stats_.active_effects = static_cast<UInt32>(active_effects_.size());

        return HapticResult::Success;
    }

    HapticRendererConfig get_config() const override { return config_; }

    void set_master_intensity(Real intensity) override {
        config_.master_intensity = std::clamp(intensity, 0.0, 1.0);
    }

    std::vector<std::shared_ptr<IHapticDevice>> get_devices() const override {
        std::vector<std::shared_ptr<IHapticDevice>> devices;
        if (controller_) devices.push_back(controller_);
        if (vest_) devices.push_back(vest_);
        if (motion_platform_) devices.push_back(motion_platform_);
        return devices;
    }

    std::shared_ptr<IHapticController> get_controller() const override {
        return controller_;
    }

    std::shared_ptr<IHapticVest> get_vest() const override {
        return vest_;
    }

    std::shared_ptr<IMotionPlatform> get_motion_platform() const override {
        return motion_platform_;
    }

    HapticEffectId play_effect(HapticDeviceType device, const HapticEffect& effect) override {
        if (!is_initialized_) return INVALID_HAPTIC_EFFECT_ID;

        HapticEffectId id = ++next_effect_id_;
        active_effects_[id] = {effect, device, 0.0, false};
        stats_.effects_played++;

        return id;
    }

    HapticResult stop_effect(HapticEffectId effect_id) override {
        auto it = active_effects_.find(effect_id);
        if (it != active_effects_.end()) {
            active_effects_.erase(it);
            return HapticResult::Success;
        }
        return HapticResult::InvalidParameter;
    }

    HapticResult stop_all() override {
        active_effects_.clear();

        if (controller_) {
            controller_->stop_all(xr::XRHand::Left);
            controller_->stop_all(xr::XRHand::Right);
        }
        if (vest_) {
            vest_->stop_all();
        }
        if (motion_platform_) {
            motion_platform_->park();
        }

        return HapticResult::Success;
    }

    HapticEffectId play_controller_effect(xr::XRHand hand, const HapticEffect& effect) override {
        if (!controller_) return INVALID_HAPTIC_EFFECT_ID;

        HapticEffect scaled_effect = effect;
        scaled_effect.amplitude *= config_.master_intensity;

        return controller_->play_effect(hand, scaled_effect);
    }

    HapticResult play_controller_vibration(xr::XRHand hand,
                                           Real low_freq, Real high_freq,
                                           Real duration) override {
        if (!controller_) return HapticResult::DeviceNotConnected;

        ControllerVibrationConfig config;
        config.hand = hand;
        config.low_frequency_amplitude = low_freq * config_.master_intensity;
        config.high_frequency_amplitude = high_freq * config_.master_intensity;
        config.effect = HapticEffect::Vibration(1.0, duration);

        return controller_->play_vibration(config);
    }

    HapticEffectId play_vest_pattern(const VestHapticPattern& pattern) override {
        if (!vest_) return INVALID_HAPTIC_EFFECT_ID;

        VestHapticPattern scaled_pattern = pattern;
        for (auto& intensity : scaled_pattern.intensities) {
            intensity *= config_.master_intensity;
        }

        return vest_->play_pattern(scaled_pattern);
    }

    HapticResult play_vest_impact(const Vec3& direction, Real intensity) override {
        if (!vest_) return HapticResult::DeviceNotConnected;

        VestHapticPattern pattern = VestHapticPattern::ImpactFromDirection(
            direction, intensity * config_.master_intensity);
        vest_->play_pattern(pattern);
        return HapticResult::Success;
    }

    HapticResult apply_motion_cueing(const Vec3& linear_accel,
                                     const Vec3& angular_vel,
                                     const Vec3& angular_accel) override {
        if (!motion_platform_) return HapticResult::DeviceNotConnected;

        return motion_platform_->apply_vehicle_state(linear_accel, angular_vel, angular_accel);
    }

    HapticResult set_engine_vibration(const EngineVibrationParams& params) override {
        engine_vibration_ = params;

        // Apply to vest as low-frequency rumble
        if (vest_ && params.rpm > 0) {
            std::array<Real, 16> intensities;
            Real base_intensity = 0.1 * params.amplitude_scale * (params.throttle * 0.5 + 0.5);

            // Seat area gets most vibration
            intensities.fill(base_intensity * 0.5);
            intensities[static_cast<size_t>(VestZone::LowerBackCenter)] = base_intensity;
            intensities[static_cast<size_t>(VestZone::LowerBackLeft)] = base_intensity * 0.8;
            intensities[static_cast<size_t>(VestZone::LowerBackRight)] = base_intensity * 0.8;

            vest_->set_zone_intensities(intensities);
        }

        return HapticResult::Success;
    }

    HapticResult set_vehicle_vibration(const VehicleVibrationState& state) override {
        vehicle_vibration_ = state;

        // Combine all sources
        Real total_intensity = state.total_amplitude * config_.master_intensity;

        if (motion_platform_ && total_intensity > 0.01) {
            HapticEffect vibration;
            vibration.waveform = HapticWaveform::Sine;
            vibration.frequency = state.dominant_frequency;
            vibration.amplitude = total_intensity;
            vibration.duration = 0;  // Continuous
            motion_platform_->add_vibration(vibration, total_intensity);
        }

        return HapticResult::Success;
    }

    HapticResult apply_g_force(const GForceState& g_state) override {
        g_force_state_ = g_state;

        // Apply G-suit compression to vest (if simulating G-suit)
        if (config_.g_force_config.simulate_gsuit && vest_ && g_state.gsuit_active) {
            Real compression = g_state.gsuit_pressure * config_.master_intensity;

            std::array<Real, 16> intensities;
            intensities.fill(0.0);

            // G-suit compresses legs and abdomen
            intensities[static_cast<size_t>(VestZone::AbdomenLeft)] = compression;
            intensities[static_cast<size_t>(VestZone::AbdomenCenter)] = compression;
            intensities[static_cast<size_t>(VestZone::AbdomenRight)] = compression;
            intensities[static_cast<size_t>(VestZone::LowerBackLeft)] = compression * 0.7;
            intensities[static_cast<size_t>(VestZone::LowerBackRight)] = compression * 0.7;

            vest_->set_zone_intensities(intensities);
        }

        // Apply motion cueing for sustained G
        if (motion_platform_ && std::abs(g_state.sustained_g) > 0.1) {
            // Tilt platform to simulate sustained G
            Vec3 accel = g_state.body_acceleration;
            motion_platform_->apply_vehicle_state(accel, Vec3::Zero(), Vec3::Zero());
        }

        return HapticResult::Success;
    }

    GForceState calculate_g_force(const Vec3& body_acceleration,
                                  const Vec3& gravity_body_frame) const override {
        GForceState state;

        // Total acceleration including gravity
        state.body_acceleration = body_acceleration;

        // G-force is acceleration divided by gravity
        constexpr Real g = 9.81;
        state.g_force = (body_acceleration - gravity_body_frame) / g;
        state.total_g = state.g_force.length();

        // Calculate onset rate (would need history for accurate calculation)
        state.onset_rate = 0.0;  // Simplified

        // Separate sustained and transient components (simplified)
        state.sustained_g = state.total_g * 0.8;
        state.transient_g = state.total_g * 0.2;

        // G-suit logic
        const auto& gconfig = config_.g_force_config;
        if (gconfig.simulate_gsuit && state.total_g > gconfig.gsuit_threshold) {
            state.gsuit_active = true;
            state.gsuit_pressure = (state.total_g - gconfig.gsuit_threshold) *
                                   gconfig.gsuit_compression;
            state.gsuit_pressure = std::clamp(state.gsuit_pressure, 0.0, 1.0);
        }

        return state;
    }

    HapticRendererStats get_stats() const override {
        HapticRendererStats stats = stats_;
        stats.connected_devices = 0;
        if (controller_ && controller_->is_ready()) stats.connected_devices++;
        if (vest_ && vest_->is_ready()) stats.connected_devices++;
        if (motion_platform_ && motion_platform_->is_ready()) stats.connected_devices++;
        return stats;
    }

    HapticResult emergency_stop_all() override {
        if (controller_) controller_->emergency_stop();
        if (vest_) vest_->emergency_stop();
        if (motion_platform_) motion_platform_->emergency_stop();

        active_effects_.clear();
        return HapticResult::Success;
    }

private:
    struct EffectData {
        HapticEffect effect;
        HapticDeviceType device;
        Real elapsed_time;
        bool completed;
    };

    HapticRendererConfig config_;
    bool is_initialized_{false};
    HapticEffectId next_effect_id_{0};
    std::unordered_map<HapticEffectId, EffectData> active_effects_;
    HapticRendererStats stats_;

    std::shared_ptr<MockHapticController> controller_;
    std::shared_ptr<MockHapticVest> vest_;
    std::shared_ptr<MockMotionPlatform> motion_platform_;

    EngineVibrationParams engine_vibration_;
    VehicleVibrationState vehicle_vibration_;
    GForceState g_force_state_;
};

// ============================================================================
// Preset Haptic Effects Implementation
// ============================================================================

namespace presets {

namespace aircraft {

HapticEffect StallBuffet(Real intensity) {
    HapticEffect effect;
    effect.waveform = HapticWaveform::Noise;
    effect.amplitude = intensity;
    effect.frequency = 8.0;  // Low frequency buffet
    effect.frequency_variation = 3.0;
    effect.amplitude_variation = 0.2;
    effect.duration = 0;  // Continuous until stopped
    return effect;
}

HapticEffect GearTransition(bool extending) {
    HapticEffect effect;
    effect.waveform = HapticWaveform::Noise;
    effect.amplitude = 0.4;
    effect.frequency = 25.0;
    effect.duration = extending ? 8.0 : 6.0;  // Extension takes longer
    effect.envelope = HapticEnvelope::ADSR(0.5, 0.2, 0.6, 0.5);
    return effect;
}

HapticEffect FlapMovement(Real deflection) {
    HapticEffect effect;
    effect.waveform = HapticWaveform::Sine;
    effect.amplitude = 0.2 + 0.2 * std::abs(deflection);
    effect.frequency = 15.0;
    effect.duration = 3.0 * std::abs(deflection);  // Proportional to movement
    return effect;
}

HapticEffect SpeedBrake(Real deployment) {
    HapticEffect effect;
    effect.waveform = HapticWaveform::Noise;
    effect.amplitude = 0.3 * deployment;
    effect.frequency = 30.0 + 20.0 * deployment;
    effect.duration = 0;  // Continuous while deployed
    return effect;
}

HapticEffect Touchdown(Real vertical_speed) {
    // Vertical speed in m/s, positive = descending
    Real severity = std::clamp(vertical_speed / 3.0, 0.0, 1.0);

    HapticEffect effect;
    effect.waveform = HapticWaveform::Constant;
    effect.amplitude = 0.5 + 0.5 * severity;
    effect.duration = 0.1 + 0.2 * severity;
    effect.envelope = HapticEnvelope::FadeOut(0.05 + 0.1 * severity);
    return effect;
}

HapticEffect EngineStart(Real progress) {
    // progress: 0 = start, 1 = idle
    HapticEffect effect;
    effect.waveform = HapticWaveform::Sine;
    effect.amplitude = 0.3;
    effect.frequency = 20.0 + 100.0 * progress;  // Spool up
    effect.duration = 0.5;
    return effect;
}

HapticEffect AfterburnerIgnition() {
    HapticEffect effect;
    effect.waveform = HapticWaveform::Noise;
    effect.amplitude = 0.8;
    effect.frequency = 50.0;
    effect.duration = 0.3;
    effect.envelope = HapticEnvelope::ADSR(0.05, 0.1, 0.7, 0.15);
    return effect;
}

HapticEffect WeaponsRelease() {
    HapticEffect effect;
    effect.waveform = HapticWaveform::Constant;
    effect.amplitude = 0.6;
    effect.duration = 0.15;
    effect.envelope = HapticEnvelope::FadeOut(0.1);
    return effect;
}

HapticEffect RefuelingContact() {
    HapticEffect effect;
    effect.waveform = HapticWaveform::Constant;
    effect.amplitude = 0.4;
    effect.duration = 0.2;
    effect.envelope = HapticEnvelope::FadeOut(0.15);
    return effect;
}

HapticEffect OverspeedWarning() {
    HapticEffect effect;
    effect.waveform = HapticWaveform::Square;
    effect.amplitude = 0.7;
    effect.frequency = 4.0;  // 4 Hz pulsing
    effect.duration = 0.25;
    effect.repeat_count = -1;  // Continuous
    effect.repeat_delay = 0.25;
    return effect;
}

} // namespace aircraft

namespace vehicle {

HapticEffect RoadTexture(Real roughness) {
    HapticEffect effect;
    effect.waveform = HapticWaveform::Noise;
    effect.amplitude = 0.1 + 0.3 * roughness;
    effect.frequency = 20.0 + 40.0 * roughness;
    effect.frequency_variation = 10.0 * roughness;
    effect.duration = 0;  // Continuous
    return effect;
}

HapticEffect BrakeApply(Real pressure) {
    HapticEffect effect;
    effect.waveform = HapticWaveform::Constant;
    effect.amplitude = 0.2 + 0.4 * pressure;
    effect.duration = 0;  // While braking
    return effect;
}

HapticEffect ABSPulse() {
    HapticEffect effect;
    effect.waveform = HapticWaveform::Square;
    effect.amplitude = 0.6;
    effect.frequency = 15.0;  // Typical ABS pulse rate
    effect.duration = 0.067;  // One pulse
    effect.repeat_count = -1;
    effect.repeat_delay = 0.0;
    return effect;
}

HapticEffect GearShift() {
    HapticEffect effect;
    effect.waveform = HapticWaveform::Constant;
    effect.amplitude = 0.5;
    effect.duration = 0.1;
    effect.envelope = HapticEnvelope::FadeOut(0.05);
    return effect;
}

HapticEffect Collision(Real severity, const Vec3& direction) {
    (void)direction;  // Used by vest pattern, not base effect
    HapticEffect effect;
    effect.waveform = HapticWaveform::Noise;
    effect.amplitude = std::clamp(severity, 0.3, 1.0);
    effect.frequency = 30.0;
    effect.duration = 0.2 + 0.3 * severity;
    effect.envelope = HapticEnvelope::FadeOut(0.1 + 0.2 * severity);
    return effect;
}

HapticEffect TrackRumble(Real speed) {
    // speed in m/s
    Real normalized_speed = std::clamp(speed / 20.0, 0.0, 1.0);

    HapticEffect effect;
    effect.waveform = HapticWaveform::Noise;
    effect.amplitude = 0.2 + 0.3 * normalized_speed;
    effect.frequency = 5.0 + 15.0 * normalized_speed;
    effect.duration = 0;  // Continuous
    return effect;
}

HapticEffect FiringRecoil(Real caliber) {
    // caliber in mm
    Real intensity = std::clamp(caliber / 125.0, 0.3, 1.0);  // 125mm tank gun = max

    HapticEffect effect;
    effect.waveform = HapticWaveform::Constant;
    effect.amplitude = intensity;
    effect.duration = 0.1 + 0.1 * intensity;
    effect.envelope = HapticEnvelope::FadeOut(0.05 + 0.05 * intensity);
    return effect;
}

} // namespace vehicle

namespace naval {

HapticEffect WaveMotion(Real sea_state) {
    // sea_state: 0-9 (Douglas sea scale)
    Real normalized = std::clamp(sea_state / 9.0, 0.0, 1.0);

    HapticEffect effect;
    effect.waveform = HapticWaveform::Sine;
    effect.amplitude = 0.1 + 0.5 * normalized;
    effect.frequency = 0.1 + 0.3 * normalized;  // Wave period decreases with severity
    effect.duration = 0;  // Continuous
    return effect;
}

HapticEffect HullVibration(Real speed) {
    // speed in knots
    Real normalized = std::clamp(speed / 30.0, 0.0, 1.0);

    HapticEffect effect;
    effect.waveform = HapticWaveform::Noise;
    effect.amplitude = 0.1 + 0.2 * normalized;
    effect.frequency = 10.0 + 20.0 * normalized;
    effect.duration = 0;  // Continuous
    return effect;
}

HapticEffect AnchorDrop() {
    HapticEffect effect;
    effect.waveform = HapticWaveform::Noise;
    effect.amplitude = 0.5;
    effect.frequency = 15.0;
    effect.duration = 5.0;
    effect.envelope = HapticEnvelope::FadeOut(2.0);
    return effect;
}

HapticEffect NavalGunFiring(Real caliber) {
    // caliber in mm (e.g., 127mm = 5 inch)
    Real intensity = std::clamp(caliber / 406.0, 0.4, 1.0);  // 406mm = 16 inch max

    HapticEffect effect;
    effect.waveform = HapticWaveform::Constant;
    effect.amplitude = intensity;
    effect.duration = 0.15 + 0.15 * intensity;
    effect.envelope = HapticEnvelope::FadeOut(0.1 + 0.1 * intensity);
    return effect;
}

HapticEffect TorpedoLaunch() {
    HapticEffect effect;
    effect.waveform = HapticWaveform::Noise;
    effect.amplitude = 0.6;
    effect.frequency = 20.0;
    effect.duration = 0.5;
    effect.envelope = HapticEnvelope::ADSR(0.1, 0.1, 0.5, 0.3);
    return effect;
}

HapticEffect DepthCharge(Real distance) {
    // distance in meters
    Real intensity = std::clamp(1.0 - (distance / 500.0), 0.1, 1.0);

    HapticEffect effect;
    effect.waveform = HapticWaveform::Noise;
    effect.amplitude = intensity;
    effect.frequency = 10.0 + 20.0 * intensity;
    effect.duration = 0.5 + 0.5 * intensity;
    effect.envelope = HapticEnvelope::ADSR(0.02, 0.1, 0.5, 0.3);
    return effect;
}

} // namespace naval

namespace helicopter {

HapticEffect RotorVibration(Real rpm) {
    // Main rotor typically 250-350 RPM
    Real normalized = std::clamp((rpm - 200.0) / 200.0, 0.0, 1.0);
    Real blade_pass_freq = rpm / 60.0 * 4.0;  // Assume 4 blades

    HapticEffect effect;
    effect.waveform = HapticWaveform::Sine;
    effect.amplitude = 0.2 + 0.2 * normalized;
    effect.frequency = blade_pass_freq;
    effect.duration = 0;  // Continuous
    return effect;
}

HapticEffect GroundResonance() {
    HapticEffect effect;
    effect.waveform = HapticWaveform::Sine;
    effect.amplitude = 0.8;
    effect.frequency = 3.0;  // Dangerous low frequency
    effect.frequency_variation = 1.0;
    effect.duration = 0;
    return effect;
}

HapticEffect VortexRingState() {
    HapticEffect effect;
    effect.waveform = HapticWaveform::Noise;
    effect.amplitude = 0.7;
    effect.frequency = 5.0;
    effect.frequency_variation = 3.0;
    effect.amplitude_variation = 0.3;
    effect.duration = 0;
    return effect;
}

HapticEffect Autorotation() {
    HapticEffect effect;
    effect.waveform = HapticWaveform::Sine;
    effect.amplitude = 0.3;
    effect.frequency = 20.0;  // Higher than normal rotor vibration
    effect.duration = 0;
    return effect;
}

HapticEffect HardLanding(Real vertical_speed) {
    Real severity = std::clamp(vertical_speed / 5.0, 0.0, 1.0);

    HapticEffect effect;
    effect.waveform = HapticWaveform::Constant;
    effect.amplitude = 0.6 + 0.4 * severity;
    effect.duration = 0.15 + 0.25 * severity;
    effect.envelope = HapticEnvelope::FadeOut(0.1 + 0.15 * severity);
    return effect;
}

} // namespace helicopter

} // namespace presets

// ============================================================================
// Factory Functions
// ============================================================================

std::unique_ptr<IHapticRenderer> create_haptic_renderer() {
    return std::make_unique<MockHapticRenderer>();
}

std::unique_ptr<IHapticRenderer> create_haptic_renderer(const HapticRendererConfig& config) {
    auto renderer = std::make_unique<MockHapticRenderer>();
    renderer->initialize(config);
    return renderer;
}

std::unique_ptr<IHapticRenderer> create_mock_haptic_renderer() {
    return std::make_unique<MockHapticRenderer>();
}

} // namespace jaguar::haptics
