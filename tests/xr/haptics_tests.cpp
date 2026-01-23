/**
 * @file haptics_tests.cpp
 * @brief Unit tests for the haptic feedback system
 */

#include <gtest/gtest.h>
#include "jaguar/xr/haptics.h"
#include <cmath>
#include <memory>

using namespace jaguar;
using namespace jaguar::haptics;

// ============================================================================
// Haptic Result Tests
// ============================================================================

TEST(HapticResultTest, SuccessString) {
    EXPECT_STREQ(haptic_result_to_string(HapticResult::Success), "Success");
}

TEST(HapticResultTest, ErrorStrings) {
    EXPECT_STREQ(haptic_result_to_string(HapticResult::NotInitialized), "Not Initialized");
    EXPECT_STREQ(haptic_result_to_string(HapticResult::DeviceNotConnected), "Device Not Connected");
    EXPECT_STREQ(haptic_result_to_string(HapticResult::InvalidParameter), "Invalid Parameter");
    EXPECT_STREQ(haptic_result_to_string(HapticResult::SafetyLimitExceeded), "Safety Limit Exceeded");
}

TEST(HapticResultTest, SucceededHelper) {
    EXPECT_TRUE(haptic_succeeded(HapticResult::Success));
    EXPECT_FALSE(haptic_succeeded(HapticResult::NotInitialized));
    EXPECT_FALSE(haptic_succeeded(HapticResult::InternalError));
}

// ============================================================================
// Haptic Envelope Tests
// ============================================================================

TEST(HapticEnvelopeTest, InstantEnvelope) {
    auto env = HapticEnvelope::Instant();
    EXPECT_DOUBLE_EQ(env.attack_time, 0.0);
    EXPECT_DOUBLE_EQ(env.decay_time, 0.0);
    EXPECT_DOUBLE_EQ(env.sustain_level, 1.0);
    EXPECT_DOUBLE_EQ(env.release_time, 0.0);
}

TEST(HapticEnvelopeTest, FadeInEnvelope) {
    auto env = HapticEnvelope::FadeIn(0.5);
    EXPECT_DOUBLE_EQ(env.attack_time, 0.5);
    EXPECT_DOUBLE_EQ(env.release_time, 0.0);
}

TEST(HapticEnvelopeTest, FadeOutEnvelope) {
    auto env = HapticEnvelope::FadeOut(0.3);
    EXPECT_DOUBLE_EQ(env.attack_time, 0.0);
    EXPECT_DOUBLE_EQ(env.release_time, 0.3);
}

TEST(HapticEnvelopeTest, ADSREnvelope) {
    auto env = HapticEnvelope::ADSR(0.1, 0.2, 0.7, 0.3);
    EXPECT_DOUBLE_EQ(env.attack_time, 0.1);
    EXPECT_DOUBLE_EQ(env.decay_time, 0.2);
    EXPECT_DOUBLE_EQ(env.sustain_level, 0.7);
    EXPECT_DOUBLE_EQ(env.release_time, 0.3);
}

TEST(HapticEnvelopeTest, TransitionDuration) {
    auto env = HapticEnvelope::ADSR(0.1, 0.2, 0.5, 0.3);
    EXPECT_DOUBLE_EQ(env.get_transition_duration(), 0.6);  // 0.1 + 0.2 + 0.3
}

// ============================================================================
// Haptic Effect Tests
// ============================================================================

TEST(HapticEffectTest, VibrationEffect) {
    auto effect = HapticEffect::Vibration(0.7, 0.5, 200.0);
    EXPECT_EQ(effect.waveform, HapticWaveform::Sine);
    EXPECT_DOUBLE_EQ(effect.amplitude, 0.7);
    EXPECT_DOUBLE_EQ(effect.duration, 0.5);
    EXPECT_DOUBLE_EQ(effect.frequency, 200.0);
}

TEST(HapticEffectTest, PulseEffect) {
    auto effect = HapticEffect::Pulse(0.8, 0.1, 0.05, 3);
    EXPECT_EQ(effect.waveform, HapticWaveform::Square);
    EXPECT_DOUBLE_EQ(effect.amplitude, 0.8);
    EXPECT_DOUBLE_EQ(effect.duration, 0.1);
    EXPECT_EQ(effect.repeat_count, 3);
    EXPECT_DOUBLE_EQ(effect.repeat_delay, 0.05);
}

TEST(HapticEffectTest, RumbleEffect) {
    auto effect = HapticEffect::Rumble(0.6, 1.0);
    EXPECT_EQ(effect.waveform, HapticWaveform::Noise);
    EXPECT_DOUBLE_EQ(effect.amplitude, 0.6);
    EXPECT_DOUBLE_EQ(effect.duration, 1.0);
    EXPECT_GT(effect.frequency_variation, 0.0);
}

TEST(HapticEffectTest, ImpactEffect) {
    auto effect = HapticEffect::Impact(0.9);
    EXPECT_EQ(effect.waveform, HapticWaveform::Constant);
    EXPECT_DOUBLE_EQ(effect.amplitude, 0.9);
    EXPECT_LE(effect.duration, 0.1);  // Short duration
    EXPECT_GT(effect.envelope.release_time, 0.0);  // Has fade-out
}

// ============================================================================
// Vest Haptic Pattern Tests
// ============================================================================

TEST(VestHapticPatternTest, SetAll) {
    VestHapticPattern pattern;
    pattern.set_all(0.5);

    for (size_t i = 0; i < static_cast<size_t>(VestZone::ZoneCount); ++i) {
        EXPECT_DOUBLE_EQ(pattern.intensities[i], 0.5);
    }
}

TEST(VestHapticPatternTest, SetFront) {
    VestHapticPattern pattern;
    pattern.set_front(0.8);

    EXPECT_DOUBLE_EQ(pattern.intensities[static_cast<size_t>(VestZone::ChestCenter)], 0.8);
    EXPECT_DOUBLE_EQ(pattern.intensities[static_cast<size_t>(VestZone::ChestLeft)], 0.8);
    EXPECT_DOUBLE_EQ(pattern.intensities[static_cast<size_t>(VestZone::AbdomenCenter)], 0.8);
    // Back should be unset (0)
    EXPECT_DOUBLE_EQ(pattern.intensities[static_cast<size_t>(VestZone::UpperBackCenter)], 0.0);
}

TEST(VestHapticPatternTest, SetBack) {
    VestHapticPattern pattern;
    pattern.set_back(0.7);

    EXPECT_DOUBLE_EQ(pattern.intensities[static_cast<size_t>(VestZone::UpperBackCenter)], 0.7);
    EXPECT_DOUBLE_EQ(pattern.intensities[static_cast<size_t>(VestZone::LowerBackCenter)], 0.7);
    // Front should be unset (0)
    EXPECT_DOUBLE_EQ(pattern.intensities[static_cast<size_t>(VestZone::ChestCenter)], 0.0);
}

TEST(VestHapticPatternTest, ImpactFromFront) {
    Vec3 front_direction{0.0, 0.0, 1.0};  // From front
    auto pattern = VestHapticPattern::ImpactFromDirection(front_direction, 0.8);

    // Front zones should be activated
    EXPECT_GT(pattern.intensities[static_cast<size_t>(VestZone::ChestCenter)], 0.5);
    EXPECT_GT(pattern.intensities[static_cast<size_t>(VestZone::ChestLeft)], 0.5);
    EXPECT_GT(pattern.intensities[static_cast<size_t>(VestZone::ChestRight)], 0.5);
    // Back zones should be minimal
    EXPECT_LT(pattern.intensities[static_cast<size_t>(VestZone::UpperBackCenter)], 0.2);
}

TEST(VestHapticPatternTest, ImpactFromBack) {
    Vec3 back_direction{0.0, 0.0, -1.0};  // From back
    auto pattern = VestHapticPattern::ImpactFromDirection(back_direction, 0.8);

    // Back zones should be activated
    EXPECT_GT(pattern.intensities[static_cast<size_t>(VestZone::UpperBackCenter)], 0.5);
    EXPECT_GT(pattern.intensities[static_cast<size_t>(VestZone::LowerBackCenter)], 0.3);
    // Front zones should be minimal
    EXPECT_LT(pattern.intensities[static_cast<size_t>(VestZone::ChestCenter)], 0.2);
}

TEST(VestHapticPatternTest, ImpactFromRight) {
    Vec3 right_direction{1.0, 0.0, 0.0};  // From right
    auto pattern = VestHapticPattern::ImpactFromDirection(right_direction, 0.8);

    // Right side should be activated
    EXPECT_GT(pattern.intensities[static_cast<size_t>(VestZone::RightSide)], 0.5);
    EXPECT_GT(pattern.intensities[static_cast<size_t>(VestZone::RightShoulder)], 0.4);
    // Left side should be minimal
    EXPECT_LT(pattern.intensities[static_cast<size_t>(VestZone::LeftSide)], 0.2);
}

TEST(VestHapticPatternTest, HeartbeatPattern) {
    auto pattern = VestHapticPattern::Heartbeat(72);  // 72 BPM

    // Chest center should be primary
    EXPECT_GT(pattern.intensities[static_cast<size_t>(VestZone::ChestCenter)], 0.5);

    // Duration should be based on BPM
    EXPECT_NEAR(pattern.duration, 60.0 / 72.0, 0.01);
}

TEST(VestHapticPatternTest, BreathingPattern) {
    auto pattern = VestHapticPattern::Breathing(12);  // 12 breaths per minute

    // Chest should be activated
    EXPECT_GT(pattern.intensities[static_cast<size_t>(VestZone::ChestCenter)], 0.5);

    // Effect should have proper envelope
    EXPECT_GT(pattern.base_effect.envelope.attack_time, 0.0);
    EXPECT_GT(pattern.base_effect.envelope.release_time, 0.0);
}

// ============================================================================
// Motion Cueing Parameters Tests
// ============================================================================

TEST(MotionCueingParamsTest, AircraftDefault) {
    auto params = MotionCueingParams::AircraftDefault();

    EXPECT_EQ(params.algorithm, WashoutAlgorithm::Classical);
    EXPECT_GT(params.tilt_coordination_gain, 0.0);

    // All DOFs should have reasonable high-pass cutoff
    for (size_t i = 0; i < 6; ++i) {
        EXPECT_GT(params.hp_cutoff_freq[i], 0.0);
        EXPECT_LE(params.hp_cutoff_freq[i], 2.0);
    }
}

TEST(MotionCueingParamsTest, GroundVehicleDefault) {
    auto params = MotionCueingParams::GroundVehicleDefault();

    // Ground vehicles need faster washout
    auto aircraft_params = MotionCueingParams::AircraftDefault();
    EXPECT_GE(params.hp_cutoff_freq[0], aircraft_params.hp_cutoff_freq[0]);
}

TEST(MotionCueingParamsTest, ShipDefault) {
    auto params = MotionCueingParams::ShipDefault();

    // Ships have slower motions, lower cutoff frequencies
    auto aircraft_params = MotionCueingParams::AircraftDefault();
    EXPECT_LE(params.hp_cutoff_freq[0], aircraft_params.hp_cutoff_freq[0]);

    // Heave emphasis for wave motion
    EXPECT_GT(params.scaling[static_cast<size_t>(MotionDOF::Heave)],
              params.scaling[static_cast<size_t>(MotionDOF::Surge)]);
}

// ============================================================================
// G-Force Configuration Tests
// ============================================================================

TEST(GForceConfigTest, FighterJetConfig) {
    auto config = GForceConfig::FighterJet();

    EXPECT_GE(config.max_sustained_g, 9.0);
    EXPECT_TRUE(config.simulate_gsuit);
    EXPECT_GT(config.gsuit_threshold, 0.0);
}

TEST(GForceConfigTest, CivilAircraftConfig) {
    auto config = GForceConfig::CivilAircraft();

    EXPECT_LE(config.max_sustained_g, 3.0);
    EXPECT_FALSE(config.simulate_gsuit);
}

TEST(GForceConfigTest, GroundVehicleConfig) {
    auto config = GForceConfig::GroundVehicle();

    EXPECT_LE(config.max_sustained_g, 2.5);
    EXPECT_LT(config.vertical_scale, 1.0);  // Less vertical emphasis
}

// ============================================================================
// Engine Vibration Tests
// ============================================================================

TEST(EngineVibrationTest, CombustionFrequency) {
    // 4-cylinder, 4-stroke at 3000 RPM
    Real freq = EngineVibrationParams::calculate_combustion_frequency(3000.0, 4, true);
    // (3000/60) * (4/2) = 50 * 2 = 100 Hz
    EXPECT_DOUBLE_EQ(freq, 100.0);

    // 6-cylinder, 4-stroke at 6000 RPM
    freq = EngineVibrationParams::calculate_combustion_frequency(6000.0, 6, true);
    // (6000/60) * (6/2) = 100 * 3 = 300 Hz
    EXPECT_DOUBLE_EQ(freq, 300.0);
}

TEST(EngineVibrationTest, PropellerFrequency) {
    // 2500 RPM with 3 blades
    Real freq = EngineVibrationParams::calculate_propeller_frequency(2500.0, 3);
    // (2500/60) * 3 = 41.67 * 3 = 125 Hz
    EXPECT_NEAR(freq, 125.0, 0.1);
}

// ============================================================================
// Mock Haptic Controller Tests
// ============================================================================

class HapticControllerTest : public ::testing::Test {
protected:
    void SetUp() override {
        renderer_ = create_mock_haptic_renderer();
        HapticRendererConfig config;
        config.auto_connect_devices = true;
        renderer_->initialize(config);
        controller_ = renderer_->get_controller();
    }

    void TearDown() override {
        renderer_->shutdown();
    }

    std::unique_ptr<IHapticRenderer> renderer_;
    std::shared_ptr<IHapticController> controller_;
};

TEST_F(HapticControllerTest, Connection) {
    EXPECT_TRUE(controller_->is_ready());
    EXPECT_EQ(controller_->get_type(), HapticDeviceType::Controller);
}

TEST_F(HapticControllerTest, Capabilities) {
    auto caps = controller_->get_capabilities();
    EXPECT_EQ(caps.device_type, HapticDeviceType::Controller);
    EXPECT_GE(caps.actuator_count, 1);
    EXPECT_GT(caps.frequency_max, caps.frequency_min);
}

TEST_F(HapticControllerTest, PlayEffect) {
    auto effect = HapticEffect::Vibration(0.5, 0.2);
    auto effect_id = controller_->play_effect(xr::XRHand::Left, effect);

    EXPECT_NE(effect_id, INVALID_HAPTIC_EFFECT_ID);

    auto state = controller_->get_effect_state(effect_id);
    EXPECT_TRUE(state.has_value());
    EXPECT_TRUE(state->is_playing);
}

TEST_F(HapticControllerTest, StopEffect) {
    auto effect = HapticEffect::Vibration(0.5, 1.0);
    auto effect_id = controller_->play_effect(xr::XRHand::Right, effect);

    EXPECT_EQ(controller_->stop_effect(effect_id), HapticResult::Success);

    auto state = controller_->get_effect_state(effect_id);
    EXPECT_FALSE(state.has_value());  // Effect should be removed
}

TEST_F(HapticControllerTest, StopAllOnHand) {
    controller_->play_effect(xr::XRHand::Left, HapticEffect::Vibration(0.5, 1.0));
    controller_->play_effect(xr::XRHand::Left, HapticEffect::Pulse(0.3, 0.1, 0.1, 5));

    EXPECT_EQ(controller_->stop_all(xr::XRHand::Left), HapticResult::Success);
}

TEST_F(HapticControllerTest, PlayVibration) {
    ControllerVibrationConfig config;
    config.hand = xr::XRHand::Left;
    config.low_frequency_amplitude = 0.5;
    config.high_frequency_amplitude = 0.3;

    EXPECT_EQ(controller_->play_vibration(config), HapticResult::Success);
}

// ============================================================================
// Mock Haptic Vest Tests
// ============================================================================

class HapticVestTest : public ::testing::Test {
protected:
    void SetUp() override {
        renderer_ = create_mock_haptic_renderer();
        HapticRendererConfig config;
        config.auto_connect_devices = true;
        renderer_->initialize(config);
        vest_ = renderer_->get_vest();
    }

    void TearDown() override {
        renderer_->shutdown();
    }

    std::unique_ptr<IHapticRenderer> renderer_;
    std::shared_ptr<IHapticVest> vest_;
};

TEST_F(HapticVestTest, Connection) {
    EXPECT_TRUE(vest_->is_ready());
    EXPECT_EQ(vest_->get_type(), HapticDeviceType::Vest);
}

TEST_F(HapticVestTest, Configuration) {
    auto config = vest_->get_config();
    EXPECT_EQ(config.zone_count, 16);
    EXPECT_GT(config.max_intensity, 0.0);
}

TEST_F(HapticVestTest, SetZoneIntensity) {
    EXPECT_EQ(vest_->set_zone_intensity(VestZone::ChestCenter, 0.8), HapticResult::Success);

    auto states = vest_->get_zone_states();
    EXPECT_DOUBLE_EQ(states[static_cast<size_t>(VestZone::ChestCenter)], 0.8);
}

TEST_F(HapticVestTest, SetZoneIntensitiesBatch) {
    std::array<Real, 16> intensities{};
    intensities.fill(0.5);
    intensities[0] = 0.9;

    EXPECT_EQ(vest_->set_zone_intensities(intensities), HapticResult::Success);

    auto states = vest_->get_zone_states();
    EXPECT_DOUBLE_EQ(states[0], 0.9);
    EXPECT_DOUBLE_EQ(states[1], 0.5);
}

TEST_F(HapticVestTest, PlayPattern) {
    VestHapticPattern pattern;
    pattern.set_all(0.6);

    auto effect_id = vest_->play_pattern(pattern);
    EXPECT_NE(effect_id, INVALID_HAPTIC_EFFECT_ID);
}

TEST_F(HapticVestTest, StopAll) {
    vest_->set_zone_intensity(VestZone::ChestCenter, 0.8);
    EXPECT_EQ(vest_->stop_all(), HapticResult::Success);

    auto states = vest_->get_zone_states();
    for (auto intensity : states) {
        EXPECT_DOUBLE_EQ(intensity, 0.0);
    }
}

// ============================================================================
// Mock Motion Platform Tests
// ============================================================================

class MotionPlatformTest : public ::testing::Test {
protected:
    void SetUp() override {
        renderer_ = create_mock_haptic_renderer();
        HapticRendererConfig config;
        config.auto_connect_devices = true;
        config.motion_cueing_params = MotionCueingParams::AircraftDefault();
        renderer_->initialize(config);
        platform_ = renderer_->get_motion_platform();
    }

    void TearDown() override {
        renderer_->shutdown();
    }

    std::unique_ptr<IHapticRenderer> renderer_;
    std::shared_ptr<IMotionPlatform> platform_;
};

TEST_F(MotionPlatformTest, Connection) {
    EXPECT_TRUE(platform_->is_ready());
    EXPECT_EQ(platform_->get_type(), HapticDeviceType::MotionPlatform);
}

TEST_F(MotionPlatformTest, Capabilities) {
    auto caps = platform_->get_platform_capabilities();

    // Should support all 6 DOF
    for (bool supported : caps.supported_dof) {
        EXPECT_TRUE(supported);
    }

    EXPECT_GT(caps.payload_capacity, 0.0);
    EXPECT_GT(caps.update_rate, 0.0);
}

TEST_F(MotionPlatformTest, State) {
    auto state = platform_->get_state();
    EXPECT_TRUE(state.is_homed);
    EXPECT_TRUE(state.is_enabled);
    EXPECT_FALSE(state.fault_active);
}

TEST_F(MotionPlatformTest, SendCommand) {
    MotionCommand cmd;
    cmd.position[static_cast<size_t>(MotionDOF::Pitch)] = 0.1;
    cmd.velocity[static_cast<size_t>(MotionDOF::Roll)] = 0.05;

    EXPECT_EQ(platform_->send_command(cmd), HapticResult::Success);

    auto state = platform_->get_state();
    EXPECT_NEAR(state.position[static_cast<size_t>(MotionDOF::Pitch)], 0.1, 0.01);
}

TEST_F(MotionPlatformTest, ApplyVehicleState) {
    Vec3 acceleration{2.0, 0.0, -9.81};  // 2G forward, 1G down
    Vec3 angular_velocity{0.0, 0.05, 0.0};
    Vec3 angular_acceleration{0.0, 0.0, 0.0};

    EXPECT_EQ(platform_->apply_vehicle_state(acceleration, angular_velocity, angular_acceleration),
              HapticResult::Success);
}

TEST_F(MotionPlatformTest, EmergencyStop) {
    EXPECT_EQ(platform_->emergency_stop(), HapticResult::Success);

    auto state = platform_->get_state();
    EXPECT_TRUE(state.fault_active);
    EXPECT_FALSE(state.is_enabled);
}

TEST_F(MotionPlatformTest, Park) {
    // First move platform
    MotionCommand cmd;
    cmd.position[0] = 0.1;
    cmd.position[1] = 0.1;
    platform_->send_command(cmd);

    // Then park
    EXPECT_EQ(platform_->park(), HapticResult::Success);

    auto state = platform_->get_state();
    for (Real pos : state.position) {
        EXPECT_NEAR(pos, 0.0, 0.01);
    }
}

// ============================================================================
// Haptic Renderer Tests
// ============================================================================

class HapticRendererTest : public ::testing::Test {
protected:
    void SetUp() override {
        renderer_ = create_mock_haptic_renderer();
    }

    std::unique_ptr<IHapticRenderer> renderer_;
};

TEST_F(HapticRendererTest, Initialize) {
    HapticRendererConfig config;
    config.master_intensity = 0.8;

    EXPECT_EQ(renderer_->initialize(config), HapticResult::Success);
    EXPECT_DOUBLE_EQ(renderer_->get_config().master_intensity, 0.8);
}

TEST_F(HapticRendererTest, Shutdown) {
    HapticRendererConfig config;
    renderer_->initialize(config);
    EXPECT_EQ(renderer_->shutdown(), HapticResult::Success);
}

TEST_F(HapticRendererTest, GetDevices) {
    HapticRendererConfig config;
    config.auto_connect_devices = true;
    renderer_->initialize(config);

    auto devices = renderer_->get_devices();
    EXPECT_GE(devices.size(), 3);  // Controller, Vest, Platform
}

TEST_F(HapticRendererTest, PlayControllerEffect) {
    HapticRendererConfig config;
    config.auto_connect_devices = true;
    renderer_->initialize(config);

    auto effect = HapticEffect::Vibration(0.5, 0.2);
    auto effect_id = renderer_->play_controller_effect(xr::XRHand::Left, effect);

    EXPECT_NE(effect_id, INVALID_HAPTIC_EFFECT_ID);
}

TEST_F(HapticRendererTest, PlayControllerVibration) {
    HapticRendererConfig config;
    config.auto_connect_devices = true;
    renderer_->initialize(config);

    EXPECT_EQ(renderer_->play_controller_vibration(xr::XRHand::Right, 0.5, 0.3, 0.2),
              HapticResult::Success);
}

TEST_F(HapticRendererTest, PlayVestPattern) {
    HapticRendererConfig config;
    config.auto_connect_devices = true;
    renderer_->initialize(config);

    VestHapticPattern pattern;
    pattern.set_front(0.7);

    auto effect_id = renderer_->play_vest_pattern(pattern);
    EXPECT_NE(effect_id, INVALID_HAPTIC_EFFECT_ID);
}

TEST_F(HapticRendererTest, PlayVestImpact) {
    HapticRendererConfig config;
    config.auto_connect_devices = true;
    renderer_->initialize(config);

    Vec3 direction{0.0, 0.0, 1.0};
    EXPECT_EQ(renderer_->play_vest_impact(direction, 0.8), HapticResult::Success);
}

TEST_F(HapticRendererTest, ApplyMotionCueing) {
    HapticRendererConfig config;
    config.auto_connect_devices = true;
    config.motion_cueing_params = MotionCueingParams::AircraftDefault();
    renderer_->initialize(config);

    Vec3 linear_accel{1.0, 0.5, -9.81};
    Vec3 angular_vel{0.0, 0.02, 0.0};
    Vec3 angular_accel{0.0, 0.0, 0.0};

    EXPECT_EQ(renderer_->apply_motion_cueing(linear_accel, angular_vel, angular_accel),
              HapticResult::Success);
}

TEST_F(HapticRendererTest, SetEngineVibration) {
    HapticRendererConfig config;
    config.auto_connect_devices = true;
    renderer_->initialize(config);

    EngineVibrationParams params;
    params.rpm = 3000.0;
    params.throttle = 0.5;
    params.source = EngineVibrationSource::Combustion;

    EXPECT_EQ(renderer_->set_engine_vibration(params), HapticResult::Success);
}

TEST_F(HapticRendererTest, CalculateGForce) {
    HapticRendererConfig config;
    config.g_force_config = GForceConfig::FighterJet();
    renderer_->initialize(config);

    // 3G pull-up: acceleration is 3*g upward in body frame
    Vec3 body_accel{0.0, 0.0, -3.0 * 9.81};
    Vec3 gravity_body{0.0, 0.0, -9.81};

    auto g_state = renderer_->calculate_g_force(body_accel, gravity_body);

    // Total G should be approximately 2 (3G accel - 1G gravity)
    EXPECT_NEAR(g_state.total_g, 2.0, 0.1);
}

TEST_F(HapticRendererTest, GSuitActivation) {
    HapticRendererConfig config;
    config.g_force_config = GForceConfig::FighterJet();
    config.g_force_config.gsuit_threshold = 2.0;
    renderer_->initialize(config);

    // High-G maneuver
    Vec3 body_accel{0.0, 0.0, -5.0 * 9.81};
    Vec3 gravity_body{0.0, 0.0, -9.81};

    auto g_state = renderer_->calculate_g_force(body_accel, gravity_body);

    EXPECT_TRUE(g_state.gsuit_active);
    EXPECT_GT(g_state.gsuit_pressure, 0.0);
}

TEST_F(HapticRendererTest, Update) {
    HapticRendererConfig config;
    config.auto_connect_devices = true;
    renderer_->initialize(config);

    // Play some effects
    renderer_->play_controller_effect(xr::XRHand::Left, HapticEffect::Vibration(0.5, 0.1));

    // Update
    EXPECT_EQ(renderer_->update(0.016), HapticResult::Success);  // ~60fps
}

TEST_F(HapticRendererTest, Statistics) {
    HapticRendererConfig config;
    config.auto_connect_devices = true;
    renderer_->initialize(config);

    renderer_->play_controller_effect(xr::XRHand::Left, HapticEffect::Vibration(0.5, 0.5));
    renderer_->update(0.016);

    auto stats = renderer_->get_stats();
    EXPECT_GE(stats.frames_rendered, 1);
    EXPECT_GE(stats.effects_played, 1);
    EXPECT_GE(stats.connected_devices, 1);
}

TEST_F(HapticRendererTest, EmergencyStopAll) {
    HapticRendererConfig config;
    config.auto_connect_devices = true;
    renderer_->initialize(config);

    EXPECT_EQ(renderer_->emergency_stop_all(), HapticResult::Success);
}

TEST_F(HapticRendererTest, MasterIntensity) {
    HapticRendererConfig config;
    config.master_intensity = 1.0;
    renderer_->initialize(config);

    renderer_->set_master_intensity(0.5);
    EXPECT_DOUBLE_EQ(renderer_->get_config().master_intensity, 0.5);

    // Clamp to valid range
    renderer_->set_master_intensity(1.5);
    EXPECT_DOUBLE_EQ(renderer_->get_config().master_intensity, 1.0);
}

// ============================================================================
// Preset Effect Tests
// ============================================================================

TEST(AircraftPresetsTest, StallBuffet) {
    auto effect = presets::aircraft::StallBuffet(0.7);
    EXPECT_EQ(effect.waveform, HapticWaveform::Noise);
    EXPECT_DOUBLE_EQ(effect.amplitude, 0.7);
    EXPECT_GT(effect.frequency_variation, 0.0);
}

TEST(AircraftPresetsTest, Touchdown) {
    auto soft = presets::aircraft::Touchdown(1.0);  // Soft landing
    auto hard = presets::aircraft::Touchdown(3.0);  // Hard landing

    EXPECT_LT(soft.amplitude, hard.amplitude);
    EXPECT_LT(soft.duration, hard.duration);
}

TEST(AircraftPresetsTest, AfterburnerIgnition) {
    auto effect = presets::aircraft::AfterburnerIgnition();
    EXPECT_GT(effect.amplitude, 0.7);
    EXPECT_GT(effect.envelope.attack_time, 0.0);
}

TEST(VehiclePresetsTest, RoadTexture) {
    auto smooth = presets::vehicle::RoadTexture(0.2);
    auto rough = presets::vehicle::RoadTexture(0.8);

    EXPECT_LT(smooth.amplitude, rough.amplitude);
    EXPECT_LT(smooth.frequency, rough.frequency);
}

TEST(VehiclePresetsTest, FiringRecoil) {
    auto small = presets::vehicle::FiringRecoil(30.0);   // 30mm cannon
    auto large = presets::vehicle::FiringRecoil(125.0);  // 125mm tank gun

    EXPECT_LT(small.amplitude, large.amplitude);
}

TEST(NavalPresetsTest, WaveMotion) {
    auto calm = presets::naval::WaveMotion(2.0);   // Slight sea
    auto rough = presets::naval::WaveMotion(7.0);  // High sea

    EXPECT_LT(calm.amplitude, rough.amplitude);
}

TEST(HelicopterPresetsTest, RotorVibration) {
    auto effect = presets::helicopter::RotorVibration(300.0);  // 300 RPM

    EXPECT_EQ(effect.waveform, HapticWaveform::Sine);
    // Blade pass frequency for 4-blade rotor at 300 RPM
    Real expected_freq = (300.0 / 60.0) * 4.0;
    EXPECT_NEAR(effect.frequency, expected_freq, 0.1);
}

TEST(HelicopterPresetsTest, HardLanding) {
    auto soft = presets::helicopter::HardLanding(2.0);
    auto hard = presets::helicopter::HardLanding(5.0);

    EXPECT_LT(soft.amplitude, hard.amplitude);
}

// ============================================================================
// Integration Tests
// ============================================================================

TEST(HapticIntegrationTest, FullFlightSequence) {
    auto renderer = create_mock_haptic_renderer();
    HapticRendererConfig config;
    config.auto_connect_devices = true;
    config.g_force_config = GForceConfig::FighterJet();
    config.motion_cueing_params = MotionCueingParams::AircraftDefault();
    renderer->initialize(config);

    // Engine start
    for (Real progress = 0.0; progress <= 1.0; progress += 0.1) {
        auto effect = presets::aircraft::EngineStart(progress);
        renderer->play_controller_effect(xr::XRHand::Left, effect);
        renderer->update(0.5);
    }

    // Taxi and takeoff
    EngineVibrationParams engine;
    engine.source = EngineVibrationSource::Turbine;
    engine.rpm = 14000;
    engine.throttle = 1.0;
    renderer->set_engine_vibration(engine);

    // Flight with G-forces
    Vec3 accel{0.0, 0.0, -4.0 * 9.81};  // 4G pull
    Vec3 gravity{0.0, 0.0, -9.81};
    auto g_state = renderer->calculate_g_force(accel, gravity);
    renderer->apply_g_force(g_state);

    // Touchdown
    auto touchdown = presets::aircraft::Touchdown(2.0);
    renderer->play_vest_impact(Vec3{0.0, -1.0, 0.0}, 0.8);
    renderer->play_controller_effect(xr::XRHand::Left, touchdown);
    renderer->play_controller_effect(xr::XRHand::Right, touchdown);

    renderer->update(0.016);

    auto stats = renderer->get_stats();
    EXPECT_GT(stats.effects_played, 10);

    renderer->shutdown();
}

TEST(HapticIntegrationTest, TankCombat) {
    auto renderer = create_mock_haptic_renderer();
    HapticRendererConfig config;
    config.auto_connect_devices = true;
    config.motion_cueing_params = MotionCueingParams::GroundVehicleDefault();
    renderer->initialize(config);

    // Engine running
    EngineVibrationParams engine;
    engine.source = EngineVibrationSource::Combustion;
    engine.rpm = 2500;
    engine.throttle = 0.7;
    renderer->set_engine_vibration(engine);

    // Track rumble
    auto track = presets::vehicle::TrackRumble(15.0);  // 15 m/s
    renderer->play_controller_effect(xr::XRHand::Left, track);

    // Fire main gun
    auto recoil = presets::vehicle::FiringRecoil(125.0);  // 125mm
    renderer->play_controller_effect(xr::XRHand::Left, recoil);
    renderer->play_controller_effect(xr::XRHand::Right, recoil);
    renderer->play_vest_impact(Vec3{0.0, 0.0, -1.0}, 1.0);

    renderer->update(0.016);

    renderer->shutdown();
}

// ============================================================================
// Main
// ============================================================================

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
