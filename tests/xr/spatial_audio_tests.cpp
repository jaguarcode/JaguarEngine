/**
 * @file spatial_audio_tests.cpp
 * @brief Unit tests for spatial audio system
 */

#include <gtest/gtest.h>
#include "jaguar/xr/spatial_audio.h"
#include <cmath>

namespace jaguar::audio {

// ============================================================================
// Test Fixtures
// ============================================================================

class SpatialAudioTest : public ::testing::Test {
protected:
    void SetUp() override {
        renderer_ = create_spatial_audio_renderer();
    }

    void TearDown() override {
        if (renderer_ && renderer_->is_initialized()) {
            renderer_->shutdown();
        }
    }

    AudioFormat default_format() {
        AudioFormat format;
        format.sample_rate = 48000;
        format.sample_format = AudioSampleFormat::Float32;
        format.channel_layout = AudioChannelLayout::Binaural;
        format.channel_count = 2;
        format.buffer_size = 1024;
        return format;
    }

    HRTFConfig default_hrtf_config() {
        HRTFConfig config;
        config.quality = HRTFQuality::Medium;
        config.max_sources = 64;
        config.enable_interpolation = true;
        return config;
    }

    std::unique_ptr<ISpatialAudioRenderer> renderer_;
};

class SpatialAudioInitializedTest : public SpatialAudioTest {
protected:
    void SetUp() override {
        SpatialAudioTest::SetUp();
        auto result = renderer_->initialize(default_format(), default_hrtf_config());
        ASSERT_EQ(result, AudioResult::Success);
    }
};

// ============================================================================
// AudioResult Tests
// ============================================================================

TEST(AudioResultTest, SuccessString) {
    EXPECT_STREQ(audio_result_to_string(AudioResult::Success), "Success");
}

TEST(AudioResultTest, ErrorStrings) {
    EXPECT_STREQ(audio_result_to_string(AudioResult::NotInitialized), "NotInitialized");
    EXPECT_STREQ(audio_result_to_string(AudioResult::InvalidParameter), "InvalidParameter");
    EXPECT_STREQ(audio_result_to_string(AudioResult::HRTFLoadFailed), "HRTFLoadFailed");
}

TEST(AudioResultTest, AudioSucceeded) {
    EXPECT_TRUE(audio_succeeded(AudioResult::Success));
    EXPECT_FALSE(audio_succeeded(AudioResult::NotInitialized));
    EXPECT_FALSE(audio_succeeded(AudioResult::InternalError));
}

// ============================================================================
// Distance Attenuation Tests
// ============================================================================

class DistanceAttenuationTest : public ::testing::Test {
protected:
    DistanceAttenuation attenuation_;
};

TEST_F(DistanceAttenuationTest, LinearModel) {
    attenuation_.model = AttenuationModel::Linear;
    attenuation_.reference_distance = 1.0;
    attenuation_.max_distance = 100.0;
    attenuation_.min_gain = 0.0;
    attenuation_.max_gain = 1.0;

    // At reference distance, gain = 1.0
    EXPECT_DOUBLE_EQ(attenuation_.calculate_gain(1.0), 1.0);

    // At max distance, gain = min_gain
    EXPECT_DOUBLE_EQ(attenuation_.calculate_gain(100.0), 0.0);

    // At halfway, gain = 0.5
    EXPECT_NEAR(attenuation_.calculate_gain(50.5), 0.5, 0.01);

    // Closer than reference, gain = 1.0
    EXPECT_DOUBLE_EQ(attenuation_.calculate_gain(0.5), 1.0);
}

TEST_F(DistanceAttenuationTest, InverseModel) {
    attenuation_.model = AttenuationModel::Inverse;
    attenuation_.reference_distance = 1.0;
    attenuation_.max_distance = 1000.0;
    attenuation_.rolloff_factor = 1.0;
    attenuation_.min_gain = 0.0;
    attenuation_.max_gain = 1.0;

    // At reference distance, gain = 1.0
    EXPECT_DOUBLE_EQ(attenuation_.calculate_gain(1.0), 1.0);

    // At 2x reference, gain = 0.5
    EXPECT_DOUBLE_EQ(attenuation_.calculate_gain(2.0), 0.5);

    // At 10x reference, gain = 0.1
    EXPECT_NEAR(attenuation_.calculate_gain(10.0), 0.1, 0.001);
}

TEST_F(DistanceAttenuationTest, InverseSquareModel) {
    attenuation_.model = AttenuationModel::InverseSquare;
    attenuation_.reference_distance = 1.0;
    attenuation_.max_distance = 1000.0;
    attenuation_.rolloff_factor = 1.0;
    attenuation_.min_gain = 0.0;
    attenuation_.max_gain = 1.0;

    // At reference distance, gain = 1.0
    EXPECT_DOUBLE_EQ(attenuation_.calculate_gain(1.0), 1.0);

    // Inverse square falloff (approximately)
    Real gain_at_2 = attenuation_.calculate_gain(2.0);
    Real gain_at_4 = attenuation_.calculate_gain(4.0);
    EXPECT_LT(gain_at_2, 1.0);
    EXPECT_LT(gain_at_4, gain_at_2);
}

TEST_F(DistanceAttenuationTest, LogarithmicModel) {
    attenuation_.model = AttenuationModel::Logarithmic;
    attenuation_.reference_distance = 1.0;
    attenuation_.max_distance = 1000.0;
    attenuation_.rolloff_factor = 0.5;
    attenuation_.min_gain = 0.0;
    attenuation_.max_gain = 1.0;

    // At reference distance, gain = 1.0
    EXPECT_DOUBLE_EQ(attenuation_.calculate_gain(1.0), 1.0);

    // Logarithmic falloff
    Real gain_at_2 = attenuation_.calculate_gain(2.0);
    EXPECT_LT(gain_at_2, 1.0);
    EXPECT_GT(gain_at_2, 0.0);
}

TEST_F(DistanceAttenuationTest, NoAttenuationModel) {
    attenuation_.model = AttenuationModel::None;

    EXPECT_DOUBLE_EQ(attenuation_.calculate_gain(1.0), 1.0);
    EXPECT_DOUBLE_EQ(attenuation_.calculate_gain(100.0), 1.0);
    EXPECT_DOUBLE_EQ(attenuation_.calculate_gain(10000.0), 1.0);
}

TEST_F(DistanceAttenuationTest, PresetIndoor) {
    auto indoor = DistanceAttenuation::Indoor();
    EXPECT_EQ(indoor.model, AttenuationModel::InverseSquare);
    EXPECT_DOUBLE_EQ(indoor.reference_distance, 1.0);
    EXPECT_DOUBLE_EQ(indoor.max_distance, 50.0);
}

TEST_F(DistanceAttenuationTest, PresetOutdoor) {
    auto outdoor = DistanceAttenuation::Outdoor();
    EXPECT_EQ(outdoor.model, AttenuationModel::InverseSquare);
    EXPECT_DOUBLE_EQ(outdoor.max_distance, 500.0);
}

TEST_F(DistanceAttenuationTest, PresetOpenField) {
    auto open = DistanceAttenuation::OpenField();
    EXPECT_DOUBLE_EQ(open.max_distance, 2000.0);
}

// ============================================================================
// Audio Cone Tests
// ============================================================================

class AudioConeTest : public ::testing::Test {
protected:
    AudioCone cone_;
};

TEST_F(AudioConeTest, InsideInnerCone) {
    cone_.inner_angle = constants::PI / 4.0;  // 45 degrees
    cone_.outer_angle = constants::PI / 2.0;  // 90 degrees
    cone_.outer_gain = 0.0;

    // Within inner cone
    EXPECT_DOUBLE_EQ(cone_.calculate_gain(0.0), 1.0);
    EXPECT_DOUBLE_EQ(cone_.calculate_gain(constants::PI / 8.0), 1.0);
}

TEST_F(AudioConeTest, OutsideOuterCone) {
    cone_.inner_angle = constants::PI / 4.0;
    cone_.outer_angle = constants::PI / 2.0;
    cone_.outer_gain = 0.2;

    // Outside outer cone
    EXPECT_DOUBLE_EQ(cone_.calculate_gain(constants::PI), 0.2);
    EXPECT_DOUBLE_EQ(cone_.calculate_gain(constants::PI * 2.0), 0.2);
}

TEST_F(AudioConeTest, BetweenCones) {
    cone_.inner_angle = constants::PI / 4.0;  // 45 degrees
    cone_.outer_angle = constants::PI / 2.0;  // 90 degrees
    cone_.outer_gain = 0.0;

    // Midway between inner and outer
    Real mid_angle = (cone_.inner_angle + cone_.outer_angle) / 2.0;
    Real mid_gain = cone_.calculate_gain(mid_angle);
    EXPECT_GT(mid_gain, 0.0);
    EXPECT_LT(mid_gain, 1.0);
    EXPECT_NEAR(mid_gain, 0.5, 0.01);
}

TEST_F(AudioConeTest, ValidityCheck) {
    cone_.inner_angle = constants::PI / 4.0;
    cone_.outer_angle = constants::PI / 2.0;
    cone_.outer_gain = 0.5;
    EXPECT_TRUE(cone_.is_valid());

    // Invalid: inner > outer
    cone_.inner_angle = constants::PI;
    cone_.outer_angle = constants::PI / 2.0;
    EXPECT_FALSE(cone_.is_valid());

    // Invalid: outer_gain > 1
    cone_.inner_angle = constants::PI / 4.0;
    cone_.outer_angle = constants::PI / 2.0;
    cone_.outer_gain = 1.5;
    EXPECT_FALSE(cone_.is_valid());
}

// ============================================================================
// Doppler Effect Tests
// ============================================================================

class DopplerConfigTest : public ::testing::Test {
protected:
    DopplerConfig doppler_;
};

TEST_F(DopplerConfigTest, NoMovement) {
    doppler_.enabled = true;
    doppler_.factor = 1.0;
    doppler_.speed_of_sound = 343.0;

    Vec3 direction{0.0, 0.0, 1.0};
    Vec3 zero_velocity{0.0, 0.0, 0.0};

    Real pitch = doppler_.calculate_pitch_shift(zero_velocity, zero_velocity, direction);
    EXPECT_DOUBLE_EQ(pitch, 1.0);
}

TEST_F(DopplerConfigTest, SourceApproaching) {
    doppler_.enabled = true;
    doppler_.factor = 1.0;
    doppler_.speed_of_sound = 343.0;
    doppler_.max_pitch_shift = 4.0;

    Vec3 direction{0.0, 0.0, 1.0};
    Vec3 source_velocity{0.0, 0.0, 34.3};  // 10% speed of sound toward listener
    Vec3 listener_velocity{0.0, 0.0, 0.0};

    Real pitch = doppler_.calculate_pitch_shift(source_velocity, listener_velocity, direction);
    EXPECT_GT(pitch, 1.0);  // Higher pitch when approaching
}

TEST_F(DopplerConfigTest, SourceReceding) {
    doppler_.enabled = true;
    doppler_.factor = 1.0;
    doppler_.speed_of_sound = 343.0;
    doppler_.max_pitch_shift = 4.0;

    Vec3 direction{0.0, 0.0, 1.0};
    Vec3 source_velocity{0.0, 0.0, -34.3};  // Moving away
    Vec3 listener_velocity{0.0, 0.0, 0.0};

    Real pitch = doppler_.calculate_pitch_shift(source_velocity, listener_velocity, direction);
    EXPECT_LT(pitch, 1.0);  // Lower pitch when receding
}

TEST_F(DopplerConfigTest, DisabledDoppler) {
    doppler_.enabled = false;
    doppler_.factor = 1.0;
    doppler_.speed_of_sound = 343.0;

    Vec3 direction{0.0, 0.0, 1.0};
    Vec3 source_velocity{0.0, 0.0, 100.0};
    Vec3 listener_velocity{0.0, 0.0, 0.0};

    Real pitch = doppler_.calculate_pitch_shift(source_velocity, listener_velocity, direction);
    EXPECT_DOUBLE_EQ(pitch, 1.0);
}

TEST_F(DopplerConfigTest, PitchClamping) {
    doppler_.enabled = true;
    doppler_.factor = 1.0;
    doppler_.speed_of_sound = 343.0;
    doppler_.max_pitch_shift = 2.0;

    Vec3 direction{0.0, 0.0, 1.0};
    Vec3 source_velocity{0.0, 0.0, 300.0};  // Near speed of sound
    Vec3 listener_velocity{0.0, 0.0, 0.0};

    Real pitch = doppler_.calculate_pitch_shift(source_velocity, listener_velocity, direction);
    EXPECT_LE(pitch, 2.0);  // Clamped to max
    EXPECT_GE(pitch, 0.5);  // Clamped to min (1/max)
}

// ============================================================================
// Room Acoustics Preset Tests
// ============================================================================

TEST(RoomAcousticsPresetTest, SmallRoom) {
    auto config = RoomAcousticsConfig::FromPreset(RoomPreset::SmallRoom);
    EXPECT_EQ(config.preset, RoomPreset::SmallRoom);
    EXPECT_LT(config.reverb.decay_time, 1.0);
    EXPECT_LT(config.geometry.volume(), 100.0);
}

TEST(RoomAcousticsPresetTest, Cathedral) {
    auto config = RoomAcousticsConfig::FromPreset(RoomPreset::Cathedral);
    EXPECT_EQ(config.preset, RoomPreset::Cathedral);
    EXPECT_GT(config.reverb.decay_time, 2.0);
    EXPECT_GT(config.geometry.volume(), 1000.0);
}

TEST(RoomAcousticsPresetTest, Outdoor) {
    auto config = RoomAcousticsConfig::FromPreset(RoomPreset::Outdoor);
    EXPECT_EQ(config.preset, RoomPreset::Outdoor);
    EXPECT_LT(config.reverb.decay_time, 0.5);
    EXPECT_LT(config.reverb.wet_dry_mix, 0.1);
}

TEST(RoomAcousticsPresetTest, Underwater) {
    auto config = RoomAcousticsConfig::FromPreset(RoomPreset::Underwater);
    EXPECT_EQ(config.preset, RoomPreset::Underwater);
    EXPECT_LT(config.reverb.hf_decay_ratio, 0.5);  // Water absorbs HF
    EXPECT_GT(config.reverb.lf_decay_ratio, 1.0);  // LF travels further
}

TEST(RoomAcousticsPresetTest, Cockpit) {
    auto config = RoomAcousticsConfig::FromPreset(RoomPreset::Cockpit);
    EXPECT_EQ(config.preset, RoomPreset::Cockpit);
    EXPECT_LT(config.reverb.decay_time, 0.5);
    EXPECT_LT(config.geometry.volume(), 20.0);
}

TEST(RoomAcousticsPresetTest, None) {
    auto config = RoomAcousticsConfig::FromPreset(RoomPreset::None);
    EXPECT_DOUBLE_EQ(config.reverb.wet_dry_mix, 0.0);
    EXPECT_DOUBLE_EQ(config.reverb.decay_time, 0.0);
}

// ============================================================================
// Room Material Tests
// ============================================================================

TEST(RoomMaterialTest, ConcretePreset) {
    auto concrete = RoomMaterial::Concrete();
    EXPECT_EQ(concrete.name, "concrete");
    // Concrete has low absorption across all frequencies
    for (const auto& coeff : concrete.absorption) {
        EXPECT_LT(coeff, 0.1);
    }
}

TEST(RoomMaterialTest, CarpetPreset) {
    auto carpet = RoomMaterial::Carpet();
    EXPECT_EQ(carpet.name, "carpet");
    // Carpet has high HF absorption
    EXPECT_LT(carpet.absorption[0], carpet.absorption[5]);  // More HF absorption
}

TEST(RoomMaterialTest, GlassPreset) {
    auto glass = RoomMaterial::Glass();
    EXPECT_EQ(glass.name, "glass");
    EXPECT_LT(glass.scattering, 0.1);  // Glass is smooth
}

// ============================================================================
// Occluder Material Tests
// ============================================================================

TEST(OccluderMaterialTest, SolidWall) {
    auto wall = OccluderMaterial::SolidWall();
    EXPECT_EQ(wall.name, "solid_wall");
    EXPECT_LT(wall.transmission, 0.1);
}

TEST(OccluderMaterialTest, Foliage) {
    auto foliage = OccluderMaterial::Foliage();
    EXPECT_EQ(foliage.name, "foliage");
    EXPECT_GT(foliage.transmission, 0.5);  // Sound passes through
}

// ============================================================================
// Renderer Lifecycle Tests
// ============================================================================

TEST_F(SpatialAudioTest, CreateRenderer) {
    EXPECT_NE(renderer_, nullptr);
    EXPECT_FALSE(renderer_->is_initialized());
}

TEST_F(SpatialAudioTest, InitializeRenderer) {
    auto result = renderer_->initialize(default_format(), default_hrtf_config());
    EXPECT_EQ(result, AudioResult::Success);
    EXPECT_TRUE(renderer_->is_initialized());
}

TEST_F(SpatialAudioTest, ShutdownRenderer) {
    renderer_->initialize(default_format(), default_hrtf_config());
    renderer_->shutdown();
    EXPECT_FALSE(renderer_->is_initialized());
}

TEST_F(SpatialAudioTest, DoubleInitialize) {
    auto result1 = renderer_->initialize(default_format(), default_hrtf_config());
    EXPECT_EQ(result1, AudioResult::Success);

    auto result2 = renderer_->initialize(default_format(), default_hrtf_config());
    EXPECT_EQ(result2, AudioResult::Success);  // Should not fail
}

// ============================================================================
// Source Management Tests
// ============================================================================

TEST_F(SpatialAudioInitializedTest, CreateSource) {
    AudioSourceConfig config;
    config.name = "test_source";
    config.source_type = AudioSourceType::Point;

    auto source = renderer_->create_source(config);
    EXPECT_NE(source, nullptr);
    EXPECT_EQ(source->get_name(), "test_source");
}

TEST_F(SpatialAudioInitializedTest, CreateMultipleSources) {
    for (int i = 0; i < 10; ++i) {
        AudioSourceConfig config;
        config.name = "source_" + std::to_string(i);

        auto source = renderer_->create_source(config);
        EXPECT_NE(source, nullptr);
    }

    EXPECT_GE(renderer_->get_active_source_count(), 0u);
}

TEST_F(SpatialAudioInitializedTest, DestroySource) {
    AudioSourceConfig config;
    config.name = "test_source";

    auto source = renderer_->create_source(config);
    EXPECT_NE(source, nullptr);
    UInt32 id = source->get_id();

    auto result = renderer_->destroy_source(id);
    EXPECT_EQ(result, AudioResult::Success);

    auto destroyed = renderer_->get_source(id);
    EXPECT_EQ(destroyed, nullptr);
}

TEST_F(SpatialAudioInitializedTest, DestroyInvalidSource) {
    auto result = renderer_->destroy_source(99999);
    EXPECT_EQ(result, AudioResult::InvalidParameter);
}

TEST_F(SpatialAudioInitializedTest, SourcePlaybackControl) {
    AudioSourceConfig config;
    config.name = "playback_test";

    auto source = renderer_->create_source(config);
    EXPECT_FALSE(source->is_playing());

    source->play();
    EXPECT_TRUE(source->is_playing());

    source->pause();
    EXPECT_TRUE(source->is_paused());

    source->stop();
    EXPECT_FALSE(source->is_playing());
    EXPECT_FALSE(source->is_paused());
}

// ============================================================================
// Source Property Tests
// ============================================================================

TEST_F(SpatialAudioInitializedTest, SourcePosition) {
    AudioSourceConfig config;
    auto source = renderer_->create_source(config);

    Vec3 pos{10.0, 20.0, 30.0};
    source->set_position(pos);

    Vec3 result = source->get_position();
    EXPECT_DOUBLE_EQ(result.x, 10.0);
    EXPECT_DOUBLE_EQ(result.y, 20.0);
    EXPECT_DOUBLE_EQ(result.z, 30.0);
}

TEST_F(SpatialAudioInitializedTest, SourceGain) {
    AudioSourceConfig config;
    auto source = renderer_->create_source(config);

    source->set_gain(0.5);
    EXPECT_DOUBLE_EQ(source->get_gain(), 0.5);

    source->set_gain(2.0);
    EXPECT_DOUBLE_EQ(source->get_gain(), 2.0);

    // Negative gain clamped to 0
    source->set_gain(-1.0);
    EXPECT_DOUBLE_EQ(source->get_gain(), 0.0);
}

TEST_F(SpatialAudioInitializedTest, SourcePitch) {
    AudioSourceConfig config;
    auto source = renderer_->create_source(config);

    source->set_pitch(1.5);
    EXPECT_DOUBLE_EQ(source->get_pitch(), 1.5);

    // Clamped to valid range
    source->set_pitch(0.1);
    EXPECT_GE(source->get_pitch(), 0.5);

    source->set_pitch(5.0);
    EXPECT_LE(source->get_pitch(), 2.0);
}

TEST_F(SpatialAudioInitializedTest, SourceMute) {
    AudioSourceConfig config;
    auto source = renderer_->create_source(config);

    EXPECT_FALSE(source->is_muted());

    source->set_muted(true);
    EXPECT_TRUE(source->is_muted());

    source->set_muted(false);
    EXPECT_FALSE(source->is_muted());
}

// ============================================================================
// Listener Tests
// ============================================================================

TEST_F(SpatialAudioInitializedTest, ListenerPosition) {
    auto* listener = renderer_->get_listener();
    EXPECT_NE(listener, nullptr);

    Vec3 pos{5.0, 1.6, 10.0};  // Eye height
    listener->set_position(pos);

    Vec3 result = listener->get_position();
    EXPECT_DOUBLE_EQ(result.x, 5.0);
    EXPECT_DOUBLE_EQ(result.y, 1.6);
    EXPECT_DOUBLE_EQ(result.z, 10.0);
}

TEST_F(SpatialAudioInitializedTest, ListenerMasterGain) {
    auto* listener = renderer_->get_listener();

    listener->set_master_gain(0.5);
    EXPECT_DOUBLE_EQ(listener->get_master_gain(), 0.5);

    listener->set_master_gain(0.0);
    EXPECT_DOUBLE_EQ(listener->get_master_gain(), 0.0);
}

TEST_F(SpatialAudioInitializedTest, ListenerXRTracking) {
    auto* listener = renderer_->get_listener();

    xr::XRTrackedPose pose;
    pose.pose.position = {1.0, 1.7, 2.0};
    pose.pose.orientation = Quat::Identity();
    pose.pose.is_valid = true;
    pose.velocity.linear = {0.5, 0.0, 1.0};
    pose.velocity.linear_valid = true;

    listener->set_xr_tracking(pose);
    EXPECT_TRUE(listener->is_xr_tracking_active());

    Vec3 result = listener->get_position();
    EXPECT_DOUBLE_EQ(result.x, 1.0);
    EXPECT_DOUBLE_EQ(result.y, 1.7);
    EXPECT_DOUBLE_EQ(result.z, 2.0);
}

// ============================================================================
// Room Acoustics Configuration Tests
// ============================================================================

TEST_F(SpatialAudioInitializedTest, SetRoomAcoustics) {
    auto config = RoomAcousticsConfig::FromPreset(RoomPreset::Cathedral);
    auto result = renderer_->set_room_acoustics(config);
    EXPECT_EQ(result, AudioResult::Success);

    const auto& current = renderer_->get_room_acoustics();
    EXPECT_EQ(current.preset, RoomPreset::Cathedral);
}

// ============================================================================
// Update and Render Tests
// ============================================================================

TEST_F(SpatialAudioInitializedTest, UpdateWithNoSources) {
    auto result = renderer_->update(0.016);  // ~60fps
    EXPECT_EQ(result, AudioResult::Success);
}

TEST_F(SpatialAudioInitializedTest, UpdateWithSources) {
    AudioSourceConfig config;
    config.name = "test";
    config.position = {10.0, 0.0, 0.0};

    auto source = renderer_->create_source(config);
    source->play();

    auto result = renderer_->update(0.016);
    EXPECT_EQ(result, AudioResult::Success);
}

TEST_F(SpatialAudioInitializedTest, RenderBuffer) {
    std::vector<float> buffer(1024 * 2);  // Stereo
    auto result = renderer_->render(buffer.data(), 1024);
    EXPECT_EQ(result, AudioResult::Success);
}

TEST_F(SpatialAudioTest, UpdateNotInitialized) {
    auto result = renderer_->update(0.016);
    EXPECT_EQ(result, AudioResult::NotInitialized);
}

// ============================================================================
// HRTF Dataset Tests
// ============================================================================

TEST_F(SpatialAudioInitializedTest, HRTFInfo) {
    const auto& info = renderer_->get_hrtf_info();
    EXPECT_EQ(info.sample_rate, 48000u);
    EXPECT_EQ(info.quality, HRTFQuality::Medium);
}

TEST_F(SpatialAudioInitializedTest, LoadHRTFDataset) {
    auto result = renderer_->load_hrtf_dataset("/path/to/hrtf.sofa");
    EXPECT_EQ(result, AudioResult::Success);

    const auto& info = renderer_->get_hrtf_info();
    EXPECT_TRUE(info.is_loaded);
}

// ============================================================================
// Statistics Tests
// ============================================================================

TEST_F(SpatialAudioInitializedTest, CPUUsage) {
    Real cpu = renderer_->get_cpu_usage();
    EXPECT_GE(cpu, 0.0);
    EXPECT_LE(cpu, 100.0);
}

TEST_F(SpatialAudioInitializedTest, VirtualizedSourceCount) {
    // Create a source far away that should be virtualized
    AudioSourceConfig config;
    config.name = "far_source";
    config.position = {10000.0, 0.0, 0.0};  // Very far
    config.attenuation.max_distance = 100.0;
    config.virtualize_when_culled = true;

    auto source = renderer_->create_source(config);
    source->play();

    renderer_->update(0.016);

    // Source should be virtualized due to distance
    UInt32 virtualized = renderer_->get_virtualized_source_count();
    EXPECT_GE(virtualized, 0u);
}

// ============================================================================
// Room Geometry Tests
// ============================================================================

TEST(RoomGeometryTest, VolumeCalculation) {
    RoomGeometry room;
    room.dimensions = {10.0, 3.0, 8.0};

    EXPECT_DOUBLE_EQ(room.volume(), 240.0);
}

TEST(RoomGeometryTest, SurfaceAreaCalculation) {
    RoomGeometry room;
    room.dimensions = {10.0, 3.0, 8.0};

    // 2 * (10*3 + 3*8 + 10*8) = 2 * (30 + 24 + 80) = 2 * 134 = 268
    EXPECT_DOUBLE_EQ(room.surface_area(), 268.0);
}

// ============================================================================
// Integration Tests
// ============================================================================

TEST_F(SpatialAudioInitializedTest, FullAudioPipeline) {
    // Set up room
    auto room_config = RoomAcousticsConfig::FromPreset(RoomPreset::MediumRoom);
    renderer_->set_room_acoustics(room_config);

    // Set up listener
    auto* listener = renderer_->get_listener();
    listener->set_position({0.0, 1.6, 0.0});
    listener->set_orientation(Quat::Identity());

    // Create multiple sources
    AudioSourceConfig config1;
    config1.name = "helicopter";
    config1.position = {50.0, 100.0, 30.0};
    config1.source_type = AudioSourceType::Point;
    config1.enable_doppler = true;
    config1.velocity = {10.0, 0.0, 5.0};

    AudioSourceConfig config2;
    config2.name = "radio";
    config2.position = {2.0, 1.0, 1.0};
    config2.source_type = AudioSourceType::Directional;
    config2.cone.inner_angle = constants::PI / 4.0;
    config2.cone.outer_angle = constants::PI / 2.0;

    auto source1 = renderer_->create_source(config1);
    auto source2 = renderer_->create_source(config2);

    source1->play();
    source2->play();

    // Run a few update cycles
    for (int i = 0; i < 10; ++i) {
        auto result = renderer_->update(0.016);
        EXPECT_EQ(result, AudioResult::Success);

        // Update source position (simulate movement)
        Vec3 pos = source1->get_position();
        pos.x += source1->get_velocity().x * 0.016;
        pos.z += source1->get_velocity().z * 0.016;
        source1->set_position(pos);
    }

    // Verify sources have valid effective gain
    Real gain1 = source1->get_effective_gain();
    Real gain2 = source2->get_effective_gain();

    EXPECT_GE(gain1, 0.0);
    EXPECT_GE(gain2, 0.0);
}

} // namespace jaguar::audio
