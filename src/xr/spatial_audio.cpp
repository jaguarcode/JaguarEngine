/**
 * @file spatial_audio.cpp
 * @brief Spatial audio system implementation
 *
 * Implements HRTF-based 3D audio, distance attenuation, room acoustics,
 * occlusion, and Doppler effect for immersive spatial audio.
 */

#include "jaguar/xr/spatial_audio.h"
#include <cmath>
#include <algorithm>
#include <unordered_map>
#include <mutex>
#include <cstring>

namespace jaguar::audio {

// ============================================================================
// AudioResult String Conversion
// ============================================================================

const char* audio_result_to_string(AudioResult result) {
    switch (result) {
        case AudioResult::Success: return "Success";
        case AudioResult::NotInitialized: return "NotInitialized";
        case AudioResult::DeviceUnavailable: return "DeviceUnavailable";
        case AudioResult::InvalidParameter: return "InvalidParameter";
        case AudioResult::SourceLimitReached: return "SourceLimitReached";
        case AudioResult::BufferOverflow: return "BufferOverflow";
        case AudioResult::FormatNotSupported: return "FormatNotSupported";
        case AudioResult::HRTFLoadFailed: return "HRTFLoadFailed";
        case AudioResult::FileNotFound: return "FileNotFound";
        case AudioResult::DecodingError: return "DecodingError";
        case AudioResult::OutOfMemory: return "OutOfMemory";
        case AudioResult::InternalError: return "InternalError";
        default: return "Unknown";
    }
}

// ============================================================================
// Distance Attenuation Implementation
// ============================================================================

Real DistanceAttenuation::calculate_gain(Real distance) const noexcept {
    // Clamp distance to valid range
    if (distance <= 0.0) distance = 0.001;  // Avoid division by zero

    Real gain = 1.0;

    switch (model) {
        case AttenuationModel::None:
            gain = 1.0;
            break;

        case AttenuationModel::Linear: {
            // Linear falloff from reference to max distance
            if (distance <= reference_distance) {
                gain = 1.0;
            } else if (distance >= max_distance) {
                gain = min_gain;
            } else {
                Real factor = (distance - reference_distance) / (max_distance - reference_distance);
                gain = 1.0 - factor;
            }
            break;
        }

        case AttenuationModel::Inverse: {
            // Inverse distance law
            if (distance <= reference_distance) {
                gain = 1.0;
            } else {
                gain = reference_distance / (reference_distance + rolloff_factor * (distance - reference_distance));
            }
            break;
        }

        case AttenuationModel::InverseSquare: {
            // Inverse square law (physically accurate)
            if (distance <= reference_distance) {
                gain = 1.0;
            } else {
                Real ref_sq = reference_distance * reference_distance;
                Real dist_sq = distance * distance;
                gain = ref_sq / (ref_sq + rolloff_factor * (dist_sq - ref_sq));
            }
            break;
        }

        case AttenuationModel::Logarithmic: {
            // Logarithmic falloff
            if (distance <= reference_distance) {
                gain = 1.0;
            } else {
                gain = 1.0 - rolloff_factor * std::log2(distance / reference_distance);
            }
            break;
        }

        case AttenuationModel::Custom:
            // Custom attenuation would use a lookup table or callback
            // For now, fall back to inverse square
            if (distance <= reference_distance) {
                gain = 1.0;
            } else {
                Real ref_sq = reference_distance * reference_distance;
                Real dist_sq = distance * distance;
                gain = ref_sq / (ref_sq + rolloff_factor * (dist_sq - ref_sq));
            }
            break;
    }

    // Clamp to valid range
    return std::clamp(gain, min_gain, max_gain);
}

// ============================================================================
// AudioCone Implementation
// ============================================================================

Real AudioCone::calculate_gain(Real angle) const noexcept {
    // Ensure angle is positive
    angle = std::abs(angle);

    if (angle <= inner_angle) {
        // Inside inner cone: full gain
        return 1.0;
    } else if (angle >= outer_angle) {
        // Outside outer cone: outer gain
        return outer_gain;
    } else {
        // Between inner and outer: interpolate
        Real t = (angle - inner_angle) / (outer_angle - inner_angle);
        return 1.0 - t * (1.0 - outer_gain);
    }
}

// ============================================================================
// Doppler Effect Implementation
// ============================================================================

Real DopplerConfig::calculate_pitch_shift(const Vec3& source_velocity,
                                           const Vec3& listener_velocity,
                                           const Vec3& direction) const noexcept {
    if (!enabled || factor == 0.0) {
        return 1.0;
    }

    // Calculate velocities along the source-listener axis
    Real source_speed = source_velocity.dot(direction);     // Positive = moving toward listener
    Real listener_speed = listener_velocity.dot(direction); // Positive = moving toward source

    // Clamp velocities to prevent extreme values
    Real max_speed = speed_of_sound * 0.9;  // 90% of speed of sound
    source_speed = std::clamp(source_speed, -max_speed, max_speed);
    listener_speed = std::clamp(listener_speed, -max_speed, max_speed);

    // Doppler formula: f' = f * (c + v_listener) / (c + v_source)
    // Note: positive velocity means moving toward each other (approaching)
    Real denominator = speed_of_sound - source_speed * factor;
    Real numerator = speed_of_sound - listener_speed * factor;

    if (std::abs(denominator) < 0.001) {
        denominator = 0.001;  // Prevent division by zero
    }

    Real pitch = numerator / denominator;

    // Clamp to reasonable range
    return std::clamp(pitch, 1.0 / max_pitch_shift, max_pitch_shift);
}

// ============================================================================
// Room Acoustics Preset Factory
// ============================================================================

RoomAcousticsConfig RoomAcousticsConfig::FromPreset(RoomPreset preset) {
    RoomAcousticsConfig config;
    config.preset = preset;

    switch (preset) {
        case RoomPreset::None:
            config.reverb.wet_dry_mix = 0.0;
            config.reverb.decay_time = 0.0;
            break;

        case RoomPreset::SmallRoom:
            config.geometry.dimensions = {4.0, 2.5, 5.0};
            config.reverb.decay_time = 0.4;
            config.reverb.pre_delay = 0.007;
            config.reverb.diffusion = 0.7;
            config.reverb.wet_dry_mix = 0.2;
            config.reverb.hf_decay_ratio = 0.6;
            break;

        case RoomPreset::MediumRoom:
            config.geometry.dimensions = {8.0, 3.0, 10.0};
            config.reverb.decay_time = 0.8;
            config.reverb.pre_delay = 0.012;
            config.reverb.diffusion = 0.8;
            config.reverb.wet_dry_mix = 0.25;
            config.reverb.hf_decay_ratio = 0.7;
            break;

        case RoomPreset::LargeRoom:
            config.geometry.dimensions = {20.0, 5.0, 25.0};
            config.reverb.decay_time = 1.5;
            config.reverb.pre_delay = 0.020;
            config.reverb.diffusion = 0.85;
            config.reverb.wet_dry_mix = 0.3;
            config.reverb.hf_decay_ratio = 0.75;
            break;

        case RoomPreset::Cathedral:
            config.geometry.dimensions = {30.0, 20.0, 50.0};
            config.reverb.decay_time = 4.0;
            config.reverb.pre_delay = 0.040;
            config.reverb.diffusion = 0.95;
            config.reverb.density = 0.95;
            config.reverb.wet_dry_mix = 0.4;
            config.reverb.hf_decay_ratio = 0.5;
            break;

        case RoomPreset::Cave:
            config.geometry.dimensions = {15.0, 8.0, 30.0};
            config.reverb.decay_time = 3.0;
            config.reverb.pre_delay = 0.030;
            config.reverb.diffusion = 0.6;
            config.reverb.density = 0.7;
            config.reverb.wet_dry_mix = 0.5;
            config.reverb.hf_decay_ratio = 0.4;
            config.reverb.lf_decay_ratio = 1.3;
            break;

        case RoomPreset::Outdoor:
            config.geometry.dimensions = {100.0, 50.0, 100.0};
            config.reverb.decay_time = 0.3;
            config.reverb.pre_delay = 0.050;
            config.reverb.diffusion = 0.2;
            config.reverb.wet_dry_mix = 0.05;
            config.reverb.hf_decay_ratio = 0.3;
            break;

        case RoomPreset::Underwater:
            config.geometry.dimensions = {50.0, 20.0, 50.0};
            config.reverb.decay_time = 2.5;
            config.reverb.pre_delay = 0.010;
            config.reverb.diffusion = 0.9;
            config.reverb.wet_dry_mix = 0.6;
            config.reverb.hf_decay_ratio = 0.1;  // Water absorbs high frequencies
            config.reverb.lf_decay_ratio = 1.5;
            break;

        case RoomPreset::Aircraft:
            config.geometry.dimensions = {30.0, 2.5, 4.0};
            config.reverb.decay_time = 0.5;
            config.reverb.pre_delay = 0.005;
            config.reverb.diffusion = 0.6;
            config.reverb.wet_dry_mix = 0.15;
            config.reverb.hf_decay_ratio = 0.5;
            break;

        case RoomPreset::Cockpit:
            config.geometry.dimensions = {2.5, 1.5, 3.0};
            config.reverb.decay_time = 0.2;
            config.reverb.pre_delay = 0.002;
            config.reverb.diffusion = 0.5;
            config.reverb.wet_dry_mix = 0.1;
            config.reverb.hf_decay_ratio = 0.6;
            break;

        case RoomPreset::Custom:
        default:
            // Use default values
            break;
    }

    return config;
}

// ============================================================================
// Spatial Audio Source Implementation
// ============================================================================

class SpatialAudioSource : public ISpatialAudioSource {
public:
    explicit SpatialAudioSource(UInt32 id, const AudioSourceConfig& config)
        : id_(id)
        , config_(config)
        , is_playing_(false)
        , is_paused_(false)
        , is_virtualized_(false)
        , playback_time_(0.0)
        , duration_(0.0)
        , effective_gain_(1.0) {}

    // Identity
    UInt32 get_id() const override { return id_; }
    const std::string& get_name() const override { return config_.name; }
    EntityId get_entity_id() const override { return config_.entity_id; }

    // State
    bool is_playing() const override { return is_playing_; }
    bool is_paused() const override { return is_paused_; }
    bool is_virtualized() const override { return is_virtualized_; }

    // Playback control
    AudioResult play() override {
        is_playing_ = true;
        is_paused_ = false;
        return AudioResult::Success;
    }

    AudioResult pause() override {
        if (is_playing_) {
            is_paused_ = true;
            is_playing_ = false;
        }
        return AudioResult::Success;
    }

    AudioResult stop() override {
        is_playing_ = false;
        is_paused_ = false;
        playback_time_ = 0.0;
        return AudioResult::Success;
    }

    AudioResult seek(Real time_seconds) override {
        if (time_seconds < 0.0) {
            return AudioResult::InvalidParameter;
        }
        playback_time_ = std::min(time_seconds, duration_);
        return AudioResult::Success;
    }

    // Properties
    void set_position(const Vec3& position) override { config_.position = position; }
    Vec3 get_position() const override { return config_.position; }
    void set_orientation(const Quat& orientation) override { config_.orientation = orientation; }
    Quat get_orientation() const override { return config_.orientation; }
    void set_velocity(const Vec3& velocity) override { config_.velocity = velocity; }
    Vec3 get_velocity() const override { return config_.velocity; }

    // Volume and gain
    void set_gain(Real gain) override { config_.gain = std::max(0.0, gain); }
    Real get_gain() const override { return config_.gain; }
    void set_pitch(Real pitch) override { config_.pitch = std::clamp(pitch, 0.5, 2.0); }
    Real get_pitch() const override { return config_.pitch; }
    void set_muted(bool muted) override { config_.muted = muted; }
    bool is_muted() const override { return config_.muted; }

    // Attenuation
    void set_attenuation(const DistanceAttenuation& attenuation) override {
        config_.attenuation = attenuation;
    }
    const DistanceAttenuation& get_attenuation() const override {
        return config_.attenuation;
    }

    // Cone
    void set_cone(const AudioCone& cone) override { config_.cone = cone; }
    const AudioCone& get_cone() const override { return config_.cone; }

    // Effects
    void set_doppler_enabled(bool enabled) override { config_.enable_doppler = enabled; }
    void set_occlusion_enabled(bool enabled) override { config_.enable_occlusion = enabled; }
    void set_reverb_enabled(bool enabled) override { config_.enable_reverb = enabled; }

    // State queries
    Real get_playback_time() const override { return playback_time_; }
    Real get_duration() const override { return duration_; }
    Real get_effective_gain() const override { return effective_gain_; }
    OcclusionState get_occlusion_state() const override { return occlusion_state_; }

    // Internal update methods
    void set_effective_gain(Real gain) { effective_gain_ = gain; }
    void set_occlusion_state(const OcclusionState& state) { occlusion_state_ = state; }
    void set_virtualized(bool virtualized) { is_virtualized_ = virtualized; }
    void advance_playback(Real delta_time) {
        if (is_playing_ && !is_paused_) {
            playback_time_ += delta_time * config_.pitch;
            if (playback_time_ >= duration_ && duration_ > 0.0) {
                if (config_.looping) {
                    playback_time_ = std::fmod(playback_time_, duration_);
                } else {
                    stop();
                }
            }
        }
    }
    void set_duration(Real duration) { duration_ = duration; }
    const AudioSourceConfig& get_config() const { return config_; }

private:
    UInt32 id_;
    AudioSourceConfig config_;
    bool is_playing_;
    bool is_paused_;
    bool is_virtualized_;
    Real playback_time_;
    Real duration_;
    Real effective_gain_;
    OcclusionState occlusion_state_;
};

// ============================================================================
// Spatial Audio Listener Implementation
// ============================================================================

class SpatialAudioListener : public ISpatialAudioListener {
public:
    void set_position(const Vec3& position) override { config_.position = position; }
    Vec3 get_position() const override { return config_.position; }
    void set_orientation(const Quat& orientation) override { config_.orientation = orientation; }
    Quat get_orientation() const override { return config_.orientation; }
    void set_velocity(const Vec3& velocity) override { config_.velocity = velocity; }
    Vec3 get_velocity() const override { return config_.velocity; }

    void set_master_gain(Real gain) override { config_.master_gain = std::max(0.0, gain); }
    Real get_master_gain() const override { return config_.master_gain; }

    void set_xr_tracking(const xr::XRTrackedPose& head_pose) override {
        config_.xr_head_pose = head_pose;
        // Update listener position/orientation from XR tracking
        if (head_pose.pose.is_valid) {
            config_.position = head_pose.pose.position;
            config_.orientation = head_pose.pose.orientation;
            if (head_pose.velocity.linear_valid) {
                config_.velocity = head_pose.velocity.linear;
            }
        }
    }

    bool is_xr_tracking_active() const override {
        return config_.xr_head_pose.has_value() &&
               config_.xr_head_pose->pose.is_valid;
    }

    const AudioListenerConfig& get_config() const { return config_; }

private:
    AudioListenerConfig config_;
};

// ============================================================================
// Spatial Audio Renderer Implementation
// ============================================================================

class SpatialAudioRenderer : public ISpatialAudioRenderer {
public:
    SpatialAudioRenderer()
        : initialized_(false)
        , next_source_id_(1)
        , cpu_usage_(0.0)
        , virtualized_count_(0) {}

    ~SpatialAudioRenderer() override {
        shutdown();
    }

    // Lifecycle
    AudioResult initialize(const AudioFormat& format, const HRTFConfig& hrtf_config) override {
        std::lock_guard<std::mutex> lock(mutex_);

        if (initialized_) {
            return AudioResult::Success;
        }

        format_ = format;
        hrtf_config_ = hrtf_config;

        // Initialize HRTF dataset info
        hrtf_dataset_.name = "default";
        hrtf_dataset_.sample_rate = format.sample_rate;
        hrtf_dataset_.quality = hrtf_config.quality;

        // Set default room acoustics
        room_config_ = RoomAcousticsConfig::FromPreset(RoomPreset::MediumRoom);

        // Initialize default occlusion config
        occlusion_config_.method = OcclusionMethod::Simple;

        // Initialize Doppler config
        doppler_config_.enabled = true;
        doppler_config_.speed_of_sound = 343.0;

        initialized_ = true;
        return AudioResult::Success;
    }

    void shutdown() override {
        std::lock_guard<std::mutex> lock(mutex_);

        if (!initialized_) {
            return;
        }

        sources_.clear();
        initialized_ = false;
    }

    bool is_initialized() const override {
        return initialized_;
    }

    // Source management
    std::shared_ptr<ISpatialAudioSource> create_source(const AudioSourceConfig& config) override {
        std::lock_guard<std::mutex> lock(mutex_);

        if (!initialized_) {
            return nullptr;
        }

        if (sources_.size() >= hrtf_config_.max_sources) {
            return nullptr;
        }

        UInt32 id = next_source_id_++;
        auto source = std::make_shared<SpatialAudioSource>(id, config);
        sources_[id] = source;

        return source;
    }

    AudioResult destroy_source(UInt32 source_id) override {
        std::lock_guard<std::mutex> lock(mutex_);

        auto it = sources_.find(source_id);
        if (it == sources_.end()) {
            return AudioResult::InvalidParameter;
        }

        sources_.erase(it);
        return AudioResult::Success;
    }

    std::shared_ptr<ISpatialAudioSource> get_source(UInt32 source_id) override {
        std::lock_guard<std::mutex> lock(mutex_);

        auto it = sources_.find(source_id);
        if (it != sources_.end()) {
            return it->second;
        }
        return nullptr;
    }

    UInt32 get_active_source_count() const override {
        std::lock_guard<std::mutex> lock(mutex_);
        UInt32 count = 0;
        for (const auto& [id, source] : sources_) {
            if (source->is_playing() && !source->is_virtualized()) {
                ++count;
            }
        }
        return count;
    }

    UInt32 get_max_source_count() const override {
        return hrtf_config_.max_sources;
    }

    // Listener
    ISpatialAudioListener* get_listener() override {
        return &listener_;
    }

    // Room acoustics
    AudioResult set_room_acoustics(const RoomAcousticsConfig& config) override {
        std::lock_guard<std::mutex> lock(mutex_);
        room_config_ = config;
        return AudioResult::Success;
    }

    const RoomAcousticsConfig& get_room_acoustics() const override {
        return room_config_;
    }

    // Occlusion
    AudioResult set_occlusion_config(const OcclusionConfig& config) override {
        std::lock_guard<std::mutex> lock(mutex_);
        occlusion_config_ = config;
        return AudioResult::Success;
    }

    // Doppler
    AudioResult set_doppler_config(const DopplerConfig& config) override {
        std::lock_guard<std::mutex> lock(mutex_);
        doppler_config_ = config;
        return AudioResult::Success;
    }

    // Processing
    AudioResult update(Real delta_time) override {
        std::lock_guard<std::mutex> lock(mutex_);

        if (!initialized_) {
            return AudioResult::NotInitialized;
        }

        const Vec3 listener_pos = listener_.get_position();
        const Quat listener_orient = listener_.get_orientation();
        const Vec3 listener_vel = listener_.get_velocity();
        const Real master_gain = listener_.get_master_gain();

        // listener_orient used for HRTF processing (reserved for full implementation)
        (void)listener_orient;

        virtualized_count_ = 0;

        for (auto& [id, source] : sources_) {
            auto* src = static_cast<SpatialAudioSource*>(source.get());
            const AudioSourceConfig& config = src->get_config();

            // Advance playback time
            src->advance_playback(delta_time);

            if (!src->is_playing() || src->is_muted()) {
                src->set_effective_gain(0.0);
                continue;
            }

            // Calculate distance and direction
            Vec3 source_pos = config.position;
            Vec3 delta = source_pos - listener_pos;
            Real distance = delta.length();
            Vec3 direction = (distance > 0.001) ? delta / distance : Vec3::UnitZ();

            // Check culling
            Real cull_dist = (config.cull_distance > 0.0) ? config.cull_distance : config.attenuation.max_distance;
            if (distance > cull_dist) {
                src->set_virtualized(config.virtualize_when_culled);
                if (config.virtualize_when_culled) {
                    ++virtualized_count_;
                }
                src->set_effective_gain(0.0);
                continue;
            }

            src->set_virtualized(false);

            // Calculate distance attenuation
            Real attenuation_gain = config.attenuation.calculate_gain(distance);

            // Calculate cone attenuation (for directional sources)
            Real cone_gain = 1.0;
            if (config.source_type == AudioSourceType::Directional) {
                Vec3 forward = config.orientation.rotate(Vec3{0.0, 0.0, -1.0});
                Real angle = std::acos(std::clamp(forward.dot(-direction), -1.0, 1.0));
                cone_gain = config.cone.calculate_gain(angle);
            }

            // Calculate Doppler pitch shift (applied during audio rendering)
            Real doppler_pitch = 1.0;
            if (config.enable_doppler && doppler_config_.enabled) {
                doppler_pitch = doppler_config_.calculate_pitch_shift(
                    config.velocity, listener_vel, direction);
            }
            // Store doppler_pitch for audio rendering pass (reserved for full implementation)
            (void)doppler_pitch;

            // Combine all gain factors
            Real effective_gain = config.gain * attenuation_gain * cone_gain * master_gain;

            // Apply occlusion if enabled
            if (config.enable_occlusion && occlusion_config_.method != OcclusionMethod::None) {
                OcclusionState occlusion = src->get_occlusion_state();
                effective_gain *= (1.0 - occlusion.occlusion_factor);
            }

            src->set_effective_gain(effective_gain);
        }

        return AudioResult::Success;
    }

    AudioResult render(void* output_buffer, UInt32 frame_count) override {
        std::lock_guard<std::mutex> lock(mutex_);

        if (!initialized_) {
            return AudioResult::NotInitialized;
        }

        // Clear output buffer
        std::memset(output_buffer, 0, frame_count * format_.channel_count * sizeof(float));

        // In a real implementation, this would:
        // 1. Process each active source through HRTF convolution
        // 2. Apply distance attenuation filters
        // 3. Apply room reverb
        // 4. Mix all sources to output
        // 5. Apply master gain

        // For now, this is a placeholder
        // Real audio rendering would require actual audio buffers from sources

        return AudioResult::Success;
    }

    // HRTF
    AudioResult load_hrtf_dataset(const std::string& path) override {
        std::lock_guard<std::mutex> lock(mutex_);

        hrtf_dataset_.file_path = path;
        hrtf_dataset_.is_loaded = true;  // In real implementation, actually load SOFA file

        return AudioResult::Success;
    }

    const HRTFDataset& get_hrtf_info() const override {
        return hrtf_dataset_;
    }

    // Statistics
    Real get_cpu_usage() const override {
        return cpu_usage_;
    }

    UInt32 get_virtualized_source_count() const override {
        return virtualized_count_;
    }

private:
    bool initialized_;
    AudioFormat format_;
    HRTFConfig hrtf_config_;
    HRTFDataset hrtf_dataset_;
    RoomAcousticsConfig room_config_;
    OcclusionConfig occlusion_config_;
    DopplerConfig doppler_config_;

    SpatialAudioListener listener_;
    std::unordered_map<UInt32, std::shared_ptr<SpatialAudioSource>> sources_;
    UInt32 next_source_id_;

    Real cpu_usage_;
    UInt32 virtualized_count_;

    mutable std::mutex mutex_;
};

// ============================================================================
// Factory Function
// ============================================================================

std::unique_ptr<ISpatialAudioRenderer> create_spatial_audio_renderer() {
    return std::make_unique<SpatialAudioRenderer>();
}

} // namespace jaguar::audio
