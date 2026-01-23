#pragma once
/**
 * @file spatial_audio.h
 * @brief Spatial audio types and interfaces for immersive 3D audio
 *
 * This file defines the spatial audio system for JaguarEngine, providing
 * HRTF-based 3D positional audio, distance attenuation, room acoustics,
 * occlusion, and Doppler effect support. Designed for XR integration.
 */

#include "jaguar/core/types.h"
#include "jaguar/xr/xr_types.h"
#include <array>
#include <vector>
#include <memory>
#include <functional>
#include <string>
#include <optional>

namespace jaguar::audio {

// ============================================================================
// Forward Declarations
// ============================================================================

class ISpatialAudioSource;
class ISpatialAudioListener;
class ISpatialAudioRenderer;
class IRoomAcoustics;

// ============================================================================
// Audio Result Types
// ============================================================================

/**
 * @brief Result codes for audio operations
 */
enum class AudioResult : Int32 {
    Success = 0,
    NotInitialized = -1,
    DeviceUnavailable = -2,
    InvalidParameter = -3,
    SourceLimitReached = -4,
    BufferOverflow = -5,
    FormatNotSupported = -6,
    HRTFLoadFailed = -7,
    FileNotFound = -8,
    DecodingError = -9,
    OutOfMemory = -10,
    InternalError = -99
};

/**
 * @brief Convert AudioResult to string for logging
 */
const char* audio_result_to_string(AudioResult result);

/**
 * @brief Check if AudioResult indicates success
 */
inline bool audio_succeeded(AudioResult result) {
    return result == AudioResult::Success;
}

// ============================================================================
// Audio Format Types
// ============================================================================

/**
 * @brief Audio sample format
 */
enum class AudioSampleFormat : UInt8 {
    Int16 = 0,          ///< 16-bit signed integer
    Int32 = 1,          ///< 32-bit signed integer
    Float32 = 2,        ///< 32-bit float (-1.0 to 1.0)
    Float64 = 3         ///< 64-bit float (-1.0 to 1.0)
};

/**
 * @brief Audio channel layout
 */
enum class AudioChannelLayout : UInt8 {
    Mono = 0,           ///< Single channel
    Stereo = 1,         ///< Left/Right stereo
    Binaural = 2,       ///< HRTF-processed binaural
    Surround51 = 3,     ///< 5.1 surround
    Surround71 = 4,     ///< 7.1 surround
    Ambisonic1 = 5,     ///< First-order ambisonics (4 channels)
    Ambisonic2 = 6,     ///< Second-order ambisonics (9 channels)
    Ambisonic3 = 7      ///< Third-order ambisonics (16 channels)
};

/**
 * @brief Audio format specification
 */
struct AudioFormat {
    UInt32 sample_rate{48000};
    AudioSampleFormat sample_format{AudioSampleFormat::Float32};
    AudioChannelLayout channel_layout{AudioChannelLayout::Stereo};
    UInt8 channel_count{2};
    UInt32 buffer_size{1024};           ///< Samples per buffer
};

// ============================================================================
// Distance Attenuation Models
// ============================================================================

/**
 * @brief Distance attenuation model type
 */
enum class AttenuationModel : UInt8 {
    None = 0,           ///< No distance attenuation
    Linear = 1,         ///< Linear falloff: 1 - clamp((d - ref) / (max - ref), 0, 1)
    Inverse = 2,        ///< Inverse distance: ref / (ref + rolloff * (d - ref))
    InverseSquare = 3,  ///< Inverse square: ref² / (ref² + rolloff * (d² - ref²))
    Logarithmic = 4,    ///< Logarithmic: 1 - rolloff * log2(d / ref)
    Custom = 5          ///< User-defined attenuation curve
};

/**
 * @brief Distance attenuation configuration
 */
struct DistanceAttenuation {
    AttenuationModel model{AttenuationModel::InverseSquare};
    Real reference_distance{1.0};       ///< Distance at which gain = 1.0 (meters)
    Real max_distance{1000.0};          ///< Maximum audible distance (meters)
    Real rolloff_factor{1.0};           ///< Rolloff rate multiplier
    Real min_gain{0.0};                 ///< Minimum gain (0 = silence at max_distance)
    Real max_gain{1.0};                 ///< Maximum gain cap

    /**
     * @brief Calculate gain at given distance
     * @param distance Distance from source to listener (meters)
     * @return Gain value [min_gain, max_gain]
     */
    Real calculate_gain(Real distance) const noexcept;

    /// Default indoor attenuation
    static DistanceAttenuation Indoor() {
        return {AttenuationModel::InverseSquare, 1.0, 50.0, 1.5, 0.0, 1.0};
    }

    /// Default outdoor attenuation
    static DistanceAttenuation Outdoor() {
        return {AttenuationModel::InverseSquare, 1.0, 500.0, 1.0, 0.0, 1.0};
    }

    /// Large open space attenuation
    static DistanceAttenuation OpenField() {
        return {AttenuationModel::InverseSquare, 2.0, 2000.0, 0.7, 0.0, 1.0};
    }
};

// ============================================================================
// Audio Source Configuration
// ============================================================================

/**
 * @brief Audio source type
 */
enum class AudioSourceType : UInt8 {
    Point = 0,          ///< Point source (omnidirectional)
    Directional = 1,    ///< Directional source (cone-based)
    Ambient = 2,        ///< Ambient/environmental (no spatialization)
    Line = 3,           ///< Line source (road, river, etc.)
    Area = 4            ///< Area source (crowd, forest, etc.)
};

/**
 * @brief Directional cone for directional sources
 */
struct AudioCone {
    Real inner_angle{constants::PI / 3.0};  ///< Full gain cone angle (radians)
    Real outer_angle{constants::PI};        ///< Zero gain cone angle (radians)
    Real outer_gain{0.0};                   ///< Gain outside outer cone [0, 1]

    /// Check if cone parameters are valid
    bool is_valid() const noexcept {
        return inner_angle >= 0.0 && inner_angle <= outer_angle &&
               outer_angle <= 2.0 * constants::PI &&
               outer_gain >= 0.0 && outer_gain <= 1.0;
    }

    /**
     * @brief Calculate cone gain for given angle
     * @param angle Angle from source forward direction (radians)
     * @return Gain value [outer_gain, 1.0]
     */
    Real calculate_gain(Real angle) const noexcept;
};

/**
 * @brief Audio source priority for voice management
 */
enum class AudioPriority : UInt8 {
    Critical = 0,       ///< Never culled (alarms, critical sounds)
    High = 1,           ///< High priority (weapons, important effects)
    Normal = 2,         ///< Normal priority (default)
    Low = 3,            ///< Low priority (ambient, background)
    Background = 4      ///< Lowest priority (can be culled first)
};

/**
 * @brief Complete audio source configuration
 */
struct AudioSourceConfig {
    // Identity
    std::string name;
    EntityId entity_id{INVALID_ENTITY_ID};

    // Source type and behavior
    AudioSourceType source_type{AudioSourceType::Point};
    AudioPriority priority{AudioPriority::Normal};

    // Spatial properties
    Vec3 position{Vec3::Zero()};
    Quat orientation{Quat::Identity()};
    Vec3 velocity{Vec3::Zero()};        ///< For Doppler effect

    // Volume and gain
    Real gain{1.0};                     ///< Master gain [0, infinity)
    Real pitch{1.0};                    ///< Pitch multiplier [0.5, 2.0]
    bool muted{false};

    // Spatialization
    DistanceAttenuation attenuation;
    AudioCone cone;                     ///< For directional sources

    // Playback
    bool looping{false};
    bool auto_play{true};
    Real start_time_offset{0.0};        ///< Start offset in seconds

    // Effects
    bool enable_doppler{true};
    bool enable_occlusion{true};
    bool enable_reverb{true};
    Real air_absorption{0.0};           ///< High-frequency absorption [0, 1]

    // LOD (Level of Detail)
    Real cull_distance{0.0};            ///< Distance to cull (0 = use max_distance)
    bool virtualize_when_culled{true};  ///< Continue playing silently when culled
};

// ============================================================================
// Audio Listener Configuration
// ============================================================================

/**
 * @brief Listener configuration
 */
struct AudioListenerConfig {
    // Spatial properties
    Vec3 position{Vec3::Zero()};
    Quat orientation{Quat::Identity()};
    Vec3 velocity{Vec3::Zero()};

    // Reference to XR head tracking (if available)
    std::optional<xr::XRTrackedPose> xr_head_pose;

    // Audio properties
    Real master_gain{1.0};
    Real meters_per_unit{1.0};          ///< World scale factor

    // HRTF customization
    Real head_radius{0.0875};           ///< Average head radius (meters)
    Real ear_spacing{0.16};             ///< Distance between ears (meters)
};

// ============================================================================
// HRTF (Head-Related Transfer Function) Types
// ============================================================================

/**
 * @brief HRTF dataset quality level
 */
enum class HRTFQuality : UInt8 {
    Low = 0,            ///< Basic HRTF (less CPU, less accurate)
    Medium = 1,         ///< Standard HRTF (balanced)
    High = 2,           ///< High-quality HRTF (more CPU, more accurate)
    Custom = 3          ///< Custom/personalized HRTF
};

/**
 * @brief HRTF dataset information
 */
struct HRTFDataset {
    std::string name;
    std::string file_path;
    HRTFQuality quality{HRTFQuality::Medium};
    UInt32 sample_rate{48000};
    UInt32 ir_length{256};              ///< Impulse response length (samples)
    UInt32 azimuth_count{72};           ///< Number of azimuth angles
    UInt32 elevation_count{18};         ///< Number of elevation angles
    bool is_loaded{false};
};

/**
 * @brief HRTF renderer configuration
 */
struct HRTFConfig {
    HRTFQuality quality{HRTFQuality::Medium};
    std::string dataset_path;           ///< Path to HRTF dataset (SOFA format)

    // Processing options
    bool enable_interpolation{true};    ///< Interpolate between HRTF measurements
    bool enable_nearfield{true};        ///< Near-field HRTF compensation
    UInt32 interpolation_order{2};      ///< HRTF interpolation order

    // Performance
    UInt32 max_sources{64};             ///< Maximum simultaneous spatialized sources
    bool use_gpu{false};                ///< Use GPU acceleration if available
};

// ============================================================================
// Room Acoustics Types
// ============================================================================

/**
 * @brief Room acoustic preset
 */
enum class RoomPreset : UInt8 {
    None = 0,           ///< No room effect
    SmallRoom = 1,      ///< Small room (bedroom, office)
    MediumRoom = 2,     ///< Medium room (living room)
    LargeRoom = 3,      ///< Large room (hall)
    Cathedral = 4,      ///< Large reverberant space
    Cave = 5,           ///< Cave/tunnel
    Outdoor = 6,        ///< Open outdoor
    Underwater = 7,     ///< Underwater acoustics
    Aircraft = 8,       ///< Aircraft cabin
    Cockpit = 9,        ///< Small enclosed cockpit
    Custom = 255
};

/**
 * @brief Room material acoustic properties
 */
struct RoomMaterial {
    std::string name;

    // Frequency-dependent absorption coefficients [0, 1]
    // Index: 0=125Hz, 1=250Hz, 2=500Hz, 3=1kHz, 4=2kHz, 5=4kHz
    std::array<Real, 6> absorption{0.1, 0.1, 0.1, 0.1, 0.1, 0.1};

    // Scattering coefficient [0, 1] (diffuse reflection)
    Real scattering{0.1};

    // Transmission coefficient [0, 1] (sound passing through)
    Real transmission{0.0};

    /// Concrete preset
    static RoomMaterial Concrete() {
        RoomMaterial m;
        m.name = "concrete";
        m.absorption = {0.01, 0.01, 0.02, 0.02, 0.02, 0.02};
        m.scattering = 0.1;
        return m;
    }

    /// Wood preset
    static RoomMaterial Wood() {
        RoomMaterial m;
        m.name = "wood";
        m.absorption = {0.15, 0.11, 0.10, 0.07, 0.06, 0.07};
        m.scattering = 0.1;
        return m;
    }

    /// Glass preset
    static RoomMaterial Glass() {
        RoomMaterial m;
        m.name = "glass";
        m.absorption = {0.35, 0.25, 0.18, 0.12, 0.07, 0.04};
        m.scattering = 0.05;
        return m;
    }

    /// Carpet preset
    static RoomMaterial Carpet() {
        RoomMaterial m;
        m.name = "carpet";
        m.absorption = {0.02, 0.06, 0.14, 0.37, 0.60, 0.65};
        m.scattering = 0.3;
        return m;
    }

    /// Fabric/curtain preset
    static RoomMaterial Fabric() {
        RoomMaterial m;
        m.name = "fabric";
        m.absorption = {0.03, 0.04, 0.11, 0.17, 0.24, 0.35};
        m.scattering = 0.5;
        return m;
    }
};

/**
 * @brief Room geometry for acoustic simulation
 */
struct RoomGeometry {
    // Simple box room
    Vec3 dimensions{10.0, 3.0, 10.0};   ///< Width, height, depth (meters)
    Vec3 center{Vec3::Zero()};

    // Surface materials (wall, floor, ceiling)
    RoomMaterial wall_material{RoomMaterial::Concrete()};
    RoomMaterial floor_material{RoomMaterial::Carpet()};
    RoomMaterial ceiling_material{RoomMaterial::Wood()};

    // Calculated properties
    Real volume() const noexcept {
        return dimensions.x * dimensions.y * dimensions.z;
    }

    Real surface_area() const noexcept {
        return 2.0 * (dimensions.x * dimensions.y +
                      dimensions.y * dimensions.z +
                      dimensions.z * dimensions.x);
    }
};

/**
 * @brief Reverb parameters (late reverberation)
 */
struct ReverbParameters {
    Real decay_time{1.5};               ///< RT60 decay time (seconds)
    Real pre_delay{0.02};               ///< Initial delay before reverb (seconds)
    Real diffusion{0.85};               ///< Reverb diffusion [0, 1]
    Real density{0.85};                 ///< Reverb density [0, 1]

    // Frequency-dependent decay
    Real hf_decay_ratio{0.8};           ///< High-frequency decay ratio [0, 1]
    Real lf_decay_ratio{1.0};           ///< Low-frequency decay ratio [0, 2]
    Real hf_reference{5000.0};          ///< High-frequency reference (Hz)
    Real lf_reference{250.0};           ///< Low-frequency reference (Hz)

    // Early reflections
    bool enable_early_reflections{true};
    Real early_reflection_delay{0.007}; ///< Early reflection delay (seconds)
    Real early_reflection_gain{0.7};    ///< Early reflection gain [0, 1]

    // Mix
    Real wet_dry_mix{0.3};              ///< Wet/dry mix [0=dry, 1=wet]
    Real gain{1.0};                     ///< Overall reverb gain
};

/**
 * @brief Complete room acoustics configuration
 */
struct RoomAcousticsConfig {
    RoomPreset preset{RoomPreset::MediumRoom};
    RoomGeometry geometry;
    ReverbParameters reverb;

    // Simulation quality
    UInt32 ray_count{1024};             ///< Rays for ray-tracing simulation
    UInt32 bounce_count{8};             ///< Maximum ray bounces
    bool enable_diffraction{true};      ///< Enable sound diffraction
    bool real_time_update{false};       ///< Update acoustics in real-time

    /// Create from preset
    static RoomAcousticsConfig FromPreset(RoomPreset preset);
};

// ============================================================================
// Audio Occlusion Types
// ============================================================================

/**
 * @brief Occlusion calculation method
 */
enum class OcclusionMethod : UInt8 {
    None = 0,           ///< No occlusion
    Simple = 1,         ///< Simple line-of-sight check
    Raycast = 2,        ///< Multiple raycasts for soft shadows
    Volumetric = 3      ///< Full volumetric occlusion
};

/**
 * @brief Occluder material properties
 */
struct OccluderMaterial {
    std::string name;
    Real transmission{0.0};             ///< Sound transmission [0, 1]
    Real low_frequency_transmission{0.3}; ///< LF passes through easier
    Real high_frequency_transmission{0.05}; ///< HF blocked more

    /// Solid wall
    static OccluderMaterial SolidWall() {
        return {"solid_wall", 0.01, 0.05, 0.001};
    }

    /// Thin wall/partition
    static OccluderMaterial ThinWall() {
        return {"thin_wall", 0.15, 0.3, 0.05};
    }

    /// Door (closed)
    static OccluderMaterial Door() {
        return {"door", 0.1, 0.2, 0.05};
    }

    /// Glass window
    static OccluderMaterial GlassWindow() {
        return {"glass_window", 0.2, 0.25, 0.15};
    }

    /// Foliage/vegetation
    static OccluderMaterial Foliage() {
        return {"foliage", 0.7, 0.8, 0.5};
    }
};

/**
 * @brief Occlusion state for a source
 */
struct OcclusionState {
    Real occlusion_factor{0.0};         ///< Overall occlusion [0=none, 1=full]
    Real direct_occlusion{0.0};         ///< Direct path occlusion
    Real reverb_occlusion{0.0};         ///< Reverb path occlusion

    // Frequency-dependent occlusion
    Real low_frequency_factor{0.0};
    Real mid_frequency_factor{0.0};
    Real high_frequency_factor{0.0};

    bool is_occluded() const noexcept { return occlusion_factor > 0.01; }
};

/**
 * @brief Occlusion system configuration
 */
struct OcclusionConfig {
    OcclusionMethod method{OcclusionMethod::Simple};
    UInt32 ray_count{8};                ///< Rays for raycast method
    Real update_interval{0.05};         ///< Update rate (seconds)
    Real smoothing_factor{0.1};         ///< Occlusion change smoothing

    // Filter parameters when occluded
    Real max_low_pass_frequency{500.0}; ///< LPF cutoff when fully occluded
    Real max_gain_reduction{-24.0};     ///< Max dB reduction when occluded
};

// ============================================================================
// Doppler Effect Types
// ============================================================================

/**
 * @brief Doppler effect configuration
 */
struct DopplerConfig {
    bool enabled{true};
    Real factor{1.0};                   ///< Doppler effect multiplier
    Real speed_of_sound{343.0};         ///< Speed of sound (m/s)
    Real max_pitch_shift{4.0};          ///< Maximum pitch multiplier
    Real smoothing_factor{0.1};         ///< Pitch change smoothing

    /**
     * @brief Calculate Doppler pitch shift
     * @param source_velocity Source velocity relative to listener (m/s, positive = approaching)
     * @param listener_velocity Listener velocity (m/s)
     * @param direction Unit vector from listener to source
     * @return Pitch multiplier
     */
    Real calculate_pitch_shift(const Vec3& source_velocity,
                                const Vec3& listener_velocity,
                                const Vec3& direction) const noexcept;
};

// ============================================================================
// Spatial Audio Interfaces
// ============================================================================

/**
 * @brief Interface for spatial audio sources
 */
class ISpatialAudioSource {
public:
    virtual ~ISpatialAudioSource() = default;

    // Identity
    virtual UInt32 get_id() const = 0;
    virtual const std::string& get_name() const = 0;
    virtual EntityId get_entity_id() const = 0;

    // State
    virtual bool is_playing() const = 0;
    virtual bool is_paused() const = 0;
    virtual bool is_virtualized() const = 0;

    // Playback control
    virtual AudioResult play() = 0;
    virtual AudioResult pause() = 0;
    virtual AudioResult stop() = 0;
    virtual AudioResult seek(Real time_seconds) = 0;

    // Properties
    virtual void set_position(const Vec3& position) = 0;
    virtual Vec3 get_position() const = 0;
    virtual void set_orientation(const Quat& orientation) = 0;
    virtual Quat get_orientation() const = 0;
    virtual void set_velocity(const Vec3& velocity) = 0;
    virtual Vec3 get_velocity() const = 0;

    // Volume and gain
    virtual void set_gain(Real gain) = 0;
    virtual Real get_gain() const = 0;
    virtual void set_pitch(Real pitch) = 0;
    virtual Real get_pitch() const = 0;
    virtual void set_muted(bool muted) = 0;
    virtual bool is_muted() const = 0;

    // Attenuation
    virtual void set_attenuation(const DistanceAttenuation& attenuation) = 0;
    virtual const DistanceAttenuation& get_attenuation() const = 0;

    // Cone (directional sources)
    virtual void set_cone(const AudioCone& cone) = 0;
    virtual const AudioCone& get_cone() const = 0;

    // Effects
    virtual void set_doppler_enabled(bool enabled) = 0;
    virtual void set_occlusion_enabled(bool enabled) = 0;
    virtual void set_reverb_enabled(bool enabled) = 0;

    // State queries
    virtual Real get_playback_time() const = 0;
    virtual Real get_duration() const = 0;
    virtual Real get_effective_gain() const = 0;  ///< After attenuation, occlusion, etc.
    virtual OcclusionState get_occlusion_state() const = 0;
};

/**
 * @brief Interface for the audio listener
 */
class ISpatialAudioListener {
public:
    virtual ~ISpatialAudioListener() = default;

    virtual void set_position(const Vec3& position) = 0;
    virtual Vec3 get_position() const = 0;
    virtual void set_orientation(const Quat& orientation) = 0;
    virtual Quat get_orientation() const = 0;
    virtual void set_velocity(const Vec3& velocity) = 0;
    virtual Vec3 get_velocity() const = 0;

    virtual void set_master_gain(Real gain) = 0;
    virtual Real get_master_gain() const = 0;

    // XR integration
    virtual void set_xr_tracking(const xr::XRTrackedPose& head_pose) = 0;
    virtual bool is_xr_tracking_active() const = 0;
};

/**
 * @brief Interface for the spatial audio renderer
 */
class ISpatialAudioRenderer {
public:
    virtual ~ISpatialAudioRenderer() = default;

    // Lifecycle
    virtual AudioResult initialize(const AudioFormat& format, const HRTFConfig& hrtf_config) = 0;
    virtual void shutdown() = 0;
    virtual bool is_initialized() const = 0;

    // Source management
    virtual std::shared_ptr<ISpatialAudioSource> create_source(const AudioSourceConfig& config) = 0;
    virtual AudioResult destroy_source(UInt32 source_id) = 0;
    virtual std::shared_ptr<ISpatialAudioSource> get_source(UInt32 source_id) = 0;
    virtual UInt32 get_active_source_count() const = 0;
    virtual UInt32 get_max_source_count() const = 0;

    // Listener
    virtual ISpatialAudioListener* get_listener() = 0;

    // Room acoustics
    virtual AudioResult set_room_acoustics(const RoomAcousticsConfig& config) = 0;
    virtual const RoomAcousticsConfig& get_room_acoustics() const = 0;

    // Occlusion
    virtual AudioResult set_occlusion_config(const OcclusionConfig& config) = 0;

    // Doppler
    virtual AudioResult set_doppler_config(const DopplerConfig& config) = 0;

    // Processing
    virtual AudioResult update(Real delta_time) = 0;
    virtual AudioResult render(void* output_buffer, UInt32 frame_count) = 0;

    // HRTF
    virtual AudioResult load_hrtf_dataset(const std::string& path) = 0;
    virtual const HRTFDataset& get_hrtf_info() const = 0;

    // Statistics
    virtual Real get_cpu_usage() const = 0;
    virtual UInt32 get_virtualized_source_count() const = 0;
};

// ============================================================================
// Factory Function
// ============================================================================

/**
 * @brief Create spatial audio renderer instance
 */
std::unique_ptr<ISpatialAudioRenderer> create_spatial_audio_renderer();

// ============================================================================
// Audio Callbacks
// ============================================================================

/**
 * @brief Callback when source finishes playing
 */
using AudioSourceCallback = std::function<void(UInt32 source_id)>;

/**
 * @brief Callback for occlusion queries (integrate with physics)
 */
using OcclusionQueryCallback = std::function<OcclusionState(
    const Vec3& source_pos,
    const Vec3& listener_pos
)>;

} // namespace jaguar::audio
