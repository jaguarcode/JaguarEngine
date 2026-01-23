# XR API Reference

Extended Reality (VR/AR) integration for immersive simulation experiences.

## Overview

The XR module provides:
- **OpenXR Integration**: Cross-platform VR/AR headset support
- **Session Management**: Multi-platform XR runtime abstraction
- **Tracking**: Head, hand, eye, and controller tracking
- **Spatial Audio**: HRTF-based 3D positional audio
- **Input Handling**: Controller and hand gesture input

## Namespaces

```cpp
namespace jaguar::xr;      // XR session and tracking
namespace jaguar::audio;   // Spatial audio system
```

---

## XR Session Types

### XRResult

Result codes for XR operations.

```cpp
enum class XRResult : Int32 {
    Success = 0,
    NotInitialized = -1,
    RuntimeUnavailable = -2,
    SessionNotRunning = -3,
    SessionLost = -4,
    FrameDiscarded = -5,
    InvalidParameter = -6,
    DeviceNotConnected = -7,
    FeatureNotSupported = -8,
    TrackingLost = -9,
    CalibrationRequired = -10,
    PermissionDenied = -11,
    OutOfMemory = -12,
    InternalError = -99
};

// Helper functions
const char* xr_result_to_string(XRResult result);
bool xr_succeeded(XRResult result);
```

### XRSessionState

```cpp
enum class XRSessionState : UInt8 {
    Unknown = 0,
    Idle = 1,           // Session created, not running
    Ready = 2,          // Session ready to begin
    Synchronized = 3,   // Synchronized with runtime
    Visible = 4,        // Content visible but not focused
    Focused = 5,        // Fully focused and interactive
    Stopping = 6,       // Session stopping
    Exiting = 7,        // Session exiting
    LossPending = 8     // Session loss imminent
};
```

### XRPose

6-DOF pose representation.

```cpp
struct XRPose {
    Vec3 position{Vec3::Zero()};
    Quat orientation{Quat::Identity()};
    TrackingConfidence position_confidence{TrackingConfidence::None};
    TrackingConfidence orientation_confidence{TrackingConfidence::None};
    bool is_valid{false};

    bool has_tracking() const;
    bool has_full_tracking() const;
    Vec3 forward() const;
    Vec3 up() const;
    Vec3 right() const;
    Vec3 transform_point(const Vec3& local_point) const;
    Vec3 transform_direction(const Vec3& local_dir) const;

    static XRPose lerp(const XRPose& a, const XRPose& b, Real t);
    static XRPose Identity();
};
```

### XRTrackedPose

Complete tracked pose with velocity.

```cpp
struct XRTrackedPose {
    XRPose pose;
    XRVelocity velocity;
    Int64 timestamp_ns{0};
};
```

### XRHeadState

HMD tracking state.

```cpp
struct XRHeadState {
    XRTrackedPose head_pose;
    std::array<XRViewState, 2> views;  // Per-eye view states
    bool is_user_present{false};
    Real ipd{0.063f};                  // Interpupillary distance
};
```

### XRControllerState

Controller input state.

```cpp
struct XRControllerState {
    XRTrackedPose pose;
    XRTrackedPose aim_pose;
    XRTrackedPose grip_pose;

    std::bitset<13> buttons_pressed;
    std::bitset<13> buttons_touched;
    std::array<Real, 6> axes;

    bool is_connected{false};
    bool is_active{false};

    bool is_pressed(XRButton button) const;
    bool is_touched(XRButton button) const;
    Real get_axis(XRAxis axis) const;
    Real trigger() const;
    Real grip() const;
    Vec3 thumbstick() const;
};
```

### XRHandState

Hand tracking state.

```cpp
struct XRHandState {
    XRHand hand{XRHand::Left};
    std::array<XRHandJointData, 26> joints;
    TrackingConfidence confidence{TrackingConfidence::None};
    bool is_active{false};

    XRHandGesture detected_gesture{XRHandGesture::None};
    Real gesture_confidence{0.0f};
    Real pinch_strength{0.0f};
    Vec3 pinch_point{Vec3::Zero()};

    const XRHandJointData& get_joint(XRHandJoint joint) const;
    bool is_valid() const;
    Vec3 index_tip() const;
    Vec3 thumb_tip() const;
    Vec3 palm_position() const;
    Quat palm_orientation() const;
};
```

---

## Spatial Audio Types

### AudioResult

Result codes for audio operations.

```cpp
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

const char* audio_result_to_string(AudioResult result);
bool audio_succeeded(AudioResult result);
```

### AudioFormat

Audio format specification.

```cpp
struct AudioFormat {
    UInt32 sample_rate{48000};
    AudioSampleFormat sample_format{AudioSampleFormat::Float32};
    AudioChannelLayout channel_layout{AudioChannelLayout::Stereo};
    UInt8 channel_count{2};
    UInt32 buffer_size{1024};
};
```

### DistanceAttenuation

Distance-based gain attenuation.

```cpp
enum class AttenuationModel : UInt8 {
    None = 0,           // No attenuation
    Linear = 1,         // Linear falloff
    Inverse = 2,        // Inverse distance
    InverseSquare = 3,  // Inverse square (physically accurate)
    Logarithmic = 4,    // Logarithmic falloff
    Custom = 5          // User-defined curve
};

struct DistanceAttenuation {
    AttenuationModel model{AttenuationModel::InverseSquare};
    Real reference_distance{1.0};   // Distance at gain = 1.0
    Real max_distance{1000.0};      // Maximum audible distance
    Real rolloff_factor{1.0};       // Rolloff rate multiplier
    Real min_gain{0.0};
    Real max_gain{1.0};

    Real calculate_gain(Real distance) const noexcept;

    static DistanceAttenuation Indoor();
    static DistanceAttenuation Outdoor();
    static DistanceAttenuation OpenField();
};
```

### AudioCone

Directional audio cone.

```cpp
struct AudioCone {
    Real inner_angle{PI / 3.0};    // Full gain cone angle
    Real outer_angle{PI};          // Zero gain cone angle
    Real outer_gain{0.0};          // Gain outside outer cone

    bool is_valid() const noexcept;
    Real calculate_gain(Real angle) const noexcept;
};
```

### AudioSourceConfig

Complete audio source configuration.

```cpp
struct AudioSourceConfig {
    std::string name;
    EntityId entity_id{INVALID_ENTITY_ID};

    AudioSourceType source_type{AudioSourceType::Point};
    AudioPriority priority{AudioPriority::Normal};

    Vec3 position{Vec3::Zero()};
    Quat orientation{Quat::Identity()};
    Vec3 velocity{Vec3::Zero()};

    Real gain{1.0};
    Real pitch{1.0};
    bool muted{false};

    DistanceAttenuation attenuation;
    AudioCone cone;

    bool looping{false};
    bool auto_play{true};

    bool enable_doppler{true};
    bool enable_occlusion{true};
    bool enable_reverb{true};
    Real air_absorption{0.0};

    Real cull_distance{0.0};
    bool virtualize_when_culled{true};
};
```

### RoomAcousticsConfig

Room acoustics and reverb configuration.

```cpp
enum class RoomPreset : UInt8 {
    None = 0,
    SmallRoom = 1,
    MediumRoom = 2,
    LargeRoom = 3,
    Cathedral = 4,
    Cave = 5,
    Outdoor = 6,
    Underwater = 7,
    Aircraft = 8,
    Cockpit = 9,
    Custom = 255
};

struct ReverbParameters {
    Real decay_time{1.5};
    Real pre_delay{0.02};
    Real diffusion{0.85};
    Real density{0.85};
    Real hf_decay_ratio{0.8};
    Real lf_decay_ratio{1.0};
    Real wet_dry_mix{0.3};
    Real gain{1.0};
};

struct RoomAcousticsConfig {
    RoomPreset preset{RoomPreset::MediumRoom};
    RoomGeometry geometry;
    ReverbParameters reverb;
    UInt32 ray_count{1024};
    UInt32 bounce_count{8};
    bool enable_diffraction{true};

    static RoomAcousticsConfig FromPreset(RoomPreset preset);
};
```

### DopplerConfig

Doppler effect configuration.

```cpp
struct DopplerConfig {
    bool enabled{true};
    Real factor{1.0};
    Real speed_of_sound{343.0};   // m/s
    Real max_pitch_shift{4.0};
    Real smoothing_factor{0.1};

    Real calculate_pitch_shift(
        const Vec3& source_velocity,
        const Vec3& listener_velocity,
        const Vec3& direction
    ) const noexcept;
};
```

---

## Spatial Audio Interfaces

### ISpatialAudioSource

Interface for spatial audio sources.

```cpp
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
    virtual void set_velocity(const Vec3& velocity) = 0;

    // Volume
    virtual void set_gain(Real gain) = 0;
    virtual Real get_gain() const = 0;
    virtual void set_pitch(Real pitch) = 0;
    virtual void set_muted(bool muted) = 0;

    // Effects
    virtual void set_doppler_enabled(bool enabled) = 0;
    virtual void set_occlusion_enabled(bool enabled) = 0;
    virtual void set_reverb_enabled(bool enabled) = 0;

    // Queries
    virtual Real get_playback_time() const = 0;
    virtual Real get_effective_gain() const = 0;
    virtual OcclusionState get_occlusion_state() const = 0;
};
```

### ISpatialAudioListener

Interface for the audio listener.

```cpp
class ISpatialAudioListener {
public:
    virtual ~ISpatialAudioListener() = default;

    virtual void set_position(const Vec3& position) = 0;
    virtual Vec3 get_position() const = 0;
    virtual void set_orientation(const Quat& orientation) = 0;
    virtual void set_velocity(const Vec3& velocity) = 0;

    virtual void set_master_gain(Real gain) = 0;
    virtual Real get_master_gain() const = 0;

    // XR integration
    virtual void set_xr_tracking(const xr::XRTrackedPose& head_pose) = 0;
    virtual bool is_xr_tracking_active() const = 0;
};
```

### ISpatialAudioRenderer

Main spatial audio renderer interface.

```cpp
class ISpatialAudioRenderer {
public:
    virtual ~ISpatialAudioRenderer() = default;

    // Lifecycle
    virtual AudioResult initialize(
        const AudioFormat& format,
        const HRTFConfig& hrtf_config
    ) = 0;
    virtual void shutdown() = 0;
    virtual bool is_initialized() const = 0;

    // Source management
    virtual std::shared_ptr<ISpatialAudioSource> create_source(
        const AudioSourceConfig& config
    ) = 0;
    virtual AudioResult destroy_source(UInt32 source_id) = 0;
    virtual std::shared_ptr<ISpatialAudioSource> get_source(UInt32 source_id) = 0;
    virtual UInt32 get_active_source_count() const = 0;
    virtual UInt32 get_max_source_count() const = 0;

    // Listener
    virtual ISpatialAudioListener* get_listener() = 0;

    // Room acoustics
    virtual AudioResult set_room_acoustics(const RoomAcousticsConfig& config) = 0;
    virtual const RoomAcousticsConfig& get_room_acoustics() const = 0;

    // Configuration
    virtual AudioResult set_occlusion_config(const OcclusionConfig& config) = 0;
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

// Factory function
std::unique_ptr<ISpatialAudioRenderer> create_spatial_audio_renderer();
```

---

## Usage Examples

### Basic XR Session

```cpp
#include <jaguar/xr/xr_session.h>

using namespace jaguar::xr;

// Create session with mock runtime
auto runtime = create_mock_xr_runtime();
XRSession session(std::move(runtime));

// Initialize for VR
auto result = session.initialize(XRSessionConfig::VR_Default());
if (!xr_succeeded(result)) {
    std::cerr << xr_result_to_string(result) << std::endl;
    return;
}

// Main loop
while (session.is_running()) {
    // Begin frame
    XRFrameTiming timing;
    session.begin_frame(timing);

    // Get tracking data
    XRHeadState head = session.get_head_state();
    XRControllerState left = session.get_controller_state(XRHand::Left);
    XRControllerState right = session.get_controller_state(XRHand::Right);

    // Use head position for camera
    Vec3 camera_pos = head.head_pose.pose.position;
    Quat camera_rot = head.head_pose.pose.orientation;

    // Check controller input
    if (right.is_pressed(XRButton::A)) {
        // Handle button press
    }

    // End frame
    session.end_frame();
}

session.shutdown();
```

### Spatial Audio with XR

```cpp
#include <jaguar/xr/spatial_audio.h>
#include <jaguar/xr/xr_session.h>

using namespace jaguar::audio;
using namespace jaguar::xr;

// Create audio renderer
auto renderer = create_spatial_audio_renderer();

AudioFormat format;
format.sample_rate = 48000;
format.channel_layout = AudioChannelLayout::Binaural;

HRTFConfig hrtf;
hrtf.quality = HRTFQuality::High;
hrtf.max_sources = 64;

renderer->initialize(format, hrtf);

// Set room acoustics
auto room = RoomAcousticsConfig::FromPreset(RoomPreset::Cockpit);
renderer->set_room_acoustics(room);

// Create audio sources
AudioSourceConfig engine_config;
engine_config.name = "engine";
engine_config.position = {0.0, 0.0, -2.0};  // Behind player
engine_config.looping = true;
engine_config.enable_doppler = false;  // Engine moves with listener

auto engine_sound = renderer->create_source(engine_config);
engine_sound->play();

AudioSourceConfig missile_config;
missile_config.name = "missile";
missile_config.position = {100.0, 50.0, 200.0};
missile_config.velocity = {-50.0, 0.0, -100.0};  // Approaching
missile_config.enable_doppler = true;

auto missile_sound = renderer->create_source(missile_config);
missile_sound->play();

// Main loop with XR integration
while (running) {
    // Get XR head tracking
    XRHeadState head = xr_session.get_head_state();

    // Update listener from XR tracking
    renderer->get_listener()->set_xr_tracking(head.head_pose);

    // Update missile position
    Vec3 pos = missile_sound->get_position();
    pos += missile_config.velocity * delta_time;
    missile_sound->set_position(pos);

    // Process audio
    renderer->update(delta_time);

    // Render to audio buffer
    std::vector<float> audio_buffer(1024 * 2);
    renderer->render(audio_buffer.data(), 1024);

    // Send to audio device...
}

renderer->shutdown();
```

### Hand Tracking

```cpp
#include <jaguar/xr/xr_session.h>

using namespace jaguar::xr;

// Enable hand tracking
XRSessionConfig config = XRSessionConfig::VR_Default();
config.enable_hand_tracking = true;

XRSession session(create_mock_xr_runtime());
session.initialize(config);

while (session.is_running()) {
    session.begin_frame(timing);

    // Get hand tracking data
    XRHandState left_hand = session.get_hand_state(XRHand::Left);
    XRHandState right_hand = session.get_hand_state(XRHand::Right);

    if (right_hand.is_valid()) {
        // Check pinch gesture
        if (right_hand.pinch_strength > 0.8) {
            Vec3 pinch_pos = right_hand.pinch_point;
            // Handle pinch at position
        }

        // Get fingertip positions
        Vec3 index_tip = right_hand.index_tip();
        Vec3 thumb_tip = right_hand.thumb_tip();

        // Check detected gesture
        if (right_hand.detected_gesture == XRHandGesture::Point) {
            // Handle pointing gesture
        }
    }

    session.end_frame();
}
```

---

## Room Material Presets

```cpp
// Acoustic material properties
RoomMaterial concrete = RoomMaterial::Concrete();  // Low absorption
RoomMaterial carpet = RoomMaterial::Carpet();      // High HF absorption
RoomMaterial glass = RoomMaterial::Glass();        // Low scattering
RoomMaterial fabric = RoomMaterial::Fabric();      // High scattering
RoomMaterial wood = RoomMaterial::Wood();          // Balanced

// Occluder materials
OccluderMaterial wall = OccluderMaterial::SolidWall();
OccluderMaterial door = OccluderMaterial::Door();
OccluderMaterial window = OccluderMaterial::GlassWindow();
OccluderMaterial foliage = OccluderMaterial::Foliage();
```

---

## See Also

- [XR Session Header](../../../include/jaguar/xr/xr_session.h)
- [XR Types Header](../../../include/jaguar/xr/xr_types.h)
- [Spatial Audio Header](../../../include/jaguar/xr/spatial_audio.h)
- [XR Events Header](../../../include/jaguar/xr/xr_events.h)
