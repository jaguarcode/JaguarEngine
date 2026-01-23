# XR API Reference

Extended Reality (VR/AR) integration for immersive simulation experiences.

## Overview

The XR module provides:
- **OpenXR Integration**: Cross-platform VR/AR headset support
- **Session Management**: Multi-platform XR runtime abstraction
- **Tracking**: Head, hand, eye, and controller tracking
- **Spatial Audio**: HRTF-based 3D positional audio
- **Haptic Feedback**: Controller vibration, vest, motion platform, and G-force simulation
- **Input Handling**: Controller and hand gesture input
- **Training Modules**: Professional simulation training systems

## Namespaces

```cpp
namespace jaguar::xr;        // XR session and tracking
namespace jaguar::audio;     // Spatial audio system
namespace jaguar::haptics;   // Haptic feedback system
namespace jaguar::training;  // Training modules
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

## Haptic Feedback Types

### HapticResult

Result codes for haptic operations.

```cpp
enum class HapticResult : Int32 {
    Success = 0,
    NotInitialized = -1,
    DeviceNotConnected = -2,
    DeviceNotSupported = -3,
    InvalidParameter = -4,
    EffectNotSupported = -5,
    EffectQueueFull = -6,
    SafetyLimitExceeded = -9,
    InternalError = -99
};

const char* haptic_result_to_string(HapticResult result);
bool haptic_succeeded(HapticResult result);
```

### HapticDeviceType

Types of haptic devices.

```cpp
enum class HapticDeviceType : UInt8 {
    Unknown = 0,
    Controller = 1,       // VR/XR controller
    Vest = 2,             // Haptic vest
    Glove = 3,            // Haptic glove
    MotionPlatform = 4,   // Motion platform
    SteeringWheel = 5,    // Force feedback wheel
    FlightControls = 6,   // Force feedback flight stick
    Exoskeleton = 7       // Full body haptic
};
```

### HapticWaveform

Basic haptic effect waveform types.

```cpp
enum class HapticWaveform : UInt8 {
    None = 0,
    Constant = 1,   // Constant amplitude
    Sine = 2,       // Sinusoidal oscillation
    Square = 3,     // Square wave
    Triangle = 4,   // Triangle wave
    Sawtooth = 5,   // Sawtooth wave
    Noise = 6,      // Random noise
    Custom = 7      // Custom waveform
};
```

### HapticEnvelope

ADSR-style envelope for haptic effects.

```cpp
struct HapticEnvelope {
    Real attack_time{0.0};
    Real attack_level{1.0};
    Real decay_time{0.0};
    Real sustain_level{1.0};
    Real release_time{0.0};

    static HapticEnvelope Instant();
    static HapticEnvelope FadeIn(Real attack_seconds);
    static HapticEnvelope FadeOut(Real release_seconds);
    static HapticEnvelope ADSR(Real attack, Real decay, Real sustain, Real release);
};
```

### HapticEffect

Core haptic effect definition.

```cpp
struct HapticEffect {
    HapticWaveform waveform{HapticWaveform::Constant};
    Real frequency{160.0};      // Hz
    Real amplitude{0.5};        // 0-1
    Real duration{0.1};         // seconds (0 = infinite)
    HapticEnvelope envelope;
    Int32 repeat_count{1};
    Real repeat_delay{0.0};

    // Factory methods
    static HapticEffect Vibration(Real amplitude, Real duration, Real frequency = 160.0);
    static HapticEffect Pulse(Real amplitude, Real on_time, Real off_time, Int32 pulses);
    static HapticEffect Rumble(Real intensity, Real duration);
    static HapticEffect Impact(Real intensity);
};
```

### VestZone

Haptic vest zone identifiers.

```cpp
enum class VestZone : UInt8 {
    ChestLeft = 0, ChestCenter = 1, ChestRight = 2,
    AbdomenLeft = 3, AbdomenCenter = 4, AbdomenRight = 5,
    UpperBackLeft = 6, UpperBackCenter = 7, UpperBackRight = 8,
    LowerBackLeft = 9, LowerBackCenter = 10, LowerBackRight = 11,
    LeftSide = 12, RightSide = 13,
    LeftShoulder = 14, RightShoulder = 15,
    ZoneCount = 16
};
```

### MotionDOF

Motion platform degrees of freedom.

```cpp
enum class MotionDOF : UInt8 {
    Surge = 0,    // Forward/backward
    Sway = 1,     // Left/right
    Heave = 2,    // Up/down
    Roll = 3,     // Rotation around longitudinal
    Pitch = 4,    // Rotation around lateral
    Yaw = 5,      // Rotation around vertical
    DOFCount = 6
};
```

### MotionCueingParams

Motion cueing filter parameters.

```cpp
struct MotionCueingParams {
    WashoutAlgorithm algorithm;
    std::array<Real, 6> hp_cutoff_freq;    // High-pass cutoff
    std::array<Real, 6> hp_damping;
    std::array<Real, 3> lp_cutoff_freq;    // Low-pass for tilt
    std::array<Real, 6> scaling;
    Real tilt_coordination_gain{0.6};
    Real max_tilt_rate{0.5};

    // Factory methods
    static MotionCueingParams AircraftDefault();
    static MotionCueingParams GroundVehicleDefault();
    static MotionCueingParams ShipDefault();
};
```

### GForceConfig

G-force feedback configuration.

```cpp
struct GForceConfig {
    Real longitudinal_scale{1.0};
    Real lateral_scale{1.0};
    Real vertical_scale{1.0};
    Real max_sustained_g{3.0};
    Real max_onset_rate{6.0};
    bool use_tilt_coordination{true};
    bool simulate_gsuit{false};
    Real gsuit_threshold{2.0};

    // Presets
    static GForceConfig FighterJet();
    static GForceConfig CivilAircraft();
    static GForceConfig GroundVehicle();
};
```

---

## Haptic Interfaces

### IHapticRenderer

Main haptic renderer interface.

```cpp
class IHapticRenderer {
public:
    virtual HapticResult initialize(const HapticRendererConfig& config) = 0;
    virtual HapticResult shutdown() = 0;
    virtual HapticResult update(Real delta_time) = 0;

    // Devices
    virtual std::shared_ptr<IHapticController> get_controller() const = 0;
    virtual std::shared_ptr<IHapticVest> get_vest() const = 0;
    virtual std::shared_ptr<IMotionPlatform> get_motion_platform() const = 0;

    // Controller shortcuts
    virtual HapticEffectId play_controller_effect(xr::XRHand hand, const HapticEffect& effect) = 0;
    virtual HapticResult play_controller_vibration(xr::XRHand hand, Real low_freq, Real high_freq, Real duration) = 0;

    // Vest shortcuts
    virtual HapticEffectId play_vest_pattern(const VestHapticPattern& pattern) = 0;
    virtual HapticResult play_vest_impact(const Vec3& direction, Real intensity) = 0;

    // Motion platform
    virtual HapticResult apply_motion_cueing(const Vec3& linear_accel, const Vec3& angular_vel, const Vec3& angular_accel) = 0;

    // Engine vibration
    virtual HapticResult set_engine_vibration(const EngineVibrationParams& params) = 0;

    // G-force
    virtual HapticResult apply_g_force(const GForceState& g_state) = 0;
    virtual GForceState calculate_g_force(const Vec3& body_accel, const Vec3& gravity_body) const = 0;

    // Emergency
    virtual HapticResult emergency_stop_all() = 0;
};

// Factory functions
std::unique_ptr<IHapticRenderer> create_haptic_renderer();
std::unique_ptr<IHapticRenderer> create_haptic_renderer(const HapticRendererConfig& config);
std::unique_ptr<IHapticRenderer> create_mock_haptic_renderer();
```

---

## Haptic Presets

### Aircraft Presets

```cpp
namespace presets::aircraft {
    HapticEffect StallBuffet(Real intensity);
    HapticEffect GearTransition(bool extending);
    HapticEffect FlapMovement(Real deflection);
    HapticEffect SpeedBrake(Real deployment);
    HapticEffect Touchdown(Real vertical_speed);
    HapticEffect EngineStart(Real progress);
    HapticEffect AfterburnerIgnition();
    HapticEffect WeaponsRelease();
    HapticEffect RefuelingContact();
    HapticEffect OverspeedWarning();
}
```

### Vehicle Presets

```cpp
namespace presets::vehicle {
    HapticEffect RoadTexture(Real roughness);
    HapticEffect BrakeApply(Real pressure);
    HapticEffect ABSPulse();
    HapticEffect GearShift();
    HapticEffect Collision(Real severity, const Vec3& direction);
    HapticEffect TrackRumble(Real speed);
    HapticEffect FiringRecoil(Real caliber);
}
```

### Naval Presets

```cpp
namespace presets::naval {
    HapticEffect WaveMotion(Real sea_state);
    HapticEffect HullVibration(Real speed);
    HapticEffect AnchorDrop();
    HapticEffect NavalGunFiring(Real caliber);
    HapticEffect TorpedoLaunch();
    HapticEffect DepthCharge(Real distance);
}
```

### Helicopter Presets

```cpp
namespace presets::helicopter {
    HapticEffect RotorVibration(Real rpm);
    HapticEffect GroundResonance();
    HapticEffect VortexRingState();
    HapticEffect Autorotation();
    HapticEffect HardLanding(Real vertical_speed);
}
```

---

## Haptic Usage Example

```cpp
using namespace jaguar::haptics;

// Create and initialize renderer
auto haptics = create_haptic_renderer();
HapticRendererConfig config;
config.auto_connect_devices = true;
config.g_force_config = GForceConfig::FighterJet();
config.motion_cueing_params = MotionCueingParams::AircraftDefault();
haptics->initialize(config);

// Main loop
while (running) {
    // Apply flight dynamics to motion platform
    Vec3 accel = aircraft.get_body_acceleration();
    Vec3 ang_vel = aircraft.get_angular_velocity();
    Vec3 ang_accel = aircraft.get_angular_acceleration();
    haptics->apply_motion_cueing(accel, ang_vel, ang_accel);

    // Calculate and apply G-force
    Vec3 gravity_body = aircraft.get_gravity_body_frame();
    auto g_state = haptics->calculate_g_force(accel, gravity_body);
    haptics->apply_g_force(g_state);

    // Engine vibration
    EngineVibrationParams engine;
    engine.source = EngineVibrationSource::Turbine;
    engine.rpm = aircraft.get_n1_percent() * 150.0;
    engine.throttle = aircraft.get_throttle();
    haptics->set_engine_vibration(engine);

    // Stall buffet
    if (aircraft.is_approaching_stall()) {
        auto buffet = presets::aircraft::StallBuffet(aircraft.get_stall_intensity());
        haptics->play_controller_effect(xr::XRHand::Left, buffet);
        haptics->play_controller_effect(xr::XRHand::Right, buffet);
    }

    // Touchdown
    if (aircraft.just_touched_down()) {
        auto touchdown = presets::aircraft::Touchdown(aircraft.get_vertical_speed());
        haptics->play_controller_effect(xr::XRHand::Left, touchdown);
        haptics->play_vest_impact(Vec3{0, -1, 0}, 0.8);
    }

    haptics->update(delta_time);
}

haptics->shutdown();
```

---

## Training Module Types

### TrainingResult

Result codes for training operations.

```cpp
enum class TrainingResult : Int32 {
    Success = 0,
    NotInitialized = -1,
    SessionNotActive = -2,
    ScenarioNotLoaded = -3,
    InvalidParameter = -4,
    ModuleNotAvailable = -5,
    InstrumentNotFound = -6,
    ObjectiveNotFound = -7,
    RecordingNotStarted = -8,
    PlaybackError = -9,
    ScriptError = -10,
    ResourceNotFound = -11,
    PermissionDenied = -12,
    OutOfMemory = -13,
    InternalError = -99
};

const char* training_result_to_string(TrainingResult result);
bool training_succeeded(TrainingResult result);
```

### TrainingDomain

Training domain categories.

```cpp
enum class TrainingDomain : UInt8 {
    Aviation = 0,           // Fixed-wing aircraft
    Rotorcraft = 1,         // Helicopters
    Naval = 2,              // Ships and submarines
    GroundVehicle = 3,      // Tanks, APCs, trucks
    AirTrafficControl = 4,  // ATC operations
    Maritime = 5,           // Port operations
    Emergency = 6,          // Emergency response
    Custom = 255
};
```

### DifficultyLevel

```cpp
enum class DifficultyLevel : UInt8 {
    Tutorial = 0,
    Beginner = 1,
    Intermediate = 2,
    Advanced = 3,
    Expert = 4,
    Instructor = 5
};
```

### InstrumentType

Cockpit instrument categories (60+ types).

```cpp
enum class InstrumentType : UInt8 {
    // Primary Flight Display
    Airspeed = 0, Altitude = 1, AttitudeIndicator = 2,
    HeadingIndicator = 3, VerticalSpeed = 4, TurnCoordinator = 5,

    // Navigation
    HSI = 10, RMI = 11, DME = 12, ADF = 13, GPS = 14, FMS = 15,

    // Engine
    Tachometer = 20, Manifold = 21, FuelFlow = 22, FuelQuantity = 23,
    OilPressure = 24, OilTemperature = 25, EGT = 26, CHT = 27, N1 = 28, N2 = 29,

    // Systems
    Electrical = 40, Hydraulic = 41, Warning = 45, Annunciator = 46,

    // Radio
    ComRadio = 50, NavRadio = 51, Transponder = 52, TCAS = 53, GPWS = 54,

    // Autopilot & Military
    FlightDirector = 60, Autopilot = 61, WeaponsPanel = 70, HUD = 72, MFD = 73,

    Custom = 255
};
```

### ObjectiveType

Training objective types.

```cpp
enum class ObjectiveType : UInt8 {
    // Flight objectives
    Takeoff = 0, Landing = 1, GoAround = 2, Approach = 3,
    Holding = 4, Navigation = 5, Emergency = 6,

    // Maneuvers
    SteepTurn = 10, Stall = 11, SlowFlight = 12, Spin = 13, AcrobaticManeuver = 14,

    // Procedures
    Checklist = 20, Communication = 21, SystemsManagement = 22,
    DecisionMaking = 23, CrewCoordination = 24,

    // Navigation
    VORNavigation = 30, ILSApproach = 31, VFRNavigation = 32, GPSNavigation = 33,

    Custom = 255
};
```

### TrainingEventType

Event types for after-action review recording.

```cpp
enum class TrainingEventType : UInt8 {
    // Session
    SessionStart = 0, SessionEnd = 1, Checkpoint = 2, Bookmark = 3,

    // Performance
    ObjectiveStarted = 10, ObjectiveCompleted = 11, ObjectiveFailed = 12,
    ErrorMade = 13, ExcellentPerformance = 14,

    // Aircraft state
    StateChange = 20, Stall = 21, Overspeed = 22, Exceedance = 23,
    Collision = 24, Crash = 25,

    // Systems & Communication
    SystemFailure = 30, RadioTransmission = 40, Instruction = 41,

    // Custom
    InstructorNote = 60, Custom = 255
};
```

### Grade

Performance grading levels.

```cpp
enum class Grade : UInt8 {
    Excellent = 0,      // 90-100%
    Good = 1,           // 80-89%
    Satisfactory = 2,   // 70-79%
    Marginal = 3,       // 60-69%
    Unsatisfactory = 4, // Below 60%
    Incomplete = 5
};
```

---

## Training Data Structures

### InstrumentState

```cpp
struct InstrumentState {
    InstrumentType type;
    std::string name;
    Real value;
    Real min_value;
    Real max_value;
    bool is_active;
    bool has_warning;
    bool has_failure;
    std::string units;
};
```

### ControlState

```cpp
struct ControlState {
    ControlType type;
    std::string name;
    Real position;      // -1 to 1 for axes, 0 or 1 for switches
    Real force;         // Force feedback (if supported)
    bool is_switch;
    bool is_momentary;
    bool is_active;
};
```

### AircraftTrack

Air traffic control radar track.

```cpp
struct AircraftTrack {
    std::string callsign;
    std::string aircraft_type;
    Vec3 position;
    Vec3 velocity;
    Real altitude;          // feet
    Real heading;           // degrees
    Real ground_speed;      // knots
    Real vertical_rate;     // feet per minute
    std::string squawk;
    bool is_departing;
    bool is_arriving;
    bool is_vfr;
    bool has_emergency;
};
```

### TrainingObjective

```cpp
struct TrainingObjective {
    UInt32 id;
    ObjectiveType type;
    std::string name;
    std::string description;
    ObjectiveStatus status;
    Real score;             // 0-100
    Real weight;            // Weight in overall score
    Real time_limit;        // seconds (0 = no limit)
    Real elapsed_time;
    std::vector<std::string> criteria;
    std::vector<std::string> feedback;
    bool is_mandatory;
    bool is_graded;
};
```

### StateSnapshot

State snapshot for replay.

```cpp
struct StateSnapshot {
    Real timestamp;
    Vec3 position;
    Quat orientation;
    Vec3 velocity;
    Vec3 angular_velocity;
    Real airspeed, altitude, heading;
    Real pitch, roll;
    Real vertical_speed;
    Real elevator, aileron, rudder, throttle, flaps;
    bool gear_down;
    std::array<Real, 4> engine_rpm;
    std::vector<InstrumentState> instruments;
};
```

### ScenarioDefinition

Training scenario definition.

```cpp
struct ScenarioDefinition {
    std::string id;
    std::string name;
    std::string description;
    TrainingDomain domain;
    DifficultyLevel difficulty;

    // Initial conditions
    Vec3 start_position;
    Quat start_orientation;
    Real start_altitude, start_airspeed, start_heading;
    WeatherPreset weather;
    TimeOfDay time;

    // Content
    std::vector<TrainingObjective> objectives;
    std::vector<ScriptRule> rules;
    std::vector<std::string> checkpoints;

    // Metadata
    Real estimated_duration;
    std::vector<std::string> prerequisites;
    std::string author;
};
```

### GradeReport

Session grade report.

```cpp
struct GradeReport {
    Grade overall_grade;
    Real overall_score;
    Real total_time;
    std::vector<ScoringCriteria> criteria;
    std::vector<TrainingObjective> objectives;
    std::vector<TrainingEvent> significant_events;
    std::string instructor_comments;
    std::vector<std::string> strengths;
    std::vector<std::string> areas_for_improvement;
    std::vector<std::string> recommendations;
    bool is_passed;
};
```

---

## Training Interfaces

### ICockpitModule

Cockpit simulator module interface.

```cpp
class ICockpitModule : public ITrainingModule {
public:
    // Instruments
    virtual std::vector<InstrumentState> get_instruments() const = 0;
    virtual std::optional<InstrumentState> get_instrument(InstrumentType type) const = 0;
    virtual TrainingResult set_instrument_value(InstrumentType type, Real value) = 0;
    virtual TrainingResult inject_instrument_failure(InstrumentType type) = 0;
    virtual TrainingResult restore_instrument(InstrumentType type) = 0;

    // Controls
    virtual std::vector<ControlState> get_controls() const = 0;
    virtual TrainingResult set_control_position(ControlType type, Real position) = 0;
    virtual TrainingResult set_control_force(ControlType type, Real force) = 0;

    // Flight state
    virtual Real get_airspeed() const = 0;
    virtual Real get_altitude() const = 0;
    virtual Real get_heading() const = 0;
    virtual Real get_vertical_speed() const = 0;
    virtual Vec3 get_attitude() const = 0;

    // Procedures
    virtual TrainingResult start_checklist(const std::string& checklist_id) = 0;
    virtual TrainingResult complete_checklist_item(UInt32 item_index) = 0;

    // Integration
    virtual void bind_xr_session(std::shared_ptr<xr::IXRRuntime> xr_runtime) = 0;
    virtual void bind_haptics(std::shared_ptr<haptics::IHapticRenderer> haptics) = 0;
    virtual void bind_audio(std::shared_ptr<audio::ISpatialAudioRenderer> audio) = 0;
};
```

### IBridgeModule

Ship bridge simulator module interface.

```cpp
class IBridgeModule : public ITrainingModule {
public:
    // Stations
    virtual std::vector<BridgeStation> get_available_stations() const = 0;
    virtual TrainingResult select_station(BridgeStation station) = 0;
    virtual BridgeStation get_current_station() const = 0;

    // Navigation
    virtual Real get_heading() const = 0;
    virtual Real get_speed() const = 0;  // knots
    virtual Real get_depth() const = 0;  // meters
    virtual Vec3 get_position() const = 0;
    virtual std::vector<AircraftTrack> get_radar_contacts() const = 0;

    // Controls
    virtual TrainingResult set_helm(Real angle) = 0;  // -35 to +35 degrees
    virtual TrainingResult set_throttle(Real power) = 0;  // -1 to 1
    virtual TrainingResult set_thruster(ShipControlType thruster, Real power) = 0;

    // Equipment
    virtual TrainingResult activate_radar() = 0;
    virtual TrainingResult deactivate_radar() = 0;
};
```

### IControlTowerModule

Air traffic control tower module interface.

```cpp
class IControlTowerModule : public ITrainingModule {
public:
    // Position
    virtual ATCPosition get_position() const = 0;
    virtual TrainingResult set_position(ATCPosition position) = 0;

    // Traffic
    virtual std::vector<AircraftTrack> get_traffic() const = 0;
    virtual std::optional<AircraftTrack> get_aircraft(const std::string& callsign) const = 0;
    virtual UInt32 get_traffic_count() const = 0;

    // Runways
    virtual std::vector<RunwayStatus> get_runways() const = 0;
    virtual TrainingResult set_active_runway(const std::string& designator) = 0;
    virtual TrainingResult close_runway(const std::string& designator) = 0;

    // Communications
    virtual TrainingResult transmit(const std::string& message) = 0;
    virtual std::vector<std::string> get_pending_readbacks() const = 0;
    virtual TrainingResult verify_readback(const std::string& callsign, bool correct) = 0;

    // Instructions
    virtual TrainingResult issue_clearance(const std::string& callsign, const std::string& clearance) = 0;
    virtual TrainingResult issue_hold(const std::string& callsign, const std::string& fix) = 0;
    virtual TrainingResult issue_approach(const std::string& callsign, const std::string& approach) = 0;

    // Sequencing
    virtual std::vector<std::string> get_departure_sequence() const = 0;
    virtual std::vector<std::string> get_arrival_sequence() const = 0;
    virtual TrainingResult resequence(const std::string& callsign, UInt32 new_position) = 0;
};
```

### IAfterActionReview

After-action review interface for recording and analyzing training sessions.

```cpp
class IAfterActionReview {
public:
    // Recording
    virtual TrainingResult start_recording() = 0;
    virtual TrainingResult stop_recording() = 0;
    virtual TrainingResult add_bookmark(const std::string& name) = 0;
    virtual TrainingResult add_instructor_note(const std::string& note) = 0;
    virtual bool is_recording() const = 0;

    // Events
    virtual TrainingResult record_event(const TrainingEvent& event) = 0;
    virtual std::vector<TrainingEvent> get_events() const = 0;
    virtual std::vector<TrainingEvent> get_events_in_range(Real start, Real end) const = 0;

    // State recording
    virtual TrainingResult record_state(const StateSnapshot& snapshot) = 0;
    virtual std::optional<StateSnapshot> get_state_at_time(Real time) const = 0;
    virtual Real get_recording_duration() const = 0;

    // Playback
    virtual TrainingResult start_playback() = 0;
    virtual TrainingResult pause_playback() = 0;
    virtual TrainingResult stop_playback() = 0;
    virtual TrainingResult seek(Real time) = 0;
    virtual TrainingResult set_playback_speed(Real speed) = 0;
    virtual PlaybackState get_playback_state() const = 0;

    // Analysis
    virtual GradeReport generate_grade_report() const = 0;
    virtual std::vector<TrainingEvent> get_errors() const = 0;
    virtual std::vector<TrainingEvent> get_excellent_performance() const = 0;

    // Export
    virtual TrainingResult export_to_file(const std::string& filepath) = 0;
    virtual TrainingResult import_from_file(const std::string& filepath) = 0;
};
```

### IScenarioEngine

Scenario engine interface for scripted training scenarios.

```cpp
class IScenarioEngine {
public:
    // Scenario management
    virtual TrainingResult load_scenario(const ScenarioDefinition& scenario) = 0;
    virtual TrainingResult load_scenario_from_file(const std::string& filepath) = 0;
    virtual TrainingResult unload_scenario() = 0;
    virtual std::optional<ScenarioDefinition> get_current_scenario() const = 0;
    virtual bool is_scenario_loaded() const = 0;

    // Execution
    virtual TrainingResult start_scenario() = 0;
    virtual TrainingResult pause_scenario() = 0;
    virtual TrainingResult resume_scenario() = 0;
    virtual TrainingResult stop_scenario() = 0;
    virtual TrainingResult reset_scenario() = 0;
    virtual bool is_running() const = 0;

    // Objectives
    virtual std::vector<TrainingObjective> get_objectives() const = 0;
    virtual std::optional<TrainingObjective> get_current_objective() const = 0;
    virtual TrainingResult complete_objective(UInt32 objective_id) = 0;
    virtual TrainingResult fail_objective(UInt32 objective_id, const std::string& reason) = 0;
    virtual TrainingResult skip_objective(UInt32 objective_id) = 0;

    // Rules
    virtual TrainingResult add_rule(const ScriptRule& rule) = 0;
    virtual TrainingResult remove_rule(UInt32 rule_id) = 0;
    virtual TrainingResult enable_rule(UInt32 rule_id) = 0;
    virtual TrainingResult disable_rule(UInt32 rule_id) = 0;

    // Triggers
    virtual TrainingResult fire_trigger(UInt32 trigger_id) = 0;
    virtual TrainingResult evaluate_triggers() = 0;
    virtual TrainingResult execute_action(const ScriptAction& action) = 0;

    // Checkpoints
    virtual std::vector<std::string> get_checkpoints() const = 0;
    virtual TrainingResult jump_to_checkpoint(const std::string& checkpoint_id) = 0;
    virtual TrainingResult save_checkpoint(const std::string& checkpoint_id) = 0;

    // Update
    virtual TrainingResult update(Real delta_time) = 0;
};
```

### ITrainingSession

Training session interface that orchestrates all training modules.

```cpp
class ITrainingSession {
public:
    // Session lifecycle
    virtual TrainingResult create_session(const std::string& trainee_id, TrainingDomain domain) = 0;
    virtual TrainingResult start_session() = 0;
    virtual TrainingResult pause_session() = 0;
    virtual TrainingResult resume_session() = 0;
    virtual TrainingResult end_session() = 0;
    virtual bool is_active() const = 0;
    virtual Real get_session_time() const = 0;

    // Modules
    virtual std::shared_ptr<ICockpitModule> get_cockpit_module() = 0;
    virtual std::shared_ptr<IBridgeModule> get_bridge_module() = 0;
    virtual std::shared_ptr<IControlTowerModule> get_tower_module() = 0;
    virtual std::shared_ptr<IAfterActionReview> get_aar() = 0;
    virtual std::shared_ptr<IScenarioEngine> get_scenario_engine() = 0;

    // Configuration
    virtual TrainingResult set_difficulty(DifficultyLevel level) = 0;
    virtual DifficultyLevel get_difficulty() const = 0;
    virtual TrainingResult set_weather(WeatherPreset weather) = 0;
    virtual TrainingResult set_time_of_day(TimeOfDay time) = 0;

    // Statistics
    virtual UInt32 get_total_objectives() const = 0;
    virtual UInt32 get_completed_objectives() const = 0;
    virtual Real get_current_score() const = 0;

    // Update
    virtual TrainingResult update(Real delta_time) = 0;
};
```

---

## Training Factory Functions

```cpp
// Cockpit modules
std::unique_ptr<ICockpitModule> create_cockpit_module(const std::string& aircraft_type);
std::unique_ptr<ICockpitModule> create_generic_cockpit_module();

// Bridge modules
std::unique_ptr<IBridgeModule> create_bridge_module(const std::string& ship_type);
std::unique_ptr<IBridgeModule> create_generic_bridge_module();

// Control tower module
std::unique_ptr<IControlTowerModule> create_tower_module(const std::string& airport_icao);

// After-action review
std::unique_ptr<IAfterActionReview> create_aar();

// Scenario engine
std::unique_ptr<IScenarioEngine> create_scenario_engine();

// Training session
std::unique_ptr<ITrainingSession> create_training_session();
std::unique_ptr<ITrainingSession> create_mock_training_session();
```

---

## Training Usage Example

```cpp
using namespace jaguar::training;

// Create and configure training session
auto session = create_training_session();
session->create_session("pilot_001", TrainingDomain::Aviation);
session->set_difficulty(DifficultyLevel::Intermediate);
session->set_weather(WeatherPreset::Clear);
session->set_time_of_day(TimeOfDay::Morning);

// Start session
session->start_session();

// Get domain-specific module
auto cockpit = session->get_cockpit_module();
auto aar = session->get_aar();
auto scenario = session->get_scenario_engine();

// Create and load a scenario
ScenarioDefinition scenario_def;
scenario_def.id = "basic_flight";
scenario_def.name = "Basic Flight Training";
scenario_def.domain = TrainingDomain::Aviation;

TrainingObjective takeoff_obj;
takeoff_obj.id = 1;
takeoff_obj.type = ObjectiveType::Takeoff;
takeoff_obj.name = "Complete Takeoff";
takeoff_obj.is_mandatory = true;
scenario_def.objectives.push_back(takeoff_obj);

scenario->load_scenario(scenario_def);
scenario->start_scenario();

// Main training loop
while (session->is_active()) {
    // Update control inputs (from XR controllers or physical controls)
    cockpit->set_control_position(ControlType::Throttle, throttle_input);
    cockpit->set_control_position(ControlType::Yoke, yoke_pitch);

    // Get flight state for visualization
    Real airspeed = cockpit->get_airspeed();
    Real altitude = cockpit->get_altitude();
    Vec3 attitude = cockpit->get_attitude();

    // Check for stall conditions
    if (airspeed < 60) {
        TrainingEvent stall_warning;
        stall_warning.type = TrainingEventType::Stall;
        stall_warning.name = "Approaching stall";
        stall_warning.severity = 0.8;
        aar->record_event(stall_warning);
    }

    // Update session (handles modules, scenarios, recording)
    session->update(delta_time);

    // Check for objective completion
    if (altitude > 1000 && !takeoff_completed) {
        scenario->complete_objective(1);
        takeoff_completed = true;
    }
}

// End session
session->end_session();

// Generate grade report
GradeReport report = aar->generate_grade_report();
std::cout << "Overall Score: " << report.overall_score << std::endl;
std::cout << "Grade: " << static_cast<int>(report.overall_grade) << std::endl;
std::cout << "Passed: " << (report.is_passed ? "Yes" : "No") << std::endl;

for (const auto& area : report.areas_for_improvement) {
    std::cout << "Improve: " << area << std::endl;
}
```

---

## See Also

- [XR Session Header](../../../include/jaguar/xr/xr_session.h)
- [XR Types Header](../../../include/jaguar/xr/xr_types.h)
- [Spatial Audio Header](../../../include/jaguar/xr/spatial_audio.h)
- [Haptics Header](../../../include/jaguar/xr/haptics.h)
- [Training Header](../../../include/jaguar/xr/training.h)
- [XR Events Header](../../../include/jaguar/xr/xr_events.h)
