# Extended Reality (XR) Module Documentation

The XR module provides comprehensive VR/AR integration through OpenXR, including headset tracking, controller input, hand tracking, eye tracking, spatial audio, haptic feedback, and motion platform support. Designed for immersive simulation experiences across defense training applications.

## Headers

| Header | Purpose |
|--------|---------|
| `jaguar/xr/xr_types.h` | Core XR types, poses, tracking states |
| `jaguar/xr/xr_session.h` | XR session lifecycle and runtime management |
| `jaguar/xr/xr_events.h` | XR-specific events |
| `jaguar/xr/spatial_audio.h` | HRTF-based 3D positional audio |
| `jaguar/xr/haptics.h` | Controller vibration, haptic vests, motion platforms |
| `jaguar/xr/training.h` | Training scenario management |

## XR Session (`xr_session.h`)

### Session Configuration

```cpp
// Default VR configuration
XRSessionConfig config = XRSessionConfig::VR_Default();

// Seated VR experience
XRSessionConfig config = XRSessionConfig::VR_Seated();

// Room-scale VR with guardian
XRSessionConfig config = XRSessionConfig::VR_RoomScale();

// AR with passthrough
XRSessionConfig config = XRSessionConfig::AR_Passthrough();

// Full features (hand tracking, eye tracking)
XRSessionConfig config = XRSessionConfig::VR_FullFeatures();
```

### Custom Configuration

```cpp
XRSessionConfig config;

// Form factor and view
config.form_factor = XRFormFactor::HeadMountedDisplay;
config.view_config = XRViewConfigType::Stereo;
config.preferred_blend_mode = XRBlendMode::Opaque;

// Tracking
config.reference_space = XRSpaceType::Stage;
config.enable_guardian = true;

// Features
config.enable_hand_tracking = true;
config.enable_eye_tracking = true;
config.enable_passthrough = false;

// Rendering
config.swapchain_sample_count = 4;  // MSAA
config.use_depth_submission = true;
config.target_frame_rate = 90.0f;

// Foveated rendering
config.foveation.level = XRFoveationLevel::Dynamic;
config.foveation.use_eye_tracking = true;

// Extensions
config.requested_extensions =
    XRExtension::HandTracking |
    XRExtension::EyeTracking |
    XRExtension::FoveatedRendering |
    XRExtension::ControllerInteraction;
```

### Session Lifecycle

```cpp
XRSession session;

// Initialize
XRResult result = session.initialize(config);
if (result != XRResult::Success) {
    // Handle error
}

// Check capabilities
const XRRuntimeCapabilities& caps = session.get_capabilities();
if (caps.supports_hand_tracking) {
    // Hand tracking available
}

// Main loop
while (session.is_running()) {
    XRFrameTiming timing;
    session.begin_frame(timing);

    if (session.should_render()) {
        // Get tracking data
        const XRHeadState& head = session.get_head_state(timing.predicted_display_time_ns);

        // Render scene using head.views[0] and head.views[1]
        render_scene(head);
    }

    session.end_frame();
}

session.shutdown();
```

### State Callbacks

```cpp
session.set_state_callback([](XRSessionState old_state, XRSessionState new_state) {
    switch (new_state) {
        case XRSessionState::Ready:
            // Session ready to begin
            break;
        case XRSessionState::Focused:
            // Session has input focus
            break;
        case XRSessionState::Stopping:
            // Session ending
            break;
    }
});
```

## Head Tracking

```cpp
// Get head state with motion prediction
const XRHeadState& head = session.get_head_state(predicted_time_ns);

// Head pose
Vec3 head_position = head.pose.position;
Quat head_orientation = head.pose.orientation;

// Per-eye views
const XRView& left_eye = head.views[0];
const XRView& right_eye = head.views[1];

// View matrices for rendering
Mat4x4 left_view = left_eye.view_matrix;
Mat4x4 left_proj = left_eye.projection_matrix;

// FOV for culling
Real left_fov = left_eye.fov_left;   // radians
Real right_fov = left_eye.fov_right;
Real up_fov = left_eye.fov_up;
Real down_fov = left_eye.fov_down;
```

## Controller Input

```cpp
// Get controller state
const XRControllerState& left = session.get_controller_state(XRHand::Left);
const XRControllerState& right = session.get_controller_state(XRHand::Right);

if (left.is_active) {
    // Poses
    Vec3 aim_position = left.aim_pose.position;
    Quat aim_orientation = left.aim_pose.orientation;
    Vec3 grip_position = left.grip_pose.position;

    // Analog inputs
    Real trigger = left.trigger;        // 0.0 - 1.0
    Real grip = left.grip;              // 0.0 - 1.0
    Vec2 thumbstick = left.thumbstick;  // -1.0 to 1.0

    // Button states
    if (left.button_pressed & XRButton::A) {
        // A button pressed this frame
    }
    if (left.button_held & XRButton::Trigger) {
        // Trigger held
    }
}

// Convenience methods
if (session.is_trigger_pressed(XRHand::Right, 0.8f)) {
    // Right trigger pressed past 80%
}

// Get interaction ray for pointing
Vec3 origin, direction;
session.get_interaction_ray(XRHand::Right, origin, direction);
```

## Hand Tracking

```cpp
if (session.is_hand_tracking_active(XRHand::Left)) {
    const XRHandState& hand = session.get_hand_state(XRHand::Left);

    // Individual joint poses
    const XRTrackedPose& wrist = hand.joints[static_cast<int>(XRHandJoint::Wrist)];
    const XRTrackedPose& index_tip = hand.joints[static_cast<int>(XRHandJoint::IndexTip)];

    // Gesture detection
    if (session.is_pinching(XRHand::Left, 0.8f)) {
        // Thumb and index finger pinching
    }

    // Pinch point for interaction
    Vec3 pinch_point = hand.pinch_pose.position;
}
```

## Eye Tracking

```cpp
if (session.is_eye_tracking_active()) {
    const XREyeTrackingState& eye = session.get_eye_tracking_state();

    if (eye.gaze_valid) {
        // Gaze ray
        Vec3 gaze_origin = eye.gaze_origin;
        Vec3 gaze_direction = eye.gaze_direction;

        // Individual eyes
        Vec3 left_gaze = eye.left_gaze_direction;
        Vec3 right_gaze = eye.right_gaze_direction;

        // Pupil data
        Real left_pupil = eye.left_pupil_diameter;  // mm
        Real openness = eye.left_eye_openness;      // 0-1
    }
}
```

## Haptic Feedback (`haptics.h`)

### Controller Vibration

```cpp
// Simple vibration
session.vibrate(XRHand::Right, 1.0f, 0.1f, 160.0f);
// amplitude, duration (seconds), frequency (Hz)

// Stop vibration
session.stop_vibration(XRHand::Right);

// Custom effect
HapticEffect effect = HapticEffect::Vibration(0.8f, 0.2f, 200.0f);
effect.envelope = HapticEnvelope::ADSR(0.05, 0.1, 0.7, 0.1);
controller->play_effect(XRHand::Right, effect);
```

### Haptic Effect Presets

```cpp
// Impact effects
HapticEffect impact = HapticEffect::Impact(0.9f);
HapticEffect pulse = HapticEffect::Pulse(0.7f, 0.05f, 0.1f, 3);
HapticEffect rumble = HapticEffect::Rumble(0.5f, 0.3f);

// Aircraft-specific
HapticEffect stall = presets::aircraft::StallBuffet(intensity);
HapticEffect touchdown = presets::aircraft::Touchdown(vertical_speed);
HapticEffect afterburner = presets::aircraft::AfterburnerIgnition();

// Vehicle-specific
HapticEffect road = presets::vehicle::RoadTexture(roughness);
HapticEffect collision = presets::vehicle::Collision(severity, direction);
HapticEffect gear_shift = presets::vehicle::GearShift();

// Naval-specific
HapticEffect waves = presets::naval::WaveMotion(sea_state);
HapticEffect gun = presets::naval::NavalGunFiring(caliber);
```

### Haptic Vest

```cpp
auto vest = haptic_renderer->get_vest();

// Set individual zone
vest->set_zone_intensity(VestZone::ChestCenter, 0.8f);

// Set pattern
VestHapticPattern pattern;
pattern.set_front(0.7f);  // All front zones
pattern.set_back(0.3f);   // All back zones
pattern.duration = 0.2f;
vest->play_pattern(pattern);

// Directional impact
Vec3 impact_direction = (hit_position - player_position).normalized();
VestHapticPattern impact = VestHapticPattern::ImpactFromDirection(impact_direction, 1.0f);
vest->play_pattern(impact);

// Biometric effects
VestHapticPattern heartbeat = VestHapticPattern::Heartbeat(120.0f);  // BPM
VestHapticPattern breathing = VestHapticPattern::Breathing(0.2f);    // rate
```

### Motion Platform

```cpp
auto platform = haptic_renderer->get_motion_platform();

// Get capabilities
MotionPlatformCapabilities caps = platform->get_platform_capabilities();
// 6-DOF: surge, sway, heave, roll, pitch, yaw

// Initialize
platform->home();
platform->enable();

// Set motion cueing parameters
MotionCueingParams params = MotionCueingParams::AircraftDefault();
params.tilt_coordination_gain = 0.6f;
platform->set_cueing_params(params);

// Apply vehicle state (called each frame)
platform->apply_vehicle_state(
    linear_acceleration,      // Vec3 (m/s^2)
    angular_velocity,         // Vec3 (rad/s)
    angular_acceleration      // Vec3 (rad/s^2)
);

// Add vibration overlay
platform->add_vibration(HapticEffect::Rumble(0.3f, 0.0f), 1.0f);

// Shutdown
platform->park();
platform->disable();
```

### G-Force Simulation

```cpp
GForceConfig g_config = GForceConfig::FighterJet();
g_config.simulate_gsuit = true;
g_config.gsuit_threshold = 2.5f;

// Calculate G-force state
GForceState g_state = haptic_renderer->calculate_g_force(
    body_acceleration,
    gravity_in_body_frame
);

// Apply to haptics
haptic_renderer->apply_g_force(g_state);

// Access state
Real total_g = g_state.total_g;
bool gsuit_active = g_state.gsuit_active;
Real gsuit_pressure = g_state.gsuit_pressure;
```

## Spatial Audio (`spatial_audio.h`)

### Audio Renderer Setup

```cpp
auto audio_renderer = create_spatial_audio_renderer();

AudioFormat format;
format.sample_rate = 48000;
format.channel_layout = AudioChannelLayout::Binaural;

HRTFConfig hrtf;
hrtf.quality = HRTFQuality::High;
hrtf.dataset_path = "data/hrtf/mit_kemar.sofa";
hrtf.max_sources = 64;

audio_renderer->initialize(format, hrtf);
```

### Audio Sources

```cpp
AudioSourceConfig config;
config.name = "engine";
config.entity_id = aircraft_entity;
config.source_type = AudioSourceType::Point;
config.priority = AudioPriority::High;

// Attenuation
config.attenuation = DistanceAttenuation::Outdoor();
config.attenuation.max_distance = 2000.0f;

// Effects
config.enable_doppler = true;
config.enable_occlusion = true;
config.enable_reverb = true;

auto source = audio_renderer->create_source(config);
source->play();

// Update position each frame
source->set_position(aircraft_position);
source->set_velocity(aircraft_velocity);  // For Doppler
```

### Directional Sources

```cpp
AudioSourceConfig config;
config.source_type = AudioSourceType::Directional;

// Audio cone
config.cone.inner_angle = 30.0 * DEG_TO_RAD;   // Full gain
config.cone.outer_angle = 90.0 * DEG_TO_RAD;   // Zero gain
config.cone.outer_gain = 0.2f;                  // Gain outside outer

auto source = audio_renderer->create_source(config);
source->set_orientation(source_orientation);
```

### Listener (XR Integration)

```cpp
ISpatialAudioListener* listener = audio_renderer->get_listener();

// Manual position
listener->set_position(head_position);
listener->set_orientation(head_orientation);
listener->set_velocity(head_velocity);

// Or link to XR tracking
listener->set_xr_tracking(session.get_head_state().pose);
```

### Room Acoustics

```cpp
RoomAcousticsConfig room;
room.preset = RoomPreset::Cockpit;

// Or custom room
room.geometry.dimensions = Vec3{3.0, 2.0, 5.0};  // meters
room.geometry.wall_material = RoomMaterial::Glass();
room.geometry.floor_material = RoomMaterial::Carpet();

room.reverb.decay_time = 0.3f;        // RT60
room.reverb.pre_delay = 0.005f;
room.reverb.wet_dry_mix = 0.2f;

audio_renderer->set_room_acoustics(room);
```

### Occlusion

```cpp
OcclusionConfig occlusion;
occlusion.method = OcclusionMethod::Raycast;
occlusion.ray_count = 8;
occlusion.max_low_pass_frequency = 500.0f;  // Cutoff when fully occluded

audio_renderer->set_occlusion_config(occlusion);

// Occlusion is calculated automatically each frame
OcclusionState state = source->get_occlusion_state();
if (state.is_occluded()) {
    // Source is behind an obstacle
}
```

### Doppler Effect

```cpp
DopplerConfig doppler;
doppler.enabled = true;
doppler.factor = 1.0f;              // Exaggeration factor
doppler.speed_of_sound = 343.0f;    // m/s
doppler.max_pitch_shift = 4.0f;     // Maximum multiplier

audio_renderer->set_doppler_config(doppler);
```

## Boundary System

```cpp
// Get play area
const XRPlayArea& area = session.get_play_area();
Real width = area.width;   // meters
Real depth = area.depth;

// Check boundary proximity
XRBoundaryWarning warning = session.check_boundary(head_position);
if (warning.level == XRBoundaryLevel::Close) {
    // Show visual warning
}

// Check all tracked devices
XRBoundaryWarning device_warning = session.check_devices_boundary();
```

## Entity Integration

```cpp
// Sync XR tracking to entities
session.sync_head_to_entity(player_head_entity);
session.sync_controller_to_entity(left_hand_entity, XRHand::Left);
session.sync_controller_to_entity(right_hand_entity, XRHand::Right);

// For hand tracking
session.sync_hand_to_entity(left_hand_entity, XRHand::Left);
```

## Performance Settings

```cpp
// Foveated rendering
XRFoveationConfig fov;
fov.level = XRFoveationLevel::Dynamic;
fov.use_eye_tracking = true;
fov.peripheral_quality = 0.5f;  // 50% resolution at periphery
session.set_foveation(fov);

// Refresh rate
session.set_refresh_rate(120.0f);  // If supported

// Passthrough
session.enable_passthrough(true, 0.8f);  // 80% opacity
```

## Statistics

```cpp
const XRSession::Statistics& stats = session.get_statistics();

UInt64 frames = stats.frames_rendered;
UInt64 dropped = stats.frames_dropped;
Real frame_time = stats.average_frame_time_ms;
Real latency = stats.compositor_latency_ms;
```

## Build Configuration

Enable XR in CMake:

```cmake
option(JAGUAR_ENABLE_XR "Enable XR (VR/AR) support" ON)
option(JAGUAR_ENABLE_OPENXR "Enable OpenXR runtime" OFF)
```
