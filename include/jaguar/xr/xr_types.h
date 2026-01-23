#pragma once
/**
 * @file xr_types.h
 * @brief Core XR types and structures for OpenXR integration
 *
 * This file defines the fundamental types used throughout the XR subsystem,
 * including poses, tracking states, controller inputs, and hand tracking data.
 * Designed to abstract OpenXR specifics while maintaining full feature support.
 */

#include "jaguar/core/types.h"
#include <array>
#include <optional>
#include <bitset>
#include <chrono>
#include <string>
#include <functional>

namespace jaguar::xr {

// ============================================================================
// Forward Declarations
// ============================================================================

struct XRSession;
class IXRRuntime;

// ============================================================================
// XR Result Types
// ============================================================================

/**
 * @brief Result codes for XR operations
 */
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

/**
 * @brief Convert XRResult to string for logging
 */
const char* xr_result_to_string(XRResult result);

/**
 * @brief Check if XRResult indicates success
 */
inline bool xr_succeeded(XRResult result) {
    return result == XRResult::Success;
}

// ============================================================================
// Device and Session Types
// ============================================================================

/**
 * @brief XR form factor (headset type)
 */
enum class XRFormFactor : UInt8 {
    HeadMountedDisplay = 0,     ///< VR headset (Oculus, Vive, etc.)
    HandheldDisplay = 1         ///< AR handheld (phone-based AR)
};

/**
 * @brief XR view configuration type
 */
enum class XRViewConfigType : UInt8 {
    Mono = 0,                   ///< Single view (AR, mono rendering)
    Stereo = 1,                 ///< Stereoscopic views (VR)
    StereoWithFoveation = 2,    ///< Stereo with foveated rendering
    Quad = 3                    ///< Four views (experimental)
};

/**
 * @brief XR blend mode for compositing
 */
enum class XRBlendMode : UInt8 {
    Opaque = 0,                 ///< Fully opaque VR (no passthrough)
    Additive = 1,               ///< Additive blending (holographic AR)
    AlphaBlend = 2              ///< Alpha blending (video passthrough AR)
};

/**
 * @brief XR session state
 */
enum class XRSessionState : UInt8 {
    Unknown = 0,
    Idle = 1,                   ///< Session created, not running
    Ready = 2,                  ///< Session ready to begin
    Synchronized = 3,           ///< Synchronized with runtime
    Visible = 4,                ///< Content visible but not focused
    Focused = 5,                ///< Fully focused and interactive
    Stopping = 6,               ///< Session stopping
    Exiting = 7,                ///< Session exiting
    LossPending = 8             ///< Session loss imminent
};

/**
 * @brief Session state change callback
 */
using SessionStateCallback = std::function<void(XRSessionState old_state, XRSessionState new_state)>;

// ============================================================================
// Tracking Types
// ============================================================================

/**
 * @brief Tracking confidence level
 */
enum class TrackingConfidence : UInt8 {
    None = 0,                   ///< No tracking
    Low = 1,                    ///< Limited tracking (inference)
    High = 2                    ///< Full tracking
};

/**
 * @brief Space reference type
 */
enum class XRSpaceType : UInt8 {
    View = 0,                   ///< View (head) relative
    Local = 1,                  ///< Local (seated/standing origin)
    Stage = 2,                  ///< Stage (room-scale boundary)
    Unbounded = 3               ///< Unbounded (large-area tracking)
};

/**
 * @brief XR pose (position + orientation)
 *
 * Represents a 6-DOF pose in 3D space. Position is in meters,
 * orientation is a unit quaternion.
 */
struct XRPose {
    Vec3 position{Vec3::Zero()};
    Quat orientation{Quat::Identity()};
    TrackingConfidence position_confidence{TrackingConfidence::None};
    TrackingConfidence orientation_confidence{TrackingConfidence::None};
    bool is_valid{false};

    /// Check if pose has any valid tracking
    bool has_tracking() const {
        return is_valid && (position_confidence != TrackingConfidence::None ||
                           orientation_confidence != TrackingConfidence::None);
    }

    /// Check if pose has full 6-DOF tracking
    bool has_full_tracking() const {
        return is_valid &&
               position_confidence == TrackingConfidence::High &&
               orientation_confidence == TrackingConfidence::High;
    }

    /// Get forward direction (-Z in OpenXR convention)
    Vec3 forward() const {
        return orientation.rotate(Vec3{0.0, 0.0, -1.0});
    }

    /// Get up direction (+Y in OpenXR convention)
    Vec3 up() const {
        return orientation.rotate(Vec3{0.0, 1.0, 0.0});
    }

    /// Get right direction (+X in OpenXR convention)
    Vec3 right() const {
        return orientation.rotate(Vec3{1.0, 0.0, 0.0});
    }

    /// Transform a point from local to world space
    Vec3 transform_point(const Vec3& local_point) const {
        return position + orientation.rotate(local_point);
    }

    /// Transform a direction from local to world space
    Vec3 transform_direction(const Vec3& local_dir) const {
        return orientation.rotate(local_dir);
    }

    /// Interpolate between two poses (linear position, slerp orientation)
    static XRPose lerp(const XRPose& a, const XRPose& b, Real t);

    /// Identity pose
    static XRPose Identity() {
        XRPose pose;
        pose.is_valid = true;
        pose.position_confidence = TrackingConfidence::High;
        pose.orientation_confidence = TrackingConfidence::High;
        return pose;
    }
};

/**
 * @brief Velocity and angular velocity data
 */
struct XRVelocity {
    Vec3 linear{Vec3::Zero()};      ///< Linear velocity (m/s)
    Vec3 angular{Vec3::Zero()};     ///< Angular velocity (rad/s)
    bool linear_valid{false};
    bool angular_valid{false};
};

/**
 * @brief Complete tracked pose with velocity
 */
struct XRTrackedPose {
    XRPose pose;
    XRVelocity velocity;
    Int64 timestamp_ns{0};          ///< Tracking timestamp in nanoseconds
};

// ============================================================================
// HMD Types
// ============================================================================

/**
 * @brief Per-eye view data
 */
struct XRViewState {
    XRPose pose;                    ///< Eye pose relative to head
    Real fov_left{-0.8f};           ///< Left FOV angle (radians)
    Real fov_right{0.8f};           ///< Right FOV angle (radians)
    Real fov_up{0.8f};              ///< Up FOV angle (radians)
    Real fov_down{-0.8f};           ///< Down FOV angle (radians)
    UInt32 recommended_width{1920}; ///< Recommended render width
    UInt32 recommended_height{1920};///< Recommended render height
};

/**
 * @brief Eye identifier
 */
enum class XREye : UInt8 {
    Left = 0,
    Right = 1,
    Count = 2
};

/**
 * @brief HMD tracking state
 */
struct XRHeadState {
    XRTrackedPose head_pose;        ///< Head center pose
    std::array<XRViewState, 2> views; ///< Per-eye view states
    bool is_user_present{false};    ///< User wearing headset
    Real ipd{0.063f};               ///< Interpupillary distance (meters)
};

// ============================================================================
// Eye Tracking Types
// ============================================================================

/**
 * @brief Single eye gaze data
 */
struct XREyeGaze {
    XRPose gaze_pose;               ///< Eye position and gaze direction
    Real openness{1.0f};            ///< Eye openness (0=closed, 1=open)
    Real pupil_diameter{0.004f};    ///< Pupil diameter (meters)
    bool is_valid{false};
};

/**
 * @brief Combined eye tracking state
 */
struct XREyeTrackingState {
    XREyeGaze left_eye;
    XREyeGaze right_eye;
    XRPose combined_gaze;           ///< Combined gaze ray
    Vec3 fixation_point{Vec3::Zero()}; ///< 3D fixation point
    bool fixation_valid{false};
    Int64 timestamp_ns{0};
};

// ============================================================================
// Controller Types
// ============================================================================

/**
 * @brief Controller hand identifier
 */
enum class XRHand : UInt8 {
    Left = 0,
    Right = 1,
    Count = 2
};

/**
 * @brief Controller button identifiers
 */
enum class XRButton : UInt8 {
    // Face buttons
    A = 0,              ///< A button (right) / X button (left)
    B = 1,              ///< B button (right) / Y button (left)
    X = 2,              ///< X button (left only on Oculus)
    Y = 3,              ///< Y button (left only on Oculus)

    // System buttons
    Menu = 4,           ///< Menu button
    System = 5,         ///< System button (may be reserved)

    // Thumbstick
    ThumbstickClick = 6,
    ThumbstickTouch = 7,

    // Trackpad (if present)
    TrackpadClick = 8,
    TrackpadTouch = 9,

    // Triggers
    TriggerTouch = 10,
    GripTouch = 11,

    // Squeeze (Index controllers)
    Squeeze = 12,

    Count = 13
};

/**
 * @brief Controller axis identifiers
 */
enum class XRAxis : UInt8 {
    ThumbstickX = 0,
    ThumbstickY = 1,
    TrackpadX = 2,
    TrackpadY = 3,
    Trigger = 4,
    Grip = 5,
    Count = 6
};

/**
 * @brief Haptic feedback parameters
 */
struct XRHapticFeedback {
    Real amplitude{1.0f};           ///< Vibration strength (0-1)
    Real frequency{160.0f};         ///< Vibration frequency (Hz)
    Real duration_seconds{0.1f};    ///< Duration in seconds
};

/**
 * @brief Controller input state
 */
struct XRControllerState {
    XRTrackedPose pose;             ///< Controller pose
    XRTrackedPose aim_pose;         ///< Aim/pointer pose
    XRTrackedPose grip_pose;        ///< Grip pose

    std::bitset<static_cast<size_t>(XRButton::Count)> buttons_pressed;
    std::bitset<static_cast<size_t>(XRButton::Count)> buttons_touched;
    std::array<Real, static_cast<size_t>(XRAxis::Count)> axes{};

    bool is_connected{false};
    bool is_active{false};          ///< User actively holding controller

    /// Check if button is pressed
    bool is_pressed(XRButton button) const {
        return buttons_pressed[static_cast<size_t>(button)];
    }

    /// Check if button is touched
    bool is_touched(XRButton button) const {
        return buttons_touched[static_cast<size_t>(button)];
    }

    /// Get axis value
    Real get_axis(XRAxis axis) const {
        return axes[static_cast<size_t>(axis)];
    }

    /// Get trigger value (convenience)
    Real trigger() const { return get_axis(XRAxis::Trigger); }

    /// Get grip value (convenience)
    Real grip() const { return get_axis(XRAxis::Grip); }

    /// Get thumbstick as Vec2 (convenience)
    Vec3 thumbstick() const {
        return Vec3{get_axis(XRAxis::ThumbstickX),
                   get_axis(XRAxis::ThumbstickY), 0.0};
    }
};

// ============================================================================
// Hand Tracking Types
// ============================================================================

/**
 * @brief Hand joint identifiers (OpenXR standard)
 */
enum class XRHandJoint : UInt8 {
    Palm = 0,
    Wrist = 1,
    ThumbMetacarpal = 2,
    ThumbProximal = 3,
    ThumbDistal = 4,
    ThumbTip = 5,
    IndexMetacarpal = 6,
    IndexProximal = 7,
    IndexIntermediate = 8,
    IndexDistal = 9,
    IndexTip = 10,
    MiddleMetacarpal = 11,
    MiddleProximal = 12,
    MiddleIntermediate = 13,
    MiddleDistal = 14,
    MiddleTip = 15,
    RingMetacarpal = 16,
    RingProximal = 17,
    RingIntermediate = 18,
    RingDistal = 19,
    RingTip = 20,
    LittleMetacarpal = 21,
    LittleProximal = 22,
    LittleIntermediate = 23,
    LittleDistal = 24,
    LittleTip = 25,
    Count = 26
};

/**
 * @brief Single hand joint data
 */
struct XRHandJointData {
    XRPose pose;
    Real radius{0.01f};             ///< Joint radius (meters)
    XRVelocity velocity;
};

/**
 * @brief Hand gesture types
 */
enum class XRHandGesture : UInt8 {
    None = 0,
    Open = 1,               ///< Open hand
    Fist = 2,               ///< Closed fist
    Point = 3,              ///< Index finger pointing
    Pinch = 4,              ///< Thumb and index pinch
    ThumbsUp = 5,           ///< Thumbs up
    Custom = 255
};

/**
 * @brief Complete hand tracking state
 */
struct XRHandState {
    XRHand hand{XRHand::Left};
    std::array<XRHandJointData, static_cast<size_t>(XRHandJoint::Count)> joints;
    TrackingConfidence confidence{TrackingConfidence::None};
    bool is_active{false};

    // Gesture detection
    XRHandGesture detected_gesture{XRHandGesture::None};
    Real gesture_confidence{0.0f};

    // Pinch detection
    Real pinch_strength{0.0f};      ///< 0 = open, 1 = pinched
    Vec3 pinch_point{Vec3::Zero()}; ///< Point between thumb and index

    /// Get specific joint data
    const XRHandJointData& get_joint(XRHandJoint joint) const {
        return joints[static_cast<size_t>(joint)];
    }

    /// Check if hand tracking is valid
    bool is_valid() const {
        return is_active && confidence != TrackingConfidence::None;
    }

    /// Get fingertip positions for common interactions
    Vec3 index_tip() const { return get_joint(XRHandJoint::IndexTip).pose.position; }
    Vec3 thumb_tip() const { return get_joint(XRHandJoint::ThumbTip).pose.position; }
    Vec3 palm_position() const { return get_joint(XRHandJoint::Palm).pose.position; }
    Quat palm_orientation() const { return get_joint(XRHandJoint::Palm).pose.orientation; }
};

// ============================================================================
// Foveated Rendering Types
// ============================================================================

/**
 * @brief Foveation level
 */
enum class XRFoveationLevel : UInt8 {
    None = 0,           ///< No foveation
    Low = 1,            ///< Subtle foveation
    Medium = 2,         ///< Moderate foveation
    High = 3,           ///< Aggressive foveation
    Dynamic = 4         ///< Eye-tracking driven
};

/**
 * @brief Foveated rendering configuration
 */
struct XRFoveationConfig {
    XRFoveationLevel level{XRFoveationLevel::None};
    bool use_eye_tracking{false};   ///< Use eye tracking for dynamic foveation
    Real vertical_offset{0.0f};     ///< Vertical center offset
    Real min_pixel_density{0.5f};   ///< Minimum peripheral density
};

// ============================================================================
// Passthrough/AR Types
// ============================================================================

/**
 * @brief Passthrough mode
 */
enum class XRPassthroughMode : UInt8 {
    Disabled = 0,
    FullPassthrough = 1,    ///< Full camera passthrough
    SelectivePassthrough = 2 ///< Passthrough with virtual occlusion
};

/**
 * @brief Passthrough layer configuration
 */
struct XRPassthroughConfig {
    XRPassthroughMode mode{XRPassthroughMode::Disabled};
    Real opacity{1.0f};             ///< Passthrough opacity (0-1)
    Real brightness{0.0f};          ///< Brightness adjustment (-1 to 1)
    Real contrast{0.0f};            ///< Contrast adjustment (-1 to 1)
    Real saturation{0.0f};          ///< Saturation adjustment (-1 to 1)
    bool edge_enhancement{false};   ///< Enable edge enhancement
};

// ============================================================================
// Input Action Types
// ============================================================================

/**
 * @brief Input action type
 */
enum class XRActionType : UInt8 {
    Boolean = 0,        ///< Button press/release
    Float = 1,          ///< Analog value (trigger, grip)
    Vector2 = 2,        ///< 2D input (thumbstick)
    Pose = 3,           ///< 6-DOF pose
    Haptic = 4          ///< Haptic output
};

/**
 * @brief Input action binding
 */
struct XRActionBinding {
    std::string action_name;
    std::string binding_path;       ///< OpenXR binding path
    XRActionType type{XRActionType::Boolean};
};

/**
 * @brief Action set for grouping related actions
 */
struct XRActionSet {
    std::string name;
    std::string localized_name;
    UInt32 priority{0};
};

// ============================================================================
// Frame Timing Types
// ============================================================================

/**
 * @brief Frame timing information
 */
struct XRFrameTiming {
    Int64 predicted_display_time_ns{0};  ///< When frame will be displayed
    Int64 predicted_display_period_ns{0}; ///< Display refresh period
    bool should_render{true};
};

/**
 * @brief Frame submission result
 */
struct XRFrameResult {
    XRResult result{XRResult::Success};
    Int64 actual_display_time_ns{0};
    Real compositor_latency_ms{0.0f};
};

// ============================================================================
// Guardian/Boundary Types
// ============================================================================

/**
 * @brief Play area boundary type
 */
enum class XRBoundaryType : UInt8 {
    Seated = 0,         ///< Seated experience (no boundary)
    Standing = 1,       ///< Standing with small boundary
    RoomScale = 2       ///< Full room-scale boundary
};

/**
 * @brief Boundary proximity warning
 */
struct XRBoundaryWarning {
    bool is_triggered{false};
    Real distance{0.0f};            ///< Distance to boundary (meters)
    Vec3 closest_point{Vec3::Zero()}; ///< Closest point on boundary
    Vec3 normal{Vec3::Zero()};      ///< Boundary normal at closest point
};

/**
 * @brief Play area dimensions
 */
struct XRPlayArea {
    XRBoundaryType type{XRBoundaryType::Seated};
    Real width{0.0f};               ///< Play area width (meters)
    Real depth{0.0f};               ///< Play area depth (meters)
    Vec3 center{Vec3::Zero()};      ///< Center in stage space
    std::vector<Vec3> boundary_points; ///< Boundary polygon vertices
    bool is_configured{false};
};

// ============================================================================
// Extension Support
// ============================================================================

/**
 * @brief Common OpenXR extensions
 */
enum class XRExtension : UInt32 {
    HandTracking = 1 << 0,
    EyeTracking = 1 << 1,
    FoveatedRendering = 1 << 2,
    Passthrough = 1 << 3,
    ControllerInteraction = 1 << 4,
    HandInteraction = 1 << 5,
    SpatialAnchor = 1 << 6,
    SceneUnderstanding = 1 << 7,
    FacialTracking = 1 << 8,
    BodyTracking = 1 << 9
};

inline XRExtension operator|(XRExtension a, XRExtension b) {
    return static_cast<XRExtension>(static_cast<UInt32>(a) | static_cast<UInt32>(b));
}

inline XRExtension operator&(XRExtension a, XRExtension b) {
    return static_cast<XRExtension>(static_cast<UInt32>(a) & static_cast<UInt32>(b));
}

inline bool has_extension(XRExtension mask, XRExtension ext) {
    return (static_cast<UInt32>(mask) & static_cast<UInt32>(ext)) != 0;
}

} // namespace jaguar::xr
