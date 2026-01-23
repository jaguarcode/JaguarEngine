#pragma once
/**
 * @file xr_events.h
 * @brief XR-specific event types for the JaguarEngine event system
 *
 * This file extends the core event system with XR-specific events for
 * tracking state changes, input events, and session state transitions.
 */

#include "jaguar/events/event.h"
#include "jaguar/xr/xr_types.h"

#include <cmath>

namespace jaguar::xr {

// ============================================================================
// XR Event Types (700-799 range)
// ============================================================================

/**
 * @brief XR-specific event type enumeration
 *
 * These extend the base EventType enum in the 700-799 range reserved for XR.
 */
enum class XREventType : UInt32 {
    // Session Events (700-709)
    SessionStateChanged     = 700,
    SessionFocusGained      = 701,
    SessionFocusLost        = 702,
    SessionVisibilityChanged = 703,

    // Tracking Events (710-729)
    TrackingLost            = 710,
    TrackingRecovered       = 711,
    RecenterRequested       = 712,
    RecenterCompleted       = 713,
    ReferenceSpaceChanged   = 714,

    // Controller Events (730-749)
    ControllerConnected     = 730,
    ControllerDisconnected  = 731,
    ControllerButtonPressed = 732,
    ControllerButtonReleased = 733,
    ControllerTriggerChanged = 734,
    ControllerGripChanged   = 735,
    ControllerThumbstickMoved = 736,

    // Hand Tracking Events (750-769)
    HandTrackingStarted     = 750,
    HandTrackingLost        = 751,
    HandGestureDetected     = 752,
    HandGestureEnded        = 753,
    PinchStarted            = 754,
    PinchEnded              = 755,
    PinchMoved              = 756,

    // Eye Tracking Events (770-779)
    EyeTrackingStarted      = 770,
    EyeTrackingLost         = 771,
    GazeTargetChanged       = 772,
    BlinkDetected           = 773,

    // Boundary Events (780-789)
    BoundaryApproaching     = 780,
    BoundaryEntered         = 781,
    BoundaryExited          = 782,
    BoundaryConfigChanged   = 783,

    // Passthrough Events (790-799)
    PassthroughStarted      = 790,
    PassthroughStopped      = 791,
    PassthroughConfigChanged = 792
};

/**
 * @brief Convert XREventType to base EventType
 */
inline events::EventType to_event_type(XREventType type) {
    return static_cast<events::EventType>(static_cast<UInt32>(type));
}

/**
 * @brief Check if EventType is an XR event
 */
inline bool is_xr_event(events::EventType type) {
    auto value = static_cast<UInt32>(type);
    return value >= 700 && value < 800;
}

// ============================================================================
// XR Event Data Structures
// ============================================================================

/**
 * @brief Session state change event data
 */
struct XRSessionEventData {
    XRSessionState old_state{XRSessionState::Unknown};
    XRSessionState new_state{XRSessionState::Unknown};
    bool is_running{false};
    bool is_focused{false};
    bool is_visible{false};
};

/**
 * @brief Tracking state change event data
 */
struct XRTrackingEventData {
    enum class Device : UInt8 {
        HMD = 0,
        LeftController = 1,
        RightController = 2,
        LeftHand = 3,
        RightHand = 4
    };

    Device device{Device::HMD};
    TrackingConfidence old_confidence{TrackingConfidence::None};
    TrackingConfidence new_confidence{TrackingConfidence::None};
    XRPose last_known_pose;
};

/**
 * @brief Controller button event data
 */
struct XRControllerButtonEventData {
    XRHand hand{XRHand::Left};
    XRButton button{XRButton::A};
    bool is_pressed{false};
    bool is_touched{false};
    Real analog_value{0.0f};        ///< For triggers/grips
};

/**
 * @brief Controller axis event data
 */
struct XRControllerAxisEventData {
    XRHand hand{XRHand::Left};
    XRAxis axis{XRAxis::ThumbstickX};
    Real old_value{0.0f};
    Real new_value{0.0f};
    Real delta{0.0f};
};

/**
 * @brief Controller thumbstick event data
 */
struct XRThumbstickEventData {
    XRHand hand{XRHand::Left};
    Real x{0.0f};                   ///< X axis (-1 to 1)
    Real y{0.0f};                   ///< Y axis (-1 to 1)
    Real magnitude{0.0f};           ///< Distance from center
    Real angle{0.0f};               ///< Angle in radians
    bool is_clicked{false};
    bool is_touched{false};
};

/**
 * @brief Hand tracking event data
 */
struct XRHandTrackingEventData {
    XRHand hand{XRHand::Left};
    TrackingConfidence confidence{TrackingConfidence::None};
    XRPose palm_pose;
};

/**
 * @brief Hand gesture event data
 */
struct XRGestureEventData {
    XRHand hand{XRHand::Left};
    XRHandGesture gesture{XRHandGesture::None};
    XRHandGesture previous_gesture{XRHandGesture::None};
    Real confidence{0.0f};
    Vec3 gesture_point{Vec3::Zero()}; ///< Focal point of gesture
};

/**
 * @brief Pinch event data
 */
struct XRPinchEventData {
    XRHand hand{XRHand::Left};
    Real strength{0.0f};            ///< 0 = open, 1 = pinched
    Vec3 pinch_point{Vec3::Zero()}; ///< Point between thumb and index
    Vec3 delta_position{Vec3::Zero()}; ///< Movement since last frame
};

/**
 * @brief Eye tracking event data
 */
struct XREyeTrackingEventData {
    XRPose combined_gaze;
    Vec3 fixation_point{Vec3::Zero()};
    bool fixation_valid{false};
    Real left_openness{1.0f};
    Real right_openness{1.0f};
};

/**
 * @brief Gaze target event data
 */
struct XRGazeTargetEventData {
    EntityId previous_target{INVALID_ENTITY_ID};
    EntityId new_target{INVALID_ENTITY_ID};
    Vec3 hit_point{Vec3::Zero()};
    Real dwell_time{0.0f};          ///< Time spent looking at target
};

/**
 * @brief Boundary event data
 */
struct XRBoundaryEventData {
    XRBoundaryWarning warning;
    XRTrackingEventData::Device device{XRTrackingEventData::Device::HMD};
    bool was_triggered{false};      ///< Previous state
    bool is_triggered{false};       ///< Current state
};

/**
 * @brief Passthrough event data
 */
struct XRPassthroughEventData {
    XRPassthroughMode old_mode{XRPassthroughMode::Disabled};
    XRPassthroughMode new_mode{XRPassthroughMode::Disabled};
    XRPassthroughConfig config;
};

// ============================================================================
// XR Event Variant Extension
// ============================================================================

/**
 * @brief Extended variant type including XR event data
 */
using XREventData = std::variant<
    std::monostate,
    XRSessionEventData,
    XRTrackingEventData,
    XRControllerButtonEventData,
    XRControllerAxisEventData,
    XRThumbstickEventData,
    XRHandTrackingEventData,
    XRGestureEventData,
    XRPinchEventData,
    XREyeTrackingEventData,
    XRGazeTargetEventData,
    XRBoundaryEventData,
    XRPassthroughEventData
>;

// ============================================================================
// XR Event Factory Functions
// ============================================================================

namespace events_factory {

/**
 * @brief Create session state changed event
 */
inline jaguar::events::Event create_session_state_changed(
    XRSessionState old_state, XRSessionState new_state, Real timestamp) {
    jaguar::events::Event event;
    event.base.type = to_event_type(XREventType::SessionStateChanged);
    event.base.priority = jaguar::events::EventPriority::High;
    event.base.timestamp = timestamp;

    XRSessionEventData data;
    data.old_state = old_state;
    data.new_state = new_state;
    data.is_running = (new_state == XRSessionState::Focused ||
                       new_state == XRSessionState::Visible);
    data.is_focused = (new_state == XRSessionState::Focused);
    data.is_visible = (new_state == XRSessionState::Visible ||
                       new_state == XRSessionState::Focused);

    // Store XR data in user event payload
    jaguar::events::UserEventData user_data;
    user_data.user_type_id = static_cast<UInt32>(XREventType::SessionStateChanged);
    user_data.user_type_name = "XRSessionEventData";
    user_data.payload = data;
    event.data = user_data;

    return event;
}

/**
 * @brief Create tracking lost event
 */
inline jaguar::events::Event create_tracking_lost(
    XRTrackingEventData::Device device, const XRPose& last_pose, Real timestamp) {
    jaguar::events::Event event;
    event.base.type = to_event_type(XREventType::TrackingLost);
    event.base.priority = jaguar::events::EventPriority::High;
    event.base.timestamp = timestamp;

    XRTrackingEventData data;
    data.device = device;
    data.old_confidence = TrackingConfidence::High;
    data.new_confidence = TrackingConfidence::None;
    data.last_known_pose = last_pose;

    jaguar::events::UserEventData user_data;
    user_data.user_type_id = static_cast<UInt32>(XREventType::TrackingLost);
    user_data.user_type_name = "XRTrackingEventData";
    user_data.payload = data;
    event.data = user_data;

    return event;
}

/**
 * @brief Create controller connected event
 */
inline jaguar::events::Event create_controller_connected(
    XRHand hand, Real timestamp) {
    jaguar::events::Event event;
    event.base.type = to_event_type(XREventType::ControllerConnected);
    event.base.priority = jaguar::events::EventPriority::Normal;
    event.base.timestamp = timestamp;

    XRControllerButtonEventData data;
    data.hand = hand;

    jaguar::events::UserEventData user_data;
    user_data.user_type_id = static_cast<UInt32>(XREventType::ControllerConnected);
    user_data.user_type_name = "XRControllerButtonEventData";
    user_data.payload = data;
    event.data = user_data;

    return event;
}

/**
 * @brief Create controller button pressed event
 */
inline jaguar::events::Event create_button_pressed(
    XRHand hand, XRButton button, Real analog_value, Real timestamp) {
    jaguar::events::Event event;
    event.base.type = to_event_type(XREventType::ControllerButtonPressed);
    event.base.priority = jaguar::events::EventPriority::Normal;
    event.base.timestamp = timestamp;

    XRControllerButtonEventData data;
    data.hand = hand;
    data.button = button;
    data.is_pressed = true;
    data.analog_value = analog_value;

    jaguar::events::UserEventData user_data;
    user_data.user_type_id = static_cast<UInt32>(XREventType::ControllerButtonPressed);
    user_data.user_type_name = "XRControllerButtonEventData";
    user_data.payload = data;
    event.data = user_data;

    return event;
}

/**
 * @brief Create thumbstick moved event
 */
inline jaguar::events::Event create_thumbstick_moved(
    XRHand hand, Real x, Real y, bool clicked, bool touched, Real timestamp) {
    jaguar::events::Event event;
    event.base.type = to_event_type(XREventType::ControllerThumbstickMoved);
    event.base.priority = jaguar::events::EventPriority::Low;
    event.base.timestamp = timestamp;

    XRThumbstickEventData data;
    data.hand = hand;
    data.x = x;
    data.y = y;
    data.magnitude = std::sqrt(x * x + y * y);
    data.angle = std::atan2(y, x);
    data.is_clicked = clicked;
    data.is_touched = touched;

    jaguar::events::UserEventData user_data;
    user_data.user_type_id = static_cast<UInt32>(XREventType::ControllerThumbstickMoved);
    user_data.user_type_name = "XRThumbstickEventData";
    user_data.payload = data;
    event.data = user_data;

    return event;
}

/**
 * @brief Create hand gesture detected event
 */
inline jaguar::events::Event create_gesture_detected(
    XRHand hand, XRHandGesture gesture, XRHandGesture previous,
    Real confidence, const Vec3& point, Real timestamp) {
    jaguar::events::Event event;
    event.base.type = to_event_type(XREventType::HandGestureDetected);
    event.base.priority = jaguar::events::EventPriority::Normal;
    event.base.timestamp = timestamp;

    XRGestureEventData data;
    data.hand = hand;
    data.gesture = gesture;
    data.previous_gesture = previous;
    data.confidence = confidence;
    data.gesture_point = point;

    jaguar::events::UserEventData user_data;
    user_data.user_type_id = static_cast<UInt32>(XREventType::HandGestureDetected);
    user_data.user_type_name = "XRGestureEventData";
    user_data.payload = data;
    event.data = user_data;

    return event;
}

/**
 * @brief Create pinch started event
 */
inline jaguar::events::Event create_pinch_started(
    XRHand hand, Real strength, const Vec3& point, Real timestamp) {
    jaguar::events::Event event;
    event.base.type = to_event_type(XREventType::PinchStarted);
    event.base.priority = jaguar::events::EventPriority::Normal;
    event.base.timestamp = timestamp;

    XRPinchEventData data;
    data.hand = hand;
    data.strength = strength;
    data.pinch_point = point;

    jaguar::events::UserEventData user_data;
    user_data.user_type_id = static_cast<UInt32>(XREventType::PinchStarted);
    user_data.user_type_name = "XRPinchEventData";
    user_data.payload = data;
    event.data = user_data;

    return event;
}

/**
 * @brief Create boundary approaching event
 */
inline jaguar::events::Event create_boundary_approaching(
    XRTrackingEventData::Device device, const XRBoundaryWarning& warning, Real timestamp) {
    jaguar::events::Event event;
    event.base.type = to_event_type(XREventType::BoundaryApproaching);
    event.base.priority = jaguar::events::EventPriority::High;
    event.base.timestamp = timestamp;

    XRBoundaryEventData data;
    data.device = device;
    data.warning = warning;
    data.was_triggered = false;
    data.is_triggered = true;

    jaguar::events::UserEventData user_data;
    user_data.user_type_id = static_cast<UInt32>(XREventType::BoundaryApproaching);
    user_data.user_type_name = "XRBoundaryEventData";
    user_data.payload = data;
    event.data = user_data;

    return event;
}

/**
 * @brief Create eye tracking gaze target changed event
 */
inline jaguar::events::Event create_gaze_target_changed(
    EntityId old_target, EntityId new_target, const Vec3& hit_point,
    Real dwell_time, Real timestamp) {
    jaguar::events::Event event;
    event.base.type = to_event_type(XREventType::GazeTargetChanged);
    event.base.priority = jaguar::events::EventPriority::Normal;
    event.base.timestamp = timestamp;

    XRGazeTargetEventData data;
    data.previous_target = old_target;
    data.new_target = new_target;
    data.hit_point = hit_point;
    data.dwell_time = dwell_time;

    jaguar::events::UserEventData user_data;
    user_data.user_type_id = static_cast<UInt32>(XREventType::GazeTargetChanged);
    user_data.user_type_name = "XRGazeTargetEventData";
    user_data.payload = data;
    event.data = user_data;

    return event;
}

/**
 * @brief Create passthrough started event
 */
inline jaguar::events::Event create_passthrough_started(
    const XRPassthroughConfig& config, Real timestamp) {
    jaguar::events::Event event;
    event.base.type = to_event_type(XREventType::PassthroughStarted);
    event.base.priority = jaguar::events::EventPriority::Normal;
    event.base.timestamp = timestamp;

    XRPassthroughEventData data;
    data.old_mode = XRPassthroughMode::Disabled;
    data.new_mode = config.mode;
    data.config = config;

    jaguar::events::UserEventData user_data;
    user_data.user_type_id = static_cast<UInt32>(XREventType::PassthroughStarted);
    user_data.user_type_name = "XRPassthroughEventData";
    user_data.payload = data;
    event.data = user_data;

    return event;
}

} // namespace events_factory

} // namespace jaguar::xr
