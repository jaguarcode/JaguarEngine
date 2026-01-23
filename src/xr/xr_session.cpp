/**
 * @file xr_session.cpp
 * @brief XR session management implementation
 */

#include "jaguar/xr/xr_session.h"
#include <cstring>
#include <algorithm>
#include <chrono>
#include <cmath>

namespace jaguar::xr {

// ============================================================================
// XRResult String Conversion
// ============================================================================

const char* xr_result_to_string(XRResult result) {
    switch (result) {
        case XRResult::Success: return "Success";
        case XRResult::NotInitialized: return "NotInitialized";
        case XRResult::RuntimeUnavailable: return "RuntimeUnavailable";
        case XRResult::SessionNotRunning: return "SessionNotRunning";
        case XRResult::SessionLost: return "SessionLost";
        case XRResult::FrameDiscarded: return "FrameDiscarded";
        case XRResult::InvalidParameter: return "InvalidParameter";
        case XRResult::DeviceNotConnected: return "DeviceNotConnected";
        case XRResult::FeatureNotSupported: return "FeatureNotSupported";
        case XRResult::TrackingLost: return "TrackingLost";
        case XRResult::CalibrationRequired: return "CalibrationRequired";
        case XRResult::PermissionDenied: return "PermissionDenied";
        case XRResult::OutOfMemory: return "OutOfMemory";
        case XRResult::InternalError: return "InternalError";
        default: return "Unknown";
    }
}

// ============================================================================
// XRPose Implementation
// ============================================================================

XRPose XRPose::lerp(const XRPose& a, const XRPose& b, Real t) {
    XRPose result;

    // Linear interpolation for position
    result.position = a.position + (b.position - a.position) * t;

    // Spherical linear interpolation for orientation
    // Simplified slerp implementation
    Real dot = a.orientation.w * b.orientation.w +
               a.orientation.x * b.orientation.x +
               a.orientation.y * b.orientation.y +
               a.orientation.z * b.orientation.z;

    // If dot product is negative, negate one quaternion to take shorter path
    Quat b_adj = b.orientation;
    if (dot < 0.0) {
        b_adj.w = -b_adj.w;
        b_adj.x = -b_adj.x;
        b_adj.y = -b_adj.y;
        b_adj.z = -b_adj.z;
        dot = -dot;
    }

    // If quaternions are very close, use linear interpolation
    if (dot > 0.9995) {
        result.orientation.w = a.orientation.w + (b_adj.w - a.orientation.w) * t;
        result.orientation.x = a.orientation.x + (b_adj.x - a.orientation.x) * t;
        result.orientation.y = a.orientation.y + (b_adj.y - a.orientation.y) * t;
        result.orientation.z = a.orientation.z + (b_adj.z - a.orientation.z) * t;
        result.orientation = result.orientation.normalized();
    } else {
        Real theta_0 = std::acos(dot);
        Real theta = theta_0 * t;
        Real sin_theta = std::sin(theta);
        Real sin_theta_0 = std::sin(theta_0);

        Real s0 = std::cos(theta) - dot * sin_theta / sin_theta_0;
        Real s1 = sin_theta / sin_theta_0;

        result.orientation.w = a.orientation.w * s0 + b_adj.w * s1;
        result.orientation.x = a.orientation.x * s0 + b_adj.x * s1;
        result.orientation.y = a.orientation.y * s0 + b_adj.y * s1;
        result.orientation.z = a.orientation.z * s0 + b_adj.z * s1;
    }

    // Take the higher confidence
    result.position_confidence = (a.position_confidence > b.position_confidence) ?
                                 a.position_confidence : b.position_confidence;
    result.orientation_confidence = (a.orientation_confidence > b.orientation_confidence) ?
                                    a.orientation_confidence : b.orientation_confidence;
    result.is_valid = a.is_valid && b.is_valid;

    return result;
}

// ============================================================================
// XRSession Implementation
// ============================================================================

class XRSession::Impl {
public:
    std::unique_ptr<IXRRuntime> runtime;
    XRSessionConfig config;
    XRRuntimeCapabilities capabilities;
    Statistics statistics;

    // Cached tracking states
    XRHeadState head_state;
    std::array<XRControllerState, 2> controller_states;
    std::array<XRHandState, 2> hand_states;
    XREyeTrackingState eye_tracking_state;

    // Frame state
    XRFrameTiming current_frame_timing;
    bool in_frame{false};
    bool should_render_{true};

    // Play area cache
    XRPlayArea play_area;

    // State callback
    SessionStateCallback state_callback;

    // Current refresh rate
    Real current_refresh_rate{90.0f};

    Impl() {
        // Initialize hand states with correct hand assignment
        hand_states[0].hand = XRHand::Left;
        hand_states[1].hand = XRHand::Right;
    }

    void update_tracking(Int64 predicted_time_ns) {
        if (!runtime || !runtime->is_initialized()) return;

        auto state = runtime->get_session_state();
        if (state != XRSessionState::Visible && state != XRSessionState::Focused) return;

        // Update head tracking
        runtime->get_head_state(head_state, predicted_time_ns);

        // Update controller tracking
        runtime->get_controller_state(XRHand::Left, controller_states[0], predicted_time_ns);
        runtime->get_controller_state(XRHand::Right, controller_states[1], predicted_time_ns);

        // Update hand tracking if enabled
        if (config.enable_hand_tracking &&
            capabilities.has_extension(XRExtension::HandTracking)) {
            runtime->get_hand_state(XRHand::Left, hand_states[0], predicted_time_ns);
            runtime->get_hand_state(XRHand::Right, hand_states[1], predicted_time_ns);
        }

        // Update eye tracking if enabled
        if (config.enable_eye_tracking &&
            capabilities.has_extension(XRExtension::EyeTracking)) {
            runtime->get_eye_tracking_state(eye_tracking_state, predicted_time_ns);
        }
    }
};

XRSession::XRSession(std::unique_ptr<IXRRuntime> runtime)
    : impl_(std::make_unique<Impl>()) {
    impl_->runtime = std::move(runtime);
}

XRSession::XRSession()
    : impl_(std::make_unique<Impl>()) {
    impl_->runtime = create_openxr_runtime();
}

XRSession::~XRSession() {
    if (impl_ && impl_->runtime && impl_->runtime->is_initialized()) {
        shutdown();
    }
}

XRSession::XRSession(XRSession&&) noexcept = default;
XRSession& XRSession::operator=(XRSession&&) noexcept = default;

// ============================================================================
// Lifecycle
// ============================================================================

XRResult XRSession::initialize(const XRSessionConfig& config) {
    if (!impl_->runtime) {
        impl_->runtime = create_openxr_runtime();
        if (!impl_->runtime) {
            return XRResult::RuntimeUnavailable;
        }
    }

    impl_->config = config;

    // Initialize runtime
    auto result = impl_->runtime->initialize(config);
    if (!xr_succeeded(result)) {
        return result;
    }

    // Cache capabilities
    impl_->capabilities = impl_->runtime->get_capabilities();

    // Set up state callback
    impl_->runtime->set_session_state_callback(
        [this](XRSessionState old_state, XRSessionState new_state) {
            if (impl_->state_callback) {
                impl_->state_callback(old_state, new_state);
            }
        });

    // Begin session
    result = impl_->runtime->begin_session();
    if (!xr_succeeded(result)) {
        impl_->runtime->shutdown();
        return result;
    }

    // Initialize play area
    impl_->runtime->get_play_area(impl_->play_area);

    // Set initial refresh rate
    impl_->runtime->get_current_refresh_rate(impl_->current_refresh_rate);

    // Initialize statistics
    impl_->statistics = Statistics{};
    impl_->statistics.session_start_time_ns =
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();

    return XRResult::Success;
}

XRResult XRSession::shutdown() {
    if (!impl_->runtime || !impl_->runtime->is_initialized()) {
        return XRResult::NotInitialized;
    }

    auto result = impl_->runtime->end_session();
    if (!xr_succeeded(result)) {
        // Continue with shutdown even if end_session fails
    }

    return impl_->runtime->shutdown();
}

bool XRSession::is_initialized() const {
    return impl_->runtime && impl_->runtime->is_initialized();
}

bool XRSession::is_running() const {
    if (!is_initialized()) return false;
    auto state = impl_->runtime->get_session_state();
    return state == XRSessionState::Visible || state == XRSessionState::Focused;
}

XRSessionState XRSession::get_state() const {
    if (!impl_->runtime || !impl_->runtime->is_initialized()) {
        return XRSessionState::Unknown;
    }
    return impl_->runtime->get_session_state();
}

const XRRuntimeCapabilities& XRSession::get_capabilities() const {
    return impl_->capabilities;
}

void XRSession::set_state_callback(SessionStateCallback callback) {
    impl_->state_callback = std::move(callback);
}

// ============================================================================
// Frame Management
// ============================================================================

XRResult XRSession::begin_frame(XRFrameTiming& timing) {
    if (!is_running()) {
        return XRResult::SessionNotRunning;
    }

    if (impl_->in_frame) {
        return XRResult::InvalidParameter;
    }

    // Wait for optimal frame timing
    auto result = impl_->runtime->wait_frame(impl_->current_frame_timing);
    if (!xr_succeeded(result)) {
        return result;
    }

    impl_->should_render_ = impl_->current_frame_timing.should_render;
    timing = impl_->current_frame_timing;

    // Begin frame
    result = impl_->runtime->begin_frame();
    if (!xr_succeeded(result)) {
        return result;
    }

    impl_->in_frame = true;

    // Update tracking data for predicted display time
    impl_->update_tracking(timing.predicted_display_time_ns);

    return XRResult::Success;
}

XRFrameResult XRSession::end_frame() {
    XRFrameResult frame_result;

    if (!impl_->in_frame) {
        frame_result.result = XRResult::InvalidParameter;
        return frame_result;
    }

    auto result = impl_->runtime->end_frame(frame_result);
    frame_result.result = result;

    impl_->in_frame = false;

    // Update statistics
    impl_->statistics.frames_rendered++;
    if (!impl_->should_render_) {
        impl_->statistics.frames_dropped++;
    }
    impl_->statistics.compositor_latency_ms = frame_result.compositor_latency_ms;

    return frame_result;
}

bool XRSession::should_render() const {
    return impl_->should_render_;
}

// ============================================================================
// Tracking Access
// ============================================================================

const XRHeadState& XRSession::get_head_state(Int64 predicted_time_ns) {
    if (predicted_time_ns != 0 && is_running()) {
        impl_->runtime->get_head_state(impl_->head_state, predicted_time_ns);
    }
    return impl_->head_state;
}

const XRControllerState& XRSession::get_controller_state(XRHand hand, Int64 predicted_time_ns) {
    auto index = static_cast<size_t>(hand);
    if (predicted_time_ns != 0 && is_running()) {
        impl_->runtime->get_controller_state(hand, impl_->controller_states[index], predicted_time_ns);
    }
    return impl_->controller_states[index];
}

const XRHandState& XRSession::get_hand_state(XRHand hand, Int64 predicted_time_ns) {
    auto index = static_cast<size_t>(hand);
    if (predicted_time_ns != 0 && is_running() && impl_->config.enable_hand_tracking) {
        impl_->runtime->get_hand_state(hand, impl_->hand_states[index], predicted_time_ns);
    }
    return impl_->hand_states[index];
}

const XREyeTrackingState& XRSession::get_eye_tracking_state(Int64 predicted_time_ns) {
    if (predicted_time_ns != 0 && is_running() && impl_->config.enable_eye_tracking) {
        impl_->runtime->get_eye_tracking_state(impl_->eye_tracking_state, predicted_time_ns);
    }
    return impl_->eye_tracking_state;
}

bool XRSession::is_hand_tracking_active(XRHand hand) const {
    auto index = static_cast<size_t>(hand);
    return impl_->hand_states[index].is_active;
}

bool XRSession::is_controller_connected(XRHand hand) const {
    auto index = static_cast<size_t>(hand);
    return impl_->controller_states[index].is_connected;
}

bool XRSession::is_eye_tracking_active() const {
    return impl_->eye_tracking_state.combined_gaze.is_valid;
}

// ============================================================================
// Input Helpers
// ============================================================================

XRPose XRSession::get_interaction_pose(XRHand hand) const {
    auto index = static_cast<size_t>(hand);

    // Prefer controller aim pose if connected
    if (impl_->controller_states[index].is_connected &&
        impl_->controller_states[index].aim_pose.pose.is_valid) {
        return impl_->controller_states[index].aim_pose.pose;
    }

    // Fall back to hand tracking pinch point
    if (impl_->hand_states[index].is_active) {
        XRPose pose;
        pose.position = impl_->hand_states[index].pinch_point;
        pose.orientation = impl_->hand_states[index].palm_orientation();
        pose.is_valid = true;
        pose.position_confidence = impl_->hand_states[index].confidence;
        pose.orientation_confidence = impl_->hand_states[index].confidence;
        return pose;
    }

    return XRPose{};
}

void XRSession::get_interaction_ray(XRHand hand, Vec3& origin, Vec3& direction) const {
    XRPose pose = get_interaction_pose(hand);
    origin = pose.position;
    direction = pose.forward();
}

bool XRSession::is_trigger_pressed(XRHand hand, Real threshold) const {
    auto index = static_cast<size_t>(hand);
    return impl_->controller_states[index].trigger() >= threshold;
}

bool XRSession::is_grip_pressed(XRHand hand, Real threshold) const {
    auto index = static_cast<size_t>(hand);
    return impl_->controller_states[index].grip() >= threshold;
}

bool XRSession::is_pinching(XRHand hand, Real threshold) const {
    auto index = static_cast<size_t>(hand);
    return impl_->hand_states[index].is_active &&
           impl_->hand_states[index].pinch_strength >= threshold;
}

// ============================================================================
// Haptic Feedback
// ============================================================================

XRResult XRSession::vibrate(XRHand hand, Real amplitude, Real duration, Real frequency) {
    if (!is_running()) {
        return XRResult::SessionNotRunning;
    }

    XRHapticFeedback feedback;
    feedback.amplitude = std::clamp(amplitude, static_cast<Real>(0.0), static_cast<Real>(1.0));
    feedback.duration_seconds = duration;
    feedback.frequency = frequency;

    auto result = impl_->runtime->send_haptic_feedback(hand, feedback);
    if (xr_succeeded(result)) {
        impl_->statistics.haptic_events_sent++;
    }
    return result;
}

XRResult XRSession::stop_vibration(XRHand hand) {
    if (!is_running()) {
        return XRResult::SessionNotRunning;
    }
    return impl_->runtime->stop_haptic_feedback(hand);
}

// ============================================================================
// Configuration
// ============================================================================

XRResult XRSession::set_passthrough(const XRPassthroughConfig& config) {
    if (!is_initialized()) {
        return XRResult::NotInitialized;
    }
    if (!impl_->capabilities.supports_passthrough) {
        return XRResult::FeatureNotSupported;
    }
    return impl_->runtime->set_passthrough_config(config);
}

XRResult XRSession::enable_passthrough(bool enable, Real opacity) {
    XRPassthroughConfig config;
    config.mode = enable ? XRPassthroughMode::FullPassthrough : XRPassthroughMode::Disabled;
    config.opacity = opacity;
    return set_passthrough(config);
}

XRResult XRSession::set_foveation(const XRFoveationConfig& config) {
    if (!is_initialized()) {
        return XRResult::NotInitialized;
    }
    if (!impl_->capabilities.supports_foveated_rendering &&
        config.level != XRFoveationLevel::None) {
        return XRResult::FeatureNotSupported;
    }
    return impl_->runtime->set_foveation_config(config);
}

XRResult XRSession::set_refresh_rate(Real rate) {
    if (!is_initialized()) {
        return XRResult::NotInitialized;
    }

    // Check if rate is supported
    const auto& rates = impl_->capabilities.supported_refresh_rates;
    if (!rates.empty()) {
        bool found = false;
        for (Real supported : rates) {
            if (std::abs(supported - rate) < 0.1f) {
                found = true;
                break;
            }
        }
        if (!found) {
            return XRResult::InvalidParameter;
        }
    }

    auto result = impl_->runtime->set_refresh_rate(rate);
    if (xr_succeeded(result)) {
        impl_->current_refresh_rate = rate;
    }
    return result;
}

Real XRSession::get_refresh_rate() const {
    return impl_->current_refresh_rate;
}

XRResult XRSession::recenter() {
    if (!is_running()) {
        return XRResult::SessionNotRunning;
    }
    return impl_->runtime->recenter();
}

// ============================================================================
// Boundary System
// ============================================================================

const XRPlayArea& XRSession::get_play_area() const {
    return impl_->play_area;
}

XRBoundaryWarning XRSession::check_boundary(const Vec3& point) const {
    XRBoundaryWarning warning;
    if (is_running() && impl_->config.enable_guardian) {
        impl_->runtime->check_boundary_proximity(point, warning);
    }
    return warning;
}

XRBoundaryWarning XRSession::check_devices_boundary() const {
    XRBoundaryWarning worst_warning;

    if (!is_running() || !impl_->config.enable_guardian) {
        return worst_warning;
    }

    // Check head position
    if (impl_->head_state.head_pose.pose.is_valid) {
        XRBoundaryWarning head_warning;
        impl_->runtime->check_boundary_proximity(
            impl_->head_state.head_pose.pose.position, head_warning);
        if (head_warning.distance < worst_warning.distance || !worst_warning.is_triggered) {
            worst_warning = head_warning;
        }
    }

    // Check controller positions
    for (size_t i = 0; i < 2; ++i) {
        if (impl_->controller_states[i].is_connected &&
            impl_->controller_states[i].pose.pose.is_valid) {
            XRBoundaryWarning ctrl_warning;
            impl_->runtime->check_boundary_proximity(
                impl_->controller_states[i].pose.pose.position, ctrl_warning);
            if (ctrl_warning.distance < worst_warning.distance || !worst_warning.is_triggered) {
                worst_warning = ctrl_warning;
            }
        }
    }

    // Check hand positions
    for (size_t i = 0; i < 2; ++i) {
        if (impl_->hand_states[i].is_active) {
            XRBoundaryWarning hand_warning;
            impl_->runtime->check_boundary_proximity(
                impl_->hand_states[i].palm_position(), hand_warning);
            if (hand_warning.distance < worst_warning.distance || !worst_warning.is_triggered) {
                worst_warning = hand_warning;
            }
        }
    }

    return worst_warning;
}

// ============================================================================
// Entity Integration
// ============================================================================

void XRSession::sync_head_to_entity(EntityId entity_id) {
    // TODO: Integrate with entity system when available
    (void)entity_id;
}

void XRSession::sync_controller_to_entity(EntityId entity_id, XRHand hand) {
    // TODO: Integrate with entity system when available
    (void)entity_id;
    (void)hand;
}

void XRSession::sync_hand_to_entity(EntityId entity_id, XRHand hand) {
    // TODO: Integrate with entity system when available
    (void)entity_id;
    (void)hand;
}

// ============================================================================
// Statistics
// ============================================================================

const XRSession::Statistics& XRSession::get_statistics() const {
    return impl_->statistics;
}

void XRSession::reset_statistics() {
    impl_->statistics = Statistics{};
    impl_->statistics.session_start_time_ns =
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
}

// ============================================================================
// Runtime Factory - OpenXR Availability Check
// ============================================================================

bool is_openxr_available() {
#ifdef JAGUAR_HAS_OPENXR
    // Try to load OpenXR loader dynamically
    // This will be implemented when OpenXR is linked
    return true;
#else
    return false;
#endif
}

std::unique_ptr<IXRRuntime> create_openxr_runtime() {
#ifdef JAGUAR_HAS_OPENXR
    // Return real OpenXR runtime implementation
    // Will be implemented when OpenXR is linked
    return nullptr;
#else
    // OpenXR not available, return nullptr
    return nullptr;
#endif
}

} // namespace jaguar::xr
