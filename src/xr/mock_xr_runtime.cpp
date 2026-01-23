/**
 * @file mock_xr_runtime.cpp
 * @brief Mock XR runtime implementation for testing
 *
 * Provides a simulated XR runtime that can be used for testing and development
 * without requiring actual XR hardware. Supports configurable tracking data,
 * input simulation, and state transitions.
 */

#include "jaguar/xr/xr_session.h"
#include <algorithm>
#include <cmath>
#include <chrono>
#include <random>

namespace jaguar::xr {

/**
 * @brief Mock XR runtime for testing
 *
 * Simulates XR hardware behavior with configurable tracking data and input states.
 * Useful for unit testing, CI/CD pipelines, and development without hardware.
 */
class MockXRRuntime : public IXRRuntime {
public:
    MockXRRuntime() {
        // Initialize capabilities with simulated hardware
        capabilities_.runtime_name = "JaguarEngine Mock XR Runtime";
        capabilities_.runtime_version = "1.0.0";
        capabilities_.system_name = "Mock HMD";

        capabilities_.supported_extensions =
            XRExtension::HandTracking |
            XRExtension::EyeTracking |
            XRExtension::FoveatedRendering |
            XRExtension::Passthrough |
            XRExtension::ControllerInteraction |
            XRExtension::HandInteraction;

        capabilities_.supported_form_factor = XRFormFactor::HeadMountedDisplay;
        capabilities_.supported_blend_modes = {XRBlendMode::Opaque, XRBlendMode::AlphaBlend};
        capabilities_.supported_view_configs = {XRViewConfigType::Stereo};

        capabilities_.supports_hand_tracking = true;
        capabilities_.supports_eye_tracking = true;
        capabilities_.supports_foveated_rendering = true;
        capabilities_.supports_passthrough = true;

        capabilities_.max_refresh_rate = 120.0f;
        capabilities_.supported_refresh_rates = {72.0f, 80.0f, 90.0f, 120.0f};
        capabilities_.max_swapchain_width = 4096;
        capabilities_.max_swapchain_height = 4096;
        capabilities_.recommended_swapchain_width = 1920;
        capabilities_.recommended_swapchain_height = 1920;

        // Set up default play area
        capabilities_.play_area.type = XRBoundaryType::RoomScale;
        capabilities_.play_area.width = 2.5f;
        capabilities_.play_area.depth = 2.5f;
        capabilities_.play_area.center = Vec3::Zero();
        capabilities_.play_area.is_configured = true;
        capabilities_.play_area.boundary_points = {
            Vec3{-1.25f, 0.0f, -1.25f},
            Vec3{1.25f, 0.0f, -1.25f},
            Vec3{1.25f, 0.0f, 1.25f},
            Vec3{-1.25f, 0.0f, 1.25f}
        };

        // Initialize random number generator for noise
        std::random_device rd;
        rng_.seed(rd());
    }

    ~MockXRRuntime() override = default;

    // ========================================================================
    // Lifecycle
    // ========================================================================

    XRResult initialize(const XRSessionConfig& config) override {
        if (initialized_) {
            return XRResult::InvalidParameter;
        }

        config_ = config;
        initialized_ = true;
        session_state_ = XRSessionState::Idle;

        // Set up initial tracking states
        initialize_tracking_states();

        return XRResult::Success;
    }

    XRResult shutdown() override {
        if (!initialized_) {
            return XRResult::NotInitialized;
        }

        initialized_ = false;
        session_running_ = false;
        session_state_ = XRSessionState::Unknown;

        return XRResult::Success;
    }

    bool is_initialized() const override {
        return initialized_;
    }

    const XRRuntimeCapabilities& get_capabilities() const override {
        return capabilities_;
    }

    // ========================================================================
    // Session Management
    // ========================================================================

    XRResult begin_session() override {
        if (!initialized_) {
            return XRResult::NotInitialized;
        }

        session_running_ = true;
        transition_state(XRSessionState::Ready);
        transition_state(XRSessionState::Synchronized);
        transition_state(XRSessionState::Visible);
        transition_state(XRSessionState::Focused);

        return XRResult::Success;
    }

    XRResult end_session() override {
        if (!session_running_) {
            return XRResult::SessionNotRunning;
        }

        transition_state(XRSessionState::Stopping);
        session_running_ = false;
        transition_state(XRSessionState::Idle);

        return XRResult::Success;
    }

    XRSessionState get_session_state() const override {
        return session_state_;
    }

    void set_session_state_callback(SessionStateCallback callback) override {
        state_callback_ = std::move(callback);
    }

    // ========================================================================
    // Frame Timing
    // ========================================================================

    XRResult wait_frame(XRFrameTiming& timing) override {
        if (!session_running_) {
            return XRResult::SessionNotRunning;
        }

        // Simulate frame timing based on refresh rate
        auto now = std::chrono::steady_clock::now();
        auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
            now.time_since_epoch()).count();

        Real frame_period = 1.0f / current_refresh_rate_;
        timing.predicted_display_time_ns = now_ns +
            static_cast<Int64>(frame_period * 1e9f);
        timing.predicted_display_period_ns = static_cast<Int64>(frame_period * 1e9f);
        timing.should_render = true;

        frame_number_++;

        return XRResult::Success;
    }

    XRResult begin_frame() override {
        if (!session_running_) {
            return XRResult::SessionNotRunning;
        }
        in_frame_ = true;
        return XRResult::Success;
    }

    XRResult end_frame(const XRFrameResult& result) override {
        if (!in_frame_) {
            return XRResult::InvalidParameter;
        }
        (void)result;
        in_frame_ = false;
        return XRResult::Success;
    }

    // ========================================================================
    // Tracking - HMD
    // ========================================================================

    XRResult get_head_state(XRHeadState& state, Int64 predicted_time_ns) override {
        if (!session_running_) {
            return XRResult::SessionNotRunning;
        }

        // Apply simulated motion
        update_simulated_motion(predicted_time_ns);

        state = head_state_;
        state.head_pose.timestamp_ns = predicted_time_ns;

        return XRResult::Success;
    }

    // ========================================================================
    // Tracking - Controllers
    // ========================================================================

    XRResult get_controller_state(XRHand hand, XRControllerState& state,
                                  Int64 predicted_time_ns) override {
        if (!session_running_) {
            return XRResult::SessionNotRunning;
        }

        auto index = static_cast<size_t>(hand);
        state = controller_states_[index];
        state.pose.timestamp_ns = predicted_time_ns;
        state.aim_pose.timestamp_ns = predicted_time_ns;
        state.grip_pose.timestamp_ns = predicted_time_ns;

        return XRResult::Success;
    }

    XRResult send_haptic_feedback(XRHand hand, const XRHapticFeedback& feedback) override {
        if (!session_running_) {
            return XRResult::SessionNotRunning;
        }
        (void)hand;
        (void)feedback;
        // Mock runtime just acknowledges haptic feedback
        return XRResult::Success;
    }

    XRResult stop_haptic_feedback(XRHand hand) override {
        if (!session_running_) {
            return XRResult::SessionNotRunning;
        }
        (void)hand;
        return XRResult::Success;
    }

    // ========================================================================
    // Tracking - Hands
    // ========================================================================

    XRResult get_hand_state(XRHand hand, XRHandState& state,
                           Int64 predicted_time_ns) override {
        if (!session_running_) {
            return XRResult::SessionNotRunning;
        }

        auto index = static_cast<size_t>(hand);
        state = hand_states_[index];
        (void)predicted_time_ns;

        return XRResult::Success;
    }

    // ========================================================================
    // Tracking - Eye
    // ========================================================================

    XRResult get_eye_tracking_state(XREyeTrackingState& state,
                                    Int64 predicted_time_ns) override {
        if (!session_running_) {
            return XRResult::SessionNotRunning;
        }

        state = eye_tracking_state_;
        state.timestamp_ns = predicted_time_ns;

        return XRResult::Success;
    }

    // ========================================================================
    // Passthrough
    // ========================================================================

    XRResult set_passthrough_config(const XRPassthroughConfig& config) override {
        if (!initialized_) {
            return XRResult::NotInitialized;
        }
        passthrough_config_ = config;
        return XRResult::Success;
    }

    XRResult get_passthrough_config(XRPassthroughConfig& config) const override {
        config = passthrough_config_;
        return XRResult::Success;
    }

    // ========================================================================
    // Foveation
    // ========================================================================

    XRResult set_foveation_config(const XRFoveationConfig& config) override {
        if (!initialized_) {
            return XRResult::NotInitialized;
        }
        foveation_config_ = config;
        return XRResult::Success;
    }

    XRResult get_foveation_config(XRFoveationConfig& config) const override {
        config = foveation_config_;
        return XRResult::Success;
    }

    // ========================================================================
    // Boundary
    // ========================================================================

    XRResult get_play_area(XRPlayArea& area) const override {
        area = capabilities_.play_area;
        return XRResult::Success;
    }

    XRResult check_boundary_proximity(const Vec3& point, XRBoundaryWarning& warning) override {
        // Simple boundary check against rectangular area
        Real half_width = capabilities_.play_area.width / 2.0;
        Real half_depth = capabilities_.play_area.depth / 2.0;

        Real abs_x = std::abs(point.x);
        Real abs_z = std::abs(point.z);

        Real dist_x = (abs_x > half_width) ? (abs_x - half_width) : static_cast<Real>(0.0);
        Real dist_z = (abs_z > half_depth) ? (abs_z - half_depth) : static_cast<Real>(0.0);
        Real distance = std::sqrt(dist_x * dist_x + dist_z * dist_z);

        Real dist_from_x_boundary = half_width - abs_x;
        Real dist_from_z_boundary = half_depth - abs_z;
        warning.distance = (dist_from_x_boundary > 0.0) ? dist_from_x_boundary : static_cast<Real>(0.0);
        Real dist_to_z = (dist_from_z_boundary > 0.0) ? dist_from_z_boundary : static_cast<Real>(0.0);
        if (dist_to_z < warning.distance) {
            warning.distance = dist_to_z;
        }

        warning.is_triggered = (warning.distance < 0.5);

        // Find closest point on boundary
        Real closest_x = (point.x < -half_width) ? -half_width :
                        ((point.x > half_width) ? half_width : point.x);
        Real closest_z = (point.z < -half_depth) ? -half_depth :
                        ((point.z > half_depth) ? half_depth : point.z);
        warning.closest_point = Vec3{closest_x, point.y, closest_z};

        // Normal pointing inward
        if (abs_x > abs_z) {
            warning.normal = Vec3{(point.x > 0.0) ? -1.0 : 1.0, 0.0, 0.0};
        } else {
            warning.normal = Vec3{0.0, 0.0, (point.z > 0.0) ? -1.0 : 1.0};
        }

        (void)distance;  // Used for rectangular calculation
        return XRResult::Success;
    }

    // ========================================================================
    // Performance
    // ========================================================================

    XRResult set_refresh_rate(Real rate) override {
        if (!initialized_) {
            return XRResult::NotInitialized;
        }
        current_refresh_rate_ = rate;
        return XRResult::Success;
    }

    XRResult get_current_refresh_rate(Real& rate) const override {
        rate = current_refresh_rate_;
        return XRResult::Success;
    }

    // ========================================================================
    // Reference Space
    // ========================================================================

    XRResult set_reference_space(XRSpaceType space) override {
        if (!initialized_) {
            return XRResult::NotInitialized;
        }
        reference_space_ = space;
        return XRResult::Success;
    }

    XRResult get_reference_space(XRSpaceType& space) const override {
        space = reference_space_;
        return XRResult::Success;
    }

    XRResult recenter() override {
        if (!session_running_) {
            return XRResult::SessionNotRunning;
        }

        // Reset head position to origin
        head_state_.head_pose.pose.position = Vec3::Zero();
        head_state_.head_pose.pose.orientation = Quat::Identity();

        return XRResult::Success;
    }

    // ========================================================================
    // Test Helpers - For programmatic control in tests
    // ========================================================================

    /**
     * @brief Set head position for testing
     */
    void set_head_position(const Vec3& position) {
        head_state_.head_pose.pose.position = position;
    }

    /**
     * @brief Set head orientation for testing
     */
    void set_head_orientation(const Quat& orientation) {
        head_state_.head_pose.pose.orientation = orientation;
    }

    /**
     * @brief Set controller position for testing
     */
    void set_controller_position(XRHand hand, const Vec3& position) {
        auto index = static_cast<size_t>(hand);
        controller_states_[index].pose.pose.position = position;
        controller_states_[index].aim_pose.pose.position = position;
        controller_states_[index].grip_pose.pose.position = position;
    }

    /**
     * @brief Set controller button state for testing
     */
    void set_button_pressed(XRHand hand, XRButton button, bool pressed) {
        auto index = static_cast<size_t>(hand);
        controller_states_[index].buttons_pressed[static_cast<size_t>(button)] = pressed;
    }

    /**
     * @brief Set controller axis value for testing
     */
    void set_axis_value(XRHand hand, XRAxis axis, Real value) {
        auto index = static_cast<size_t>(hand);
        controller_states_[index].axes[static_cast<size_t>(axis)] = value;
    }

    /**
     * @brief Set hand tracking active for testing
     */
    void set_hand_tracking_active(XRHand hand, bool active) {
        auto index = static_cast<size_t>(hand);
        hand_states_[index].is_active = active;
        hand_states_[index].confidence = active ? TrackingConfidence::High : TrackingConfidence::None;
    }

    /**
     * @brief Set pinch strength for testing
     */
    void set_pinch_strength(XRHand hand, Real strength) {
        auto index = static_cast<size_t>(hand);
        hand_states_[index].pinch_strength = strength;
    }

    /**
     * @brief Simulate tracking loss for testing
     */
    void simulate_tracking_loss() {
        head_state_.head_pose.pose.is_valid = false;
        head_state_.head_pose.pose.position_confidence = TrackingConfidence::None;
        head_state_.head_pose.pose.orientation_confidence = TrackingConfidence::None;
    }

    /**
     * @brief Simulate tracking recovery for testing
     */
    void simulate_tracking_recovery() {
        head_state_.head_pose.pose.is_valid = true;
        head_state_.head_pose.pose.position_confidence = TrackingConfidence::High;
        head_state_.head_pose.pose.orientation_confidence = TrackingConfidence::High;
    }

    /**
     * @brief Simulate session loss for testing
     */
    void simulate_session_loss() {
        transition_state(XRSessionState::LossPending);
        session_running_ = false;
    }

    /**
     * @brief Enable automatic simulated motion
     */
    void enable_simulated_motion(bool enable) {
        simulate_motion_ = enable;
    }

private:
    void transition_state(XRSessionState new_state) {
        XRSessionState old_state = session_state_;
        session_state_ = new_state;
        if (state_callback_) {
            state_callback_(old_state, new_state);
        }
    }

    void initialize_tracking_states() {
        // Initialize head state
        head_state_.head_pose.pose.position = Vec3{0.0f, 1.6f, 0.0f}; // Standing height
        head_state_.head_pose.pose.orientation = Quat::Identity();
        head_state_.head_pose.pose.is_valid = true;
        head_state_.head_pose.pose.position_confidence = TrackingConfidence::High;
        head_state_.head_pose.pose.orientation_confidence = TrackingConfidence::High;
        head_state_.is_user_present = true;
        head_state_.ipd = 0.063f;

        // Initialize view states for stereo
        Real half_ipd = head_state_.ipd / 2.0f;
        head_state_.views[0].pose.position = Vec3{-half_ipd, 0.0f, 0.0f};
        head_state_.views[0].pose.orientation = Quat::Identity();
        head_state_.views[0].pose.is_valid = true;
        head_state_.views[0].pose.position_confidence = TrackingConfidence::High;
        head_state_.views[0].pose.orientation_confidence = TrackingConfidence::High;
        head_state_.views[0].fov_left = -0.8f;
        head_state_.views[0].fov_right = 0.7f;
        head_state_.views[0].fov_up = 0.8f;
        head_state_.views[0].fov_down = -0.8f;

        head_state_.views[1].pose.position = Vec3{half_ipd, 0.0f, 0.0f};
        head_state_.views[1].pose.orientation = Quat::Identity();
        head_state_.views[1].pose.is_valid = true;
        head_state_.views[1].pose.position_confidence = TrackingConfidence::High;
        head_state_.views[1].pose.orientation_confidence = TrackingConfidence::High;
        head_state_.views[1].fov_left = -0.7f;
        head_state_.views[1].fov_right = 0.8f;
        head_state_.views[1].fov_up = 0.8f;
        head_state_.views[1].fov_down = -0.8f;

        // Initialize controller states
        for (size_t i = 0; i < 2; ++i) {
            Real x_offset = (i == 0) ? -0.3f : 0.3f; // Left or right
            controller_states_[i].pose.pose.position = Vec3{x_offset, 1.0f, -0.3f};
            controller_states_[i].pose.pose.orientation = Quat::Identity();
            controller_states_[i].pose.pose.is_valid = true;
            controller_states_[i].pose.pose.position_confidence = TrackingConfidence::High;
            controller_states_[i].pose.pose.orientation_confidence = TrackingConfidence::High;

            controller_states_[i].aim_pose = controller_states_[i].pose;
            controller_states_[i].grip_pose = controller_states_[i].pose;

            controller_states_[i].is_connected = true;
            controller_states_[i].is_active = true;
        }

        // Initialize hand states
        for (size_t i = 0; i < 2; ++i) {
            hand_states_[i].hand = static_cast<XRHand>(i);
            hand_states_[i].is_active = false; // Disabled by default
            hand_states_[i].confidence = TrackingConfidence::None;

            Real x_offset = (i == 0) ? -0.25f : 0.25f;
            Vec3 palm_pos{x_offset, 1.0f, -0.3f};

            // Initialize all joints with default poses
            for (size_t j = 0; j < static_cast<size_t>(XRHandJoint::Count); ++j) {
                hand_states_[i].joints[j].pose.position = palm_pos;
                hand_states_[i].joints[j].pose.orientation = Quat::Identity();
                hand_states_[i].joints[j].pose.is_valid = false;
                hand_states_[i].joints[j].radius = 0.01f;
            }

            hand_states_[i].pinch_point = palm_pos;
            hand_states_[i].detected_gesture = XRHandGesture::None;
        }

        // Initialize eye tracking state
        eye_tracking_state_.combined_gaze.position = head_state_.head_pose.pose.position;
        eye_tracking_state_.combined_gaze.orientation = head_state_.head_pose.pose.orientation;
        eye_tracking_state_.combined_gaze.is_valid = true;
        eye_tracking_state_.combined_gaze.position_confidence = TrackingConfidence::High;
        eye_tracking_state_.combined_gaze.orientation_confidence = TrackingConfidence::High;

        eye_tracking_state_.left_eye.gaze_pose = eye_tracking_state_.combined_gaze;
        eye_tracking_state_.left_eye.is_valid = true;
        eye_tracking_state_.left_eye.openness = 1.0f;

        eye_tracking_state_.right_eye.gaze_pose = eye_tracking_state_.combined_gaze;
        eye_tracking_state_.right_eye.is_valid = true;
        eye_tracking_state_.right_eye.openness = 1.0f;

        eye_tracking_state_.fixation_point = Vec3{0.0f, 1.6f, -2.0f};
        eye_tracking_state_.fixation_valid = true;
    }

    void update_simulated_motion(Int64 predicted_time_ns) {
        if (!simulate_motion_) return;

        // Simulate gentle head motion
        Real time_sec = predicted_time_ns / 1e9f;
        Real yaw_amplitude = 0.1f;
        Real pitch_amplitude = 0.05f;

        Real yaw = std::sin(time_sec * 0.5f) * yaw_amplitude;
        Real pitch = std::sin(time_sec * 0.7f) * pitch_amplitude;

        head_state_.head_pose.pose.orientation =
            Quat::from_euler(pitch, yaw, 0.0f);

        // Add subtle position noise
        std::normal_distribution<Real> noise(0.0f, 0.001f);
        head_state_.head_pose.pose.position.x += noise(rng_);
        head_state_.head_pose.pose.position.y += noise(rng_);
        head_state_.head_pose.pose.position.z += noise(rng_);
    }

private:
    bool initialized_{false};
    bool session_running_{false};
    bool in_frame_{false};
    bool simulate_motion_{false};
    UInt64 frame_number_{0};

    XRSessionConfig config_;
    XRRuntimeCapabilities capabilities_;
    XRSessionState session_state_{XRSessionState::Unknown};
    SessionStateCallback state_callback_;

    // Tracking states
    XRHeadState head_state_;
    std::array<XRControllerState, 2> controller_states_;
    std::array<XRHandState, 2> hand_states_;
    XREyeTrackingState eye_tracking_state_;

    // Configuration
    XRPassthroughConfig passthrough_config_;
    XRFoveationConfig foveation_config_;
    XRSpaceType reference_space_{XRSpaceType::Local};
    Real current_refresh_rate_{90.0f};

    // Random number generator for noise
    std::mt19937 rng_;
};

// ============================================================================
// Factory Function
// ============================================================================

std::unique_ptr<IXRRuntime> create_mock_xr_runtime() {
    return std::make_unique<MockXRRuntime>();
}

} // namespace jaguar::xr
