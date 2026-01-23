/**
 * @file xr_session_tests.cpp
 * @brief Unit tests for XR session management and mock runtime
 */

#include <gtest/gtest.h>
#include "jaguar/xr/xr_session.h"
#include "jaguar/xr/xr_events.h"
#include <thread>
#include <chrono>

namespace jaguar::xr {

// ============================================================================
// Test Fixtures
// ============================================================================

class XRSessionTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Create session with mock runtime
        session_ = std::make_unique<XRSession>(create_mock_xr_runtime());
    }

    void TearDown() override {
        if (session_ && session_->is_initialized()) {
            session_->shutdown();
        }
    }

    std::unique_ptr<XRSession> session_;
};

class XRSessionInitializedTest : public XRSessionTest {
protected:
    void SetUp() override {
        XRSessionTest::SetUp();
        auto result = session_->initialize(XRSessionConfig::VR_Default());
        ASSERT_EQ(result, XRResult::Success);
    }
};

// ============================================================================
// Initialization Tests
// ============================================================================

TEST_F(XRSessionTest, DefaultConstruction) {
    EXPECT_FALSE(session_->is_initialized());
    EXPECT_FALSE(session_->is_running());
    EXPECT_EQ(session_->get_state(), XRSessionState::Unknown);
}

TEST_F(XRSessionTest, InitializeDefault) {
    auto result = session_->initialize();
    EXPECT_EQ(result, XRResult::Success);
    EXPECT_TRUE(session_->is_initialized());
    EXPECT_TRUE(session_->is_running());
    EXPECT_EQ(session_->get_state(), XRSessionState::Focused);
}

TEST_F(XRSessionTest, InitializeVRDefault) {
    auto result = session_->initialize(XRSessionConfig::VR_Default());
    EXPECT_EQ(result, XRResult::Success);
    EXPECT_TRUE(session_->is_initialized());
}

TEST_F(XRSessionTest, InitializeVRSeated) {
    auto result = session_->initialize(XRSessionConfig::VR_Seated());
    EXPECT_EQ(result, XRResult::Success);
    EXPECT_TRUE(session_->is_initialized());
}

TEST_F(XRSessionTest, InitializeVRRoomScale) {
    auto result = session_->initialize(XRSessionConfig::VR_RoomScale());
    EXPECT_EQ(result, XRResult::Success);
    EXPECT_TRUE(session_->is_initialized());
}

TEST_F(XRSessionTest, InitializeARPassthrough) {
    auto result = session_->initialize(XRSessionConfig::AR_Passthrough());
    EXPECT_EQ(result, XRResult::Success);
    EXPECT_TRUE(session_->is_initialized());
}

TEST_F(XRSessionTest, InitializeVREyeTracking) {
    auto result = session_->initialize(XRSessionConfig::VR_EyeTracking());
    EXPECT_EQ(result, XRResult::Success);
    EXPECT_TRUE(session_->is_initialized());
}

TEST_F(XRSessionTest, InitializeVRFullFeatures) {
    auto result = session_->initialize(XRSessionConfig::VR_FullFeatures());
    EXPECT_EQ(result, XRResult::Success);
    EXPECT_TRUE(session_->is_initialized());
}

TEST_F(XRSessionTest, Shutdown) {
    session_->initialize();
    EXPECT_TRUE(session_->is_initialized());

    auto result = session_->shutdown();
    EXPECT_EQ(result, XRResult::Success);
    EXPECT_FALSE(session_->is_initialized());
    EXPECT_FALSE(session_->is_running());
}

TEST_F(XRSessionTest, ShutdownWithoutInit) {
    auto result = session_->shutdown();
    EXPECT_EQ(result, XRResult::NotInitialized);
}

// ============================================================================
// Capabilities Tests
// ============================================================================

TEST_F(XRSessionInitializedTest, GetCapabilities) {
    const auto& caps = session_->get_capabilities();

    EXPECT_FALSE(caps.runtime_name.empty());
    EXPECT_FALSE(caps.system_name.empty());
    EXPECT_GT(caps.max_refresh_rate, 0.0f);
    EXPECT_GT(caps.recommended_swapchain_width, 0u);
    EXPECT_GT(caps.recommended_swapchain_height, 0u);
}

TEST_F(XRSessionInitializedTest, CapabilitiesIncludeHandTracking) {
    const auto& caps = session_->get_capabilities();
    EXPECT_TRUE(caps.supports_hand_tracking);
    EXPECT_TRUE(caps.has_extension(XRExtension::HandTracking));
}

TEST_F(XRSessionInitializedTest, CapabilitiesIncludeEyeTracking) {
    const auto& caps = session_->get_capabilities();
    EXPECT_TRUE(caps.supports_eye_tracking);
    EXPECT_TRUE(caps.has_extension(XRExtension::EyeTracking));
}

TEST_F(XRSessionInitializedTest, CapabilitiesIncludeFoveatedRendering) {
    const auto& caps = session_->get_capabilities();
    EXPECT_TRUE(caps.supports_foveated_rendering);
    EXPECT_TRUE(caps.has_extension(XRExtension::FoveatedRendering));
}

TEST_F(XRSessionInitializedTest, CapabilitiesIncludePassthrough) {
    const auto& caps = session_->get_capabilities();
    EXPECT_TRUE(caps.supports_passthrough);
    EXPECT_TRUE(caps.has_extension(XRExtension::Passthrough));
}

// ============================================================================
// Frame Management Tests
// ============================================================================

TEST_F(XRSessionInitializedTest, BeginEndFrame) {
    XRFrameTiming timing;
    auto result = session_->begin_frame(timing);
    EXPECT_EQ(result, XRResult::Success);
    EXPECT_GT(timing.predicted_display_time_ns, 0);
    EXPECT_GT(timing.predicted_display_period_ns, 0);
    EXPECT_TRUE(timing.should_render);
    EXPECT_TRUE(session_->should_render());

    auto frame_result = session_->end_frame();
    EXPECT_EQ(frame_result.result, XRResult::Success);
}

TEST_F(XRSessionInitializedTest, MultipleFrames) {
    for (int i = 0; i < 10; ++i) {
        XRFrameTiming timing;
        auto result = session_->begin_frame(timing);
        EXPECT_EQ(result, XRResult::Success);

        auto frame_result = session_->end_frame();
        EXPECT_EQ(frame_result.result, XRResult::Success);
    }
}

TEST_F(XRSessionTest, BeginFrameWithoutInit) {
    XRFrameTiming timing;
    auto result = session_->begin_frame(timing);
    EXPECT_EQ(result, XRResult::SessionNotRunning);
}

// ============================================================================
// Head Tracking Tests
// ============================================================================

TEST_F(XRSessionInitializedTest, GetHeadState) {
    XRFrameTiming timing;
    session_->begin_frame(timing);

    const auto& head = session_->get_head_state();
    EXPECT_TRUE(head.head_pose.pose.is_valid);
    EXPECT_EQ(head.head_pose.pose.position_confidence, TrackingConfidence::High);
    EXPECT_EQ(head.head_pose.pose.orientation_confidence, TrackingConfidence::High);
    EXPECT_TRUE(head.is_user_present);
    EXPECT_GT(head.ipd, 0.0f);

    session_->end_frame();
}

TEST_F(XRSessionInitializedTest, HeadPoseDirections) {
    XRFrameTiming timing;
    session_->begin_frame(timing);

    const auto& head = session_->get_head_state();
    const auto& pose = head.head_pose.pose;

    // With identity orientation, forward should be -Z
    Vec3 forward = pose.forward();
    EXPECT_NEAR(forward.x, 0.0, 0.01);
    EXPECT_NEAR(forward.y, 0.0, 0.01);
    EXPECT_NEAR(forward.z, -1.0, 0.01);

    // Up should be +Y
    Vec3 up = pose.up();
    EXPECT_NEAR(up.x, 0.0, 0.01);
    EXPECT_NEAR(up.y, 1.0, 0.01);
    EXPECT_NEAR(up.z, 0.0, 0.01);

    // Right should be +X
    Vec3 right = pose.right();
    EXPECT_NEAR(right.x, 1.0, 0.01);
    EXPECT_NEAR(right.y, 0.0, 0.01);
    EXPECT_NEAR(right.z, 0.0, 0.01);

    session_->end_frame();
}

TEST_F(XRSessionInitializedTest, StereoViews) {
    XRFrameTiming timing;
    session_->begin_frame(timing);

    const auto& head = session_->get_head_state();

    // Should have two views for stereo
    EXPECT_TRUE(head.views[0].pose.is_valid);
    EXPECT_TRUE(head.views[1].pose.is_valid);

    // Left eye should be offset to the left (-X)
    EXPECT_LT(head.views[0].pose.position.x, 0.0);
    // Right eye should be offset to the right (+X)
    EXPECT_GT(head.views[1].pose.position.x, 0.0);

    session_->end_frame();
}

// ============================================================================
// Controller Tracking Tests
// ============================================================================

TEST_F(XRSessionInitializedTest, GetControllerState) {
    XRFrameTiming timing;
    session_->begin_frame(timing);

    const auto& left = session_->get_controller_state(XRHand::Left);
    EXPECT_TRUE(left.is_connected);
    EXPECT_TRUE(left.pose.pose.is_valid);

    const auto& right = session_->get_controller_state(XRHand::Right);
    EXPECT_TRUE(right.is_connected);
    EXPECT_TRUE(right.pose.pose.is_valid);

    session_->end_frame();
}

TEST_F(XRSessionInitializedTest, ControllerConnected) {
    EXPECT_TRUE(session_->is_controller_connected(XRHand::Left));
    EXPECT_TRUE(session_->is_controller_connected(XRHand::Right));
}

TEST_F(XRSessionInitializedTest, ControllerAxes) {
    XRFrameTiming timing;
    session_->begin_frame(timing);

    const auto& left = session_->get_controller_state(XRHand::Left);

    // Default values should be 0
    EXPECT_EQ(left.trigger(), 0.0f);
    EXPECT_EQ(left.grip(), 0.0f);
    EXPECT_EQ(left.get_axis(XRAxis::ThumbstickX), 0.0f);
    EXPECT_EQ(left.get_axis(XRAxis::ThumbstickY), 0.0f);

    session_->end_frame();
}

TEST_F(XRSessionInitializedTest, TriggerPressed) {
    // Default should not be pressed
    EXPECT_FALSE(session_->is_trigger_pressed(XRHand::Left));
    EXPECT_FALSE(session_->is_trigger_pressed(XRHand::Right));
}

TEST_F(XRSessionInitializedTest, GripPressed) {
    // Default should not be pressed
    EXPECT_FALSE(session_->is_grip_pressed(XRHand::Left));
    EXPECT_FALSE(session_->is_grip_pressed(XRHand::Right));
}

// ============================================================================
// Hand Tracking Tests
// ============================================================================

TEST_F(XRSessionInitializedTest, HandTrackingNotActiveByDefault) {
    // Mock runtime has hand tracking disabled by default
    EXPECT_FALSE(session_->is_hand_tracking_active(XRHand::Left));
    EXPECT_FALSE(session_->is_hand_tracking_active(XRHand::Right));
}

TEST_F(XRSessionInitializedTest, GetHandState) {
    XRFrameTiming timing;
    session_->begin_frame(timing);

    const auto& left = session_->get_hand_state(XRHand::Left);
    EXPECT_EQ(left.hand, XRHand::Left);

    const auto& right = session_->get_hand_state(XRHand::Right);
    EXPECT_EQ(right.hand, XRHand::Right);

    session_->end_frame();
}

TEST_F(XRSessionInitializedTest, PinchNotActiveByDefault) {
    EXPECT_FALSE(session_->is_pinching(XRHand::Left));
    EXPECT_FALSE(session_->is_pinching(XRHand::Right));
}

// ============================================================================
// Eye Tracking Tests
// ============================================================================

TEST_F(XRSessionInitializedTest, EyeTrackingState) {
    // Initialize with eye tracking enabled
    session_->shutdown();
    session_->initialize(XRSessionConfig::VR_EyeTracking());

    XRFrameTiming timing;
    session_->begin_frame(timing);

    const auto& eye_state = session_->get_eye_tracking_state();
    EXPECT_TRUE(eye_state.combined_gaze.is_valid);
    EXPECT_TRUE(eye_state.left_eye.is_valid);
    EXPECT_TRUE(eye_state.right_eye.is_valid);
    EXPECT_TRUE(eye_state.fixation_valid);

    session_->end_frame();
}

TEST_F(XRSessionInitializedTest, EyeTrackingActive) {
    // Re-initialize with eye tracking
    session_->shutdown();
    session_->initialize(XRSessionConfig::VR_EyeTracking());

    XRFrameTiming timing;
    session_->begin_frame(timing);
    session_->end_frame();

    EXPECT_TRUE(session_->is_eye_tracking_active());
}

// ============================================================================
// Interaction Pose Tests
// ============================================================================

TEST_F(XRSessionInitializedTest, GetInteractionPose) {
    XRFrameTiming timing;
    session_->begin_frame(timing);

    // With controllers connected, should return controller aim pose
    XRPose left_pose = session_->get_interaction_pose(XRHand::Left);
    EXPECT_TRUE(left_pose.is_valid);

    XRPose right_pose = session_->get_interaction_pose(XRHand::Right);
    EXPECT_TRUE(right_pose.is_valid);

    session_->end_frame();
}

TEST_F(XRSessionInitializedTest, GetInteractionRay) {
    XRFrameTiming timing;
    session_->begin_frame(timing);

    Vec3 origin, direction;
    session_->get_interaction_ray(XRHand::Right, origin, direction);

    // Direction should be normalized (length ~1)
    Real length = std::sqrt(direction.x * direction.x +
                           direction.y * direction.y +
                           direction.z * direction.z);
    EXPECT_NEAR(length, 1.0, 0.01);

    session_->end_frame();
}

// ============================================================================
// Haptic Feedback Tests
// ============================================================================

TEST_F(XRSessionInitializedTest, Vibrate) {
    auto result = session_->vibrate(XRHand::Left, 0.5f, 0.1f, 160.0f);
    EXPECT_EQ(result, XRResult::Success);

    result = session_->vibrate(XRHand::Right, 1.0f, 0.2f, 200.0f);
    EXPECT_EQ(result, XRResult::Success);

    const auto& stats = session_->get_statistics();
    EXPECT_EQ(stats.haptic_events_sent, 2u);
}

TEST_F(XRSessionInitializedTest, StopVibration) {
    session_->vibrate(XRHand::Left);
    auto result = session_->stop_vibration(XRHand::Left);
    EXPECT_EQ(result, XRResult::Success);
}

TEST_F(XRSessionTest, VibrateWithoutSession) {
    auto result = session_->vibrate(XRHand::Left);
    EXPECT_EQ(result, XRResult::SessionNotRunning);
}

// ============================================================================
// Configuration Tests
// ============================================================================

TEST_F(XRSessionInitializedTest, SetRefreshRate) {
    auto result = session_->set_refresh_rate(90.0f);
    EXPECT_EQ(result, XRResult::Success);
    EXPECT_NEAR(session_->get_refresh_rate(), 90.0f, 0.1f);
}

TEST_F(XRSessionInitializedTest, SetPassthrough) {
    XRPassthroughConfig config;
    config.mode = XRPassthroughMode::FullPassthrough;
    config.opacity = 0.8f;

    auto result = session_->set_passthrough(config);
    EXPECT_EQ(result, XRResult::Success);
}

TEST_F(XRSessionInitializedTest, EnablePassthrough) {
    auto result = session_->enable_passthrough(true, 0.5f);
    EXPECT_EQ(result, XRResult::Success);

    result = session_->enable_passthrough(false);
    EXPECT_EQ(result, XRResult::Success);
}

TEST_F(XRSessionInitializedTest, SetFoveation) {
    XRFoveationConfig config;
    config.level = XRFoveationLevel::High;

    auto result = session_->set_foveation(config);
    EXPECT_EQ(result, XRResult::Success);
}

TEST_F(XRSessionInitializedTest, Recenter) {
    auto result = session_->recenter();
    EXPECT_EQ(result, XRResult::Success);
}

// ============================================================================
// Boundary Tests
// ============================================================================

TEST_F(XRSessionInitializedTest, GetPlayArea) {
    const auto& area = session_->get_play_area();
    EXPECT_TRUE(area.is_configured);
    EXPECT_GT(area.width, 0.0f);
    EXPECT_GT(area.depth, 0.0f);
    EXPECT_EQ(area.type, XRBoundaryType::RoomScale);
}

TEST_F(XRSessionInitializedTest, CheckBoundaryCenter) {
    // Center of play area should be safe
    auto warning = session_->check_boundary(Vec3::Zero());
    EXPECT_FALSE(warning.is_triggered);
}

TEST_F(XRSessionInitializedTest, CheckBoundaryEdge) {
    // Near edge should trigger warning
    auto warning = session_->check_boundary(Vec3{1.2f, 0.0f, 0.0f});
    EXPECT_TRUE(warning.is_triggered);
    EXPECT_GT(warning.closest_point.length_squared(), 0.0);
}

TEST_F(XRSessionInitializedTest, CheckDevicesBoundary) {
    XRFrameTiming timing;
    session_->begin_frame(timing);

    // Default positions should be safe
    auto warning = session_->check_devices_boundary();
    // May or may not be triggered depending on default positions

    session_->end_frame();
}

// ============================================================================
// Statistics Tests
// ============================================================================

TEST_F(XRSessionInitializedTest, Statistics) {
    // Run a few frames
    for (int i = 0; i < 5; ++i) {
        XRFrameTiming timing;
        session_->begin_frame(timing);
        session_->end_frame();
    }

    const auto& stats = session_->get_statistics();
    EXPECT_EQ(stats.frames_rendered, 5u);
    EXPECT_GT(stats.session_start_time_ns, 0);
}

TEST_F(XRSessionInitializedTest, ResetStatistics) {
    // Run some frames
    for (int i = 0; i < 3; ++i) {
        XRFrameTiming timing;
        session_->begin_frame(timing);
        session_->end_frame();
    }

    session_->reset_statistics();

    const auto& stats = session_->get_statistics();
    EXPECT_EQ(stats.frames_rendered, 0u);
    EXPECT_EQ(stats.frames_dropped, 0u);
    EXPECT_EQ(stats.haptic_events_sent, 0u);
}

// ============================================================================
// State Callback Tests
// ============================================================================

TEST_F(XRSessionTest, StateCallback) {
    std::vector<std::pair<XRSessionState, XRSessionState>> transitions;

    session_->set_state_callback([&](XRSessionState old_state, XRSessionState new_state) {
        transitions.push_back({old_state, new_state});
    });

    session_->initialize();

    // Should have received state transitions
    EXPECT_GT(transitions.size(), 0u);

    // Final state should be Focused
    EXPECT_EQ(transitions.back().second, XRSessionState::Focused);
}

// ============================================================================
// XRPose Tests
// ============================================================================

TEST(XRPoseTest, Identity) {
    XRPose pose = XRPose::Identity();
    EXPECT_TRUE(pose.is_valid);
    EXPECT_EQ(pose.position.x, 0.0);
    EXPECT_EQ(pose.position.y, 0.0);
    EXPECT_EQ(pose.position.z, 0.0);
    EXPECT_EQ(pose.orientation.w, 1.0);
    EXPECT_EQ(pose.orientation.x, 0.0);
    EXPECT_EQ(pose.orientation.y, 0.0);
    EXPECT_EQ(pose.orientation.z, 0.0);
}

TEST(XRPoseTest, HasTracking) {
    XRPose pose;
    pose.is_valid = true;
    pose.position_confidence = TrackingConfidence::High;
    pose.orientation_confidence = TrackingConfidence::Low;

    EXPECT_TRUE(pose.has_tracking());
    EXPECT_FALSE(pose.has_full_tracking());

    pose.orientation_confidence = TrackingConfidence::High;
    EXPECT_TRUE(pose.has_full_tracking());
}

TEST(XRPoseTest, Lerp) {
    XRPose a = XRPose::Identity();
    a.position = Vec3{0.0, 0.0, 0.0};

    XRPose b = XRPose::Identity();
    b.position = Vec3{10.0, 0.0, 0.0};

    XRPose mid = XRPose::lerp(a, b, 0.5);
    EXPECT_NEAR(mid.position.x, 5.0, 0.01);
    EXPECT_NEAR(mid.position.y, 0.0, 0.01);
    EXPECT_NEAR(mid.position.z, 0.0, 0.01);
}

TEST(XRPoseTest, TransformPoint) {
    XRPose pose = XRPose::Identity();
    pose.position = Vec3{1.0, 2.0, 3.0};

    Vec3 local_point{0.5, 0.5, 0.5};
    Vec3 world_point = pose.transform_point(local_point);

    EXPECT_NEAR(world_point.x, 1.5, 0.01);
    EXPECT_NEAR(world_point.y, 2.5, 0.01);
    EXPECT_NEAR(world_point.z, 3.5, 0.01);
}

// ============================================================================
// XRResult Tests
// ============================================================================

TEST(XRResultTest, SucceededCheck) {
    EXPECT_TRUE(xr_succeeded(XRResult::Success));
    EXPECT_FALSE(xr_succeeded(XRResult::NotInitialized));
    EXPECT_FALSE(xr_succeeded(XRResult::SessionNotRunning));
    EXPECT_FALSE(xr_succeeded(XRResult::InternalError));
}

TEST(XRResultTest, ResultToString) {
    EXPECT_STREQ(xr_result_to_string(XRResult::Success), "Success");
    EXPECT_STREQ(xr_result_to_string(XRResult::NotInitialized), "NotInitialized");
    EXPECT_STREQ(xr_result_to_string(XRResult::SessionNotRunning), "SessionNotRunning");
    EXPECT_STREQ(xr_result_to_string(XRResult::TrackingLost), "TrackingLost");
}

// ============================================================================
// XR Event Tests
// ============================================================================

TEST(XREventTest, IsXREvent) {
    EXPECT_TRUE(is_xr_event(to_event_type(XREventType::SessionStateChanged)));
    EXPECT_TRUE(is_xr_event(to_event_type(XREventType::ControllerButtonPressed)));
    EXPECT_TRUE(is_xr_event(to_event_type(XREventType::HandGestureDetected)));
    EXPECT_TRUE(is_xr_event(to_event_type(XREventType::PassthroughStarted)));

    // Non-XR events
    EXPECT_FALSE(is_xr_event(events::EventType::SimulationStarted));
    EXPECT_FALSE(is_xr_event(events::EventType::CollisionEnter));
}

TEST(XREventTest, CreateSessionStateChangedEvent) {
    auto event = events_factory::create_session_state_changed(
        XRSessionState::Ready, XRSessionState::Focused, 1.0);

    EXPECT_EQ(event.type(), to_event_type(XREventType::SessionStateChanged));
    EXPECT_EQ(event.priority(), events::EventPriority::High);
    EXPECT_NEAR(event.timestamp(), 1.0, 0.001);
}

TEST(XREventTest, CreateButtonPressedEvent) {
    auto event = events_factory::create_button_pressed(
        XRHand::Right, XRButton::A, 1.0f, 2.5);

    EXPECT_EQ(event.type(), to_event_type(XREventType::ControllerButtonPressed));
    EXPECT_NEAR(event.timestamp(), 2.5, 0.001);
}

TEST(XREventTest, CreateGestureDetectedEvent) {
    auto event = events_factory::create_gesture_detected(
        XRHand::Left, XRHandGesture::Pinch, XRHandGesture::Open,
        0.95f, Vec3{0.1f, 0.2f, 0.3f}, 3.0);

    EXPECT_EQ(event.type(), to_event_type(XREventType::HandGestureDetected));
}

// ============================================================================
// Controller State Helper Tests
// ============================================================================

TEST(XRControllerStateTest, ButtonChecks) {
    XRControllerState state;
    state.buttons_pressed[static_cast<size_t>(XRButton::A)] = true;
    state.buttons_touched[static_cast<size_t>(XRButton::B)] = true;

    EXPECT_TRUE(state.is_pressed(XRButton::A));
    EXPECT_FALSE(state.is_pressed(XRButton::B));
    EXPECT_FALSE(state.is_touched(XRButton::A));
    EXPECT_TRUE(state.is_touched(XRButton::B));
}

TEST(XRControllerStateTest, AxisAccess) {
    XRControllerState state;
    state.axes[static_cast<size_t>(XRAxis::Trigger)] = 0.75f;
    state.axes[static_cast<size_t>(XRAxis::Grip)] = 0.5f;
    state.axes[static_cast<size_t>(XRAxis::ThumbstickX)] = -0.3f;
    state.axes[static_cast<size_t>(XRAxis::ThumbstickY)] = 0.8f;

    EXPECT_NEAR(state.trigger(), 0.75f, 0.001f);
    EXPECT_NEAR(state.grip(), 0.5f, 0.001f);

    Vec3 thumbstick = state.thumbstick();
    EXPECT_NEAR(thumbstick.x, -0.3f, 0.001f);
    EXPECT_NEAR(thumbstick.y, 0.8f, 0.001f);
}

// ============================================================================
// Hand State Helper Tests
// ============================================================================

TEST(XRHandStateTest, JointAccess) {
    XRHandState state;
    state.hand = XRHand::Left;
    state.is_active = true;
    state.confidence = TrackingConfidence::High;

    // Set palm joint
    state.joints[static_cast<size_t>(XRHandJoint::Palm)].pose.position = Vec3{0.1f, 0.2f, 0.3f};
    state.joints[static_cast<size_t>(XRHandJoint::Palm)].pose.is_valid = true;

    EXPECT_TRUE(state.is_valid());
    EXPECT_NEAR(state.palm_position().x, 0.1f, 0.001f);
    EXPECT_NEAR(state.palm_position().y, 0.2f, 0.001f);
    EXPECT_NEAR(state.palm_position().z, 0.3f, 0.001f);
}

TEST(XRHandStateTest, FingertipAccess) {
    XRHandState state;
    state.joints[static_cast<size_t>(XRHandJoint::IndexTip)].pose.position = Vec3{1.0f, 2.0f, 3.0f};
    state.joints[static_cast<size_t>(XRHandJoint::ThumbTip)].pose.position = Vec3{1.1f, 2.1f, 3.1f};

    EXPECT_NEAR(state.index_tip().x, 1.0f, 0.001f);
    EXPECT_NEAR(state.thumb_tip().x, 1.1f, 0.001f);
}

// ============================================================================
// Extension Support Tests
// ============================================================================

TEST(XRExtensionTest, HasExtension) {
    XRExtension mask = XRExtension::HandTracking | XRExtension::EyeTracking;

    EXPECT_TRUE(has_extension(mask, XRExtension::HandTracking));
    EXPECT_TRUE(has_extension(mask, XRExtension::EyeTracking));
    EXPECT_FALSE(has_extension(mask, XRExtension::Passthrough));
    EXPECT_FALSE(has_extension(mask, XRExtension::SpatialAnchor));
}

// ============================================================================
// Performance Tests
// ============================================================================

TEST_F(XRSessionInitializedTest, DISABLED_FramePerformance) {
    // Measure frame timing performance
    const int num_frames = 1000;
    auto start = std::chrono::high_resolution_clock::now();

    for (int i = 0; i < num_frames; ++i) {
        XRFrameTiming timing;
        session_->begin_frame(timing);
        session_->end_frame();
    }

    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);

    double avg_frame_us = static_cast<double>(duration.count()) / num_frames;
    std::cout << "Average frame time: " << avg_frame_us << " us" << std::endl;

    // Should be fast (< 1ms per frame for mock runtime)
    EXPECT_LT(avg_frame_us, 1000.0);
}

} // namespace jaguar::xr
