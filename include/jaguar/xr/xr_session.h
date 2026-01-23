#pragma once
/**
 * @file xr_session.h
 * @brief XR session management and OpenXR runtime integration
 *
 * This file defines the XRSession class for managing XR session lifecycle,
 * the IXRRuntime interface for runtime abstraction, and configuration structures.
 * Supports VR and AR experiences with comprehensive tracking capabilities.
 */

#include "jaguar/xr/xr_types.h"
#include <memory>
#include <vector>
#include <unordered_map>
#include <mutex>
#include <atomic>

namespace jaguar::xr {

// ============================================================================
// Configuration Structures
// ============================================================================

/**
 * @brief XR session configuration
 *
 * Configure session parameters before initialization. Sensible defaults
 * provided for common VR use cases.
 */
struct XRSessionConfig {
    // Basic session settings
    XRFormFactor form_factor{XRFormFactor::HeadMountedDisplay};
    XRViewConfigType view_config{XRViewConfigType::Stereo};
    XRBlendMode preferred_blend_mode{XRBlendMode::Opaque};

    // Tracking configuration
    XRSpaceType reference_space{XRSpaceType::Local};
    bool enable_guardian{true};     ///< Enable boundary system

    // Feature toggles
    bool enable_hand_tracking{true};
    bool enable_eye_tracking{false};
    bool enable_passthrough{false};

    // Rendering configuration
    XRFoveationConfig foveation;
    UInt32 swapchain_sample_count{1}; ///< MSAA samples (1 = no MSAA)
    bool use_depth_submission{true};  ///< Submit depth for reprojection

    // Performance settings
    bool enable_performance_settings{true}; ///< Allow runtime performance hints
    Real target_frame_rate{90.0f};  ///< Target refresh rate (Hz)

    // Extensions to request
    XRExtension requested_extensions{
        XRExtension::HandTracking |
        XRExtension::ControllerInteraction
    };

    // Application info
    std::string application_name{"JaguarEngine XR"};
    UInt32 application_version{1};
    std::string engine_name{"JaguarEngine"};
    UInt32 engine_version{1};

    // Factory methods for common configurations
    static XRSessionConfig VR_Default() {
        return XRSessionConfig{};
    }

    static XRSessionConfig VR_Seated() {
        XRSessionConfig config;
        config.reference_space = XRSpaceType::Local;
        config.enable_guardian = false;
        return config;
    }

    static XRSessionConfig VR_RoomScale() {
        XRSessionConfig config;
        config.reference_space = XRSpaceType::Stage;
        config.enable_guardian = true;
        return config;
    }

    static XRSessionConfig AR_Passthrough() {
        XRSessionConfig config;
        config.preferred_blend_mode = XRBlendMode::AlphaBlend;
        config.enable_passthrough = true;
        config.reference_space = XRSpaceType::Local;
        config.requested_extensions = config.requested_extensions | XRExtension::Passthrough;
        return config;
    }

    static XRSessionConfig VR_EyeTracking() {
        XRSessionConfig config;
        config.enable_eye_tracking = true;
        config.foveation.level = XRFoveationLevel::Dynamic;
        config.foveation.use_eye_tracking = true;
        config.requested_extensions = config.requested_extensions | XRExtension::EyeTracking;
        return config;
    }

    static XRSessionConfig VR_FullFeatures() {
        XRSessionConfig config;
        config.reference_space = XRSpaceType::Stage;
        config.enable_guardian = true;
        config.enable_hand_tracking = true;
        config.enable_eye_tracking = true;
        config.foveation.level = XRFoveationLevel::Dynamic;
        config.foveation.use_eye_tracking = true;
        config.requested_extensions =
            XRExtension::HandTracking |
            XRExtension::EyeTracking |
            XRExtension::FoveatedRendering |
            XRExtension::ControllerInteraction |
            XRExtension::HandInteraction;
        return config;
    }
};

/**
 * @brief Runtime capabilities discovered during initialization
 */
struct XRRuntimeCapabilities {
    // Basic info
    std::string runtime_name;
    std::string runtime_version;
    std::string system_name;

    // Supported features
    XRExtension supported_extensions{static_cast<XRExtension>(0)};
    XRFormFactor supported_form_factor{XRFormFactor::HeadMountedDisplay};
    std::vector<XRBlendMode> supported_blend_modes;
    std::vector<XRViewConfigType> supported_view_configs;

    // Tracking capabilities
    bool supports_hand_tracking{false};
    bool supports_eye_tracking{false};
    bool supports_foveated_rendering{false};
    bool supports_passthrough{false};
    bool supports_body_tracking{false};
    bool supports_spatial_anchors{false};

    // Display properties
    Real max_refresh_rate{90.0f};
    std::vector<Real> supported_refresh_rates;
    UInt32 max_swapchain_width{4096};
    UInt32 max_swapchain_height{4096};
    UInt32 recommended_swapchain_width{1920};
    UInt32 recommended_swapchain_height{1920};

    // Tracking properties
    XRPlayArea play_area;

    /// Check if an extension is supported
    bool has_extension(XRExtension ext) const {
        return jaguar::xr::has_extension(supported_extensions, ext);
    }
};

// ============================================================================
// XR Runtime Interface
// ============================================================================

/**
 * @brief Abstract interface for XR runtime backends
 *
 * This interface abstracts the OpenXR runtime, allowing for mock implementations
 * in testing and potential future support for other XR APIs.
 */
class IXRRuntime {
public:
    virtual ~IXRRuntime() = default;

    // Lifecycle
    virtual XRResult initialize(const XRSessionConfig& config) = 0;
    virtual XRResult shutdown() = 0;
    virtual bool is_initialized() const = 0;

    // Capabilities
    virtual const XRRuntimeCapabilities& get_capabilities() const = 0;

    // Session management
    virtual XRResult begin_session() = 0;
    virtual XRResult end_session() = 0;
    virtual XRSessionState get_session_state() const = 0;
    virtual void set_session_state_callback(SessionStateCallback callback) = 0;

    // Frame timing
    virtual XRResult wait_frame(XRFrameTiming& timing) = 0;
    virtual XRResult begin_frame() = 0;
    virtual XRResult end_frame(const XRFrameResult& result) = 0;

    // Tracking - HMD
    virtual XRResult get_head_state(XRHeadState& state, Int64 predicted_time_ns = 0) = 0;

    // Tracking - Controllers
    virtual XRResult get_controller_state(XRHand hand, XRControllerState& state,
                                          Int64 predicted_time_ns = 0) = 0;
    virtual XRResult send_haptic_feedback(XRHand hand, const XRHapticFeedback& feedback) = 0;
    virtual XRResult stop_haptic_feedback(XRHand hand) = 0;

    // Tracking - Hands
    virtual XRResult get_hand_state(XRHand hand, XRHandState& state,
                                    Int64 predicted_time_ns = 0) = 0;

    // Tracking - Eye
    virtual XRResult get_eye_tracking_state(XREyeTrackingState& state,
                                            Int64 predicted_time_ns = 0) = 0;

    // Passthrough
    virtual XRResult set_passthrough_config(const XRPassthroughConfig& config) = 0;
    virtual XRResult get_passthrough_config(XRPassthroughConfig& config) const = 0;

    // Foveation
    virtual XRResult set_foveation_config(const XRFoveationConfig& config) = 0;
    virtual XRResult get_foveation_config(XRFoveationConfig& config) const = 0;

    // Boundary
    virtual XRResult get_play_area(XRPlayArea& area) const = 0;
    virtual XRResult check_boundary_proximity(const Vec3& point, XRBoundaryWarning& warning) = 0;

    // Performance
    virtual XRResult set_refresh_rate(Real rate) = 0;
    virtual XRResult get_current_refresh_rate(Real& rate) const = 0;

    // Reference space
    virtual XRResult set_reference_space(XRSpaceType space) = 0;
    virtual XRResult get_reference_space(XRSpaceType& space) const = 0;
    virtual XRResult recenter() = 0;
};

// ============================================================================
// XR Session Class
// ============================================================================

/**
 * @brief Main XR session manager
 *
 * XRSession provides a high-level interface for XR functionality, managing
 * the runtime lifecycle, tracking state, and input processing. Designed for
 * integration with JaguarEngine's entity system.
 */
class XRSession {
public:
    /**
     * @brief Construct XR session with custom runtime
     * @param runtime Custom runtime implementation (for testing/mocking)
     */
    explicit XRSession(std::unique_ptr<IXRRuntime> runtime);

    /**
     * @brief Construct XR session with default OpenXR runtime
     */
    XRSession();

    ~XRSession();

    // Non-copyable, movable
    XRSession(const XRSession&) = delete;
    XRSession& operator=(const XRSession&) = delete;
    XRSession(XRSession&&) noexcept;
    XRSession& operator=(XRSession&&) noexcept;

    // ========================================================================
    // Lifecycle
    // ========================================================================

    /**
     * @brief Initialize XR session with configuration
     * @param config Session configuration
     * @return XRResult::Success or error code
     */
    XRResult initialize(const XRSessionConfig& config = XRSessionConfig::VR_Default());

    /**
     * @brief Shutdown XR session and release resources
     */
    XRResult shutdown();

    /**
     * @brief Check if session is initialized
     */
    bool is_initialized() const;

    /**
     * @brief Check if session is running (ready for rendering)
     */
    bool is_running() const;

    /**
     * @brief Get current session state
     */
    XRSessionState get_state() const;

    /**
     * @brief Get runtime capabilities
     */
    const XRRuntimeCapabilities& get_capabilities() const;

    /**
     * @brief Register callback for session state changes
     */
    void set_state_callback(SessionStateCallback callback);

    // ========================================================================
    // Frame Management
    // ========================================================================

    /**
     * @brief Begin XR frame - call at start of render loop
     *
     * Waits for frame timing, updates tracking data, and prepares for rendering.
     * @param timing Output frame timing information
     * @return XRResult::Success or error code
     */
    XRResult begin_frame(XRFrameTiming& timing);

    /**
     * @brief End XR frame - call after rendering
     * @return Frame submission result
     */
    XRFrameResult end_frame();

    /**
     * @brief Check if frame should be rendered
     *
     * Returns false if runtime requests frame skip (e.g., during session transitions)
     */
    bool should_render() const;

    // ========================================================================
    // Tracking Access
    // ========================================================================

    /**
     * @brief Get current head tracking state
     * @param predicted_time_ns Prediction time (0 = latest)
     */
    const XRHeadState& get_head_state(Int64 predicted_time_ns = 0);

    /**
     * @brief Get controller state for specified hand
     * @param hand Left or right hand
     * @param predicted_time_ns Prediction time (0 = latest)
     */
    const XRControllerState& get_controller_state(XRHand hand, Int64 predicted_time_ns = 0);

    /**
     * @brief Get hand tracking state for specified hand
     * @param hand Left or right hand
     * @param predicted_time_ns Prediction time (0 = latest)
     */
    const XRHandState& get_hand_state(XRHand hand, Int64 predicted_time_ns = 0);

    /**
     * @brief Get eye tracking state
     * @param predicted_time_ns Prediction time (0 = latest)
     */
    const XREyeTrackingState& get_eye_tracking_state(Int64 predicted_time_ns = 0);

    /**
     * @brief Check if hand tracking is available and active
     */
    bool is_hand_tracking_active(XRHand hand) const;

    /**
     * @brief Check if controllers are connected
     */
    bool is_controller_connected(XRHand hand) const;

    /**
     * @brief Check if eye tracking is available
     */
    bool is_eye_tracking_active() const;

    // ========================================================================
    // Input Helpers
    // ========================================================================

    /**
     * @brief Get primary interaction pose (controller or hand)
     *
     * Returns controller aim pose if available, falls back to hand pinch point.
     * @param hand Left or right hand
     */
    XRPose get_interaction_pose(XRHand hand) const;

    /**
     * @brief Get interaction ray for pointing/selection
     * @param hand Left or right hand
     * @param origin Output ray origin
     * @param direction Output ray direction
     */
    void get_interaction_ray(XRHand hand, Vec3& origin, Vec3& direction) const;

    /**
     * @brief Check if trigger is pressed (analog threshold)
     * @param hand Left or right hand
     * @param threshold Analog threshold (default 0.8)
     */
    bool is_trigger_pressed(XRHand hand, Real threshold = 0.8f) const;

    /**
     * @brief Check if grip is pressed (analog threshold)
     * @param hand Left or right hand
     * @param threshold Analog threshold (default 0.8)
     */
    bool is_grip_pressed(XRHand hand, Real threshold = 0.8f) const;

    /**
     * @brief Check if hand is pinching (for hand tracking)
     * @param hand Left or right hand
     * @param threshold Pinch strength threshold (default 0.8)
     */
    bool is_pinching(XRHand hand, Real threshold = 0.8f) const;

    // ========================================================================
    // Haptic Feedback
    // ========================================================================

    /**
     * @brief Send haptic pulse to controller
     * @param hand Left or right hand
     * @param amplitude Vibration strength (0-1)
     * @param duration Duration in seconds
     * @param frequency Vibration frequency in Hz
     */
    XRResult vibrate(XRHand hand, Real amplitude = 1.0f, Real duration = 0.1f,
                     Real frequency = 160.0f);

    /**
     * @brief Stop haptic feedback on controller
     * @param hand Left or right hand
     */
    XRResult stop_vibration(XRHand hand);

    // ========================================================================
    // Configuration
    // ========================================================================

    /**
     * @brief Set passthrough mode and configuration
     */
    XRResult set_passthrough(const XRPassthroughConfig& config);

    /**
     * @brief Enable/disable passthrough (convenience method)
     */
    XRResult enable_passthrough(bool enable, Real opacity = 1.0f);

    /**
     * @brief Set foveated rendering configuration
     */
    XRResult set_foveation(const XRFoveationConfig& config);

    /**
     * @brief Set display refresh rate
     * @param rate Target refresh rate in Hz
     */
    XRResult set_refresh_rate(Real rate);

    /**
     * @brief Get current refresh rate
     */
    Real get_refresh_rate() const;

    /**
     * @brief Recenter tracking origin
     */
    XRResult recenter();

    // ========================================================================
    // Boundary System
    // ========================================================================

    /**
     * @brief Get play area configuration
     */
    const XRPlayArea& get_play_area() const;

    /**
     * @brief Check proximity to boundary
     * @param point Point to check (in reference space)
     */
    XRBoundaryWarning check_boundary(const Vec3& point) const;

    /**
     * @brief Check if any tracked device is near boundary
     */
    XRBoundaryWarning check_devices_boundary() const;

    // ========================================================================
    // Entity Integration
    // ========================================================================

    /**
     * @brief Synchronize XR head pose with an entity
     * @param entity_id Entity to update with head pose
     */
    void sync_head_to_entity(EntityId entity_id);

    /**
     * @brief Synchronize XR controller pose with an entity
     * @param entity_id Entity to update
     * @param hand Which controller to sync
     */
    void sync_controller_to_entity(EntityId entity_id, XRHand hand);

    /**
     * @brief Synchronize XR hand pose with an entity
     * @param entity_id Entity to update
     * @param hand Which hand to sync
     */
    void sync_hand_to_entity(EntityId entity_id, XRHand hand);

    // ========================================================================
    // Statistics
    // ========================================================================

    /**
     * @brief XR session statistics
     */
    struct Statistics {
        UInt64 frames_rendered{0};
        UInt64 frames_dropped{0};
        Real average_frame_time_ms{0.0f};
        Real compositor_latency_ms{0.0f};
        Real tracking_latency_ms{0.0f};
        UInt64 haptic_events_sent{0};
        Int64 session_start_time_ns{0};
    };

    /**
     * @brief Get session statistics
     */
    const Statistics& get_statistics() const;

    /**
     * @brief Reset statistics
     */
    void reset_statistics();

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

// ============================================================================
// OpenXR Runtime Factory
// ============================================================================

/**
 * @brief Create OpenXR runtime instance
 *
 * Creates the default OpenXR runtime implementation. Returns nullptr
 * if OpenXR is not available on the system.
 */
std::unique_ptr<IXRRuntime> create_openxr_runtime();

/**
 * @brief Check if OpenXR is available
 */
bool is_openxr_available();

/**
 * @brief Create mock XR runtime for testing
 *
 * Creates a runtime that simulates XR behavior without actual hardware.
 * Useful for testing and development without a headset.
 */
std::unique_ptr<IXRRuntime> create_mock_xr_runtime();

} // namespace jaguar::xr
