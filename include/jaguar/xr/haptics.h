#pragma once
/**
 * @file haptics.h
 * @brief Haptic feedback types and interfaces for immersive XR experiences
 *
 * This file defines the haptic feedback system for JaguarEngine, providing
 * controller vibration, haptic vest integration, seat motion cueing,
 * engine vibration feedback, and G-force simulation. Designed for XR integration
 * with flight, vehicle, and naval simulation scenarios.
 */

#include "jaguar/core/types.h"
#include "jaguar/xr/xr_types.h"
#include <array>
#include <vector>
#include <memory>
#include <functional>
#include <string>
#include <optional>
#include <chrono>
#include <queue>

namespace jaguar::haptics {

// ============================================================================
// Forward Declarations
// ============================================================================

class IHapticDevice;
class IHapticController;
class IHapticVest;
class IMotionPlatform;
class IHapticRenderer;

// ============================================================================
// Haptic Result Types
// ============================================================================

/**
 * @brief Result codes for haptic operations
 */
enum class HapticResult : Int32 {
    Success = 0,
    NotInitialized = -1,
    DeviceNotConnected = -2,
    DeviceNotSupported = -3,
    InvalidParameter = -4,
    EffectNotSupported = -5,
    EffectQueueFull = -6,
    BufferOverflow = -7,
    CalibrationRequired = -8,
    SafetyLimitExceeded = -9,
    CommunicationError = -10,
    PermissionDenied = -11,
    OutOfMemory = -12,
    InternalError = -99
};

/**
 * @brief Convert HapticResult to string for logging
 */
const char* haptic_result_to_string(HapticResult result);

/**
 * @brief Check if HapticResult indicates success
 */
inline bool haptic_succeeded(HapticResult result) {
    return result == HapticResult::Success;
}

// ============================================================================
// Haptic Device Types
// ============================================================================

/**
 * @brief Types of haptic devices
 */
enum class HapticDeviceType : UInt8 {
    Unknown = 0,
    Controller = 1,         ///< VR/XR controller with vibration motors
    Vest = 2,               ///< Haptic vest with multiple actuators
    Glove = 3,              ///< Haptic glove for hand feedback
    MotionPlatform = 4,     ///< Motion platform/seat cueing system
    SteeringWheel = 5,      ///< Force feedback steering wheel
    FlightControls = 6,     ///< Force feedback flight stick/yoke
    Exoskeleton = 7         ///< Full body haptic exoskeleton
};

/**
 * @brief Haptic actuator types
 */
enum class ActuatorType : UInt8 {
    ERM = 0,                ///< Eccentric Rotating Mass (vibration motor)
    LRA = 1,                ///< Linear Resonant Actuator
    VCA = 2,                ///< Voice Coil Actuator
    Piezo = 3,              ///< Piezoelectric actuator
    Pneumatic = 4,          ///< Pneumatic actuator
    Hydraulic = 5,          ///< Hydraulic actuator
    Electric = 6            ///< Electric motor (force feedback)
};

/**
 * @brief Haptic device capabilities
 */
struct HapticDeviceCapabilities {
    HapticDeviceType device_type{HapticDeviceType::Unknown};
    ActuatorType actuator_type{ActuatorType::ERM};
    UInt8 actuator_count{1};
    Real frequency_min{0.0};        ///< Minimum frequency (Hz)
    Real frequency_max{1000.0};     ///< Maximum frequency (Hz)
    Real amplitude_resolution{256}; ///< Amplitude steps
    Real max_force{0.0};            ///< Maximum force output (N)
    Real max_displacement{0.0};     ///< Maximum displacement (m)
    bool supports_custom_waveforms{false};
    bool supports_force_feedback{false};
    bool supports_thermal{false};
    std::string device_name;
    std::string manufacturer;
    std::string serial_number;
};

/**
 * @brief Haptic device connection state
 */
enum class HapticConnectionState : UInt8 {
    Disconnected = 0,
    Connecting = 1,
    Connected = 2,
    Calibrating = 3,
    Ready = 4,
    Error = 5
};

// ============================================================================
// Haptic Effect Types
// ============================================================================

/**
 * @brief Basic haptic effect waveform types
 */
enum class HapticWaveform : UInt8 {
    None = 0,
    Constant = 1,           ///< Constant amplitude
    Sine = 2,               ///< Sinusoidal oscillation
    Square = 3,             ///< Square wave
    Triangle = 4,           ///< Triangle wave
    Sawtooth = 5,           ///< Sawtooth wave
    Noise = 6,              ///< Random noise
    Custom = 7              ///< Custom waveform buffer
};

/**
 * @brief Haptic effect envelope (ADSR-style)
 */
struct HapticEnvelope {
    Real attack_time{0.0};      ///< Attack time (seconds)
    Real attack_level{1.0};     ///< Attack peak level (0-1)
    Real decay_time{0.0};       ///< Decay time (seconds)
    Real sustain_level{1.0};    ///< Sustain level (0-1)
    Real release_time{0.0};     ///< Release time (seconds)

    /// Create instant envelope (no fade)
    static HapticEnvelope Instant() {
        return HapticEnvelope{};
    }

    /// Create fade-in envelope
    static HapticEnvelope FadeIn(Real attack_seconds) {
        HapticEnvelope env;
        env.attack_time = attack_seconds;
        return env;
    }

    /// Create fade-out envelope
    static HapticEnvelope FadeOut(Real release_seconds) {
        HapticEnvelope env;
        env.release_time = release_seconds;
        return env;
    }

    /// Create ADSR envelope
    static HapticEnvelope ADSR(Real attack, Real decay, Real sustain, Real release) {
        HapticEnvelope env;
        env.attack_time = attack;
        env.decay_time = decay;
        env.sustain_level = sustain;
        env.release_time = release;
        return env;
    }

    /// Get total envelope duration (excluding sustain)
    Real get_transition_duration() const {
        return attack_time + decay_time + release_time;
    }
};

/**
 * @brief Core haptic effect definition
 */
struct HapticEffect {
    HapticWaveform waveform{HapticWaveform::Constant};
    Real frequency{160.0};      ///< Base frequency (Hz)
    Real amplitude{0.5};        ///< Amplitude (0-1)
    Real duration{0.1};         ///< Duration (seconds), 0 = infinite
    HapticEnvelope envelope;
    Int32 repeat_count{1};      ///< Number of repeats (-1 = infinite)
    Real repeat_delay{0.0};     ///< Delay between repeats (seconds)

    // Optional frequency/amplitude modulation
    Real frequency_variation{0.0};  ///< Random frequency variation (Hz)
    Real amplitude_variation{0.0};  ///< Random amplitude variation (0-1)

    /// Create simple vibration
    static HapticEffect Vibration(Real amplitude, Real duration, Real frequency = 160.0) {
        HapticEffect effect;
        effect.waveform = HapticWaveform::Sine;
        effect.amplitude = amplitude;
        effect.duration = duration;
        effect.frequency = frequency;
        return effect;
    }

    /// Create pulse effect
    static HapticEffect Pulse(Real amplitude, Real on_time, Real off_time, Int32 pulses = 1) {
        HapticEffect effect;
        effect.waveform = HapticWaveform::Square;
        effect.amplitude = amplitude;
        effect.duration = on_time;
        effect.repeat_count = pulses;
        effect.repeat_delay = off_time;
        return effect;
    }

    /// Create rumble effect
    static HapticEffect Rumble(Real intensity, Real duration) {
        HapticEffect effect;
        effect.waveform = HapticWaveform::Noise;
        effect.amplitude = intensity;
        effect.duration = duration;
        effect.frequency_variation = 50.0;
        return effect;
    }

    /// Create impact effect
    static HapticEffect Impact(Real intensity) {
        HapticEffect effect;
        effect.waveform = HapticWaveform::Constant;
        effect.amplitude = intensity;
        effect.duration = 0.05;
        effect.envelope = HapticEnvelope::FadeOut(0.03);
        return effect;
    }
};

/**
 * @brief Haptic effect instance (active playback)
 */
using HapticEffectId = UInt64;
constexpr HapticEffectId INVALID_HAPTIC_EFFECT_ID = 0;

struct HapticEffectState {
    HapticEffectId id{INVALID_HAPTIC_EFFECT_ID};
    Real elapsed_time{0.0};
    Real current_amplitude{0.0};
    Int32 current_repeat{0};
    bool is_playing{false};
    bool is_paused{false};
};

// ============================================================================
// Controller Haptics
// ============================================================================

/**
 * @brief Controller haptic channel
 */
enum class ControllerHapticChannel : UInt8 {
    Primary = 0,        ///< Main haptic motor
    Secondary = 1,      ///< Secondary motor (if available)
    Trigger = 2,        ///< Trigger haptics (PS5 DualSense)
    Trackpad = 3        ///< Trackpad haptics (Steam Controller)
};

/**
 * @brief Controller vibration configuration
 */
struct ControllerVibrationConfig {
    xr::XRHand hand{xr::XRHand::Left};
    ControllerHapticChannel channel{ControllerHapticChannel::Primary};
    HapticEffect effect;
    Real low_frequency_amplitude{0.0};  ///< Low-freq motor (rumble)
    Real high_frequency_amplitude{0.0}; ///< High-freq motor (buzz)
};

/**
 * @brief Adaptive trigger effect types (PS5 DualSense style)
 */
enum class TriggerEffectType : UInt8 {
    None = 0,
    Resistance = 1,     ///< Continuous resistance
    WeaponTrigger = 2,  ///< Weapon trigger simulation
    Vibration = 3,      ///< Trigger vibration
    SectionedResistance = 4,  ///< Variable resistance zones
    Feedback = 5        ///< Position-based feedback
};

/**
 * @brief Adaptive trigger configuration
 */
struct AdaptiveTriggerConfig {
    TriggerEffectType effect_type{TriggerEffectType::None};
    Real start_position{0.0};       ///< Effect start position (0-1)
    Real end_position{1.0};         ///< Effect end position (0-1)
    Real resistance{0.5};           ///< Resistance force (0-1)
    Real frequency{0.0};            ///< Vibration frequency (Hz)
    std::array<Real, 10> zones{};   ///< Zone-based resistance values
};

// ============================================================================
// Haptic Vest System
// ============================================================================

/**
 * @brief Vest actuator zone identifiers
 */
enum class VestZone : UInt8 {
    // Front zones
    ChestLeft = 0,
    ChestCenter = 1,
    ChestRight = 2,
    AbdomenLeft = 3,
    AbdomenCenter = 4,
    AbdomenRight = 5,

    // Back zones
    UpperBackLeft = 6,
    UpperBackCenter = 7,
    UpperBackRight = 8,
    LowerBackLeft = 9,
    LowerBackCenter = 10,
    LowerBackRight = 11,

    // Side zones
    LeftSide = 12,
    RightSide = 13,

    // Shoulder zones
    LeftShoulder = 14,
    RightShoulder = 15,

    // Count
    ZoneCount = 16
};

/**
 * @brief Vest haptic pattern
 */
struct VestHapticPattern {
    std::array<Real, static_cast<size_t>(VestZone::ZoneCount)> intensities{};
    HapticEffect base_effect;
    Real duration{0.1};

    /// Set all zones to same intensity
    void set_all(Real intensity) {
        intensities.fill(intensity);
    }

    /// Set front zones
    void set_front(Real intensity) {
        intensities[static_cast<size_t>(VestZone::ChestLeft)] = intensity;
        intensities[static_cast<size_t>(VestZone::ChestCenter)] = intensity;
        intensities[static_cast<size_t>(VestZone::ChestRight)] = intensity;
        intensities[static_cast<size_t>(VestZone::AbdomenLeft)] = intensity;
        intensities[static_cast<size_t>(VestZone::AbdomenCenter)] = intensity;
        intensities[static_cast<size_t>(VestZone::AbdomenRight)] = intensity;
    }

    /// Set back zones
    void set_back(Real intensity) {
        intensities[static_cast<size_t>(VestZone::UpperBackLeft)] = intensity;
        intensities[static_cast<size_t>(VestZone::UpperBackCenter)] = intensity;
        intensities[static_cast<size_t>(VestZone::UpperBackRight)] = intensity;
        intensities[static_cast<size_t>(VestZone::LowerBackLeft)] = intensity;
        intensities[static_cast<size_t>(VestZone::LowerBackCenter)] = intensity;
        intensities[static_cast<size_t>(VestZone::LowerBackRight)] = intensity;
    }

    /// Create impact pattern from direction
    static VestHapticPattern ImpactFromDirection(const Vec3& direction, Real intensity);

    /// Create heartbeat pattern
    static VestHapticPattern Heartbeat(Real bpm);

    /// Create breathing pattern
    static VestHapticPattern Breathing(Real rate);
};

/**
 * @brief Vest configuration
 */
struct VestConfig {
    UInt8 zone_count{16};
    bool supports_thermal{false};
    Real max_intensity{1.0};
    Real response_time{0.005};  ///< Response time (seconds)
    std::string device_id;
};

// ============================================================================
// Motion Platform / Seat Cueing
// ============================================================================

/**
 * @brief Motion platform degrees of freedom
 */
enum class MotionDOF : UInt8 {
    Surge = 0,      ///< Forward/backward translation
    Sway = 1,       ///< Left/right translation
    Heave = 2,      ///< Up/down translation
    Roll = 3,       ///< Rotation around longitudinal axis
    Pitch = 4,      ///< Rotation around lateral axis
    Yaw = 5,        ///< Rotation around vertical axis
    DOFCount = 6
};

/**
 * @brief Motion platform capabilities
 */
struct MotionPlatformCapabilities {
    std::array<bool, 6> supported_dof{true, true, true, true, true, true};
    std::array<Real, 6> max_displacement{};  ///< Maximum displacement per DOF (m or rad)
    std::array<Real, 6> max_velocity{};      ///< Maximum velocity per DOF
    std::array<Real, 6> max_acceleration{};  ///< Maximum acceleration per DOF
    Real payload_capacity{200.0};            ///< Maximum payload (kg)
    Real update_rate{1000.0};                ///< Update rate (Hz)
    bool supports_washout{true};
    bool supports_vibration{true};
};

/**
 * @brief Motion cueing washout algorithm type
 */
enum class WashoutAlgorithm : UInt8 {
    Classical = 0,          ///< Classical washout filter
    Adaptive = 1,           ///< Adaptive washout
    OptimalControl = 2,     ///< Optimal control based
    Neural = 3              ///< Neural network based
};

/**
 * @brief Motion cueing filter parameters
 */
struct MotionCueingParams {
    WashoutAlgorithm algorithm{WashoutAlgorithm::Classical};

    // High-pass filter parameters (for sustained motion washout)
    std::array<Real, 6> hp_cutoff_freq{0.5, 0.5, 0.5, 0.3, 0.3, 0.3};  ///< Hz
    std::array<Real, 6> hp_damping{0.707, 0.707, 0.707, 0.707, 0.707, 0.707};

    // Low-pass filter parameters (for tilt coordination)
    std::array<Real, 3> lp_cutoff_freq{1.0, 1.0, 2.5};  ///< For roll, pitch, heave
    std::array<Real, 3> lp_damping{0.707, 0.707, 0.707};

    // Scaling factors
    std::array<Real, 6> scaling{1.0, 1.0, 1.0, 1.0, 1.0, 1.0};

    // Tilt coordination gains
    Real tilt_coordination_gain{0.6};
    Real max_tilt_rate{0.5};  ///< rad/s

    /// Create default aircraft cueing parameters
    static MotionCueingParams AircraftDefault();

    /// Create default ground vehicle cueing parameters
    static MotionCueingParams GroundVehicleDefault();

    /// Create default ship/naval cueing parameters
    static MotionCueingParams ShipDefault();
};

/**
 * @brief Motion platform command
 */
struct MotionCommand {
    std::array<Real, 6> position{};     ///< Target position (m or rad)
    std::array<Real, 6> velocity{};     ///< Target velocity
    std::array<Real, 6> acceleration{}; ///< Target acceleration (feed-forward)
    Int64 timestamp_ns{0};
    bool emergency_stop{false};
};

/**
 * @brief Motion platform state
 */
struct MotionPlatformState {
    std::array<Real, 6> position{};
    std::array<Real, 6> velocity{};
    std::array<Real, 6> acceleration{};
    bool is_homed{false};
    bool is_enabled{false};
    bool fault_active{false};
    std::string fault_message;
};

// ============================================================================
// Engine/Vehicle Vibration
// ============================================================================

/**
 * @brief Engine vibration source types
 */
enum class EngineVibrationSource : UInt8 {
    Idle = 0,
    Combustion = 1,         ///< Engine combustion
    Rotation = 2,           ///< Rotational imbalance
    Turbine = 3,            ///< Jet turbine
    Propeller = 4,          ///< Propeller
    Rotor = 5,              ///< Helicopter rotor
    Track = 6,              ///< Tracked vehicle track
    Road = 7,               ///< Road surface interaction
    Aerodynamic = 8,        ///< Aerodynamic buffet
    Hydraulic = 9           ///< Hydraulic system
};

/**
 * @brief Engine vibration parameters
 */
struct EngineVibrationParams {
    EngineVibrationSource source{EngineVibrationSource::Combustion};
    Real rpm{0.0};                  ///< Engine RPM
    Real throttle{0.0};             ///< Throttle position (0-1)
    Real load{0.0};                 ///< Engine load (0-1)
    Real base_frequency{0.0};       ///< Calculated base frequency (Hz)
    Real amplitude_scale{1.0};      ///< Amplitude scaling factor

    // Harmonic content
    std::array<Real, 8> harmonics{1.0, 0.3, 0.1, 0.05, 0.02, 0.01, 0.005, 0.002};

    /// Calculate vibration frequency from RPM and cylinder count
    static Real calculate_combustion_frequency(Real rpm, UInt8 cylinders, bool four_stroke = true) {
        // Combustion frequency = (RPM / 60) * (cylinders / (4-stroke ? 2 : 1))
        Real firing_factor = four_stroke ? 2.0 : 1.0;
        return (rpm / 60.0) * (static_cast<Real>(cylinders) / firing_factor);
    }

    /// Calculate turbine frequency
    static Real calculate_turbine_frequency(Real n1_percent) {
        // N1 typically 0-100%, representing compressor RPM
        // Typical idle ~25%, max ~100% corresponding to ~15,000 RPM
        Real rpm = n1_percent * 150.0;  // Approximate N1% to Hz
        return rpm;
    }

    /// Calculate propeller frequency
    static Real calculate_propeller_frequency(Real rpm, UInt8 blades) {
        return (rpm / 60.0) * static_cast<Real>(blades);
    }
};

/**
 * @brief Vehicle vibration state
 */
struct VehicleVibrationState {
    std::vector<EngineVibrationParams> sources;
    Vec3 total_vibration_acceleration;  ///< Combined vibration (m/s²)
    Real dominant_frequency;             ///< Dominant frequency (Hz)
    Real total_amplitude;                ///< Combined amplitude
};

// ============================================================================
// G-Force Feedback
// ============================================================================

/**
 * @brief G-force feedback configuration
 */
struct GForceConfig {
    // Scaling factors (platform motion to G-force mapping)
    Real longitudinal_scale{1.0};   ///< Forward/aft G scaling
    Real lateral_scale{1.0};        ///< Left/right G scaling
    Real vertical_scale{1.0};       ///< Up/down G scaling

    // Limits
    Real max_sustained_g{3.0};      ///< Maximum sustained G
    Real max_onset_rate{6.0};       ///< Maximum G onset rate (G/s)

    // Tilt coordination for sustained G
    bool use_tilt_coordination{true};
    Real tilt_coordination_rate{0.3};  ///< rad/s

    // G-suit simulation (for high-G aircraft)
    bool simulate_gsuit{false};
    Real gsuit_threshold{2.0};      ///< G-suit activation threshold
    Real gsuit_compression{0.8};    ///< Compression factor

    /// Create fighter jet configuration
    static GForceConfig FighterJet() {
        GForceConfig config;
        config.max_sustained_g = 9.0;
        config.max_onset_rate = 15.0;
        config.simulate_gsuit = true;
        config.gsuit_threshold = 2.5;
        return config;
    }

    /// Create civil aircraft configuration
    static GForceConfig CivilAircraft() {
        GForceConfig config;
        config.max_sustained_g = 2.5;
        config.max_onset_rate = 3.0;
        return config;
    }

    /// Create ground vehicle configuration
    static GForceConfig GroundVehicle() {
        GForceConfig config;
        config.max_sustained_g = 2.0;
        config.max_onset_rate = 5.0;
        config.vertical_scale = 0.5;  // Less vertical emphasis
        return config;
    }
};

/**
 * @brief G-force state
 */
struct GForceState {
    Vec3 body_acceleration;         ///< Acceleration in body frame (m/s²)
    Vec3 g_force;                   ///< G-force vector (multiples of g)
    Real total_g;                   ///< Total G magnitude
    Real onset_rate;                ///< Current G onset rate (G/s)
    Real sustained_g;               ///< Sustained G component
    Real transient_g;               ///< Transient G component
    bool gsuit_active{false};
    Real gsuit_pressure{0.0};
};

// ============================================================================
// Haptic Device Interfaces
// ============================================================================

/**
 * @brief Base haptic device interface
 */
class IHapticDevice {
public:
    virtual ~IHapticDevice() = default;

    /// Get device type
    virtual HapticDeviceType get_type() const = 0;

    /// Get device capabilities
    virtual HapticDeviceCapabilities get_capabilities() const = 0;

    /// Get connection state
    virtual HapticConnectionState get_connection_state() const = 0;

    /// Connect to device
    virtual HapticResult connect() = 0;

    /// Disconnect from device
    virtual HapticResult disconnect() = 0;

    /// Calibrate device
    virtual HapticResult calibrate() = 0;

    /// Check if device is ready
    virtual bool is_ready() const = 0;

    /// Emergency stop all haptic output
    virtual HapticResult emergency_stop() = 0;

    /// Get last error message
    virtual std::string get_last_error() const = 0;
};

/**
 * @brief Haptic controller interface (VR controllers)
 */
class IHapticController : public IHapticDevice {
public:
    /// Play haptic effect on controller
    virtual HapticEffectId play_effect(xr::XRHand hand, const HapticEffect& effect) = 0;

    /// Play controller vibration
    virtual HapticResult play_vibration(const ControllerVibrationConfig& config) = 0;

    /// Stop effect
    virtual HapticResult stop_effect(HapticEffectId effect_id) = 0;

    /// Stop all effects on hand
    virtual HapticResult stop_all(xr::XRHand hand) = 0;

    /// Set adaptive trigger effect
    virtual HapticResult set_adaptive_trigger(xr::XRHand hand,
                                              bool is_left_trigger,
                                              const AdaptiveTriggerConfig& config) = 0;

    /// Get effect state
    virtual std::optional<HapticEffectState> get_effect_state(HapticEffectId effect_id) const = 0;
};

/**
 * @brief Haptic vest interface
 */
class IHapticVest : public IHapticDevice {
public:
    /// Get vest configuration
    virtual VestConfig get_config() const = 0;

    /// Play pattern on vest
    virtual HapticEffectId play_pattern(const VestHapticPattern& pattern) = 0;

    /// Set single zone intensity
    virtual HapticResult set_zone_intensity(VestZone zone, Real intensity) = 0;

    /// Set multiple zone intensities
    virtual HapticResult set_zone_intensities(const std::array<Real, 16>& intensities) = 0;

    /// Set thermal feedback (if supported)
    virtual HapticResult set_thermal(VestZone zone, Real temperature_delta) = 0;

    /// Stop all vest haptics
    virtual HapticResult stop_all() = 0;

    /// Get current zone states
    virtual std::array<Real, 16> get_zone_states() const = 0;
};

/**
 * @brief Motion platform interface
 */
class IMotionPlatform : public IHapticDevice {
public:
    /// Get platform capabilities
    virtual MotionPlatformCapabilities get_platform_capabilities() const = 0;

    /// Get current platform state
    virtual MotionPlatformState get_state() const = 0;

    /// Initialize/home the platform
    virtual HapticResult home() = 0;

    /// Enable platform motion
    virtual HapticResult enable() = 0;

    /// Disable platform motion
    virtual HapticResult disable() = 0;

    /// Send motion command
    virtual HapticResult send_command(const MotionCommand& command) = 0;

    /// Set motion cueing parameters
    virtual HapticResult set_cueing_params(const MotionCueingParams& params) = 0;

    /// Get current cueing parameters
    virtual MotionCueingParams get_cueing_params() const = 0;

    /// Apply vehicle state for motion cueing
    virtual HapticResult apply_vehicle_state(const Vec3& acceleration,
                                             const Vec3& angular_velocity,
                                             const Vec3& angular_acceleration) = 0;

    /// Add vibration overlay
    virtual HapticResult add_vibration(const HapticEffect& effect, Real amplitude = 1.0) = 0;

    /// Park platform (return to neutral)
    virtual HapticResult park() = 0;
};

// ============================================================================
// Haptic Renderer (Orchestration)
// ============================================================================

/**
 * @brief Haptic renderer configuration
 */
struct HapticRendererConfig {
    bool enable_controllers{true};
    bool enable_vest{true};
    bool enable_motion_platform{true};
    Real update_rate{1000.0};       ///< Haptic update rate (Hz)
    Real master_intensity{1.0};     ///< Master intensity scale (0-1)
    bool auto_connect_devices{true};
    GForceConfig g_force_config;
    MotionCueingParams motion_cueing_params;
};

/**
 * @brief Haptic renderer statistics
 */
struct HapticRendererStats {
    UInt64 frames_rendered{0};
    UInt64 effects_played{0};
    Real average_latency_ms{0.0};
    Real max_latency_ms{0.0};
    UInt32 active_effects{0};
    UInt32 connected_devices{0};
};

/**
 * @brief Main haptic renderer interface
 */
class IHapticRenderer {
public:
    virtual ~IHapticRenderer() = default;

    /// Initialize haptic system
    virtual HapticResult initialize(const HapticRendererConfig& config) = 0;

    /// Shutdown haptic system
    virtual HapticResult shutdown() = 0;

    /// Update haptic system (call every frame)
    virtual HapticResult update(Real delta_time) = 0;

    /// Get renderer configuration
    virtual HapticRendererConfig get_config() const = 0;

    /// Set master intensity
    virtual void set_master_intensity(Real intensity) = 0;

    // Device management
    virtual std::vector<std::shared_ptr<IHapticDevice>> get_devices() const = 0;
    virtual std::shared_ptr<IHapticController> get_controller() const = 0;
    virtual std::shared_ptr<IHapticVest> get_vest() const = 0;
    virtual std::shared_ptr<IMotionPlatform> get_motion_platform() const = 0;

    // Effect playback
    virtual HapticEffectId play_effect(HapticDeviceType device, const HapticEffect& effect) = 0;
    virtual HapticResult stop_effect(HapticEffectId effect_id) = 0;
    virtual HapticResult stop_all() = 0;

    // Controller shortcuts
    virtual HapticEffectId play_controller_effect(xr::XRHand hand, const HapticEffect& effect) = 0;
    virtual HapticResult play_controller_vibration(xr::XRHand hand,
                                                   Real low_freq, Real high_freq,
                                                   Real duration) = 0;

    // Vest shortcuts
    virtual HapticEffectId play_vest_pattern(const VestHapticPattern& pattern) = 0;
    virtual HapticResult play_vest_impact(const Vec3& direction, Real intensity) = 0;

    // Motion platform
    virtual HapticResult apply_motion_cueing(const Vec3& linear_accel,
                                             const Vec3& angular_vel,
                                             const Vec3& angular_accel) = 0;

    // Engine vibration
    virtual HapticResult set_engine_vibration(const EngineVibrationParams& params) = 0;
    virtual HapticResult set_vehicle_vibration(const VehicleVibrationState& state) = 0;

    // G-force feedback
    virtual HapticResult apply_g_force(const GForceState& g_state) = 0;
    virtual GForceState calculate_g_force(const Vec3& body_acceleration,
                                          const Vec3& gravity_body_frame) const = 0;

    // Statistics
    virtual HapticRendererStats get_stats() const = 0;

    // Emergency
    virtual HapticResult emergency_stop_all() = 0;
};

// ============================================================================
// Preset Haptic Effects
// ============================================================================

namespace presets {

/**
 * @brief Aircraft-specific haptic presets
 */
namespace aircraft {
    /// Stall buffet
    HapticEffect StallBuffet(Real intensity);

    /// Gear extension/retraction
    HapticEffect GearTransition(bool extending);

    /// Flap movement
    HapticEffect FlapMovement(Real deflection);

    /// Speed brake deployment
    HapticEffect SpeedBrake(Real deployment);

    /// Touchdown impact
    HapticEffect Touchdown(Real vertical_speed);

    /// Engine start sequence
    HapticEffect EngineStart(Real progress);

    /// Afterburner ignition
    HapticEffect AfterburnerIgnition();

    /// Weapons release
    HapticEffect WeaponsRelease();

    /// Aerial refueling contact
    HapticEffect RefuelingContact();

    /// Overspeed warning
    HapticEffect OverspeedWarning();
}

/**
 * @brief Ground vehicle haptic presets
 */
namespace vehicle {
    /// Road surface texture
    HapticEffect RoadTexture(Real roughness);

    /// Brake application
    HapticEffect BrakeApply(Real pressure);

    /// ABS activation
    HapticEffect ABSPulse();

    /// Gear shift
    HapticEffect GearShift();

    /// Collision impact
    HapticEffect Collision(Real severity, const Vec3& direction);

    /// Track rumble (tanks)
    HapticEffect TrackRumble(Real speed);

    /// Firing recoil
    HapticEffect FiringRecoil(Real caliber);
}

/**
 * @brief Naval vessel haptic presets
 */
namespace naval {
    /// Wave motion
    HapticEffect WaveMotion(Real sea_state);

    /// Hull vibration
    HapticEffect HullVibration(Real speed);

    /// Anchor drop
    HapticEffect AnchorDrop();

    /// Weapons firing
    HapticEffect NavalGunFiring(Real caliber);

    /// Torpedo launch
    HapticEffect TorpedoLaunch();

    /// Depth charge
    HapticEffect DepthCharge(Real distance);
}

/**
 * @brief Helicopter haptic presets
 */
namespace helicopter {
    /// Rotor vibration
    HapticEffect RotorVibration(Real rpm);

    /// Ground resonance
    HapticEffect GroundResonance();

    /// Vortex ring state
    HapticEffect VortexRingState();

    /// Autorotation
    HapticEffect Autorotation();

    /// Hard landing
    HapticEffect HardLanding(Real vertical_speed);
}

} // namespace presets

// ============================================================================
// Factory Functions
// ============================================================================

/**
 * @brief Create haptic renderer with default configuration
 */
std::unique_ptr<IHapticRenderer> create_haptic_renderer();

/**
 * @brief Create haptic renderer with custom configuration
 */
std::unique_ptr<IHapticRenderer> create_haptic_renderer(const HapticRendererConfig& config);

/**
 * @brief Create mock haptic renderer for testing
 */
std::unique_ptr<IHapticRenderer> create_mock_haptic_renderer();

} // namespace jaguar::haptics
