#pragma once
/**
 * @file training.h
 * @brief Training module types and interfaces for simulation-based training
 *
 * This file defines the training system for JaguarEngine, providing
 * cockpit simulators, ship bridge simulators, control tower modules,
 * after-action review, and training scenario scripting capabilities.
 * Designed for professional military and civilian training applications.
 */

#include "jaguar/core/types.h"
#include "jaguar/xr/xr_types.h"
#include "jaguar/xr/haptics.h"
#include "jaguar/xr/spatial_audio.h"
#include <array>
#include <vector>
#include <memory>
#include <functional>
#include <string>
#include <optional>
#include <chrono>
#include <variant>
#include <map>
#include <unordered_map>

namespace jaguar::training {

// ============================================================================
// Forward Declarations
// ============================================================================

class ITrainingModule;
class ICockpitModule;
class IBridgeModule;
class IControlTowerModule;
class IAfterActionReview;
class IScenarioEngine;
class ITrainingSession;

// ============================================================================
// Training Result Types
// ============================================================================

/**
 * @brief Result codes for training operations
 */
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

/**
 * @brief Convert TrainingResult to string for logging
 */
const char* training_result_to_string(TrainingResult result);

/**
 * @brief Check if TrainingResult indicates success
 */
inline bool training_succeeded(TrainingResult result) {
    return result == TrainingResult::Success;
}

// ============================================================================
// Training Domain Types
// ============================================================================

/**
 * @brief Training domain categories
 */
enum class TrainingDomain : UInt8 {
    Aviation = 0,           ///< Fixed-wing aircraft
    Rotorcraft = 1,         ///< Helicopters
    Naval = 2,              ///< Ships and submarines
    GroundVehicle = 3,      ///< Tanks, APCs, trucks
    AirTrafficControl = 4,  ///< ATC operations
    Maritime = 5,           ///< Port operations
    Emergency = 6,          ///< Emergency response
    Custom = 255
};

/**
 * @brief Training difficulty levels
 */
enum class DifficultyLevel : UInt8 {
    Tutorial = 0,
    Beginner = 1,
    Intermediate = 2,
    Advanced = 3,
    Expert = 4,
    Instructor = 5
};

/**
 * @brief Weather condition presets for training
 */
enum class WeatherPreset : UInt8 {
    Clear = 0,
    FewClouds = 1,
    Scattered = 2,
    Broken = 3,
    Overcast = 4,
    Rain = 5,
    Thunderstorm = 6,
    Snow = 7,
    Fog = 8,
    LowVisibility = 9,
    Turbulence = 10,
    Icing = 11,
    Crosswind = 12,
    Custom = 255
};

/**
 * @brief Time of day presets
 */
enum class TimeOfDay : UInt8 {
    Dawn = 0,
    Morning = 1,
    Noon = 2,
    Afternoon = 3,
    Dusk = 4,
    Night = 5,
    NightNVG = 6,       ///< Night with NVG
    Custom = 255
};

// ============================================================================
// Cockpit Instrument Types
// ============================================================================

/**
 * @brief Cockpit instrument categories
 */
enum class InstrumentType : UInt8 {
    // Primary Flight Display
    Airspeed = 0,
    Altitude = 1,
    AttitudeIndicator = 2,
    HeadingIndicator = 3,
    VerticalSpeed = 4,
    TurnCoordinator = 5,

    // Navigation
    HSI = 10,               ///< Horizontal Situation Indicator
    RMI = 11,               ///< Radio Magnetic Indicator
    DME = 12,               ///< Distance Measuring Equipment
    ADF = 13,               ///< Automatic Direction Finder
    GPS = 14,
    FMS = 15,               ///< Flight Management System

    // Engine
    Tachometer = 20,
    Manifold = 21,
    FuelFlow = 22,
    FuelQuantity = 23,
    OilPressure = 24,
    OilTemperature = 25,
    EGT = 26,               ///< Exhaust Gas Temperature
    CHT = 27,               ///< Cylinder Head Temperature
    N1 = 28,                ///< Fan/Compressor RPM
    N2 = 29,                ///< Core/Turbine RPM
    ITT = 30,               ///< Interstage Turbine Temperature

    // Systems
    Electrical = 40,
    Hydraulic = 41,
    Pneumatic = 42,
    Fuel = 43,
    Environmental = 44,
    Warning = 45,
    Annunciator = 46,

    // Radio
    ComRadio = 50,
    NavRadio = 51,
    Transponder = 52,
    TCAS = 53,              ///< Traffic Collision Avoidance
    GPWS = 54,              ///< Ground Proximity Warning

    // Autopilot
    FlightDirector = 60,
    Autopilot = 61,
    Autothrottle = 62,

    // Weapons (Military)
    WeaponsPanel = 70,
    RadarDisplay = 71,
    HUD = 72,               ///< Heads-Up Display
    MFD = 73,               ///< Multi-Function Display

    Custom = 255
};

/**
 * @brief Instrument state
 */
struct InstrumentState {
    InstrumentType type{InstrumentType::Custom};
    std::string name;
    Real value{0.0};
    Real min_value{0.0};
    Real max_value{100.0};
    bool is_active{true};
    bool has_warning{false};
    bool has_failure{false};
    std::string units;
};

/**
 * @brief Cockpit control types
 */
enum class ControlType : UInt8 {
    // Flight Controls
    Yoke = 0,
    Stick = 1,
    Throttle = 2,
    Rudder = 3,
    Collective = 4,         ///< Helicopter
    Cyclic = 5,             ///< Helicopter

    // Secondary Controls
    Flaps = 10,
    Gear = 11,
    SpeedBrake = 12,
    Trim = 13,
    Mixture = 14,
    Propeller = 15,

    // Switches
    MasterSwitch = 20,
    AvionicsMaster = 21,
    FuelPump = 22,
    LandingLights = 23,
    Strobe = 24,
    Beacon = 25,
    PitotHeat = 26,
    DeIce = 27,

    // Circuit Breakers
    CircuitBreaker = 30,

    Custom = 255
};

/**
 * @brief Control input state
 */
struct ControlState {
    ControlType type{ControlType::Custom};
    std::string name;
    Real position{0.0};     ///< -1 to 1 for axes, 0 or 1 for switches
    Real force{0.0};        ///< Force feedback (if supported)
    bool is_switch{false};
    bool is_momentary{false};
    bool is_active{false};
};

// ============================================================================
// Ship Bridge Types
// ============================================================================

/**
 * @brief Ship bridge station types
 */
enum class BridgeStation : UInt8 {
    Helm = 0,               ///< Steering/navigation
    Engineering = 1,        ///< Engine controls
    Radar = 2,              ///< Radar operator
    Communications = 3,     ///< Radio operator
    Weapons = 4,            ///< Combat systems (naval)
    CIC = 5,                ///< Combat Information Center
    Captain = 6,            ///< Command station
    Navigation = 7,         ///< Navigation officer
    Custom = 255
};

/**
 * @brief Ship navigation instruments
 */
enum class NavInstrumentType : UInt8 {
    Compass = 0,
    Gyrocompass = 1,
    SpeedLog = 2,
    Fathometer = 3,         ///< Depth sounder
    Radar = 4,
    ECDIS = 5,              ///< Electronic Chart Display
    AIS = 6,                ///< Automatic Identification System
    GPS = 7,
    WindIndicator = 8,
    Rudder = 9,
    RPM = 10,
    Custom = 255
};

/**
 * @brief Ship control types
 */
enum class ShipControlType : UInt8 {
    Wheel = 0,              ///< Steering wheel
    Helm = 1,               ///< Helm station
    Throttle = 2,           ///< Engine order telegraph
    BowThruster = 3,
    SternThruster = 4,
    Anchor = 5,
    Mooring = 6,
    Custom = 255
};

// ============================================================================
// Control Tower Types
// ============================================================================

/**
 * @brief ATC position types
 */
enum class ATCPosition : UInt8 {
    Ground = 0,             ///< Ground control
    Tower = 1,              ///< Tower control
    Approach = 2,           ///< Approach control
    Departure = 3,          ///< Departure control
    Center = 4,             ///< En-route center
    TRACON = 5,             ///< Terminal radar
    Clearance = 6,          ///< Clearance delivery
    Custom = 255
};

/**
 * @brief Aircraft track information
 */
struct AircraftTrack {
    std::string callsign;
    std::string aircraft_type;
    Vec3 position;
    Vec3 velocity;
    Real altitude;          ///< feet
    Real heading;           ///< degrees
    Real ground_speed;      ///< knots
    Real vertical_rate;     ///< feet per minute
    std::string squawk;     ///< Transponder code
    bool is_departing{false};
    bool is_arriving{false};
    bool is_vfr{false};
    bool has_emergency{false};
    Int64 timestamp_ns{0};
};

/**
 * @brief Runway status
 */
struct RunwayStatus {
    std::string designator; ///< e.g., "27L"
    bool is_active{false};
    bool is_occupied{false};
    bool is_closed{false};
    std::string current_aircraft;
    Real crosswind_component;
    Real headwind_component;
};

// ============================================================================
// Training Objectives
// ============================================================================

/**
 * @brief Training objective types
 */
enum class ObjectiveType : UInt8 {
    // Flight objectives
    Takeoff = 0,
    Landing = 1,
    GoAround = 2,
    Approach = 3,
    Holding = 4,
    Navigation = 5,
    Emergency = 6,

    // Maneuvers
    SteepTurn = 10,
    Stall = 11,
    SlowFlight = 12,
    Spin = 13,
    AcrobaticManeuver = 14,

    // Procedures
    Checklist = 20,
    Communication = 21,
    SystemsManagement = 22,
    DecisionMaking = 23,
    CrewCoordination = 24,

    // Navigation
    VORNavigation = 30,
    ILSApproach = 31,
    VFRNavigation = 32,
    GPSNavigation = 33,

    // Custom
    Custom = 255
};

/**
 * @brief Objective completion status
 */
enum class ObjectiveStatus : UInt8 {
    NotStarted = 0,
    InProgress = 1,
    Completed = 2,
    Failed = 3,
    Skipped = 4
};

/**
 * @brief Training objective definition
 */
struct TrainingObjective {
    UInt32 id{0};
    ObjectiveType type{ObjectiveType::Custom};
    std::string name;
    std::string description;
    ObjectiveStatus status{ObjectiveStatus::NotStarted};
    Real score{0.0};        ///< 0-100
    Real weight{1.0};       ///< Weight in overall score
    Real time_limit{0.0};   ///< Seconds (0 = no limit)
    Real elapsed_time{0.0};
    std::vector<std::string> criteria;
    std::vector<std::string> feedback;
    bool is_mandatory{true};
    bool is_graded{true};
};

// ============================================================================
// After-Action Review Types
// ============================================================================

/**
 * @brief Event types for recording
 */
enum class TrainingEventType : UInt8 {
    // Session
    SessionStart = 0,
    SessionEnd = 1,
    Checkpoint = 2,
    Bookmark = 3,

    // Performance
    ObjectiveStarted = 10,
    ObjectiveCompleted = 11,
    ObjectiveFailed = 12,
    ErrorMade = 13,
    ExcellentPerformance = 14,

    // Aircraft state
    StateChange = 20,
    Stall = 21,
    Overspeed = 22,
    Exceedance = 23,
    Collision = 24,
    Crash = 25,

    // Systems
    SystemFailure = 30,
    SystemActivation = 31,
    ChecklistItem = 32,

    // Communication
    RadioTransmission = 40,
    Instruction = 41,
    Readback = 42,

    // Weather/Environment
    WeatherChange = 50,
    TimeChange = 51,

    // Custom
    InstructorNote = 60,
    Custom = 255
};

/**
 * @brief Recorded training event
 */
struct TrainingEvent {
    UInt64 id{0};
    TrainingEventType type{TrainingEventType::Custom};
    std::string name;
    std::string description;
    Real timestamp{0.0};    ///< Seconds from session start
    Vec3 position;          ///< World position at event
    Real severity{0.0};     ///< 0-1, importance level
    std::string data;       ///< Additional JSON data
};

/**
 * @brief State snapshot for replay
 */
struct StateSnapshot {
    Real timestamp{0.0};

    // Position/Orientation
    Vec3 position;
    Quat orientation;
    Vec3 velocity;
    Vec3 angular_velocity;

    // Aircraft state
    Real airspeed{0.0};
    Real altitude{0.0};
    Real heading{0.0};
    Real pitch{0.0};
    Real roll{0.0};
    Real vertical_speed{0.0};

    // Control inputs
    Real elevator{0.0};
    Real aileron{0.0};
    Real rudder{0.0};
    Real throttle{0.0};
    Real flaps{0.0};
    bool gear_down{false};

    // Engine state
    std::array<Real, 4> engine_rpm{};
    std::array<Real, 4> engine_throttle{};

    // Systems
    std::vector<InstrumentState> instruments;
};

/**
 * @brief Playback mode
 */
enum class PlaybackMode : UInt8 {
    Stopped = 0,
    Playing = 1,
    Paused = 2,
    FastForward = 3,
    Rewind = 4,
    FrameStep = 5
};

/**
 * @brief AAR playback controls
 */
struct PlaybackState {
    PlaybackMode mode{PlaybackMode::Stopped};
    Real current_time{0.0};
    Real duration{0.0};
    Real playback_speed{1.0};
    bool loop_enabled{false};
    Real loop_start{0.0};
    Real loop_end{0.0};
};

// ============================================================================
// Scenario Scripting Types
// ============================================================================

/**
 * @brief Script trigger types
 */
enum class TriggerType : UInt8 {
    // Time-based
    OnTime = 0,
    AfterDelay = 1,
    AtInterval = 2,

    // Location-based
    OnEnterArea = 10,
    OnExitArea = 11,
    OnApproachPoint = 12,
    OnAltitude = 13,

    // Event-based
    OnEvent = 20,
    OnObjective = 21,
    OnCondition = 22,

    // User input
    OnKeyPress = 30,
    OnRadioCall = 31,
    OnInstructorCommand = 32,

    Custom = 255
};

/**
 * @brief Script action types
 */
enum class ActionType : UInt8 {
    // Environment
    SetWeather = 0,
    SetTime = 1,
    SetVisibility = 2,
    SetWind = 3,

    // Traffic
    SpawnAircraft = 10,
    RemoveAircraft = 11,
    SetAircraftRoute = 12,
    TriggerEmergency = 13,

    // Systems
    InjectFailure = 20,
    RestoreSystem = 21,
    SetInstrument = 22,

    // Objectives
    StartObjective = 30,
    CompleteObjective = 31,
    FailObjective = 32,
    AddObjective = 33,

    // Feedback
    ShowMessage = 40,
    PlayAudio = 41,
    PlayHaptic = 42,
    Highlight = 43,

    // Flow control
    PauseScenario = 50,
    ResumeScenario = 51,
    EndScenario = 52,
    JumpToCheckpoint = 53,

    // ATC
    SendATCInstruction = 60,
    ExpectReadback = 61,

    Custom = 255
};

/**
 * @brief Trigger condition
 */
struct ScriptTrigger {
    UInt32 id{0};
    TriggerType type{TriggerType::OnTime};
    std::string name;
    Real parameter{0.0};    ///< Time or distance
    Vec3 location;          ///< For location triggers
    Real radius{0.0};       ///< For area triggers
    std::string condition;  ///< Expression for conditions
    bool is_repeatable{false};
    bool has_fired{false};
};

/**
 * @brief Script action
 */
struct ScriptAction {
    UInt32 id{0};
    ActionType type{ActionType::ShowMessage};
    std::string name;
    std::string parameters;  ///< JSON parameters
    Real delay{0.0};        ///< Delay after trigger
    bool is_async{false};
};

/**
 * @brief Script rule (trigger + actions)
 */
struct ScriptRule {
    UInt32 id{0};
    std::string name;
    ScriptTrigger trigger;
    std::vector<ScriptAction> actions;
    bool is_enabled{true};
};

/**
 * @brief Training scenario definition
 */
struct ScenarioDefinition {
    std::string id;
    std::string name;
    std::string description;
    TrainingDomain domain{TrainingDomain::Aviation};
    DifficultyLevel difficulty{DifficultyLevel::Intermediate};

    // Initial conditions
    Vec3 start_position;
    Quat start_orientation;
    Real start_altitude{0.0};
    Real start_airspeed{0.0};
    Real start_heading{0.0};
    WeatherPreset weather{WeatherPreset::Clear};
    TimeOfDay time{TimeOfDay::Morning};

    // Content
    std::vector<TrainingObjective> objectives;
    std::vector<ScriptRule> rules;
    std::vector<std::string> checkpoints;

    // Metadata
    Real estimated_duration{0.0};
    std::vector<std::string> prerequisites;
    std::vector<std::string> tags;
    std::string author;
    std::string version;
};

// ============================================================================
// Grading and Scoring
// ============================================================================

/**
 * @brief Grade level
 */
enum class Grade : UInt8 {
    Excellent = 0,      ///< 90-100%
    Good = 1,           ///< 80-89%
    Satisfactory = 2,   ///< 70-79%
    Marginal = 3,       ///< 60-69%
    Unsatisfactory = 4, ///< Below 60%
    Incomplete = 5
};

/**
 * @brief Scoring criteria
 */
struct ScoringCriteria {
    std::string name;
    std::string description;
    Real weight{1.0};
    Real target_value{0.0};
    Real tolerance{0.0};
    Real actual_value{0.0};
    Real score{0.0};
};

/**
 * @brief Session grade report
 */
struct GradeReport {
    Grade overall_grade{Grade::Incomplete};
    Real overall_score{0.0};
    Real total_time{0.0};

    std::vector<ScoringCriteria> criteria;
    std::vector<TrainingObjective> objectives;
    std::vector<TrainingEvent> significant_events;

    std::string instructor_comments;
    std::vector<std::string> strengths;
    std::vector<std::string> areas_for_improvement;
    std::vector<std::string> recommendations;

    bool is_passed{false};
    Int64 completion_timestamp{0};
};

// ============================================================================
// Training Module Interfaces
// ============================================================================

/**
 * @brief Base training module interface
 */
class ITrainingModule {
public:
    virtual ~ITrainingModule() = default;

    /// Get module domain
    virtual TrainingDomain get_domain() const = 0;

    /// Get module name
    virtual const std::string& get_name() const = 0;

    /// Initialize module
    virtual TrainingResult initialize() = 0;

    /// Shutdown module
    virtual TrainingResult shutdown() = 0;

    /// Update module state
    virtual TrainingResult update(Real delta_time) = 0;

    /// Check if module is ready
    virtual bool is_ready() const = 0;

    /// Get last error message
    virtual std::string get_last_error() const = 0;
};

/**
 * @brief Cockpit simulator module interface
 */
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
    virtual std::optional<ControlState> get_control(ControlType type) const = 0;
    virtual TrainingResult set_control_position(ControlType type, Real position) = 0;
    virtual TrainingResult set_control_force(ControlType type, Real force) = 0;

    // Flight state
    virtual Real get_airspeed() const = 0;
    virtual Real get_altitude() const = 0;
    virtual Real get_heading() const = 0;
    virtual Real get_vertical_speed() const = 0;
    virtual Vec3 get_attitude() const = 0;  // pitch, roll, yaw

    // Procedures
    virtual TrainingResult start_checklist(const std::string& checklist_id) = 0;
    virtual TrainingResult complete_checklist_item(UInt32 item_index) = 0;
    virtual std::vector<std::string> get_active_checklists() const = 0;

    // Systems
    virtual TrainingResult set_system_state(const std::string& system, bool active) = 0;
    virtual bool get_system_state(const std::string& system) const = 0;

    // Integration
    virtual void bind_xr_session(std::shared_ptr<xr::IXRRuntime> xr_runtime) = 0;
    virtual void bind_haptics(std::shared_ptr<haptics::IHapticRenderer> haptics) = 0;
    virtual void bind_audio(std::shared_ptr<audio::ISpatialAudioRenderer> audio) = 0;
};

/**
 * @brief Ship bridge simulator module interface
 */
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
    virtual Vec3 get_position() const = 0;  // lat/lon/alt
    virtual std::vector<AircraftTrack> get_radar_contacts() const = 0;

    // Controls
    virtual TrainingResult set_helm(Real angle) = 0;  // -35 to +35 degrees
    virtual TrainingResult set_throttle(Real power) = 0;  // -1 to 1 (reverse to full)
    virtual TrainingResult set_thruster(ShipControlType thruster, Real power) = 0;

    // Equipment
    virtual std::optional<InstrumentState> get_nav_instrument(NavInstrumentType type) const = 0;
    virtual TrainingResult activate_radar() = 0;
    virtual TrainingResult deactivate_radar() = 0;

    // Integration
    virtual void bind_xr_session(std::shared_ptr<xr::IXRRuntime> xr_runtime) = 0;
    virtual void bind_haptics(std::shared_ptr<haptics::IHapticRenderer> haptics) = 0;
};

/**
 * @brief Control tower module interface
 */
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

    // Integration
    virtual void bind_xr_session(std::shared_ptr<xr::IXRRuntime> xr_runtime) = 0;
};

/**
 * @brief After-action review interface
 */
class IAfterActionReview {
public:
    virtual ~IAfterActionReview() = default;

    // Recording
    virtual TrainingResult start_recording() = 0;
    virtual TrainingResult stop_recording() = 0;
    virtual TrainingResult add_bookmark(const std::string& name) = 0;
    virtual TrainingResult add_instructor_note(const std::string& note) = 0;
    virtual bool is_recording() const = 0;

    // Events
    virtual TrainingResult record_event(const TrainingEvent& event) = 0;
    virtual std::vector<TrainingEvent> get_events() const = 0;
    virtual std::vector<TrainingEvent> get_events_in_range(Real start_time, Real end_time) const = 0;

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

/**
 * @brief Scenario engine interface
 */
class IScenarioEngine {
public:
    virtual ~IScenarioEngine() = default;

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
    virtual std::vector<ScriptRule> get_rules() const = 0;

    // Triggers
    virtual TrainingResult fire_trigger(UInt32 trigger_id) = 0;
    virtual TrainingResult evaluate_triggers() = 0;

    // Actions
    virtual TrainingResult execute_action(const ScriptAction& action) = 0;

    // Checkpoints
    virtual std::vector<std::string> get_checkpoints() const = 0;
    virtual TrainingResult jump_to_checkpoint(const std::string& checkpoint_id) = 0;
    virtual TrainingResult save_checkpoint(const std::string& checkpoint_id) = 0;

    // Update
    virtual TrainingResult update(Real delta_time) = 0;
};

/**
 * @brief Training session interface
 */
class ITrainingSession {
public:
    virtual ~ITrainingSession() = default;

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

    // Trainee info
    virtual std::string get_trainee_id() const = 0;
    virtual std::string get_session_id() const = 0;

    // Statistics
    virtual UInt32 get_total_objectives() const = 0;
    virtual UInt32 get_completed_objectives() const = 0;
    virtual Real get_current_score() const = 0;

    // Update
    virtual TrainingResult update(Real delta_time) = 0;
};

// ============================================================================
// Factory Functions
// ============================================================================

/**
 * @brief Create cockpit module for specific aircraft type
 */
std::unique_ptr<ICockpitModule> create_cockpit_module(const std::string& aircraft_type);

/**
 * @brief Create generic cockpit module
 */
std::unique_ptr<ICockpitModule> create_generic_cockpit_module();

/**
 * @brief Create bridge module for specific ship type
 */
std::unique_ptr<IBridgeModule> create_bridge_module(const std::string& ship_type);

/**
 * @brief Create generic bridge module
 */
std::unique_ptr<IBridgeModule> create_generic_bridge_module();

/**
 * @brief Create control tower module
 */
std::unique_ptr<IControlTowerModule> create_tower_module(const std::string& airport_icao);

/**
 * @brief Create after-action review system
 */
std::unique_ptr<IAfterActionReview> create_aar();

/**
 * @brief Create scenario engine
 */
std::unique_ptr<IScenarioEngine> create_scenario_engine();

/**
 * @brief Create training session
 */
std::unique_ptr<ITrainingSession> create_training_session();

/**
 * @brief Create mock training session for testing
 */
std::unique_ptr<ITrainingSession> create_mock_training_session();

} // namespace jaguar::training
