/**
 * @file training.cpp
 * @brief Training module system implementation
 *
 * Implements cockpit simulators, ship bridge simulators, control tower modules,
 * after-action review, and training scenario scripting.
 */

#include "jaguar/xr/training.h"
#define _USE_MATH_DEFINES
#include <cmath>
#include <algorithm>
#include <sstream>
#include <iomanip>
#include <random>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace jaguar::training {

// ============================================================================
// Result String Conversion
// ============================================================================

const char* training_result_to_string(TrainingResult result) {
    switch (result) {
        case TrainingResult::Success: return "Success";
        case TrainingResult::NotInitialized: return "Not Initialized";
        case TrainingResult::SessionNotActive: return "Session Not Active";
        case TrainingResult::ScenarioNotLoaded: return "Scenario Not Loaded";
        case TrainingResult::InvalidParameter: return "Invalid Parameter";
        case TrainingResult::ModuleNotAvailable: return "Module Not Available";
        case TrainingResult::InstrumentNotFound: return "Instrument Not Found";
        case TrainingResult::ObjectiveNotFound: return "Objective Not Found";
        case TrainingResult::RecordingNotStarted: return "Recording Not Started";
        case TrainingResult::PlaybackError: return "Playback Error";
        case TrainingResult::ScriptError: return "Script Error";
        case TrainingResult::ResourceNotFound: return "Resource Not Found";
        case TrainingResult::PermissionDenied: return "Permission Denied";
        case TrainingResult::OutOfMemory: return "Out of Memory";
        case TrainingResult::InternalError: return "Internal Error";
        default: return "Unknown Error";
    }
}

// ============================================================================
// Mock Cockpit Module Implementation
// ============================================================================

class MockCockpitModule : public ICockpitModule {
public:
    MockCockpitModule(const std::string& aircraft_type)
        : aircraft_type_(aircraft_type) {
        initialize_instruments();
        initialize_controls();
    }

    TrainingDomain get_domain() const override { return TrainingDomain::Aviation; }
    const std::string& get_name() const override { return name_; }

    TrainingResult initialize() override {
        is_initialized_ = true;
        return TrainingResult::Success;
    }

    TrainingResult shutdown() override {
        is_initialized_ = false;
        return TrainingResult::Success;
    }

    TrainingResult update(Real delta_time) override {
        if (!is_initialized_) return TrainingResult::NotInitialized;

        elapsed_time_ += delta_time;

        // Simulate basic flight physics
        update_flight_state(delta_time);

        // Update instrument readings
        update_instruments();

        return TrainingResult::Success;
    }

    bool is_ready() const override { return is_initialized_; }
    std::string get_last_error() const override { return last_error_; }

    // Instruments
    std::vector<InstrumentState> get_instruments() const override {
        return instruments_;
    }

    std::optional<InstrumentState> get_instrument(InstrumentType type) const override {
        for (const auto& inst : instruments_) {
            if (inst.type == type) return inst;
        }
        return std::nullopt;
    }

    TrainingResult set_instrument_value(InstrumentType type, Real value) override {
        for (auto& inst : instruments_) {
            if (inst.type == type) {
                inst.value = std::clamp(value, inst.min_value, inst.max_value);
                return TrainingResult::Success;
            }
        }
        return TrainingResult::InstrumentNotFound;
    }

    TrainingResult inject_instrument_failure(InstrumentType type) override {
        for (auto& inst : instruments_) {
            if (inst.type == type) {
                inst.has_failure = true;
                inst.is_active = false;
                return TrainingResult::Success;
            }
        }
        return TrainingResult::InstrumentNotFound;
    }

    TrainingResult restore_instrument(InstrumentType type) override {
        for (auto& inst : instruments_) {
            if (inst.type == type) {
                inst.has_failure = false;
                inst.is_active = true;
                return TrainingResult::Success;
            }
        }
        return TrainingResult::InstrumentNotFound;
    }

    // Controls
    std::vector<ControlState> get_controls() const override {
        return controls_;
    }

    std::optional<ControlState> get_control(ControlType type) const override {
        for (const auto& ctrl : controls_) {
            if (ctrl.type == type) return ctrl;
        }
        return std::nullopt;
    }

    TrainingResult set_control_position(ControlType type, Real position) override {
        for (auto& ctrl : controls_) {
            if (ctrl.type == type) {
                ctrl.position = std::clamp(position, -1.0, 1.0);
                ctrl.is_active = true;
                return TrainingResult::Success;
            }
        }
        return TrainingResult::InvalidParameter;
    }

    TrainingResult set_control_force(ControlType type, Real force) override {
        for (auto& ctrl : controls_) {
            if (ctrl.type == type) {
                ctrl.force = force;
                return TrainingResult::Success;
            }
        }
        return TrainingResult::InvalidParameter;
    }

    // Flight state
    Real get_airspeed() const override { return airspeed_; }
    Real get_altitude() const override { return altitude_; }
    Real get_heading() const override { return heading_; }
    Real get_vertical_speed() const override { return vertical_speed_; }

    Vec3 get_attitude() const override {
        return Vec3{pitch_, roll_, heading_};
    }

    // Procedures
    TrainingResult start_checklist(const std::string& checklist_id) override {
        active_checklists_.push_back(checklist_id);
        return TrainingResult::Success;
    }

    TrainingResult complete_checklist_item(UInt32 item_index) override {
        (void)item_index;
        return TrainingResult::Success;
    }

    std::vector<std::string> get_active_checklists() const override {
        return active_checklists_;
    }

    // Systems
    TrainingResult set_system_state(const std::string& system, bool active) override {
        system_states_[system] = active;
        return TrainingResult::Success;
    }

    bool get_system_state(const std::string& system) const override {
        auto it = system_states_.find(system);
        return it != system_states_.end() ? it->second : false;
    }

    // Integration
    void bind_xr_session(std::shared_ptr<xr::IXRRuntime> xr_runtime) override {
        xr_runtime_ = xr_runtime;
    }

    void bind_haptics(std::shared_ptr<haptics::IHapticRenderer> haptics) override {
        haptics_ = haptics;
    }

    void bind_audio(std::shared_ptr<audio::ISpatialAudioRenderer> audio) override {
        audio_ = audio;
    }

private:
    void initialize_instruments() {
        // Primary flight instruments
        instruments_.push_back({InstrumentType::Airspeed, "Airspeed Indicator", 0, 0, 300, true, false, false, "kts"});
        instruments_.push_back({InstrumentType::Altitude, "Altimeter", 0, 0, 50000, true, false, false, "ft"});
        instruments_.push_back({InstrumentType::AttitudeIndicator, "Attitude Indicator", 0, -90, 90, true, false, false, "deg"});
        instruments_.push_back({InstrumentType::HeadingIndicator, "Heading Indicator", 0, 0, 360, true, false, false, "deg"});
        instruments_.push_back({InstrumentType::VerticalSpeed, "VSI", 0, -6000, 6000, true, false, false, "fpm"});
        instruments_.push_back({InstrumentType::TurnCoordinator, "Turn Coordinator", 0, -2, 2, true, false, false, ""});

        // Engine instruments
        instruments_.push_back({InstrumentType::Tachometer, "Tachometer", 0, 0, 3000, true, false, false, "rpm"});
        instruments_.push_back({InstrumentType::Manifold, "Manifold Pressure", 0, 10, 40, true, false, false, "inHg"});
        instruments_.push_back({InstrumentType::FuelFlow, "Fuel Flow", 0, 0, 50, true, false, false, "gph"});
        instruments_.push_back({InstrumentType::OilPressure, "Oil Pressure", 0, 0, 100, true, false, false, "psi"});
        instruments_.push_back({InstrumentType::OilTemperature, "Oil Temperature", 0, 0, 300, true, false, false, "F"});

        // Navigation
        instruments_.push_back({InstrumentType::HSI, "HSI", 0, 0, 360, true, false, false, "deg"});
        instruments_.push_back({InstrumentType::DME, "DME", 0, 0, 300, true, false, false, "nm"});
    }

    void initialize_controls() {
        controls_.push_back({ControlType::Yoke, "Control Yoke", 0, 0, false, false, true});
        controls_.push_back({ControlType::Throttle, "Throttle", 0, 0, false, false, true});
        controls_.push_back({ControlType::Rudder, "Rudder Pedals", 0, 0, false, false, true});
        controls_.push_back({ControlType::Flaps, "Flaps", 0, 0, false, false, true});
        controls_.push_back({ControlType::Gear, "Landing Gear", 1, 0, true, false, true});
        controls_.push_back({ControlType::Trim, "Elevator Trim", 0, 0, false, false, true});
        controls_.push_back({ControlType::Mixture, "Mixture", 1, 0, false, false, true});
        controls_.push_back({ControlType::MasterSwitch, "Master Switch", 1, 0, true, false, true});
    }

    void update_flight_state(Real dt) {
        // Get control inputs
        Real elevator = 0, aileron = 0, throttle = 0.5;
        for (const auto& ctrl : controls_) {
            if (ctrl.type == ControlType::Yoke) {
                elevator = ctrl.position;
            } else if (ctrl.type == ControlType::Throttle) {
                throttle = (ctrl.position + 1.0) / 2.0;  // -1..1 to 0..1
            }
        }

        // Simple flight model
        Real target_airspeed = 60 + throttle * 140;  // 60-200 kts range
        airspeed_ += (target_airspeed - airspeed_) * dt * 0.5;

        // Pitch affects vertical speed
        Real pitch_rate = elevator * 5.0;  // 5 deg/s max pitch rate
        pitch_ += pitch_rate * dt;
        pitch_ = std::clamp(pitch_, -30.0, 30.0);

        // Vertical speed from pitch
        vertical_speed_ = std::sin(pitch_ * M_PI / 180.0) * airspeed_ * 101.269;  // kts to fpm conversion factor

        // Altitude from vertical speed
        altitude_ += vertical_speed_ * dt / 60.0;  // fpm to ft/s
        altitude_ = std::max(0.0, altitude_);

        // Heading changes with aileron
        (void)aileron;
    }

    void update_instruments() {
        for (auto& inst : instruments_) {
            if (!inst.is_active || inst.has_failure) continue;

            switch (inst.type) {
                case InstrumentType::Airspeed:
                    inst.value = airspeed_;
                    break;
                case InstrumentType::Altitude:
                    inst.value = altitude_;
                    break;
                case InstrumentType::AttitudeIndicator:
                    inst.value = pitch_;
                    break;
                case InstrumentType::HeadingIndicator:
                    inst.value = heading_;
                    break;
                case InstrumentType::VerticalSpeed:
                    inst.value = vertical_speed_;
                    break;
                default:
                    break;
            }
        }
    }

    std::string aircraft_type_;
    std::string name_{"Mock Cockpit Module"};
    std::string last_error_;
    bool is_initialized_{false};
    Real elapsed_time_{0.0};

    // Flight state
    Real airspeed_{0.0};
    Real altitude_{0.0};
    Real heading_{0.0};
    Real vertical_speed_{0.0};
    Real pitch_{0.0};
    Real roll_{0.0};

    // Components
    std::vector<InstrumentState> instruments_;
    std::vector<ControlState> controls_;
    std::vector<std::string> active_checklists_;
    std::map<std::string, bool> system_states_;

    // Integration
    std::shared_ptr<xr::IXRRuntime> xr_runtime_;
    std::shared_ptr<haptics::IHapticRenderer> haptics_;
    std::shared_ptr<audio::ISpatialAudioRenderer> audio_;
};

// ============================================================================
// Mock Bridge Module Implementation
// ============================================================================

class MockBridgeModule : public IBridgeModule {
public:
    MockBridgeModule(const std::string& ship_type)
        : ship_type_(ship_type) {
        initialize_instruments();
    }

    TrainingDomain get_domain() const override { return TrainingDomain::Naval; }
    const std::string& get_name() const override { return name_; }

    TrainingResult initialize() override {
        is_initialized_ = true;
        return TrainingResult::Success;
    }

    TrainingResult shutdown() override {
        is_initialized_ = false;
        return TrainingResult::Success;
    }

    TrainingResult update(Real delta_time) override {
        if (!is_initialized_) return TrainingResult::NotInitialized;

        elapsed_time_ += delta_time;

        // Simple ship physics
        update_ship_state(delta_time);

        return TrainingResult::Success;
    }

    bool is_ready() const override { return is_initialized_; }
    std::string get_last_error() const override { return last_error_; }

    // Stations
    std::vector<BridgeStation> get_available_stations() const override {
        return {BridgeStation::Helm, BridgeStation::Engineering,
                BridgeStation::Radar, BridgeStation::Communications,
                BridgeStation::Navigation, BridgeStation::Captain};
    }

    TrainingResult select_station(BridgeStation station) override {
        current_station_ = station;
        return TrainingResult::Success;
    }

    BridgeStation get_current_station() const override {
        return current_station_;
    }

    // Navigation
    Real get_heading() const override { return heading_; }
    Real get_speed() const override { return speed_; }
    Real get_depth() const override { return depth_; }

    Vec3 get_position() const override {
        return position_;
    }

    std::vector<AircraftTrack> get_radar_contacts() const override {
        return radar_contacts_;
    }

    // Controls
    TrainingResult set_helm(Real angle) override {
        helm_angle_ = std::clamp(angle, -35.0, 35.0);
        return TrainingResult::Success;
    }

    TrainingResult set_throttle(Real power) override {
        throttle_ = std::clamp(power, -1.0, 1.0);
        return TrainingResult::Success;
    }

    TrainingResult set_thruster(ShipControlType thruster, Real power) override {
        if (thruster == ShipControlType::BowThruster) {
            bow_thruster_ = std::clamp(power, -1.0, 1.0);
        } else if (thruster == ShipControlType::SternThruster) {
            stern_thruster_ = std::clamp(power, -1.0, 1.0);
        }
        return TrainingResult::Success;
    }

    // Equipment
    std::optional<InstrumentState> get_nav_instrument(NavInstrumentType type) const override {
        for (const auto& inst : nav_instruments_) {
            if (static_cast<NavInstrumentType>(static_cast<UInt8>(inst.type) - 100) == type) {
                return inst;
            }
        }
        return std::nullopt;
    }

    TrainingResult activate_radar() override {
        radar_active_ = true;
        return TrainingResult::Success;
    }

    TrainingResult deactivate_radar() override {
        radar_active_ = false;
        radar_contacts_.clear();
        return TrainingResult::Success;
    }

    // Integration
    void bind_xr_session(std::shared_ptr<xr::IXRRuntime> xr_runtime) override {
        xr_runtime_ = xr_runtime;
    }

    void bind_haptics(std::shared_ptr<haptics::IHapticRenderer> haptics) override {
        haptics_ = haptics;
    }

private:
    void initialize_instruments() {
        // Navigation instruments (using custom type offset)
        InstrumentState compass;
        compass.type = static_cast<InstrumentType>(100 + static_cast<UInt8>(NavInstrumentType::Compass));
        compass.name = "Compass";
        compass.min_value = 0;
        compass.max_value = 360;
        compass.units = "deg";
        nav_instruments_.push_back(compass);

        InstrumentState speedlog;
        speedlog.type = static_cast<InstrumentType>(100 + static_cast<UInt8>(NavInstrumentType::SpeedLog));
        speedlog.name = "Speed Log";
        speedlog.min_value = 0;
        speedlog.max_value = 50;
        speedlog.units = "kts";
        nav_instruments_.push_back(speedlog);

        InstrumentState fathometer;
        fathometer.type = static_cast<InstrumentType>(100 + static_cast<UInt8>(NavInstrumentType::Fathometer));
        fathometer.name = "Fathometer";
        fathometer.min_value = 0;
        fathometer.max_value = 1000;
        fathometer.units = "m";
        nav_instruments_.push_back(fathometer);
    }

    void update_ship_state(Real dt) {
        // Simple ship physics

        // Speed from throttle (very simplified)
        Real target_speed = throttle_ * 30.0;  // Max 30 knots
        speed_ += (target_speed - speed_) * dt * 0.1;  // Slow response

        // Heading from helm
        Real turn_rate = helm_angle_ * speed_ / 500.0;  // Degrees per second
        heading_ += turn_rate * dt;
        if (heading_ < 0) heading_ += 360;
        if (heading_ >= 360) heading_ -= 360;

        // Position update (simplified, not geographic)
        Real heading_rad = heading_ * M_PI / 180.0;
        Real distance = speed_ * 0.514444 * dt;  // knots to m/s
        position_.x += std::sin(heading_rad) * distance;
        position_.y += std::cos(heading_rad) * distance;

        // Update nav instruments
        for (auto& inst : nav_instruments_) {
            auto nav_type = static_cast<NavInstrumentType>(static_cast<UInt8>(inst.type) - 100);
            switch (nav_type) {
                case NavInstrumentType::Compass:
                    inst.value = heading_;
                    break;
                case NavInstrumentType::SpeedLog:
                    inst.value = speed_;
                    break;
                case NavInstrumentType::Fathometer:
                    inst.value = depth_;
                    break;
                default:
                    break;
            }
        }
    }

    std::string ship_type_;
    std::string name_{"Mock Bridge Module"};
    std::string last_error_;
    bool is_initialized_{false};
    Real elapsed_time_{0.0};

    // Ship state
    Vec3 position_{0, 0, 0};
    Real heading_{0.0};
    Real speed_{0.0};
    Real depth_{50.0};  // Water depth below keel

    // Controls
    Real helm_angle_{0.0};
    Real throttle_{0.0};
    Real bow_thruster_{0.0};
    Real stern_thruster_{0.0};

    // Equipment
    std::vector<InstrumentState> nav_instruments_;
    std::vector<AircraftTrack> radar_contacts_;
    bool radar_active_{false};

    BridgeStation current_station_{BridgeStation::Helm};

    // Integration
    std::shared_ptr<xr::IXRRuntime> xr_runtime_;
    std::shared_ptr<haptics::IHapticRenderer> haptics_;
};

// ============================================================================
// Mock Control Tower Module Implementation
// ============================================================================

class MockControlTowerModule : public IControlTowerModule {
public:
    MockControlTowerModule(const std::string& airport_icao)
        : airport_icao_(airport_icao) {
        initialize_runways();
        initialize_traffic();
    }

    TrainingDomain get_domain() const override { return TrainingDomain::AirTrafficControl; }
    const std::string& get_name() const override { return name_; }

    TrainingResult initialize() override {
        is_initialized_ = true;
        return TrainingResult::Success;
    }

    TrainingResult shutdown() override {
        is_initialized_ = false;
        return TrainingResult::Success;
    }

    TrainingResult update(Real delta_time) override {
        if (!is_initialized_) return TrainingResult::NotInitialized;

        elapsed_time_ += delta_time;

        // Update traffic positions
        update_traffic(delta_time);

        return TrainingResult::Success;
    }

    bool is_ready() const override { return is_initialized_; }
    std::string get_last_error() const override { return last_error_; }

    // Position
    ATCPosition get_position() const override { return current_position_; }

    TrainingResult set_position(ATCPosition position) override {
        current_position_ = position;
        return TrainingResult::Success;
    }

    // Traffic
    std::vector<AircraftTrack> get_traffic() const override {
        return traffic_;
    }

    std::optional<AircraftTrack> get_aircraft(const std::string& callsign) const override {
        for (const auto& ac : traffic_) {
            if (ac.callsign == callsign) return ac;
        }
        return std::nullopt;
    }

    UInt32 get_traffic_count() const override {
        return static_cast<UInt32>(traffic_.size());
    }

    // Runways
    std::vector<RunwayStatus> get_runways() const override {
        return runways_;
    }

    TrainingResult set_active_runway(const std::string& designator) override {
        for (auto& rwy : runways_) {
            rwy.is_active = (rwy.designator == designator);
        }
        return TrainingResult::Success;
    }

    TrainingResult close_runway(const std::string& designator) override {
        for (auto& rwy : runways_) {
            if (rwy.designator == designator) {
                rwy.is_closed = true;
                rwy.is_active = false;
                return TrainingResult::Success;
            }
        }
        return TrainingResult::InvalidParameter;
    }

    // Communications
    TrainingResult transmit(const std::string& message) override {
        transmissions_.push_back(message);
        return TrainingResult::Success;
    }

    std::vector<std::string> get_pending_readbacks() const override {
        return pending_readbacks_;
    }

    TrainingResult verify_readback(const std::string& callsign, bool correct) override {
        auto it = std::find(pending_readbacks_.begin(), pending_readbacks_.end(), callsign);
        if (it != pending_readbacks_.end()) {
            pending_readbacks_.erase(it);
            if (!correct) {
                readback_errors_++;
            }
            return TrainingResult::Success;
        }
        return TrainingResult::InvalidParameter;
    }

    // Instructions
    TrainingResult issue_clearance(const std::string& callsign, const std::string& clearance) override {
        clearances_[callsign] = clearance;
        pending_readbacks_.push_back(callsign);
        return TrainingResult::Success;
    }

    TrainingResult issue_hold(const std::string& callsign, const std::string& fix) override {
        holds_[callsign] = fix;
        pending_readbacks_.push_back(callsign);
        return TrainingResult::Success;
    }

    TrainingResult issue_approach(const std::string& callsign, const std::string& approach) override {
        approaches_[callsign] = approach;
        pending_readbacks_.push_back(callsign);
        return TrainingResult::Success;
    }

    // Sequencing
    std::vector<std::string> get_departure_sequence() const override {
        return departure_sequence_;
    }

    std::vector<std::string> get_arrival_sequence() const override {
        return arrival_sequence_;
    }

    TrainingResult resequence(const std::string& callsign, UInt32 new_position) override {
        // Find in arrival sequence
        auto it = std::find(arrival_sequence_.begin(), arrival_sequence_.end(), callsign);
        if (it != arrival_sequence_.end()) {
            arrival_sequence_.erase(it);
            if (new_position < arrival_sequence_.size()) {
                arrival_sequence_.insert(arrival_sequence_.begin() + new_position, callsign);
            } else {
                arrival_sequence_.push_back(callsign);
            }
            return TrainingResult::Success;
        }

        // Find in departure sequence
        it = std::find(departure_sequence_.begin(), departure_sequence_.end(), callsign);
        if (it != departure_sequence_.end()) {
            departure_sequence_.erase(it);
            if (new_position < departure_sequence_.size()) {
                departure_sequence_.insert(departure_sequence_.begin() + new_position, callsign);
            } else {
                departure_sequence_.push_back(callsign);
            }
            return TrainingResult::Success;
        }

        return TrainingResult::InvalidParameter;
    }

    // Integration
    void bind_xr_session(std::shared_ptr<xr::IXRRuntime> xr_runtime) override {
        xr_runtime_ = xr_runtime;
    }

private:
    void initialize_runways() {
        runways_.push_back({"27L", true, false, false, "", 5.0, 10.0});
        runways_.push_back({"27R", false, false, false, "", 5.0, 10.0});
        runways_.push_back({"09L", false, false, false, "", -5.0, -10.0});
        runways_.push_back({"09R", false, false, false, "", -5.0, -10.0});
    }

    void initialize_traffic() {
        // Sample traffic
        AircraftTrack ac1;
        ac1.callsign = "AAL123";
        ac1.aircraft_type = "B738";
        ac1.position = Vec3{-10000, 0, -3000};
        ac1.altitude = 3000;
        ac1.heading = 270;
        ac1.ground_speed = 180;
        ac1.vertical_rate = -500;
        ac1.squawk = "4521";
        ac1.is_arriving = true;
        traffic_.push_back(ac1);

        AircraftTrack ac2;
        ac2.callsign = "UAL456";
        ac2.aircraft_type = "A320";
        ac2.position = Vec3{0, 0, 0};
        ac2.altitude = 0;
        ac2.heading = 270;
        ac2.ground_speed = 0;
        ac2.vertical_rate = 0;
        ac2.squawk = "1234";
        ac2.is_departing = true;
        traffic_.push_back(ac2);

        arrival_sequence_.push_back("AAL123");
        departure_sequence_.push_back("UAL456");
    }

    void update_traffic(Real dt) {
        for (auto& ac : traffic_) {
            // Simple movement
            Real heading_rad = ac.heading * M_PI / 180.0;
            Real speed_ms = ac.ground_speed * 0.514444;  // knots to m/s

            ac.position.x += std::sin(heading_rad) * speed_ms * dt;
            ac.position.y += std::cos(heading_rad) * speed_ms * dt;
            ac.position.z += ac.vertical_rate * 0.00508 * dt;  // fpm to m/s

            ac.altitude += ac.vertical_rate * dt / 60.0;
        }
    }

    std::string airport_icao_;
    std::string name_{"Mock Control Tower Module"};
    std::string last_error_;
    bool is_initialized_{false};
    Real elapsed_time_{0.0};

    ATCPosition current_position_{ATCPosition::Tower};

    std::vector<AircraftTrack> traffic_;
    std::vector<RunwayStatus> runways_;
    std::vector<std::string> transmissions_;
    std::vector<std::string> pending_readbacks_;
    std::vector<std::string> departure_sequence_;
    std::vector<std::string> arrival_sequence_;

    std::map<std::string, std::string> clearances_;
    std::map<std::string, std::string> holds_;
    std::map<std::string, std::string> approaches_;

    UInt32 readback_errors_{0};

    std::shared_ptr<xr::IXRRuntime> xr_runtime_;
};

// ============================================================================
// Mock After-Action Review Implementation
// ============================================================================

class MockAfterActionReview : public IAfterActionReview {
public:
    // Recording
    TrainingResult start_recording() override {
        if (is_recording_) return TrainingResult::Success;
        is_recording_ = true;
        recording_start_time_ = std::chrono::steady_clock::now();
        events_.clear();
        snapshots_.clear();

        TrainingEvent start_event;
        start_event.id = next_event_id_++;
        start_event.type = TrainingEventType::SessionStart;
        start_event.name = "Recording Started";
        start_event.timestamp = 0.0;
        events_.push_back(start_event);

        return TrainingResult::Success;
    }

    TrainingResult stop_recording() override {
        if (!is_recording_) return TrainingResult::RecordingNotStarted;

        TrainingEvent end_event;
        end_event.id = next_event_id_++;
        end_event.type = TrainingEventType::SessionEnd;
        end_event.name = "Recording Stopped";
        end_event.timestamp = get_recording_duration();
        events_.push_back(end_event);

        is_recording_ = false;
        return TrainingResult::Success;
    }

    TrainingResult add_bookmark(const std::string& name) override {
        if (!is_recording_) return TrainingResult::RecordingNotStarted;

        TrainingEvent bookmark;
        bookmark.id = next_event_id_++;
        bookmark.type = TrainingEventType::Bookmark;
        bookmark.name = name;
        bookmark.timestamp = get_recording_duration();
        events_.push_back(bookmark);

        return TrainingResult::Success;
    }

    TrainingResult add_instructor_note(const std::string& note) override {
        if (!is_recording_) return TrainingResult::RecordingNotStarted;

        TrainingEvent instructor_note;
        instructor_note.id = next_event_id_++;
        instructor_note.type = TrainingEventType::InstructorNote;
        instructor_note.name = "Instructor Note";
        instructor_note.description = note;
        instructor_note.timestamp = get_recording_duration();
        events_.push_back(instructor_note);

        return TrainingResult::Success;
    }

    bool is_recording() const override { return is_recording_; }

    // Events
    TrainingResult record_event(const TrainingEvent& event) override {
        if (!is_recording_) return TrainingResult::RecordingNotStarted;

        TrainingEvent recorded = event;
        recorded.id = next_event_id_++;
        if (recorded.timestamp == 0.0) {
            recorded.timestamp = get_recording_duration();
        }
        events_.push_back(recorded);

        return TrainingResult::Success;
    }

    std::vector<TrainingEvent> get_events() const override {
        return events_;
    }

    std::vector<TrainingEvent> get_events_in_range(Real start_time, Real end_time) const override {
        std::vector<TrainingEvent> result;
        for (const auto& event : events_) {
            if (event.timestamp >= start_time && event.timestamp <= end_time) {
                result.push_back(event);
            }
        }
        return result;
    }

    // State recording
    TrainingResult record_state(const StateSnapshot& snapshot) override {
        if (!is_recording_) return TrainingResult::RecordingNotStarted;

        StateSnapshot recorded = snapshot;
        if (recorded.timestamp == 0.0) {
            recorded.timestamp = get_recording_duration();
        }
        snapshots_.push_back(recorded);

        return TrainingResult::Success;
    }

    std::optional<StateSnapshot> get_state_at_time(Real time) const override {
        if (snapshots_.empty()) return std::nullopt;

        // Find nearest snapshot
        auto it = std::lower_bound(snapshots_.begin(), snapshots_.end(), time,
            [](const StateSnapshot& s, Real t) { return s.timestamp < t; });

        if (it == snapshots_.end()) {
            return snapshots_.back();
        }
        if (it == snapshots_.begin()) {
            return *it;
        }

        // Interpolate between two nearest snapshots (simplified: return nearest)
        auto prev = std::prev(it);
        if (time - prev->timestamp < it->timestamp - time) {
            return *prev;
        }
        return *it;
    }

    Real get_recording_duration() const override {
        if (!is_recording_ && snapshots_.empty()) return 0.0;

        if (is_recording_) {
            auto now = std::chrono::steady_clock::now();
            return std::chrono::duration<Real>(now - recording_start_time_).count();
        }

        if (!snapshots_.empty()) {
            return snapshots_.back().timestamp;
        }

        return 0.0;
    }

    // Playback
    TrainingResult start_playback() override {
        if (is_recording_) return TrainingResult::InvalidParameter;
        if (snapshots_.empty()) return TrainingResult::PlaybackError;

        playback_state_.mode = PlaybackMode::Playing;
        playback_state_.current_time = 0.0;
        playback_state_.duration = snapshots_.empty() ? 0.0 : snapshots_.back().timestamp;
        playback_start_time_ = std::chrono::steady_clock::now();

        return TrainingResult::Success;
    }

    TrainingResult pause_playback() override {
        if (playback_state_.mode != PlaybackMode::Playing) {
            return TrainingResult::PlaybackError;
        }
        playback_state_.mode = PlaybackMode::Paused;
        return TrainingResult::Success;
    }

    TrainingResult stop_playback() override {
        playback_state_.mode = PlaybackMode::Stopped;
        playback_state_.current_time = 0.0;
        return TrainingResult::Success;
    }

    TrainingResult seek(Real time) override {
        playback_state_.current_time = std::clamp(time, 0.0, playback_state_.duration);
        return TrainingResult::Success;
    }

    TrainingResult set_playback_speed(Real speed) override {
        playback_state_.playback_speed = std::clamp(speed, 0.1, 10.0);
        return TrainingResult::Success;
    }

    PlaybackState get_playback_state() const override {
        return playback_state_;
    }

    // Analysis
    GradeReport generate_grade_report() const override {
        GradeReport report;

        // Count events by type
        UInt32 errors = 0;
        UInt32 excellent = 0;
        UInt32 completed_objectives = 0;
        UInt32 failed_objectives = 0;

        for (const auto& event : events_) {
            switch (event.type) {
                case TrainingEventType::ErrorMade:
                    errors++;
                    report.significant_events.push_back(event);
                    break;
                case TrainingEventType::ExcellentPerformance:
                    excellent++;
                    report.significant_events.push_back(event);
                    break;
                case TrainingEventType::ObjectiveCompleted:
                    completed_objectives++;
                    break;
                case TrainingEventType::ObjectiveFailed:
                    failed_objectives++;
                    break;
                default:
                    break;
            }
        }

        // Calculate score
        Real base_score = 100.0;
        base_score -= errors * 5.0;
        base_score += excellent * 2.0;

        if (completed_objectives + failed_objectives > 0) {
            Real completion_rate = static_cast<Real>(completed_objectives) /
                                   (completed_objectives + failed_objectives);
            base_score = base_score * 0.5 + completion_rate * 100.0 * 0.5;
        }

        report.overall_score = std::clamp(base_score, 0.0, 100.0);

        // Assign grade
        if (report.overall_score >= 90.0) {
            report.overall_grade = Grade::Excellent;
        } else if (report.overall_score >= 80.0) {
            report.overall_grade = Grade::Good;
        } else if (report.overall_score >= 70.0) {
            report.overall_grade = Grade::Satisfactory;
        } else if (report.overall_score >= 60.0) {
            report.overall_grade = Grade::Marginal;
        } else {
            report.overall_grade = Grade::Unsatisfactory;
        }

        report.is_passed = report.overall_score >= 70.0;
        report.total_time = get_recording_duration();

        // Add recommendations
        if (errors > 3) {
            report.areas_for_improvement.push_back("Reduce procedural errors");
        }
        if (excellent > 2) {
            report.strengths.push_back("Good situational awareness");
        }

        return report;
    }

    std::vector<TrainingEvent> get_errors() const override {
        std::vector<TrainingEvent> errors;
        for (const auto& event : events_) {
            if (event.type == TrainingEventType::ErrorMade ||
                event.type == TrainingEventType::Stall ||
                event.type == TrainingEventType::Overspeed ||
                event.type == TrainingEventType::Crash) {
                errors.push_back(event);
            }
        }
        return errors;
    }

    std::vector<TrainingEvent> get_excellent_performance() const override {
        std::vector<TrainingEvent> excellent;
        for (const auto& event : events_) {
            if (event.type == TrainingEventType::ExcellentPerformance) {
                excellent.push_back(event);
            }
        }
        return excellent;
    }

    // Export
    TrainingResult export_to_file(const std::string& filepath) override {
        (void)filepath;
        // Would write events and snapshots to file
        return TrainingResult::Success;
    }

    TrainingResult import_from_file(const std::string& filepath) override {
        (void)filepath;
        // Would read events and snapshots from file
        return TrainingResult::Success;
    }

private:
    bool is_recording_{false};
    std::chrono::steady_clock::time_point recording_start_time_;
    std::chrono::steady_clock::time_point playback_start_time_;

    std::vector<TrainingEvent> events_;
    std::vector<StateSnapshot> snapshots_;
    UInt64 next_event_id_{1};

    PlaybackState playback_state_;
};

// ============================================================================
// Mock Scenario Engine Implementation
// ============================================================================

class MockScenarioEngine : public IScenarioEngine {
public:
    // Scenario management
    TrainingResult load_scenario(const ScenarioDefinition& scenario) override {
        current_scenario_ = scenario;
        is_loaded_ = true;
        reset_state();
        return TrainingResult::Success;
    }

    TrainingResult load_scenario_from_file(const std::string& filepath) override {
        (void)filepath;
        // Would parse scenario file
        return TrainingResult::ResourceNotFound;
    }

    TrainingResult unload_scenario() override {
        current_scenario_ = std::nullopt;
        is_loaded_ = false;
        is_running_ = false;
        return TrainingResult::Success;
    }

    std::optional<ScenarioDefinition> get_current_scenario() const override {
        return current_scenario_;
    }

    bool is_scenario_loaded() const override { return is_loaded_; }

    // Execution
    TrainingResult start_scenario() override {
        if (!is_loaded_) return TrainingResult::ScenarioNotLoaded;
        is_running_ = true;
        elapsed_time_ = 0.0;
        return TrainingResult::Success;
    }

    TrainingResult pause_scenario() override {
        if (!is_running_) return TrainingResult::SessionNotActive;
        is_running_ = false;
        return TrainingResult::Success;
    }

    TrainingResult resume_scenario() override {
        if (!is_loaded_) return TrainingResult::ScenarioNotLoaded;
        is_running_ = true;
        return TrainingResult::Success;
    }

    TrainingResult stop_scenario() override {
        is_running_ = false;
        return TrainingResult::Success;
    }

    TrainingResult reset_scenario() override {
        if (!is_loaded_) return TrainingResult::ScenarioNotLoaded;
        reset_state();
        return TrainingResult::Success;
    }

    bool is_running() const override { return is_running_; }

    // Objectives
    std::vector<TrainingObjective> get_objectives() const override {
        if (!current_scenario_) return {};
        return current_scenario_->objectives;
    }

    std::optional<TrainingObjective> get_current_objective() const override {
        if (!current_scenario_) return std::nullopt;

        for (const auto& obj : current_scenario_->objectives) {
            if (obj.status == ObjectiveStatus::InProgress) {
                return obj;
            }
        }

        // Return first not-started objective
        for (const auto& obj : current_scenario_->objectives) {
            if (obj.status == ObjectiveStatus::NotStarted) {
                return obj;
            }
        }

        return std::nullopt;
    }

    TrainingResult complete_objective(UInt32 objective_id) override {
        if (!current_scenario_) return TrainingResult::ScenarioNotLoaded;

        for (auto& obj : current_scenario_->objectives) {
            if (obj.id == objective_id) {
                obj.status = ObjectiveStatus::Completed;
                obj.score = 100.0;
                return TrainingResult::Success;
            }
        }
        return TrainingResult::ObjectiveNotFound;
    }

    TrainingResult fail_objective(UInt32 objective_id, const std::string& reason) override {
        if (!current_scenario_) return TrainingResult::ScenarioNotLoaded;

        for (auto& obj : current_scenario_->objectives) {
            if (obj.id == objective_id) {
                obj.status = ObjectiveStatus::Failed;
                obj.feedback.push_back(reason);
                return TrainingResult::Success;
            }
        }
        return TrainingResult::ObjectiveNotFound;
    }

    TrainingResult skip_objective(UInt32 objective_id) override {
        if (!current_scenario_) return TrainingResult::ScenarioNotLoaded;

        for (auto& obj : current_scenario_->objectives) {
            if (obj.id == objective_id) {
                obj.status = ObjectiveStatus::Skipped;
                return TrainingResult::Success;
            }
        }
        return TrainingResult::ObjectiveNotFound;
    }

    // Rules
    TrainingResult add_rule(const ScriptRule& rule) override {
        if (!current_scenario_) return TrainingResult::ScenarioNotLoaded;
        current_scenario_->rules.push_back(rule);
        return TrainingResult::Success;
    }

    TrainingResult remove_rule(UInt32 rule_id) override {
        if (!current_scenario_) return TrainingResult::ScenarioNotLoaded;

        auto& rules = current_scenario_->rules;
        rules.erase(std::remove_if(rules.begin(), rules.end(),
            [rule_id](const ScriptRule& r) { return r.id == rule_id; }),
            rules.end());

        return TrainingResult::Success;
    }

    TrainingResult enable_rule(UInt32 rule_id) override {
        if (!current_scenario_) return TrainingResult::ScenarioNotLoaded;

        for (auto& rule : current_scenario_->rules) {
            if (rule.id == rule_id) {
                rule.is_enabled = true;
                return TrainingResult::Success;
            }
        }
        return TrainingResult::InvalidParameter;
    }

    TrainingResult disable_rule(UInt32 rule_id) override {
        if (!current_scenario_) return TrainingResult::ScenarioNotLoaded;

        for (auto& rule : current_scenario_->rules) {
            if (rule.id == rule_id) {
                rule.is_enabled = false;
                return TrainingResult::Success;
            }
        }
        return TrainingResult::InvalidParameter;
    }

    std::vector<ScriptRule> get_rules() const override {
        if (!current_scenario_) return {};
        return current_scenario_->rules;
    }

    // Triggers
    TrainingResult fire_trigger(UInt32 trigger_id) override {
        if (!current_scenario_) return TrainingResult::ScenarioNotLoaded;

        for (auto& rule : current_scenario_->rules) {
            if (rule.trigger.id == trigger_id && rule.is_enabled) {
                for (const auto& action : rule.actions) {
                    execute_action(action);
                }
                rule.trigger.has_fired = true;
                return TrainingResult::Success;
            }
        }
        return TrainingResult::InvalidParameter;
    }

    TrainingResult evaluate_triggers() override {
        if (!current_scenario_ || !is_running_) return TrainingResult::SessionNotActive;

        for (auto& rule : current_scenario_->rules) {
            if (!rule.is_enabled) continue;
            if (!rule.trigger.is_repeatable && rule.trigger.has_fired) continue;

            bool should_fire = false;

            switch (rule.trigger.type) {
                case TriggerType::OnTime:
                    should_fire = (elapsed_time_ >= rule.trigger.parameter &&
                                   elapsed_time_ < rule.trigger.parameter + 0.1);
                    break;
                case TriggerType::AfterDelay:
                    should_fire = (elapsed_time_ >= rule.trigger.parameter);
                    break;
                default:
                    break;
            }

            if (should_fire) {
                for (const auto& action : rule.actions) {
                    execute_action(action);
                }
                rule.trigger.has_fired = true;
            }
        }

        return TrainingResult::Success;
    }

    // Actions
    TrainingResult execute_action(const ScriptAction& action) override {
        switch (action.type) {
            case ActionType::ShowMessage:
                pending_messages_.push_back(action.parameters);
                break;
            case ActionType::StartObjective:
                // Parse objective ID from parameters and start it
                break;
            case ActionType::EndScenario:
                is_running_ = false;
                break;
            default:
                break;
        }
        return TrainingResult::Success;
    }

    // Checkpoints
    std::vector<std::string> get_checkpoints() const override {
        if (!current_scenario_) return {};
        return current_scenario_->checkpoints;
    }

    TrainingResult jump_to_checkpoint(const std::string& checkpoint_id) override {
        auto it = checkpoint_states_.find(checkpoint_id);
        if (it != checkpoint_states_.end()) {
            // Restore state
            elapsed_time_ = it->second;
            return TrainingResult::Success;
        }
        return TrainingResult::ResourceNotFound;
    }

    TrainingResult save_checkpoint(const std::string& checkpoint_id) override {
        checkpoint_states_[checkpoint_id] = elapsed_time_;
        if (current_scenario_) {
            current_scenario_->checkpoints.push_back(checkpoint_id);
        }
        return TrainingResult::Success;
    }

    // Update
    TrainingResult update(Real delta_time) override {
        if (!is_running_) return TrainingResult::SessionNotActive;

        elapsed_time_ += delta_time;
        evaluate_triggers();

        return TrainingResult::Success;
    }

private:
    void reset_state() {
        elapsed_time_ = 0.0;
        pending_messages_.clear();
        checkpoint_states_.clear();

        if (current_scenario_) {
            for (auto& obj : current_scenario_->objectives) {
                obj.status = ObjectiveStatus::NotStarted;
                obj.score = 0.0;
                obj.elapsed_time = 0.0;
            }
            for (auto& rule : current_scenario_->rules) {
                rule.trigger.has_fired = false;
            }
        }
    }

    std::optional<ScenarioDefinition> current_scenario_;
    bool is_loaded_{false};
    bool is_running_{false};
    Real elapsed_time_{0.0};

    std::vector<std::string> pending_messages_;
    std::map<std::string, Real> checkpoint_states_;
};

// ============================================================================
// Mock Training Session Implementation
// ============================================================================

class MockTrainingSession : public ITrainingSession {
public:
    // Session lifecycle
    TrainingResult create_session(const std::string& trainee_id, TrainingDomain domain) override {
        trainee_id_ = trainee_id;
        domain_ = domain;
        session_id_ = "session_" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count());

        // Create appropriate modules
        switch (domain) {
            case TrainingDomain::Aviation:
            case TrainingDomain::Rotorcraft:
                cockpit_module_ = std::make_shared<MockCockpitModule>("generic");
                break;
            case TrainingDomain::Naval:
                bridge_module_ = std::make_shared<MockBridgeModule>("generic");
                break;
            case TrainingDomain::AirTrafficControl:
                tower_module_ = std::make_shared<MockControlTowerModule>("KJFK");
                break;
            default:
                break;
        }

        aar_ = std::make_shared<MockAfterActionReview>();
        scenario_engine_ = std::make_shared<MockScenarioEngine>();

        return TrainingResult::Success;
    }

    TrainingResult start_session() override {
        if (trainee_id_.empty()) return TrainingResult::NotInitialized;

        is_active_ = true;
        start_time_ = std::chrono::steady_clock::now();

        // Initialize modules
        if (cockpit_module_) cockpit_module_->initialize();
        if (bridge_module_) bridge_module_->initialize();
        if (tower_module_) tower_module_->initialize();

        // Start recording
        if (aar_) aar_->start_recording();

        return TrainingResult::Success;
    }

    TrainingResult pause_session() override {
        if (!is_active_) return TrainingResult::SessionNotActive;
        is_paused_ = true;
        if (scenario_engine_) scenario_engine_->pause_scenario();
        return TrainingResult::Success;
    }

    TrainingResult resume_session() override {
        if (!is_active_) return TrainingResult::SessionNotActive;
        is_paused_ = false;
        if (scenario_engine_) scenario_engine_->resume_scenario();
        return TrainingResult::Success;
    }

    TrainingResult end_session() override {
        if (!is_active_) return TrainingResult::SessionNotActive;

        is_active_ = false;

        // Stop recording
        if (aar_) aar_->stop_recording();

        // Stop scenario
        if (scenario_engine_) scenario_engine_->stop_scenario();

        // Shutdown modules
        if (cockpit_module_) cockpit_module_->shutdown();
        if (bridge_module_) bridge_module_->shutdown();
        if (tower_module_) tower_module_->shutdown();

        return TrainingResult::Success;
    }

    bool is_active() const override { return is_active_; }

    Real get_session_time() const override {
        if (!is_active_) return 0.0;
        auto now = std::chrono::steady_clock::now();
        return std::chrono::duration<Real>(now - start_time_).count();
    }

    // Modules
    std::shared_ptr<ICockpitModule> get_cockpit_module() override {
        return cockpit_module_;
    }

    std::shared_ptr<IBridgeModule> get_bridge_module() override {
        return bridge_module_;
    }

    std::shared_ptr<IControlTowerModule> get_tower_module() override {
        return tower_module_;
    }

    std::shared_ptr<IAfterActionReview> get_aar() override {
        return aar_;
    }

    std::shared_ptr<IScenarioEngine> get_scenario_engine() override {
        return scenario_engine_;
    }

    // Configuration
    TrainingResult set_difficulty(DifficultyLevel level) override {
        difficulty_ = level;
        return TrainingResult::Success;
    }

    DifficultyLevel get_difficulty() const override {
        return difficulty_;
    }

    TrainingResult set_weather(WeatherPreset weather) override {
        weather_ = weather;
        return TrainingResult::Success;
    }

    TrainingResult set_time_of_day(TimeOfDay time) override {
        time_of_day_ = time;
        return TrainingResult::Success;
    }

    // Trainee info
    std::string get_trainee_id() const override { return trainee_id_; }
    std::string get_session_id() const override { return session_id_; }

    // Statistics
    UInt32 get_total_objectives() const override {
        if (!scenario_engine_) return 0;
        auto scenario = scenario_engine_->get_current_scenario();
        return scenario ? static_cast<UInt32>(scenario->objectives.size()) : 0;
    }

    UInt32 get_completed_objectives() const override {
        if (!scenario_engine_) return 0;
        UInt32 count = 0;
        for (const auto& obj : scenario_engine_->get_objectives()) {
            if (obj.status == ObjectiveStatus::Completed) count++;
        }
        return count;
    }

    Real get_current_score() const override {
        if (!scenario_engine_) return 0.0;

        Real total_score = 0.0;
        Real total_weight = 0.0;

        for (const auto& obj : scenario_engine_->get_objectives()) {
            if (obj.is_graded) {
                total_score += obj.score * obj.weight;
                total_weight += obj.weight;
            }
        }

        return total_weight > 0 ? total_score / total_weight : 0.0;
    }

    // Update
    TrainingResult update(Real delta_time) override {
        if (!is_active_ || is_paused_) return TrainingResult::Success;

        // Update modules
        if (cockpit_module_) cockpit_module_->update(delta_time);
        if (bridge_module_) bridge_module_->update(delta_time);
        if (tower_module_) tower_module_->update(delta_time);

        // Update scenario
        if (scenario_engine_) scenario_engine_->update(delta_time);

        // Record state for AAR
        if (aar_ && aar_->is_recording()) {
            StateSnapshot snapshot;
            snapshot.timestamp = get_session_time();

            if (cockpit_module_) {
                snapshot.airspeed = cockpit_module_->get_airspeed();
                snapshot.altitude = cockpit_module_->get_altitude();
                snapshot.heading = cockpit_module_->get_heading();
                auto attitude = cockpit_module_->get_attitude();
                snapshot.pitch = attitude.x;
                snapshot.roll = attitude.y;
                snapshot.vertical_speed = cockpit_module_->get_vertical_speed();
            }

            aar_->record_state(snapshot);
        }

        return TrainingResult::Success;
    }

private:
    std::string trainee_id_;
    std::string session_id_;
    TrainingDomain domain_{TrainingDomain::Aviation};
    DifficultyLevel difficulty_{DifficultyLevel::Intermediate};
    WeatherPreset weather_{WeatherPreset::Clear};
    TimeOfDay time_of_day_{TimeOfDay::Morning};

    bool is_active_{false};
    bool is_paused_{false};
    std::chrono::steady_clock::time_point start_time_;

    std::shared_ptr<MockCockpitModule> cockpit_module_;
    std::shared_ptr<MockBridgeModule> bridge_module_;
    std::shared_ptr<MockControlTowerModule> tower_module_;
    std::shared_ptr<MockAfterActionReview> aar_;
    std::shared_ptr<MockScenarioEngine> scenario_engine_;
};

// ============================================================================
// Factory Functions
// ============================================================================

std::unique_ptr<ICockpitModule> create_cockpit_module(const std::string& aircraft_type) {
    return std::make_unique<MockCockpitModule>(aircraft_type);
}

std::unique_ptr<ICockpitModule> create_generic_cockpit_module() {
    return std::make_unique<MockCockpitModule>("generic");
}

std::unique_ptr<IBridgeModule> create_bridge_module(const std::string& ship_type) {
    return std::make_unique<MockBridgeModule>(ship_type);
}

std::unique_ptr<IBridgeModule> create_generic_bridge_module() {
    return std::make_unique<MockBridgeModule>("generic");
}

std::unique_ptr<IControlTowerModule> create_tower_module(const std::string& airport_icao) {
    return std::make_unique<MockControlTowerModule>(airport_icao);
}

std::unique_ptr<IAfterActionReview> create_aar() {
    return std::make_unique<MockAfterActionReview>();
}

std::unique_ptr<IScenarioEngine> create_scenario_engine() {
    return std::make_unique<MockScenarioEngine>();
}

std::unique_ptr<ITrainingSession> create_training_session() {
    return std::make_unique<MockTrainingSession>();
}

std::unique_ptr<ITrainingSession> create_mock_training_session() {
    return std::make_unique<MockTrainingSession>();
}

} // namespace jaguar::training
