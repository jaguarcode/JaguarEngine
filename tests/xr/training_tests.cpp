/**
 * @file training_tests.cpp
 * @brief Comprehensive tests for training module system
 *
 * Tests cockpit simulators, ship bridge simulators, control tower modules,
 * after-action review, and training scenario scripting.
 */

#include <gtest/gtest.h>
#include "jaguar/xr/training.h"
#include <thread>
#include <chrono>

using namespace jaguar;
using namespace jaguar::training;

// ============================================================================
// Training Result Tests
// ============================================================================

TEST(TrainingResultTest, ResultToString) {
    EXPECT_STREQ("Success", training_result_to_string(TrainingResult::Success));
    EXPECT_STREQ("Not Initialized", training_result_to_string(TrainingResult::NotInitialized));
    EXPECT_STREQ("Session Not Active", training_result_to_string(TrainingResult::SessionNotActive));
    EXPECT_STREQ("Scenario Not Loaded", training_result_to_string(TrainingResult::ScenarioNotLoaded));
    EXPECT_STREQ("Invalid Parameter", training_result_to_string(TrainingResult::InvalidParameter));
    EXPECT_STREQ("Module Not Available", training_result_to_string(TrainingResult::ModuleNotAvailable));
    EXPECT_STREQ("Instrument Not Found", training_result_to_string(TrainingResult::InstrumentNotFound));
    EXPECT_STREQ("Objective Not Found", training_result_to_string(TrainingResult::ObjectiveNotFound));
    EXPECT_STREQ("Recording Not Started", training_result_to_string(TrainingResult::RecordingNotStarted));
    EXPECT_STREQ("Playback Error", training_result_to_string(TrainingResult::PlaybackError));
    EXPECT_STREQ("Script Error", training_result_to_string(TrainingResult::ScriptError));
    EXPECT_STREQ("Internal Error", training_result_to_string(TrainingResult::InternalError));
}

TEST(TrainingResultTest, SuccessCheck) {
    EXPECT_TRUE(training_succeeded(TrainingResult::Success));
    EXPECT_FALSE(training_succeeded(TrainingResult::NotInitialized));
    EXPECT_FALSE(training_succeeded(TrainingResult::InternalError));
}

// ============================================================================
// Cockpit Module Tests
// ============================================================================

class CockpitModuleTest : public ::testing::Test {
protected:
    void SetUp() override {
        cockpit_ = create_generic_cockpit_module();
        ASSERT_NE(cockpit_, nullptr);
    }

    std::unique_ptr<ICockpitModule> cockpit_;
};

TEST_F(CockpitModuleTest, Creation) {
    EXPECT_NE(cockpit_, nullptr);
    EXPECT_EQ(cockpit_->get_domain(), TrainingDomain::Aviation);
    EXPECT_FALSE(cockpit_->get_name().empty());
}

TEST_F(CockpitModuleTest, Initialization) {
    EXPECT_FALSE(cockpit_->is_ready());
    EXPECT_EQ(cockpit_->initialize(), TrainingResult::Success);
    EXPECT_TRUE(cockpit_->is_ready());
    EXPECT_EQ(cockpit_->shutdown(), TrainingResult::Success);
    EXPECT_FALSE(cockpit_->is_ready());
}

TEST_F(CockpitModuleTest, Instruments) {
    cockpit_->initialize();

    auto instruments = cockpit_->get_instruments();
    EXPECT_GT(instruments.size(), 0);

    // Check primary flight instruments exist
    auto airspeed = cockpit_->get_instrument(InstrumentType::Airspeed);
    EXPECT_TRUE(airspeed.has_value());
    EXPECT_EQ(airspeed->type, InstrumentType::Airspeed);

    auto altitude = cockpit_->get_instrument(InstrumentType::Altitude);
    EXPECT_TRUE(altitude.has_value());

    auto attitude = cockpit_->get_instrument(InstrumentType::AttitudeIndicator);
    EXPECT_TRUE(attitude.has_value());

    auto heading = cockpit_->get_instrument(InstrumentType::HeadingIndicator);
    EXPECT_TRUE(heading.has_value());

    auto vsi = cockpit_->get_instrument(InstrumentType::VerticalSpeed);
    EXPECT_TRUE(vsi.has_value());
}

TEST_F(CockpitModuleTest, SetInstrumentValue) {
    cockpit_->initialize();

    EXPECT_EQ(cockpit_->set_instrument_value(InstrumentType::Airspeed, 150.0), TrainingResult::Success);
    auto airspeed = cockpit_->get_instrument(InstrumentType::Airspeed);
    EXPECT_TRUE(airspeed.has_value());
    EXPECT_NEAR(airspeed->value, 150.0, 0.1);

    // Test clamping to max value
    EXPECT_EQ(cockpit_->set_instrument_value(InstrumentType::Airspeed, 500.0), TrainingResult::Success);
    airspeed = cockpit_->get_instrument(InstrumentType::Airspeed);
    EXPECT_LE(airspeed->value, airspeed->max_value);
}

TEST_F(CockpitModuleTest, InstrumentFailure) {
    cockpit_->initialize();

    EXPECT_EQ(cockpit_->inject_instrument_failure(InstrumentType::Airspeed), TrainingResult::Success);
    auto airspeed = cockpit_->get_instrument(InstrumentType::Airspeed);
    EXPECT_TRUE(airspeed.has_value());
    EXPECT_TRUE(airspeed->has_failure);
    EXPECT_FALSE(airspeed->is_active);

    EXPECT_EQ(cockpit_->restore_instrument(InstrumentType::Airspeed), TrainingResult::Success);
    airspeed = cockpit_->get_instrument(InstrumentType::Airspeed);
    EXPECT_FALSE(airspeed->has_failure);
    EXPECT_TRUE(airspeed->is_active);
}

TEST_F(CockpitModuleTest, Controls) {
    cockpit_->initialize();

    auto controls = cockpit_->get_controls();
    EXPECT_GT(controls.size(), 0);

    auto yoke = cockpit_->get_control(ControlType::Yoke);
    EXPECT_TRUE(yoke.has_value());

    auto throttle = cockpit_->get_control(ControlType::Throttle);
    EXPECT_TRUE(throttle.has_value());

    auto rudder = cockpit_->get_control(ControlType::Rudder);
    EXPECT_TRUE(rudder.has_value());
}

TEST_F(CockpitModuleTest, SetControlPosition) {
    cockpit_->initialize();

    EXPECT_EQ(cockpit_->set_control_position(ControlType::Throttle, 0.75), TrainingResult::Success);
    auto throttle = cockpit_->get_control(ControlType::Throttle);
    EXPECT_TRUE(throttle.has_value());
    EXPECT_NEAR(throttle->position, 0.75, 0.01);

    // Test clamping
    EXPECT_EQ(cockpit_->set_control_position(ControlType::Yoke, 2.0), TrainingResult::Success);
    auto yoke = cockpit_->get_control(ControlType::Yoke);
    EXPECT_LE(yoke->position, 1.0);
}

TEST_F(CockpitModuleTest, FlightState) {
    cockpit_->initialize();

    Real airspeed = cockpit_->get_airspeed();
    Real altitude = cockpit_->get_altitude();
    Real heading = cockpit_->get_heading();
    Real vs = cockpit_->get_vertical_speed();
    Vec3 attitude = cockpit_->get_attitude();

    // Initial values should be reasonable
    EXPECT_GE(airspeed, 0.0);
    EXPECT_GE(altitude, 0.0);
    EXPECT_GE(heading, 0.0);
    EXPECT_LE(heading, 360.0);

    // Attitude should have pitch, roll, yaw
    (void)attitude;  // Vec3 with pitch, roll, yaw
}

TEST_F(CockpitModuleTest, Checklists) {
    cockpit_->initialize();

    EXPECT_EQ(cockpit_->start_checklist("before_takeoff"), TrainingResult::Success);
    auto checklists = cockpit_->get_active_checklists();
    EXPECT_EQ(checklists.size(), 1);
    EXPECT_EQ(checklists[0], "before_takeoff");

    EXPECT_EQ(cockpit_->complete_checklist_item(0), TrainingResult::Success);
}

TEST_F(CockpitModuleTest, Systems) {
    cockpit_->initialize();

    EXPECT_EQ(cockpit_->set_system_state("electrical", true), TrainingResult::Success);
    EXPECT_TRUE(cockpit_->get_system_state("electrical"));

    EXPECT_EQ(cockpit_->set_system_state("electrical", false), TrainingResult::Success);
    EXPECT_FALSE(cockpit_->get_system_state("electrical"));

    EXPECT_FALSE(cockpit_->get_system_state("nonexistent_system"));
}

TEST_F(CockpitModuleTest, Update) {
    cockpit_->initialize();

    // Set throttle and run simulation
    cockpit_->set_control_position(ControlType::Throttle, 1.0);

    Real initial_speed = cockpit_->get_airspeed();
    cockpit_->update(1.0);
    Real new_speed = cockpit_->get_airspeed();

    // Speed should increase with throttle
    EXPECT_GE(new_speed, initial_speed);
}

TEST_F(CockpitModuleTest, UpdateWithoutInit) {
    EXPECT_EQ(cockpit_->update(0.1), TrainingResult::NotInitialized);
}

// ============================================================================
// Bridge Module Tests
// ============================================================================

class BridgeModuleTest : public ::testing::Test {
protected:
    void SetUp() override {
        bridge_ = create_generic_bridge_module();
        ASSERT_NE(bridge_, nullptr);
    }

    std::unique_ptr<IBridgeModule> bridge_;
};

TEST_F(BridgeModuleTest, Creation) {
    EXPECT_NE(bridge_, nullptr);
    EXPECT_EQ(bridge_->get_domain(), TrainingDomain::Naval);
    EXPECT_FALSE(bridge_->get_name().empty());
}

TEST_F(BridgeModuleTest, Initialization) {
    EXPECT_FALSE(bridge_->is_ready());
    EXPECT_EQ(bridge_->initialize(), TrainingResult::Success);
    EXPECT_TRUE(bridge_->is_ready());
    EXPECT_EQ(bridge_->shutdown(), TrainingResult::Success);
    EXPECT_FALSE(bridge_->is_ready());
}

TEST_F(BridgeModuleTest, Stations) {
    bridge_->initialize();

    auto stations = bridge_->get_available_stations();
    EXPECT_GT(stations.size(), 0);

    EXPECT_EQ(bridge_->select_station(BridgeStation::Helm), TrainingResult::Success);
    EXPECT_EQ(bridge_->get_current_station(), BridgeStation::Helm);

    EXPECT_EQ(bridge_->select_station(BridgeStation::Engineering), TrainingResult::Success);
    EXPECT_EQ(bridge_->get_current_station(), BridgeStation::Engineering);
}

TEST_F(BridgeModuleTest, Navigation) {
    bridge_->initialize();

    Real heading = bridge_->get_heading();
    Real speed = bridge_->get_speed();
    Real depth = bridge_->get_depth();
    Vec3 position = bridge_->get_position();

    EXPECT_GE(heading, 0.0);
    EXPECT_LE(heading, 360.0);
    EXPECT_GE(speed, 0.0);
    EXPECT_GE(depth, 0.0);
    (void)position;
}

TEST_F(BridgeModuleTest, Controls) {
    bridge_->initialize();

    EXPECT_EQ(bridge_->set_helm(15.0), TrainingResult::Success);
    EXPECT_EQ(bridge_->set_throttle(0.5), TrainingResult::Success);
    EXPECT_EQ(bridge_->set_thruster(ShipControlType::BowThruster, 0.3), TrainingResult::Success);

    // Test clamping
    EXPECT_EQ(bridge_->set_helm(50.0), TrainingResult::Success);  // Should clamp to 35
}

TEST_F(BridgeModuleTest, Radar) {
    bridge_->initialize();

    EXPECT_EQ(bridge_->activate_radar(), TrainingResult::Success);
    auto contacts = bridge_->get_radar_contacts();
    // May or may not have contacts initially

    EXPECT_EQ(bridge_->deactivate_radar(), TrainingResult::Success);
}

TEST_F(BridgeModuleTest, Update) {
    bridge_->initialize();

    bridge_->set_throttle(1.0);
    Real initial_speed = bridge_->get_speed();

    for (int i = 0; i < 100; ++i) {
        bridge_->update(0.1);
    }

    Real new_speed = bridge_->get_speed();
    EXPECT_GT(new_speed, initial_speed);
}

// ============================================================================
// Control Tower Module Tests
// ============================================================================

class ControlTowerModuleTest : public ::testing::Test {
protected:
    void SetUp() override {
        tower_ = create_tower_module("KJFK");
        ASSERT_NE(tower_, nullptr);
    }

    std::unique_ptr<IControlTowerModule> tower_;
};

TEST_F(ControlTowerModuleTest, Creation) {
    EXPECT_NE(tower_, nullptr);
    EXPECT_EQ(tower_->get_domain(), TrainingDomain::AirTrafficControl);
    EXPECT_FALSE(tower_->get_name().empty());
}

TEST_F(ControlTowerModuleTest, Initialization) {
    EXPECT_FALSE(tower_->is_ready());
    EXPECT_EQ(tower_->initialize(), TrainingResult::Success);
    EXPECT_TRUE(tower_->is_ready());
}

TEST_F(ControlTowerModuleTest, Position) {
    tower_->initialize();

    ATCPosition pos = tower_->get_position();
    EXPECT_EQ(pos, ATCPosition::Tower);

    EXPECT_EQ(tower_->set_position(ATCPosition::Approach), TrainingResult::Success);
    EXPECT_EQ(tower_->get_position(), ATCPosition::Approach);
}

TEST_F(ControlTowerModuleTest, Traffic) {
    tower_->initialize();

    auto traffic = tower_->get_traffic();
    EXPECT_GT(traffic.size(), 0);

    UInt32 count = tower_->get_traffic_count();
    EXPECT_EQ(count, traffic.size());

    // Get specific aircraft
    auto aircraft = tower_->get_aircraft("AAL123");
    EXPECT_TRUE(aircraft.has_value());
    EXPECT_EQ(aircraft->callsign, "AAL123");

    auto nonexistent = tower_->get_aircraft("NONEXISTENT");
    EXPECT_FALSE(nonexistent.has_value());
}

TEST_F(ControlTowerModuleTest, Runways) {
    tower_->initialize();

    auto runways = tower_->get_runways();
    EXPECT_GT(runways.size(), 0);

    EXPECT_EQ(tower_->set_active_runway("27R"), TrainingResult::Success);
    runways = tower_->get_runways();

    bool found_active = false;
    for (const auto& rwy : runways) {
        if (rwy.designator == "27R") {
            EXPECT_TRUE(rwy.is_active);
            found_active = true;
        }
    }
    EXPECT_TRUE(found_active);

    EXPECT_EQ(tower_->close_runway("27R"), TrainingResult::Success);
    runways = tower_->get_runways();
    for (const auto& rwy : runways) {
        if (rwy.designator == "27R") {
            EXPECT_TRUE(rwy.is_closed);
        }
    }
}

TEST_F(ControlTowerModuleTest, Communications) {
    tower_->initialize();

    EXPECT_EQ(tower_->transmit("AAL123, cleared for approach"), TrainingResult::Success);

    EXPECT_EQ(tower_->issue_clearance("AAL123", "ILS 27L"), TrainingResult::Success);
    auto pending = tower_->get_pending_readbacks();
    EXPECT_GT(pending.size(), 0);

    EXPECT_EQ(tower_->verify_readback("AAL123", true), TrainingResult::Success);
}

TEST_F(ControlTowerModuleTest, Instructions) {
    tower_->initialize();

    EXPECT_EQ(tower_->issue_clearance("UAL456", "Cleared for takeoff 27L"), TrainingResult::Success);
    EXPECT_EQ(tower_->issue_hold("AAL123", "CAMRN"), TrainingResult::Success);
    EXPECT_EQ(tower_->issue_approach("AAL123", "ILS 27L"), TrainingResult::Success);
}

TEST_F(ControlTowerModuleTest, Sequencing) {
    tower_->initialize();

    auto departures = tower_->get_departure_sequence();
    auto arrivals = tower_->get_arrival_sequence();

    EXPECT_GT(departures.size() + arrivals.size(), 0);

    if (!arrivals.empty()) {
        std::string callsign = arrivals[0];
        EXPECT_EQ(tower_->resequence(callsign, arrivals.size()), TrainingResult::Success);
    }
}

TEST_F(ControlTowerModuleTest, Update) {
    tower_->initialize();

    auto traffic_before = tower_->get_traffic();
    Vec3 pos_before = traffic_before[0].position;

    tower_->update(10.0);

    auto traffic_after = tower_->get_traffic();
    Vec3 pos_after = traffic_after[0].position;

    // Position should change if aircraft has speed
    if (traffic_before[0].ground_speed > 0) {
        EXPECT_NE(pos_before.x, pos_after.x);
    }
}

// ============================================================================
// After-Action Review Tests
// ============================================================================

class AfterActionReviewTest : public ::testing::Test {
protected:
    void SetUp() override {
        aar_ = create_aar();
        ASSERT_NE(aar_, nullptr);
    }

    std::unique_ptr<IAfterActionReview> aar_;
};

TEST_F(AfterActionReviewTest, Creation) {
    EXPECT_NE(aar_, nullptr);
}

TEST_F(AfterActionReviewTest, Recording) {
    EXPECT_FALSE(aar_->is_recording());

    EXPECT_EQ(aar_->start_recording(), TrainingResult::Success);
    EXPECT_TRUE(aar_->is_recording());

    EXPECT_EQ(aar_->stop_recording(), TrainingResult::Success);
    EXPECT_FALSE(aar_->is_recording());
}

TEST_F(AfterActionReviewTest, RecordingDuration) {
    aar_->start_recording();

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    Real duration = aar_->get_recording_duration();
    EXPECT_GT(duration, 0.0);

    aar_->stop_recording();
}

TEST_F(AfterActionReviewTest, Bookmarks) {
    aar_->start_recording();

    EXPECT_EQ(aar_->add_bookmark("Takeoff"), TrainingResult::Success);
    EXPECT_EQ(aar_->add_bookmark("Level Off"), TrainingResult::Success);

    auto events = aar_->get_events();
    int bookmark_count = 0;
    for (const auto& e : events) {
        if (e.type == TrainingEventType::Bookmark) bookmark_count++;
    }
    EXPECT_EQ(bookmark_count, 2);

    aar_->stop_recording();
}

TEST_F(AfterActionReviewTest, InstructorNotes) {
    aar_->start_recording();

    EXPECT_EQ(aar_->add_instructor_note("Good altitude control"), TrainingResult::Success);

    auto events = aar_->get_events();
    bool found_note = false;
    for (const auto& e : events) {
        if (e.type == TrainingEventType::InstructorNote) {
            found_note = true;
            EXPECT_EQ(e.description, "Good altitude control");
        }
    }
    EXPECT_TRUE(found_note);

    aar_->stop_recording();
}

TEST_F(AfterActionReviewTest, Events) {
    aar_->start_recording();

    TrainingEvent event;
    event.type = TrainingEventType::ErrorMade;
    event.name = "Altitude Deviation";
    event.severity = 0.5;
    EXPECT_EQ(aar_->record_event(event), TrainingResult::Success);

    auto events = aar_->get_events();
    bool found_error = false;
    for (const auto& e : events) {
        if (e.type == TrainingEventType::ErrorMade) found_error = true;
    }
    EXPECT_TRUE(found_error);

    aar_->stop_recording();
}

TEST_F(AfterActionReviewTest, EventsInRange) {
    aar_->start_recording();

    TrainingEvent early;
    early.type = TrainingEventType::Checkpoint;
    early.timestamp = 1.0;
    aar_->record_event(early);

    TrainingEvent late;
    late.type = TrainingEventType::Checkpoint;
    late.timestamp = 5.0;
    aar_->record_event(late);

    auto events = aar_->get_events_in_range(0.5, 2.0);
    int checkpoint_count = 0;
    for (const auto& e : events) {
        if (e.type == TrainingEventType::Checkpoint) checkpoint_count++;
    }
    EXPECT_GE(checkpoint_count, 1);

    aar_->stop_recording();
}

TEST_F(AfterActionReviewTest, StateRecording) {
    aar_->start_recording();

    StateSnapshot snapshot;
    snapshot.timestamp = 0.0;
    snapshot.airspeed = 150.0;
    snapshot.altitude = 5000.0;
    snapshot.heading = 270.0;
    EXPECT_EQ(aar_->record_state(snapshot), TrainingResult::Success);

    StateSnapshot snapshot2;
    snapshot2.timestamp = 1.0;
    snapshot2.airspeed = 155.0;
    snapshot2.altitude = 5100.0;
    EXPECT_EQ(aar_->record_state(snapshot2), TrainingResult::Success);

    auto retrieved = aar_->get_state_at_time(0.5);
    EXPECT_TRUE(retrieved.has_value());

    aar_->stop_recording();
}

TEST_F(AfterActionReviewTest, Playback) {
    aar_->start_recording();

    StateSnapshot snapshot;
    snapshot.timestamp = 0.0;
    snapshot.airspeed = 150.0;
    aar_->record_state(snapshot);

    aar_->stop_recording();

    EXPECT_EQ(aar_->start_playback(), TrainingResult::Success);
    auto state = aar_->get_playback_state();
    EXPECT_EQ(state.mode, PlaybackMode::Playing);

    EXPECT_EQ(aar_->pause_playback(), TrainingResult::Success);
    state = aar_->get_playback_state();
    EXPECT_EQ(state.mode, PlaybackMode::Paused);

    EXPECT_EQ(aar_->seek(0.5), TrainingResult::Success);

    EXPECT_EQ(aar_->set_playback_speed(2.0), TrainingResult::Success);
    state = aar_->get_playback_state();
    EXPECT_NEAR(state.playback_speed, 2.0, 0.01);

    EXPECT_EQ(aar_->stop_playback(), TrainingResult::Success);
    state = aar_->get_playback_state();
    EXPECT_EQ(state.mode, PlaybackMode::Stopped);
}

TEST_F(AfterActionReviewTest, GradeReport) {
    aar_->start_recording();

    // Record some events for grading
    TrainingEvent good;
    good.type = TrainingEventType::ExcellentPerformance;
    aar_->record_event(good);

    TrainingEvent complete;
    complete.type = TrainingEventType::ObjectiveCompleted;
    aar_->record_event(complete);

    aar_->stop_recording();

    GradeReport report = aar_->generate_grade_report();
    EXPECT_GT(report.overall_score, 0.0);
    EXPECT_LE(report.overall_score, 100.0);
}

TEST_F(AfterActionReviewTest, GetErrors) {
    aar_->start_recording();

    TrainingEvent error;
    error.type = TrainingEventType::ErrorMade;
    error.name = "Error 1";
    aar_->record_event(error);

    TrainingEvent stall;
    stall.type = TrainingEventType::Stall;
    stall.name = "Stall warning";
    aar_->record_event(stall);

    aar_->stop_recording();

    auto errors = aar_->get_errors();
    EXPECT_GE(errors.size(), 2);
}

TEST_F(AfterActionReviewTest, ExportImport) {
    aar_->start_recording();
    aar_->add_bookmark("Test");
    aar_->stop_recording();

    EXPECT_EQ(aar_->export_to_file("/tmp/test_aar.json"), TrainingResult::Success);
    EXPECT_EQ(aar_->import_from_file("/tmp/test_aar.json"), TrainingResult::Success);
}

TEST_F(AfterActionReviewTest, RecordingNotStartedErrors) {
    EXPECT_EQ(aar_->add_bookmark("Test"), TrainingResult::RecordingNotStarted);
    EXPECT_EQ(aar_->add_instructor_note("Note"), TrainingResult::RecordingNotStarted);

    TrainingEvent event;
    EXPECT_EQ(aar_->record_event(event), TrainingResult::RecordingNotStarted);

    StateSnapshot snapshot;
    EXPECT_EQ(aar_->record_state(snapshot), TrainingResult::RecordingNotStarted);
}

// ============================================================================
// Scenario Engine Tests
// ============================================================================

class ScenarioEngineTest : public ::testing::Test {
protected:
    void SetUp() override {
        engine_ = create_scenario_engine();
        ASSERT_NE(engine_, nullptr);

        // Create a test scenario
        scenario_.id = "test_scenario";
        scenario_.name = "Test Scenario";
        scenario_.description = "A test scenario for unit tests";
        scenario_.domain = TrainingDomain::Aviation;
        scenario_.difficulty = DifficultyLevel::Intermediate;
        scenario_.weather = WeatherPreset::Clear;
        scenario_.time = TimeOfDay::Morning;

        TrainingObjective obj1;
        obj1.id = 1;
        obj1.type = ObjectiveType::Takeoff;
        obj1.name = "Complete Takeoff";
        obj1.is_mandatory = true;
        scenario_.objectives.push_back(obj1);

        TrainingObjective obj2;
        obj2.id = 2;
        obj2.type = ObjectiveType::Navigation;
        obj2.name = "Navigate to Waypoint";
        obj2.is_mandatory = false;
        scenario_.objectives.push_back(obj2);
    }

    std::unique_ptr<IScenarioEngine> engine_;
    ScenarioDefinition scenario_;
};

TEST_F(ScenarioEngineTest, Creation) {
    EXPECT_NE(engine_, nullptr);
    EXPECT_FALSE(engine_->is_scenario_loaded());
    EXPECT_FALSE(engine_->is_running());
}

TEST_F(ScenarioEngineTest, LoadScenario) {
    EXPECT_EQ(engine_->load_scenario(scenario_), TrainingResult::Success);
    EXPECT_TRUE(engine_->is_scenario_loaded());

    auto loaded = engine_->get_current_scenario();
    EXPECT_TRUE(loaded.has_value());
    EXPECT_EQ(loaded->name, "Test Scenario");
}

TEST_F(ScenarioEngineTest, UnloadScenario) {
    engine_->load_scenario(scenario_);
    EXPECT_EQ(engine_->unload_scenario(), TrainingResult::Success);
    EXPECT_FALSE(engine_->is_scenario_loaded());
    EXPECT_FALSE(engine_->get_current_scenario().has_value());
}

TEST_F(ScenarioEngineTest, Execution) {
    engine_->load_scenario(scenario_);

    EXPECT_EQ(engine_->start_scenario(), TrainingResult::Success);
    EXPECT_TRUE(engine_->is_running());

    EXPECT_EQ(engine_->pause_scenario(), TrainingResult::Success);
    EXPECT_FALSE(engine_->is_running());

    EXPECT_EQ(engine_->resume_scenario(), TrainingResult::Success);
    EXPECT_TRUE(engine_->is_running());

    EXPECT_EQ(engine_->stop_scenario(), TrainingResult::Success);
    EXPECT_FALSE(engine_->is_running());
}

TEST_F(ScenarioEngineTest, ExecutionWithoutLoad) {
    EXPECT_EQ(engine_->start_scenario(), TrainingResult::ScenarioNotLoaded);
}

TEST_F(ScenarioEngineTest, Objectives) {
    engine_->load_scenario(scenario_);

    auto objectives = engine_->get_objectives();
    EXPECT_EQ(objectives.size(), 2);

    auto current = engine_->get_current_objective();
    EXPECT_TRUE(current.has_value());
    EXPECT_EQ(current->status, ObjectiveStatus::NotStarted);
}

TEST_F(ScenarioEngineTest, CompleteObjective) {
    engine_->load_scenario(scenario_);

    EXPECT_EQ(engine_->complete_objective(1), TrainingResult::Success);
    auto objectives = engine_->get_objectives();
    bool found = false;
    for (const auto& obj : objectives) {
        if (obj.id == 1) {
            EXPECT_EQ(obj.status, ObjectiveStatus::Completed);
            EXPECT_NEAR(obj.score, 100.0, 0.1);
            found = true;
        }
    }
    EXPECT_TRUE(found);
}

TEST_F(ScenarioEngineTest, FailObjective) {
    engine_->load_scenario(scenario_);

    EXPECT_EQ(engine_->fail_objective(1, "Crashed on takeoff"), TrainingResult::Success);
    auto objectives = engine_->get_objectives();
    for (const auto& obj : objectives) {
        if (obj.id == 1) {
            EXPECT_EQ(obj.status, ObjectiveStatus::Failed);
            EXPECT_GT(obj.feedback.size(), 0);
        }
    }
}

TEST_F(ScenarioEngineTest, SkipObjective) {
    engine_->load_scenario(scenario_);

    EXPECT_EQ(engine_->skip_objective(2), TrainingResult::Success);
    auto objectives = engine_->get_objectives();
    for (const auto& obj : objectives) {
        if (obj.id == 2) {
            EXPECT_EQ(obj.status, ObjectiveStatus::Skipped);
        }
    }
}

TEST_F(ScenarioEngineTest, ObjectiveNotFound) {
    engine_->load_scenario(scenario_);

    EXPECT_EQ(engine_->complete_objective(999), TrainingResult::ObjectiveNotFound);
    EXPECT_EQ(engine_->fail_objective(999, "reason"), TrainingResult::ObjectiveNotFound);
    EXPECT_EQ(engine_->skip_objective(999), TrainingResult::ObjectiveNotFound);
}

TEST_F(ScenarioEngineTest, Rules) {
    engine_->load_scenario(scenario_);

    ScriptRule rule;
    rule.id = 1;
    rule.name = "Test Rule";
    rule.trigger.type = TriggerType::OnTime;
    rule.trigger.parameter = 10.0;

    ScriptAction action;
    action.type = ActionType::ShowMessage;
    action.parameters = "Hello World";
    rule.actions.push_back(action);

    EXPECT_EQ(engine_->add_rule(rule), TrainingResult::Success);
    auto rules = engine_->get_rules();
    EXPECT_GT(rules.size(), 0);

    EXPECT_EQ(engine_->disable_rule(1), TrainingResult::Success);
    EXPECT_EQ(engine_->enable_rule(1), TrainingResult::Success);
    EXPECT_EQ(engine_->remove_rule(1), TrainingResult::Success);
}

TEST_F(ScenarioEngineTest, Triggers) {
    engine_->load_scenario(scenario_);

    ScriptRule rule;
    rule.id = 1;
    rule.trigger.id = 1;
    rule.trigger.type = TriggerType::AfterDelay;
    rule.trigger.parameter = 0.0;  // Immediate

    ScriptAction action;
    action.type = ActionType::ShowMessage;
    rule.actions.push_back(action);

    engine_->add_rule(rule);
    engine_->start_scenario();

    EXPECT_EQ(engine_->evaluate_triggers(), TrainingResult::Success);
}

TEST_F(ScenarioEngineTest, Checkpoints) {
    engine_->load_scenario(scenario_);
    engine_->start_scenario();

    EXPECT_EQ(engine_->save_checkpoint("checkpoint_1"), TrainingResult::Success);
    auto checkpoints = engine_->get_checkpoints();
    EXPECT_GT(checkpoints.size(), 0);

    engine_->update(5.0);  // Advance time

    EXPECT_EQ(engine_->jump_to_checkpoint("checkpoint_1"), TrainingResult::Success);
}

TEST_F(ScenarioEngineTest, Reset) {
    engine_->load_scenario(scenario_);
    engine_->complete_objective(1);

    EXPECT_EQ(engine_->reset_scenario(), TrainingResult::Success);

    auto objectives = engine_->get_objectives();
    for (const auto& obj : objectives) {
        EXPECT_EQ(obj.status, ObjectiveStatus::NotStarted);
    }
}

TEST_F(ScenarioEngineTest, Update) {
    engine_->load_scenario(scenario_);
    engine_->start_scenario();

    EXPECT_EQ(engine_->update(0.1), TrainingResult::Success);
}

TEST_F(ScenarioEngineTest, UpdateWithoutRunning) {
    engine_->load_scenario(scenario_);
    // Not started
    EXPECT_EQ(engine_->update(0.1), TrainingResult::SessionNotActive);
}

// ============================================================================
// Training Session Tests
// ============================================================================

class TrainingSessionTest : public ::testing::Test {
protected:
    void SetUp() override {
        session_ = create_training_session();
        ASSERT_NE(session_, nullptr);
    }

    std::unique_ptr<ITrainingSession> session_;
};

TEST_F(TrainingSessionTest, Creation) {
    EXPECT_NE(session_, nullptr);
    EXPECT_FALSE(session_->is_active());
}

TEST_F(TrainingSessionTest, CreateSession) {
    EXPECT_EQ(session_->create_session("trainee_001", TrainingDomain::Aviation), TrainingResult::Success);
    EXPECT_EQ(session_->get_trainee_id(), "trainee_001");
    EXPECT_FALSE(session_->get_session_id().empty());
}

TEST_F(TrainingSessionTest, AviationSession) {
    session_->create_session("pilot", TrainingDomain::Aviation);
    session_->start_session();

    auto cockpit = session_->get_cockpit_module();
    EXPECT_NE(cockpit, nullptr);
    EXPECT_TRUE(cockpit->is_ready());
}

TEST_F(TrainingSessionTest, NavalSession) {
    session_->create_session("navigator", TrainingDomain::Naval);
    session_->start_session();

    auto bridge = session_->get_bridge_module();
    EXPECT_NE(bridge, nullptr);
    EXPECT_TRUE(bridge->is_ready());
}

TEST_F(TrainingSessionTest, ATCSession) {
    session_->create_session("controller", TrainingDomain::AirTrafficControl);
    session_->start_session();

    auto tower = session_->get_tower_module();
    EXPECT_NE(tower, nullptr);
    EXPECT_TRUE(tower->is_ready());
}

TEST_F(TrainingSessionTest, SessionLifecycle) {
    session_->create_session("trainee", TrainingDomain::Aviation);

    EXPECT_EQ(session_->start_session(), TrainingResult::Success);
    EXPECT_TRUE(session_->is_active());

    EXPECT_EQ(session_->pause_session(), TrainingResult::Success);
    EXPECT_TRUE(session_->is_active());  // Still active, just paused

    EXPECT_EQ(session_->resume_session(), TrainingResult::Success);

    EXPECT_EQ(session_->end_session(), TrainingResult::Success);
    EXPECT_FALSE(session_->is_active());
}

TEST_F(TrainingSessionTest, SessionTime) {
    session_->create_session("trainee", TrainingDomain::Aviation);
    session_->start_session();

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    Real time = session_->get_session_time();
    EXPECT_GT(time, 0.0);

    session_->end_session();
}

TEST_F(TrainingSessionTest, Difficulty) {
    session_->create_session("trainee", TrainingDomain::Aviation);

    EXPECT_EQ(session_->set_difficulty(DifficultyLevel::Expert), TrainingResult::Success);
    EXPECT_EQ(session_->get_difficulty(), DifficultyLevel::Expert);
}

TEST_F(TrainingSessionTest, Weather) {
    session_->create_session("trainee", TrainingDomain::Aviation);

    EXPECT_EQ(session_->set_weather(WeatherPreset::Thunderstorm), TrainingResult::Success);
}

TEST_F(TrainingSessionTest, TimeOfDay) {
    session_->create_session("trainee", TrainingDomain::Aviation);

    EXPECT_EQ(session_->set_time_of_day(TimeOfDay::Night), TrainingResult::Success);
}

TEST_F(TrainingSessionTest, AAR) {
    session_->create_session("trainee", TrainingDomain::Aviation);
    session_->start_session();

    auto aar = session_->get_aar();
    EXPECT_NE(aar, nullptr);
    EXPECT_TRUE(aar->is_recording());

    session_->end_session();
    EXPECT_FALSE(aar->is_recording());
}

TEST_F(TrainingSessionTest, ScenarioEngine) {
    session_->create_session("trainee", TrainingDomain::Aviation);
    session_->start_session();

    auto scenario = session_->get_scenario_engine();
    EXPECT_NE(scenario, nullptr);

    session_->end_session();
}

TEST_F(TrainingSessionTest, Statistics) {
    session_->create_session("trainee", TrainingDomain::Aviation);
    session_->start_session();

    UInt32 total = session_->get_total_objectives();
    UInt32 completed = session_->get_completed_objectives();
    Real score = session_->get_current_score();

    EXPECT_GE(total, 0);
    EXPECT_GE(completed, 0);
    EXPECT_GE(score, 0.0);

    session_->end_session();
}

TEST_F(TrainingSessionTest, Update) {
    session_->create_session("trainee", TrainingDomain::Aviation);
    session_->start_session();

    EXPECT_EQ(session_->update(0.1), TrainingResult::Success);

    session_->end_session();
}

TEST_F(TrainingSessionTest, SessionNotActiveErrors) {
    EXPECT_EQ(session_->pause_session(), TrainingResult::SessionNotActive);
    EXPECT_EQ(session_->resume_session(), TrainingResult::SessionNotActive);
    EXPECT_EQ(session_->end_session(), TrainingResult::SessionNotActive);
}

TEST_F(TrainingSessionTest, StartWithoutCreate) {
    EXPECT_EQ(session_->start_session(), TrainingResult::NotInitialized);
}

// ============================================================================
// Factory Function Tests
// ============================================================================

TEST(FactoryTest, CreateCockpitModule) {
    auto cockpit = create_cockpit_module("F16");
    EXPECT_NE(cockpit, nullptr);

    auto generic = create_generic_cockpit_module();
    EXPECT_NE(generic, nullptr);
}

TEST(FactoryTest, CreateBridgeModule) {
    auto bridge = create_bridge_module("DDG51");
    EXPECT_NE(bridge, nullptr);

    auto generic = create_generic_bridge_module();
    EXPECT_NE(generic, nullptr);
}

TEST(FactoryTest, CreateTowerModule) {
    auto tower = create_tower_module("KLAX");
    EXPECT_NE(tower, nullptr);
}

TEST(FactoryTest, CreateAAR) {
    auto aar = create_aar();
    EXPECT_NE(aar, nullptr);
}

TEST(FactoryTest, CreateScenarioEngine) {
    auto engine = create_scenario_engine();
    EXPECT_NE(engine, nullptr);
}

TEST(FactoryTest, CreateTrainingSession) {
    auto session = create_training_session();
    EXPECT_NE(session, nullptr);

    auto mock = create_mock_training_session();
    EXPECT_NE(mock, nullptr);
}

// ============================================================================
// Data Structure Tests
// ============================================================================

TEST(DataStructureTest, InstrumentState) {
    InstrumentState inst;
    inst.type = InstrumentType::Airspeed;
    inst.name = "Airspeed Indicator";
    inst.value = 150.0;
    inst.min_value = 0.0;
    inst.max_value = 300.0;
    inst.is_active = true;
    inst.has_warning = false;
    inst.has_failure = false;
    inst.units = "kts";

    EXPECT_EQ(inst.type, InstrumentType::Airspeed);
    EXPECT_EQ(inst.name, "Airspeed Indicator");
    EXPECT_NEAR(inst.value, 150.0, 0.01);
}

TEST(DataStructureTest, ControlState) {
    ControlState ctrl;
    ctrl.type = ControlType::Throttle;
    ctrl.name = "Throttle";
    ctrl.position = 0.75;
    ctrl.force = 0.0;
    ctrl.is_switch = false;

    EXPECT_EQ(ctrl.type, ControlType::Throttle);
    EXPECT_NEAR(ctrl.position, 0.75, 0.01);
}

TEST(DataStructureTest, AircraftTrack) {
    AircraftTrack track;
    track.callsign = "AAL123";
    track.aircraft_type = "B738";
    track.altitude = 35000;
    track.heading = 270;
    track.ground_speed = 450;
    track.squawk = "4521";
    track.is_arriving = true;

    EXPECT_EQ(track.callsign, "AAL123");
    EXPECT_EQ(track.altitude, 35000);
}

TEST(DataStructureTest, TrainingObjective) {
    TrainingObjective obj;
    obj.id = 1;
    obj.type = ObjectiveType::Landing;
    obj.name = "Land safely";
    obj.status = ObjectiveStatus::InProgress;
    obj.score = 85.0;
    obj.weight = 2.0;
    obj.is_mandatory = true;
    obj.is_graded = true;

    EXPECT_EQ(obj.id, 1);
    EXPECT_EQ(obj.type, ObjectiveType::Landing);
    EXPECT_EQ(obj.status, ObjectiveStatus::InProgress);
}

TEST(DataStructureTest, TrainingEvent) {
    TrainingEvent event;
    event.id = 100;
    event.type = TrainingEventType::ErrorMade;
    event.name = "Altitude Deviation";
    event.timestamp = 120.5;
    event.severity = 0.3;

    EXPECT_EQ(event.id, 100);
    EXPECT_EQ(event.type, TrainingEventType::ErrorMade);
}

TEST(DataStructureTest, StateSnapshot) {
    StateSnapshot snapshot;
    snapshot.timestamp = 60.0;
    snapshot.airspeed = 250.0;
    snapshot.altitude = 10000.0;
    snapshot.heading = 90.0;
    snapshot.pitch = 5.0;
    snapshot.roll = 0.0;
    snapshot.elevator = 0.1;
    snapshot.throttle = 0.8;

    EXPECT_NEAR(snapshot.airspeed, 250.0, 0.01);
    EXPECT_NEAR(snapshot.altitude, 10000.0, 0.01);
}

TEST(DataStructureTest, ScenarioDefinition) {
    ScenarioDefinition scenario;
    scenario.id = "basic_flight";
    scenario.name = "Basic Flight Training";
    scenario.domain = TrainingDomain::Aviation;
    scenario.difficulty = DifficultyLevel::Beginner;
    scenario.weather = WeatherPreset::Clear;
    scenario.estimated_duration = 3600.0;

    EXPECT_EQ(scenario.id, "basic_flight");
    EXPECT_EQ(scenario.domain, TrainingDomain::Aviation);
}

TEST(DataStructureTest, GradeReport) {
    GradeReport report;
    report.overall_grade = Grade::Good;
    report.overall_score = 85.0;
    report.total_time = 1800.0;
    report.is_passed = true;
    report.strengths.push_back("Good airmanship");
    report.areas_for_improvement.push_back("Landing technique");

    EXPECT_EQ(report.overall_grade, Grade::Good);
    EXPECT_TRUE(report.is_passed);
    EXPECT_EQ(report.strengths.size(), 1);
}

TEST(DataStructureTest, PlaybackState) {
    PlaybackState state;
    state.mode = PlaybackMode::Playing;
    state.current_time = 30.0;
    state.duration = 120.0;
    state.playback_speed = 2.0;
    state.loop_enabled = true;

    EXPECT_EQ(state.mode, PlaybackMode::Playing);
    EXPECT_NEAR(state.playback_speed, 2.0, 0.01);
}

// ============================================================================
// Integration Tests
// ============================================================================

TEST(IntegrationTest, CompleteTrainingFlow) {
    // Create session
    auto session = create_training_session();
    session->create_session("test_trainee", TrainingDomain::Aviation);

    // Configure
    session->set_difficulty(DifficultyLevel::Intermediate);
    session->set_weather(WeatherPreset::Clear);
    session->set_time_of_day(TimeOfDay::Morning);

    // Start session
    EXPECT_EQ(session->start_session(), TrainingResult::Success);

    // Get modules
    auto cockpit = session->get_cockpit_module();
    auto aar = session->get_aar();
    auto scenario = session->get_scenario_engine();

    EXPECT_NE(cockpit, nullptr);
    EXPECT_NE(aar, nullptr);
    EXPECT_NE(scenario, nullptr);

    // Simulate flight
    cockpit->set_control_position(ControlType::Throttle, 1.0);

    for (int i = 0; i < 10; ++i) {
        session->update(0.1);
    }

    // Check flight state changed
    EXPECT_GT(cockpit->get_airspeed(), 0.0);

    // End session
    EXPECT_EQ(session->end_session(), TrainingResult::Success);

    // Generate grade report
    auto report = aar->generate_grade_report();
    EXPECT_GE(report.overall_score, 0.0);
}

TEST(IntegrationTest, ScenarioExecution) {
    auto session = create_training_session();
    session->create_session("trainee", TrainingDomain::Aviation);
    session->start_session();

    auto scenario = session->get_scenario_engine();

    // Create and load scenario
    ScenarioDefinition def;
    def.id = "test";
    def.name = "Test Scenario";
    def.domain = TrainingDomain::Aviation;

    TrainingObjective obj;
    obj.id = 1;
    obj.type = ObjectiveType::Takeoff;
    obj.name = "Takeoff";
    def.objectives.push_back(obj);

    scenario->load_scenario(def);
    scenario->start_scenario();

    // Execute scenario
    scenario->update(1.0);
    scenario->complete_objective(1);

    // Check objective completed
    auto objectives = scenario->get_objectives();
    EXPECT_EQ(objectives[0].status, ObjectiveStatus::Completed);

    session->end_session();
}
