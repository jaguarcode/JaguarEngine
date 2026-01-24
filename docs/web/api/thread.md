# Digital Thread API Reference

Digital Thread module (`jaguar_thread`) provides lifecycle tracking, history management, and predictive degradation modeling for digital twin applications.

## Overview

The Digital Thread module enables:
- **History Store**: State snapshots, delta tracking, and time-travel debugging
- **Degradation Model**: Physics-based failure prediction and maintenance scheduling

---

## History Store

### Header

```cpp
#include <jaguar/thread/history_store.h>
```

### Core Types

```cpp
namespace jaguar::thread {

// Snapshot types
enum class SnapshotType {
    Full,           // Complete state capture
    Delta,          // Changes since last snapshot
    Checkpoint,     // Recoverable checkpoint
    Incremental     // Streaming incremental updates
};

// Export formats
enum class ExportFormat {
    Parquet,        // Apache Parquet (columnar)
    CSV,            // Comma-separated values
    JSON,           // JSON format
    Binary          // Native binary format
};

// State snapshot
struct StateSnapshot {
    uint64_t snapshot_id;
    uint64_t version;
    SnapshotType type;
    std::chrono::system_clock::time_point timestamp;
    std::chrono::nanoseconds simulation_time;
    std::vector<uint8_t> state_data;
    std::vector<uint8_t> metadata;
    size_t compressed_size;
    size_t uncompressed_size;
    std::string checksum;
};

// History query
struct HistoryQuery {
    std::optional<uint64_t> entity_id;
    std::optional<std::chrono::system_clock::time_point> start_time;
    std::optional<std::chrono::system_clock::time_point> end_time;
    std::optional<std::chrono::nanoseconds> sim_time_start;
    std::optional<std::chrono::nanoseconds> sim_time_end;
    std::optional<SnapshotType> snapshot_type;
    size_t limit = 1000;
    size_t offset = 0;
    bool include_data = true;
    bool ascending = true;
};

// Retention policy
struct RetentionPolicy {
    std::chrono::hours max_age{24 * 30};          // 30 days default
    size_t max_snapshots = 10000;
    size_t max_storage_bytes = 10ULL * 1024 * 1024 * 1024;  // 10 GB
    bool auto_compact = true;
    std::chrono::hours compact_interval{24};
    std::vector<SnapshotType> preserve_types = {SnapshotType::Checkpoint};
};

// History result
struct HistoryResult {
    bool success;
    std::string error_message;
    uint64_t snapshot_id;
    size_t snapshots_affected;
    std::chrono::microseconds duration;
};

// Configuration
struct HistoryStoreConfig {
    std::string storage_path = "./history";
    SnapshotType default_snapshot_type = SnapshotType::Delta;
    std::chrono::milliseconds snapshot_interval{1000};
    bool compression_enabled = true;
    int compression_level = 6;
    bool async_writes = true;
    size_t write_buffer_size = 1024 * 1024;  // 1 MB
    RetentionPolicy retention;
};

}  // namespace jaguar::thread
```

### HistoryStore Class

```cpp
class HistoryStore {
public:
    // Initialize history store
    HistoryResult initialize(const HistoryStoreConfig& config);

    // Shutdown
    void shutdown();

    // Capture current state
    HistoryResult capture_snapshot(const void* state_data, size_t size,
                                   SnapshotType type = SnapshotType::Delta);

    // Capture entity-specific state
    HistoryResult capture_entity_snapshot(uint64_t entity_id,
                                          const void* state_data, size_t size);

    // Retrieve snapshot by ID
    std::optional<StateSnapshot> get_snapshot(uint64_t snapshot_id) const;

    // Query snapshots
    std::vector<StateSnapshot> query_snapshots(const HistoryQuery& query) const;

    // Get latest snapshot
    std::optional<StateSnapshot> get_latest_snapshot() const;

    // Get snapshot at simulation time
    std::optional<StateSnapshot> get_snapshot_at_time(
        std::chrono::nanoseconds sim_time) const;

    // Get snapshots in time range
    std::vector<StateSnapshot> get_snapshots_in_range(
        std::chrono::nanoseconds start,
        std::chrono::nanoseconds end) const;

    // Restore from snapshot
    HistoryResult restore_snapshot(uint64_t snapshot_id);

    // Create checkpoint (guaranteed recoverable)
    HistoryResult create_checkpoint(const std::string& name = "");

    // List checkpoints
    std::vector<StateSnapshot> list_checkpoints() const;

    // Delete snapshot
    HistoryResult delete_snapshot(uint64_t snapshot_id);

    // Compact history (apply retention policy)
    HistoryResult compact();

    // Export history
    HistoryResult export_history(const std::string& output_path,
                                 ExportFormat format,
                                 const HistoryQuery& query = {});

    // Import history
    HistoryResult import_history(const std::string& input_path);

    // Get storage statistics
    struct StorageStats {
        size_t total_snapshots;
        size_t total_bytes;
        size_t compressed_bytes;
        double compression_ratio;
        std::chrono::system_clock::time_point oldest_snapshot;
        std::chrono::system_clock::time_point newest_snapshot;
        size_t checkpoints_count;
    };
    StorageStats get_storage_stats() const;
};

// Factory functions
std::unique_ptr<HistoryStore> create_history_store(
    const HistoryStoreConfig& config);
std::unique_ptr<IHistoryBackend> create_memory_backend();
std::unique_ptr<IHistoryBackend> create_file_backend(const std::string& path);
std::unique_ptr<IHistoryExporter> create_parquet_exporter();
std::unique_ptr<IHistoryExporter> create_csv_exporter();
```

---

## Degradation Model

### Header

```cpp
#include <jaguar/thread/degradation_model.h>
```

### Core Types

```cpp
namespace jaguar::thread {

// Degradation mechanisms
enum class DegradationType {
    Wear,           // Mechanical wear
    Fatigue,        // Cyclic fatigue
    Corrosion,      // Environmental corrosion
    Thermal,        // Thermal degradation
    Electrical,     // Electrical aging
    Chemical,       // Chemical degradation
    Radiation,      // Radiation damage
    Creep,          // Material creep
    Erosion         // Surface erosion
};

// Component types
enum class ComponentType {
    Structural,     // Airframe, hull, chassis
    Propulsion,     // Engine, motor, thruster
    Avionics,       // Electronics, sensors
    Hydraulic,      // Hydraulic systems
    Electrical,     // Electrical systems
    Fuel,           // Fuel systems
    Landing,        // Landing gear, wheels
    Control,        // Control surfaces, actuators
    Environmental,  // HVAC, pressurization
    Weapon          // Weapon systems
};

// Health status
enum class HealthStatus {
    Excellent,      // 90-100% health
    Good,           // 70-90% health
    Fair,           // 50-70% health
    Poor,           // 30-50% health
    Critical,       // 10-30% health
    Failed          // 0-10% health
};

// Maintenance action types
enum class MaintenanceAction {
    Inspect,        // Visual/instrument inspection
    Service,        // Routine service
    Repair,         // Component repair
    Replace,        // Component replacement
    Overhaul,       // Major overhaul
    Condemn         // Remove from service
};

// Component health
struct ComponentHealth {
    std::string component_id;
    std::string component_name;
    ComponentType type;
    HealthStatus status;
    double health_percentage;       // 0.0 - 100.0
    double degradation_rate;        // % per hour
    std::chrono::hours operating_hours;
    std::chrono::hours hours_since_maintenance;
    size_t cycle_count;
    std::vector<DegradationType> active_degradation_modes;
};

// Operating conditions
struct OperatingConditions {
    double temperature_celsius;
    double humidity_percent;
    double altitude_meters;
    double speed_mps;
    double load_factor;             // g-force
    double vibration_level;         // m/s^2
    bool is_combat;
    bool is_extreme_environment;
    std::chrono::hours continuous_operation;
};

// Wear factor
struct WearFactor {
    DegradationType type;
    double base_rate;               // Base degradation rate
    double environment_multiplier;
    double load_multiplier;
    double cumulative_damage;
};

// Failure prediction
struct FailurePrediction {
    std::string component_id;
    double failure_probability;     // 0.0 - 1.0
    std::chrono::hours time_to_failure_mean;
    std::chrono::hours time_to_failure_std;
    std::chrono::hours time_to_failure_p10;   // 10th percentile
    std::chrono::hours time_to_failure_p90;   // 90th percentile
    DegradationType likely_failure_mode;
    double confidence;              // 0.0 - 1.0
};

// Maintenance recommendation
struct MaintenanceRecommendation {
    std::string component_id;
    MaintenanceAction action;
    std::string description;
    std::chrono::hours recommended_within;
    double urgency;                 // 0.0 - 1.0
    double estimated_cost;
    double estimated_downtime_hours;
    double risk_if_deferred;        // 0.0 - 1.0
    std::vector<std::string> required_parts;
    std::vector<std::string> required_tools;
};

// Degradation result
struct DegradationResult {
    bool success;
    std::string error_message;
    ComponentHealth health;
    std::chrono::microseconds computation_time;
};

// Configuration
struct DegradationConfig {
    bool enable_wear = true;
    bool enable_fatigue = true;
    bool enable_corrosion = true;
    bool enable_thermal = true;
    double base_wear_rate = 0.001;          // % per hour
    double fatigue_exponent = 3.0;          // Paris law exponent
    double corrosion_rate_base = 0.0001;    // % per hour
    double thermal_activation_energy = 50000;  // J/mol
    std::chrono::hours prediction_horizon{1000};
    size_t monte_carlo_samples = 10000;
};

}  // namespace jaguar::thread
```

### DegradationModel Class

```cpp
class DegradationModel {
public:
    // Initialize degradation model
    DegradationResult initialize(const DegradationConfig& config);

    // Shutdown
    void shutdown();

    // Register component
    DegradationResult register_component(
        const std::string& component_id,
        const std::string& name,
        ComponentType type,
        double initial_health = 100.0);

    // Update component state
    DegradationResult update_component(
        const std::string& component_id,
        const OperatingConditions& conditions,
        std::chrono::hours delta_time);

    // Get component health
    ComponentHealth get_component_health(const std::string& component_id) const;

    // Get all component healths
    std::vector<ComponentHealth> get_all_component_healths() const;

    // Get components by status
    std::vector<ComponentHealth> get_components_by_status(
        HealthStatus status) const;

    // Calculate wear factors
    std::vector<WearFactor> calculate_wear_factors(
        const std::string& component_id,
        const OperatingConditions& conditions) const;

    // Predict failure
    FailurePrediction predict_failure(const std::string& component_id) const;

    // Predict all failures
    std::vector<FailurePrediction> predict_all_failures() const;

    // Get maintenance recommendations
    MaintenanceRecommendation get_maintenance_recommendation(
        const std::string& component_id) const;

    // Get all maintenance recommendations
    std::vector<MaintenanceRecommendation> get_all_maintenance_recommendations() const;

    // Get maintenance schedule
    struct MaintenanceSchedule {
        std::vector<MaintenanceRecommendation> immediate;   // Within 24 hours
        std::vector<MaintenanceRecommendation> soon;        // Within 7 days
        std::vector<MaintenanceRecommendation> scheduled;   // Within 30 days
        std::vector<MaintenanceRecommendation> planned;     // Within 90 days
    };
    MaintenanceSchedule get_maintenance_schedule() const;

    // Record maintenance action
    DegradationResult record_maintenance(
        const std::string& component_id,
        MaintenanceAction action,
        double health_restored = 100.0);

    // Simulate degradation over time
    std::vector<ComponentHealth> simulate_degradation(
        const std::string& component_id,
        const OperatingConditions& conditions,
        std::chrono::hours duration,
        std::chrono::hours step_size) const;

    // Get fleet statistics (multiple entities)
    struct FleetHealth {
        size_t total_components;
        size_t excellent_count;
        size_t good_count;
        size_t fair_count;
        size_t poor_count;
        size_t critical_count;
        size_t failed_count;
        double average_health;
        std::vector<std::string> components_needing_attention;
    };
    FleetHealth get_fleet_health() const;
};

// Factory functions
std::unique_ptr<DegradationModel> create_degradation_model(
    const DegradationConfig& config);
std::unique_ptr<IDegradationCalculator> create_linear_calculator();
std::unique_ptr<IDegradationCalculator> create_paris_law_calculator(
    double exponent);
std::unique_ptr<IFailurePredictor> create_weibull_predictor();
std::unique_ptr<IMaintenanceAdvisor> create_rule_based_advisor();
std::unique_ptr<IMaintenanceAdvisor> create_ml_based_advisor(
    const std::string& model_path);
```

---

## Usage Examples

### History Store for Time-Travel Debugging

```cpp
#include <jaguar/thread/history_store.h>

using namespace jaguar::thread;

// Configure history store
HistoryStoreConfig config;
config.storage_path = "./simulation_history";
config.default_snapshot_type = SnapshotType::Delta;
config.snapshot_interval = std::chrono::milliseconds{100};
config.compression_enabled = true;

config.retention.max_age = std::chrono::hours{24 * 7};  // 1 week
config.retention.max_storage_bytes = 50ULL * 1024 * 1024 * 1024;  // 50 GB
config.retention.auto_compact = true;

auto history = create_history_store(config);
history->initialize(config);

// Simulation loop with automatic snapshots
while (running) {
    engine.step(dt);

    // Capture snapshot every 100ms
    auto state = engine.serialize_state();
    history->capture_snapshot(state.data(), state.size());

    // Create checkpoint at mission waypoints
    if (reached_waypoint) {
        history->create_checkpoint("waypoint_" + std::to_string(waypoint_id));
    }
}

// Time-travel debugging: restore to specific time
auto sim_time = std::chrono::nanoseconds{5000000000};  // 5 seconds
auto snapshot = history->get_snapshot_at_time(sim_time);
if (snapshot) {
    history->restore_snapshot(snapshot->snapshot_id);
    std::cout << "Restored to simulation time: "
              << snapshot->simulation_time.count() << " ns\n";
}

// Query history for analysis
HistoryQuery query;
query.start_time = start;
query.end_time = end;
query.entity_id = aircraft_id;

auto snapshots = history->query_snapshots(query);
for (const auto& snap : snapshots) {
    analyze_state(snap.state_data);
}

// Export for offline analysis
history->export_history("./analysis/mission_data.parquet",
                       ExportFormat::Parquet);
```

### Predictive Maintenance

```cpp
#include <jaguar/thread/degradation_model.h>

using namespace jaguar::thread;

// Configure degradation model
DegradationConfig config;
config.enable_wear = true;
config.enable_fatigue = true;
config.enable_thermal = true;
config.prediction_horizon = std::chrono::hours{2000};
config.monte_carlo_samples = 50000;

auto model = create_degradation_model(config);
model->initialize(config);

// Register aircraft components
model->register_component("engine-1", "Port Engine", ComponentType::Propulsion);
model->register_component("engine-2", "Starboard Engine", ComponentType::Propulsion);
model->register_component("landing-gear", "Main Landing Gear", ComponentType::Landing);
model->register_component("avionics-suite", "Avionics Suite", ComponentType::Avionics);
model->register_component("airframe", "Airframe Structure", ComponentType::Structural);

// During simulation, update with operating conditions
OperatingConditions conditions;
conditions.temperature_celsius = 35.0;
conditions.altitude_meters = 10000;
conditions.speed_mps = 250;
conditions.load_factor = 1.5;
conditions.vibration_level = 2.0;
conditions.is_combat = false;

model->update_component("engine-1", conditions, std::chrono::hours{1});
model->update_component("engine-2", conditions, std::chrono::hours{1});

// Check component health
auto engine_health = model->get_component_health("engine-1");
std::cout << "Engine 1 health: " << engine_health.health_percentage << "%\n";
std::cout << "Status: " << static_cast<int>(engine_health.status) << "\n";
std::cout << "Operating hours: " << engine_health.operating_hours.count() << "\n";

// Get failure prediction
auto prediction = model->predict_failure("engine-1");
std::cout << "Failure probability (next 1000h): "
          << prediction.failure_probability * 100 << "%\n";
std::cout << "Mean time to failure: "
          << prediction.time_to_failure_mean.count() << " hours\n";

// Get maintenance recommendations
auto schedule = model->get_maintenance_schedule();

std::cout << "\n=== IMMEDIATE MAINTENANCE ===\n";
for (const auto& rec : schedule.immediate) {
    std::cout << rec.component_id << ": " << rec.description << "\n";
    std::cout << "  Urgency: " << rec.urgency << "\n";
    std::cout << "  Est. cost: $" << rec.estimated_cost << "\n";
}

std::cout << "\n=== SCHEDULED MAINTENANCE ===\n";
for (const auto& rec : schedule.scheduled) {
    std::cout << rec.component_id << ": " << rec.description << "\n";
    std::cout << "  Due within: " << rec.recommended_within.count() << " hours\n";
}

// Record maintenance performed
model->record_maintenance("engine-1", MaintenanceAction::Service, 95.0);

// Fleet-level health summary
auto fleet = model->get_fleet_health();
std::cout << "\n=== FLEET HEALTH ===\n";
std::cout << "Average health: " << fleet.average_health << "%\n";
std::cout << "Components needing attention: "
          << fleet.components_needing_attention.size() << "\n";
```

### Digital Twin Integration

```cpp
#include <jaguar/thread/history_store.h>
#include <jaguar/thread/degradation_model.h>

using namespace jaguar::thread;

// Integrated digital twin
class DigitalTwin {
    std::unique_ptr<HistoryStore> history_;
    std::unique_ptr<DegradationModel> degradation_;

public:
    void initialize() {
        // Initialize history store
        HistoryStoreConfig hist_cfg;
        hist_cfg.storage_path = "./digital_twin/history";
        history_ = create_history_store(hist_cfg);
        history_->initialize(hist_cfg);

        // Initialize degradation model
        DegradationConfig deg_cfg;
        degradation_ = create_degradation_model(deg_cfg);
        degradation_->initialize(deg_cfg);
    }

    void update(const SimulationState& state,
                const OperatingConditions& conditions,
                std::chrono::hours delta) {
        // Capture state history
        auto serialized = serialize(state);
        history_->capture_snapshot(serialized.data(), serialized.size());

        // Update degradation for all components
        for (const auto& component : state.components) {
            degradation_->update_component(component.id, conditions, delta);
        }
    }

    MaintenanceSchedule get_maintenance_schedule() const {
        return degradation_->get_maintenance_schedule();
    }

    void replay_from(std::chrono::nanoseconds sim_time) {
        auto snapshot = history_->get_snapshot_at_time(sim_time);
        if (snapshot) {
            restore_state(snapshot->state_data);
        }
    }
};
```

---

## See Also

- [Architecture](../advanced/architecture.md) - System architecture overview
- [Cloud Burst API](cloud.md) - Distributed simulation
- [Machine Learning API](ml.md) - Neural networks for prediction
- [Configuration](configuration.md) - Engine configuration
