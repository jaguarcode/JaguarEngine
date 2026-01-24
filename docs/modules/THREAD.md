# Digital Thread Module - Entity Lifecycle and Degradation Tracking

## Overview

The Digital Thread module provides comprehensive entity lifecycle tracking, state history management, and physics-based degradation modeling throughout the complete design-to-decommission lifecycle. It enables entities to be tracked from conception through active operation, maintenance cycles, and eventual retirement, supporting long-term audit trails, predictive maintenance, and detailed historical analysis.

## Key Features

- **Lifecycle Management**: Track entities through Design, Production, Operational, Maintenance, and Decommissioned phases
- **Phase Transitions**: Configurable validation rules and audit trail for all state changes
- **Event Logging**: Record significant lifecycle events with metadata and timestamps
- **State History**: Time-series snapshots with full, delta, and checkpoint types
- **Retention Policies**: Automatic cleanup with configurable retention periods per snapshot type
- **Query API**: Advanced temporal queries with filtering and pagination
- **Multi-Format Export**: Parquet, CSV, JSON, and binary export formats
- **Degradation Modeling**: Physics-based wear calculations tracking component health
- **Predictive Maintenance**: Failure prediction and maintenance recommendations
- **Observer Pattern**: Real-time notifications for lifecycle and event changes
- **Persistent Storage**: Pluggable backends for long-term audit trails

## Architecture

### Core Components

```
┌─────────────────────────────────────────────────────────────────┐
│                    Digital Thread Module                        │
├─────────────────────────────────────────────────────────────────┤
│  ┌──────────────────┐  ┌──────────────────┐  ┌──────────────┐  │
│  │ LifecycleManager │  │  HistoryStore    │  │Degradation   │  │
│  │                  │  │                  │  │  Model       │  │
│  │ - Phases         │  │ - Snapshots      │  │ - Components │  │
│  │ - Transitions    │  │ - Queries        │  │ - Wear       │  │
│  │ - Events         │  │ - Retention      │  │ - Predictions│  │
│  │ - Observers      │  │ - Export         │  │ - Maintenance│  │
│  └──────────────────┘  └──────────────────┘  └──────────────┘  │
│  ┌────────────────────────────────────────────────────────────┐ │
│  │          Storage Backends & Exporters                      │ │
│  │  Memory | Disk | Parquet | CSV | JSON | Binary            │ │
│  └────────────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────────┘
     ↓
PartitionManager (Cloud module)
DistributedTimeManager (Cloud module)
```

### Component Relationships

- **LifecycleManager**: Primary interface for tracking entities through lifecycle phases
- **HistoryStore**: Maintains time-series snapshots of entity state with efficient storage and querying
- **DegradationModel**: Tracks component health and predicts maintenance needs
- **Storage Backends**: Pluggable implementations for different storage strategies
- **Exporters**: Multi-format export for offline analysis

## Lifecycle Management

### Overview

The lifecycle management system tracks entities from design through decommissioning, providing complete audit trails and phase transition validation.

### Lifecycle Phases

```
Design ──────────────→ Production ──────────────→ Operational
  ↑                        ↑                           ↓
  └────────────────────────┴──────────── Maintenance ─┘
                                           ↓
                                   Decommissioned (Terminal)
```

**Valid Transitions:**
- **Design**: → Production (manufacturing begins) or → Decommissioned (cancelled)
- **Production**: → Operational (ready for use) or → Design (rework/redesign)
- **Operational**: → Maintenance (requires service) or → Decommissioned (end of service)
- **Maintenance**: → Operational (fixed) or → Decommissioned (irreparable)
- **Decommissioned**: Terminal state, no further transitions

### LifecycleManager API

```cpp
#include <jaguar/thread/lifecycle_manager.h>

using namespace jaguar::thread;

// Create with default configuration
auto manager = create_lifecycle_manager(LifecycleConfig::default_config());

// Initialize
manager->initialize();

// Track an entity
LifecycleResult result = manager->track_entity(entity_id, LifecyclePhase::Design);

// Transition to next phase
result = manager->transition(
    entity_id,
    LifecyclePhase::Production,
    "Ready for manufacturing",
    "user-123"
);

// Query current phase
std::optional<LifecyclePhase> phase = manager->get_phase(entity_id);
if (phase && *phase == LifecyclePhase::Operational) {
    // Entity is in production use
}

// Check if transition is valid
bool valid = manager->is_valid_transition(
    LifecyclePhase::Operational,
    LifecyclePhase::Maintenance
);

// Get complete lifecycle info
std::optional<EntityLifecycleInfo> info = manager->get_lifecycle_info(entity_id);
if (info) {
    std::cout << "Total transitions: " << info->transition_count << "\n";
    std::cout << "Current phase: " << lifecycle_phase_to_string(info->current_phase) << "\n";
}

// Get all entities in a phase
std::vector<EntityId> operational = manager->get_entities_in_phase(LifecyclePhase::Operational);

// Query history
std::vector<LifecycleTransition> history = manager->get_transition_history(entity_id);
for (const auto& transition : history) {
    std::cout << lifecycle_phase_to_string(transition.from_phase) << " → "
              << lifecycle_phase_to_string(transition.to_phase) << "\n";
}

// Get time metrics
auto time_in_phase = manager->get_time_in_phase(entity_id);
auto lifetime = manager->get_lifetime(entity_id);

// Statistics
LifecycleStats stats = manager->get_stats();
std::cout << "Total entities: " << stats.total_entities << "\n";
std::cout << "Total transitions: " << stats.total_transitions << "\n";
```

### Event Logging

Log significant events throughout an entity's lifecycle:

```cpp
// Log event
LifecycleEvent event(entity_id, "inspection", "Scheduled maintenance inspection");
event.metadata["technician"] = "john-smith";
event.metadata["findings"] = "wear-on-bearing";
manager->log_event(entity_id, event);

// Or use convenience overload
manager->log_event(entity_id, "repair_completed", "Bearing replaced, tested");

// Retrieve event history
std::vector<LifecycleEvent> events = manager->get_event_history(entity_id);
for (const auto& evt : events) {
    std::cout << evt.event_type << ": " << evt.description << "\n";
}
```

### Observer Pattern

Register observers to receive notifications about lifecycle changes:

```cpp
class MaintenanceObserver : public ILifecycleObserver {
public:
    void on_phase_changed(EntityId entity_id,
                         LifecyclePhase from_phase,
                         LifecyclePhase to_phase) override {
        std::cout << "Entity " << entity_id << " moved to "
                  << lifecycle_phase_to_string(to_phase) << "\n";
    }

    void on_event_logged(EntityId entity_id,
                        const LifecycleEvent& event) override {
        if (event.event_type == "inspection") {
            schedule_follow_up(entity_id);
        }
    }
};

auto observer = std::make_shared<MaintenanceObserver>();
manager->add_observer(observer);

// Events trigger callbacks
manager->transition(entity_id, LifecyclePhase::Maintenance, "Needs service");
// → on_phase_changed() called

manager->log_event(entity_id, "inspection", "Visual inspection completed");
// → on_event_logged() called
```

### Persistent Storage

Save and load lifecycle data:

```cpp
// Set storage backend
auto storage = create_memory_storage();
manager->set_storage(storage);

// Save single entity
manager->save_entity(entity_id);

// Save all tracked entities
manager->save_all();

// Load from storage
manager->load_entity(entity_id);
manager->load_all();

// Cleanup old data
UInt64 removed = manager->cleanup_old_data();  // Based on retention_period
UInt64 trimmed = manager->trim_history();      // Based on max_history_size
```

### Configuration

```cpp
// Default configuration (balanced)
auto config = LifecycleConfig::default_config();
// - max_history_size: 1000 entries per entity
// - retention_period: 30 days
// - strict_transitions: true
// - event_logging: enabled

// Minimal configuration (light-weight)
auto config = LifecycleConfig::minimal();
// - max_history_size: 100
// - retention_period: no limit
// - strict_transitions: false
// - event_logging: disabled

// Full audit configuration (compliance)
auto config = LifecycleConfig::full_audit();
// - max_history_size: 10,000
// - retention_period: 1 year
// - strict_transitions: true

// Production configuration (balanced audit)
auto config = LifecycleConfig::production();
// - max_history_size: 5,000
// - retention_period: 90 days
// - strict_transitions: true

auto manager = create_lifecycle_manager(config);
```

## History Store

### Overview

The History Store maintains time-series snapshots of entity state for debugging, replay, and offline analysis.

### Snapshot Types

- **Full**: Complete entity state (high fidelity, larger size)
- **Delta**: Changes since last snapshot (compact, efficient)
- **Checkpoint**: Periodic full snapshots for recovery points

### HistoryStore API

```cpp
#include <jaguar/thread/history_store.h>

using namespace jaguar::thread;

// Create with default configuration
auto store = create_history_store(HistoryStoreConfig::default_config());
store->initialize();

// Store a snapshot
StateSnapshot snapshot;
snapshot.entity_id = entity_id;
snapshot.sequence_number = 1;
snapshot.type = SnapshotType::Full;
snapshot.timestamp = std::chrono::system_clock::now();
snapshot.state_data = serialize_entity_state(entity);
snapshot.state_hash = compute_state_hash(snapshot.state_data);
snapshot.source_node = "node-1";

HistoryResult result = store->store_snapshot(snapshot);

// Batch storage
std::vector<StateSnapshot> snapshots;
for (auto& entity : entities) {
    snapshots.push_back(create_snapshot(entity));
}
result = store->store_snapshots(snapshots);

// Retrieve specific snapshot
auto snap = store->get_snapshot(entity_id, sequence_number);

// Get latest snapshot
auto latest = store->get_latest_snapshot(entity_id);

// Get snapshot at specific time
auto snap_at_time = store->get_snapshot_at_time(
    entity_id,
    std::chrono::system_clock::now() - std::chrono::hours(1)
);

// Get first snapshot (creation point)
auto first = store->get_first_snapshot(entity_id);
```

### Querying History

```cpp
// Query all snapshots for entity
HistoryQuery query = HistoryQuery::for_entity(entity_id, 100);
auto result = store->query(query);
std::cout << "Found " << result.count() << " snapshots\n";

// Query by time range
QueryTimeRange range(
    std::chrono::system_clock::now() - std::chrono::hours(24),
    std::chrono::system_clock::now()
);
query = HistoryQuery::for_time_range(range, 1000);
result = store->get_snapshots_in_range(range, 1000);

// Query by type
query = HistoryQuery::for_type(SnapshotType::Checkpoint, 50);
result = store->query(query);

// Advanced filtering
query.entity_id = entity_id;
query.time_range = QueryTimeRange(start_time, end_time);
query.snapshot_type = SnapshotType::Full;
query.limit = 500;
query.offset = 100;  // Pagination
query.ascending = false;  // Newest first (default)
result = store->query(query);

// Access results
for (const auto& snapshot : result.snapshots) {
    auto data = decompress_snapshot_data(snapshot.state_data);
    EntityState state = deserialize_entity_state(data);
    process_snapshot(state);
}

// Check if more results available
if (result.has_more) {
    query.offset += query.limit;  // Get next page
    result = store->query(query);
}
```

### Retention Management

```cpp
// Default retention policy
RetentionPolicy policy = RetentionPolicy::default_policy();
// - Full: 30 days
// - Delta: 7 days
// - Checkpoint: 90 days
// - Max: 10,000 per entity

// Aggressive retention (short-term, real-time)
policy = RetentionPolicy::aggressive();
// - Full: 1 day
// - Delta: 6 hours
// - Checkpoint: 3 days

// Relaxed retention (long-term, analysis)
policy = RetentionPolicy::relaxed();
// - Full: 90 days
// - Delta: 30 days
// - Checkpoint: 1 year

store->set_retention_policy(policy);

// Apply retention manually
store->apply_retention();

// Auto-cleanup runs periodically (configurable interval)
// Delete old snapshots in background

// Manual deletion
store->delete_snapshot(entity_id, sequence_number);
store->delete_entity_history(entity_id);
store->delete_before(cutoff_time);  // Delete older than cutoff
```

### Data Export

```cpp
// Export to Parquet (columnar format, efficient)
ExportConfig config = ExportConfig::to_parquet("/data/history.parquet");
store->export_history(config);

// Export to CSV
config = ExportConfig::to_csv("/data/history.csv");
config.query = HistoryQuery::for_entity(entity_id);
config.batch_size = 5000;
store->export_history(config);

// Export to JSON Lines
config = ExportConfig::to_json("/data/history.jsonl");
store->export_history(config);

// Configure export options
config.compress = true;           // Gzip compression
config.batch_size = 10000;        // Records per batch
config.format = ExportFormat::Parquet;
config.output_path = "/data/export.parquet";
config.query = HistoryQuery::for_time_range(
    time_range_last_days(7),
    1000000
);

// Execute export
HistoryResult result = store->export_history(config);
if (result != HistoryResult::Success) {
    std::cerr << "Export failed: " << history_result_to_string(result) << "\n";
}
```

### Configuration Presets

```cpp
// In-memory (no persistence, aggressive cleanup)
auto config = HistoryStoreConfig::in_memory();
// - Max: 1M snapshots, 512MB
// - Aggressive retention (7 days)

// Persistent (disk-backed, long-term storage)
auto config = HistoryStoreConfig::persistent("/data/history");
// - Max: 100M snapshots, 2GB
// - Relaxed retention (90 days)

// High-frequency capture (many snapshots)
auto config = HistoryStoreConfig::high_frequency();
// - Max: 50M snapshots, 4GB
// - Compression enabled
// - Indexing enabled

auto store = create_history_store(config);
```

## Degradation Modeling

### Overview

The degradation model tracks component health, simulates wear under operating conditions, and predicts failures for proactive maintenance planning.

### Component Types

- **Mechanical**: Gears, bearings, joints (friction-based wear)
- **Electrical**: Motors, sensors, wiring (electrical aging)
- **Hydraulic**: Pumps, valves, lines (fluid degradation)
- **Structural**: Frame, body, supports (fatigue and corrosion)
- **Electronic**: Controllers, processors (thermal and electrical stress)
- **Consumable**: Filters, fluids, seals (rapid wear)

### Degradation Types

- **Wear**: Mechanical friction degradation
- **Fatigue**: Stress-cycle fatigue
- **Corrosion**: Chemical/environmental attack
- **Thermal**: Heat-related degradation
- **Chemical**: Chemical reaction degradation
- **Electrical**: Electrical component aging
- **Combined**: Multiple mechanisms

### Health Status Categories

- **Excellent**: > 90% (no action needed)
- **Good**: 70-90% (monitor closely)
- **Fair**: 50-70% (schedule maintenance)
- **Poor**: 30-50% (urgent maintenance)
- **Critical**: < 30% (immediate replacement)
- **Failed**: 0% (component non-functional)

### DegradationModel API

```cpp
#include <jaguar/thread/degradation_model.h>

using namespace jaguar::thread;

// Create model
auto config = DegradationModelConfig::default_config();
auto model = create_degradation_model(config);
model->initialize();

// Register entity and components
model->register_entity(entity_id);

model->add_component(entity_id, "left_wheel", ComponentType::Mechanical, 100.0);
model->add_component(entity_id, "motor", ComponentType::Electrical, 100.0);
model->add_component(entity_id, "hydraulic_pump", ComponentType::Hydraulic, 100.0);

// Update operating conditions
OperatingConditions conditions;
conditions.load_factor = 0.8;           // 80% of max load
conditions.temperature_celsius = 85.0;  // Elevated temperature
conditions.humidity_percent = 65.0;     // Moderate humidity
conditions.vibration_level = 0.5;       // Medium vibration
conditions.contamination_level = 0.2;   // Slight contamination
conditions.continuous_operation = std::chrono::hours(6);

model->update_operating_conditions(entity_id, conditions);

// Apply wear over time
model->apply_wear(entity_id, std::chrono::hours(24));  // 1 day of operation

// Get component health
auto wheel_health = model->get_component_health(entity_id, "left_wheel");
if (wheel_health) {
    std::cout << "Wheel health: " << wheel_health->health_percentage << "%\n";
    std::cout << "Status: " << health_status_to_string(wheel_health->status) << "\n";
    std::cout << "RUL: " << wheel_health->remaining_useful_life.count() << " hours\n";
}

// Get overall entity degradation
auto degradation = model->get_entity_degradation(entity_id);
if (degradation) {
    std::cout << "Overall health: " << degradation->overall_health << "%\n";
    std::cout << "MTTF: " << degradation->mean_time_to_failure.count() << " hours\n";
}

// Record maintenance performed
model->record_maintenance(entity_id, "left_wheel", MaintenanceAction::Replace);

// Get statistics
DegradationStats stats = model->get_stats();
std::cout << "Entities tracked: " << stats.total_entities << "\n";
std::cout << "Critical components: " << stats.critical_components << "\n";
```

### Wear Factors

Configure wear parameters for each component type:

```cpp
WearFactor mechanical_wear;
mechanical_wear.base_rate = 0.001;              // Degradation per hour
mechanical_wear.load_multiplier = 1.5;          // Higher load = faster wear
mechanical_wear.temperature_coefficient = 0.01; // Sensitivity to temperature
mechanical_wear.humidity_coefficient = 0.001;   // Sensitivity to humidity
mechanical_wear.age_acceleration = 0.0001;      // Older components degrade faster

// Automatic wear factors from configuration
auto config = DegradationModelConfig::default_config();
// Includes precomputed factors for all component types
```

### Maintenance Recommendations

```cpp
// Get recommendations for an entity
std::vector<MaintenanceRecommendation> recommendations =
    model->get_recommendations(entity_id);

for (const auto& rec : recommendations) {
    std::cout << "Component: " << rec.component_id << "\n";
    std::cout << "Action: " << maintenance_action_to_string(rec.action) << "\n";
    std::cout << "Description: " << rec.description << "\n";
    std::cout << "Urgency: " << (rec.urgency * 100) << "%\n";
    std::cout << "Recommended within: " << rec.recommended_within.count() << " hours\n";
    std::cout << "Estimated cost: $" << rec.estimated_cost << "\n";
    std::cout << "Confidence: " << (rec.confidence * 100) << "%\n";
}

// Sort by urgency
auto sorted = sort_recommendations_by_urgency(recommendations);
for (const auto& rec : sorted) {
    // Process most urgent first
}
```

### Failure Predictions

```cpp
// Get failure predictions
std::vector<FailurePrediction> predictions = model->get_predictions(entity_id);

for (const auto& pred : predictions) {
    std::cout << "Component: " << pred.component_id << " expected to fail\n";

    auto now = std::chrono::system_clock::now();
    auto time_to_failure = pred.predicted_failure - now;
    auto hours = std::chrono::duration_cast<std::chrono::hours>(time_to_failure).count();

    std::cout << "Time to failure: " << hours << " hours\n";
    std::cout << "Confidence: " << (pred.confidence * 100) << "%\n";
    std::cout << "Failure mode: " << pred.failure_mode << "\n";

    // Show preventive actions
    for (const auto& action : pred.preventive_actions) {
        std::cout << "  → " << action.description << "\n";
    }
}
```

### Custom Calculators

```cpp
// Use linear degradation calculator (simple)
auto linear_calc = create_linear_calculator();
model->set_calculator(linear_calc);
// wear = base_rate * load * (1 + temp_coeff * ΔT) * time

// Use exponential degradation calculator (age-aware)
auto exp_calc = create_exponential_calculator();
model->set_calculator(exp_calc);
// wear = base_rate * load * exp(age * age_accel) * time

// Create custom calculator
class CustomCalculator : public IDegradationCalculator {
public:
    Real calculate_wear(const ComponentHealth& health,
                       const OperatingConditions& conditions,
                       std::chrono::hours duration) override {
        // Your custom wear model
        return 0.0;
    }

    Real estimate_remaining_life(const ComponentHealth& health,
                                 const OperatingConditions& conditions) override {
        // Your custom RUL estimation
        return 1000.0;
    }

    HealthStatus determine_status(Real health_percentage) override {
        // Your custom status categorization
        return health_percentage_to_status(health_percentage);
    }
};

auto custom = std::make_shared<CustomCalculator>();
model->set_calculator(custom);
```

### Configuration Presets

```cpp
// Default (balanced assessment every 60 minutes)
auto config = DegradationModelConfig::default_config();

// High-frequency (real-time assessment every 15 minutes)
auto config = DegradationModelConfig::high_frequency();
// - Assessment interval: 15 minutes
// - Prediction horizon: 7 days
// - For critical systems

// Long-term tracking (daily assessment)
auto config = DegradationModelConfig::long_term();
// - Assessment interval: 24 hours
// - Prediction horizon: 1 year
// - For slow-degrading components

// Critical systems (every 5 minutes, high confidence)
auto config = DegradationModelConfig::critical_systems();
// - Assessment interval: 5 minutes
// - Prediction horizon: 7 days
// - Minimum confidence: 90%

auto model = create_degradation_model(config);
```

## API Reference

### Result Codes

```cpp
// Lifecycle Operations
enum class LifecycleResult : UInt8 {
    Success,
    InvalidConfiguration, InvalidEntityId, InvalidPhase,
    TransitionFailed, TransitionNotAllowed, ObserverFailed,
    NotInitialized, AlreadyInitialized, EntityNotTracked,
    OutOfMemory, StorageError
};

// History Operations
enum class HistoryResult : UInt8 {
    Success,
    InvalidConfiguration, InvalidEntityId, InvalidTimeRange, InvalidQuery,
    StoreFailed, RetrieveFailed, DeleteFailed, ExportFailed,
    NotInitialized, AlreadyInitialized, EntityNotFound, SnapshotNotFound,
    OutOfMemory, StorageError, CapacityExceeded,
    ExportNotSupported, ExportIOError
};

// Degradation Operations
enum class DegradationResult : UInt8 {
    Success,
    InvalidConfiguration, InvalidEntityId, InvalidComponent, InvalidModel,
    CalculationFailed, PredictionFailed, UpdateFailed,
    NotInitialized, AlreadyInitialized, ComponentNotFound,
    OutOfMemory, ModelNotLoaded
};
```

### Helper Functions

```cpp
// Lifecycle helpers
std::vector<LifecyclePhase> get_valid_transitions(LifecyclePhase phase);
bool is_transition_valid(LifecyclePhase from, LifecyclePhase to);
std::string format_timestamp(const std::chrono::system_clock::time_point& tp);
std::chrono::seconds calculate_duration(
    const std::chrono::system_clock::time_point& start,
    const std::chrono::system_clock::time_point& end);

// History helpers
UInt64 compute_state_hash(const std::vector<UInt8>& data);
bool verify_snapshot_integrity(const StateSnapshot& snapshot);
std::vector<UInt8> compress_snapshot_data(const std::vector<UInt8>& data);
std::vector<UInt8> decompress_snapshot_data(const std::vector<UInt8>& data);
UInt64 calculate_snapshot_age_hours(
    const StateSnapshot& snapshot,
    std::chrono::system_clock::time_point now = std::chrono::system_clock::now());
QueryTimeRange time_range_last_hours(UInt64 hours);
QueryTimeRange time_range_last_days(UInt64 days);

// Degradation helpers
HealthStatus health_percentage_to_status(Real health_percentage);
Real calculate_urgency_from_rul(
    std::chrono::hours rul,
    std::chrono::hours threshold = std::chrono::hours(168));
Real calculate_weighted_health(const std::vector<ComponentHealth>& components);
std::chrono::hours estimate_mttf(const std::vector<ComponentHealth>& components);
std::vector<ComponentHealth> sort_by_urgency(std::vector<ComponentHealth> components);
Real calculate_environmental_severity(const OperatingConditions& conditions);
bool has_critical_components(const std::vector<ComponentHealth>& components);
std::string generate_health_report(const ComponentHealth& health);
std::string generate_recommendation_report(const MaintenanceRecommendation& rec);
```

## Usage Examples

### Complete Lifecycle Tracking

```cpp
#include <jaguar/thread/lifecycle_manager.h>
#include <jaguar/thread/history_store.h>
#include <jaguar/thread/degradation_model.h>

using namespace jaguar::thread;

class EntityDigitalThread {
private:
    std::unique_ptr<LifecycleManager> lifecycle_;
    std::unique_ptr<HistoryStore> history_;
    std::unique_ptr<DegradationModel> degradation_;

public:
    void initialize(EntityId entity_id) {
        // Initialize managers
        lifecycle_ = create_lifecycle_manager(LifecycleConfig::production());
        history_ = create_history_store(HistoryStoreConfig::default_config());
        degradation_ = create_degradation_model(DegradationModelConfig::default_config());

        lifecycle_->initialize();
        history_->initialize();
        degradation_->initialize();

        // Track entity from design phase
        lifecycle_->track_entity(entity_id, LifecyclePhase::Design);

        // Register degradation tracking
        degradation_->register_entity(entity_id);
        degradation_->add_component(entity_id, "engine", ComponentType::Mechanical);
        degradation_->add_component(entity_id, "battery", ComponentType::Electrical);
    }

    void transition_to_production(EntityId entity_id) {
        lifecycle_->transition(
            entity_id,
            LifecyclePhase::Production,
            "Design approved, manufacturing began"
        );
        lifecycle_->log_event(entity_id, "design_review", "Design review passed");
    }

    void deploy_to_field(EntityId entity_id) {
        lifecycle_->transition(
            entity_id,
            LifecyclePhase::Operational,
            "Quality check complete, deployed to field"
        );
        lifecycle_->log_event(entity_id, "deployment", "Unit deployed to field location");
    }

    void update_operational_state(EntityId entity_id) {
        // Update conditions and capture state
        OperatingConditions conditions;
        conditions.load_factor = 0.9;
        conditions.temperature_celsius = 75.0;
        conditions.humidity_percent = 60.0;

        degradation_->update_operating_conditions(entity_id, conditions);
        degradation_->apply_wear(entity_id, std::chrono::hours(1));

        // Capture snapshot
        StateSnapshot snapshot;
        snapshot.entity_id = entity_id;
        snapshot.timestamp = std::chrono::system_clock::now();
        snapshot.type = SnapshotType::Delta;
        snapshot.state_data = capture_entity_state(entity_id);
        snapshot.state_hash = compute_state_hash(snapshot.state_data);

        history_->store_snapshot(snapshot);

        // Check for maintenance needs
        auto degradation = degradation_->get_entity_degradation(entity_id);
        if (degradation && degradation->overall_health < 50.0) {
            lifecycle_->transition(
                entity_id,
                LifecyclePhase::Maintenance,
                "Health dropped below 50%, scheduling maintenance"
            );

            auto recs = degradation_->get_recommendations(entity_id);
            for (const auto& rec : recs) {
                lifecycle_->log_event(
                    entity_id,
                    "maintenance_recommended",
                    rec.description
                );
            }
        }
    }

    void perform_maintenance(EntityId entity_id) {
        degradation_->record_maintenance(entity_id, "engine", MaintenanceAction::Repair);
        lifecycle_->log_event(entity_id, "maintenance", "Engine repaired, tested");
        lifecycle_->transition(entity_id, LifecyclePhase::Operational, "Maintenance complete");
    }

    void retirement(EntityId entity_id) {
        lifecycle_->transition(
            entity_id,
            LifecyclePhase::Decommissioned,
            "Reached end of service life"
        );
        lifecycle_->log_event(entity_id, "retirement", "Unit retired from service");
    }

    void export_analysis() {
        // Export historical data for analysis
        ExportConfig config = ExportConfig::to_parquet("/data/analysis.parquet");
        config.query = HistoryQuery::for_time_range(
            time_range_last_days(30),
            100000
        );
        history_->export_history(config);
    }

    std::vector<UInt8> capture_entity_state(EntityId entity_id) {
        // Serialize current entity state
        return std::vector<UInt8>();
    }
};
```

### Real-Time Monitoring

```cpp
void monitor_entity(EntityId entity_id, DegradationModel* model) {
    while (true) {
        auto degradation = model->get_entity_degradation(entity_id);
        if (!degradation) break;

        std::cout << "Entity " << entity_id << " Health Report:\n";
        std::cout << "Overall Health: " << degradation->overall_health << "%\n";
        std::cout << "Components:\n";

        for (const auto& component : degradation->components) {
            std::cout << "  " << component.component_id << ": "
                      << health_status_to_string(component.status) << " ("
                      << component.health_percentage << "%)\n";

            if (component.health_percentage < 50.0) {
                std::cout << "    WARNING: RUL = " << component.remaining_useful_life.count()
                          << " hours\n";
            }
        }

        // Check for critical predictions
        auto predictions = model->get_predictions(entity_id);
        if (!predictions.empty()) {
            std::cout << "Failure Predictions:\n";
            for (const auto& pred : predictions) {
                std::cout << "  " << pred.component_id << " - Confidence: "
                          << (pred.confidence * 100) << "%\n";
            }
        }

        std::this_thread::sleep_for(std::chrono::seconds(60));
    }
}
```

## Performance Considerations

### Memory Usage

| Component | Bytes per Entity |
|-----------|-----------------|
| Lifecycle info | ~200 |
| Snapshot (full) | Variable (depends on entity state) |
| Component health | ~64 |
| Degradation model | ~100 |
| Event record | ~256 |

### Time Complexity

| Operation | Complexity |
|-----------|-----------|
| Track entity | O(1) |
| Transition phase | O(1) |
| Log event | O(1) |
| Query history | O(log n) with indexing |
| Apply wear | O(m) where m = component count |
| Calculate recommendations | O(m * k) where k = rule complexity |

### Scaling Guidelines

| Entity Count | Recommended Configuration |
|--------------|--------------------------|
| < 1,000 | Default (1000 history, 30 day retention) |
| 1,000 - 10,000 | Production (5000 history, 90 day retention) |
| 10,000 - 100,000 | Persistent storage with aggressive cleanup |
| > 100,000 | High-frequency config with delta snapshots |

### Storage Optimization

- **Snapshot Compression**: Enable compression to reduce storage by 50-80%
- **Delta Snapshots**: Use delta type to store only changes (~10-20% of full snapshot size)
- **Retention Policies**: Aggressive retention for short-term analysis, relaxed for compliance
- **Indexing**: Enable indexing for fast queries on large datasets

## Thread Safety

All module components are thread-safe:

- LifecycleManager operations use mutex-protected maps
- HistoryStore queries use read-write locks for concurrent access
- DegradationModel wear updates are atomic per entity
- Statistics collection is lock-free when possible

## Integration with Cloud Module

### With PartitionManager

```cpp
// Sync lifecycle phase with partition strategy
if (lifecycle_->get_phase(entity_id) == LifecyclePhase::Operational) {
    // Include entity in active partition
    partition_manager->assign_entity(entity_id, position, domain);
} else if (lifecycle_->get_phase(entity_id) == LifecyclePhase::Decommissioned) {
    // Remove from partitions
    partition_manager->remove_entity(entity_id);
}
```

### With DistributedTimeManager

```cpp
// Use distributed time for event timestamps
SimulationTime sim_time = time_manager->get_simulation_time();
LifecycleEvent event(entity_id, "state_change", "Description");
event.timestamp = std::chrono::system_clock::now();
lifecycle_->log_event(entity_id, event);
```

### State Synchronization

```cpp
// Export lifecycle state for replication
EntityLifecycleInfo info = lifecycle_->get_lifecycle_info(entity_id);
// Sync to other nodes via state synchronizer
```

## Error Handling

```cpp
LifecycleResult result = lifecycle_->track_entity(entity_id);
switch (result) {
    case LifecycleResult::Success:
        // Entity tracked
        break;
    case LifecycleResult::InvalidEntityId:
        std::cerr << "Invalid entity ID\n";
        break;
    case LifecycleResult::AlreadyInitialized:
        std::cerr << "Entity already tracked\n";
        break;
    default:
        std::cerr << "Error: " << lifecycle_result_to_string(result) << "\n";
}

HistoryResult hist_result = history_->store_snapshot(snapshot);
if (hist_result != HistoryResult::Success) {
    std::cerr << "Failed to store: " << history_result_to_string(hist_result) << "\n";
}

DegradationResult deg_result = degradation_->apply_wear(entity_id, duration);
if (deg_result != DegradationResult::Success) {
    std::cerr << "Wear calculation failed\n";
}
```

## See Also

- [Cloud Module](CLOUD.md) - Distributed simulation infrastructure
- [Physics Module](PHYSICS.md) - Entity physics integration
- [Core Module](CORE.md) - Basic types and utilities
- [API Reference](../API_REFERENCE.md) - Complete API documentation
