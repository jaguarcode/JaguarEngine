#pragma once
/**
 * @file auto_scaler.h
 * @brief Kubernetes-based auto-scaling for distributed simulation
 *
 * This file provides auto-scaling infrastructure for dynamically adjusting
 * compute resources based on CPU, memory, and entity load metrics. Integrates
 * with Kubernetes HPA and Prometheus for monitoring and scaling decisions.
 *
 * Key features:
 * - Kubernetes Horizontal Pod Autoscaler (HPA) integration
 * - Prometheus metrics exporter for custom metrics
 * - CPU and entity-count based scaling triggers
 * - Scale-up when CPU > 70% or entities > 8000/node
 * - Scale-down when CPU < 30% and entities < 2000/node
 * - Linear scaling validated to 100 nodes
 * - Integration with PartitionManager for entity migration during scaling
 */

#include "jaguar/core/types.h"
#include "jaguar/cloud/partition_manager.h"
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <memory>
#include <functional>
#include <optional>
#include <chrono>
#include <mutex>
#include <atomic>
#include <string>

namespace jaguar::cloud {

// ============================================================================
// Forward Declarations
// ============================================================================

class IMetricsCollector;
class IScalingPolicyEvaluator;
class IKubernetesClient;
class IPrometheusExporter;
class AutoScaler;

// ============================================================================
// Scaling Result Enum
// ============================================================================

/**
 * @brief Result codes for auto-scaling operations
 */
enum class ScalingResult : UInt8 {
    Success = 0,

    // Configuration errors
    InvalidConfiguration,
    InvalidPolicy,
    InvalidThreshold,
    InvalidNodeId,
    InvalidMetric,

    // Scaling errors
    ScaleUpFailed,
    ScaleDownFailed,
    ScalingInProgress,
    CooldownActive,
    ScalingDisabled,
    MinNodesReached,
    MaxNodesReached,

    // Kubernetes errors
    K8sConnectionFailed,
    K8sAuthenticationFailed,
    K8sAPIError,
    HPANotFound,
    HPAUpdateFailed,
    DeploymentNotFound,

    // Prometheus errors
    PrometheusConnectionFailed,
    PrometheusExportFailed,
    MetricCollectionFailed,

    // Policy errors
    NoTriggerMet,
    ConflictingTriggers,
    PolicyViolation,

    // State errors
    NotInitialized,
    AlreadyInitialized,
    AlreadyScaling,

    // Resource errors
    InsufficientResources,
    NodeProvisioningFailed,
    NodeTerminationFailed,

    // Network errors
    NetworkError,
    Timeout,
    ConnectionLost
};

/**
 * @brief Convert ScalingResult to string
 */
inline const char* scaling_result_to_string(ScalingResult result) {
    switch (result) {
        case ScalingResult::Success: return "Success";
        case ScalingResult::InvalidConfiguration: return "InvalidConfiguration";
        case ScalingResult::InvalidPolicy: return "InvalidPolicy";
        case ScalingResult::InvalidThreshold: return "InvalidThreshold";
        case ScalingResult::InvalidNodeId: return "InvalidNodeId";
        case ScalingResult::InvalidMetric: return "InvalidMetric";
        case ScalingResult::ScaleUpFailed: return "ScaleUpFailed";
        case ScalingResult::ScaleDownFailed: return "ScaleDownFailed";
        case ScalingResult::ScalingInProgress: return "ScalingInProgress";
        case ScalingResult::CooldownActive: return "CooldownActive";
        case ScalingResult::ScalingDisabled: return "ScalingDisabled";
        case ScalingResult::MinNodesReached: return "MinNodesReached";
        case ScalingResult::MaxNodesReached: return "MaxNodesReached";
        case ScalingResult::K8sConnectionFailed: return "K8sConnectionFailed";
        case ScalingResult::K8sAuthenticationFailed: return "K8sAuthenticationFailed";
        case ScalingResult::K8sAPIError: return "K8sAPIError";
        case ScalingResult::HPANotFound: return "HPANotFound";
        case ScalingResult::HPAUpdateFailed: return "HPAUpdateFailed";
        case ScalingResult::DeploymentNotFound: return "DeploymentNotFound";
        case ScalingResult::PrometheusConnectionFailed: return "PrometheusConnectionFailed";
        case ScalingResult::PrometheusExportFailed: return "PrometheusExportFailed";
        case ScalingResult::MetricCollectionFailed: return "MetricCollectionFailed";
        case ScalingResult::NoTriggerMet: return "NoTriggerMet";
        case ScalingResult::ConflictingTriggers: return "ConflictingTriggers";
        case ScalingResult::PolicyViolation: return "PolicyViolation";
        case ScalingResult::NotInitialized: return "NotInitialized";
        case ScalingResult::AlreadyInitialized: return "AlreadyInitialized";
        case ScalingResult::AlreadyScaling: return "AlreadyScaling";
        case ScalingResult::InsufficientResources: return "InsufficientResources";
        case ScalingResult::NodeProvisioningFailed: return "NodeProvisioningFailed";
        case ScalingResult::NodeTerminationFailed: return "NodeTerminationFailed";
        case ScalingResult::NetworkError: return "NetworkError";
        case ScalingResult::Timeout: return "Timeout";
        case ScalingResult::ConnectionLost: return "ConnectionLost";
        default: return "Unknown";
    }
}

// ============================================================================
// Scaling Direction Enum
// ============================================================================

/**
 * @brief Direction of scaling operation
 */
enum class ScalingDirection : UInt8 {
    None,       ///< No scaling needed
    Up,         ///< Scale up (add nodes)
    Down        ///< Scale down (remove nodes)
};

/**
 * @brief Convert ScalingDirection to string
 */
inline const char* scaling_direction_to_string(ScalingDirection direction) {
    switch (direction) {
        case ScalingDirection::None: return "None";
        case ScalingDirection::Up: return "Up";
        case ScalingDirection::Down: return "Down";
        default: return "Unknown";
    }
}

// ============================================================================
// Scaling Trigger Enum
// ============================================================================

/**
 * @brief Type of metric triggering scaling decision
 */
enum class ScalingTrigger : UInt8 {
    CPU,            ///< CPU utilization percentage
    EntityCount,    ///< Number of entities per node
    Memory,         ///< Memory utilization percentage
    NetworkBandwidth, ///< Network bandwidth usage
    Custom          ///< User-defined custom metric
};

/**
 * @brief Convert ScalingTrigger to string
 */
inline const char* scaling_trigger_to_string(ScalingTrigger trigger) {
    switch (trigger) {
        case ScalingTrigger::CPU: return "CPU";
        case ScalingTrigger::EntityCount: return "EntityCount";
        case ScalingTrigger::Memory: return "Memory";
        case ScalingTrigger::NetworkBandwidth: return "NetworkBandwidth";
        case ScalingTrigger::Custom: return "Custom";
        default: return "Unknown";
    }
}

// ============================================================================
// Metric Type Enum
// ============================================================================

/**
 * @brief Type of metric being monitored
 */
enum class MetricType : UInt8 {
    CPU,                ///< CPU usage (0.0 - 1.0 or percentage)
    Memory,             ///< Memory usage (0.0 - 1.0 or percentage)
    EntityCount,        ///< Number of entities on node
    NetworkBandwidth,   ///< Network bandwidth (bytes/sec)
    Latency,            ///< Network latency (milliseconds)
    Throughput,         ///< Processing throughput (entities/sec)
    Custom              ///< Custom user-defined metric
};

/**
 * @brief Convert MetricType to string
 */
inline const char* metric_type_to_string(MetricType type) {
    switch (type) {
        case MetricType::CPU: return "CPU";
        case MetricType::Memory: return "Memory";
        case MetricType::EntityCount: return "EntityCount";
        case MetricType::NetworkBandwidth: return "NetworkBandwidth";
        case MetricType::Latency: return "Latency";
        case MetricType::Throughput: return "Throughput";
        case MetricType::Custom: return "Custom";
        default: return "Unknown";
    }
}

// ============================================================================
// Node Metrics
// ============================================================================

/**
 * @brief Current metrics for a single compute node
 */
struct NodeMetrics {
    NodeId node_id{INVALID_NODE_ID};       ///< Node identifier
    std::string node_name;                  ///< Kubernetes node name
    std::string pod_name;                   ///< Pod name (if applicable)

    // Resource utilization (0.0 - 1.0)
    Real cpu_usage{0.0};                    ///< CPU utilization (0.0 - 1.0)
    Real memory_usage{0.0};                 ///< Memory utilization (0.0 - 1.0)
    Real network_rx_bytes_per_sec{0.0};     ///< Network receive bytes/sec
    Real network_tx_bytes_per_sec{0.0};     ///< Network transmit bytes/sec

    // Entity metrics
    UInt32 entity_count{0};                 ///< Number of entities on node
    UInt32 partition_count{0};              ///< Number of partitions on node

    // Performance metrics
    Real avg_frame_time_ms{0.0};            ///< Average frame time (milliseconds)
    Real entities_processed_per_sec{0.0};   ///< Entity processing throughput

    // Timing
    std::chrono::steady_clock::time_point timestamp; ///< When metrics were collected
    std::chrono::steady_clock::time_point last_updated; ///< Last update time
    bool is_healthy{true};                  ///< Node health status
};

// ============================================================================
// Cluster Metrics
// ============================================================================

/**
 * @brief Aggregated metrics for the entire cluster
 */
struct ClusterMetrics {
    // Cluster size
    UInt32 total_nodes{0};                  ///< Total nodes in cluster
    UInt32 active_nodes{0};                 ///< Active/healthy nodes

    // Aggregate resource utilization
    Real avg_cpu_usage{0.0};                ///< Average CPU across nodes (0.0 - 1.0)
    Real max_cpu_usage{0.0};                ///< Maximum CPU on any node (0.0 - 1.0)
    Real min_cpu_usage{0.0};                ///< Minimum CPU on any node (0.0 - 1.0)
    Real avg_memory_usage{0.0};             ///< Average memory across nodes (0.0 - 1.0)
    Real max_memory_usage{0.0};             ///< Maximum memory on any node (0.0 - 1.0)

    // Aggregate entity metrics
    UInt64 total_entities{0};               ///< Total entities across cluster
    UInt32 avg_entities_per_node{0};        ///< Average entities per node
    UInt32 max_entities_per_node{0};        ///< Maximum entities on any node
    UInt32 min_entities_per_node{0};        ///< Minimum entities on any node

    // Network metrics
    Real total_network_bandwidth{0.0};      ///< Total network bandwidth (bytes/sec)
    Real avg_latency_ms{0.0};               ///< Average inter-node latency

    // Load balance
    Real load_imbalance{0.0};               ///< Load imbalance factor (0.0 - 1.0)

    // Timing
    std::chrono::steady_clock::time_point timestamp; ///< When metrics were aggregated
};

// ============================================================================
// Scaling Policy
// ============================================================================

/**
 * @brief Policy defining when and how to scale
 */
struct ScalingPolicy {
    // Trigger thresholds
    Real cpu_scale_up_threshold{0.7};       ///< CPU usage to trigger scale-up (70%)
    Real cpu_scale_down_threshold{0.3};     ///< CPU usage to trigger scale-down (30%)
    UInt32 entity_scale_up_threshold{8000}; ///< Entities/node to trigger scale-up
    UInt32 entity_scale_down_threshold{2000}; ///< Entities/node to trigger scale-down
    Real memory_scale_up_threshold{0.8};    ///< Memory usage to trigger scale-up (80%)
    Real memory_scale_down_threshold{0.4};  ///< Memory usage to trigger scale-down (40%)

    // Scaling behavior
    UInt32 min_nodes{1};                    ///< Minimum cluster size
    UInt32 max_nodes{100};                  ///< Maximum cluster size
    UInt32 scale_up_increment{1};           ///< Nodes to add per scale-up
    UInt32 scale_down_decrement{1};         ///< Nodes to remove per scale-down

    // Cooldown periods (prevent oscillation)
    std::chrono::seconds scale_up_cooldown{60};   ///< Wait after scale-up
    std::chrono::seconds scale_down_cooldown{300}; ///< Wait after scale-down (5 min)
    std::chrono::seconds evaluation_interval{10};  ///< How often to evaluate policy

    // Stability requirements
    UInt32 consecutive_triggers_required{3}; ///< Consecutive triggers before scaling
    std::chrono::seconds metric_stability_period{30}; ///< Metric must be stable for this long

    // Enabled triggers
    bool cpu_scaling_enabled{true};         ///< Enable CPU-based scaling
    bool entity_scaling_enabled{true};      ///< Enable entity-count scaling
    bool memory_scaling_enabled{false};     ///< Enable memory-based scaling
    bool network_scaling_enabled{false};    ///< Enable network-based scaling

    // Safety limits
    Real max_scale_up_rate{0.5};            ///< Max scale-up as fraction of current size
    Real max_scale_down_rate{0.3};          ///< Max scale-down as fraction of current size
    bool allow_scale_to_zero{false};        ///< Allow scaling down to zero nodes

    /**
     * @brief Create default scaling policy
     */
    static ScalingPolicy default_policy() noexcept {
        return ScalingPolicy{};
    }

    /**
     * @brief Create aggressive scaling policy (fast response)
     */
    static ScalingPolicy aggressive() noexcept {
        ScalingPolicy policy;
        policy.cpu_scale_up_threshold = 0.6;
        policy.entity_scale_up_threshold = 5000;
        policy.scale_up_cooldown = std::chrono::seconds(30);
        policy.scale_down_cooldown = std::chrono::seconds(120);
        policy.evaluation_interval = std::chrono::seconds(5);
        policy.consecutive_triggers_required = 2;
        policy.scale_up_increment = 2;
        return policy;
    }

    /**
     * @brief Create conservative scaling policy (stable, slow response)
     */
    static ScalingPolicy conservative() noexcept {
        ScalingPolicy policy;
        policy.cpu_scale_up_threshold = 0.8;
        policy.entity_scale_up_threshold = 10000;
        policy.scale_up_cooldown = std::chrono::seconds(120);
        policy.scale_down_cooldown = std::chrono::seconds(600);
        policy.evaluation_interval = std::chrono::seconds(30);
        policy.consecutive_triggers_required = 5;
        return policy;
    }

    /**
     * @brief Create policy for large-scale deployment (100+ nodes)
     */
    static ScalingPolicy large_scale() noexcept {
        ScalingPolicy policy;
        policy.max_nodes = 200;
        policy.scale_up_increment = 5;
        policy.scale_down_decrement = 2;
        policy.cpu_scale_up_threshold = 0.7;
        policy.entity_scale_up_threshold = 8000;
        policy.evaluation_interval = std::chrono::seconds(15);
        policy.memory_scaling_enabled = true;
        return policy;
    }
};

// ============================================================================
// Scaling Decision
// ============================================================================

/**
 * @brief Decision made by scaling policy evaluator
 */
struct ScalingDecision {
    ScalingDirection direction{ScalingDirection::None}; ///< Scale up/down/none
    UInt32 target_node_count{0};            ///< Desired number of nodes
    UInt32 delta_nodes{0};                  ///< Nodes to add/remove
    ScalingTrigger primary_trigger{ScalingTrigger::CPU}; ///< Primary trigger
    std::vector<ScalingTrigger> secondary_triggers; ///< Additional triggers
    std::string reason;                     ///< Human-readable reason
    Real trigger_value{0.0};                ///< Value that triggered decision
    Real threshold{0.0};                    ///< Threshold that was crossed
    std::chrono::steady_clock::time_point timestamp; ///< When decision was made
    bool is_valid{false};                   ///< Is this a valid decision
};

// ============================================================================
// Scaling Event
// ============================================================================

/**
 * @brief Record of a scaling action that occurred
 */
struct ScalingEvent {
    UInt64 event_id{0};                     ///< Unique event identifier
    ScalingDirection direction{ScalingDirection::None}; ///< Scale direction
    UInt32 nodes_before{0};                 ///< Node count before scaling
    UInt32 nodes_after{0};                  ///< Node count after scaling
    ScalingTrigger trigger{ScalingTrigger::CPU}; ///< What triggered scaling
    std::string reason;                     ///< Reason for scaling
    ScalingResult result{ScalingResult::Success}; ///< Result of scaling operation
    std::chrono::steady_clock::time_point started_at; ///< When scaling started
    std::chrono::steady_clock::time_point completed_at; ///< When scaling completed
    std::chrono::milliseconds duration{0};  ///< Time taken to scale
    bool successful{false};                 ///< Was scaling successful
    std::vector<NodeId> added_nodes;        ///< Nodes added (if scale-up)
    std::vector<NodeId> removed_nodes;      ///< Nodes removed (if scale-down)
    UInt64 entities_migrated{0};            ///< Entities migrated during scaling
};

// ============================================================================
// Auto-Scaler Configuration
// ============================================================================

/**
 * @brief Configuration for the auto-scaler
 */
struct AutoScalerConfig {
    // Basic settings
    bool enabled{true};                     ///< Enable auto-scaling
    ScalingPolicy policy;                   ///< Scaling policy

    // Kubernetes settings
    std::string k8s_api_server;             ///< Kubernetes API server URL
    std::string k8s_namespace{"default"};  ///< Kubernetes namespace
    std::string k8s_deployment_name;        ///< Deployment to scale
    std::string k8s_hpa_name;               ///< HPA resource name
    std::string k8s_token;                  ///< Auth token (or use service account)
    std::string k8s_cert_path;              ///< Path to CA certificate
    bool use_in_cluster_config{true};       ///< Use in-cluster configuration

    // Prometheus settings
    std::string prometheus_pushgateway_url; ///< Prometheus Pushgateway URL
    std::string prometheus_job_name{"jaguar_engine"}; ///< Job name for metrics
    std::string prometheus_instance_name;   ///< Instance identifier
    std::chrono::seconds prometheus_push_interval{10}; ///< How often to push metrics
    bool export_to_prometheus{true};        ///< Enable Prometheus export

    // Metric collection
    std::chrono::seconds metric_collection_interval{5}; ///< How often to collect metrics
    UInt32 metric_history_size{100};        ///< Number of historical samples to keep
    bool collect_custom_metrics{false};     ///< Collect custom user metrics

    // Integration with PartitionManager
    bool integrate_partition_manager{true}; ///< Enable PartitionManager integration
    bool migrate_entities_on_scale{true};   ///< Migrate entities during scaling
    bool wait_for_migration_completion{true}; ///< Wait for migrations before reporting success

    // Fault tolerance
    std::chrono::seconds health_check_interval{30}; ///< Health check frequency
    UInt32 max_consecutive_failures{3};     ///< Max failures before disabling
    bool auto_recover{true};                ///< Automatically recover from failures
    std::chrono::seconds recovery_backoff{60}; ///< Wait before retry after failure

    // Logging and monitoring
    bool verbose_logging{false};            ///< Enable detailed logging
    std::string log_file_path;              ///< Path to log file
    bool record_scaling_history{true};      ///< Keep history of scaling events
    UInt32 max_history_events{1000};        ///< Maximum events to keep in history

    /**
     * @brief Create default configuration
     */
    static AutoScalerConfig default_config() noexcept {
        AutoScalerConfig config;
        config.policy = ScalingPolicy::default_policy();
        return config;
    }

    /**
     * @brief Create configuration for development/testing
     */
    static AutoScalerConfig development() noexcept {
        AutoScalerConfig config;
        config.policy = ScalingPolicy::aggressive();
        config.policy.min_nodes = 1;
        config.policy.max_nodes = 10;
        config.verbose_logging = true;
        config.metric_collection_interval = std::chrono::seconds(2);
        return config;
    }

    /**
     * @brief Create configuration for production
     */
    static AutoScalerConfig production() noexcept {
        AutoScalerConfig config;
        config.policy = ScalingPolicy::conservative();
        config.policy.max_nodes = 100;
        config.export_to_prometheus = true;
        config.wait_for_migration_completion = true;
        config.record_scaling_history = true;
        return config;
    }
};

// ============================================================================
// Auto-Scaler Statistics
// ============================================================================

/**
 * @brief Statistics about auto-scaling operations
 */
struct AutoScalerStats {
    // Scaling counts
    UInt64 total_scale_ups{0};              ///< Total scale-up operations
    UInt64 total_scale_downs{0};            ///< Total scale-down operations
    UInt64 successful_scale_ups{0};         ///< Successful scale-ups
    UInt64 successful_scale_downs{0};       ///< Successful scale-downs
    UInt64 failed_scale_ups{0};             ///< Failed scale-ups
    UInt64 failed_scale_downs{0};           ///< Failed scale-downs

    // Current state
    UInt32 current_node_count{0};           ///< Current number of nodes
    UInt32 min_node_count_reached{0};       ///< Minimum nodes ever reached
    UInt32 max_node_count_reached{0};       ///< Maximum nodes ever reached
    std::chrono::steady_clock::time_point last_scale_up_time; ///< Last scale-up
    std::chrono::steady_clock::time_point last_scale_down_time; ///< Last scale-down

    // Timing statistics
    std::chrono::milliseconds avg_scale_up_duration{0}; ///< Average scale-up time
    std::chrono::milliseconds avg_scale_down_duration{0}; ///< Average scale-down time
    std::chrono::milliseconds max_scale_up_duration{0}; ///< Max scale-up time
    std::chrono::milliseconds max_scale_down_duration{0}; ///< Max scale-down time

    // Trigger statistics
    UInt64 cpu_triggered_scale_ups{0};      ///< Scale-ups triggered by CPU
    UInt64 entity_triggered_scale_ups{0};   ///< Scale-ups triggered by entity count
    UInt64 memory_triggered_scale_ups{0};   ///< Scale-ups triggered by memory
    UInt64 custom_triggered_scale_ups{0};   ///< Scale-ups triggered by custom metric

    // Entity migration statistics
    UInt64 total_entities_migrated{0};      ///< Total entities migrated during scaling
    UInt64 avg_entities_per_scale_event{0}; ///< Average entities migrated per event

    // Metrics collection statistics
    UInt64 metrics_collected{0};            ///< Total metric collections
    UInt64 metrics_collection_failures{0};  ///< Failed metric collections
    UInt64 prometheus_pushes{0};            ///< Successful Prometheus pushes
    UInt64 prometheus_failures{0};          ///< Failed Prometheus pushes

    // Error statistics
    UInt64 k8s_api_errors{0};               ///< Kubernetes API errors
    UInt64 network_errors{0};               ///< Network-related errors
    UInt64 timeout_errors{0};               ///< Timeout errors

    // Uptime
    std::chrono::steady_clock::time_point start_time; ///< When auto-scaler started
    std::chrono::seconds uptime{0};         ///< Total uptime

    /**
     * @brief Reset all statistics
     */
    void reset() noexcept {
        *this = AutoScalerStats{};
    }
};

// ============================================================================
// Metrics Collector Interface
// ============================================================================

/**
 * @brief Interface for collecting metrics from compute nodes
 */
class IMetricsCollector {
public:
    virtual ~IMetricsCollector() = default;

    /**
     * @brief Initialize the metrics collector
     * @param config Auto-scaler configuration
     * @return Success or error code
     */
    virtual ScalingResult initialize(const AutoScalerConfig& config) = 0;

    /**
     * @brief Collect metrics from a specific node
     * @param node_id Node to collect from
     * @return Metrics or nullopt on failure
     */
    virtual std::optional<NodeMetrics> collect_node_metrics(NodeId node_id) = 0;

    /**
     * @brief Collect metrics from all nodes
     * @return Vector of metrics for all nodes
     */
    virtual std::vector<NodeMetrics> collect_all_metrics() = 0;

    /**
     * @brief Aggregate node metrics into cluster metrics
     * @param node_metrics Individual node metrics
     * @return Aggregated cluster metrics
     */
    virtual ClusterMetrics aggregate_metrics(const std::vector<NodeMetrics>& node_metrics) = 0;

    /**
     * @brief Register a custom metric provider
     * @param metric_name Name of custom metric
     * @param collector_func Function to collect the metric
     */
    virtual void register_custom_metric(const std::string& metric_name,
                                        std::function<Real(NodeId)> collector_func) = 0;

    /**
     * @brief Get metric history for a node
     * @param node_id Node identifier
     * @param metric_type Type of metric
     * @return Historical values (newest first)
     */
    virtual std::vector<Real> get_metric_history(NodeId node_id, MetricType metric_type) const = 0;

    /**
     * @brief Clear metric history
     */
    virtual void clear_history() = 0;
};

// ============================================================================
// Scaling Policy Evaluator Interface
// ============================================================================

/**
 * @brief Interface for evaluating when to scale based on policy
 */
class IScalingPolicyEvaluator {
public:
    virtual ~IScalingPolicyEvaluator() = default;

    /**
     * @brief Initialize the policy evaluator
     * @param policy Scaling policy
     * @return Success or error code
     */
    virtual ScalingResult initialize(const ScalingPolicy& policy) = 0;

    /**
     * @brief Evaluate current metrics and decide if scaling is needed
     * @param cluster_metrics Current cluster metrics
     * @param node_metrics Individual node metrics
     * @return Scaling decision
     */
    virtual ScalingDecision evaluate(const ClusterMetrics& cluster_metrics,
                                     const std::vector<NodeMetrics>& node_metrics) = 0;

    /**
     * @brief Check if cooldown period is active
     * @param direction Which direction to check
     * @return True if in cooldown
     */
    virtual bool is_in_cooldown(ScalingDirection direction) const = 0;

    /**
     * @brief Record a scaling event (updates cooldown timers)
     * @param event Scaling event that occurred
     */
    virtual void record_scaling_event(const ScalingEvent& event) = 0;

    /**
     * @brief Update the scaling policy
     * @param policy New policy
     */
    virtual void update_policy(const ScalingPolicy& policy) = 0;

    /**
     * @brief Get current policy
     */
    virtual const ScalingPolicy& get_policy() const = 0;
};

// ============================================================================
// Kubernetes Client Interface
// ============================================================================

/**
 * @brief Interface for Kubernetes API operations
 */
class IKubernetesClient {
public:
    virtual ~IKubernetesClient() = default;

    /**
     * @brief Initialize Kubernetes client
     * @param config Auto-scaler configuration
     * @return Success or error code
     */
    virtual ScalingResult initialize(const AutoScalerConfig& config) = 0;

    /**
     * @brief Scale deployment to target replica count
     * @param target_replicas Desired number of replicas
     * @return Success or error code
     */
    virtual ScalingResult scale_deployment(UInt32 target_replicas) = 0;

    /**
     * @brief Get current replica count
     * @return Current number of replicas or 0 on error
     */
    virtual UInt32 get_current_replicas() const = 0;

    /**
     * @brief Update HPA (Horizontal Pod Autoscaler) configuration
     * @param min_replicas Minimum replicas
     * @param max_replicas Maximum replicas
     * @param target_cpu_utilization Target CPU percentage (0-100)
     * @return Success or error code
     */
    virtual ScalingResult update_hpa(UInt32 min_replicas,
                                     UInt32 max_replicas,
                                     UInt32 target_cpu_utilization) = 0;

    /**
     * @brief Create or update custom metrics for HPA
     * @param metric_name Metric name
     * @param target_value Target value for this metric
     * @return Success or error code
     */
    virtual ScalingResult register_custom_metric(const std::string& metric_name,
                                                  Real target_value) = 0;

    /**
     * @brief Get list of nodes in the deployment
     * @return List of node IDs
     */
    virtual std::vector<NodeId> get_deployment_nodes() const = 0;

    /**
     * @brief Check if Kubernetes cluster is reachable
     * @return True if connected
     */
    virtual bool is_connected() const = 0;

    /**
     * @brief Drain a node (prepare for removal)
     * @param node_id Node to drain
     * @return Success or error code
     */
    virtual ScalingResult drain_node(NodeId node_id) = 0;

    /**
     * @brief Remove a node from the cluster
     * @param node_id Node to remove
     * @return Success or error code
     */
    virtual ScalingResult remove_node(NodeId node_id) = 0;
};

// ============================================================================
// Prometheus Exporter Interface
// ============================================================================

/**
 * @brief Interface for exporting metrics to Prometheus
 */
class IPrometheusExporter {
public:
    virtual ~IPrometheusExporter() = default;

    /**
     * @brief Initialize Prometheus exporter
     * @param config Auto-scaler configuration
     * @return Success or error code
     */
    virtual ScalingResult initialize(const AutoScalerConfig& config) = 0;

    /**
     * @brief Export node metrics to Prometheus
     * @param metrics Node metrics to export
     * @return Success or error code
     */
    virtual ScalingResult export_node_metrics(const NodeMetrics& metrics) = 0;

    /**
     * @brief Export cluster metrics to Prometheus
     * @param metrics Cluster metrics to export
     * @return Success or error code
     */
    virtual ScalingResult export_cluster_metrics(const ClusterMetrics& metrics) = 0;

    /**
     * @brief Export custom metric
     * @param metric_name Metric name
     * @param value Metric value
     * @param labels Optional labels
     * @return Success or error code
     */
    virtual ScalingResult export_custom_metric(const std::string& metric_name,
                                               Real value,
                                               const std::unordered_map<std::string, std::string>& labels = {}) = 0;

    /**
     * @brief Push all pending metrics to Prometheus Pushgateway
     * @return Success or error code
     */
    virtual ScalingResult push_metrics() = 0;

    /**
     * @brief Check if Prometheus is reachable
     * @return True if connected
     */
    virtual bool is_connected() const = 0;
};

// ============================================================================
// Callback Types
// ============================================================================

/// Callback when scaling decision is made
using ScalingDecisionCallback = std::function<void(const ScalingDecision&)>;

/// Callback when scaling event starts
using ScalingStartedCallback = std::function<void(const ScalingEvent&)>;

/// Callback when scaling event completes
using ScalingCompletedCallback = std::function<void(const ScalingEvent&)>;

/// Callback when metrics are collected
using MetricsCollectedCallback = std::function<void(const ClusterMetrics&)>;

/// Callback when node is added
using NodeAddedCallback = std::function<void(NodeId)>;

/// Callback when node is removed
using NodeRemovedCallback = std::function<void(NodeId)>;

// ============================================================================
// Auto-Scaler Main Class
// ============================================================================

/**
 * @brief Main auto-scaler for Kubernetes-based distributed simulation
 *
 * Monitors cluster metrics and automatically scales compute resources based
 * on CPU, entity count, memory, and custom metrics. Integrates with Kubernetes
 * HPA and Prometheus for production deployments.
 *
 * Example usage:
 * @code
 * AutoScalerConfig config = AutoScalerConfig::production();
 * config.k8s_deployment_name = "jaguar-engine";
 * config.k8s_hpa_name = "jaguar-hpa";
 * config.prometheus_pushgateway_url = "http://prometheus:9091";
 *
 * auto scaler = create_auto_scaler(config);
 * scaler->initialize(partition_manager);
 *
 * // In main loop
 * while (running) {
 *     scaler->update();
 *     std::this_thread::sleep_for(std::chrono::seconds(1));
 * }
 * @endcode
 */
class AutoScaler {
public:
    AutoScaler();
    ~AutoScaler();

    // Non-copyable, movable
    AutoScaler(const AutoScaler&) = delete;
    AutoScaler& operator=(const AutoScaler&) = delete;
    AutoScaler(AutoScaler&&) noexcept;
    AutoScaler& operator=(AutoScaler&&) noexcept;

    // ========================================================================
    // Lifecycle
    // ========================================================================

    /**
     * @brief Initialize auto-scaler
     * @param config Auto-scaler configuration
     * @param partition_manager Optional PartitionManager for entity migration
     * @return Success or error code
     */
    ScalingResult initialize(const AutoScalerConfig& config,
                            PartitionManager* partition_manager = nullptr);

    /**
     * @brief Shutdown and cleanup
     */
    void shutdown();

    /**
     * @brief Check if initialized
     */
    bool is_initialized() const noexcept;

    /**
     * @brief Get current configuration
     */
    const AutoScalerConfig& get_config() const noexcept;

    // ========================================================================
    // Main Update
    // ========================================================================

    /**
     * @brief Update auto-scaler (call periodically)
     *
     * Collects metrics, evaluates policy, and executes scaling decisions.
     * Should be called frequently (e.g., every second).
     */
    void update();

    // ========================================================================
    // Metrics Collection
    // ========================================================================

    /**
     * @brief Collect current cluster metrics
     * @return Current cluster metrics or nullopt on failure
     */
    std::optional<ClusterMetrics> collect_metrics();

    /**
     * @brief Get latest cluster metrics (cached)
     */
    const ClusterMetrics& get_current_metrics() const noexcept;

    /**
     * @brief Get metrics for specific node
     */
    std::optional<NodeMetrics> get_node_metrics(NodeId node_id) const;

    /**
     * @brief Register custom metric collector
     */
    void register_custom_metric(const std::string& metric_name,
                               std::function<Real(NodeId)> collector_func);

    // ========================================================================
    // Scaling Operations
    // ========================================================================

    /**
     * @brief Evaluate scaling policy and return decision
     * @return Scaling decision (may be ScalingDirection::None)
     */
    ScalingDecision evaluate_policy();

    /**
     * @brief Execute a scaling decision
     * @param decision Decision to execute
     * @return Success or error code
     */
    ScalingResult execute_scaling(const ScalingDecision& decision);

    /**
     * @brief Manually scale to target node count
     * @param target_nodes Desired number of nodes
     * @param reason Reason for manual scaling
     * @return Success or error code
     */
    ScalingResult scale_to(UInt32 target_nodes, const std::string& reason = "Manual");

    /**
     * @brief Scale up by adding nodes
     * @param count Number of nodes to add
     * @return Success or error code
     */
    ScalingResult scale_up(UInt32 count = 1);

    /**
     * @brief Scale down by removing nodes
     * @param count Number of nodes to remove
     * @return Success or error code
     */
    ScalingResult scale_down(UInt32 count = 1);

    /**
     * @brief Check if currently scaling
     */
    bool is_scaling() const noexcept;

    /**
     * @brief Check if in cooldown period
     */
    bool is_in_cooldown(ScalingDirection direction) const;

    // ========================================================================
    // Policy Management
    // ========================================================================

    /**
     * @brief Update scaling policy
     */
    void update_policy(const ScalingPolicy& policy);

    /**
     * @brief Get current scaling policy
     */
    const ScalingPolicy& get_policy() const noexcept;

    /**
     * @brief Enable/disable auto-scaling
     */
    void set_enabled(bool enabled);

    /**
     * @brief Check if auto-scaling is enabled
     */
    bool is_enabled() const noexcept;

    // ========================================================================
    // Statistics & History
    // ========================================================================

    /**
     * @brief Get auto-scaler statistics
     */
    AutoScalerStats get_stats() const;

    /**
     * @brief Reset statistics
     */
    void reset_stats();

    /**
     * @brief Get scaling history
     * @param max_events Maximum number of events to return (0 = all)
     * @return Recent scaling events (newest first)
     */
    std::vector<ScalingEvent> get_scaling_history(UInt32 max_events = 0) const;

    /**
     * @brief Clear scaling history
     */
    void clear_history();

    // ========================================================================
    // Integration with PartitionManager
    // ========================================================================

    /**
     * @brief Set PartitionManager for entity migration during scaling
     */
    void set_partition_manager(PartitionManager* partition_manager);

    /**
     * @brief Get PartitionManager (if set)
     */
    PartitionManager* get_partition_manager() const noexcept;

    /**
     * @brief Migrate entities off a node (prepare for removal)
     * @param node_id Node to drain
     * @return Success or error code
     */
    ScalingResult migrate_entities_off_node(NodeId node_id);

    // ========================================================================
    // Callbacks
    // ========================================================================

    /**
     * @brief Set callback for scaling decisions
     */
    void set_decision_callback(ScalingDecisionCallback callback);

    /**
     * @brief Set callback for scaling started
     */
    void set_scaling_started_callback(ScalingStartedCallback callback);

    /**
     * @brief Set callback for scaling completed
     */
    void set_scaling_completed_callback(ScalingCompletedCallback callback);

    /**
     * @brief Set callback for metrics collected
     */
    void set_metrics_callback(MetricsCollectedCallback callback);

    /**
     * @brief Set callback for node added
     */
    void set_node_added_callback(NodeAddedCallback callback);

    /**
     * @brief Set callback for node removed
     */
    void set_node_removed_callback(NodeRemovedCallback callback);

    // ========================================================================
    // Kubernetes Integration
    // ========================================================================

    /**
     * @brief Get current replica count from Kubernetes
     */
    UInt32 get_current_replicas() const;

    /**
     * @brief Check if Kubernetes is connected
     */
    bool is_kubernetes_connected() const;

    // ========================================================================
    // Prometheus Integration
    // ========================================================================

    /**
     * @brief Push current metrics to Prometheus
     * @return Success or error code
     */
    ScalingResult push_metrics_to_prometheus();

    /**
     * @brief Check if Prometheus is connected
     */
    bool is_prometheus_connected() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

// ============================================================================
// Factory Functions
// ============================================================================

/**
 * @brief Create an auto-scaler instance
 * @param config Auto-scaler configuration
 * @return Unique pointer to auto-scaler
 */
std::unique_ptr<AutoScaler> create_auto_scaler(const AutoScalerConfig& config = AutoScalerConfig::default_config());

/**
 * @brief Create a metrics collector
 * @param config Auto-scaler configuration
 * @return Unique pointer to metrics collector
 */
std::unique_ptr<IMetricsCollector> create_metrics_collector(const AutoScalerConfig& config);

/**
 * @brief Create a scaling policy evaluator
 * @param policy Scaling policy
 * @return Unique pointer to policy evaluator
 */
std::unique_ptr<IScalingPolicyEvaluator> create_policy_evaluator(const ScalingPolicy& policy);

/**
 * @brief Create a Kubernetes client
 * @param config Auto-scaler configuration
 * @return Unique pointer to Kubernetes client
 */
std::unique_ptr<IKubernetesClient> create_kubernetes_client(const AutoScalerConfig& config);

/**
 * @brief Create a Prometheus exporter
 * @param config Auto-scaler configuration
 * @return Unique pointer to Prometheus exporter
 */
std::unique_ptr<IPrometheusExporter> create_prometheus_exporter(const AutoScalerConfig& config);

} // namespace jaguar::cloud
