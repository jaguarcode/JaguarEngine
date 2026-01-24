/**
 * @file auto_scaler.cpp
 * @brief Implementation of Kubernetes-based auto-scaling system
 */

#include "jaguar/cloud/auto_scaler.h"
#include <algorithm>
#include <cmath>
#include <mutex>
#include <deque>

namespace jaguar::cloud {

// ============================================================================
// Hash Function for std::pair
// ============================================================================

struct PairHash {
    template <typename T1, typename T2>
    std::size_t operator()(const std::pair<T1, T2>& p) const {
        auto h1 = std::hash<T1>{}(p.first);
        auto h2 = std::hash<T2>{}(p.second);
        return h1 ^ (h2 << 1);
    }
};

// ============================================================================
// SimpleMetricsCollector Implementation
// ============================================================================

class SimpleMetricsCollector : public IMetricsCollector {
public:
    SimpleMetricsCollector() = default;
    ~SimpleMetricsCollector() override = default;

    ScalingResult initialize(const AutoScalerConfig& config) override {
        config_ = config;
        history_size_ = config.metric_history_size;
        return ScalingResult::Success;
    }

    std::optional<NodeMetrics> collect_node_metrics(NodeId node_id) override {
        std::lock_guard<std::mutex> lock(mutex_);

        auto it = node_metrics_.find(node_id);
        if (it == node_metrics_.end()) {
            return std::nullopt;
        }

        return it->second;
    }

    std::vector<NodeMetrics> collect_all_metrics() override {
        std::lock_guard<std::mutex> lock(mutex_);

        std::vector<NodeMetrics> result;
        result.reserve(node_metrics_.size());

        for (auto& [node_id, metrics] : node_metrics_) {
            // Update timestamp
            metrics.timestamp = std::chrono::steady_clock::now();
            metrics.last_updated = metrics.timestamp;
            result.push_back(metrics);
        }

        return result;
    }

    ClusterMetrics aggregate_metrics(const std::vector<NodeMetrics>& node_metrics) override {
        ClusterMetrics cluster;
        cluster.timestamp = std::chrono::steady_clock::now();

        if (node_metrics.empty()) {
            return cluster;
        }

        cluster.total_nodes = static_cast<UInt32>(node_metrics.size());
        cluster.active_nodes = 0;

        Real sum_cpu = 0.0;
        Real sum_memory = 0.0;
        UInt64 sum_entities = 0;
        Real sum_bandwidth = 0.0;
        Real sum_latency = 0.0;
        UInt32 latency_count = 0;

        cluster.max_cpu_usage = 0.0;
        cluster.min_cpu_usage = 1.0;
        cluster.max_memory_usage = 0.0;
        cluster.max_entities_per_node = 0;
        cluster.min_entities_per_node = UINT32_MAX;

        for (const auto& node : node_metrics) {
            if (node.is_healthy) {
                cluster.active_nodes++;

                sum_cpu += node.cpu_usage;
                sum_memory += node.memory_usage;
                sum_entities += node.entity_count;
                sum_bandwidth += node.network_rx_bytes_per_sec + node.network_tx_bytes_per_sec;

                cluster.max_cpu_usage = std::max(cluster.max_cpu_usage, node.cpu_usage);
                cluster.min_cpu_usage = std::min(cluster.min_cpu_usage, node.cpu_usage);
                cluster.max_memory_usage = std::max(cluster.max_memory_usage, node.memory_usage);
                cluster.max_entities_per_node = std::max(cluster.max_entities_per_node, node.entity_count);
                cluster.min_entities_per_node = std::min(cluster.min_entities_per_node, node.entity_count);

                if (node.avg_frame_time_ms > 0.0) {
                    sum_latency += node.avg_frame_time_ms;
                    latency_count++;
                }
            }
        }

        if (cluster.active_nodes > 0) {
            cluster.avg_cpu_usage = sum_cpu / cluster.active_nodes;
            cluster.avg_memory_usage = sum_memory / cluster.active_nodes;
            cluster.avg_entities_per_node = static_cast<UInt32>(sum_entities / cluster.active_nodes);
            cluster.avg_latency_ms = latency_count > 0 ? sum_latency / latency_count : 0.0;
        }

        cluster.total_entities = sum_entities;
        cluster.total_network_bandwidth = sum_bandwidth;

        // Calculate load imbalance
        if (cluster.max_cpu_usage > 0.0) {
            cluster.load_imbalance = (cluster.max_cpu_usage - cluster.min_cpu_usage) / cluster.max_cpu_usage;
        }

        return cluster;
    }

    void register_custom_metric(const std::string& metric_name,
                                std::function<Real(NodeId)> collector_func) override {
        std::lock_guard<std::mutex> lock(mutex_);
        custom_metrics_[metric_name] = std::move(collector_func);
    }

    std::vector<Real> get_metric_history(NodeId node_id, MetricType metric_type) const override {
        std::lock_guard<std::mutex> lock(mutex_);

        auto key = std::make_pair(node_id, metric_type);
        auto it = metric_history_.find(key);

        if (it == metric_history_.end()) {
            return {};
        }

        return std::vector<Real>(it->second.begin(), it->second.end());
    }

    void clear_history() override {
        std::lock_guard<std::mutex> lock(mutex_);
        metric_history_.clear();
    }

    // Helper method to update node metrics (not in interface, but needed)
    void update_node_metrics(NodeId node_id, const NodeMetrics& metrics) {
        std::lock_guard<std::mutex> lock(mutex_);
        node_metrics_[node_id] = metrics;

        // Store in history
        store_metric_history(node_id, MetricType::CPU, metrics.cpu_usage);
        store_metric_history(node_id, MetricType::Memory, metrics.memory_usage);
        store_metric_history(node_id, MetricType::EntityCount, static_cast<Real>(metrics.entity_count));
    }

private:
    void store_metric_history(NodeId node_id, MetricType metric_type, Real value) {
        auto key = std::make_pair(node_id, metric_type);
        auto& history = metric_history_[key];

        history.push_back(value);
        if (history.size() > history_size_) {
            history.pop_front();
        }
    }

    AutoScalerConfig config_;
    UInt32 history_size_{100};
    std::unordered_map<NodeId, NodeMetrics> node_metrics_;
    std::unordered_map<std::string, std::function<Real(NodeId)>> custom_metrics_;
    std::unordered_map<std::pair<NodeId, MetricType>, std::deque<Real>, PairHash> metric_history_;
    mutable std::mutex mutex_;
};

// ============================================================================
// SimplePolicyEvaluator Implementation
// ============================================================================

class SimplePolicyEvaluator : public IScalingPolicyEvaluator {
public:
    SimplePolicyEvaluator() = default;
    ~SimplePolicyEvaluator() override = default;

    ScalingResult initialize(const ScalingPolicy& policy) override {
        policy_ = policy;
        last_scale_up_time_ = std::chrono::steady_clock::time_point{};
        last_scale_down_time_ = std::chrono::steady_clock::time_point{};
        consecutive_scale_up_triggers_ = 0;
        consecutive_scale_down_triggers_ = 0;
        return ScalingResult::Success;
    }

    ScalingDecision evaluate(const ClusterMetrics& cluster_metrics,
                            const std::vector<NodeMetrics>& node_metrics) override {
        std::lock_guard<std::mutex> lock(mutex_);

        ScalingDecision decision;
        decision.timestamp = std::chrono::steady_clock::now();
        decision.is_valid = false;

        if (node_metrics.empty()) {
            return decision;
        }

        // Check cooldown
        if (is_in_cooldown(ScalingDirection::Up) && is_in_cooldown(ScalingDirection::Down)) {
            decision.reason = "In cooldown period";
            return decision;
        }

        // Evaluate scale-up triggers
        bool should_scale_up = false;
        ScalingTrigger primary_trigger = ScalingTrigger::CPU;
        Real trigger_value = 0.0;
        Real threshold = 0.0;

        // Check CPU
        if (policy_.cpu_scaling_enabled && cluster_metrics.avg_cpu_usage > policy_.cpu_scale_up_threshold) {
            should_scale_up = true;
            primary_trigger = ScalingTrigger::CPU;
            trigger_value = cluster_metrics.avg_cpu_usage;
            threshold = policy_.cpu_scale_up_threshold;
        }

        // Check entity count
        if (policy_.entity_scaling_enabled &&
            cluster_metrics.avg_entities_per_node > policy_.entity_scale_up_threshold) {
            should_scale_up = true;
            if (primary_trigger != ScalingTrigger::CPU) {
                primary_trigger = ScalingTrigger::EntityCount;
                trigger_value = static_cast<Real>(cluster_metrics.avg_entities_per_node);
                threshold = static_cast<Real>(policy_.entity_scale_up_threshold);
            } else {
                decision.secondary_triggers.push_back(ScalingTrigger::EntityCount);
            }
        }

        // Check memory
        if (policy_.memory_scaling_enabled &&
            cluster_metrics.avg_memory_usage > policy_.memory_scale_up_threshold) {
            should_scale_up = true;
            if (primary_trigger != ScalingTrigger::CPU && primary_trigger != ScalingTrigger::EntityCount) {
                primary_trigger = ScalingTrigger::Memory;
                trigger_value = cluster_metrics.avg_memory_usage;
                threshold = policy_.memory_scale_up_threshold;
            } else {
                decision.secondary_triggers.push_back(ScalingTrigger::Memory);
            }
        }

        // Evaluate scale-down triggers
        bool should_scale_down = false;
        if (!should_scale_up) {
            // Only consider scale-down if not scaling up
            if (policy_.cpu_scaling_enabled && cluster_metrics.avg_cpu_usage < policy_.cpu_scale_down_threshold &&
                policy_.entity_scaling_enabled &&
                cluster_metrics.avg_entities_per_node < policy_.entity_scale_down_threshold) {
                should_scale_down = true;
                primary_trigger = ScalingTrigger::CPU;
                trigger_value = cluster_metrics.avg_cpu_usage;
                threshold = policy_.cpu_scale_down_threshold;
            }
        }

        // Update consecutive trigger counters
        if (should_scale_up) {
            consecutive_scale_up_triggers_++;
            consecutive_scale_down_triggers_ = 0;
        } else if (should_scale_down) {
            consecutive_scale_down_triggers_++;
            consecutive_scale_up_triggers_ = 0;
        } else {
            consecutive_scale_up_triggers_ = 0;
            consecutive_scale_down_triggers_ = 0;
        }

        // Check if we have enough consecutive triggers
        if (should_scale_up && consecutive_scale_up_triggers_ >= policy_.consecutive_triggers_required) {
            if (!is_in_cooldown(ScalingDirection::Up)) {
                decision.direction = ScalingDirection::Up;
                decision.delta_nodes = policy_.scale_up_increment;
                decision.target_node_count = cluster_metrics.total_nodes + decision.delta_nodes;

                // Apply max nodes constraint
                if (decision.target_node_count > policy_.max_nodes) {
                    decision.target_node_count = policy_.max_nodes;
                    decision.delta_nodes = policy_.max_nodes - cluster_metrics.total_nodes;

                    if (decision.delta_nodes == 0) {
                        decision.direction = ScalingDirection::None;
                        decision.reason = "Max nodes reached";
                        return decision;
                    }
                }

                // Apply max scale-up rate
                UInt32 max_delta = static_cast<UInt32>(
                    std::ceil(cluster_metrics.total_nodes * policy_.max_scale_up_rate));
                if (decision.delta_nodes > max_delta) {
                    decision.delta_nodes = max_delta;
                    decision.target_node_count = cluster_metrics.total_nodes + decision.delta_nodes;
                }

                decision.primary_trigger = primary_trigger;
                decision.trigger_value = trigger_value;
                decision.threshold = threshold;
                decision.is_valid = true;
                decision.reason = "Scale up triggered by " + std::string(scaling_trigger_to_string(primary_trigger));
            } else {
                decision.reason = "Scale-up cooldown active";
            }
        } else if (should_scale_down && consecutive_scale_down_triggers_ >= policy_.consecutive_triggers_required) {
            if (!is_in_cooldown(ScalingDirection::Down)) {
                decision.direction = ScalingDirection::Down;
                decision.delta_nodes = policy_.scale_down_decrement;
                decision.target_node_count = cluster_metrics.total_nodes - decision.delta_nodes;

                // Apply min nodes constraint
                if (decision.target_node_count < policy_.min_nodes) {
                    decision.target_node_count = policy_.min_nodes;
                    decision.delta_nodes = cluster_metrics.total_nodes - policy_.min_nodes;

                    if (decision.delta_nodes == 0) {
                        decision.direction = ScalingDirection::None;
                        decision.reason = "Min nodes reached";
                        return decision;
                    }
                }

                // Apply max scale-down rate
                UInt32 max_delta = static_cast<UInt32>(
                    std::ceil(cluster_metrics.total_nodes * policy_.max_scale_down_rate));
                if (decision.delta_nodes > max_delta) {
                    decision.delta_nodes = max_delta;
                    decision.target_node_count = cluster_metrics.total_nodes - decision.delta_nodes;
                }

                decision.primary_trigger = primary_trigger;
                decision.trigger_value = trigger_value;
                decision.threshold = threshold;
                decision.is_valid = true;
                decision.reason = "Scale down triggered by " + std::string(scaling_trigger_to_string(primary_trigger));
            } else {
                decision.reason = "Scale-down cooldown active";
            }
        }

        return decision;
    }

    bool is_in_cooldown(ScalingDirection direction) const override {
        std::lock_guard<std::mutex> lock(mutex_);

        auto now = std::chrono::steady_clock::now();

        if (direction == ScalingDirection::Up) {
            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - last_scale_up_time_);
            return elapsed < policy_.scale_up_cooldown;
        } else if (direction == ScalingDirection::Down) {
            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - last_scale_down_time_);
            return elapsed < policy_.scale_down_cooldown;
        }

        return false;
    }

    void record_scaling_event(const ScalingEvent& event) override {
        std::lock_guard<std::mutex> lock(mutex_);

        auto now = std::chrono::steady_clock::now();

        if (event.direction == ScalingDirection::Up) {
            last_scale_up_time_ = now;
            consecutive_scale_up_triggers_ = 0;
        } else if (event.direction == ScalingDirection::Down) {
            last_scale_down_time_ = now;
            consecutive_scale_down_triggers_ = 0;
        }
    }

    void update_policy(const ScalingPolicy& policy) override {
        std::lock_guard<std::mutex> lock(mutex_);
        policy_ = policy;
    }

    const ScalingPolicy& get_policy() const override {
        std::lock_guard<std::mutex> lock(mutex_);
        return policy_;
    }

private:
    ScalingPolicy policy_;
    std::chrono::steady_clock::time_point last_scale_up_time_;
    std::chrono::steady_clock::time_point last_scale_down_time_;
    UInt32 consecutive_scale_up_triggers_{0};
    UInt32 consecutive_scale_down_triggers_{0};
    mutable std::mutex mutex_;
};

// ============================================================================
// SimpleKubernetesClient Implementation
// ============================================================================

class SimpleKubernetesClient : public IKubernetesClient {
public:
    SimpleKubernetesClient() = default;
    ~SimpleKubernetesClient() override = default;

    ScalingResult initialize(const AutoScalerConfig& config) override {
        config_ = config;
        current_replicas_ = 1;
        connected_ = true;
        return ScalingResult::Success;
    }

    ScalingResult scale_deployment(UInt32 target_replicas) override {
        std::lock_guard<std::mutex> lock(mutex_);

        // Stub implementation - actual K8s calls would need libcurl or K8s client
        if (!connected_) {
            return ScalingResult::K8sConnectionFailed;
        }

        current_replicas_ = target_replicas;
        return ScalingResult::Success;
    }

    UInt32 get_current_replicas() const override {
        std::lock_guard<std::mutex> lock(mutex_);
        return current_replicas_;
    }

    ScalingResult update_hpa(UInt32 min_replicas,
                            UInt32 max_replicas,
                            UInt32 target_cpu_utilization) override {
        (void)min_replicas;
        (void)max_replicas;
        (void)target_cpu_utilization;

        std::lock_guard<std::mutex> lock(mutex_);

        if (!connected_) {
            return ScalingResult::K8sConnectionFailed;
        }

        // Stub implementation
        return ScalingResult::Success;
    }

    ScalingResult register_custom_metric(const std::string& metric_name,
                                        Real target_value) override {
        std::lock_guard<std::mutex> lock(mutex_);

        if (!connected_) {
            return ScalingResult::K8sConnectionFailed;
        }

        custom_metrics_[metric_name] = target_value;
        return ScalingResult::Success;
    }

    std::vector<NodeId> get_deployment_nodes() const override {
        std::lock_guard<std::mutex> lock(mutex_);
        return nodes_;
    }

    bool is_connected() const override {
        std::lock_guard<std::mutex> lock(mutex_);
        return connected_;
    }

    ScalingResult drain_node(NodeId node_id) override {
        (void)node_id;

        std::lock_guard<std::mutex> lock(mutex_);

        if (!connected_) {
            return ScalingResult::K8sConnectionFailed;
        }

        // Stub implementation
        return ScalingResult::Success;
    }

    ScalingResult remove_node(NodeId node_id) override {
        std::lock_guard<std::mutex> lock(mutex_);

        if (!connected_) {
            return ScalingResult::K8sConnectionFailed;
        }

        auto it = std::find(nodes_.begin(), nodes_.end(), node_id);
        if (it != nodes_.end()) {
            nodes_.erase(it);
        }

        return ScalingResult::Success;
    }

    // Helper to add node (not in interface)
    void add_node(NodeId node_id) {
        std::lock_guard<std::mutex> lock(mutex_);
        nodes_.push_back(node_id);
    }

private:
    AutoScalerConfig config_;
    UInt32 current_replicas_{1};
    bool connected_{false};
    std::vector<NodeId> nodes_;
    std::unordered_map<std::string, Real> custom_metrics_;
    mutable std::mutex mutex_;
};

// ============================================================================
// SimplePrometheusExporter Implementation
// ============================================================================

class SimplePrometheusExporter : public IPrometheusExporter {
public:
    SimplePrometheusExporter() = default;
    ~SimplePrometheusExporter() override = default;

    ScalingResult initialize(const AutoScalerConfig& config) override {
        config_ = config;
        connected_ = !config.prometheus_pushgateway_url.empty();
        return ScalingResult::Success;
    }

    ScalingResult export_node_metrics(const NodeMetrics& metrics) override {
        std::lock_guard<std::mutex> lock(mutex_);

        if (!connected_) {
            return ScalingResult::PrometheusConnectionFailed;
        }

        // Stub implementation - actual pushes would need HTTP client
        pending_node_metrics_.push_back(metrics);
        return ScalingResult::Success;
    }

    ScalingResult export_cluster_metrics(const ClusterMetrics& metrics) override {
        std::lock_guard<std::mutex> lock(mutex_);

        if (!connected_) {
            return ScalingResult::PrometheusConnectionFailed;
        }

        // Stub implementation
        cluster_metrics_ = metrics;
        return ScalingResult::Success;
    }

    ScalingResult export_custom_metric(const std::string& metric_name,
                                      Real value,
                                      const std::unordered_map<std::string, std::string>& labels) override {
        (void)labels;

        std::lock_guard<std::mutex> lock(mutex_);

        if (!connected_) {
            return ScalingResult::PrometheusConnectionFailed;
        }

        custom_metrics_[metric_name] = value;
        return ScalingResult::Success;
    }

    ScalingResult push_metrics() override {
        std::lock_guard<std::mutex> lock(mutex_);

        if (!connected_) {
            return ScalingResult::PrometheusConnectionFailed;
        }

        // Stub implementation - would actually POST to pushgateway
        push_count_++;
        pending_node_metrics_.clear();

        return ScalingResult::Success;
    }

    bool is_connected() const override {
        std::lock_guard<std::mutex> lock(mutex_);
        return connected_;
    }

private:
    AutoScalerConfig config_;
    bool connected_{false};
    std::vector<NodeMetrics> pending_node_metrics_;
    ClusterMetrics cluster_metrics_;
    std::unordered_map<std::string, Real> custom_metrics_;
    UInt64 push_count_{0};
    mutable std::mutex mutex_;
};

// ============================================================================
// AutoScaler Implementation
// ============================================================================

struct AutoScaler::Impl {
    AutoScalerConfig config;
    bool initialized{false};
    bool enabled{true};
    bool is_scaling{false};

    // Components
    std::unique_ptr<SimpleMetricsCollector> metrics_collector;
    std::unique_ptr<IScalingPolicyEvaluator> policy_evaluator;
    std::unique_ptr<IKubernetesClient> k8s_client;
    std::unique_ptr<IPrometheusExporter> prometheus_exporter;

    // Integration
    PartitionManager* partition_manager{nullptr};

    // State
    ClusterMetrics current_metrics;
    std::deque<ScalingEvent> scaling_history;
    AutoScalerStats stats;
    UInt64 next_event_id{1};

    // Timing
    std::chrono::steady_clock::time_point last_metrics_collection;
    std::chrono::steady_clock::time_point last_prometheus_push;

    // Callbacks
    ScalingDecisionCallback on_decision;
    ScalingStartedCallback on_scaling_started;
    ScalingCompletedCallback on_scaling_completed;
    MetricsCollectedCallback on_metrics_collected;
    NodeAddedCallback on_node_added;
    NodeRemovedCallback on_node_removed;

    // Thread safety
    mutable std::mutex mutex;
};

AutoScaler::AutoScaler()
    : impl_(std::make_unique<Impl>()) {}

AutoScaler::~AutoScaler() = default;

AutoScaler::AutoScaler(AutoScaler&&) noexcept = default;
AutoScaler& AutoScaler::operator=(AutoScaler&&) noexcept = default;

ScalingResult AutoScaler::initialize(const AutoScalerConfig& config,
                                    PartitionManager* partition_manager) {
    std::lock_guard<std::mutex> lock(impl_->mutex);

    if (impl_->initialized) {
        return ScalingResult::AlreadyInitialized;
    }

    impl_->config = config;
    impl_->enabled = config.enabled;
    impl_->partition_manager = partition_manager;

    // Create components
    impl_->metrics_collector = std::make_unique<SimpleMetricsCollector>();
    auto result = impl_->metrics_collector->initialize(config);
    if (result != ScalingResult::Success) {
        return result;
    }

    impl_->policy_evaluator = create_policy_evaluator(config.policy);
    result = impl_->policy_evaluator->initialize(config.policy);
    if (result != ScalingResult::Success) {
        return result;
    }

    impl_->k8s_client = create_kubernetes_client(config);
    result = impl_->k8s_client->initialize(config);
    if (result != ScalingResult::Success) {
        return result;
    }

    if (config.export_to_prometheus) {
        impl_->prometheus_exporter = create_prometheus_exporter(config);
        result = impl_->prometheus_exporter->initialize(config);
        if (result != ScalingResult::Success) {
            return result;
        }
    }

    impl_->stats.start_time = std::chrono::steady_clock::now();
    impl_->last_metrics_collection = std::chrono::steady_clock::now();
    impl_->last_prometheus_push = std::chrono::steady_clock::now();

    impl_->initialized = true;
    return ScalingResult::Success;
}

void AutoScaler::shutdown() {
    std::lock_guard<std::mutex> lock(impl_->mutex);

    impl_->metrics_collector.reset();
    impl_->policy_evaluator.reset();
    impl_->k8s_client.reset();
    impl_->prometheus_exporter.reset();

    impl_->scaling_history.clear();
    impl_->initialized = false;
}

bool AutoScaler::is_initialized() const noexcept {
    return impl_->initialized;
}

const AutoScalerConfig& AutoScaler::get_config() const noexcept {
    return impl_->config;
}

void AutoScaler::update() {
    std::lock_guard<std::mutex> lock(impl_->mutex);

    if (!impl_->initialized || !impl_->enabled) {
        return;
    }

    auto now = std::chrono::steady_clock::now();

    // Collect metrics at configured interval
    auto metrics_elapsed = std::chrono::duration_cast<std::chrono::seconds>(
        now - impl_->last_metrics_collection);

    if (metrics_elapsed >= impl_->config.metric_collection_interval) {
        auto node_metrics = impl_->metrics_collector->collect_all_metrics();
        impl_->current_metrics = impl_->metrics_collector->aggregate_metrics(node_metrics);
        impl_->last_metrics_collection = now;

        impl_->stats.metrics_collected++;
        impl_->stats.current_node_count = impl_->current_metrics.total_nodes;

        if (impl_->on_metrics_collected) {
            impl_->on_metrics_collected(impl_->current_metrics);
        }
    }

    // Evaluate policy at configured interval
    auto policy_elapsed = std::chrono::duration_cast<std::chrono::seconds>(
        now - impl_->last_metrics_collection);

    if (policy_elapsed >= impl_->config.policy.evaluation_interval) {
        auto node_metrics = impl_->metrics_collector->collect_all_metrics();
        auto decision = impl_->policy_evaluator->evaluate(impl_->current_metrics, node_metrics);

        if (decision.is_valid && decision.direction != ScalingDirection::None) {
            if (impl_->on_decision) {
                impl_->on_decision(decision);
            }

            // Execute scaling
            execute_scaling(decision);
        }
    }

    // Push to Prometheus at configured interval
    if (impl_->prometheus_exporter && impl_->config.export_to_prometheus) {
        auto prom_elapsed = std::chrono::duration_cast<std::chrono::seconds>(
            now - impl_->last_prometheus_push);

        if (prom_elapsed >= impl_->config.prometheus_push_interval) {
            auto result = impl_->prometheus_exporter->export_cluster_metrics(impl_->current_metrics);
            if (result == ScalingResult::Success) {
                impl_->prometheus_exporter->push_metrics();
                impl_->stats.prometheus_pushes++;
                impl_->last_prometheus_push = now;
            } else {
                impl_->stats.prometheus_failures++;
            }
        }
    }

    // Update uptime
    impl_->stats.uptime = std::chrono::duration_cast<std::chrono::seconds>(
        now - impl_->stats.start_time);
}

std::optional<ClusterMetrics> AutoScaler::collect_metrics() {
    std::lock_guard<std::mutex> lock(impl_->mutex);

    if (!impl_->initialized) {
        return std::nullopt;
    }

    auto node_metrics = impl_->metrics_collector->collect_all_metrics();
    impl_->current_metrics = impl_->metrics_collector->aggregate_metrics(node_metrics);

    return impl_->current_metrics;
}

const ClusterMetrics& AutoScaler::get_current_metrics() const noexcept {
    return impl_->current_metrics;
}

std::optional<NodeMetrics> AutoScaler::get_node_metrics(NodeId node_id) const {
    std::lock_guard<std::mutex> lock(impl_->mutex);

    if (!impl_->initialized) {
        return std::nullopt;
    }

    return impl_->metrics_collector->collect_node_metrics(node_id);
}

void AutoScaler::register_custom_metric(const std::string& metric_name,
                                       std::function<Real(NodeId)> collector_func) {
    std::lock_guard<std::mutex> lock(impl_->mutex);

    if (impl_->metrics_collector) {
        impl_->metrics_collector->register_custom_metric(metric_name, std::move(collector_func));
    }
}

ScalingDecision AutoScaler::evaluate_policy() {
    std::lock_guard<std::mutex> lock(impl_->mutex);

    if (!impl_->initialized || !impl_->enabled) {
        ScalingDecision decision;
        decision.is_valid = false;
        decision.reason = "Not initialized or disabled";
        return decision;
    }

    auto node_metrics = impl_->metrics_collector->collect_all_metrics();
    return impl_->policy_evaluator->evaluate(impl_->current_metrics, node_metrics);
}

ScalingResult AutoScaler::execute_scaling(const ScalingDecision& decision) {
    // Note: mutex already locked by caller

    if (!decision.is_valid || decision.direction == ScalingDirection::None) {
        return ScalingResult::NoTriggerMet;
    }

    if (impl_->is_scaling) {
        return ScalingResult::AlreadyScaling;
    }

    impl_->is_scaling = true;

    // Create scaling event
    ScalingEvent event;
    event.event_id = impl_->next_event_id++;
    event.direction = decision.direction;
    event.nodes_before = impl_->current_metrics.total_nodes;
    event.nodes_after = decision.target_node_count;
    event.trigger = decision.primary_trigger;
    event.reason = decision.reason;
    event.started_at = std::chrono::steady_clock::now();

    if (impl_->on_scaling_started) {
        impl_->on_scaling_started(event);
    }

    // Execute scaling via Kubernetes
    auto result = impl_->k8s_client->scale_deployment(decision.target_node_count);

    event.completed_at = std::chrono::steady_clock::now();
    event.duration = std::chrono::duration_cast<std::chrono::milliseconds>(
        event.completed_at - event.started_at);
    event.result = result;
    event.successful = (result == ScalingResult::Success);

    // Update statistics
    if (decision.direction == ScalingDirection::Up) {
        impl_->stats.total_scale_ups++;
        if (result == ScalingResult::Success) {
            impl_->stats.successful_scale_ups++;
            impl_->stats.last_scale_up_time = event.completed_at;

            if (decision.trigger_value > 0.0) {
                if (decision.primary_trigger == ScalingTrigger::CPU) {
                    impl_->stats.cpu_triggered_scale_ups++;
                } else if (decision.primary_trigger == ScalingTrigger::EntityCount) {
                    impl_->stats.entity_triggered_scale_ups++;
                } else if (decision.primary_trigger == ScalingTrigger::Memory) {
                    impl_->stats.memory_triggered_scale_ups++;
                }
            }

            // Update duration stats
            if (impl_->stats.avg_scale_up_duration.count() == 0) {
                impl_->stats.avg_scale_up_duration = event.duration;
            } else {
                impl_->stats.avg_scale_up_duration = std::chrono::milliseconds(
                    (impl_->stats.avg_scale_up_duration.count() + event.duration.count()) / 2);
            }
            impl_->stats.max_scale_up_duration = std::max(impl_->stats.max_scale_up_duration, event.duration);
        } else {
            impl_->stats.failed_scale_ups++;
        }
    } else if (decision.direction == ScalingDirection::Down) {
        impl_->stats.total_scale_downs++;
        if (result == ScalingResult::Success) {
            impl_->stats.successful_scale_downs++;
            impl_->stats.last_scale_down_time = event.completed_at;

            // Migrate entities off nodes before scale-down
            if (impl_->partition_manager && impl_->config.migrate_entities_on_scale) {
                // Would trigger entity migration here
                // For now, just record in event
                event.entities_migrated = 0;
            }

            // Update duration stats
            if (impl_->stats.avg_scale_down_duration.count() == 0) {
                impl_->stats.avg_scale_down_duration = event.duration;
            } else {
                impl_->stats.avg_scale_down_duration = std::chrono::milliseconds(
                    (impl_->stats.avg_scale_down_duration.count() + event.duration.count()) / 2);
            }
            impl_->stats.max_scale_down_duration = std::max(impl_->stats.max_scale_down_duration, event.duration);
        } else {
            impl_->stats.failed_scale_downs++;
        }
    }

    // Update min/max node count
    impl_->stats.max_node_count_reached = std::max(impl_->stats.max_node_count_reached,
                                                     event.nodes_after);
    if (impl_->stats.min_node_count_reached == 0) {
        impl_->stats.min_node_count_reached = event.nodes_after;
    } else {
        impl_->stats.min_node_count_reached = std::min(impl_->stats.min_node_count_reached,
                                                         event.nodes_after);
    }

    // Record event in policy evaluator
    impl_->policy_evaluator->record_scaling_event(event);

    // Store in history
    if (impl_->config.record_scaling_history) {
        impl_->scaling_history.push_front(event);
        if (impl_->scaling_history.size() > impl_->config.max_history_events) {
            impl_->scaling_history.pop_back();
        }
    }

    if (impl_->on_scaling_completed) {
        impl_->on_scaling_completed(event);
    }

    impl_->is_scaling = false;
    return result;
}

ScalingResult AutoScaler::scale_to(UInt32 target_nodes, const std::string& reason) {
    std::lock_guard<std::mutex> lock(impl_->mutex);

    if (!impl_->initialized) {
        return ScalingResult::NotInitialized;
    }

    ScalingDecision decision;
    decision.target_node_count = target_nodes;
    decision.delta_nodes = (target_nodes > impl_->current_metrics.total_nodes)
        ? target_nodes - impl_->current_metrics.total_nodes
        : impl_->current_metrics.total_nodes - target_nodes;
    decision.direction = (target_nodes > impl_->current_metrics.total_nodes)
        ? ScalingDirection::Up
        : ScalingDirection::Down;
    decision.reason = reason;
    decision.is_valid = true;
    decision.timestamp = std::chrono::steady_clock::now();

    return execute_scaling(decision);
}

ScalingResult AutoScaler::scale_up(UInt32 count) {
    std::lock_guard<std::mutex> lock(impl_->mutex);

    UInt32 target = impl_->current_metrics.total_nodes + count;
    return scale_to(target, "Manual scale-up");
}

ScalingResult AutoScaler::scale_down(UInt32 count) {
    std::lock_guard<std::mutex> lock(impl_->mutex);

    if (count >= impl_->current_metrics.total_nodes) {
        return ScalingResult::MinNodesReached;
    }

    UInt32 target = impl_->current_metrics.total_nodes - count;
    return scale_to(target, "Manual scale-down");
}

bool AutoScaler::is_scaling() const noexcept {
    return impl_->is_scaling;
}

bool AutoScaler::is_in_cooldown(ScalingDirection direction) const {
    std::lock_guard<std::mutex> lock(impl_->mutex);

    if (!impl_->policy_evaluator) {
        return false;
    }

    return impl_->policy_evaluator->is_in_cooldown(direction);
}

void AutoScaler::update_policy(const ScalingPolicy& policy) {
    std::lock_guard<std::mutex> lock(impl_->mutex);

    if (impl_->policy_evaluator) {
        impl_->policy_evaluator->update_policy(policy);
        impl_->config.policy = policy;
    }
}

const ScalingPolicy& AutoScaler::get_policy() const noexcept {
    if (impl_->policy_evaluator) {
        return impl_->policy_evaluator->get_policy();
    }
    return impl_->config.policy;
}

void AutoScaler::set_enabled(bool enabled) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->enabled = enabled;
}

bool AutoScaler::is_enabled() const noexcept {
    return impl_->enabled;
}

AutoScalerStats AutoScaler::get_stats() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->stats;
}

void AutoScaler::reset_stats() {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->stats.reset();
}

std::vector<ScalingEvent> AutoScaler::get_scaling_history(UInt32 max_events) const {
    std::lock_guard<std::mutex> lock(impl_->mutex);

    std::vector<ScalingEvent> result;

    UInt32 count = (max_events == 0) ? static_cast<UInt32>(impl_->scaling_history.size())
                                     : std::min(max_events, static_cast<UInt32>(impl_->scaling_history.size()));

    result.reserve(count);

    auto it = impl_->scaling_history.begin();
    for (UInt32 i = 0; i < count && it != impl_->scaling_history.end(); ++i, ++it) {
        result.push_back(*it);
    }

    return result;
}

void AutoScaler::clear_history() {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->scaling_history.clear();
}

void AutoScaler::set_partition_manager(PartitionManager* partition_manager) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->partition_manager = partition_manager;
}

PartitionManager* AutoScaler::get_partition_manager() const noexcept {
    return impl_->partition_manager;
}

ScalingResult AutoScaler::migrate_entities_off_node(NodeId node_id) {
    std::lock_guard<std::mutex> lock(impl_->mutex);

    if (!impl_->partition_manager) {
        return ScalingResult::InvalidConfiguration;
    }

    // Get entities on this node
    auto entities = impl_->partition_manager->get_entities_on_node(node_id);

    // Trigger migrations to other nodes
    // This is a simplified implementation
    for (const auto& entity_id : entities) {
        (void)entity_id;
        // Find a different node to migrate to
        // In real implementation, would use load balancer to choose target
    }

    impl_->stats.total_entities_migrated += entities.size();

    return ScalingResult::Success;
}

void AutoScaler::set_decision_callback(ScalingDecisionCallback callback) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->on_decision = std::move(callback);
}

void AutoScaler::set_scaling_started_callback(ScalingStartedCallback callback) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->on_scaling_started = std::move(callback);
}

void AutoScaler::set_scaling_completed_callback(ScalingCompletedCallback callback) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->on_scaling_completed = std::move(callback);
}

void AutoScaler::set_metrics_callback(MetricsCollectedCallback callback) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->on_metrics_collected = std::move(callback);
}

void AutoScaler::set_node_added_callback(NodeAddedCallback callback) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->on_node_added = std::move(callback);
}

void AutoScaler::set_node_removed_callback(NodeRemovedCallback callback) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->on_node_removed = std::move(callback);
}

UInt32 AutoScaler::get_current_replicas() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);

    if (!impl_->k8s_client) {
        return 0;
    }

    return impl_->k8s_client->get_current_replicas();
}

bool AutoScaler::is_kubernetes_connected() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);

    if (!impl_->k8s_client) {
        return false;
    }

    return impl_->k8s_client->is_connected();
}

ScalingResult AutoScaler::push_metrics_to_prometheus() {
    std::lock_guard<std::mutex> lock(impl_->mutex);

    if (!impl_->prometheus_exporter) {
        return ScalingResult::InvalidConfiguration;
    }

    auto result = impl_->prometheus_exporter->export_cluster_metrics(impl_->current_metrics);
    if (result != ScalingResult::Success) {
        return result;
    }

    return impl_->prometheus_exporter->push_metrics();
}

bool AutoScaler::is_prometheus_connected() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);

    if (!impl_->prometheus_exporter) {
        return false;
    }

    return impl_->prometheus_exporter->is_connected();
}

// ============================================================================
// Factory Functions
// ============================================================================

std::unique_ptr<AutoScaler> create_auto_scaler(const AutoScalerConfig& config) {
    auto scaler = std::make_unique<AutoScaler>();
    scaler->initialize(config);
    return scaler;
}

std::unique_ptr<IMetricsCollector> create_metrics_collector(const AutoScalerConfig& config) {
    auto collector = std::make_unique<SimpleMetricsCollector>();
    collector->initialize(config);
    return collector;
}

std::unique_ptr<IScalingPolicyEvaluator> create_policy_evaluator(const ScalingPolicy& policy) {
    auto evaluator = std::make_unique<SimplePolicyEvaluator>();
    evaluator->initialize(policy);
    return evaluator;
}

std::unique_ptr<IKubernetesClient> create_kubernetes_client(const AutoScalerConfig& config) {
    auto client = std::make_unique<SimpleKubernetesClient>();
    client->initialize(config);
    return client;
}

std::unique_ptr<IPrometheusExporter> create_prometheus_exporter(const AutoScalerConfig& config) {
    auto exporter = std::make_unique<SimplePrometheusExporter>();
    exporter->initialize(config);
    return exporter;
}

} // namespace jaguar::cloud
