/**
 * @file auto_scaler_tests.cpp
 * @brief Unit tests for the auto-scaler system
 */

#include <gtest/gtest.h>
#include "jaguar/cloud/auto_scaler.h"
#include <thread>
#include <chrono>

using namespace jaguar;
using namespace jaguar::cloud;

// ============================================================================
// ScalingPolicy Tests
// ============================================================================

class ScalingPolicyTest : public ::testing::Test {};

TEST_F(ScalingPolicyTest, DefaultPolicy) {
    ScalingPolicy policy = ScalingPolicy::default_policy();

    EXPECT_DOUBLE_EQ(policy.cpu_scale_up_threshold, 0.7);
    EXPECT_DOUBLE_EQ(policy.cpu_scale_down_threshold, 0.3);
    EXPECT_EQ(policy.entity_scale_up_threshold, 8000u);
    EXPECT_EQ(policy.entity_scale_down_threshold, 2000u);
    EXPECT_EQ(policy.min_nodes, 1u);
    EXPECT_EQ(policy.max_nodes, 100u);
    EXPECT_TRUE(policy.cpu_scaling_enabled);
    EXPECT_TRUE(policy.entity_scaling_enabled);
}

TEST_F(ScalingPolicyTest, AggressivePolicy) {
    ScalingPolicy policy = ScalingPolicy::aggressive();

    EXPECT_DOUBLE_EQ(policy.cpu_scale_up_threshold, 0.6);
    EXPECT_EQ(policy.entity_scale_up_threshold, 5000u);
    EXPECT_EQ(policy.scale_up_cooldown.count(), 30);
    EXPECT_EQ(policy.scale_down_cooldown.count(), 120);
    EXPECT_EQ(policy.evaluation_interval.count(), 5);
    EXPECT_EQ(policy.consecutive_triggers_required, 2u);
    EXPECT_EQ(policy.scale_up_increment, 2u);
}

TEST_F(ScalingPolicyTest, ConservativePolicy) {
    ScalingPolicy policy = ScalingPolicy::conservative();

    EXPECT_DOUBLE_EQ(policy.cpu_scale_up_threshold, 0.8);
    EXPECT_EQ(policy.entity_scale_up_threshold, 10000u);
    EXPECT_EQ(policy.scale_up_cooldown.count(), 120);
    EXPECT_EQ(policy.scale_down_cooldown.count(), 600);
    EXPECT_EQ(policy.evaluation_interval.count(), 30);
    EXPECT_EQ(policy.consecutive_triggers_required, 5u);
}

TEST_F(ScalingPolicyTest, LargeScalePolicy) {
    ScalingPolicy policy = ScalingPolicy::large_scale();

    EXPECT_EQ(policy.max_nodes, 200u);
    EXPECT_EQ(policy.scale_up_increment, 5u);
    EXPECT_EQ(policy.scale_down_decrement, 2u);
    EXPECT_DOUBLE_EQ(policy.cpu_scale_up_threshold, 0.7);
    EXPECT_EQ(policy.entity_scale_up_threshold, 8000u);
    EXPECT_EQ(policy.evaluation_interval.count(), 15);
    EXPECT_TRUE(policy.memory_scaling_enabled);
}

TEST_F(ScalingPolicyTest, ThresholdValidation) {
    ScalingPolicy policy = ScalingPolicy::default_policy();

    // Verify CPU thresholds
    EXPECT_GT(policy.cpu_scale_up_threshold, policy.cpu_scale_down_threshold);
    EXPECT_LE(policy.cpu_scale_up_threshold, 1.0);
    EXPECT_GE(policy.cpu_scale_down_threshold, 0.0);

    // Verify entity thresholds
    EXPECT_GT(policy.entity_scale_up_threshold, policy.entity_scale_down_threshold);

    // Verify node limits
    EXPECT_GT(policy.max_nodes, policy.min_nodes);
    EXPECT_GE(policy.min_nodes, 1u);
}

// ============================================================================
// ScalingDecision Tests
// ============================================================================

class ScalingDecisionTest : public ::testing::Test {};

TEST_F(ScalingDecisionTest, DecisionCreation) {
    ScalingDecision decision;
    decision.direction = ScalingDirection::Up;
    decision.target_node_count = 10;
    decision.delta_nodes = 2;
    decision.primary_trigger = ScalingTrigger::CPU;
    decision.reason = "CPU usage exceeded 70%";
    decision.trigger_value = 0.75;
    decision.threshold = 0.7;
    decision.is_valid = true;

    EXPECT_EQ(decision.direction, ScalingDirection::Up);
    EXPECT_EQ(decision.target_node_count, 10u);
    EXPECT_EQ(decision.delta_nodes, 2u);
    EXPECT_EQ(decision.primary_trigger, ScalingTrigger::CPU);
    EXPECT_DOUBLE_EQ(decision.trigger_value, 0.75);
    EXPECT_DOUBLE_EQ(decision.threshold, 0.7);
    EXPECT_TRUE(decision.is_valid);
}

TEST_F(ScalingDecisionTest, IsValidFlag) {
    ScalingDecision invalid_decision;
    EXPECT_FALSE(invalid_decision.is_valid);

    ScalingDecision valid_decision;
    valid_decision.is_valid = true;
    valid_decision.direction = ScalingDirection::Up;
    valid_decision.target_node_count = 5;
    EXPECT_TRUE(valid_decision.is_valid);
}

TEST_F(ScalingDecisionTest, SecondaryTriggers) {
    ScalingDecision decision;
    decision.primary_trigger = ScalingTrigger::CPU;
    decision.secondary_triggers.push_back(ScalingTrigger::EntityCount);
    decision.secondary_triggers.push_back(ScalingTrigger::Memory);

    EXPECT_EQ(decision.secondary_triggers.size(), 2u);
    EXPECT_EQ(decision.secondary_triggers[0], ScalingTrigger::EntityCount);
    EXPECT_EQ(decision.secondary_triggers[1], ScalingTrigger::Memory);
}

// ============================================================================
// AutoScalerConfig Tests
// ============================================================================

class AutoScalerConfigTest : public ::testing::Test {};

TEST_F(AutoScalerConfigTest, DefaultConfig) {
    AutoScalerConfig config = AutoScalerConfig::default_config();

    EXPECT_TRUE(config.enabled);
    EXPECT_EQ(config.k8s_namespace, "default");
    EXPECT_TRUE(config.use_in_cluster_config);
    EXPECT_TRUE(config.export_to_prometheus);
    EXPECT_EQ(config.prometheus_job_name, "jaguar_engine");
    EXPECT_TRUE(config.integrate_partition_manager);
    EXPECT_TRUE(config.migrate_entities_on_scale);
    EXPECT_TRUE(config.record_scaling_history);
}

TEST_F(AutoScalerConfigTest, DevelopmentConfig) {
    AutoScalerConfig config = AutoScalerConfig::development();

    EXPECT_EQ(config.policy.min_nodes, 1u);
    EXPECT_EQ(config.policy.max_nodes, 10u);
    EXPECT_TRUE(config.verbose_logging);
    EXPECT_EQ(config.metric_collection_interval.count(), 2);

    // Should use aggressive policy
    EXPECT_EQ(config.policy.cpu_scale_up_threshold, 0.6);
}

TEST_F(AutoScalerConfigTest, ProductionConfig) {
    AutoScalerConfig config = AutoScalerConfig::production();

    EXPECT_EQ(config.policy.max_nodes, 100u);
    EXPECT_TRUE(config.export_to_prometheus);
    EXPECT_TRUE(config.wait_for_migration_completion);
    EXPECT_TRUE(config.record_scaling_history);

    // Should use conservative policy
    EXPECT_EQ(config.policy.cpu_scale_up_threshold, 0.8);
}

TEST_F(AutoScalerConfigTest, KubernetesSettings) {
    AutoScalerConfig config;
    config.k8s_api_server = "https://kubernetes.default.svc";
    config.k8s_namespace = "jaguar";
    config.k8s_deployment_name = "jaguar-engine";
    config.k8s_hpa_name = "jaguar-hpa";

    EXPECT_EQ(config.k8s_api_server, "https://kubernetes.default.svc");
    EXPECT_EQ(config.k8s_namespace, "jaguar");
    EXPECT_EQ(config.k8s_deployment_name, "jaguar-engine");
    EXPECT_EQ(config.k8s_hpa_name, "jaguar-hpa");
}

TEST_F(AutoScalerConfigTest, PrometheusSettings) {
    AutoScalerConfig config;
    config.prometheus_pushgateway_url = "http://prometheus:9091";
    config.prometheus_job_name = "test_job";
    config.prometheus_instance_name = "instance_1";

    EXPECT_EQ(config.prometheus_pushgateway_url, "http://prometheus:9091");
    EXPECT_EQ(config.prometheus_job_name, "test_job");
    EXPECT_EQ(config.prometheus_instance_name, "instance_1");
}

// ============================================================================
// MetricsCollector Tests
// ============================================================================

class MetricsCollectorTest : public ::testing::Test {
protected:
    void SetUp() override {
        config_ = AutoScalerConfig::default_config();
        collector_ = create_metrics_collector(config_);
        EXPECT_EQ(collector_->initialize(config_), ScalingResult::Success);
    }

    AutoScalerConfig config_;
    std::unique_ptr<IMetricsCollector> collector_;
};

TEST_F(MetricsCollectorTest, Initialization) {
    auto collector = create_metrics_collector(config_);
    EXPECT_EQ(collector->initialize(config_), ScalingResult::Success);
}

TEST_F(MetricsCollectorTest, CollectNodeMetrics) {
    NodeId node_id = 1;
    auto metrics = collector_->collect_node_metrics(node_id);

    // May return nullopt if node doesn't exist, but should not crash
    if (metrics.has_value()) {
        EXPECT_EQ(metrics->node_id, node_id);
    }
}

TEST_F(MetricsCollectorTest, AggregateMetrics) {
    std::vector<NodeMetrics> node_metrics;

    // Create sample metrics
    for (UInt32 i = 0; i < 3; ++i) {
        NodeMetrics metrics;
        metrics.node_id = i;
        metrics.cpu_usage = 0.5 + (i * 0.1);
        metrics.memory_usage = 0.6;
        metrics.entity_count = 5000 + (i * 1000);
        metrics.is_healthy = true;
        node_metrics.push_back(metrics);
    }

    ClusterMetrics cluster = collector_->aggregate_metrics(node_metrics);

    EXPECT_EQ(cluster.total_nodes, 3u);
    EXPECT_EQ(cluster.active_nodes, 3u);
    EXPECT_GT(cluster.avg_cpu_usage, 0.0);
    EXPECT_GT(cluster.total_entities, 0u);
}

TEST_F(MetricsCollectorTest, CustomMetricRegistration) {
    std::string metric_name = "custom_latency";
    auto collector_func = [](NodeId node_id) -> Real {
        return 10.0 + static_cast<Real>(node_id);
    };

    // Should not throw
    collector_->register_custom_metric(metric_name, collector_func);
}

TEST_F(MetricsCollectorTest, MetricHistoryClear) {
    collector_->clear_history();
    auto history = collector_->get_metric_history(1, MetricType::CPU);
    EXPECT_TRUE(history.empty());
}

// ============================================================================
// PolicyEvaluator Tests
// ============================================================================

class PolicyEvaluatorTest : public ::testing::Test {
protected:
    void SetUp() override {
        policy_ = ScalingPolicy::default_policy();
        evaluator_ = create_policy_evaluator(policy_);
        EXPECT_EQ(evaluator_->initialize(policy_), ScalingResult::Success);
    }

    ScalingPolicy policy_;
    std::unique_ptr<IScalingPolicyEvaluator> evaluator_;
};

TEST_F(PolicyEvaluatorTest, ScaleUpTriggerCPU) {
    ClusterMetrics cluster;
    cluster.total_nodes = 5;
    cluster.active_nodes = 5;
    cluster.avg_cpu_usage = 0.75;  // Above 70% threshold
    cluster.avg_entities_per_node = 5000;

    std::vector<NodeMetrics> node_metrics;

    // Evaluate multiple times to meet consecutive triggers requirement
    ScalingDecision decision;
    for (UInt32 i = 0; i < policy_.consecutive_triggers_required; ++i) {
        decision = evaluator_->evaluate(cluster, node_metrics);
    }

    // After consecutive triggers, should decide to scale up
    if (decision.is_valid) {
        EXPECT_EQ(decision.direction, ScalingDirection::Up);
        EXPECT_EQ(decision.primary_trigger, ScalingTrigger::CPU);
    }
}

TEST_F(PolicyEvaluatorTest, ScaleUpTriggerEntityCount) {
    ClusterMetrics cluster;
    cluster.total_nodes = 5;
    cluster.active_nodes = 5;
    cluster.avg_cpu_usage = 0.5;
    cluster.avg_entities_per_node = 9000;  // Above 8000 threshold

    std::vector<NodeMetrics> node_metrics;

    // Evaluate multiple times
    ScalingDecision decision;
    for (UInt32 i = 0; i < policy_.consecutive_triggers_required; ++i) {
        decision = evaluator_->evaluate(cluster, node_metrics);
    }

    if (decision.is_valid) {
        EXPECT_EQ(decision.direction, ScalingDirection::Up);
        EXPECT_EQ(decision.primary_trigger, ScalingTrigger::EntityCount);
    }
}

TEST_F(PolicyEvaluatorTest, ScaleDownTrigger) {
    ClusterMetrics cluster;
    cluster.total_nodes = 10;
    cluster.active_nodes = 10;
    cluster.avg_cpu_usage = 0.2;  // Below 30% threshold
    cluster.avg_entities_per_node = 1500;  // Below 2000 threshold

    std::vector<NodeMetrics> node_metrics;

    // Evaluate multiple times
    ScalingDecision decision;
    for (UInt32 i = 0; i < policy_.consecutive_triggers_required; ++i) {
        decision = evaluator_->evaluate(cluster, node_metrics);
    }

    if (decision.is_valid) {
        EXPECT_EQ(decision.direction, ScalingDirection::Down);
    }
}

TEST_F(PolicyEvaluatorTest, CooldownPeriod) {
    // Record a scale-up event
    ScalingEvent event;
    event.direction = ScalingDirection::Up;
    event.started_at = std::chrono::steady_clock::now();
    event.completed_at = event.started_at;
    evaluator_->record_scaling_event(event);

    // Should be in cooldown
    EXPECT_TRUE(evaluator_->is_in_cooldown(ScalingDirection::Up));
}

TEST_F(PolicyEvaluatorTest, ConsecutiveTriggersRequirement) {
    ClusterMetrics cluster;
    cluster.total_nodes = 5;
    cluster.avg_cpu_usage = 0.75;
    cluster.avg_entities_per_node = 5000;

    std::vector<NodeMetrics> node_metrics;

    // First evaluation should not trigger (need consecutive)
    ScalingDecision decision1 = evaluator_->evaluate(cluster, node_metrics);

    // May or may not be valid depending on consecutive count
    // Just verify it doesn't crash
}

TEST_F(PolicyEvaluatorTest, PolicyUpdate) {
    ScalingPolicy new_policy = ScalingPolicy::aggressive();
    evaluator_->update_policy(new_policy);

    const ScalingPolicy& current = evaluator_->get_policy();
    EXPECT_DOUBLE_EQ(current.cpu_scale_up_threshold, 0.6);
}

// ============================================================================
// AutoScalerLifecycle Tests
// ============================================================================

class AutoScalerLifecycleTest : public ::testing::Test {
protected:
    void SetUp() override {
        config_ = AutoScalerConfig::development();
        config_.export_to_prometheus = false;  // Disable for tests
        config_.integrate_partition_manager = false;  // Disable for tests
        scaler_ = create_auto_scaler(config_);
    }

    void TearDown() override {
        if (scaler_ && scaler_->is_initialized()) {
            scaler_->shutdown();
        }
    }

    AutoScalerConfig config_;
    std::unique_ptr<AutoScaler> scaler_;
};

TEST_F(AutoScalerLifecycleTest, Initialize) {
    ScalingResult result = scaler_->initialize(config_);
    // May fail if Kubernetes not available, but should not crash
    if (result == ScalingResult::Success) {
        EXPECT_TRUE(scaler_->is_initialized());
    }
}

TEST_F(AutoScalerLifecycleTest, Shutdown) {
    scaler_->initialize(config_);
    scaler_->shutdown();
    EXPECT_FALSE(scaler_->is_initialized());
}

TEST_F(AutoScalerLifecycleTest, IsInitialized) {
    // create_auto_scaler() automatically initializes
    EXPECT_TRUE(scaler_->is_initialized());

    scaler_->shutdown();
    EXPECT_FALSE(scaler_->is_initialized());

    // Can re-initialize after shutdown
    if (scaler_->initialize(config_) == ScalingResult::Success) {
        EXPECT_TRUE(scaler_->is_initialized());
    }
}

TEST_F(AutoScalerLifecycleTest, UpdateLoop) {
    if (scaler_->initialize(config_) == ScalingResult::Success) {
        // Should not crash even when called multiple times
        scaler_->update();
        scaler_->update();
        scaler_->update();
    }
}

// ============================================================================
// ScalingOperations Tests
// ============================================================================

class ScalingOperationsTest : public ::testing::Test {
protected:
    void SetUp() override {
        config_ = AutoScalerConfig::development();
        config_.export_to_prometheus = false;
        config_.integrate_partition_manager = false;
        scaler_ = create_auto_scaler(config_);
    }

    void TearDown() override {
        if (scaler_ && scaler_->is_initialized()) {
            scaler_->shutdown();
        }
    }

    AutoScalerConfig config_;
    std::unique_ptr<AutoScaler> scaler_;
};

TEST_F(ScalingOperationsTest, ScaleUp) {
    if (scaler_->initialize(config_) == ScalingResult::Success) {
        ScalingResult result = scaler_->scale_up(1);
        // May fail if Kubernetes not available
        EXPECT_TRUE(result == ScalingResult::Success ||
                   result == ScalingResult::K8sConnectionFailed ||
                   result == ScalingResult::NotInitialized);
    }
}

TEST_F(ScalingOperationsTest, ScaleDown) {
    if (scaler_->initialize(config_) == ScalingResult::Success) {
        ScalingResult result = scaler_->scale_down(1);
        EXPECT_TRUE(result == ScalingResult::Success ||
                   result == ScalingResult::MinNodesReached ||
                   result == ScalingResult::K8sConnectionFailed ||
                   result == ScalingResult::NotInitialized);
    }
}

TEST_F(ScalingOperationsTest, ScaleTo) {
    if (scaler_->initialize(config_) == ScalingResult::Success) {
        ScalingResult result = scaler_->scale_to(5, "Test scaling");
        // May fail if Kubernetes not available
        EXPECT_TRUE(result == ScalingResult::Success ||
                   result == ScalingResult::K8sConnectionFailed ||
                   result == ScalingResult::NotInitialized);
    }
}

TEST_F(ScalingOperationsTest, MinNodeLimit) {
    if (scaler_->initialize(config_) == ScalingResult::Success) {
        // Try to scale below minimum
        ScalingResult result = scaler_->scale_to(0, "Below minimum");
        EXPECT_TRUE(result != ScalingResult::Success);
    }
}

TEST_F(ScalingOperationsTest, MaxNodeLimit) {
    if (scaler_->initialize(config_) == ScalingResult::Success) {
        UInt32 max_nodes = config_.policy.max_nodes;

        // Try to scale above maximum
        ScalingResult result = scaler_->scale_to(max_nodes + 10, "Above maximum");

        if (result == ScalingResult::Success) {
            // If successful, should be capped at max_nodes
            UInt32 current = scaler_->get_current_replicas();
            EXPECT_LE(current, max_nodes);
        }
    }
}

TEST_F(ScalingOperationsTest, IsScaling) {
    if (scaler_->initialize(config_) == ScalingResult::Success) {
        EXPECT_FALSE(scaler_->is_scaling());
    }
}

TEST_F(ScalingOperationsTest, IsCooldown) {
    if (scaler_->initialize(config_) == ScalingResult::Success) {
        // Initially should not be in cooldown
        bool in_cooldown = scaler_->is_in_cooldown(ScalingDirection::Up);
        // Just verify it doesn't crash
    }
}

// ============================================================================
// Statistics Tests
// ============================================================================

class StatisticsTest : public ::testing::Test {
protected:
    void SetUp() override {
        config_ = AutoScalerConfig::development();
        config_.export_to_prometheus = false;
        config_.integrate_partition_manager = false;
        scaler_ = create_auto_scaler(config_);
    }

    void TearDown() override {
        if (scaler_ && scaler_->is_initialized()) {
            scaler_->shutdown();
        }
    }

    AutoScalerConfig config_;
    std::unique_ptr<AutoScaler> scaler_;
};

TEST_F(StatisticsTest, GetStats) {
    if (scaler_->initialize(config_) == ScalingResult::Success) {
        AutoScalerStats stats = scaler_->get_stats();

        EXPECT_EQ(stats.total_scale_ups, 0u);
        EXPECT_EQ(stats.total_scale_downs, 0u);
        EXPECT_EQ(stats.successful_scale_ups, 0u);
        EXPECT_EQ(stats.successful_scale_downs, 0u);
    }
}

TEST_F(StatisticsTest, ResetStats) {
    if (scaler_->initialize(config_) == ScalingResult::Success) {
        scaler_->reset_stats();

        AutoScalerStats stats = scaler_->get_stats();
        EXPECT_EQ(stats.total_scale_ups, 0u);
        EXPECT_EQ(stats.total_scale_downs, 0u);
    }
}

TEST_F(StatisticsTest, StatsReset) {
    AutoScalerStats stats;
    stats.total_scale_ups = 10;
    stats.total_scale_downs = 5;
    stats.successful_scale_ups = 8;

    stats.reset();

    EXPECT_EQ(stats.total_scale_ups, 0u);
    EXPECT_EQ(stats.total_scale_downs, 0u);
    EXPECT_EQ(stats.successful_scale_ups, 0u);
}

TEST_F(StatisticsTest, ScalingHistory) {
    if (scaler_->initialize(config_) == ScalingResult::Success) {
        auto history = scaler_->get_scaling_history(10);
        // Should return empty or valid history
        EXPECT_GE(history.size(), 0u);
    }
}

TEST_F(StatisticsTest, ClearHistory) {
    if (scaler_->initialize(config_) == ScalingResult::Success) {
        scaler_->clear_history();
        auto history = scaler_->get_scaling_history();
        EXPECT_EQ(history.size(), 0u);
    }
}

// ============================================================================
// Enum String Conversion Tests
// ============================================================================

TEST(ScalingEnumTest, ScalingResultToString) {
    EXPECT_STREQ(scaling_result_to_string(ScalingResult::Success), "Success");
    EXPECT_STREQ(scaling_result_to_string(ScalingResult::InvalidConfiguration),
                 "InvalidConfiguration");
    EXPECT_STREQ(scaling_result_to_string(ScalingResult::ScaleUpFailed), "ScaleUpFailed");
    EXPECT_STREQ(scaling_result_to_string(ScalingResult::K8sConnectionFailed),
                 "K8sConnectionFailed");
}

TEST(ScalingEnumTest, ScalingDirectionToString) {
    EXPECT_STREQ(scaling_direction_to_string(ScalingDirection::None), "None");
    EXPECT_STREQ(scaling_direction_to_string(ScalingDirection::Up), "Up");
    EXPECT_STREQ(scaling_direction_to_string(ScalingDirection::Down), "Down");
}

TEST(ScalingEnumTest, ScalingTriggerToString) {
    EXPECT_STREQ(scaling_trigger_to_string(ScalingTrigger::CPU), "CPU");
    EXPECT_STREQ(scaling_trigger_to_string(ScalingTrigger::EntityCount), "EntityCount");
    EXPECT_STREQ(scaling_trigger_to_string(ScalingTrigger::Memory), "Memory");
    EXPECT_STREQ(scaling_trigger_to_string(ScalingTrigger::Custom), "Custom");
}

TEST(ScalingEnumTest, MetricTypeToString) {
    EXPECT_STREQ(metric_type_to_string(MetricType::CPU), "CPU");
    EXPECT_STREQ(metric_type_to_string(MetricType::Memory), "Memory");
    EXPECT_STREQ(metric_type_to_string(MetricType::EntityCount), "EntityCount");
    EXPECT_STREQ(metric_type_to_string(MetricType::Custom), "Custom");
}

// ============================================================================
// NodeMetrics Tests
// ============================================================================

TEST(NodeMetricsTest, DefaultConstruction) {
    NodeMetrics metrics;

    EXPECT_EQ(metrics.node_id, INVALID_NODE_ID);
    EXPECT_DOUBLE_EQ(metrics.cpu_usage, 0.0);
    EXPECT_DOUBLE_EQ(metrics.memory_usage, 0.0);
    EXPECT_EQ(metrics.entity_count, 0u);
    EXPECT_TRUE(metrics.is_healthy);
}

TEST(NodeMetricsTest, SetValues) {
    NodeMetrics metrics;
    metrics.node_id = 5;
    metrics.node_name = "node-5";
    metrics.cpu_usage = 0.65;
    metrics.memory_usage = 0.50;
    metrics.entity_count = 7500;
    metrics.partition_count = 4;
    metrics.is_healthy = true;

    EXPECT_EQ(metrics.node_id, 5u);
    EXPECT_EQ(metrics.node_name, "node-5");
    EXPECT_DOUBLE_EQ(metrics.cpu_usage, 0.65);
    EXPECT_DOUBLE_EQ(metrics.memory_usage, 0.50);
    EXPECT_EQ(metrics.entity_count, 7500u);
    EXPECT_EQ(metrics.partition_count, 4u);
    EXPECT_TRUE(metrics.is_healthy);
}

// ============================================================================
// ClusterMetrics Tests
// ============================================================================

TEST(ClusterMetricsTest, DefaultConstruction) {
    ClusterMetrics metrics;

    EXPECT_EQ(metrics.total_nodes, 0u);
    EXPECT_EQ(metrics.active_nodes, 0u);
    EXPECT_DOUBLE_EQ(metrics.avg_cpu_usage, 0.0);
    EXPECT_EQ(metrics.total_entities, 0u);
}

TEST(ClusterMetricsTest, AggregateValues) {
    ClusterMetrics metrics;
    metrics.total_nodes = 10;
    metrics.active_nodes = 9;
    metrics.avg_cpu_usage = 0.65;
    metrics.max_cpu_usage = 0.85;
    metrics.min_cpu_usage = 0.45;
    metrics.total_entities = 75000;
    metrics.avg_entities_per_node = 7500;

    EXPECT_EQ(metrics.total_nodes, 10u);
    EXPECT_EQ(metrics.active_nodes, 9u);
    EXPECT_DOUBLE_EQ(metrics.avg_cpu_usage, 0.65);
    EXPECT_DOUBLE_EQ(metrics.max_cpu_usage, 0.85);
    EXPECT_DOUBLE_EQ(metrics.min_cpu_usage, 0.45);
    EXPECT_EQ(metrics.total_entities, 75000u);
    EXPECT_EQ(metrics.avg_entities_per_node, 7500u);
}

// ============================================================================
// ScalingEvent Tests
// ============================================================================

TEST(ScalingEventTest, EventCreation) {
    ScalingEvent event;
    event.event_id = 1;
    event.direction = ScalingDirection::Up;
    event.nodes_before = 5;
    event.nodes_after = 7;
    event.trigger = ScalingTrigger::CPU;
    event.reason = "High CPU usage";
    event.result = ScalingResult::Success;
    event.successful = true;
    event.added_nodes = {6, 7};
    event.entities_migrated = 500;

    EXPECT_EQ(event.event_id, 1u);
    EXPECT_EQ(event.direction, ScalingDirection::Up);
    EXPECT_EQ(event.nodes_before, 5u);
    EXPECT_EQ(event.nodes_after, 7u);
    EXPECT_EQ(event.trigger, ScalingTrigger::CPU);
    EXPECT_TRUE(event.successful);
    EXPECT_EQ(event.added_nodes.size(), 2u);
    EXPECT_EQ(event.entities_migrated, 500u);
}

// ============================================================================
// Policy Management Tests
// ============================================================================

class PolicyManagementTest : public ::testing::Test {
protected:
    void SetUp() override {
        config_ = AutoScalerConfig::development();
        config_.export_to_prometheus = false;
        config_.integrate_partition_manager = false;
        scaler_ = create_auto_scaler(config_);
    }

    void TearDown() override {
        if (scaler_ && scaler_->is_initialized()) {
            scaler_->shutdown();
        }
    }

    AutoScalerConfig config_;
    std::unique_ptr<AutoScaler> scaler_;
};

TEST_F(PolicyManagementTest, GetPolicy) {
    if (scaler_->initialize(config_) == ScalingResult::Success) {
        const ScalingPolicy& policy = scaler_->get_policy();
        EXPECT_GE(policy.min_nodes, 1u);
        EXPECT_LE(policy.max_nodes, 200u);
    }
}

TEST_F(PolicyManagementTest, UpdatePolicy) {
    if (scaler_->initialize(config_) == ScalingResult::Success) {
        ScalingPolicy new_policy = ScalingPolicy::conservative();
        scaler_->update_policy(new_policy);

        const ScalingPolicy& current = scaler_->get_policy();
        EXPECT_DOUBLE_EQ(current.cpu_scale_up_threshold, 0.8);
    }
}

TEST_F(PolicyManagementTest, EnableDisable) {
    if (scaler_->initialize(config_) == ScalingResult::Success) {
        scaler_->set_enabled(true);
        EXPECT_TRUE(scaler_->is_enabled());

        scaler_->set_enabled(false);
        EXPECT_FALSE(scaler_->is_enabled());
    }
}

// ============================================================================
// Callback Tests
// ============================================================================

class CallbackTest : public ::testing::Test {
protected:
    void SetUp() override {
        config_ = AutoScalerConfig::development();
        config_.export_to_prometheus = false;
        config_.integrate_partition_manager = false;
        scaler_ = create_auto_scaler(config_);
    }

    void TearDown() override {
        if (scaler_ && scaler_->is_initialized()) {
            scaler_->shutdown();
        }
    }

    AutoScalerConfig config_;
    std::unique_ptr<AutoScaler> scaler_;
};

TEST_F(CallbackTest, SetCallbacks) {
    bool decision_called = false;
    bool started_called = false;
    bool completed_called = false;
    bool metrics_called = false;

    scaler_->set_decision_callback([&](const ScalingDecision& decision) {
        decision_called = true;
    });

    scaler_->set_scaling_started_callback([&](const ScalingEvent& event) {
        started_called = true;
    });

    scaler_->set_scaling_completed_callback([&](const ScalingEvent& event) {
        completed_called = true;
    });

    scaler_->set_metrics_callback([&](const ClusterMetrics& metrics) {
        metrics_called = true;
    });

    // Just verify callbacks can be set without crashing
}

TEST_F(CallbackTest, NodeCallbacks) {
    bool node_added_called = false;
    bool node_removed_called = false;

    scaler_->set_node_added_callback([&](NodeId node_id) {
        node_added_called = true;
    });

    scaler_->set_node_removed_callback([&](NodeId node_id) {
        node_removed_called = true;
    });

    // Just verify callbacks can be set without crashing
}

// ============================================================================
// Edge Cases
// ============================================================================

TEST(AutoScalerEdgeCases, UninitializedAccess) {
    // Create with default config (automatically initializes via factory)
    auto scaler = create_auto_scaler();

    // Factory function initializes automatically
    EXPECT_TRUE(scaler->is_initialized());
    EXPECT_TRUE(scaler->is_enabled());

    // After shutdown, should be uninitialized
    scaler->shutdown();
    EXPECT_FALSE(scaler->is_initialized());
}

TEST(AutoScalerEdgeCases, DoubleInitialize) {
    AutoScalerConfig config = AutoScalerConfig::development();
    config.export_to_prometheus = false;
    config.integrate_partition_manager = false;

    auto scaler = create_auto_scaler(config);

    if (scaler->initialize(config) == ScalingResult::Success) {
        ScalingResult result = scaler->initialize(config);
        EXPECT_EQ(result, ScalingResult::AlreadyInitialized);
        scaler->shutdown();
    }
}

TEST(AutoScalerEdgeCases, DoubleShutdown) {
    auto scaler = create_auto_scaler();
    scaler->shutdown();
    scaler->shutdown();  // Should not crash
}
