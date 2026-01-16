// Copyright JaguarEngine Team. All Rights Reserved.
//
// Performance Telemetry System
// Real-time metrics collection and export for simulation monitoring

#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

namespace jaguar::telemetry {

//==============================================================================
// Forward Declarations
//==============================================================================

class Meter;
class Counter;
class Gauge;
class Histogram;
class Timer;
class TelemetryExporter;
class TelemetryRegistry;

//==============================================================================
// Metric Types
//==============================================================================

/// Metric value variant
struct MetricValue {
    enum class Type { Counter, Gauge, Histogram };

    Type type;

    union {
        int64_t counter_value;
        double gauge_value;
        struct {
            double sum;
            uint64_t count;
            double min;
            double max;
            double p50;
            double p90;
            double p99;
        } histogram;
    };

    MetricValue() : type(Type::Counter), counter_value(0) {}
};

/// Metric label pair
struct Label {
    std::string key;
    std::string value;

    bool operator==(const Label& other) const {
        return key == other.key && value == other.value;
    }
};

/// Metric metadata
struct MetricDescriptor {
    std::string name;
    std::string description;
    std::string unit;
    MetricValue::Type type;
    std::vector<std::string> label_keys;
};

/// Metric sample with labels and timestamp
struct MetricSample {
    std::string metric_name;
    std::vector<Label> labels;
    MetricValue value;
    std::chrono::system_clock::time_point timestamp;
};

//==============================================================================
// Counter - Monotonically increasing metric
//==============================================================================

class Counter {
public:
    Counter(std::string_view name, std::string_view description,
            std::vector<std::string> label_keys = {});
    ~Counter() = default;

    // Non-copyable, movable
    Counter(const Counter&) = delete;
    Counter& operator=(const Counter&) = delete;
    Counter(Counter&&) = default;
    Counter& operator=(Counter&&) = default;

    /// Increment counter by 1
    void increment();

    /// Increment counter by given amount
    void increment(int64_t amount);

    /// Increment with specific labels
    void increment(const std::vector<Label>& labels, int64_t amount = 1);

    /// Get current value
    int64_t value() const;

    /// Get value for specific labels
    int64_t value(const std::vector<Label>& labels) const;

    /// Get descriptor
    const MetricDescriptor& descriptor() const { return descriptor_; }

    /// Collect all samples
    std::vector<MetricSample> collect() const;

private:
    MetricDescriptor descriptor_;
    mutable std::mutex mutex_;
    std::atomic<int64_t> default_value_{0};
    std::unordered_map<std::string, std::atomic<int64_t>> labeled_values_;

    std::string make_label_key(const std::vector<Label>& labels) const;
};

//==============================================================================
// Gauge - Value that can go up and down
//==============================================================================

class Gauge {
public:
    Gauge(std::string_view name, std::string_view description,
          std::vector<std::string> label_keys = {});
    ~Gauge() = default;

    /// Set gauge value
    void set(double value);

    /// Set with labels
    void set(const std::vector<Label>& labels, double value);

    /// Increment gauge
    void increment(double amount = 1.0);

    /// Decrement gauge
    void decrement(double amount = 1.0);

    /// Get current value
    double value() const;

    /// Get value for labels
    double value(const std::vector<Label>& labels) const;

    /// Get descriptor
    const MetricDescriptor& descriptor() const { return descriptor_; }

    /// Collect all samples
    std::vector<MetricSample> collect() const;

private:
    MetricDescriptor descriptor_;
    mutable std::mutex mutex_;
    std::atomic<double> default_value_{0.0};
    std::unordered_map<std::string, std::atomic<double>> labeled_values_;

    std::string make_label_key(const std::vector<Label>& labels) const;
};

//==============================================================================
// Histogram - Distribution of values
//==============================================================================

class Histogram {
public:
    /// Default bucket boundaries for latency histograms (seconds)
    static const std::vector<double> DEFAULT_LATENCY_BUCKETS;

    /// Default bucket boundaries for size histograms
    static const std::vector<double> DEFAULT_SIZE_BUCKETS;

    Histogram(std::string_view name, std::string_view description,
              std::vector<double> bucket_boundaries = DEFAULT_LATENCY_BUCKETS,
              std::vector<std::string> label_keys = {});
    ~Histogram() = default;

    /// Record a value
    void observe(double value);

    /// Record with labels
    void observe(const std::vector<Label>& labels, double value);

    /// Get statistics
    struct Stats {
        double sum = 0.0;
        uint64_t count = 0;
        double min = 0.0;
        double max = 0.0;
        std::vector<uint64_t> bucket_counts;
    };

    Stats stats() const;
    Stats stats(const std::vector<Label>& labels) const;

    /// Get descriptor
    const MetricDescriptor& descriptor() const { return descriptor_; }

    /// Collect all samples
    std::vector<MetricSample> collect() const;

    /// Reset histogram
    void reset();

private:
    MetricDescriptor descriptor_;
    std::vector<double> bucket_boundaries_;
    mutable std::mutex mutex_;

    struct HistogramData {
        std::atomic<double> sum{0.0};
        std::atomic<uint64_t> count{0};
        std::atomic<double> min{std::numeric_limits<double>::max()};
        std::atomic<double> max{std::numeric_limits<double>::lowest()};
        std::vector<std::atomic<uint64_t>> bucket_counts;

        HistogramData(size_t num_buckets) : bucket_counts(num_buckets + 1) {
            for (auto& c : bucket_counts) c = 0;
        }
    };

    std::unique_ptr<HistogramData> default_data_;
    std::unordered_map<std::string, std::unique_ptr<HistogramData>> labeled_data_;

    std::string make_label_key(const std::vector<Label>& labels) const;
    void observe_internal(HistogramData& data, double value);
    double percentile(const HistogramData& data, double p) const;
};

//==============================================================================
// Timer - Convenience wrapper for timing operations
//==============================================================================

class Timer {
public:
    Timer(Histogram& histogram, std::vector<Label> labels = {});
    ~Timer();

    // Non-copyable
    Timer(const Timer&) = delete;
    Timer& operator=(const Timer&) = delete;

    /// Stop timer manually (otherwise stops in destructor)
    void stop();

    /// Get elapsed time in seconds
    double elapsed() const;

private:
    Histogram& histogram_;
    std::vector<Label> labels_;
    std::chrono::high_resolution_clock::time_point start_;
    bool stopped_ = false;
};

/// RAII scoped timer macro
#define JAGUAR_SCOPED_TIMER(histogram) \
    jaguar::telemetry::Timer _timer_##__LINE__(histogram)

#define JAGUAR_SCOPED_TIMER_LABELS(histogram, labels) \
    jaguar::telemetry::Timer _timer_##__LINE__(histogram, labels)

//==============================================================================
// Telemetry Exporter Interface
//==============================================================================

class TelemetryExporter {
public:
    virtual ~TelemetryExporter() = default;

    /// Export metrics
    virtual void export_metrics(const std::vector<MetricSample>& samples) = 0;

    /// Flush any buffered data
    virtual void flush() = 0;

    /// Shutdown exporter
    virtual void shutdown() = 0;
};

//==============================================================================
// Prometheus Exporter
//==============================================================================

class PrometheusExporter : public TelemetryExporter {
public:
    struct Config {
        std::string host = "0.0.0.0";
        uint16_t port = 9090;
        std::string path = "/metrics";
        std::string job_name = "jaguar_simulation";
    };

    explicit PrometheusExporter(const Config& config = Config{});
    ~PrometheusExporter() override;

    void export_metrics(const std::vector<MetricSample>& samples) override;
    void flush() override;
    void shutdown() override;

    /// Get Prometheus-formatted metrics string
    std::string format_metrics(const std::vector<MetricSample>& samples) const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

//==============================================================================
// OpenTelemetry Exporter
//==============================================================================

class OTLPExporter : public TelemetryExporter {
public:
    struct Config {
        std::string endpoint = "localhost:4317";
        bool use_tls = false;
        std::string tls_cert_path;
        std::chrono::milliseconds timeout{30000};
        std::unordered_map<std::string, std::string> headers;
        std::unordered_map<std::string, std::string> resource_attributes;
    };

    explicit OTLPExporter(const Config& config = Config{});
    ~OTLPExporter() override;

    void export_metrics(const std::vector<MetricSample>& samples) override;
    void flush() override;
    void shutdown() override;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

//==============================================================================
// Telemetry Registry - Central metric management
//==============================================================================

class TelemetryRegistry {
public:
    /// Get singleton instance
    static TelemetryRegistry& instance();

    /// Register a counter
    Counter& counter(std::string_view name, std::string_view description,
                     std::vector<std::string> label_keys = {});

    /// Register a gauge
    Gauge& gauge(std::string_view name, std::string_view description,
                 std::vector<std::string> label_keys = {});

    /// Register a histogram
    Histogram& histogram(std::string_view name, std::string_view description,
                         std::vector<double> bucket_boundaries = Histogram::DEFAULT_LATENCY_BUCKETS,
                         std::vector<std::string> label_keys = {});

    /// Add exporter
    void add_exporter(std::shared_ptr<TelemetryExporter> exporter);

    /// Remove exporter
    void remove_exporter(const TelemetryExporter* exporter);

    /// Collect all metrics
    std::vector<MetricSample> collect() const;

    /// Export all metrics to registered exporters
    void export_metrics();

    /// Start periodic export (interval in milliseconds)
    void start_periodic_export(std::chrono::milliseconds interval);

    /// Stop periodic export
    void stop_periodic_export();

    /// Shutdown all exporters
    void shutdown();

    /// Clear all metrics (for testing)
    void clear();

private:
    TelemetryRegistry();
    ~TelemetryRegistry();

    TelemetryRegistry(const TelemetryRegistry&) = delete;
    TelemetryRegistry& operator=(const TelemetryRegistry&) = delete;

    struct Impl;
    std::unique_ptr<Impl> impl_;
};

//==============================================================================
// Simulation-Specific Metrics
//==============================================================================

namespace metrics {

/// Pre-defined simulation metrics
struct SimulationMetrics {
    // Entity metrics
    Counter& entities_created;
    Counter& entities_destroyed;
    Gauge& active_entities;

    // Physics metrics
    Histogram& physics_step_duration;
    Counter& physics_collisions;
    Gauge& physics_bodies;

    // Network metrics
    Counter& dis_packets_sent;
    Counter& dis_packets_received;
    Counter& dis_packets_dropped;
    Histogram& dis_packet_latency;
    Gauge& network_connections;

    // Terrain metrics
    Counter& terrain_cache_hits;
    Counter& terrain_cache_misses;
    Histogram& terrain_query_duration;
    Gauge& terrain_cache_size_mb;

    // Performance metrics
    Gauge& simulation_time;
    Gauge& wall_clock_time;
    Gauge& realtime_ratio;
    Histogram& frame_duration;
    Gauge& memory_usage_mb;

    // Error metrics
    Counter& errors_total;
    Counter& warnings_total;

    static SimulationMetrics& instance();

private:
    SimulationMetrics();
};

/// Get pre-defined simulation metrics
inline SimulationMetrics& simulation() {
    return SimulationMetrics::instance();
}

} // namespace metrics

//==============================================================================
// Convenience Functions
//==============================================================================

/// Record entity spawn
inline void record_entity_spawned() {
    metrics::simulation().entities_created.increment();
    metrics::simulation().active_entities.increment();
}

/// Record entity destroyed
inline void record_entity_destroyed() {
    metrics::simulation().entities_destroyed.increment();
    metrics::simulation().active_entities.decrement();
}

/// Record physics step timing
inline Timer time_physics_step() {
    return Timer(metrics::simulation().physics_step_duration);
}

/// Record DIS packet sent
inline void record_dis_packet_sent() {
    metrics::simulation().dis_packets_sent.increment();
}

/// Record DIS packet received
inline void record_dis_packet_received() {
    metrics::simulation().dis_packets_received.increment();
}

/// Record DIS packet dropped
inline void record_dis_packet_dropped() {
    metrics::simulation().dis_packets_dropped.increment();
}

/// Record terrain cache hit
inline void record_terrain_cache_hit() {
    metrics::simulation().terrain_cache_hits.increment();
}

/// Record terrain cache miss
inline void record_terrain_cache_miss() {
    metrics::simulation().terrain_cache_misses.increment();
}

/// Update simulation timing
inline void update_simulation_timing(double sim_time, double wall_time) {
    auto& m = metrics::simulation();
    m.simulation_time.set(sim_time);
    m.wall_clock_time.set(wall_time);
    m.realtime_ratio.set(wall_time > 0 ? sim_time / wall_time : 1.0);
}

/// Record error
inline void record_error(std::string_view type) {
    metrics::simulation().errors_total.increment(
        {{{"type", std::string(type)}}});
}

/// Record warning
inline void record_warning(std::string_view type) {
    metrics::simulation().warnings_total.increment(
        {{{"type", std::string(type)}}});
}

} // namespace jaguar::telemetry
