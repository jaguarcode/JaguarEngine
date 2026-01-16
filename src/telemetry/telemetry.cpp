// Copyright JaguarEngine Team. All Rights Reserved.

#include "jaguar/telemetry/telemetry.h"

#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <iomanip>
#include <sstream>
#include <thread>

namespace jaguar::telemetry {

//==============================================================================
// Default Bucket Boundaries
//==============================================================================

const std::vector<double> Histogram::DEFAULT_LATENCY_BUCKETS = {
    0.0001, 0.0005, 0.001, 0.005, 0.01, 0.025, 0.05, 0.1, 0.25, 0.5, 1.0, 2.5, 5.0, 10.0
};

const std::vector<double> Histogram::DEFAULT_SIZE_BUCKETS = {
    100, 500, 1000, 5000, 10000, 50000, 100000, 500000, 1000000
};

//==============================================================================
// Counter Implementation
//==============================================================================

Counter::Counter(std::string_view name, std::string_view description,
                 std::vector<std::string> label_keys) {
    descriptor_.name = std::string(name);
    descriptor_.description = std::string(description);
    descriptor_.type = MetricValue::Type::Counter;
    descriptor_.label_keys = std::move(label_keys);
}

void Counter::increment() {
    default_value_.fetch_add(1, std::memory_order_relaxed);
}

void Counter::increment(int64_t amount) {
    default_value_.fetch_add(amount, std::memory_order_relaxed);
}

void Counter::increment(const std::vector<Label>& labels, int64_t amount) {
    if (labels.empty()) {
        increment(amount);
        return;
    }

    std::string key = make_label_key(labels);
    std::lock_guard<std::mutex> lock(mutex_);

    auto it = labeled_values_.find(key);
    if (it == labeled_values_.end()) {
        labeled_values_[key].store(amount, std::memory_order_relaxed);
    } else {
        it->second.fetch_add(amount, std::memory_order_relaxed);
    }
}

int64_t Counter::value() const {
    return default_value_.load(std::memory_order_relaxed);
}

int64_t Counter::value(const std::vector<Label>& labels) const {
    if (labels.empty()) {
        return value();
    }

    std::string key = make_label_key(labels);
    std::lock_guard<std::mutex> lock(mutex_);

    auto it = labeled_values_.find(key);
    if (it != labeled_values_.end()) {
        return it->second.load(std::memory_order_relaxed);
    }
    return 0;
}

std::vector<MetricSample> Counter::collect() const {
    std::vector<MetricSample> samples;
    auto now = std::chrono::system_clock::now();

    // Default (no labels) sample
    MetricSample sample;
    sample.metric_name = descriptor_.name;
    sample.timestamp = now;
    sample.value.type = MetricValue::Type::Counter;
    sample.value.counter_value = default_value_.load(std::memory_order_relaxed);
    samples.push_back(sample);

    // Labeled samples
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto& [key, val] : labeled_values_) {
        MetricSample labeled_sample;
        labeled_sample.metric_name = descriptor_.name;
        labeled_sample.timestamp = now;
        labeled_sample.value.type = MetricValue::Type::Counter;
        labeled_sample.value.counter_value = val.load(std::memory_order_relaxed);

        // Parse labels from key
        std::istringstream iss(key);
        std::string label_pair;
        while (std::getline(iss, label_pair, ',')) {
            size_t eq_pos = label_pair.find('=');
            if (eq_pos != std::string::npos) {
                labeled_sample.labels.push_back({
                    label_pair.substr(0, eq_pos),
                    label_pair.substr(eq_pos + 1)
                });
            }
        }
        samples.push_back(labeled_sample);
    }

    return samples;
}

std::string Counter::make_label_key(const std::vector<Label>& labels) const {
    std::ostringstream oss;
    bool first = true;
    for (const auto& label : labels) {
        if (!first) oss << ",";
        oss << label.key << "=" << label.value;
        first = false;
    }
    return oss.str();
}

//==============================================================================
// Gauge Implementation
//==============================================================================

Gauge::Gauge(std::string_view name, std::string_view description,
             std::vector<std::string> label_keys) {
    descriptor_.name = std::string(name);
    descriptor_.description = std::string(description);
    descriptor_.type = MetricValue::Type::Gauge;
    descriptor_.label_keys = std::move(label_keys);
}

void Gauge::set(double value) {
    default_value_.store(value, std::memory_order_relaxed);
}

void Gauge::set(const std::vector<Label>& labels, double value) {
    if (labels.empty()) {
        set(value);
        return;
    }

    std::string key = make_label_key(labels);
    std::lock_guard<std::mutex> lock(mutex_);
    labeled_values_[key].store(value, std::memory_order_relaxed);
}

void Gauge::increment(double amount) {
    double expected = default_value_.load(std::memory_order_relaxed);
    while (!default_value_.compare_exchange_weak(expected, expected + amount,
                                                  std::memory_order_relaxed)) {}
}

void Gauge::decrement(double amount) {
    increment(-amount);
}

double Gauge::value() const {
    return default_value_.load(std::memory_order_relaxed);
}

double Gauge::value(const std::vector<Label>& labels) const {
    if (labels.empty()) {
        return value();
    }

    std::string key = make_label_key(labels);
    std::lock_guard<std::mutex> lock(mutex_);

    auto it = labeled_values_.find(key);
    if (it != labeled_values_.end()) {
        return it->second.load(std::memory_order_relaxed);
    }
    return 0.0;
}

std::vector<MetricSample> Gauge::collect() const {
    std::vector<MetricSample> samples;
    auto now = std::chrono::system_clock::now();

    MetricSample sample;
    sample.metric_name = descriptor_.name;
    sample.timestamp = now;
    sample.value.type = MetricValue::Type::Gauge;
    sample.value.gauge_value = default_value_.load(std::memory_order_relaxed);
    samples.push_back(sample);

    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto& [key, val] : labeled_values_) {
        MetricSample labeled_sample;
        labeled_sample.metric_name = descriptor_.name;
        labeled_sample.timestamp = now;
        labeled_sample.value.type = MetricValue::Type::Gauge;
        labeled_sample.value.gauge_value = val.load(std::memory_order_relaxed);

        std::istringstream iss(key);
        std::string label_pair;
        while (std::getline(iss, label_pair, ',')) {
            size_t eq_pos = label_pair.find('=');
            if (eq_pos != std::string::npos) {
                labeled_sample.labels.push_back({
                    label_pair.substr(0, eq_pos),
                    label_pair.substr(eq_pos + 1)
                });
            }
        }
        samples.push_back(labeled_sample);
    }

    return samples;
}

std::string Gauge::make_label_key(const std::vector<Label>& labels) const {
    std::ostringstream oss;
    bool first = true;
    for (const auto& label : labels) {
        if (!first) oss << ",";
        oss << label.key << "=" << label.value;
        first = false;
    }
    return oss.str();
}

//==============================================================================
// Histogram Implementation
//==============================================================================

Histogram::Histogram(std::string_view name, std::string_view description,
                     std::vector<double> bucket_boundaries,
                     std::vector<std::string> label_keys)
    : bucket_boundaries_(std::move(bucket_boundaries)) {

    std::sort(bucket_boundaries_.begin(), bucket_boundaries_.end());

    descriptor_.name = std::string(name);
    descriptor_.description = std::string(description);
    descriptor_.type = MetricValue::Type::Histogram;
    descriptor_.label_keys = std::move(label_keys);

    default_data_ = std::make_unique<HistogramData>(bucket_boundaries_.size());
}

void Histogram::observe(double value) {
    observe_internal(*default_data_, value);
}

void Histogram::observe(const std::vector<Label>& labels, double value) {
    if (labels.empty()) {
        observe(value);
        return;
    }

    std::string key = make_label_key(labels);
    std::lock_guard<std::mutex> lock(mutex_);

    auto it = labeled_data_.find(key);
    if (it == labeled_data_.end()) {
        labeled_data_[key] = std::make_unique<HistogramData>(bucket_boundaries_.size());
        it = labeled_data_.find(key);
    }
    observe_internal(*it->second, value);
}

void Histogram::observe_internal(HistogramData& data, double value) {
    // Update sum
    double old_sum = data.sum.load(std::memory_order_relaxed);
    while (!data.sum.compare_exchange_weak(old_sum, old_sum + value,
                                           std::memory_order_relaxed)) {}

    // Update count
    data.count.fetch_add(1, std::memory_order_relaxed);

    // Update min
    double old_min = data.min.load(std::memory_order_relaxed);
    while (value < old_min) {
        if (data.min.compare_exchange_weak(old_min, value,
                                           std::memory_order_relaxed)) break;
    }

    // Update max
    double old_max = data.max.load(std::memory_order_relaxed);
    while (value > old_max) {
        if (data.max.compare_exchange_weak(old_max, value,
                                           std::memory_order_relaxed)) break;
    }

    // Find bucket and increment
    auto it = std::upper_bound(bucket_boundaries_.begin(), bucket_boundaries_.end(), value);
    size_t bucket_idx = static_cast<size_t>(it - bucket_boundaries_.begin());
    data.bucket_counts[bucket_idx].fetch_add(1, std::memory_order_relaxed);
}

Histogram::Stats Histogram::stats() const {
    Stats s;
    s.sum = default_data_->sum.load(std::memory_order_relaxed);
    s.count = default_data_->count.load(std::memory_order_relaxed);
    s.min = default_data_->min.load(std::memory_order_relaxed);
    s.max = default_data_->max.load(std::memory_order_relaxed);

    if (s.count == 0) {
        s.min = 0.0;
        s.max = 0.0;
    }

    s.bucket_counts.reserve(default_data_->bucket_counts.size());
    for (const auto& bc : default_data_->bucket_counts) {
        s.bucket_counts.push_back(bc.load(std::memory_order_relaxed));
    }

    return s;
}

double Histogram::percentile(const HistogramData& data, double p) const {
    uint64_t total = data.count.load(std::memory_order_relaxed);
    if (total == 0) return 0.0;

    uint64_t target = static_cast<uint64_t>(total * p);
    uint64_t cumulative = 0;

    for (size_t i = 0; i < data.bucket_counts.size(); ++i) {
        cumulative += data.bucket_counts[i].load(std::memory_order_relaxed);
        if (cumulative >= target) {
            if (i == 0) {
                return bucket_boundaries_.empty() ? 0.0 : bucket_boundaries_[0];
            } else if (i >= bucket_boundaries_.size()) {
                return bucket_boundaries_.empty() ? 0.0 : bucket_boundaries_.back();
            }
            return bucket_boundaries_[i - 1];
        }
    }

    return bucket_boundaries_.empty() ? 0.0 : bucket_boundaries_.back();
}

std::vector<MetricSample> Histogram::collect() const {
    std::vector<MetricSample> samples;
    auto now = std::chrono::system_clock::now();

    MetricSample sample;
    sample.metric_name = descriptor_.name;
    sample.timestamp = now;
    sample.value.type = MetricValue::Type::Histogram;

    sample.value.histogram.sum = default_data_->sum.load(std::memory_order_relaxed);
    sample.value.histogram.count = default_data_->count.load(std::memory_order_relaxed);
    sample.value.histogram.min = default_data_->min.load(std::memory_order_relaxed);
    sample.value.histogram.max = default_data_->max.load(std::memory_order_relaxed);

    if (sample.value.histogram.count == 0) {
        sample.value.histogram.min = 0.0;
        sample.value.histogram.max = 0.0;
    }

    sample.value.histogram.p50 = percentile(*default_data_, 0.50);
    sample.value.histogram.p90 = percentile(*default_data_, 0.90);
    sample.value.histogram.p99 = percentile(*default_data_, 0.99);

    samples.push_back(sample);

    return samples;
}

void Histogram::reset() {
    default_data_->sum.store(0.0, std::memory_order_relaxed);
    default_data_->count.store(0, std::memory_order_relaxed);
    default_data_->min.store(std::numeric_limits<double>::max(), std::memory_order_relaxed);
    default_data_->max.store(std::numeric_limits<double>::lowest(), std::memory_order_relaxed);
    for (auto& bc : default_data_->bucket_counts) {
        bc.store(0, std::memory_order_relaxed);
    }

    std::lock_guard<std::mutex> lock(mutex_);
    labeled_data_.clear();
}

std::string Histogram::make_label_key(const std::vector<Label>& labels) const {
    std::ostringstream oss;
    bool first = true;
    for (const auto& label : labels) {
        if (!first) oss << ",";
        oss << label.key << "=" << label.value;
        first = false;
    }
    return oss.str();
}

//==============================================================================
// Timer Implementation
//==============================================================================

Timer::Timer(Histogram& histogram, std::vector<Label> labels)
    : histogram_(histogram)
    , labels_(std::move(labels))
    , start_(std::chrono::high_resolution_clock::now()) {}

Timer::~Timer() {
    if (!stopped_) {
        stop();
    }
}

void Timer::stop() {
    if (stopped_) return;
    stopped_ = true;

    auto end = std::chrono::high_resolution_clock::now();
    double duration = std::chrono::duration<double>(end - start_).count();

    if (labels_.empty()) {
        histogram_.observe(duration);
    } else {
        histogram_.observe(labels_, duration);
    }
}

double Timer::elapsed() const {
    auto now = std::chrono::high_resolution_clock::now();
    return std::chrono::duration<double>(now - start_).count();
}

//==============================================================================
// Prometheus Exporter Implementation
//==============================================================================

struct PrometheusExporter::Impl {
    Config config;
    std::vector<MetricSample> last_samples;
    std::mutex mutex;
    std::atomic<bool> running{false};
};

PrometheusExporter::PrometheusExporter(const Config& config)
    : impl_(std::make_unique<Impl>()) {
    impl_->config = config;
}

PrometheusExporter::~PrometheusExporter() {
    shutdown();
}

void PrometheusExporter::export_metrics(const std::vector<MetricSample>& samples) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->last_samples = samples;
}

void PrometheusExporter::flush() {
    // Prometheus exporter is pull-based, nothing to flush
}

void PrometheusExporter::shutdown() {
    impl_->running.store(false);
}

std::string PrometheusExporter::format_metrics(const std::vector<MetricSample>& samples) const {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(6);

    for (const auto& sample : samples) {
        // Format labels
        std::string labels_str;
        if (!sample.labels.empty()) {
            std::ostringstream labels_oss;
            labels_oss << "{";
            bool first = true;
            for (const auto& label : sample.labels) {
                if (!first) labels_oss << ",";
                labels_oss << label.key << "=\"" << label.value << "\"";
                first = false;
            }
            labels_oss << "}";
            labels_str = labels_oss.str();
        }

        switch (sample.value.type) {
            case MetricValue::Type::Counter:
                oss << sample.metric_name << "_total" << labels_str << " "
                    << sample.value.counter_value << "\n";
                break;

            case MetricValue::Type::Gauge:
                oss << sample.metric_name << labels_str << " "
                    << sample.value.gauge_value << "\n";
                break;

            case MetricValue::Type::Histogram:
                oss << sample.metric_name << "_sum" << labels_str << " "
                    << sample.value.histogram.sum << "\n";
                oss << sample.metric_name << "_count" << labels_str << " "
                    << sample.value.histogram.count << "\n";
                oss << sample.metric_name << "_min" << labels_str << " "
                    << sample.value.histogram.min << "\n";
                oss << sample.metric_name << "_max" << labels_str << " "
                    << sample.value.histogram.max << "\n";
                break;
        }
    }

    return oss.str();
}

//==============================================================================
// OTLP Exporter Implementation
//==============================================================================

struct OTLPExporter::Impl {
    Config config;
    std::atomic<bool> running{true};
};

OTLPExporter::OTLPExporter(const Config& config)
    : impl_(std::make_unique<Impl>()) {
    impl_->config = config;
}

OTLPExporter::~OTLPExporter() {
    shutdown();
}

void OTLPExporter::export_metrics(const std::vector<MetricSample>& samples) {
    // OTLP export would use gRPC to send to collector
    // Placeholder implementation
    (void)samples;
}

void OTLPExporter::flush() {
    // Would flush gRPC buffer
}

void OTLPExporter::shutdown() {
    impl_->running.store(false);
}

//==============================================================================
// Telemetry Registry Implementation
//==============================================================================

struct TelemetryRegistry::Impl {
    std::mutex mutex;
    std::unordered_map<std::string, std::unique_ptr<Counter>> counters;
    std::unordered_map<std::string, std::unique_ptr<Gauge>> gauges;
    std::unordered_map<std::string, std::unique_ptr<Histogram>> histograms;
    std::vector<std::shared_ptr<TelemetryExporter>> exporters;

    std::atomic<bool> periodic_export_running{false};
    std::thread periodic_export_thread;
    std::condition_variable cv;
};

TelemetryRegistry::TelemetryRegistry()
    : impl_(std::make_unique<Impl>()) {}

TelemetryRegistry::~TelemetryRegistry() {
    shutdown();
}

TelemetryRegistry& TelemetryRegistry::instance() {
    static TelemetryRegistry instance;
    return instance;
}

Counter& TelemetryRegistry::counter(std::string_view name, std::string_view description,
                                     std::vector<std::string> label_keys) {
    std::string name_str(name);
    std::lock_guard<std::mutex> lock(impl_->mutex);

    auto it = impl_->counters.find(name_str);
    if (it != impl_->counters.end()) {
        return *it->second;
    }

    auto counter = std::make_unique<Counter>(name, description, std::move(label_keys));
    auto& ref = *counter;
    impl_->counters[name_str] = std::move(counter);
    return ref;
}

Gauge& TelemetryRegistry::gauge(std::string_view name, std::string_view description,
                                 std::vector<std::string> label_keys) {
    std::string name_str(name);
    std::lock_guard<std::mutex> lock(impl_->mutex);

    auto it = impl_->gauges.find(name_str);
    if (it != impl_->gauges.end()) {
        return *it->second;
    }

    auto gauge = std::make_unique<Gauge>(name, description, std::move(label_keys));
    auto& ref = *gauge;
    impl_->gauges[name_str] = std::move(gauge);
    return ref;
}

Histogram& TelemetryRegistry::histogram(std::string_view name, std::string_view description,
                                         std::vector<double> bucket_boundaries,
                                         std::vector<std::string> label_keys) {
    std::string name_str(name);
    std::lock_guard<std::mutex> lock(impl_->mutex);

    auto it = impl_->histograms.find(name_str);
    if (it != impl_->histograms.end()) {
        return *it->second;
    }

    auto histogram = std::make_unique<Histogram>(name, description,
                                                  std::move(bucket_boundaries),
                                                  std::move(label_keys));
    auto& ref = *histogram;
    impl_->histograms[name_str] = std::move(histogram);
    return ref;
}

void TelemetryRegistry::add_exporter(std::shared_ptr<TelemetryExporter> exporter) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->exporters.push_back(std::move(exporter));
}

void TelemetryRegistry::remove_exporter(const TelemetryExporter* exporter) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->exporters.erase(
        std::remove_if(impl_->exporters.begin(), impl_->exporters.end(),
                       [exporter](const auto& e) { return e.get() == exporter; }),
        impl_->exporters.end());
}

std::vector<MetricSample> TelemetryRegistry::collect() const {
    std::vector<MetricSample> samples;
    std::lock_guard<std::mutex> lock(impl_->mutex);

    for (const auto& [name, counter] : impl_->counters) {
        auto counter_samples = counter->collect();
        samples.insert(samples.end(), counter_samples.begin(), counter_samples.end());
    }

    for (const auto& [name, gauge] : impl_->gauges) {
        auto gauge_samples = gauge->collect();
        samples.insert(samples.end(), gauge_samples.begin(), gauge_samples.end());
    }

    for (const auto& [name, histogram] : impl_->histograms) {
        auto histogram_samples = histogram->collect();
        samples.insert(samples.end(), histogram_samples.begin(), histogram_samples.end());
    }

    return samples;
}

void TelemetryRegistry::export_metrics() {
    auto samples = collect();

    std::lock_guard<std::mutex> lock(impl_->mutex);
    for (auto& exporter : impl_->exporters) {
        exporter->export_metrics(samples);
    }
}

void TelemetryRegistry::start_periodic_export(std::chrono::milliseconds interval) {
    if (impl_->periodic_export_running.exchange(true)) {
        return;  // Already running
    }

    impl_->periodic_export_thread = std::thread([this, interval]() {
        while (impl_->periodic_export_running.load()) {
            export_metrics();

            std::unique_lock<std::mutex> lock(impl_->mutex);
            impl_->cv.wait_for(lock, interval, [this]() {
                return !impl_->periodic_export_running.load();
            });
        }
    });
}

void TelemetryRegistry::stop_periodic_export() {
    if (!impl_->periodic_export_running.exchange(false)) {
        return;  // Not running
    }

    impl_->cv.notify_all();
    if (impl_->periodic_export_thread.joinable()) {
        impl_->periodic_export_thread.join();
    }
}

void TelemetryRegistry::shutdown() {
    stop_periodic_export();

    std::lock_guard<std::mutex> lock(impl_->mutex);
    for (auto& exporter : impl_->exporters) {
        exporter->flush();
        exporter->shutdown();
    }
    impl_->exporters.clear();
}

void TelemetryRegistry::clear() {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->counters.clear();
    impl_->gauges.clear();
    impl_->histograms.clear();
}

//==============================================================================
// Simulation Metrics Implementation
//==============================================================================

namespace metrics {

SimulationMetrics::SimulationMetrics()
    : entities_created(TelemetryRegistry::instance().counter(
          "jaguar_entities_created", "Total entities created"))
    , entities_destroyed(TelemetryRegistry::instance().counter(
          "jaguar_entities_destroyed", "Total entities destroyed"))
    , active_entities(TelemetryRegistry::instance().gauge(
          "jaguar_active_entities", "Currently active entities"))
    , physics_step_duration(TelemetryRegistry::instance().histogram(
          "jaguar_physics_step_duration_seconds", "Physics step duration"))
    , physics_collisions(TelemetryRegistry::instance().counter(
          "jaguar_physics_collisions", "Total physics collisions detected"))
    , physics_bodies(TelemetryRegistry::instance().gauge(
          "jaguar_physics_bodies", "Number of physics bodies"))
    , dis_packets_sent(TelemetryRegistry::instance().counter(
          "jaguar_dis_packets_sent", "DIS packets sent"))
    , dis_packets_received(TelemetryRegistry::instance().counter(
          "jaguar_dis_packets_received", "DIS packets received"))
    , dis_packets_dropped(TelemetryRegistry::instance().counter(
          "jaguar_dis_packets_dropped", "DIS packets dropped"))
    , dis_packet_latency(TelemetryRegistry::instance().histogram(
          "jaguar_dis_packet_latency_seconds", "DIS packet latency"))
    , network_connections(TelemetryRegistry::instance().gauge(
          "jaguar_network_connections", "Active network connections"))
    , terrain_cache_hits(TelemetryRegistry::instance().counter(
          "jaguar_terrain_cache_hits", "Terrain cache hits"))
    , terrain_cache_misses(TelemetryRegistry::instance().counter(
          "jaguar_terrain_cache_misses", "Terrain cache misses"))
    , terrain_query_duration(TelemetryRegistry::instance().histogram(
          "jaguar_terrain_query_duration_seconds", "Terrain query duration"))
    , terrain_cache_size_mb(TelemetryRegistry::instance().gauge(
          "jaguar_terrain_cache_size_mb", "Terrain cache size in MB"))
    , simulation_time(TelemetryRegistry::instance().gauge(
          "jaguar_simulation_time_seconds", "Current simulation time"))
    , wall_clock_time(TelemetryRegistry::instance().gauge(
          "jaguar_wall_clock_time_seconds", "Wall clock time elapsed"))
    , realtime_ratio(TelemetryRegistry::instance().gauge(
          "jaguar_realtime_ratio", "Simulation to real-time ratio"))
    , frame_duration(TelemetryRegistry::instance().histogram(
          "jaguar_frame_duration_seconds", "Frame duration"))
    , memory_usage_mb(TelemetryRegistry::instance().gauge(
          "jaguar_memory_usage_mb", "Memory usage in MB"))
    , errors_total(TelemetryRegistry::instance().counter(
          "jaguar_errors", "Total errors", {"type"}))
    , warnings_total(TelemetryRegistry::instance().counter(
          "jaguar_warnings", "Total warnings", {"type"})) {}

SimulationMetrics& SimulationMetrics::instance() {
    static SimulationMetrics instance;
    return instance;
}

} // namespace metrics

} // namespace jaguar::telemetry
