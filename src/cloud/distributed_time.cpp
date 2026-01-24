/**
 * @file distributed_time.cpp
 * @brief Implementation of distributed time synchronization
 */

#include "jaguar/cloud/distributed_time.h"
#include <algorithm>
#include <random>
#include <thread>

namespace jaguar::cloud {

// ============================================================================
// Simple Raft Time Master Implementation
// ============================================================================

class SimpleTimeMaster : public ITimeMaster {
public:
    SimpleTimeMaster(const NodeId& local_id,
                     const std::vector<NodeId>& cluster_nodes,
                     const TimeSyncConfig& config)
        : local_id_(local_id)
        , config_(config)
        , state_(RaftState::Follower)
        , current_term_(0)
        , commit_index_(0)
        , last_applied_(0)
        , votes_received_(0)
        , last_heartbeat_(std::chrono::steady_clock::now()) {
        // Initialize cluster nodes
        for (const auto& node_id : cluster_nodes) {
            if (node_id != local_id_) {
                peers_.insert(node_id);
                next_index_[node_id] = 1;
                match_index_[node_id] = 0;
            }
        }
        reset_election_timeout();
    }

    RaftState get_state() const override {
        return state_;
    }

    UInt64 get_current_term() const override {
        return current_term_;
    }

    std::optional<NodeId> get_leader_id() const override {
        return leader_id_;
    }

    bool is_leader() const override {
        return state_ == RaftState::Leader;
    }

    TimeSyncResult start_election() override {
        if (state_ == RaftState::Leader) {
            return TimeSyncResult::Success;
        }

        // Transition to candidate
        state_ = RaftState::Candidate;
        current_term_++;
        voted_for_ = local_id_;
        votes_received_ = 1;  // Vote for self
        leader_id_.reset();

        reset_election_timeout();

        // Single-node cluster: immediately become leader with self-vote
        if (peers_.empty()) {
            become_leader();
            return TimeSyncResult::Success;
        }

        return TimeSyncResult::ElectionInProgress;
    }

    TimeSyncResult handle_vote_request(const VoteRequest& request, VoteResponse& response) override {
        response.term = current_term_;
        response.voter_id = local_id_;
        response.vote_granted = false;

        // Rule 1: Reply false if term < currentTerm
        if (request.term < current_term_) {
            return TimeSyncResult::Success;
        }

        // Step down if we see higher term
        if (request.term > current_term_) {
            step_down(request.term);
        }

        // Rule 2: Grant vote if haven't voted or already voted for this candidate
        //         AND candidate's log is at least as up-to-date
        bool can_vote = !voted_for_.has_value() || voted_for_.value() == request.candidate_id;
        bool log_ok = is_log_up_to_date(request.last_log_index, request.last_log_term);

        if (can_vote && log_ok) {
            voted_for_ = request.candidate_id;
            response.vote_granted = true;
            last_heartbeat_ = std::chrono::steady_clock::now();
        }

        return TimeSyncResult::Success;
    }

    TimeSyncResult handle_vote_response(const VoteResponse& response) override {
        if (state_ != RaftState::Candidate) {
            return TimeSyncResult::Success;
        }

        if (response.term > current_term_) {
            step_down(response.term);
            return TimeSyncResult::Success;
        }

        if (response.term == current_term_ && response.vote_granted) {
            votes_received_++;
            // Check for majority
            if (votes_received_ > (peers_.size() + 1) / 2) {
                become_leader();
            }
        }

        return TimeSyncResult::Success;
    }

    TimeSyncResult append_entry(const RaftLogEntry& entry) override {
        if (state_ != RaftState::Leader) {
            return TimeSyncResult::NotLeader;
        }

        RaftLogEntry new_entry = entry;
        new_entry.term = current_term_;
        new_entry.index = log_.empty() ? 1 : log_.back().index + 1;
        log_.push_back(new_entry);

        return TimeSyncResult::Success;
    }

    TimeSyncResult handle_append_request(const AppendEntriesRequest& request,
                                         AppendEntriesResponse& response) override {
        response.term = current_term_;
        response.follower_id = local_id_;
        response.success = false;
        response.match_index = 0;

        // Rule 1: Reply false if term < currentTerm
        if (request.term < current_term_) {
            return TimeSyncResult::Success;
        }

        // Reset election timeout on valid AppendEntries
        last_heartbeat_ = std::chrono::steady_clock::now();

        // Step down if we see higher or equal term from leader
        if (request.term >= current_term_) {
            if (request.term > current_term_ || state_ != RaftState::Follower) {
                step_down(request.term);
            }
            leader_id_ = request.leader_id;
        }

        // Rule 2: Reply false if log doesn't contain entry at prevLogIndex
        //         whose term matches prevLogTerm
        if (request.prev_log_index > 0) {
            if (request.prev_log_index > log_.size()) {
                return TimeSyncResult::Success;
            }
            if (log_[request.prev_log_index - 1].term != request.prev_log_term) {
                return TimeSyncResult::Success;
            }
        }

        // Rule 3: If existing entry conflicts with new one, delete it and following
        UInt64 insert_index = request.prev_log_index;
        for (const auto& entry : request.entries) {
            insert_index++;
            if (insert_index <= log_.size()) {
                if (log_[insert_index - 1].term != entry.term) {
                    log_.resize(insert_index - 1);
                    log_.push_back(entry);
                }
            } else {
                log_.push_back(entry);
            }
        }

        // Rule 5: Update commit index
        if (request.leader_commit > commit_index_) {
            commit_index_ = std::min(request.leader_commit,
                                     log_.empty() ? 0 : log_.back().index);
        }

        response.success = true;
        response.match_index = log_.empty() ? 0 : log_.back().index;

        return TimeSyncResult::Success;
    }

    TimeSyncResult handle_append_response(const AppendEntriesResponse& response) override {
        if (state_ != RaftState::Leader) {
            return TimeSyncResult::Success;
        }

        if (response.term > current_term_) {
            step_down(response.term);
            return TimeSyncResult::Success;
        }

        if (response.success) {
            next_index_[response.follower_id] = response.match_index + 1;
            match_index_[response.follower_id] = response.match_index;
            update_commit_index();
        } else {
            // Decrement next_index and retry
            if (next_index_[response.follower_id] > 1) {
                next_index_[response.follower_id]--;
            }
        }

        return TimeSyncResult::Success;
    }

    UInt64 get_commit_index() const override {
        return commit_index_;
    }

    UInt64 get_last_log_index() const override {
        return log_.empty() ? 0 : log_.back().index;
    }

    std::optional<RaftLogEntry> get_log_entry(UInt64 index) const override {
        if (index == 0 || index > log_.size()) {
            return std::nullopt;
        }
        return log_[index - 1];
    }

    void tick() override {
        auto now = std::chrono::steady_clock::now();
        auto elapsed = now - last_heartbeat_;

        switch (state_) {
            case RaftState::Follower:
            case RaftState::Candidate:
                if (elapsed >= election_timeout_) {
                    start_election();
                }
                break;

            case RaftState::Leader:
                if (elapsed >= config_.heartbeat_interval) {
                    last_heartbeat_ = now;
                    // In real implementation, send heartbeats to all peers
                }
                break;
        }
    }

    void reset() override {
        state_ = RaftState::Follower;
        current_term_ = 0;
        voted_for_.reset();
        leader_id_.reset();
        log_.clear();
        commit_index_ = 0;
        last_applied_ = 0;
        votes_received_ = 0;
        reset_election_timeout();
    }

private:
    void step_down(UInt64 new_term) {
        current_term_ = new_term;
        state_ = RaftState::Follower;
        voted_for_.reset();
        reset_election_timeout();
    }

    void become_leader() {
        state_ = RaftState::Leader;
        leader_id_ = local_id_;

        // Initialize leader state
        UInt64 last_index = get_last_log_index();
        for (const auto& peer : peers_) {
            next_index_[peer] = last_index + 1;
            match_index_[peer] = 0;
        }

        last_heartbeat_ = std::chrono::steady_clock::now();
    }

    bool is_log_up_to_date(UInt64 last_index, UInt64 last_term) const {
        UInt64 my_last_index = get_last_log_index();
        UInt64 my_last_term = log_.empty() ? 0 : log_.back().term;

        if (last_term != my_last_term) {
            return last_term > my_last_term;
        }
        return last_index >= my_last_index;
    }

    void update_commit_index() {
        // Find N such that majority of match_index[i] >= N
        std::vector<UInt64> indices;
        indices.push_back(get_last_log_index());  // Leader's own log
        for (const auto& [peer, index] : match_index_) {
            indices.push_back(index);
        }
        std::sort(indices.begin(), indices.end());
        UInt64 median_index = indices[indices.size() / 2];

        if (median_index > commit_index_ && !log_.empty() &&
            log_[median_index - 1].term == current_term_) {
            commit_index_ = median_index;
        }
    }

    void reset_election_timeout() {
        std::random_device rd;
        std::mt19937 gen(rd());
        auto min_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            config_.election_timeout_min).count();
        auto max_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            config_.election_timeout_max).count();
        std::uniform_int_distribution<> dis(static_cast<int>(min_ms),
                                            static_cast<int>(max_ms));
        election_timeout_ = std::chrono::milliseconds(dis(gen));
        last_heartbeat_ = std::chrono::steady_clock::now();
    }

    NodeId local_id_;
    TimeSyncConfig config_;
    std::unordered_set<NodeId> peers_;

    RaftState state_;
    UInt64 current_term_;
    std::optional<NodeId> voted_for_;
    std::optional<NodeId> leader_id_;

    std::vector<RaftLogEntry> log_;
    UInt64 commit_index_;
    UInt64 last_applied_;

    std::unordered_map<NodeId, UInt64> next_index_;
    std::unordered_map<NodeId, UInt64> match_index_;

    UInt32 votes_received_;
    std::chrono::steady_clock::time_point last_heartbeat_;
    Duration election_timeout_;
};

// ============================================================================
// NTP-like Clock Synchronizer Implementation
// ============================================================================

class NTPClockSynchronizer : public IClockSynchronizer {
public:
    explicit NTPClockSynchronizer(const NodeId& local_id, UInt32 sample_count = 8)
        : local_id_(local_id)
        , sample_count_(sample_count)
        , offset_(Duration::zero())
        , rtt_(Duration::zero())
        , drift_rate_(0.0)
        , sync_count_(0)
        , max_observed_offset_(Duration::zero())
        , sequence_(0) {
    }

    TimeSyncResult request_sync(const NodeId& /*master_id*/) override {
        // Create request with current time
        pending_request_.sender_id = local_id_;
        pending_request_.t1 = std::chrono::steady_clock::now();
        pending_request_.sequence = ++sequence_;
        return TimeSyncResult::Success;
    }

    TimeSyncResult handle_sync_request(const ClockSyncRequest& request,
                                       ClockSyncResponse& response) override {
        response.responder_id = local_id_;
        response.t1 = request.t1;
        response.t2 = std::chrono::steady_clock::now();
        response.t3 = std::chrono::steady_clock::now();  // Immediately after
        response.sequence = request.sequence;
        return TimeSyncResult::Success;
    }

    TimeSyncResult handle_sync_response(const ClockSyncResponse& response) override {
        if (response.sequence != pending_request_.sequence) {
            return TimeSyncResult::StaleEvent;
        }

        auto t4 = std::chrono::steady_clock::now();

        // NTP offset calculation:
        // offset = ((t2 - t1) + (t3 - t4)) / 2
        auto d1 = response.t2 - pending_request_.t1;
        auto d2 = response.t3 - t4;
        auto new_offset = (d1 + d2) / 2;

        // Round-trip time: (t4 - t1) - (t3 - t2)
        auto new_rtt = (t4 - pending_request_.t1) - (response.t3 - response.t2);

        // Add to samples
        offset_samples_.push_back(new_offset);
        rtt_samples_.push_back(new_rtt);

        // Keep only recent samples
        while (offset_samples_.size() > sample_count_) {
            offset_samples_.erase(offset_samples_.begin());
        }
        while (rtt_samples_.size() > sample_count_) {
            rtt_samples_.erase(rtt_samples_.begin());
        }

        // Calculate drift rate if we have enough samples
        if (offset_samples_.size() >= 2) {
            auto prev_offset = offset_samples_[offset_samples_.size() - 2];
            auto curr_offset = offset_samples_[offset_samples_.size() - 1];
            auto diff = curr_offset - prev_offset;
            // Simplified drift rate in ppm (parts per million)
            auto diff_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(diff).count();
            auto rtt_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(new_rtt).count();
            if (rtt_ns > 0) {
                drift_rate_ = static_cast<Float64>(diff_ns) / static_cast<Float64>(rtt_ns) * 1e6;
            }
        }

        // Update filtered offset (median filtering)
        update_filtered_offset();

        sync_count_++;

        // Track max observed offset
        auto abs_offset = offset_ < Duration::zero() ? -offset_ : offset_;
        if (abs_offset > max_observed_offset_) {
            max_observed_offset_ = abs_offset;
        }

        return TimeSyncResult::Success;
    }

    Duration get_offset() const override {
        return offset_;
    }

    Duration get_round_trip_time() const override {
        return rtt_;
    }

    Float64 get_drift_rate() const override {
        return drift_rate_;
    }

    WallClockTime get_adjusted_time() const override {
        return std::chrono::steady_clock::now() + offset_;
    }

    Duration adjust_duration(Duration local_duration) const override {
        // Adjust for drift
        auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(local_duration).count();
        auto adjusted_ns = ns + static_cast<Int64>(ns * drift_rate_ / 1e6);
        return std::chrono::nanoseconds(adjusted_ns);
    }

    UInt64 get_sync_count() const override {
        return sync_count_;
    }

    Duration get_max_observed_offset() const override {
        return max_observed_offset_;
    }

private:
    void update_filtered_offset() {
        if (offset_samples_.empty()) return;

        // Use median filtering for robustness
        std::vector<Duration> sorted = offset_samples_;
        std::sort(sorted.begin(), sorted.end());
        offset_ = sorted[sorted.size() / 2];

        // Also update RTT
        if (!rtt_samples_.empty()) {
            std::vector<Duration> sorted_rtt = rtt_samples_;
            std::sort(sorted_rtt.begin(), sorted_rtt.end());
            rtt_ = sorted_rtt[sorted_rtt.size() / 2];
        }
    }

    NodeId local_id_;
    UInt32 sample_count_;

    Duration offset_;
    Duration rtt_;
    Float64 drift_rate_;

    std::vector<Duration> offset_samples_;
    std::vector<Duration> rtt_samples_;

    UInt64 sync_count_;
    Duration max_observed_offset_;

    ClockSyncRequest pending_request_;
    UInt64 sequence_;
};

// ============================================================================
// Causality Manager Implementation
// ============================================================================

class SimpleCausalityManager : public ICausalityManager {
public:
    SimpleCausalityManager(const NodeId& local_id, bool enable_vector_clocks)
        : local_id_(local_id)
        , enable_vector_clocks_(enable_vector_clocks)
        , logical_time_(0)
        , next_event_id_(1) {
        vector_clock_.set(local_id_, 0);
    }

    LogicalTime get_logical_time() const override {
        return logical_time_;
    }

    LogicalTime increment_and_get() override {
        return ++logical_time_;
    }

    void update_on_receive(LogicalTime received_time) override {
        // Lamport clock rule: set to max(local, received) + 1
        logical_time_ = std::max(logical_time_, received_time) + 1;
    }

    VectorClock get_vector_clock() const override {
        return vector_clock_;
    }

    void increment_vector_clock() override {
        vector_clock_.increment(local_id_);
    }

    void merge_vector_clock(const VectorClock& other) override {
        vector_clock_.merge(other);
        vector_clock_.increment(local_id_);
    }

    TimestampedEvent create_event(const std::string& event_type,
                                  SimulationTime sim_time,
                                  std::vector<UInt8> payload) override {
        TimestampedEvent event;
        event.event_id = next_event_id_++;
        event.source_node = local_id_;
        event.sim_time = sim_time;
        event.logical_time = increment_and_get();
        event.wall_time = std::chrono::steady_clock::now();
        event.event_type = event_type;
        event.payload = std::move(payload);
        event.processed = false;

        if (enable_vector_clocks_) {
            increment_vector_clock();
            event.vector_clock = vector_clock_;
        }

        return event;
    }

    bool happens_before(const TimestampedEvent& a, const TimestampedEvent& b) const override {
        // If vector clocks enabled, use them for more precise ordering
        if (enable_vector_clocks_) {
            return a.vector_clock.happens_before(b.vector_clock);
        }
        // Fall back to Lamport clock
        if (a.logical_time < b.logical_time) return true;
        if (a.logical_time == b.logical_time) {
            // Tie-breaker: use source node ID
            return a.source_node < b.source_node;
        }
        return false;
    }

    bool are_concurrent(const TimestampedEvent& a, const TimestampedEvent& b) const override {
        if (enable_vector_clocks_) {
            return a.vector_clock.concurrent_with(b.vector_clock);
        }
        // With only Lamport clocks, we can't definitively say events are concurrent
        // Two events with the same logical time from different nodes may be concurrent
        return a.logical_time == b.logical_time && a.source_node != b.source_node;
    }

private:
    NodeId local_id_;
    bool enable_vector_clocks_;
    LogicalTime logical_time_;
    VectorClock vector_clock_;
    UInt64 next_event_id_;
};

// ============================================================================
// Event Reorderer Implementation
// ============================================================================

class SimpleEventReorderer : public IEventReorderer {
public:
    SimpleEventReorderer(Duration buffer_time, UInt32 max_buffered)
        : buffer_time_(buffer_time)
        , max_buffered_(max_buffered)
        , safe_time_(0.0) {
    }

    TimeSyncResult submit_event(TimestampedEvent event) override {
        if (buffer_.size() >= max_buffered_) {
            return TimeSyncResult::CausalityViolation;  // Buffer full
        }

        buffer_.push_back(std::move(event));

        // Sort by simulation time, then logical time, then source node
        std::sort(buffer_.begin(), buffer_.end(),
            [](const TimestampedEvent& a, const TimestampedEvent& b) {
                if (a.sim_time != b.sim_time) return a.sim_time < b.sim_time;
                if (a.logical_time != b.logical_time) return a.logical_time < b.logical_time;
                return a.source_node < b.source_node;
            });

        return TimeSyncResult::Success;
    }

    TimeSyncResult submit_events(std::vector<TimestampedEvent> events) override {
        for (auto& event : events) {
            auto result = submit_event(std::move(event));
            if (result != TimeSyncResult::Success) {
                return result;
            }
        }
        return TimeSyncResult::Success;
    }

    std::vector<TimestampedEvent> get_ready_events() override {
        std::vector<TimestampedEvent> ready;
        auto now = std::chrono::steady_clock::now();

        // Find events that are ready (before safe_time or aged out of buffer)
        auto it = buffer_.begin();
        while (it != buffer_.end()) {
            bool is_ready = it->sim_time <= safe_time_;

            // Also release events that have been buffered long enough
            if (!is_ready) {
                auto age = now - it->wall_time;
                is_ready = age >= buffer_time_;
            }

            if (is_ready) {
                ready.push_back(std::move(*it));
                it = buffer_.erase(it);
            } else {
                ++it;
            }
        }

        return ready;
    }

    std::optional<TimestampedEvent> pop_next_event() override {
        auto ready = get_ready_events();
        if (ready.empty()) {
            return std::nullopt;
        }
        // Return first ready event, put rest back
        auto result = std::move(ready[0]);
        for (size_t i = 1; i < ready.size(); i++) {
            buffer_.push_back(std::move(ready[i]));
        }
        return result;
    }

    UInt32 get_buffered_count() const override {
        return static_cast<UInt32>(buffer_.size());
    }

    void flush() override {
        // Mark all events as ready by setting safe_time to max
        safe_time_ = std::numeric_limits<SimulationTime>::max();
    }

    void clear() override {
        buffer_.clear();
        safe_time_ = 0.0;
    }

    SimulationTime get_safe_time() const override {
        return safe_time_;
    }

    void advance_safe_time(SimulationTime new_safe_time) override {
        if (new_safe_time > safe_time_) {
            safe_time_ = new_safe_time;
        }
    }

private:
    Duration buffer_time_;
    UInt32 max_buffered_;
    SimulationTime safe_time_;
    std::vector<TimestampedEvent> buffer_;
};

// ============================================================================
// Distributed Time Manager Implementation
// ============================================================================

class DistributedTimeManagerImpl : public DistributedTimeManager {
public:
    explicit DistributedTimeManagerImpl(const TimeSyncConfig& config)
        : config_(config)
        , initialized_(false)
        , simulation_time_(config.initial_sim_time)
        , real_time_start_(std::chrono::steady_clock::now())
        , next_barrier_id_(1) {
        // Create sub-components
        time_master_ = std::make_unique<SimpleTimeMaster>(
            config.local_node_id,
            config.cluster_nodes,
            config);

        clock_sync_ = std::make_unique<NTPClockSynchronizer>(
            config.local_node_id,
            config.sync_samples);

        causality_mgr_ = std::make_unique<SimpleCausalityManager>(
            config.local_node_id,
            config.enable_vector_clocks);

        event_reorderer_ = std::make_unique<SimpleEventReorderer>(
            config.event_buffer_time,
            config.max_buffered_events);

        // Initialize local node info
        local_node_.id = config.local_node_id;
        local_node_.is_active = true;
        local_node_.raft_state = RaftState::Follower;
        local_node_.last_heartbeat = std::chrono::steady_clock::now();
        nodes_[config.local_node_id] = local_node_;

        // Add cluster nodes
        for (const auto& node_id : config.cluster_nodes) {
            if (node_id != config.local_node_id) {
                TimeSyncNode node;
                node.id = node_id;
                node.is_active = true;
                node.raft_state = RaftState::Follower;
                nodes_[node_id] = node;
            }
        }
    }

    // ========================================================================
    // Lifecycle
    // ========================================================================

    TimeSyncResult initialize() override {
        if (initialized_) {
            return TimeSyncResult::AlreadyInitialized;
        }

        real_time_start_ = std::chrono::steady_clock::now();
        simulation_time_ = config_.initial_sim_time;
        initialized_ = true;

        return TimeSyncResult::Success;
    }

    TimeSyncResult shutdown() override {
        if (!initialized_) {
            return TimeSyncResult::NotInitialized;
        }

        initialized_ = false;
        time_master_->reset();
        event_reorderer_->clear();

        return TimeSyncResult::Success;
    }

    bool is_initialized() const override {
        return initialized_;
    }

    void tick() override {
        if (!initialized_) return;

        time_master_->tick();

        // Update local node Raft state
        local_node_.raft_state = time_master_->get_state();
        nodes_[config_.local_node_id] = local_node_;

        // Process ready events
        auto ready_events = event_reorderer_->get_ready_events();
        for (auto& event : ready_events) {
            if (event_callback_) {
                event_callback_(event);
            }
        }

        // Check for state changes
        auto leader = time_master_->get_leader_id();
        if (election_callback_ && leader != last_known_leader_) {
            election_callback_(time_master_->get_state(), leader);
            last_known_leader_ = leader;
        }
    }

    // ========================================================================
    // Node Management
    // ========================================================================

    const NodeId& get_local_node_id() const override {
        return config_.local_node_id;
    }

    TimeSyncResult add_node(const TimeSyncNode& node) override {
        std::lock_guard<std::mutex> lock(mutex_);
        nodes_[node.id] = node;
        return TimeSyncResult::Success;
    }

    TimeSyncResult remove_node(const NodeId& node_id) override {
        if (node_id == config_.local_node_id) {
            return TimeSyncResult::InvalidNodeId;
        }
        std::lock_guard<std::mutex> lock(mutex_);
        nodes_.erase(node_id);
        return TimeSyncResult::Success;
    }

    std::optional<TimeSyncNode> get_node_info(const NodeId& node_id) const override {
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = nodes_.find(node_id);
        if (it == nodes_.end()) {
            return std::nullopt;
        }
        return it->second;
    }

    std::vector<TimeSyncNode> get_all_nodes() const override {
        std::lock_guard<std::mutex> lock(mutex_);
        std::vector<TimeSyncNode> result;
        result.reserve(nodes_.size());
        for (const auto& [id, node] : nodes_) {
            result.push_back(node);
        }
        return result;
    }

    UInt32 get_active_node_count() const override {
        std::lock_guard<std::mutex> lock(mutex_);
        UInt32 count = 0;
        for (const auto& [id, node] : nodes_) {
            if (node.is_active) count++;
        }
        return count;
    }

    // ========================================================================
    // Leader Election
    // ========================================================================

    RaftState get_raft_state() const override {
        return time_master_->get_state();
    }

    UInt64 get_current_term() const override {
        return time_master_->get_current_term();
    }

    std::optional<NodeId> get_leader_id() const override {
        return time_master_->get_leader_id();
    }

    bool is_leader() const override {
        return time_master_->is_leader();
    }

    TimeSyncResult trigger_election() override {
        auto result = time_master_->start_election();
        stats_.elections_held++;
        return result;
    }

    TimeSyncResult handle_vote_request(const VoteRequest& request, VoteResponse& response) override {
        return time_master_->handle_vote_request(request, response);
    }

    TimeSyncResult handle_append_entries(const AppendEntriesRequest& request,
                                         AppendEntriesResponse& response) override {
        auto result = time_master_->handle_append_request(request, response);

        // Update simulation time from leader
        if (result == TimeSyncResult::Success && !is_leader()) {
            simulation_time_ = request.current_sim_time;
        }

        return result;
    }

    // ========================================================================
    // Clock Synchronization
    // ========================================================================

    TimeSyncResult sync_clock() override {
        auto leader = get_leader_id();
        if (!leader) {
            return TimeSyncResult::LeaderNotElected;
        }
        stats_.sync_requests_sent++;
        return clock_sync_->request_sync(leader.value());
    }

    TimeSyncResult handle_sync_request(const ClockSyncRequest& request,
                                       ClockSyncResponse& response) override {
        return clock_sync_->handle_sync_request(request, response);
    }

    TimeSyncResult handle_sync_response(const ClockSyncResponse& response) override {
        auto result = clock_sync_->handle_sync_response(response);
        if (result == TimeSyncResult::Success) {
            stats_.sync_responses_received++;
            stats_.avg_clock_offset = clock_sync_->get_offset();
            stats_.avg_round_trip_time = clock_sync_->get_round_trip_time();
            if (clock_sync_->get_max_observed_offset() > stats_.max_clock_offset) {
                stats_.max_clock_offset = clock_sync_->get_max_observed_offset();
            }
        }
        return result;
    }

    Duration get_clock_offset() const override {
        return clock_sync_->get_offset();
    }

    WallClockTime get_adjusted_wall_time() const override {
        return clock_sync_->get_adjusted_time();
    }

    // ========================================================================
    // Simulation Time
    // ========================================================================

    SimulationTime get_simulation_time() const override {
        return simulation_time_;
    }

    TimeSyncResult advance_time(SimulationTime delta) override {
        if (!is_leader()) {
            return TimeSyncResult::NotLeader;
        }

        simulation_time_ += delta;
        stats_.current_sim_time = simulation_time_;

        // Update real-time ratio
        auto now = std::chrono::steady_clock::now();
        auto real_elapsed = std::chrono::duration<Float64>(now - real_time_start_).count();
        if (real_elapsed > 0) {
            stats_.real_time_ratio = simulation_time_ / real_elapsed;
        }

        // Notify via callback
        if (time_advance_callback_) {
            time_advance_callback_(simulation_time_);
        }

        // Advance safe time for event ordering
        event_reorderer_->advance_safe_time(simulation_time_ - config_.max_lookahead);

        // Create log entry for replication
        RaftLogEntry entry;
        entry.sim_time = simulation_time_;
        entry.logical_time = causality_mgr_->increment_and_get();
        entry.command = "advance_time";
        time_master_->append_entry(entry);

        return TimeSyncResult::Success;
    }

    TimeSyncResult set_simulation_time(SimulationTime time) override {
        if (!is_leader()) {
            return TimeSyncResult::NotLeader;
        }

        simulation_time_ = time;
        stats_.current_sim_time = simulation_time_;
        real_time_start_ = std::chrono::steady_clock::now();

        if (time_advance_callback_) {
            time_advance_callback_(simulation_time_);
        }

        return TimeSyncResult::Success;
    }

    Float64 get_real_time_ratio() const override {
        return stats_.real_time_ratio;
    }

    // ========================================================================
    // Barrier Synchronization
    // ========================================================================

    UInt64 create_barrier(SimulationTime target_time) override {
        std::lock_guard<std::mutex> lock(mutex_);

        BarrierState barrier;
        barrier.barrier_id = next_barrier_id_++;
        barrier.target_time = target_time;
        barrier.created_at = std::chrono::steady_clock::now();
        barrier.timeout = config_.barrier_timeout;
        barrier.completed = false;

        // All active nodes expected
        for (const auto& [id, node] : nodes_) {
            if (node.is_active) {
                barrier.expected_nodes.insert(id);
            }
        }

        barriers_[barrier.barrier_id] = barrier;
        return barrier.barrier_id;
    }

    TimeSyncResult arrive_at_barrier(UInt64 barrier_id) override {
        std::lock_guard<std::mutex> lock(mutex_);

        auto it = barriers_.find(barrier_id);
        if (it == barriers_.end()) {
            return TimeSyncResult::InvalidTimestamp;
        }

        it->second.arrived_nodes.insert(config_.local_node_id);

        // Check if barrier is complete
        if (it->second.arrived_nodes.size() == it->second.expected_nodes.size()) {
            it->second.completed = true;
            stats_.barriers_completed++;
            if (barrier_callback_) {
                barrier_callback_(barrier_id, true);
            }
        }

        return TimeSyncResult::Success;
    }

    TimeSyncResult wait_for_barrier(UInt64 barrier_id, Duration timeout) override {
        auto start = std::chrono::steady_clock::now();

        while (true) {
            {
                std::lock_guard<std::mutex> lock(mutex_);
                auto it = barriers_.find(barrier_id);
                if (it == barriers_.end()) {
                    return TimeSyncResult::InvalidTimestamp;
                }
                if (it->second.completed) {
                    return TimeSyncResult::Success;
                }
            }

            auto elapsed = std::chrono::steady_clock::now() - start;
            if (elapsed >= timeout) {
                stats_.barriers_timed_out++;
                if (barrier_callback_) {
                    barrier_callback_(barrier_id, false);
                }
                return TimeSyncResult::BarrierTimeout;
            }

            // Small sleep to avoid busy waiting
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    bool is_barrier_complete(UInt64 barrier_id) const override {
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = barriers_.find(barrier_id);
        if (it == barriers_.end()) {
            return false;
        }
        return it->second.completed;
    }

    // ========================================================================
    // Causality Management
    // ========================================================================

    LogicalTime get_logical_time() const override {
        return causality_mgr_->get_logical_time();
    }

    LogicalTime next_logical_time() override {
        auto time = causality_mgr_->increment_and_get();
        stats_.current_logical_time = time;
        return time;
    }

    void update_logical_time(LogicalTime received_time) override {
        causality_mgr_->update_on_receive(received_time);
        stats_.current_logical_time = causality_mgr_->get_logical_time();
    }

    VectorClock get_vector_clock() const override {
        return causality_mgr_->get_vector_clock();
    }

    // ========================================================================
    // Event Ordering
    // ========================================================================

    TimestampedEvent create_event(const std::string& event_type,
                                  std::vector<UInt8> payload) override {
        return causality_mgr_->create_event(event_type, simulation_time_, std::move(payload));
    }

    TimeSyncResult submit_event(TimestampedEvent event) override {
        // Update logical time from event
        causality_mgr_->update_on_receive(event.logical_time);

        auto result = event_reorderer_->submit_event(std::move(event));
        if (result != TimeSyncResult::Success) {
            stats_.causality_violations++;
        }
        return result;
    }

    std::vector<TimestampedEvent> get_ready_events() override {
        auto events = event_reorderer_->get_ready_events();
        if (!events.empty()) {
            stats_.events_reordered += events.size();
        }
        return events;
    }

    SimulationTime get_safe_time() const override {
        return event_reorderer_->get_safe_time();
    }

    // ========================================================================
    // Callbacks
    // ========================================================================

    void set_election_callback(ElectionCallback callback) override {
        election_callback_ = std::move(callback);
    }

    void set_time_advance_callback(TimeAdvanceCallback callback) override {
        time_advance_callback_ = std::move(callback);
    }

    void set_barrier_callback(BarrierCallback callback) override {
        barrier_callback_ = std::move(callback);
    }

    void set_event_callback(EventCallback callback) override {
        event_callback_ = std::move(callback);
    }

    // ========================================================================
    // Statistics
    // ========================================================================

    TimeSyncStats get_statistics() const override {
        return stats_;
    }

    void reset_statistics() override {
        stats_ = TimeSyncStats{};
        stats_.current_sim_time = simulation_time_;
        stats_.current_logical_time = causality_mgr_->get_logical_time();
    }

    // ========================================================================
    // Configuration
    // ========================================================================

    const TimeSyncConfig& get_config() const override {
        return config_;
    }

    TimeSyncResult set_advancement_mode(TimeAdvancementMode mode) override {
        config_.advancement_mode = mode;
        return TimeSyncResult::Success;
    }

    TimeSyncResult set_sync_interval(Duration interval) override {
        config_.sync_interval = interval;
        return TimeSyncResult::Success;
    }

private:
    TimeSyncConfig config_;
    bool initialized_;

    std::unique_ptr<ITimeMaster> time_master_;
    std::unique_ptr<IClockSynchronizer> clock_sync_;
    std::unique_ptr<ICausalityManager> causality_mgr_;
    std::unique_ptr<IEventReorderer> event_reorderer_;

    TimeSyncNode local_node_;
    std::unordered_map<NodeId, TimeSyncNode> nodes_;
    std::unordered_map<UInt64, BarrierState> barriers_;

    SimulationTime simulation_time_;
    WallClockTime real_time_start_;
    UInt64 next_barrier_id_;

    std::optional<NodeId> last_known_leader_;

    TimeSyncStats stats_;

    ElectionCallback election_callback_;
    TimeAdvanceCallback time_advance_callback_;
    BarrierCallback barrier_callback_;
    EventCallback event_callback_;

    mutable std::mutex mutex_;
};

// ============================================================================
// Factory Functions
// ============================================================================

std::unique_ptr<DistributedTimeManager> create_distributed_time_manager(const TimeSyncConfig& config) {
    return std::make_unique<DistributedTimeManagerImpl>(config);
}

std::unique_ptr<ITimeMaster> create_time_master(const NodeId& local_node_id,
                                                 const std::vector<NodeId>& cluster_nodes,
                                                 const TimeSyncConfig& config) {
    return std::make_unique<SimpleTimeMaster>(local_node_id, cluster_nodes, config);
}

std::unique_ptr<IClockSynchronizer> create_clock_synchronizer(const NodeId& local_node_id,
                                                               ClockSyncAlgorithm /*algorithm*/) {
    // Currently only NTP-like algorithm is implemented
    return std::make_unique<NTPClockSynchronizer>(local_node_id);
}

std::unique_ptr<ICausalityManager> create_causality_manager(const NodeId& local_node_id,
                                                             bool enable_vector_clocks) {
    return std::make_unique<SimpleCausalityManager>(local_node_id, enable_vector_clocks);
}

std::unique_ptr<IEventReorderer> create_event_reorderer(Duration buffer_time,
                                                         UInt32 max_buffered) {
    return std::make_unique<SimpleEventReorderer>(buffer_time, max_buffered);
}

} // namespace jaguar::cloud
