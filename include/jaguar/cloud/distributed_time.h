#pragma once
/**
 * @file distributed_time.h
 * @brief Distributed time synchronization for multi-node simulation
 *
 * This file provides time synchronization infrastructure for distributed
 * simulations running across multiple compute nodes. Implements consensus-based
 * time master election, clock synchronization, and causality tracking.
 *
 * Key features:
 * - Raft-based time master election for fault tolerance
 * - NTP-like clock synchronization protocol
 * - Lamport logical clocks for causality ordering
 * - Simulation time advancement with barrier synchronization
 * - Out-of-order event handling with reordering
 */

#include "jaguar/core/types.h"
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
#include <queue>
#include <deque>
#include <condition_variable>

namespace jaguar::cloud {

// ============================================================================
// Forward Declarations
// ============================================================================

class ITimeMaster;
class IClockSynchronizer;
class ICausalityManager;
class IEventReorderer;
class DistributedTimeManager;

// ============================================================================
// Type Aliases
// ============================================================================

using NodeId = std::string;
using LogicalTime = UInt64;
using SimulationTime = Float64;
using WallClockTime = std::chrono::steady_clock::time_point;
using Duration = std::chrono::nanoseconds;

// ============================================================================
// Time Sync Result Enum
// ============================================================================

/**
 * @brief Result codes for time synchronization operations
 */
enum class TimeSyncResult : UInt8 {
    Success = 0,

    // Configuration errors
    InvalidConfiguration,
    InvalidNodeId,
    InvalidTimestamp,

    // Election errors
    ElectionInProgress,
    ElectionFailed,
    NoQuorum,
    NotLeader,
    LeaderNotElected,

    // Synchronization errors
    SyncFailed,
    ClockDriftTooHigh,
    TimeoutExpired,
    BarrierTimeout,

    // Causality errors
    CausalityViolation,
    EventOrderingFailed,
    StaleEvent,

    // State errors
    NotInitialized,
    AlreadyInitialized,
    ShuttingDown,

    // Network errors
    NetworkError,
    NodeUnreachable,
    ConnectionLost,
    MessageDropped
};

/**
 * @brief Convert TimeSyncResult to string
 */
inline const char* time_sync_result_to_string(TimeSyncResult result) {
    switch (result) {
        case TimeSyncResult::Success: return "Success";
        case TimeSyncResult::InvalidConfiguration: return "InvalidConfiguration";
        case TimeSyncResult::InvalidNodeId: return "InvalidNodeId";
        case TimeSyncResult::InvalidTimestamp: return "InvalidTimestamp";
        case TimeSyncResult::ElectionInProgress: return "ElectionInProgress";
        case TimeSyncResult::ElectionFailed: return "ElectionFailed";
        case TimeSyncResult::NoQuorum: return "NoQuorum";
        case TimeSyncResult::NotLeader: return "NotLeader";
        case TimeSyncResult::LeaderNotElected: return "LeaderNotElected";
        case TimeSyncResult::SyncFailed: return "SyncFailed";
        case TimeSyncResult::ClockDriftTooHigh: return "ClockDriftTooHigh";
        case TimeSyncResult::TimeoutExpired: return "TimeoutExpired";
        case TimeSyncResult::BarrierTimeout: return "BarrierTimeout";
        case TimeSyncResult::CausalityViolation: return "CausalityViolation";
        case TimeSyncResult::EventOrderingFailed: return "EventOrderingFailed";
        case TimeSyncResult::StaleEvent: return "StaleEvent";
        case TimeSyncResult::NotInitialized: return "NotInitialized";
        case TimeSyncResult::AlreadyInitialized: return "AlreadyInitialized";
        case TimeSyncResult::ShuttingDown: return "ShuttingDown";
        case TimeSyncResult::NetworkError: return "NetworkError";
        case TimeSyncResult::NodeUnreachable: return "NodeUnreachable";
        case TimeSyncResult::ConnectionLost: return "ConnectionLost";
        case TimeSyncResult::MessageDropped: return "MessageDropped";
        default: return "Unknown";
    }
}

// ============================================================================
// Raft State Enum
// ============================================================================

/**
 * @brief Raft consensus node states
 */
enum class RaftState : UInt8 {
    Follower,   ///< Following the current leader
    Candidate,  ///< Running for leader election
    Leader      ///< Current cluster leader (time master)
};

/**
 * @brief Convert RaftState to string
 */
inline const char* raft_state_to_string(RaftState state) {
    switch (state) {
        case RaftState::Follower: return "Follower";
        case RaftState::Candidate: return "Candidate";
        case RaftState::Leader: return "Leader";
        default: return "Unknown";
    }
}

// ============================================================================
// Time Advancement Mode
// ============================================================================

/**
 * @brief How simulation time advances across nodes
 */
enum class TimeAdvancementMode : UInt8 {
    Conservative,   ///< Wait for all nodes before advancing (slowest but safest)
    Optimistic,     ///< Advance optimistically with rollback on conflict
    Bounded,        ///< Advance with bounded lookahead
    Adaptive        ///< Automatically adjust based on network conditions
};

/**
 * @brief Convert TimeAdvancementMode to string
 */
inline const char* time_advancement_mode_to_string(TimeAdvancementMode mode) {
    switch (mode) {
        case TimeAdvancementMode::Conservative: return "Conservative";
        case TimeAdvancementMode::Optimistic: return "Optimistic";
        case TimeAdvancementMode::Bounded: return "Bounded";
        case TimeAdvancementMode::Adaptive: return "Adaptive";
        default: return "Unknown";
    }
}

// ============================================================================
// Clock Sync Algorithm
// ============================================================================

/**
 * @brief Clock synchronization algorithms
 */
enum class ClockSyncAlgorithm : UInt8 {
    NTP,            ///< Network Time Protocol-like algorithm
    PTP,            ///< Precision Time Protocol (IEEE 1588)
    Cristian,       ///< Cristian's algorithm
    Berkeley,       ///< Berkeley algorithm (averaging)
    Custom          ///< User-defined synchronization
};

/**
 * @brief Convert ClockSyncAlgorithm to string
 */
inline const char* clock_sync_algorithm_to_string(ClockSyncAlgorithm algo) {
    switch (algo) {
        case ClockSyncAlgorithm::NTP: return "NTP";
        case ClockSyncAlgorithm::PTP: return "PTP";
        case ClockSyncAlgorithm::Cristian: return "Cristian";
        case ClockSyncAlgorithm::Berkeley: return "Berkeley";
        case ClockSyncAlgorithm::Custom: return "Custom";
        default: return "Unknown";
    }
}

// ============================================================================
// Structs
// ============================================================================

/**
 * @brief Information about a time sync node
 */
struct TimeSyncNode {
    NodeId id;                          ///< Unique node identifier
    std::string hostname;               ///< Node hostname
    UInt16 port{0};                     ///< Sync service port
    bool is_active{false};              ///< Whether node is active
    RaftState raft_state{RaftState::Follower};
    UInt64 current_term{0};             ///< Raft term number
    WallClockTime last_heartbeat;       ///< Last heartbeat received
    Duration clock_offset{0};           ///< Estimated clock offset from master
    Duration round_trip_time{0};        ///< Network round-trip time
    Float64 clock_drift_rate{0.0};      ///< Clock drift rate (ppm)
    UInt32 sync_failures{0};            ///< Consecutive sync failures
};

/**
 * @brief Raft log entry for consensus
 */
struct RaftLogEntry {
    UInt64 term{0};                     ///< Term when entry was created
    UInt64 index{0};                    ///< Log index
    SimulationTime sim_time{0.0};       ///< Simulation time command
    LogicalTime logical_time{0};        ///< Logical timestamp
    std::string command;                ///< Command type (e.g., "advance", "barrier")
    std::vector<UInt8> data;            ///< Command data
};

/**
 * @brief Vote request for Raft election
 */
struct VoteRequest {
    UInt64 term{0};                     ///< Candidate's term
    NodeId candidate_id;                ///< Candidate requesting vote
    UInt64 last_log_index{0};           ///< Index of candidate's last log entry
    UInt64 last_log_term{0};            ///< Term of candidate's last log entry
};

/**
 * @brief Vote response for Raft election
 */
struct VoteResponse {
    UInt64 term{0};                     ///< Current term (for candidate update)
    bool vote_granted{false};           ///< True if vote was granted
    NodeId voter_id;                    ///< ID of the voting node
};

/**
 * @brief Append entries request (Raft heartbeat/replication)
 */
struct AppendEntriesRequest {
    UInt64 term{0};                     ///< Leader's term
    NodeId leader_id;                   ///< Leader's ID
    UInt64 prev_log_index{0};           ///< Index of log entry before new ones
    UInt64 prev_log_term{0};            ///< Term of prev_log_index entry
    std::vector<RaftLogEntry> entries;  ///< Log entries to replicate
    UInt64 leader_commit{0};            ///< Leader's commit index
    SimulationTime current_sim_time{0.0}; ///< Current simulation time
};

/**
 * @brief Append entries response
 */
struct AppendEntriesResponse {
    UInt64 term{0};                     ///< Current term
    bool success{false};                ///< True if successful
    NodeId follower_id;                 ///< Responding follower's ID
    UInt64 match_index{0};              ///< Highest log entry known to be replicated
};

/**
 * @brief Clock synchronization request
 */
struct ClockSyncRequest {
    NodeId sender_id;                   ///< Request sender
    WallClockTime t1;                   ///< Client send time
    UInt64 sequence{0};                 ///< Request sequence number
};

/**
 * @brief Clock synchronization response
 */
struct ClockSyncResponse {
    NodeId responder_id;                ///< Response sender (master)
    WallClockTime t1;                   ///< Original client send time
    WallClockTime t2;                   ///< Server receive time
    WallClockTime t3;                   ///< Server send time
    UInt64 sequence{0};                 ///< Request sequence number
};

/**
 * @brief Vector clock for causality tracking
 */
struct VectorClock {
    std::unordered_map<NodeId, LogicalTime> clocks;

    /**
     * @brief Increment clock for a node
     */
    void increment(const NodeId& node_id) {
        clocks[node_id]++;
    }

    /**
     * @brief Get clock value for a node
     */
    LogicalTime get(const NodeId& node_id) const {
        auto it = clocks.find(node_id);
        return it != clocks.end() ? it->second : 0;
    }

    /**
     * @brief Set clock value for a node
     */
    void set(const NodeId& node_id, LogicalTime time) {
        clocks[node_id] = time;
    }

    /**
     * @brief Merge with another vector clock (take max)
     */
    void merge(const VectorClock& other) {
        for (const auto& [node_id, time] : other.clocks) {
            clocks[node_id] = std::max(clocks[node_id], time);
        }
    }

    /**
     * @brief Check if this clock happens-before another
     */
    bool happens_before(const VectorClock& other) const {
        bool at_least_one_less = false;
        for (const auto& [node_id, time] : clocks) {
            LogicalTime other_time = other.get(node_id);
            if (time > other_time) return false;
            if (time < other_time) at_least_one_less = true;
        }
        // Check nodes only in other
        for (const auto& [node_id, time] : other.clocks) {
            if (clocks.find(node_id) == clocks.end() && time > 0) {
                at_least_one_less = true;
            }
        }
        return at_least_one_less;
    }

    /**
     * @brief Check if clocks are concurrent (neither happens-before)
     */
    bool concurrent_with(const VectorClock& other) const {
        return !happens_before(other) && !other.happens_before(*this);
    }
};

/**
 * @brief Timestamped event for ordering
 */
struct TimestampedEvent {
    UInt64 event_id{0};                 ///< Unique event identifier
    NodeId source_node;                 ///< Node that generated the event
    SimulationTime sim_time{0.0};       ///< Simulation timestamp
    LogicalTime logical_time{0};        ///< Lamport logical timestamp
    VectorClock vector_clock;           ///< Full vector clock (optional)
    WallClockTime wall_time;            ///< Wall clock time when created
    std::string event_type;             ///< Event type identifier
    std::vector<UInt8> payload;         ///< Event data
    bool processed{false};              ///< Whether event has been processed
};

/**
 * @brief Barrier synchronization state
 */
struct BarrierState {
    UInt64 barrier_id{0};               ///< Unique barrier identifier
    SimulationTime target_time{0.0};    ///< Target simulation time
    std::unordered_set<NodeId> arrived_nodes;  ///< Nodes that reached barrier
    std::unordered_set<NodeId> expected_nodes; ///< All nodes expected
    WallClockTime created_at;           ///< When barrier was created
    Duration timeout{std::chrono::seconds(30)}; ///< Barrier timeout
    bool completed{false};              ///< Whether barrier is complete
};

/**
 * @brief Time synchronization statistics
 */
struct TimeSyncStats {
    // Election stats
    UInt32 elections_held{0};           ///< Total elections
    UInt32 terms_served_as_leader{0};   ///< Times this node was leader
    Duration avg_election_time{0};      ///< Average election duration

    // Clock sync stats
    UInt64 sync_requests_sent{0};       ///< Total sync requests
    UInt64 sync_responses_received{0};  ///< Total sync responses
    Duration avg_clock_offset{0};       ///< Average clock offset
    Duration max_clock_offset{0};       ///< Maximum clock offset seen
    Duration avg_round_trip_time{0};    ///< Average RTT

    // Simulation time stats
    SimulationTime current_sim_time{0.0}; ///< Current simulation time
    Float64 real_time_ratio{1.0};       ///< Sim time / real time
    UInt64 barriers_completed{0};       ///< Successful barriers
    UInt64 barriers_timed_out{0};       ///< Timed out barriers

    // Causality stats
    LogicalTime current_logical_time{0}; ///< Current Lamport clock
    UInt64 events_reordered{0};         ///< Events that needed reordering
    UInt64 causality_violations{0};     ///< Detected causality violations

    // Error stats
    UInt64 network_errors{0};           ///< Network-related errors
    UInt64 timeout_errors{0};           ///< Timeout errors
    UInt64 sync_failures{0};            ///< Failed synchronizations
};

// ============================================================================
// Configuration
// ============================================================================

/**
 * @brief Configuration for distributed time synchronization
 */
struct TimeSyncConfig {
    // Node identity
    NodeId local_node_id;               ///< This node's unique ID
    std::vector<NodeId> cluster_nodes;  ///< All nodes in the cluster

    // Time advancement
    TimeAdvancementMode advancement_mode{TimeAdvancementMode::Conservative};
    SimulationTime initial_sim_time{0.0};       ///< Starting simulation time
    SimulationTime time_step{0.001};            ///< Default time step (1ms)
    SimulationTime max_lookahead{0.1};          ///< Maximum lookahead for optimistic

    // Clock synchronization
    ClockSyncAlgorithm sync_algorithm{ClockSyncAlgorithm::NTP};
    Duration sync_interval{std::chrono::milliseconds(100)}; ///< How often to sync
    Duration max_clock_drift{std::chrono::milliseconds(10)}; ///< Max allowed drift
    UInt32 sync_samples{8};                     ///< Samples for offset estimation

    // Raft consensus
    Duration election_timeout_min{std::chrono::milliseconds(150)};
    Duration election_timeout_max{std::chrono::milliseconds(300)};
    Duration heartbeat_interval{std::chrono::milliseconds(50)};
    UInt32 max_entries_per_append{100};         ///< Max log entries per message

    // Barrier synchronization
    Duration barrier_timeout{std::chrono::seconds(30)};
    bool auto_barrier_on_step{true};            ///< Auto-barrier each time step

    // Event ordering
    bool enable_vector_clocks{false};           ///< Use full vector clocks
    Duration event_buffer_time{std::chrono::milliseconds(50)}; ///< Buffer for reordering
    UInt32 max_buffered_events{10000};          ///< Max events to buffer

    // Network
    Duration network_timeout{std::chrono::seconds(5)};
    UInt32 max_retry_attempts{3};
    Duration retry_backoff{std::chrono::milliseconds(100)};

    // Fault tolerance
    UInt32 min_quorum_size{0};                  ///< 0 = auto (n/2 + 1)
    Duration node_timeout{std::chrono::seconds(10)}; ///< Consider node dead after
    bool auto_remove_dead_nodes{false};         ///< Auto-remove unresponsive nodes

    /**
     * @brief Create default configuration
     */
    static TimeSyncConfig default_config() noexcept {
        return TimeSyncConfig{};
    }

    /**
     * @brief Create configuration for low-latency simulation
     */
    static TimeSyncConfig low_latency() noexcept {
        TimeSyncConfig config;
        config.advancement_mode = TimeAdvancementMode::Optimistic;
        config.sync_interval = std::chrono::milliseconds(50);
        config.heartbeat_interval = std::chrono::milliseconds(25);
        config.event_buffer_time = std::chrono::milliseconds(20);
        config.barrier_timeout = std::chrono::seconds(10);
        return config;
    }

    /**
     * @brief Create configuration for high-fidelity simulation
     */
    static TimeSyncConfig high_fidelity() noexcept {
        TimeSyncConfig config;
        config.advancement_mode = TimeAdvancementMode::Conservative;
        config.sync_interval = std::chrono::milliseconds(10);
        config.max_clock_drift = std::chrono::microseconds(100);
        config.sync_samples = 16;
        config.enable_vector_clocks = true;
        return config;
    }

    /**
     * @brief Create configuration for large cluster
     */
    static TimeSyncConfig large_cluster() noexcept {
        TimeSyncConfig config;
        config.advancement_mode = TimeAdvancementMode::Bounded;
        config.max_lookahead = 0.05;  // 50ms
        config.heartbeat_interval = std::chrono::milliseconds(100);
        config.max_entries_per_append = 50;
        config.max_buffered_events = 50000;
        config.auto_remove_dead_nodes = true;
        return config;
    }
};

// ============================================================================
// Interfaces
// ============================================================================

/**
 * @brief Interface for time master election (Raft consensus)
 */
class ITimeMaster {
public:
    virtual ~ITimeMaster() = default;

    // State queries
    virtual RaftState get_state() const = 0;
    virtual UInt64 get_current_term() const = 0;
    virtual std::optional<NodeId> get_leader_id() const = 0;
    virtual bool is_leader() const = 0;

    // Election
    virtual TimeSyncResult start_election() = 0;
    virtual TimeSyncResult handle_vote_request(const VoteRequest& request, VoteResponse& response) = 0;
    virtual TimeSyncResult handle_vote_response(const VoteResponse& response) = 0;

    // Replication
    virtual TimeSyncResult append_entry(const RaftLogEntry& entry) = 0;
    virtual TimeSyncResult handle_append_request(const AppendEntriesRequest& request, AppendEntriesResponse& response) = 0;
    virtual TimeSyncResult handle_append_response(const AppendEntriesResponse& response) = 0;

    // Log management
    virtual UInt64 get_commit_index() const = 0;
    virtual UInt64 get_last_log_index() const = 0;
    virtual std::optional<RaftLogEntry> get_log_entry(UInt64 index) const = 0;

    // Lifecycle
    virtual void tick() = 0;  // Called periodically for timeouts
    virtual void reset() = 0;
};

/**
 * @brief Interface for clock synchronization
 */
class IClockSynchronizer {
public:
    virtual ~IClockSynchronizer() = default;

    // Synchronization
    virtual TimeSyncResult request_sync(const NodeId& master_id) = 0;
    virtual TimeSyncResult handle_sync_request(const ClockSyncRequest& request, ClockSyncResponse& response) = 0;
    virtual TimeSyncResult handle_sync_response(const ClockSyncResponse& response) = 0;

    // Clock queries
    virtual Duration get_offset() const = 0;
    virtual Duration get_round_trip_time() const = 0;
    virtual Float64 get_drift_rate() const = 0;

    // Adjusted time
    virtual WallClockTime get_adjusted_time() const = 0;
    virtual Duration adjust_duration(Duration local_duration) const = 0;

    // Statistics
    virtual UInt64 get_sync_count() const = 0;
    virtual Duration get_max_observed_offset() const = 0;
};

/**
 * @brief Interface for causality management
 */
class ICausalityManager {
public:
    virtual ~ICausalityManager() = default;

    // Lamport clock operations
    virtual LogicalTime get_logical_time() const = 0;
    virtual LogicalTime increment_and_get() = 0;
    virtual void update_on_receive(LogicalTime received_time) = 0;

    // Vector clock operations (if enabled)
    virtual VectorClock get_vector_clock() const = 0;
    virtual void increment_vector_clock() = 0;
    virtual void merge_vector_clock(const VectorClock& other) = 0;

    // Event timestamp creation
    virtual TimestampedEvent create_event(const std::string& event_type,
                                          SimulationTime sim_time,
                                          std::vector<UInt8> payload = {}) = 0;

    // Causality checks
    virtual bool happens_before(const TimestampedEvent& a, const TimestampedEvent& b) const = 0;
    virtual bool are_concurrent(const TimestampedEvent& a, const TimestampedEvent& b) const = 0;
};

/**
 * @brief Interface for out-of-order event handling
 */
class IEventReorderer {
public:
    virtual ~IEventReorderer() = default;

    // Event submission
    virtual TimeSyncResult submit_event(TimestampedEvent event) = 0;
    virtual TimeSyncResult submit_events(std::vector<TimestampedEvent> events) = 0;

    // Event retrieval (in causal order)
    virtual std::vector<TimestampedEvent> get_ready_events() = 0;
    virtual std::optional<TimestampedEvent> pop_next_event() = 0;

    // Buffer management
    virtual UInt32 get_buffered_count() const = 0;
    virtual void flush() = 0;
    virtual void clear() = 0;

    // Safe time (all events before this are ready)
    virtual SimulationTime get_safe_time() const = 0;
    virtual void advance_safe_time(SimulationTime new_safe_time) = 0;
};

/**
 * @brief Callback types for distributed time events
 */
using ElectionCallback = std::function<void(RaftState new_state, const std::optional<NodeId>& leader_id)>;
using TimeAdvanceCallback = std::function<void(SimulationTime new_time)>;
using BarrierCallback = std::function<void(UInt64 barrier_id, bool success)>;
using EventCallback = std::function<void(const TimestampedEvent& event)>;

// ============================================================================
// Main Distributed Time Manager
// ============================================================================

/**
 * @brief Central manager for distributed time synchronization
 *
 * Coordinates time master election, clock synchronization, simulation time
 * advancement, and causal event ordering across a distributed simulation.
 *
 * Example usage:
 * @code
 * TimeSyncConfig config;
 * config.local_node_id = "node-1";
 * config.cluster_nodes = {"node-1", "node-2", "node-3"};
 *
 * auto manager = create_distributed_time_manager(config);
 * manager->initialize();
 *
 * // Wait for leader election
 * while (!manager->get_leader_id()) {
 *     manager->tick();
 *     std::this_thread::sleep_for(std::chrono::milliseconds(10));
 * }
 *
 * // Advance simulation time
 * if (manager->is_leader()) {
 *     manager->advance_time(0.001);  // 1ms
 * }
 * @endcode
 */
class DistributedTimeManager {
public:
    virtual ~DistributedTimeManager() = default;

    // ========================================================================
    // Lifecycle
    // ========================================================================

    /**
     * @brief Initialize the time manager
     * @return Success or error code
     */
    virtual TimeSyncResult initialize() = 0;

    /**
     * @brief Shutdown the time manager
     * @return Success or error code
     */
    virtual TimeSyncResult shutdown() = 0;

    /**
     * @brief Check if initialized
     */
    virtual bool is_initialized() const = 0;

    /**
     * @brief Periodic tick for processing timeouts and messages
     */
    virtual void tick() = 0;

    // ========================================================================
    // Node Management
    // ========================================================================

    /**
     * @brief Get local node ID
     */
    virtual const NodeId& get_local_node_id() const = 0;

    /**
     * @brief Add a node to the cluster
     */
    virtual TimeSyncResult add_node(const TimeSyncNode& node) = 0;

    /**
     * @brief Remove a node from the cluster
     */
    virtual TimeSyncResult remove_node(const NodeId& node_id) = 0;

    /**
     * @brief Get information about a node
     */
    virtual std::optional<TimeSyncNode> get_node_info(const NodeId& node_id) const = 0;

    /**
     * @brief Get all nodes in the cluster
     */
    virtual std::vector<TimeSyncNode> get_all_nodes() const = 0;

    /**
     * @brief Get count of active nodes
     */
    virtual UInt32 get_active_node_count() const = 0;

    // ========================================================================
    // Leader Election (Raft)
    // ========================================================================

    /**
     * @brief Get current Raft state
     */
    virtual RaftState get_raft_state() const = 0;

    /**
     * @brief Get current Raft term
     */
    virtual UInt64 get_current_term() const = 0;

    /**
     * @brief Get current leader ID (if elected)
     */
    virtual std::optional<NodeId> get_leader_id() const = 0;

    /**
     * @brief Check if this node is the leader
     */
    virtual bool is_leader() const = 0;

    /**
     * @brief Trigger a leader election
     */
    virtual TimeSyncResult trigger_election() = 0;

    /**
     * @brief Handle incoming vote request
     */
    virtual TimeSyncResult handle_vote_request(const VoteRequest& request, VoteResponse& response) = 0;

    /**
     * @brief Handle incoming append entries request
     */
    virtual TimeSyncResult handle_append_entries(const AppendEntriesRequest& request, AppendEntriesResponse& response) = 0;

    // ========================================================================
    // Clock Synchronization
    // ========================================================================

    /**
     * @brief Request clock synchronization with the master
     */
    virtual TimeSyncResult sync_clock() = 0;

    /**
     * @brief Handle incoming sync request (as master)
     */
    virtual TimeSyncResult handle_sync_request(const ClockSyncRequest& request, ClockSyncResponse& response) = 0;

    /**
     * @brief Handle incoming sync response
     */
    virtual TimeSyncResult handle_sync_response(const ClockSyncResponse& response) = 0;

    /**
     * @brief Get estimated clock offset from master
     */
    virtual Duration get_clock_offset() const = 0;

    /**
     * @brief Get adjusted wall clock time
     */
    virtual WallClockTime get_adjusted_wall_time() const = 0;

    // ========================================================================
    // Simulation Time
    // ========================================================================

    /**
     * @brief Get current simulation time
     */
    virtual SimulationTime get_simulation_time() const = 0;

    /**
     * @brief Advance simulation time (leader only)
     * @param delta Time increment
     * @return Success or error code
     */
    virtual TimeSyncResult advance_time(SimulationTime delta) = 0;

    /**
     * @brief Set simulation time (leader only, for initialization)
     * @param time New simulation time
     * @return Success or error code
     */
    virtual TimeSyncResult set_simulation_time(SimulationTime time) = 0;

    /**
     * @brief Get real-time ratio (simulation time / wall time)
     */
    virtual Float64 get_real_time_ratio() const = 0;

    // ========================================================================
    // Barrier Synchronization
    // ========================================================================

    /**
     * @brief Create a barrier at a simulation time
     * @param target_time Simulation time for barrier
     * @return Barrier ID or 0 on failure
     */
    virtual UInt64 create_barrier(SimulationTime target_time) = 0;

    /**
     * @brief Signal arrival at a barrier
     * @param barrier_id Barrier to signal
     * @return Success or error code
     */
    virtual TimeSyncResult arrive_at_barrier(UInt64 barrier_id) = 0;

    /**
     * @brief Wait for barrier completion
     * @param barrier_id Barrier to wait for
     * @param timeout Maximum wait time
     * @return Success or timeout
     */
    virtual TimeSyncResult wait_for_barrier(UInt64 barrier_id, Duration timeout) = 0;

    /**
     * @brief Check if barrier is complete
     */
    virtual bool is_barrier_complete(UInt64 barrier_id) const = 0;

    // ========================================================================
    // Causality Management
    // ========================================================================

    /**
     * @brief Get current Lamport logical time
     */
    virtual LogicalTime get_logical_time() const = 0;

    /**
     * @brief Increment and get next logical time
     */
    virtual LogicalTime next_logical_time() = 0;

    /**
     * @brief Update logical time on message receive
     */
    virtual void update_logical_time(LogicalTime received_time) = 0;

    /**
     * @brief Get current vector clock (if enabled)
     */
    virtual VectorClock get_vector_clock() const = 0;

    // ========================================================================
    // Event Ordering
    // ========================================================================

    /**
     * @brief Create a timestamped event
     */
    virtual TimestampedEvent create_event(const std::string& event_type,
                                          std::vector<UInt8> payload = {}) = 0;

    /**
     * @brief Submit an event for ordering
     */
    virtual TimeSyncResult submit_event(TimestampedEvent event) = 0;

    /**
     * @brief Get events ready for processing (in causal order)
     */
    virtual std::vector<TimestampedEvent> get_ready_events() = 0;

    /**
     * @brief Get the safe time (all events before this are ready)
     */
    virtual SimulationTime get_safe_time() const = 0;

    // ========================================================================
    // Callbacks
    // ========================================================================

    /**
     * @brief Set callback for election state changes
     */
    virtual void set_election_callback(ElectionCallback callback) = 0;

    /**
     * @brief Set callback for simulation time advances
     */
    virtual void set_time_advance_callback(TimeAdvanceCallback callback) = 0;

    /**
     * @brief Set callback for barrier completions
     */
    virtual void set_barrier_callback(BarrierCallback callback) = 0;

    /**
     * @brief Set callback for ready events
     */
    virtual void set_event_callback(EventCallback callback) = 0;

    // ========================================================================
    // Statistics
    // ========================================================================

    /**
     * @brief Get synchronization statistics
     */
    virtual TimeSyncStats get_statistics() const = 0;

    /**
     * @brief Reset statistics
     */
    virtual void reset_statistics() = 0;

    // ========================================================================
    // Configuration
    // ========================================================================

    /**
     * @brief Get current configuration
     */
    virtual const TimeSyncConfig& get_config() const = 0;

    /**
     * @brief Update time advancement mode
     */
    virtual TimeSyncResult set_advancement_mode(TimeAdvancementMode mode) = 0;

    /**
     * @brief Update sync interval
     */
    virtual TimeSyncResult set_sync_interval(Duration interval) = 0;
};

// ============================================================================
// Factory Functions
// ============================================================================

/**
 * @brief Create a distributed time manager
 * @param config Configuration for time synchronization
 * @return Unique pointer to the manager
 */
std::unique_ptr<DistributedTimeManager> create_distributed_time_manager(const TimeSyncConfig& config);

/**
 * @brief Create a time master (Raft consensus) component
 * @param local_node_id This node's ID
 * @param cluster_nodes All nodes in the cluster
 * @param config Configuration
 * @return Unique pointer to the time master
 */
std::unique_ptr<ITimeMaster> create_time_master(const NodeId& local_node_id,
                                                 const std::vector<NodeId>& cluster_nodes,
                                                 const TimeSyncConfig& config);

/**
 * @brief Create a clock synchronizer component
 * @param local_node_id This node's ID
 * @param algorithm Synchronization algorithm to use
 * @return Unique pointer to the synchronizer
 */
std::unique_ptr<IClockSynchronizer> create_clock_synchronizer(const NodeId& local_node_id,
                                                               ClockSyncAlgorithm algorithm);

/**
 * @brief Create a causality manager component
 * @param local_node_id This node's ID
 * @param enable_vector_clocks Whether to use full vector clocks
 * @return Unique pointer to the causality manager
 */
std::unique_ptr<ICausalityManager> create_causality_manager(const NodeId& local_node_id,
                                                             bool enable_vector_clocks);

/**
 * @brief Create an event reorderer component
 * @param buffer_time How long to buffer events for reordering
 * @param max_buffered Maximum events to buffer
 * @return Unique pointer to the event reorderer
 */
std::unique_ptr<IEventReorderer> create_event_reorderer(Duration buffer_time,
                                                         UInt32 max_buffered);

} // namespace jaguar::cloud
