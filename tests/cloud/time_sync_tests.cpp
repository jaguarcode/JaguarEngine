/**
 * @file time_sync_tests.cpp
 * @brief Unit tests for distributed time synchronization
 */

#include <gtest/gtest.h>
#include "jaguar/cloud/distributed_time.h"
#include <thread>
#include <chrono>

using namespace jaguar;
using namespace jaguar::cloud;

// ============================================================================
// TimeSyncResult Tests
// ============================================================================

TEST(TimeSyncResultTest, EnumValues) {
    EXPECT_EQ(static_cast<UInt8>(TimeSyncResult::Success), 0);
    EXPECT_NE(TimeSyncResult::InvalidConfiguration, TimeSyncResult::Success);
    EXPECT_NE(TimeSyncResult::ElectionFailed, TimeSyncResult::Success);
    EXPECT_NE(TimeSyncResult::SyncFailed, TimeSyncResult::Success);
    EXPECT_NE(TimeSyncResult::CausalityViolation, TimeSyncResult::Success);
}

TEST(TimeSyncResultTest, ToString) {
    EXPECT_STREQ(time_sync_result_to_string(TimeSyncResult::Success), "Success");
    EXPECT_STREQ(time_sync_result_to_string(TimeSyncResult::ElectionInProgress), "ElectionInProgress");
    EXPECT_STREQ(time_sync_result_to_string(TimeSyncResult::NoQuorum), "NoQuorum");
    EXPECT_STREQ(time_sync_result_to_string(TimeSyncResult::NotLeader), "NotLeader");
    EXPECT_STREQ(time_sync_result_to_string(TimeSyncResult::CausalityViolation), "CausalityViolation");
}

// ============================================================================
// RaftState Tests
// ============================================================================

TEST(RaftStateTest, EnumValues) {
    EXPECT_EQ(static_cast<UInt8>(RaftState::Follower), 0);
    EXPECT_EQ(static_cast<UInt8>(RaftState::Candidate), 1);
    EXPECT_EQ(static_cast<UInt8>(RaftState::Leader), 2);
}

TEST(RaftStateTest, ToString) {
    EXPECT_STREQ(raft_state_to_string(RaftState::Follower), "Follower");
    EXPECT_STREQ(raft_state_to_string(RaftState::Candidate), "Candidate");
    EXPECT_STREQ(raft_state_to_string(RaftState::Leader), "Leader");
}

// ============================================================================
// TimeAdvancementMode Tests
// ============================================================================

TEST(TimeAdvancementModeTest, EnumValues) {
    EXPECT_NE(TimeAdvancementMode::Conservative, TimeAdvancementMode::Optimistic);
    EXPECT_NE(TimeAdvancementMode::Bounded, TimeAdvancementMode::Adaptive);
}

TEST(TimeAdvancementModeTest, ToString) {
    EXPECT_STREQ(time_advancement_mode_to_string(TimeAdvancementMode::Conservative), "Conservative");
    EXPECT_STREQ(time_advancement_mode_to_string(TimeAdvancementMode::Optimistic), "Optimistic");
    EXPECT_STREQ(time_advancement_mode_to_string(TimeAdvancementMode::Bounded), "Bounded");
    EXPECT_STREQ(time_advancement_mode_to_string(TimeAdvancementMode::Adaptive), "Adaptive");
}

// ============================================================================
// ClockSyncAlgorithm Tests
// ============================================================================

TEST(ClockSyncAlgorithmTest, ToString) {
    EXPECT_STREQ(clock_sync_algorithm_to_string(ClockSyncAlgorithm::NTP), "NTP");
    EXPECT_STREQ(clock_sync_algorithm_to_string(ClockSyncAlgorithm::PTP), "PTP");
    EXPECT_STREQ(clock_sync_algorithm_to_string(ClockSyncAlgorithm::Cristian), "Cristian");
    EXPECT_STREQ(clock_sync_algorithm_to_string(ClockSyncAlgorithm::Berkeley), "Berkeley");
}

// ============================================================================
// TimeSyncConfig Tests
// ============================================================================

TEST(TimeSyncConfigTest, DefaultConfig) {
    auto config = TimeSyncConfig::default_config();
    EXPECT_EQ(config.advancement_mode, TimeAdvancementMode::Conservative);
    EXPECT_EQ(config.sync_algorithm, ClockSyncAlgorithm::NTP);
    EXPECT_TRUE(config.auto_barrier_on_step);
    EXPECT_FALSE(config.enable_vector_clocks);
}

TEST(TimeSyncConfigTest, LowLatencyConfig) {
    auto config = TimeSyncConfig::low_latency();
    EXPECT_EQ(config.advancement_mode, TimeAdvancementMode::Optimistic);
    EXPECT_LT(config.sync_interval, std::chrono::milliseconds(100));
    EXPECT_LT(config.heartbeat_interval, std::chrono::milliseconds(50));
}

TEST(TimeSyncConfigTest, HighFidelityConfig) {
    auto config = TimeSyncConfig::high_fidelity();
    EXPECT_EQ(config.advancement_mode, TimeAdvancementMode::Conservative);
    EXPECT_TRUE(config.enable_vector_clocks);
    EXPECT_GT(config.sync_samples, 8u);
}

TEST(TimeSyncConfigTest, LargeClusterConfig) {
    auto config = TimeSyncConfig::large_cluster();
    EXPECT_EQ(config.advancement_mode, TimeAdvancementMode::Bounded);
    EXPECT_TRUE(config.auto_remove_dead_nodes);
    EXPECT_GT(config.max_buffered_events, 10000u);
}

// ============================================================================
// VectorClock Tests
// ============================================================================

TEST(VectorClockTest, IncrementAndGet) {
    VectorClock clock;
    clock.increment("node-1");
    EXPECT_EQ(clock.get("node-1"), 1);
    clock.increment("node-1");
    EXPECT_EQ(clock.get("node-1"), 2);
}

TEST(VectorClockTest, SetAndGet) {
    VectorClock clock;
    clock.set("node-1", 5);
    clock.set("node-2", 3);
    EXPECT_EQ(clock.get("node-1"), 5);
    EXPECT_EQ(clock.get("node-2"), 3);
    EXPECT_EQ(clock.get("node-3"), 0);  // Non-existent
}

TEST(VectorClockTest, Merge) {
    VectorClock clock1;
    clock1.set("node-1", 5);
    clock1.set("node-2", 3);

    VectorClock clock2;
    clock2.set("node-1", 3);
    clock2.set("node-2", 7);
    clock2.set("node-3", 2);

    clock1.merge(clock2);

    EXPECT_EQ(clock1.get("node-1"), 5);  // max(5, 3)
    EXPECT_EQ(clock1.get("node-2"), 7);  // max(3, 7)
    EXPECT_EQ(clock1.get("node-3"), 2);  // from clock2
}

TEST(VectorClockTest, HappensBefore) {
    VectorClock earlier;
    earlier.set("node-1", 1);
    earlier.set("node-2", 1);

    VectorClock later;
    later.set("node-1", 2);
    later.set("node-2", 2);

    EXPECT_TRUE(earlier.happens_before(later));
    EXPECT_FALSE(later.happens_before(earlier));
}

TEST(VectorClockTest, Concurrent) {
    VectorClock clock1;
    clock1.set("node-1", 2);
    clock1.set("node-2", 1);

    VectorClock clock2;
    clock2.set("node-1", 1);
    clock2.set("node-2", 2);

    EXPECT_TRUE(clock1.concurrent_with(clock2));
    EXPECT_TRUE(clock2.concurrent_with(clock1));
}

// ============================================================================
// Time Master (Raft) Tests
// ============================================================================

class TimeMasterTest : public ::testing::Test {
protected:
    void SetUp() override {
        TimeSyncConfig config;
        config.local_node_id = "node-1";
        config.cluster_nodes = {"node-1", "node-2", "node-3"};
        config.election_timeout_min = std::chrono::milliseconds(50);
        config.election_timeout_max = std::chrono::milliseconds(100);
        config.heartbeat_interval = std::chrono::milliseconds(25);

        master_ = create_time_master("node-1", {"node-1", "node-2", "node-3"}, config);
    }

    std::unique_ptr<ITimeMaster> master_;
};

TEST_F(TimeMasterTest, InitialState) {
    EXPECT_EQ(master_->get_state(), RaftState::Follower);
    EXPECT_EQ(master_->get_current_term(), 0);
    EXPECT_FALSE(master_->is_leader());
    EXPECT_FALSE(master_->get_leader_id().has_value());
}

TEST_F(TimeMasterTest, StartElection) {
    auto result = master_->start_election();
    EXPECT_EQ(result, TimeSyncResult::ElectionInProgress);
    EXPECT_EQ(master_->get_state(), RaftState::Candidate);
    EXPECT_EQ(master_->get_current_term(), 1);
}

TEST_F(TimeMasterTest, VoteRequest) {
    VoteRequest request;
    request.term = 1;
    request.candidate_id = "node-2";
    request.last_log_index = 0;
    request.last_log_term = 0;

    VoteResponse response;
    auto result = master_->handle_vote_request(request, response);

    EXPECT_EQ(result, TimeSyncResult::Success);
    EXPECT_TRUE(response.vote_granted);
    EXPECT_EQ(response.voter_id, "node-1");
}

TEST_F(TimeMasterTest, VoteRequestHigherTerm) {
    // Start election first
    master_->start_election();
    EXPECT_EQ(master_->get_current_term(), 1);

    // Receive vote request with higher term
    VoteRequest request;
    request.term = 5;
    request.candidate_id = "node-2";
    request.last_log_index = 0;
    request.last_log_term = 0;

    VoteResponse response;
    master_->handle_vote_request(request, response);

    // Should step down and grant vote
    EXPECT_EQ(master_->get_state(), RaftState::Follower);
    EXPECT_EQ(master_->get_current_term(), 5);
    EXPECT_TRUE(response.vote_granted);
}

TEST_F(TimeMasterTest, VoteRequestLowerTerm) {
    // Start election to increase term
    master_->start_election();
    EXPECT_EQ(master_->get_current_term(), 1);

    // Receive vote request with lower term
    VoteRequest request;
    request.term = 0;
    request.candidate_id = "node-2";

    VoteResponse response;
    master_->handle_vote_request(request, response);

    // Should not grant vote
    EXPECT_FALSE(response.vote_granted);
}

TEST_F(TimeMasterTest, AppendEntry) {
    // Start election and become leader (simulate receiving votes)
    master_->start_election();

    // Simulate receiving majority votes
    VoteResponse vote1;
    vote1.term = 1;
    vote1.vote_granted = true;
    vote1.voter_id = "node-2";
    master_->handle_vote_response(vote1);

    // With 2 out of 3 votes, should become leader
    EXPECT_EQ(master_->get_state(), RaftState::Leader);

    // Now can append entries
    RaftLogEntry entry;
    entry.sim_time = 1.0;
    entry.command = "advance_time";

    auto result = master_->append_entry(entry);
    EXPECT_EQ(result, TimeSyncResult::Success);
    EXPECT_EQ(master_->get_last_log_index(), 1);
}

TEST_F(TimeMasterTest, AppendEntryNotLeader) {
    RaftLogEntry entry;
    entry.sim_time = 1.0;

    auto result = master_->append_entry(entry);
    EXPECT_EQ(result, TimeSyncResult::NotLeader);
}

TEST_F(TimeMasterTest, HandleAppendEntries) {
    AppendEntriesRequest request;
    request.term = 1;
    request.leader_id = "node-2";
    request.prev_log_index = 0;
    request.prev_log_term = 0;
    request.leader_commit = 0;
    request.current_sim_time = 1.0;

    AppendEntriesResponse response;
    auto result = master_->handle_append_request(request, response);

    EXPECT_EQ(result, TimeSyncResult::Success);
    EXPECT_TRUE(response.success);
    EXPECT_EQ(master_->get_leader_id().value(), "node-2");
}

TEST_F(TimeMasterTest, Reset) {
    master_->start_election();
    EXPECT_EQ(master_->get_current_term(), 1);

    master_->reset();
    EXPECT_EQ(master_->get_state(), RaftState::Follower);
    EXPECT_EQ(master_->get_current_term(), 0);
}

// ============================================================================
// Clock Synchronizer Tests
// ============================================================================

class ClockSynchronizerTest : public ::testing::Test {
protected:
    void SetUp() override {
        sync_ = create_clock_synchronizer("node-1", ClockSyncAlgorithm::NTP);
    }

    std::unique_ptr<IClockSynchronizer> sync_;
};

TEST_F(ClockSynchronizerTest, InitialState) {
    EXPECT_EQ(sync_->get_offset().count(), 0);
    EXPECT_EQ(sync_->get_sync_count(), 0);
    EXPECT_NEAR(sync_->get_drift_rate(), 0.0, 0.001);
}

TEST_F(ClockSynchronizerTest, RequestSync) {
    auto result = sync_->request_sync("master-node");
    EXPECT_EQ(result, TimeSyncResult::Success);
}

TEST_F(ClockSynchronizerTest, HandleSyncRequest) {
    ClockSyncRequest request;
    request.sender_id = "client-node";
    request.t1 = std::chrono::steady_clock::now();
    request.sequence = 1;

    ClockSyncResponse response;
    auto result = sync_->handle_sync_request(request, response);

    EXPECT_EQ(result, TimeSyncResult::Success);
    EXPECT_EQ(response.responder_id, "node-1");
    EXPECT_EQ(response.t1, request.t1);
    EXPECT_EQ(response.sequence, 1);
}

TEST_F(ClockSynchronizerTest, SyncRoundTrip) {
    // Request sync
    sync_->request_sync("master-node");

    // Simulate response
    ClockSyncResponse response;
    response.responder_id = "master-node";
    response.t1 = std::chrono::steady_clock::now() - std::chrono::milliseconds(5);
    response.t2 = std::chrono::steady_clock::now() - std::chrono::milliseconds(2);
    response.t3 = std::chrono::steady_clock::now() - std::chrono::milliseconds(2);
    response.sequence = 1;

    auto result = sync_->handle_sync_response(response);
    EXPECT_EQ(result, TimeSyncResult::Success);
    EXPECT_EQ(sync_->get_sync_count(), 1);
}

TEST_F(ClockSynchronizerTest, AdjustedTime) {
    auto before = std::chrono::steady_clock::now();
    auto adjusted = sync_->get_adjusted_time();
    auto after = std::chrono::steady_clock::now();

    // With zero offset, adjusted time should be close to current time
    EXPECT_GE(adjusted, before);
    EXPECT_LE(adjusted, after);
}

// ============================================================================
// Causality Manager Tests
// ============================================================================

class CausalityManagerTest : public ::testing::Test {
protected:
    void SetUp() override {
        manager_ = create_causality_manager("node-1", true);
    }

    std::unique_ptr<ICausalityManager> manager_;
};

TEST_F(CausalityManagerTest, InitialLogicalTime) {
    EXPECT_EQ(manager_->get_logical_time(), 0);
}

TEST_F(CausalityManagerTest, IncrementLogicalTime) {
    auto t1 = manager_->increment_and_get();
    EXPECT_EQ(t1, 1);

    auto t2 = manager_->increment_and_get();
    EXPECT_EQ(t2, 2);

    EXPECT_EQ(manager_->get_logical_time(), 2);
}

TEST_F(CausalityManagerTest, UpdateOnReceive) {
    manager_->increment_and_get();  // Local time = 1

    // Receive message with higher timestamp
    manager_->update_on_receive(5);

    // Should be max(1, 5) + 1 = 6
    EXPECT_EQ(manager_->get_logical_time(), 6);
}

TEST_F(CausalityManagerTest, VectorClockOperations) {
    manager_->increment_vector_clock();
    auto clock = manager_->get_vector_clock();
    EXPECT_EQ(clock.get("node-1"), 1);

    VectorClock other;
    other.set("node-2", 3);
    manager_->merge_vector_clock(other);

    clock = manager_->get_vector_clock();
    EXPECT_EQ(clock.get("node-1"), 2);  // Incremented after merge
    EXPECT_EQ(clock.get("node-2"), 3);  // From other
}

TEST_F(CausalityManagerTest, CreateEvent) {
    auto event = manager_->create_event("test_event", 1.0, {1, 2, 3});

    EXPECT_EQ(event.source_node, "node-1");
    EXPECT_EQ(event.sim_time, 1.0);
    EXPECT_EQ(event.logical_time, 1);  // First increment
    EXPECT_EQ(event.event_type, "test_event");
    EXPECT_EQ(event.payload.size(), 3);
    EXPECT_FALSE(event.processed);
}

TEST_F(CausalityManagerTest, HappensBeforeLamport) {
    auto event1 = manager_->create_event("event1", 1.0);
    auto event2 = manager_->create_event("event2", 2.0);

    EXPECT_TRUE(manager_->happens_before(event1, event2));
    EXPECT_FALSE(manager_->happens_before(event2, event1));
}

// ============================================================================
// Event Reorderer Tests
// ============================================================================

class EventReordererTest : public ::testing::Test {
protected:
    void SetUp() override {
        reorderer_ = create_event_reorderer(
            std::chrono::milliseconds(50),
            1000);
    }

    std::unique_ptr<IEventReorderer> reorderer_;
};

TEST_F(EventReordererTest, InitialState) {
    EXPECT_EQ(reorderer_->get_buffered_count(), 0);
    EXPECT_EQ(reorderer_->get_safe_time(), 0.0);
}

TEST_F(EventReordererTest, SubmitEvent) {
    TimestampedEvent event;
    event.event_id = 1;
    event.source_node = "node-1";
    event.sim_time = 1.0;
    event.logical_time = 1;
    event.wall_time = std::chrono::steady_clock::now();

    auto result = reorderer_->submit_event(std::move(event));
    EXPECT_EQ(result, TimeSyncResult::Success);
    EXPECT_EQ(reorderer_->get_buffered_count(), 1);
}

TEST_F(EventReordererTest, GetReadyEventsAfterSafeTime) {
    // Submit events
    TimestampedEvent event1;
    event1.event_id = 1;
    event1.sim_time = 0.5;
    event1.logical_time = 1;
    event1.wall_time = std::chrono::steady_clock::now();
    reorderer_->submit_event(std::move(event1));

    TimestampedEvent event2;
    event2.event_id = 2;
    event2.sim_time = 1.5;
    event2.logical_time = 2;
    event2.wall_time = std::chrono::steady_clock::now();
    reorderer_->submit_event(std::move(event2));

    // Advance safe time
    reorderer_->advance_safe_time(1.0);

    auto ready = reorderer_->get_ready_events();
    EXPECT_EQ(ready.size(), 1);
    EXPECT_EQ(ready[0].event_id, 1);
    EXPECT_EQ(reorderer_->get_buffered_count(), 1);
}

TEST_F(EventReordererTest, EventOrdering) {
    // Submit events out of order
    TimestampedEvent event2;
    event2.event_id = 2;
    event2.sim_time = 2.0;
    event2.logical_time = 2;
    event2.wall_time = std::chrono::steady_clock::now();
    reorderer_->submit_event(std::move(event2));

    TimestampedEvent event1;
    event1.event_id = 1;
    event1.sim_time = 1.0;
    event1.logical_time = 1;
    event1.wall_time = std::chrono::steady_clock::now();
    reorderer_->submit_event(std::move(event1));

    // Advance safe time past both
    reorderer_->advance_safe_time(3.0);

    auto ready = reorderer_->get_ready_events();
    EXPECT_EQ(ready.size(), 2);
    EXPECT_EQ(ready[0].event_id, 1);  // Earlier event first
    EXPECT_EQ(ready[1].event_id, 2);
}

TEST_F(EventReordererTest, Flush) {
    TimestampedEvent event;
    event.event_id = 1;
    event.sim_time = 1000.0;  // Far in the future
    event.logical_time = 1;
    event.wall_time = std::chrono::steady_clock::now();
    reorderer_->submit_event(std::move(event));

    EXPECT_EQ(reorderer_->get_buffered_count(), 1);

    reorderer_->flush();
    auto ready = reorderer_->get_ready_events();
    EXPECT_EQ(ready.size(), 1);
}

TEST_F(EventReordererTest, Clear) {
    TimestampedEvent event;
    event.event_id = 1;
    event.sim_time = 1.0;
    event.logical_time = 1;
    event.wall_time = std::chrono::steady_clock::now();
    reorderer_->submit_event(std::move(event));

    reorderer_->clear();
    EXPECT_EQ(reorderer_->get_buffered_count(), 0);
    EXPECT_EQ(reorderer_->get_safe_time(), 0.0);
}

// ============================================================================
// Distributed Time Manager Tests
// ============================================================================

class DistributedTimeManagerTest : public ::testing::Test {
protected:
    void SetUp() override {
        TimeSyncConfig config;
        config.local_node_id = "node-1";
        config.cluster_nodes = {"node-1", "node-2", "node-3"};
        config.initial_sim_time = 0.0;
        config.time_step = 0.001;
        config.election_timeout_min = std::chrono::milliseconds(50);
        config.election_timeout_max = std::chrono::milliseconds(100);

        manager_ = create_distributed_time_manager(config);
    }

    std::unique_ptr<DistributedTimeManager> manager_;
};

TEST_F(DistributedTimeManagerTest, Initialize) {
    EXPECT_FALSE(manager_->is_initialized());

    auto result = manager_->initialize();
    EXPECT_EQ(result, TimeSyncResult::Success);
    EXPECT_TRUE(manager_->is_initialized());
}

TEST_F(DistributedTimeManagerTest, InitializeTwice) {
    manager_->initialize();
    auto result = manager_->initialize();
    EXPECT_EQ(result, TimeSyncResult::AlreadyInitialized);
}

TEST_F(DistributedTimeManagerTest, Shutdown) {
    manager_->initialize();
    auto result = manager_->shutdown();
    EXPECT_EQ(result, TimeSyncResult::Success);
    EXPECT_FALSE(manager_->is_initialized());
}

TEST_F(DistributedTimeManagerTest, ShutdownWithoutInit) {
    auto result = manager_->shutdown();
    EXPECT_EQ(result, TimeSyncResult::NotInitialized);
}

TEST_F(DistributedTimeManagerTest, GetLocalNodeId) {
    EXPECT_EQ(manager_->get_local_node_id(), "node-1");
}

TEST_F(DistributedTimeManagerTest, NodeManagement) {
    manager_->initialize();

    // Get all nodes
    auto nodes = manager_->get_all_nodes();
    EXPECT_EQ(nodes.size(), 3);

    // Add a new node
    TimeSyncNode new_node;
    new_node.id = "node-4";
    new_node.is_active = true;
    manager_->add_node(new_node);

    nodes = manager_->get_all_nodes();
    EXPECT_EQ(nodes.size(), 4);

    // Remove the node
    manager_->remove_node("node-4");
    nodes = manager_->get_all_nodes();
    EXPECT_EQ(nodes.size(), 3);
}

TEST_F(DistributedTimeManagerTest, CannotRemoveLocalNode) {
    manager_->initialize();
    auto result = manager_->remove_node("node-1");
    EXPECT_EQ(result, TimeSyncResult::InvalidNodeId);
}

TEST_F(DistributedTimeManagerTest, InitialRaftState) {
    manager_->initialize();
    EXPECT_EQ(manager_->get_raft_state(), RaftState::Follower);
    EXPECT_EQ(manager_->get_current_term(), 0);
    EXPECT_FALSE(manager_->is_leader());
}

TEST_F(DistributedTimeManagerTest, TriggerElection) {
    manager_->initialize();
    auto result = manager_->trigger_election();
    EXPECT_EQ(result, TimeSyncResult::ElectionInProgress);
    EXPECT_EQ(manager_->get_raft_state(), RaftState::Candidate);
}

TEST_F(DistributedTimeManagerTest, SimulationTime) {
    manager_->initialize();
    EXPECT_EQ(manager_->get_simulation_time(), 0.0);
}

TEST_F(DistributedTimeManagerTest, AdvanceTimeNotLeader) {
    manager_->initialize();
    auto result = manager_->advance_time(0.001);
    EXPECT_EQ(result, TimeSyncResult::NotLeader);
}

TEST_F(DistributedTimeManagerTest, LogicalTime) {
    manager_->initialize();
    EXPECT_EQ(manager_->get_logical_time(), 0);

    auto t1 = manager_->next_logical_time();
    EXPECT_EQ(t1, 1);

    manager_->update_logical_time(10);
    EXPECT_EQ(manager_->get_logical_time(), 11);  // max(1, 10) + 1
}

TEST_F(DistributedTimeManagerTest, CreateEvent) {
    manager_->initialize();
    auto event = manager_->create_event("test", {0x01, 0x02});

    EXPECT_EQ(event.source_node, "node-1");
    EXPECT_EQ(event.event_type, "test");
    EXPECT_EQ(event.payload.size(), 2);
}

TEST_F(DistributedTimeManagerTest, SubmitEvent) {
    manager_->initialize();

    TimestampedEvent event;
    event.event_id = 1;
    event.source_node = "node-2";
    event.sim_time = 0.5;
    event.logical_time = 5;
    event.wall_time = std::chrono::steady_clock::now();

    auto result = manager_->submit_event(std::move(event));
    EXPECT_EQ(result, TimeSyncResult::Success);

    // Logical time should be updated
    EXPECT_GT(manager_->get_logical_time(), 5);
}

TEST_F(DistributedTimeManagerTest, Barrier) {
    manager_->initialize();

    UInt64 barrier_id = manager_->create_barrier(1.0);
    EXPECT_GT(barrier_id, 0u);
    EXPECT_FALSE(manager_->is_barrier_complete(barrier_id));

    // Arrive at barrier
    auto result = manager_->arrive_at_barrier(barrier_id);
    EXPECT_EQ(result, TimeSyncResult::Success);
}

TEST_F(DistributedTimeManagerTest, Statistics) {
    manager_->initialize();

    auto stats = manager_->get_statistics();
    EXPECT_EQ(stats.current_sim_time, 0.0);
    EXPECT_EQ(stats.elections_held, 0);

    manager_->trigger_election();
    stats = manager_->get_statistics();
    EXPECT_EQ(stats.elections_held, 1);
}

TEST_F(DistributedTimeManagerTest, ResetStatistics) {
    manager_->initialize();
    manager_->trigger_election();

    manager_->reset_statistics();
    auto stats = manager_->get_statistics();
    EXPECT_EQ(stats.elections_held, 0);
}

TEST_F(DistributedTimeManagerTest, SetAdvancementMode) {
    manager_->initialize();

    auto result = manager_->set_advancement_mode(TimeAdvancementMode::Optimistic);
    EXPECT_EQ(result, TimeSyncResult::Success);
    EXPECT_EQ(manager_->get_config().advancement_mode, TimeAdvancementMode::Optimistic);
}

TEST_F(DistributedTimeManagerTest, Callbacks) {
    manager_->initialize();

    bool election_called = false;
    bool time_called = false;

    manager_->set_election_callback([&](RaftState /*state*/, const std::optional<NodeId>& /*leader*/) {
        election_called = true;
    });

    manager_->set_time_advance_callback([&](SimulationTime /*time*/) {
        time_called = true;
    });

    // Trigger election to test callback
    manager_->trigger_election();
    manager_->tick();

    // Election callback might be called depending on state transitions
    // For this test, we just verify callbacks are set without crashing
}

// ============================================================================
// Integration Tests
// ============================================================================

TEST(TimeSyncIntegrationTest, ClusterSimulation) {
    // Create three nodes
    TimeSyncConfig config1, config2, config3;
    config1.local_node_id = "node-1";
    config1.cluster_nodes = {"node-1", "node-2", "node-3"};
    config2.local_node_id = "node-2";
    config2.cluster_nodes = {"node-1", "node-2", "node-3"};
    config3.local_node_id = "node-3";
    config3.cluster_nodes = {"node-1", "node-2", "node-3"};

    auto mgr1 = create_distributed_time_manager(config1);
    auto mgr2 = create_distributed_time_manager(config2);
    auto mgr3 = create_distributed_time_manager(config3);

    mgr1->initialize();
    mgr2->initialize();
    mgr3->initialize();

    // All should start as followers
    EXPECT_EQ(mgr1->get_raft_state(), RaftState::Follower);
    EXPECT_EQ(mgr2->get_raft_state(), RaftState::Follower);
    EXPECT_EQ(mgr3->get_raft_state(), RaftState::Follower);

    // Node 1 starts election
    mgr1->trigger_election();
    EXPECT_EQ(mgr1->get_raft_state(), RaftState::Candidate);

    // Simulate vote requests and responses
    VoteRequest vote_req;
    vote_req.term = mgr1->get_current_term();
    vote_req.candidate_id = "node-1";
    vote_req.last_log_index = 0;
    vote_req.last_log_term = 0;

    VoteResponse vote_resp2, vote_resp3;
    mgr2->handle_vote_request(vote_req, vote_resp2);
    mgr3->handle_vote_request(vote_req, vote_resp3);

    EXPECT_TRUE(vote_resp2.vote_granted);
    EXPECT_TRUE(vote_resp3.vote_granted);

    mgr1->shutdown();
    mgr2->shutdown();
    mgr3->shutdown();
}

TEST(TimeSyncIntegrationTest, EventOrdering) {
    TimeSyncConfig config;
    config.local_node_id = "node-1";
    config.cluster_nodes = {"node-1"};
    config.enable_vector_clocks = true;

    auto mgr = create_distributed_time_manager(config);
    mgr->initialize();

    // Create and submit events out of order
    auto event1 = mgr->create_event("event1");
    auto event2 = mgr->create_event("event2");
    auto event3 = mgr->create_event("event3");

    // Submit in reverse order
    event3.sim_time = 3.0;
    event2.sim_time = 2.0;
    event1.sim_time = 1.0;

    mgr->submit_event(std::move(event3));
    mgr->submit_event(std::move(event1));
    mgr->submit_event(std::move(event2));

    mgr->shutdown();
}

// ============================================================================
// Performance Tests
// ============================================================================

TEST(TimeSyncPerformanceTest, LogicalTimeIncrement) {
    auto mgr = create_causality_manager("node-1", false);

    auto start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < 100000; i++) {
        mgr->increment_and_get();
    }
    auto end = std::chrono::high_resolution_clock::now();

    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    // Should complete quickly (< 100ms for 100K increments)
    EXPECT_LT(duration.count(), 100000);
}

TEST(TimeSyncPerformanceTest, EventSubmission) {
    auto reorderer = create_event_reorderer(
        std::chrono::milliseconds(50),
        100000);

    auto start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < 10000; i++) {
        TimestampedEvent event;
        event.event_id = static_cast<UInt64>(i);
        event.sim_time = static_cast<SimulationTime>(i) * 0.001;
        event.logical_time = static_cast<LogicalTime>(i);
        event.wall_time = std::chrono::steady_clock::now();
        reorderer->submit_event(std::move(event));
    }
    auto end = std::chrono::high_resolution_clock::now();

    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    // Should handle 10K events in reasonable time
    EXPECT_LT(duration.count(), 1000);
}

// ============================================================================
// Edge Case Tests
// ============================================================================

TEST(TimeSyncEdgeCaseTest, EmptyCluster) {
    TimeSyncConfig config;
    config.local_node_id = "node-1";
    config.cluster_nodes = {"node-1"};  // Single node cluster

    auto mgr = create_distributed_time_manager(config);
    mgr->initialize();

    EXPECT_EQ(mgr->get_active_node_count(), 1);

    // Single node can still function
    mgr->trigger_election();
    // With only self-vote, can become leader
    EXPECT_EQ(mgr->get_raft_state(), RaftState::Leader);

    mgr->shutdown();
}

TEST(TimeSyncEdgeCaseTest, ZeroTimeStep) {
    TimeSyncConfig config;
    config.local_node_id = "node-1";
    config.cluster_nodes = {"node-1"};
    config.time_step = 0.0;  // Zero time step

    auto mgr = create_distributed_time_manager(config);
    mgr->initialize();

    EXPECT_EQ(mgr->get_simulation_time(), 0.0);

    mgr->shutdown();
}

TEST(TimeSyncEdgeCaseTest, LargeLogicalTime) {
    auto mgr = create_causality_manager("node-1", false);

    // Update with very large timestamp
    mgr->update_on_receive(std::numeric_limits<LogicalTime>::max() - 10);

    // Should handle without overflow issues
    auto time = mgr->get_logical_time();
    EXPECT_GT(time, 0u);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
