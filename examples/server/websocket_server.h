/**
 * @file websocket_server.h
 * @brief WebSocket server for JaguarEngine frontend communication
 */

#pragma once

#include "jaguar/jaguar.h"
#include <string>
#include <vector>
#include <memory>
#include <functional>
#include <unordered_map>
#include <mutex>
#include <queue>
#include <map>

// libwebsockets header
#include <libwebsockets.h>

/**
 * @brief Simulation status
 */
enum class SimulationStatus {
    Idle,
    Running,
    Paused,
    Stopped
};

/**
 * @brief Autopilot mode for AI agent control (Air domain)
 */
enum class AutopilotMode : uint8_t {
    OFF,
    ALTITUDE_HOLD,
    HEADING_HOLD,
    SPEED_HOLD,
    NAV  // Waypoint navigation
};

/**
 * @brief Vehicle control mode for land entities
 */
enum class VehicleControlMode : uint8_t {
    MANUAL,
    CRUISE_CONTROL,    // Maintain speed
    WAYPOINT_FOLLOW,   // Follow waypoints
    FORMATION          // Maintain formation
};

/**
 * @brief Ship control mode for sea entities
 */
enum class ShipControlMode : uint8_t {
    MANUAL,
    AUTOPILOT,         // Heading/speed hold
    WAYPOINT_NAV,      // Waypoint navigation
    STATION_KEEPING    // Maintain position
};

/**
 * @brief Space control mode for space entities
 */
enum class SpaceControlMode : uint8_t {
    DRIFT,             // No active control
    ATTITUDE_HOLD,     // Maintain orientation
    ORBIT_MAINTAIN,    // Station keeping
    MANEUVER           // Executing burn
};

/**
 * @brief Waypoint for navigation
 */
struct Waypoint {
    std::string name;
    jaguar::Vec3 position;  // ECEF position
    double altitude = 0.0;
    double speed = 0.0;     // Target speed, 0 = maintain current

    enum class Type { FLY_BY, FLY_OVER, HOLD, INTERCEPT };
    Type type = Type::FLY_BY;
};

/**
 * @brief Autopilot command for air entity control
 */
struct AutopilotCommand {
    AutopilotMode mode = AutopilotMode::OFF;
    double target_altitude = 0.0;
    double target_heading = 0.0;
    double target_speed = 0.0;
    std::vector<Waypoint> flight_plan;
    size_t active_waypoint = 0;
};

/**
 * @brief Vehicle command for land entity control
 */
struct VehicleCommand {
    VehicleControlMode mode = VehicleControlMode::MANUAL;
    double throttle = 0.0;      // 0.0 to 1.0
    double steering = 0.0;      // -1.0 (left) to 1.0 (right)
    double brake = 0.0;         // 0.0 to 1.0
    double target_speed = 0.0;  // For cruise control
    double target_heading = 0.0;
    std::vector<Waypoint> route;
    size_t active_waypoint = 0;
};

/**
 * @brief Ship command for sea entity control
 */
struct ShipCommand {
    ShipControlMode mode = ShipControlMode::MANUAL;
    double throttle = 0.0;      // -1.0 (reverse) to 1.0 (ahead)
    double rudder = 0.0;        // -1.0 (port) to 1.0 (starboard)
    double target_speed = 0.0;
    double target_heading = 0.0;
    jaguar::Vec3 station_position;  // For station keeping
    std::vector<Waypoint> route;
    size_t active_waypoint = 0;
};

/**
 * @brief Space command for space entity control
 */
struct SpaceCommand {
    SpaceControlMode mode = SpaceControlMode::DRIFT;
    double thrust_x = 0.0;      // -1.0 to 1.0 (lateral)
    double thrust_y = 0.0;      // -1.0 to 1.0 (vertical)
    double thrust_z = 0.0;      // -1.0 to 1.0 (forward/back)
    double roll_rate = 0.0;     // Target roll rate
    double pitch_rate = 0.0;    // Target pitch rate
    double yaw_rate = 0.0;      // Target yaw rate
    jaguar::Vec3 target_attitude;  // Target orientation (Euler angles)
};

/**
 * @brief WebSocket message types matching frontend protocol
 */
enum class MessageType {
    WorldState,
    Command,
    EntitySpawn,
    EntityDestroy,
    Error
};

/**
 * @brief Entity state for JSON serialization
 */
struct EntityData {
    jaguar::EntityId id;
    std::string name;
    jaguar::Domain domain;
    jaguar::physics::EntityState state;
    bool is_active;
    double health;
    double damage;
};

/**
 * @brief World state snapshot for broadcasting
 */
struct WorldState {
    std::vector<EntityData> entities;
    SimulationStatus status;
    double simulation_time;
    double wall_clock_time;
    double delta_time;
    double realtime_ratio;
    double frame_rate;
    double physics_time;
    std::unordered_map<jaguar::Domain, int> entities_by_domain;
    int total_entities;
    int active_entities;
};

/**
 * @brief WebSocket server bridging JaguarEngine to web clients
 */
class WebSocketServer {
public:
    /**
     * @brief Construct server with engine reference
     * @param engine Reference to JaguarEngine instance
     * @param port WebSocket port (default 8081)
     */
    WebSocketServer(jaguar::Engine& engine, uint16_t port = 8081);
    ~WebSocketServer();

    // Non-copyable
    WebSocketServer(const WebSocketServer&) = delete;
    WebSocketServer& operator=(const WebSocketServer&) = delete;

    /**
     * @brief Start the WebSocket server
     * @return true if started successfully
     */
    bool start();

    /**
     * @brief Stop the server
     */
    void stop();

    /**
     * @brief Poll for WebSocket events (non-blocking)
     */
    void poll();

    /**
     * @brief Broadcast current world state to all connected clients
     */
    void broadcast_world_state();

    /**
     * @brief Get current simulation status
     */
    SimulationStatus get_simulation_status() const { return status_; }

    /**
     * @brief Set simulation status
     */
    void set_simulation_status(SimulationStatus status) { status_ = status; }

    /**
     * @brief Get number of connected clients
     */
    size_t get_client_count() const;

    /**
     * @brief Capture initial states of all current entities for reset functionality
     * Call this after creating initial entities
     */
    void capture_initial_states();

    /**
     * @brief Apply autopilot guidance to entities with active waypoint navigation
     * Call this before stepping the simulation
     */
    void apply_autopilot_guidance();

private:
    // Internal methods
    void handle_message(lws* wsi, const std::string& message);
    void handle_command(const std::string& command, const std::string& params, lws* wsi);
    void handle_entity_spawn(const std::string& data, lws* wsi);
    void handle_entity_destroy(const std::string& data, lws* wsi);
    // Air domain handlers
    void handle_set_flight_controls(const std::string& message, lws* wsi);
    void handle_set_autopilot(const std::string& message, lws* wsi);
    void handle_set_waypoints(const std::string& message, lws* wsi);

    // Land domain handlers
    void handle_set_vehicle_controls(const std::string& message, lws* wsi);
    void handle_set_vehicle_autopilot(const std::string& message, lws* wsi);

    // Sea domain handlers
    void handle_set_ship_controls(const std::string& message, lws* wsi);
    void handle_set_ship_autopilot(const std::string& message, lws* wsi);

    // Space domain handlers
    void handle_set_space_controls(const std::string& message, lws* wsi);
    void handle_set_space_autopilot(const std::string& message, lws* wsi);
    void send_message(lws* wsi, const std::string& message);
    void broadcast_message(const std::string& message);

    // JSON serialization
    std::string serialize_world_state();
    std::string serialize_entity(const EntityData& entity);
    std::string serialize_error(const std::string& error);

    // State collection
    WorldState collect_world_state();
    std::vector<EntityData> collect_entities();

    // Members
    jaguar::Engine& engine_;
    uint16_t port_;
    lws_context* context_;
    SimulationStatus status_;
    std::vector<lws*> clients_;
    mutable std::mutex clients_mutex_;

    // Message queue per client for proper lws_write handling
    std::map<lws*, std::queue<std::string>> message_queues_;
    std::mutex queue_mutex_;

    // Performance tracking
    double last_frame_time_;
    double physics_time_;
    int frame_count_;
    std::chrono::steady_clock::time_point start_time_;
    std::chrono::steady_clock::time_point last_fps_update_;
    double current_fps_;

    // Initial entity states for reset functionality
    std::unordered_map<jaguar::EntityId, jaguar::physics::EntityState> initial_entity_states_;

    // Commands per entity for AI agent control (organized by domain)
    std::unordered_map<jaguar::EntityId, AutopilotCommand> autopilot_commands_;  // Air
    std::unordered_map<jaguar::EntityId, VehicleCommand> vehicle_commands_;      // Land
    std::unordered_map<jaguar::EntityId, ShipCommand> ship_commands_;            // Sea
    std::unordered_map<jaguar::EntityId, SpaceCommand> space_commands_;          // Space

    // Callback for libwebsockets
    static int callback_websocket(lws* wsi, enum lws_callback_reasons reason, void* user, void* in, size_t len);
    static WebSocketServer* instance_;  // For callback access
};
