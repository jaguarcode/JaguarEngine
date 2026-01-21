/**
 * @file websocket_server.cpp
 * @brief WebSocket server implementation using libwebsockets
 */

#include "websocket_server.h"
#include "jaguar/core/coordinates.h"
#include <libwebsockets.h>
#include <sstream>
#include <iomanip>
#include <chrono>
#include <cstring>
#include <algorithm>
#include <iostream>
#include <cmath>

// Static instance for callback access
WebSocketServer* WebSocketServer::instance_ = nullptr;

// ============================================================================
// Construction / Destruction
// ============================================================================

WebSocketServer::WebSocketServer(jaguar::Engine& engine, uint16_t port)
    : engine_(engine)
    , port_(port)
    , context_(nullptr)
    , status_(SimulationStatus::Idle)
    , last_frame_time_(0.0)
    , physics_time_(0.0)
    , frame_count_(0)
    , start_time_(std::chrono::steady_clock::now())
    , last_fps_update_(std::chrono::steady_clock::now())
    , current_fps_(0.0)
{
    instance_ = this;
}

WebSocketServer::~WebSocketServer() {
    stop();
    instance_ = nullptr;
}

// ============================================================================
// Server Lifecycle
// ============================================================================

bool WebSocketServer::start() {
    // WebSocket protocol definition
    // Note: First protocol must be the HTTP handler for WebSocket upgrade to work
    static lws_protocols protocols[] = {
        {
            "http",  // HTTP protocol for WebSocket handshake
            lws_callback_http_dummy,  // Use built-in dummy HTTP handler
            0,
            0,
            0, nullptr, 0
        },
        {
            "jaguar-protocol",
            WebSocketServer::callback_websocket,
            0,  // per-session data size
            65536,  // rx buffer size (increased for large messages)
            0, nullptr, 0
        },
        { nullptr, nullptr, 0, 0, 0, nullptr, 0 }  // terminator
    };

    // Server creation info
    lws_context_creation_info info;
    std::memset(&info, 0, sizeof(info));
    info.port = port_;
    info.protocols = protocols;
    info.gid = -1;
    info.uid = -1;
    info.options = LWS_SERVER_OPTION_VALIDATE_UTF8 |
                   LWS_SERVER_OPTION_DISABLE_IPV6;

    // Disable validity checking to prevent connection timeouts
    info.timeout_secs = 0;            // Disable general timeout
    info.ka_time = 0;                 // Disable TCP keepalive timeout
    info.ka_probes = 0;
    info.ka_interval = 0;

    context_ = lws_create_context(&info);
    if (!context_) {
        return false;
    }

    return true;
}

void WebSocketServer::stop() {
    if (context_) {
        lws_context_destroy(context_);
        context_ = nullptr;
    }
    clients_.clear();
}

void WebSocketServer::poll() {
    if (context_) {
        lws_service(context_, 0);  // Non-blocking
    }
}

size_t WebSocketServer::get_client_count() const {
    std::lock_guard<std::mutex> lock(clients_mutex_);
    return clients_.size();
}

void WebSocketServer::capture_initial_states() {
    // Capture current state of all entities as their initial states
    auto& mgr = engine_.get_entity_manager();
    mgr.for_each([this](const jaguar::physics::Entity& entity) {
        initial_entity_states_[entity.id] = engine_.get_entity_state(entity.id);
    });
}

// ============================================================================
// Message Broadcasting
// ============================================================================

void WebSocketServer::broadcast_world_state() {
    std::string message = serialize_world_state();
    broadcast_message(message);
}

void WebSocketServer::broadcast_message(const std::string& message) {
    std::lock_guard<std::mutex> lock(clients_mutex_);
    for (auto* client : clients_) {
        send_message(client, message);
    }
}

void WebSocketServer::send_message(lws* wsi, const std::string& message) {
    // Queue the message and request writeable callback
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        message_queues_[wsi].push(message);
    }
    // Request the callback to actually write the data
    lws_callback_on_writable(wsi);
}

// ============================================================================
// JSON Parsing Helper
// ============================================================================

// Simple JSON value extraction - for production, use a proper JSON library
static std::string find_json_value(const std::string& json, const std::string& key, size_t start_pos = 0) {
    std::string search = "\"" + key + "\"";
    auto pos = json.find(search, start_pos);
    if (pos == std::string::npos) return "";

    pos = json.find(':', pos);
    if (pos == std::string::npos) return "";

    pos = json.find_first_not_of(" \t\n\r", pos + 1);
    if (pos == std::string::npos) return "";

    if (json[pos] == '"') {
        auto end = json.find('"', pos + 1);
        if (end != std::string::npos) {
            return json.substr(pos + 1, end - pos - 1);
        }
    } else if (json[pos] == '{') {
        // Find matching closing brace for nested object
        int depth = 1;
        size_t end = pos + 1;
        while (end < json.size() && depth > 0) {
            if (json[end] == '{') depth++;
            else if (json[end] == '}') depth--;
            end++;
        }
        return json.substr(pos, end - pos);
    } else {
        auto end = json.find_first_of(",}\n", pos);
        if (end != std::string::npos) {
            return json.substr(pos, end - pos);
        }
    }
    return "";
}

// ============================================================================
// Message Handling
// ============================================================================

void WebSocketServer::handle_message(lws* wsi, const std::string& message) {
    // Simple JSON parsing (for production, use a proper JSON library)
    // Expected format: {"type": "command", "data": {...}}

    std::string type = find_json_value(message, "type");

    if (type == "command") {
        // Find the data object first
        std::string data = find_json_value(message, "data");
        // Then find the command inside data
        std::string command = find_json_value(data, "command");
        handle_command(command, message, wsi);
    } else if (type == "entity_spawn") {
        handle_entity_spawn(message, wsi);
    } else if (type == "entity_destroy") {
        handle_entity_destroy(message, wsi);
    }
}

void WebSocketServer::handle_command(const std::string& command, const std::string& params, lws* wsi) {
    bool should_broadcast = false;  // Flag to trigger immediate broadcast after state changes

    if (command == "start") {
        status_ = SimulationStatus::Running;
        should_broadcast = true;
    } else if (command == "pause") {
        status_ = SimulationStatus::Paused;
        should_broadcast = true;
    } else if (command == "stop") {
        status_ = SimulationStatus::Stopped;
        should_broadcast = true;
    } else if (command == "reset") {
        status_ = SimulationStatus::Idle;
        // Reset all entities to their initial positions
        for (const auto& [entity_id, initial_state] : initial_entity_states_) {
            engine_.set_entity_state(entity_id, initial_state);
        }
        // Clear any accumulated forces
        engine_.get_entity_manager().get_state_storage().clear_forces();
        should_broadcast = true;
    } else if (command == "set_time_scale") {
        // Parse scale from params
        auto pos = params.find("\"scale\"");
        if (pos != std::string::npos) {
            pos = params.find(':', pos);
            if (pos != std::string::npos) {
                try {
                    double scale = std::stod(params.substr(pos + 1));
                    engine_.set_time_scale(scale);
                } catch (...) {}
            }
        }
    }
    // Air domain controls
    else if (command == "set_flight_controls") {
        handle_set_flight_controls(params, wsi);
    } else if (command == "set_autopilot") {
        handle_set_autopilot(params, wsi);
    } else if (command == "set_waypoints") {
        handle_set_waypoints(params, wsi);
    }
    // Land domain controls
    else if (command == "set_vehicle_controls") {
        handle_set_vehicle_controls(params, wsi);
    } else if (command == "set_vehicle_autopilot") {
        handle_set_vehicle_autopilot(params, wsi);
    }
    // Sea domain controls
    else if (command == "set_ship_controls") {
        handle_set_ship_controls(params, wsi);
    } else if (command == "set_ship_autopilot") {
        handle_set_ship_autopilot(params, wsi);
    }
    // Space domain controls
    else if (command == "set_space_controls") {
        handle_set_space_controls(params, wsi);
    } else if (command == "set_space_autopilot") {
        handle_set_space_autopilot(params, wsi);
    }

    // Broadcast world state immediately after status changes for responsive UI
    if (should_broadcast) {
        broadcast_world_state();
    }
}

void WebSocketServer::handle_entity_spawn(const std::string& message, lws* wsi) {
    using namespace jaguar;
    using namespace jaguar::coord;

    // Parse spawn request and create entity
    // Extract the data object first
    std::string data = find_json_value(message, "data");

    // Extract domain
    std::string domain_str = find_json_value(data, "domain");
    Domain domain = Domain::Air;
    if (domain_str == "land") domain = Domain::Land;
    else if (domain_str == "sea") domain = Domain::Sea;
    else if (domain_str == "space") domain = Domain::Space;

    // Extract name (default to "Spawned Entity")
    std::string name = find_json_value(data, "name");
    if (name.empty()) name = "Spawned Entity";

    // Create entity
    auto id = engine_.create_entity(name, domain);
    if (id != INVALID_ENTITY_ID) {
        // Extract position if provided
        std::string position_obj = find_json_value(data, "position");
        physics::EntityState state;

        double lat = 37.5665;  // Default: Seoul
        double lon = 126.978;
        double alt = 5000.0;

        if (!position_obj.empty()) {
            // Parse position values
            std::string lon_str = find_json_value(position_obj, "longitude");
            std::string lat_str = find_json_value(position_obj, "latitude");
            std::string alt_str = find_json_value(position_obj, "altitude");

            lon = !lon_str.empty() ? std::stod(lon_str) : 126.978;
            lat = !lat_str.empty() ? std::stod(lat_str) : 37.5665;
            alt = !alt_str.empty() ? std::stod(alt_str) : 5000.0;
        }

        // Convert geodetic (lat, lon, alt) to ECEF coordinates
        GeodeticPosition lla = GeodeticPosition::from_degrees(lat, lon, alt);
        state.position = lla_to_ecef(lla);

        // Set default velocity based on domain (in NED, then convert to ECEF)
        double default_speed = 0.0;
        double default_mass = 1000.0;
        switch (domain) {
            case Domain::Air:
                default_speed = 100.0;  // 100 m/s (~360 km/h) for aircraft
                default_mass = 10000.0;  // 10 tons
                break;
            case Domain::Land:
                default_speed = 10.0;   // 10 m/s (~36 km/h) for ground vehicles
                default_mass = 30000.0; // 30 tons
                break;
            case Domain::Sea:
                default_speed = 5.0;    // 5 m/s (~10 knots) for ships
                default_mass = 5000000.0; // 5000 tons
                break;
            case Domain::Space:
                default_speed = 7000.0; // 7 km/s orbital velocity
                default_mass = 100000.0; // 100 tons
                break;
        }

        // Set velocity (eastward by default in NED frame, then convert to ECEF)
        Vec3 vel_ned{0, default_speed, 0};  // North=0, East=speed, Down=0
        state.velocity = ned_to_ecef(vel_ned, lla);

        // Set orientation (body aligned with local NED frame)
        state.orientation = ned_quat_to_ecef_quat(Quat::Identity(), lla);

        // Set mass and inertia
        state.mass = default_mass;
        Real inertia_scale = default_mass * 10.0;
        state.inertia = Mat3x3::Diagonal(inertia_scale, inertia_scale, inertia_scale);

        std::cout << "Spawning entity '" << name << "' at lat=" << lat << ", lon=" << lon << ", alt=" << alt
                  << " speed=" << default_speed << " m/s, mass=" << default_mass << " kg\n";

        engine_.set_entity_state(id, state);

        // Store initial state for reset functionality
        initial_entity_states_[id] = state;

        // Send confirmation
        std::ostringstream ss;
        ss << "{\"type\":\"entity_spawned\",\"data\":{\"id\":\"entity_" << id << "\",\"name\":\"" << name << "\",\"domain\":\"" << domain_str << "\"}}";
        send_message(wsi, ss.str());
    }
}

void WebSocketServer::handle_entity_destroy(const std::string& message, lws* wsi) {
    // Extract the data object first
    std::string data = find_json_value(message, "data");

    // Parse entity ID (format: "entity_X" or just a number)
    std::string entity_id_str = find_json_value(data, "entityId");
    if (!entity_id_str.empty()) {
        try {
            // Remove "entity_" prefix if present
            if (entity_id_str.find("entity_") == 0) {
                entity_id_str = entity_id_str.substr(7);
            }
            jaguar::EntityId id = static_cast<jaguar::EntityId>(std::stoul(entity_id_str));
            engine_.destroy_entity(id);

            // Remove from initial states map
            initial_entity_states_.erase(id);

            // Send confirmation
            std::ostringstream ss;
            ss << "{\"type\":\"entity_destroyed\",\"data\":{\"id\":\"entity_" << id << "\"}}";
            send_message(wsi, ss.str());
        } catch (...) {}
    }
}

void WebSocketServer::handle_set_flight_controls(const std::string& message, lws* wsi) {
    using namespace jaguar;
    using namespace jaguar::coord;

    // Extract the data object (contains command, entityId, params)
    std::string data = find_json_value(message, "data");

    // Parse entity ID
    std::string entity_id_str = find_json_value(data, "entityId");
    if (entity_id_str.empty()) {
        send_message(wsi, serialize_error("Missing entityId"));
        return;
    }

    // Remove "entity_" prefix if present
    if (entity_id_str.find("entity_") == 0) {
        entity_id_str = entity_id_str.substr(7);
    }

    try {
        EntityId entity_id = static_cast<EntityId>(std::stoul(entity_id_str));

        // Check if entity exists
        if (!engine_.entity_exists(entity_id)) {
            send_message(wsi, serialize_error("Entity not found"));
            return;
        }

        // Extract params object
        std::string params = find_json_value(data, "params");
        if (params.empty()) {
            params = data;
        }

        // Parse control values
        std::string elevator_str = find_json_value(params, "elevator");
        std::string aileron_str = find_json_value(params, "aileron");
        std::string rudder_str = find_json_value(params, "rudder");
        std::string throttle_str = find_json_value(params, "throttle");

        double elevator = !elevator_str.empty() ? std::stod(elevator_str) : 0.0;
        double aileron = !aileron_str.empty() ? std::stod(aileron_str) : 0.0;
        double rudder = !rudder_str.empty() ? std::stod(rudder_str) : 0.0;
        double throttle = !throttle_str.empty() ? std::stod(throttle_str) : 0.5;

        // Get current entity state
        physics::EntityState state = engine_.get_entity_state(entity_id);
        GeodeticPosition lla = ecef_to_lla(state.position);
        Vec3 vel_ned = ecef_to_ned(state.velocity, lla);

        // Calculate current speed
        double current_speed = std::sqrt(vel_ned.x * vel_ned.x + vel_ned.y * vel_ned.y);
        double current_heading = std::atan2(vel_ned.y, vel_ned.x);

        // Apply flight control effects (simplified flight model)
        // Throttle affects speed (target speed = throttle * max_speed)
        double max_speed = 200.0;  // m/s for aircraft
        double target_speed = throttle * max_speed;
        double speed_change = std::clamp((target_speed - current_speed) * 0.1, -5.0, 5.0);
        double new_speed = std::max(10.0, current_speed + speed_change);

        // Elevator affects pitch/vertical speed
        double vertical_speed_change = elevator * 20.0;  // m/s per unit elevator
        vel_ned.z = std::clamp(-vertical_speed_change, -50.0, 50.0);  // NED: negative z is up

        // Aileron/rudder affect heading
        double heading_change = (aileron + rudder) * 0.05;  // radians per tick
        double new_heading = current_heading + heading_change;

        // Reconstruct velocity
        vel_ned.x = new_speed * std::cos(new_heading);
        vel_ned.y = new_speed * std::sin(new_heading);

        // Convert back to ECEF and update state
        state.velocity = ned_to_ecef(vel_ned, lla);
        engine_.set_entity_state(entity_id, state);

        std::cout << "Flight controls set for entity " << entity_id
                  << ": elev=" << elevator << ", ail=" << aileron
                  << ", rud=" << rudder << ", thr=" << throttle << "\n";

    } catch (const std::exception& e) {
        send_message(wsi, serialize_error("Failed to set flight controls: " + std::string(e.what())));
    }
}

void WebSocketServer::handle_set_autopilot(const std::string& message, lws* wsi) {
    // Extract the data object (contains command, entityId, params)
    std::string data = find_json_value(message, "data");

    // Parse entity ID
    std::string entity_id_str = find_json_value(data, "entityId");
    if (entity_id_str.empty()) {
        send_message(wsi, serialize_error("Missing entityId"));
        return;
    }

    // Remove "entity_" prefix if present
    if (entity_id_str.find("entity_") == 0) {
        entity_id_str = entity_id_str.substr(7);
    }

    try {
        jaguar::EntityId entity_id = static_cast<jaguar::EntityId>(std::stoul(entity_id_str));

        // Check if entity exists
        if (!engine_.entity_exists(entity_id)) {
            send_message(wsi, serialize_error("Entity not found"));
            return;
        }

        // Extract params object (mode, altitude, heading, speed are inside params)
        std::string params = find_json_value(data, "params");
        if (params.empty()) {
            // Fallback: params might be at the data level directly
            params = data;
        }

        // Parse autopilot mode (accept both uppercase and lowercase)
        std::string mode_str = find_json_value(params, "mode");
        AutopilotMode mode = AutopilotMode::OFF;

        // Convert mode string to uppercase for comparison
        std::string mode_upper = mode_str;
        std::transform(mode_upper.begin(), mode_upper.end(), mode_upper.begin(), ::toupper);

        if (mode_upper == "OFF") mode = AutopilotMode::OFF;
        else if (mode_upper == "ALTITUDE_HOLD") mode = AutopilotMode::ALTITUDE_HOLD;
        else if (mode_upper == "HEADING_HOLD") mode = AutopilotMode::HEADING_HOLD;
        else if (mode_upper == "SPEED_HOLD") mode = AutopilotMode::SPEED_HOLD;
        else if (mode_upper == "NAV") mode = AutopilotMode::NAV;

        // Get or create autopilot command for this entity
        AutopilotCommand& cmd = autopilot_commands_[entity_id];
        cmd.mode = mode;

        // Parse optional target values from params (accept both "target_X" and "X" naming)
        std::string alt_str = find_json_value(params, "target_altitude");
        if (alt_str.empty()) {
            alt_str = find_json_value(params, "altitude");
        }
        if (!alt_str.empty()) {
            double requested_alt = std::stod(alt_str);
            // Enforce minimum altitude for aircraft (prevent underground flying)
            constexpr double MIN_AIRCRAFT_ALTITUDE = 100.0;  // meters above ground
            cmd.target_altitude = std::max(requested_alt, MIN_AIRCRAFT_ALTITUDE);
            if (requested_alt < MIN_AIRCRAFT_ALTITUDE) {
                std::cout << "WARNING: Clamped requested altitude " << requested_alt
                          << " to minimum " << MIN_AIRCRAFT_ALTITUDE << "m\n";
            }
        }

        std::string hdg_str = find_json_value(params, "target_heading");
        if (hdg_str.empty()) {
            hdg_str = find_json_value(params, "heading");
        }
        if (!hdg_str.empty()) {
            cmd.target_heading = std::stod(hdg_str) * M_PI / 180.0;  // Convert deg to rad
        }

        std::string spd_str = find_json_value(params, "target_speed");
        if (spd_str.empty()) {
            spd_str = find_json_value(params, "speed");
        }
        if (!spd_str.empty()) {
            cmd.target_speed = std::stod(spd_str);
        }

        // Send confirmation
        std::ostringstream ss;
        ss << "{\"type\":\"autopilot_set\",\"data\":{";
        ss << "\"entityId\":\"entity_" << entity_id << "\",";
        ss << "\"mode\":\"" << mode_str << "\",";
        ss << "\"target_altitude\":" << cmd.target_altitude << ",";
        ss << "\"target_heading\":" << (cmd.target_heading * 180.0 / M_PI) << ",";
        ss << "\"target_speed\":" << cmd.target_speed;
        ss << "}}";
        send_message(wsi, ss.str());

        std::cout << "Autopilot set for entity " << entity_id << ": mode=" << mode_str
                  << ", alt=" << cmd.target_altitude << ", hdg=" << (cmd.target_heading * 180.0 / M_PI)
                  << ", spd=" << cmd.target_speed << "\n";

    } catch (const std::exception& e) {
        send_message(wsi, serialize_error("Failed to set autopilot: " + std::string(e.what())));
    }
}

void WebSocketServer::handle_set_waypoints(const std::string& message, lws* wsi) {
    using namespace jaguar;
    using namespace jaguar::coord;

    // Extract the data object
    std::string data = find_json_value(message, "data");

    // Parse entity ID
    std::string entity_id_str = find_json_value(data, "entityId");
    if (entity_id_str.empty()) {
        send_message(wsi, serialize_error("Missing entityId"));
        return;
    }

    // Remove "entity_" prefix if present
    if (entity_id_str.find("entity_") == 0) {
        entity_id_str = entity_id_str.substr(7);
    }

    try {
        EntityId entity_id = static_cast<EntityId>(std::stoul(entity_id_str));

        // Check if entity exists
        if (!engine_.entity_exists(entity_id)) {
            send_message(wsi, serialize_error("Entity not found"));
            return;
        }

        // Get or create autopilot command for this entity
        AutopilotCommand& cmd = autopilot_commands_[entity_id];
        cmd.flight_plan.clear();

        // Find waypoints array - simple parsing for array of objects
        // Format: "waypoints":[{...},{...}]
        size_t wp_start = data.find("\"waypoints\"");
        if (wp_start == std::string::npos) {
            send_message(wsi, serialize_error("Missing waypoints array"));
            return;
        }

        size_t arr_start = data.find('[', wp_start);
        if (arr_start == std::string::npos) {
            send_message(wsi, serialize_error("Invalid waypoints format"));
            return;
        }

        // Parse each waypoint object in the array
        size_t pos = arr_start + 1;
        while (pos < data.size()) {
            // Find next waypoint object
            size_t obj_start = data.find('{', pos);
            if (obj_start == std::string::npos) break;

            // Find matching closing brace
            int depth = 1;
            size_t obj_end = obj_start + 1;
            while (obj_end < data.size() && depth > 0) {
                if (data[obj_end] == '{') depth++;
                else if (data[obj_end] == '}') depth--;
                obj_end++;
            }

            std::string wp_obj = data.substr(obj_start, obj_end - obj_start);

            // Parse waypoint fields
            Waypoint wp;
            wp.name = find_json_value(wp_obj, "name");
            if (wp.name.empty()) {
                wp.name = "WP" + std::to_string(cmd.flight_plan.size() + 1);
            }

            std::string lat_str = find_json_value(wp_obj, "latitude");
            std::string lon_str = find_json_value(wp_obj, "longitude");
            std::string alt_str = find_json_value(wp_obj, "altitude");
            std::string spd_str = find_json_value(wp_obj, "speed");
            std::string type_str = find_json_value(wp_obj, "type");

            double lat = !lat_str.empty() ? std::stod(lat_str) : 0.0;
            double lon = !lon_str.empty() ? std::stod(lon_str) : 0.0;
            wp.altitude = !alt_str.empty() ? std::stod(alt_str) : 5000.0;
            wp.speed = !spd_str.empty() ? std::stod(spd_str) : 0.0;

            // Convert geodetic to ECEF position
            GeodeticPosition lla = GeodeticPosition::from_degrees(lat, lon, wp.altitude);
            wp.position = lla_to_ecef(lla);

            // Parse waypoint type
            if (type_str == "FLY_OVER") {
                wp.type = Waypoint::Type::FLY_OVER;
            } else if (type_str == "HOLD") {
                wp.type = Waypoint::Type::HOLD;
            } else if (type_str == "INTERCEPT") {
                wp.type = Waypoint::Type::INTERCEPT;
            } else {
                wp.type = Waypoint::Type::FLY_BY;  // Default
            }

            cmd.flight_plan.push_back(wp);

            pos = obj_end;
        }

        // Auto-enable NAV mode and reset active waypoint
        if (!cmd.flight_plan.empty()) {
            cmd.mode = AutopilotMode::NAV;
            cmd.active_waypoint = 0;
        }

        // Send confirmation
        std::ostringstream ss;
        ss << "{\"type\":\"waypoints_set\",\"data\":{";
        ss << "\"entityId\":\"entity_" << entity_id << "\",";
        ss << "\"mode\":\"NAV\",";
        ss << "\"waypoint_count\":" << cmd.flight_plan.size() << ",";
        ss << "\"active_waypoint\":0";
        ss << "}}";
        send_message(wsi, ss.str());

        std::cout << "Waypoints set for entity " << entity_id << ": " << cmd.flight_plan.size() << " waypoints\n";

    } catch (const std::exception& e) {
        send_message(wsi, serialize_error("Failed to set waypoints: " + std::string(e.what())));
    }
}

// ============================================================================
// Land Domain Control Handlers
// ============================================================================

void WebSocketServer::handle_set_vehicle_controls(const std::string& message, lws* wsi) {
    using namespace jaguar;
    using namespace jaguar::coord;

    std::string data = find_json_value(message, "data");
    std::string entity_id_str = find_json_value(data, "entityId");
    if (entity_id_str.empty()) {
        send_message(wsi, serialize_error("Missing entityId"));
        return;
    }

    if (entity_id_str.find("entity_") == 0) {
        entity_id_str = entity_id_str.substr(7);
    }

    try {
        EntityId entity_id = static_cast<EntityId>(std::stoul(entity_id_str));
        if (!engine_.entity_exists(entity_id)) {
            send_message(wsi, serialize_error("Entity not found"));
            return;
        }

        std::string params = find_json_value(data, "params");
        if (params.empty()) params = data;

        // Parse vehicle control values
        std::string throttle_str = find_json_value(params, "throttle");
        std::string steering_str = find_json_value(params, "steering");
        std::string brake_str = find_json_value(params, "brake");

        double throttle = !throttle_str.empty() ? std::stod(throttle_str) : 0.0;
        double steering = !steering_str.empty() ? std::stod(steering_str) : 0.0;
        double brake = !brake_str.empty() ? std::stod(brake_str) : 0.0;

        // Clamp values
        throttle = std::max(0.0, std::min(1.0, throttle));
        steering = std::max(-1.0, std::min(1.0, steering));
        brake = std::max(0.0, std::min(1.0, brake));

        // Get or create vehicle command
        VehicleCommand& cmd = vehicle_commands_[entity_id];
        cmd.throttle = throttle;
        cmd.steering = steering;
        cmd.brake = brake;
        cmd.mode = VehicleControlMode::MANUAL;

        // Apply vehicle controls
        physics::EntityState state = engine_.get_entity_state(entity_id);
        GeodeticPosition lla = ecef_to_lla(state.position);
        Vec3 vel_ned = ecef_to_ned(state.velocity, lla);

        double current_speed = std::sqrt(vel_ned.x * vel_ned.x + vel_ned.y * vel_ned.y);
        double current_heading = std::atan2(vel_ned.y, vel_ned.x);

        // Vehicle dynamics (simplified)
        double max_speed = 30.0;  // m/s (~108 km/h)
        double target_speed = throttle * max_speed * (1.0 - brake);
        double speed_diff = target_speed - current_speed;
        double accel = std::max(-10.0, std::min(5.0, speed_diff * 0.5));
        double new_speed = std::max(0.0, current_speed + accel);

        // Steering affects heading (more at lower speeds)
        double turn_rate = steering * 0.1 * (1.0 - new_speed / max_speed * 0.5);
        double new_heading = current_heading + turn_rate;

        // Reconstruct velocity (ground vehicles stay at altitude 0)
        vel_ned.x = new_speed * std::cos(new_heading);
        vel_ned.y = new_speed * std::sin(new_heading);
        vel_ned.z = 0.0;  // No vertical movement for ground vehicles

        state.velocity = ned_to_ecef(vel_ned, lla);

        // Keep on ground
        if (lla.altitude > 10.0) {
            lla.altitude = 0.0;
            state.position = lla_to_ecef(lla);
        }

        engine_.set_entity_state(entity_id, state);

        std::cout << "Vehicle controls set for entity " << entity_id
                  << ": throttle=" << throttle << ", steering=" << steering
                  << ", brake=" << brake << "\n";

    } catch (const std::exception& e) {
        send_message(wsi, serialize_error("Failed to set vehicle controls: " + std::string(e.what())));
    }
}

void WebSocketServer::handle_set_vehicle_autopilot(const std::string& message, lws* wsi) {
    using namespace jaguar;

    std::string data = find_json_value(message, "data");
    std::string entity_id_str = find_json_value(data, "entityId");
    if (entity_id_str.empty()) {
        send_message(wsi, serialize_error("Missing entityId"));
        return;
    }

    if (entity_id_str.find("entity_") == 0) {
        entity_id_str = entity_id_str.substr(7);
    }

    try {
        EntityId entity_id = static_cast<EntityId>(std::stoul(entity_id_str));
        if (!engine_.entity_exists(entity_id)) {
            send_message(wsi, serialize_error("Entity not found"));
            return;
        }

        std::string params = find_json_value(data, "params");
        if (params.empty()) params = data;

        VehicleCommand& cmd = vehicle_commands_[entity_id];

        // Parse mode
        std::string mode_str = find_json_value(params, "mode");
        std::string mode_upper = mode_str;
        std::transform(mode_upper.begin(), mode_upper.end(), mode_upper.begin(), ::toupper);

        if (mode_upper == "MANUAL") cmd.mode = VehicleControlMode::MANUAL;
        else if (mode_upper == "CRUISE_CONTROL" || mode_upper == "CRUISE") cmd.mode = VehicleControlMode::CRUISE_CONTROL;
        else if (mode_upper == "WAYPOINT_FOLLOW" || mode_upper == "NAV") cmd.mode = VehicleControlMode::WAYPOINT_FOLLOW;
        else if (mode_upper == "FORMATION") cmd.mode = VehicleControlMode::FORMATION;

        // Parse targets
        std::string speed_str = find_json_value(params, "speed");
        if (speed_str.empty()) speed_str = find_json_value(params, "target_speed");
        if (!speed_str.empty()) cmd.target_speed = std::stod(speed_str);

        std::string hdg_str = find_json_value(params, "heading");
        if (hdg_str.empty()) hdg_str = find_json_value(params, "target_heading");
        if (!hdg_str.empty()) cmd.target_heading = std::stod(hdg_str) * M_PI / 180.0;

        std::ostringstream ss;
        ss << "{\"type\":\"vehicle_autopilot_set\",\"data\":{";
        ss << "\"entityId\":\"entity_" << entity_id << "\",";
        ss << "\"mode\":\"" << mode_str << "\",";
        ss << "\"target_speed\":" << cmd.target_speed << ",";
        ss << "\"target_heading\":" << (cmd.target_heading * 180.0 / M_PI);
        ss << "}}";
        send_message(wsi, ss.str());

        std::cout << "Vehicle autopilot set for entity " << entity_id << ": mode=" << mode_str << "\n";

    } catch (const std::exception& e) {
        send_message(wsi, serialize_error("Failed to set vehicle autopilot: " + std::string(e.what())));
    }
}

// ============================================================================
// Sea Domain Control Handlers
// ============================================================================

void WebSocketServer::handle_set_ship_controls(const std::string& message, lws* wsi) {
    using namespace jaguar;
    using namespace jaguar::coord;

    std::string data = find_json_value(message, "data");
    std::string entity_id_str = find_json_value(data, "entityId");
    if (entity_id_str.empty()) {
        send_message(wsi, serialize_error("Missing entityId"));
        return;
    }

    if (entity_id_str.find("entity_") == 0) {
        entity_id_str = entity_id_str.substr(7);
    }

    try {
        EntityId entity_id = static_cast<EntityId>(std::stoul(entity_id_str));
        if (!engine_.entity_exists(entity_id)) {
            send_message(wsi, serialize_error("Entity not found"));
            return;
        }

        std::string params = find_json_value(data, "params");
        if (params.empty()) params = data;

        // Parse ship control values
        std::string throttle_str = find_json_value(params, "throttle");
        std::string rudder_str = find_json_value(params, "rudder");

        double throttle = !throttle_str.empty() ? std::stod(throttle_str) : 0.0;
        double rudder = !rudder_str.empty() ? std::stod(rudder_str) : 0.0;

        // Clamp values
        throttle = std::max(-1.0, std::min(1.0, throttle));  // Ships can reverse
        rudder = std::max(-1.0, std::min(1.0, rudder));

        // Get or create ship command
        ShipCommand& cmd = ship_commands_[entity_id];
        cmd.throttle = throttle;
        cmd.rudder = rudder;
        cmd.mode = ShipControlMode::MANUAL;

        // Apply ship controls
        physics::EntityState state = engine_.get_entity_state(entity_id);
        GeodeticPosition lla = ecef_to_lla(state.position);
        Vec3 vel_ned = ecef_to_ned(state.velocity, lla);

        double current_speed = std::sqrt(vel_ned.x * vel_ned.x + vel_ned.y * vel_ned.y);
        double current_heading = std::atan2(vel_ned.y, vel_ned.x);

        // Ship dynamics (simplified, slower than vehicles)
        double max_speed_forward = 15.0;   // m/s (~29 knots)
        double max_speed_reverse = 5.0;    // m/s (~10 knots)
        double target_speed = throttle >= 0
            ? throttle * max_speed_forward
            : throttle * max_speed_reverse;

        double speed_diff = target_speed - current_speed;
        double accel = std::max(-2.0, std::min(1.0, speed_diff * 0.1));  // Ships accelerate slowly
        double new_speed = current_speed + accel;

        // Rudder affects heading (ships turn slowly, more at higher speeds)
        double turn_rate = rudder * 0.02 * std::abs(new_speed) / max_speed_forward;
        double new_heading = current_heading + turn_rate;

        // Reconstruct velocity (ships stay at sea level)
        vel_ned.x = new_speed * std::cos(new_heading);
        vel_ned.y = new_speed * std::sin(new_heading);
        vel_ned.z = 0.0;  // Stay at sea level

        state.velocity = ned_to_ecef(vel_ned, lla);

        // Keep at sea level
        if (std::abs(lla.altitude) > 5.0) {
            lla.altitude = 0.0;
            state.position = lla_to_ecef(lla);
        }

        engine_.set_entity_state(entity_id, state);

        std::cout << "Ship controls set for entity " << entity_id
                  << ": throttle=" << throttle << ", rudder=" << rudder << "\n";

    } catch (const std::exception& e) {
        send_message(wsi, serialize_error("Failed to set ship controls: " + std::string(e.what())));
    }
}

void WebSocketServer::handle_set_ship_autopilot(const std::string& message, lws* wsi) {
    using namespace jaguar;

    std::string data = find_json_value(message, "data");
    std::string entity_id_str = find_json_value(data, "entityId");
    if (entity_id_str.empty()) {
        send_message(wsi, serialize_error("Missing entityId"));
        return;
    }

    if (entity_id_str.find("entity_") == 0) {
        entity_id_str = entity_id_str.substr(7);
    }

    try {
        EntityId entity_id = static_cast<EntityId>(std::stoul(entity_id_str));
        if (!engine_.entity_exists(entity_id)) {
            send_message(wsi, serialize_error("Entity not found"));
            return;
        }

        std::string params = find_json_value(data, "params");
        if (params.empty()) params = data;

        ShipCommand& cmd = ship_commands_[entity_id];

        // Parse mode
        std::string mode_str = find_json_value(params, "mode");
        std::string mode_upper = mode_str;
        std::transform(mode_upper.begin(), mode_upper.end(), mode_upper.begin(), ::toupper);

        if (mode_upper == "MANUAL") cmd.mode = ShipControlMode::MANUAL;
        else if (mode_upper == "AUTOPILOT" || mode_upper == "HEADING_HOLD") cmd.mode = ShipControlMode::AUTOPILOT;
        else if (mode_upper == "WAYPOINT_NAV" || mode_upper == "NAV") cmd.mode = ShipControlMode::WAYPOINT_NAV;
        else if (mode_upper == "STATION_KEEPING" || mode_upper == "STATION") cmd.mode = ShipControlMode::STATION_KEEPING;

        // Parse targets
        std::string speed_str = find_json_value(params, "speed");
        if (speed_str.empty()) speed_str = find_json_value(params, "target_speed");
        if (!speed_str.empty()) cmd.target_speed = std::stod(speed_str);

        std::string hdg_str = find_json_value(params, "heading");
        if (hdg_str.empty()) hdg_str = find_json_value(params, "target_heading");
        if (!hdg_str.empty()) cmd.target_heading = std::stod(hdg_str) * M_PI / 180.0;

        std::ostringstream ss;
        ss << "{\"type\":\"ship_autopilot_set\",\"data\":{";
        ss << "\"entityId\":\"entity_" << entity_id << "\",";
        ss << "\"mode\":\"" << mode_str << "\",";
        ss << "\"target_speed\":" << cmd.target_speed << ",";
        ss << "\"target_heading\":" << (cmd.target_heading * 180.0 / M_PI);
        ss << "}}";
        send_message(wsi, ss.str());

        std::cout << "Ship autopilot set for entity " << entity_id << ": mode=" << mode_str << "\n";

    } catch (const std::exception& e) {
        send_message(wsi, serialize_error("Failed to set ship autopilot: " + std::string(e.what())));
    }
}

// ============================================================================
// Space Domain Control Handlers
// ============================================================================

void WebSocketServer::handle_set_space_controls(const std::string& message, lws* wsi) {
    using namespace jaguar;
    using namespace jaguar::coord;

    std::string data = find_json_value(message, "data");
    std::string entity_id_str = find_json_value(data, "entityId");
    if (entity_id_str.empty()) {
        send_message(wsi, serialize_error("Missing entityId"));
        return;
    }

    if (entity_id_str.find("entity_") == 0) {
        entity_id_str = entity_id_str.substr(7);
    }

    try {
        EntityId entity_id = static_cast<EntityId>(std::stoul(entity_id_str));
        if (!engine_.entity_exists(entity_id)) {
            send_message(wsi, serialize_error("Entity not found"));
            return;
        }

        std::string params = find_json_value(data, "params");
        if (params.empty()) params = data;

        // Parse space control values (thrust and rotation)
        std::string thrust_x_str = find_json_value(params, "thrust_x");
        std::string thrust_y_str = find_json_value(params, "thrust_y");
        std::string thrust_z_str = find_json_value(params, "thrust_z");
        std::string roll_str = find_json_value(params, "roll_rate");
        std::string pitch_str = find_json_value(params, "pitch_rate");
        std::string yaw_str = find_json_value(params, "yaw_rate");

        double thrust_x = !thrust_x_str.empty() ? std::stod(thrust_x_str) : 0.0;
        double thrust_y = !thrust_y_str.empty() ? std::stod(thrust_y_str) : 0.0;
        double thrust_z = !thrust_z_str.empty() ? std::stod(thrust_z_str) : 0.0;

        // Clamp thrust values
        thrust_x = std::max(-1.0, std::min(1.0, thrust_x));
        thrust_y = std::max(-1.0, std::min(1.0, thrust_y));
        thrust_z = std::max(-1.0, std::min(1.0, thrust_z));

        // Get or create space command
        SpaceCommand& cmd = space_commands_[entity_id];
        cmd.thrust_x = thrust_x;
        cmd.thrust_y = thrust_y;
        cmd.thrust_z = thrust_z;
        if (!roll_str.empty()) cmd.roll_rate = std::stod(roll_str);
        if (!pitch_str.empty()) cmd.pitch_rate = std::stod(pitch_str);
        if (!yaw_str.empty()) cmd.yaw_rate = std::stod(yaw_str);
        cmd.mode = SpaceControlMode::MANEUVER;

        // Apply space controls
        physics::EntityState state = engine_.get_entity_state(entity_id);

        // Space thrust (simplified - just add delta-v in spacecraft frame)
        // In reality this would be more complex with orbital mechanics
        double thrust_magnitude = 10.0;  // m/s^2 acceleration
        Vec3 delta_v(
            thrust_x * thrust_magnitude * 0.05,  // dt = 0.05s
            thrust_y * thrust_magnitude * 0.05,
            thrust_z * thrust_magnitude * 0.05
        );

        // Add delta-v to current velocity (simplified, ignoring orientation)
        state.velocity = state.velocity + delta_v;

        engine_.set_entity_state(entity_id, state);

        std::cout << "Space controls set for entity " << entity_id
                  << ": thrust=(" << thrust_x << "," << thrust_y << "," << thrust_z << ")\n";

    } catch (const std::exception& e) {
        send_message(wsi, serialize_error("Failed to set space controls: " + std::string(e.what())));
    }
}

void WebSocketServer::handle_set_space_autopilot(const std::string& message, lws* wsi) {
    using namespace jaguar;

    std::string data = find_json_value(message, "data");
    std::string entity_id_str = find_json_value(data, "entityId");
    if (entity_id_str.empty()) {
        send_message(wsi, serialize_error("Missing entityId"));
        return;
    }

    if (entity_id_str.find("entity_") == 0) {
        entity_id_str = entity_id_str.substr(7);
    }

    try {
        EntityId entity_id = static_cast<EntityId>(std::stoul(entity_id_str));
        if (!engine_.entity_exists(entity_id)) {
            send_message(wsi, serialize_error("Entity not found"));
            return;
        }

        std::string params = find_json_value(data, "params");
        if (params.empty()) params = data;

        SpaceCommand& cmd = space_commands_[entity_id];

        // Parse mode
        std::string mode_str = find_json_value(params, "mode");
        std::string mode_upper = mode_str;
        std::transform(mode_upper.begin(), mode_upper.end(), mode_upper.begin(), ::toupper);

        if (mode_upper == "DRIFT" || mode_upper == "OFF") cmd.mode = SpaceControlMode::DRIFT;
        else if (mode_upper == "ATTITUDE_HOLD" || mode_upper == "ATTITUDE") cmd.mode = SpaceControlMode::ATTITUDE_HOLD;
        else if (mode_upper == "ORBIT_MAINTAIN" || mode_upper == "STATION") cmd.mode = SpaceControlMode::ORBIT_MAINTAIN;
        else if (mode_upper == "MANEUVER") cmd.mode = SpaceControlMode::MANEUVER;

        // Parse target attitude (if provided)
        std::string roll_str = find_json_value(params, "target_roll");
        std::string pitch_str = find_json_value(params, "target_pitch");
        std::string yaw_str = find_json_value(params, "target_yaw");

        if (!roll_str.empty()) cmd.target_attitude.x = std::stod(roll_str) * M_PI / 180.0;
        if (!pitch_str.empty()) cmd.target_attitude.y = std::stod(pitch_str) * M_PI / 180.0;
        if (!yaw_str.empty()) cmd.target_attitude.z = std::stod(yaw_str) * M_PI / 180.0;

        std::ostringstream ss;
        ss << "{\"type\":\"space_autopilot_set\",\"data\":{";
        ss << "\"entityId\":\"entity_" << entity_id << "\",";
        ss << "\"mode\":\"" << mode_str << "\"";
        ss << "}}";
        send_message(wsi, ss.str());

        std::cout << "Space autopilot set for entity " << entity_id << ": mode=" << mode_str << "\n";

    } catch (const std::exception& e) {
        send_message(wsi, serialize_error("Failed to set space autopilot: " + std::string(e.what())));
    }
}

// ============================================================================
// Autopilot Guidance Application
// ============================================================================

void WebSocketServer::apply_autopilot_guidance() {
    using namespace jaguar;
    using namespace jaguar::coord;

    // Minimum safe altitudes by domain
    constexpr double MIN_AIR_ALTITUDE = 100.0;    // meters
    constexpr double MIN_GROUND_ALTITUDE = 0.0;   // ground level
    constexpr double MIN_SEA_ALTITUDE = 0.0;      // sea level
    constexpr double MIN_SPACE_ALTITUDE = 100000.0; // 100km (Karman line)

    // First pass: enforce ground collision for ALL air entities
    auto& mgr = engine_.get_entity_manager();
    mgr.for_each([&](const physics::Entity& entity) {
        if (entity.primary_domain != Domain::Air) return;

        physics::EntityState state = engine_.get_entity_state(entity.id);
        GeodeticPosition lla = ecef_to_lla(state.position);

        // If aircraft is below minimum altitude, force emergency climb
        if (lla.altitude < MIN_AIR_ALTITUDE) {
            Vec3 vel_ned = ecef_to_ned(state.velocity, lla);

            // Force strong upward velocity
            double climb_rate = std::max(50.0, (MIN_AIR_ALTITUDE - lla.altitude) * 0.5);
            vel_ned.z = -climb_rate;  // NED: negative z is up

            // Maintain some minimum forward speed
            double horiz_speed = std::sqrt(vel_ned.x * vel_ned.x + vel_ned.y * vel_ned.y);
            if (horiz_speed < 50.0) {
                double heading = std::atan2(vel_ned.y, vel_ned.x);
                vel_ned.x = 50.0 * std::cos(heading);
                vel_ned.y = 50.0 * std::sin(heading);
            }

            state.velocity = ned_to_ecef(vel_ned, lla);

            // Also fix position if underground
            if (lla.altitude < 0) {
                lla.altitude = MIN_AIR_ALTITUDE;
                state.position = lla_to_ecef(lla);
            }

            engine_.set_entity_state(entity.id, state);
        }
    });

    // Second pass: apply autopilot commands
    for (auto& [entity_id, cmd] : autopilot_commands_) {
        // Skip if autopilot is off
        if (cmd.mode == AutopilotMode::OFF) {
            continue;
        }

        // Check if entity still exists
        if (!engine_.entity_exists(entity_id)) {
            continue;
        }

        // Get current entity state
        physics::EntityState state = engine_.get_entity_state(entity_id);
        bool state_modified = false;

        // Convert current position to geodetic for altitude calculations
        GeodeticPosition current_lla = ecef_to_lla(state.position);
        double current_altitude = current_lla.altitude;

        // Convert velocity to NED for direction calculations
        Vec3 vel_ned = ecef_to_ned(state.velocity, current_lla);
        double current_speed = std::sqrt(vel_ned.x * vel_ned.x + vel_ned.y * vel_ned.y);
        double current_heading = std::atan2(vel_ned.y, vel_ned.x);  // radians

        // AI Agent Enhancement: Apply ALL specified parameters regardless of mode
        // This allows the AI to control altitude, heading, AND speed simultaneously
        // The mode indicates primary control intent, but all parameters are honored

        double new_heading = current_heading;
        double new_speed = current_speed;
        double new_vertical = vel_ned.z;

        // Apply heading control if target heading is set (any mode)
        if (cmd.target_heading != 0.0 || cmd.mode == AutopilotMode::HEADING_HOLD) {
            double heading_error = cmd.target_heading - current_heading;

            // Normalize to [-pi, pi]
            while (heading_error > M_PI) heading_error -= 2.0 * M_PI;
            while (heading_error < -M_PI) heading_error += 2.0 * M_PI;

            // Gradual turn toward target heading
            double turn_rate = std::clamp(heading_error * 0.5, -0.2, 0.2);  // radians per tick
            new_heading = current_heading + turn_rate;
            state_modified = true;
        }

        // Apply altitude control if target altitude is set (any mode)
        if (cmd.target_altitude > 0.0 || cmd.mode == AutopilotMode::ALTITUDE_HOLD) {
            double alt_error = cmd.target_altitude - current_altitude;
            double climb_rate = std::clamp(alt_error * 0.1, -50.0, 50.0);  // Max 50 m/s climb/descend
            new_vertical = -climb_rate;  // NED: negative z is up
            state_modified = true;
        }

        // Apply speed control if target speed is set (any mode)
        if (cmd.target_speed > 0.0 || cmd.mode == AutopilotMode::SPEED_HOLD) {
            double target_speed = cmd.target_speed > 0.0 ? cmd.target_speed : 100.0;  // Default 100 m/s
            double speed_error = target_speed - current_speed;
            double accel = std::clamp(speed_error * 0.1, -10.0, 10.0);  // m/s per tick
            new_speed = std::max(10.0, current_speed + accel);  // Minimum 10 m/s
            state_modified = true;
        }

        // Handle NAV mode separately (it has its own velocity calculation)
        if (cmd.mode == AutopilotMode::NAV) {
            // Waypoint navigation mode
            if (cmd.flight_plan.empty() || cmd.active_waypoint >= cmd.flight_plan.size()) {
                continue;
            }

            const Waypoint& wp = cmd.flight_plan[cmd.active_waypoint];

            // Calculate vector to waypoint (in ECEF)
            Vec3 to_wp = wp.position - state.position;
            double distance = to_wp.length();

            // Check if waypoint reached (within 500m)
            constexpr double WAYPOINT_CAPTURE_RADIUS = 500.0;
            if (distance < WAYPOINT_CAPTURE_RADIUS) {
                cmd.active_waypoint++;
                std::cout << "Entity " << entity_id << " reached waypoint " << wp.name << "\n";

                if (cmd.active_waypoint >= cmd.flight_plan.size()) {
                    std::cout << "Entity " << entity_id << " completed flight plan\n";
                    cmd.mode = AutopilotMode::OFF;
                    continue;
                }
                continue;
            }

            // Determine target speed
            double nav_target_speed = wp.speed > 0 ? wp.speed : cmd.target_speed;
            if (nav_target_speed <= 0) {
                nav_target_speed = current_speed > 10.0 ? current_speed : 100.0;
            }

            // Set velocity toward waypoint at target speed
            Vec3 direction = to_wp.normalized();
            state.velocity = direction * nav_target_speed;
            state_modified = true;
        } else if (state_modified) {
            // For ALTITUDE_HOLD, HEADING_HOLD, SPEED_HOLD modes:
            // Apply the combined new_heading, new_speed, new_vertical values
            vel_ned.x = new_speed * std::cos(new_heading);
            vel_ned.y = new_speed * std::sin(new_heading);
            vel_ned.z = new_vertical;

            // Convert back to ECEF
            state.velocity = ned_to_ecef(vel_ned, current_lla);
        }

        // Update entity state if modified
        if (state_modified) {
            engine_.set_entity_state(entity_id, state);
        }
    }

    // Third pass: apply vehicle (land) autopilot commands
    for (auto& [entity_id, cmd] : vehicle_commands_) {
        if (cmd.mode == VehicleControlMode::MANUAL) continue;
        if (!engine_.entity_exists(entity_id)) continue;

        physics::EntityState state = engine_.get_entity_state(entity_id);
        GeodeticPosition lla = ecef_to_lla(state.position);
        Vec3 vel_ned = ecef_to_ned(state.velocity, lla);

        double current_speed = std::sqrt(vel_ned.x * vel_ned.x + vel_ned.y * vel_ned.y);
        double current_heading = std::atan2(vel_ned.y, vel_ned.x);
        bool modified = false;

        // Cruise control - maintain target speed
        if (cmd.mode == VehicleControlMode::CRUISE_CONTROL && cmd.target_speed > 0) {
            double speed_error = cmd.target_speed - current_speed;
            double accel = std::max(-5.0, std::min(3.0, speed_error * 0.3));
            double new_speed = std::max(0.0, current_speed + accel);

            vel_ned.x = new_speed * std::cos(current_heading);
            vel_ned.y = new_speed * std::sin(current_heading);
            modified = true;
        }

        // Also apply heading if set
        if (cmd.target_heading != 0.0) {
            double heading_error = cmd.target_heading - current_heading;
            while (heading_error > M_PI) heading_error -= 2.0 * M_PI;
            while (heading_error < -M_PI) heading_error += 2.0 * M_PI;

            double turn_rate = std::max(-0.1, std::min(0.1, heading_error * 0.3));
            double new_heading = current_heading + turn_rate;
            double speed = std::sqrt(vel_ned.x * vel_ned.x + vel_ned.y * vel_ned.y);

            vel_ned.x = speed * std::cos(new_heading);
            vel_ned.y = speed * std::sin(new_heading);
            modified = true;
        }

        if (modified) {
            vel_ned.z = 0.0;  // Keep on ground
            state.velocity = ned_to_ecef(vel_ned, lla);

            // Enforce ground level
            if (lla.altitude > 10.0 || lla.altitude < -10.0) {
                lla.altitude = 0.0;
                state.position = lla_to_ecef(lla);
            }

            engine_.set_entity_state(entity_id, state);
        }
    }

    // Fourth pass: apply ship (sea) autopilot commands
    for (auto& [entity_id, cmd] : ship_commands_) {
        if (cmd.mode == ShipControlMode::MANUAL) continue;
        if (!engine_.entity_exists(entity_id)) continue;

        physics::EntityState state = engine_.get_entity_state(entity_id);
        GeodeticPosition lla = ecef_to_lla(state.position);
        Vec3 vel_ned = ecef_to_ned(state.velocity, lla);

        double current_speed = std::sqrt(vel_ned.x * vel_ned.x + vel_ned.y * vel_ned.y);
        double current_heading = std::atan2(vel_ned.y, vel_ned.x);
        bool modified = false;

        // Autopilot - heading and speed hold
        if (cmd.mode == ShipControlMode::AUTOPILOT) {
            // Speed control
            if (cmd.target_speed > 0) {
                double speed_error = cmd.target_speed - current_speed;
                double accel = std::max(-1.0, std::min(0.5, speed_error * 0.1));
                current_speed = std::max(0.0, current_speed + accel);
                modified = true;
            }

            // Heading control
            if (cmd.target_heading != 0.0) {
                double heading_error = cmd.target_heading - current_heading;
                while (heading_error > M_PI) heading_error -= 2.0 * M_PI;
                while (heading_error < -M_PI) heading_error += 2.0 * M_PI;

                double turn_rate = std::max(-0.05, std::min(0.05, heading_error * 0.2));
                current_heading += turn_rate;
                modified = true;
            }
        }

        if (modified) {
            vel_ned.x = current_speed * std::cos(current_heading);
            vel_ned.y = current_speed * std::sin(current_heading);
            vel_ned.z = 0.0;  // Stay at sea level
            state.velocity = ned_to_ecef(vel_ned, lla);

            // Enforce sea level
            if (std::abs(lla.altitude) > 5.0) {
                lla.altitude = 0.0;
                state.position = lla_to_ecef(lla);
            }

            engine_.set_entity_state(entity_id, state);
        }
    }

    // Fifth pass: apply space autopilot commands
    for (auto& [entity_id, cmd] : space_commands_) {
        if (cmd.mode == SpaceControlMode::DRIFT) continue;
        if (!engine_.entity_exists(entity_id)) continue;

        physics::EntityState state = engine_.get_entity_state(entity_id);
        bool modified = false;

        // Orbit maintain - simple station keeping
        if (cmd.mode == SpaceControlMode::ORBIT_MAINTAIN) {
            // In reality this would involve complex orbital mechanics
            // For now, just maintain current velocity (no drag in space)
            modified = false;  // No change needed
        }

        // Attitude hold - maintain target attitude (simplified)
        if (cmd.mode == SpaceControlMode::ATTITUDE_HOLD) {
            // Would require quaternion/rotation tracking
            // Simplified: no velocity change
            modified = false;
        }

        if (modified) {
            engine_.set_entity_state(entity_id, state);
        }
    }
}

// ============================================================================
// State Collection
// ============================================================================

WorldState WebSocketServer::collect_world_state() {
    auto now = std::chrono::steady_clock::now();

    // Update FPS calculation (rolling average over 1 second)
    frame_count_++;
    auto fps_elapsed = std::chrono::duration<double>(now - last_fps_update_).count();
    if (fps_elapsed >= 1.0) {
        current_fps_ = frame_count_ / fps_elapsed;
        frame_count_ = 0;
        last_fps_update_ = now;
    }

    // Calculate wall clock time since start
    double wall_clock = std::chrono::duration<double>(now - start_time_).count();

    WorldState world;
    world.entities = collect_entities();
    world.status = status_;
    world.simulation_time = engine_.get_time();
    world.wall_clock_time = wall_clock;
    world.delta_time = 0.05;  // 20 Hz tick
    world.realtime_ratio = (wall_clock > 0) ? (world.simulation_time / wall_clock) : 1.0;
    world.frame_rate = current_fps_ > 0 ? current_fps_ : 20.0;  // Default to broadcast rate
    world.physics_time = physics_time_;
    world.total_entities = static_cast<int>(world.entities.size());
    world.active_entities = world.total_entities;

    // Count by domain
    world.entities_by_domain[jaguar::Domain::Air] = 0;
    world.entities_by_domain[jaguar::Domain::Land] = 0;
    world.entities_by_domain[jaguar::Domain::Sea] = 0;
    world.entities_by_domain[jaguar::Domain::Space] = 0;

    for (const auto& e : world.entities) {
        world.entities_by_domain[e.domain]++;
    }

    return world;
}

std::vector<EntityData> WebSocketServer::collect_entities() {
    std::vector<EntityData> entities;
    auto& mgr = engine_.get_entity_manager();

    mgr.for_each([&](const jaguar::physics::Entity& entity) {
        EntityData data;
        data.id = entity.id;
        data.name = entity.name;
        data.domain = entity.primary_domain;
        data.state = engine_.get_entity_state(entity.id);
        data.is_active = entity.active;
        data.health = 100.0;  // Would come from damage system
        data.damage = 0.0;
        entities.push_back(data);
    });

    return entities;
}

// ============================================================================
// JSON Serialization
// ============================================================================

std::string WebSocketServer::serialize_world_state() {
    auto world = collect_world_state();
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(6);

    ss << "{\"type\":\"world_state\",\"timestamp\":" << std::chrono::duration_cast<std::chrono::milliseconds>(
           std::chrono::system_clock::now().time_since_epoch()).count()
       << ",\"data\":{";

    // Entities array
    ss << "\"entities\":[";
    bool first = true;
    for (const auto& e : world.entities) {
        if (!first) ss << ",";
        first = false;
        ss << serialize_entity(e);
    }
    ss << "],";

    // Stats
    ss << "\"stats\":{";
    ss << "\"simulationTime\":" << world.simulation_time << ",";
    ss << "\"wallClockTime\":" << world.wall_clock_time << ",";
    ss << "\"deltaTime\":" << world.delta_time << ",";
    ss << "\"realtimeRatio\":" << world.realtime_ratio << ",";
    ss << "\"frameRate\":" << world.frame_rate << ",";
    ss << "\"physicsTime\":" << world.physics_time << ",";
    ss << "\"totalEntities\":" << world.total_entities << ",";
    ss << "\"activeEntities\":" << world.active_entities << ",";
    ss << "\"entitiesByDomain\":{";
    ss << "\"air\":" << world.entities_by_domain[jaguar::Domain::Air] << ",";
    ss << "\"land\":" << world.entities_by_domain[jaguar::Domain::Land] << ",";
    ss << "\"sea\":" << world.entities_by_domain[jaguar::Domain::Sea] << ",";
    ss << "\"space\":" << world.entities_by_domain[jaguar::Domain::Space];
    ss << "},";
    ss << "\"collisionChecks\":0,\"activeCollisions\":0";
    ss << "},";

    // Status
    const char* status_str = "idle";
    switch (world.status) {
        case SimulationStatus::Running: status_str = "running"; break;
        case SimulationStatus::Paused: status_str = "paused"; break;
        case SimulationStatus::Stopped: status_str = "stopped"; break;
        default: break;
    }
    ss << "\"status\":\"" << status_str << "\"";
    ss << "}}";

    return ss.str();
}

std::string WebSocketServer::serialize_entity(const EntityData& e) {
    using namespace jaguar;
    using namespace jaguar::coord;

    std::ostringstream ss;
    ss << std::fixed << std::setprecision(6);

    // Map domain to string
    const char* domain_str = "air";
    switch (e.domain) {
        case Domain::Land: domain_str = "land"; break;
        case Domain::Sea: domain_str = "sea"; break;
        case Domain::Space: domain_str = "space"; break;
        default: break;
    }

    ss << "{";
    ss << "\"id\":\"entity_" << e.id << "\",";
    ss << "\"name\":\"" << e.name << "\",";
    ss << "\"domain\":\"" << domain_str << "\",";
    ss << "\"kind\":\"platform\",";
    ss << "\"isActive\":" << (e.is_active ? "true" : "false") << ",";
    ss << "\"health\":" << e.health << ",";
    ss << "\"damage\":" << e.damage << ",";

    // Convert ECEF position to geodetic (lat, lon, alt)
    GeodeticPosition lla = ecef_to_lla(e.state.position);
    double lat_deg = math::rad_to_deg(lla.latitude);
    double lon_deg = math::rad_to_deg(lla.longitude);
    double alt_m = lla.altitude;

    ss << "\"position\":{";
    ss << "\"longitude\":" << lon_deg << ",";
    ss << "\"latitude\":" << lat_deg << ",";
    ss << "\"altitude\":" << alt_m;
    ss << "},";

    // Convert ECEF velocity to NED velocity for display
    Vec3 vel_ned = ecef_to_ned(e.state.velocity, lla);
    double vel_north_ms = vel_ned.x;
    double vel_east_ms = vel_ned.y;
    double vel_down_ms = vel_ned.z;

    ss << "\"velocity\":{";
    ss << "\"north\":" << vel_north_ms << ",";
    ss << "\"east\":" << vel_east_ms << ",";
    ss << "\"down\":" << vel_down_ms;
    ss << "},";

    // Orientation (simplified - would convert quaternion to Euler)
    ss << "\"orientation\":{";
    ss << "\"roll\":0,\"pitch\":0,\"yaw\":0";
    ss << "},";

    // Calculate horizontal speed in m/s
    double speed_ms = std::sqrt(vel_north_ms * vel_north_ms + vel_east_ms * vel_east_ms);

    // Domain-specific data
    if (e.domain == Domain::Air) {
        ss << "\"airspeed\":" << speed_ms << ",";
        ss << "\"machNumber\":" << (speed_ms / 343.0) << ",";
        ss << "\"altitude\":" << alt_m << ",";
        ss << "\"verticalSpeed\":" << (-vel_down_ms) << ",";  // Positive = climbing
        ss << "\"heading\":0,";
        ss << "\"throttle\":0.7,\"fuel\":80,\"gearDown\":false,\"flapsPosition\":0,";
        ss << "\"aoa\":2,\"sideslip\":0,\"gForce\":1.0,";
        ss << "\"elevator\":0,\"aileron\":0,\"rudder\":0";
    } else if (e.domain == Domain::Land) {
        ss << "\"speed\":" << speed_ms << ",";
        ss << "\"heading\":0,\"throttle\":0.5,\"steering\":0,\"brake\":0,";
        ss << "\"fuel\":70,\"engineRPM\":1500,\"gear\":3,";
        ss << "\"groundContact\":true,\"sinkage\":0.02";
    } else if (e.domain == Domain::Sea) {
        ss << "\"speed\":" << speed_ms << ",";
        ss << "\"heading\":0,\"roll\":0,\"pitch\":0,\"heave\":0,";
        ss << "\"rudder\":0,\"throttle\":0.5,";
        ss << "\"draft\":8,\"displacement\":9000,\"waveHeight\":2,\"seaState\":3";
    } else if (e.domain == Domain::Space) {
        ss << "\"semiMajorAxis\":6778,\"eccentricity\":0.001,";
        ss << "\"inclination\":51.6,\"raan\":0,\"argOfPerigee\":0,\"trueAnomaly\":0,";
        ss << "\"altitude\":" << (alt_m / 1000.0) << ",";  // km
        ss << "\"velocity\":" << speed_ms << ",";  // m/s
        ss << "\"period\":92,\"power\":95,\"fuel\":100";
    }

    ss << "}";
    return ss.str();
}

std::string WebSocketServer::serialize_error(const std::string& error) {
    return "{\"type\":\"error\",\"message\":\"" + error + "\"}";
}

// ============================================================================
// WebSocket Callback
// ============================================================================

int WebSocketServer::callback_websocket(lws* wsi, enum lws_callback_reasons reason, void* user, void* in, size_t len) {
    if (!instance_) return 0;

    switch (static_cast<lws_callback_reasons>(reason)) {
        case LWS_CALLBACK_ESTABLISHED: {
            {
                std::lock_guard<std::mutex> lock(instance_->clients_mutex_);
                instance_->clients_.push_back(wsi);
                std::cout << "Client connected (total: " << instance_->clients_.size() << ")\n";
            }
            // Initialize message queue for this client
            {
                std::lock_guard<std::mutex> lock(instance_->queue_mutex_);
                instance_->message_queues_[wsi] = std::queue<std::string>();
            }
            // Send initial world state immediately to new client
            std::string initial_state = instance_->serialize_world_state();
            instance_->send_message(wsi, initial_state);
            std::cout << "Sent initial world state to new client\n";
            break;
        }

        case LWS_CALLBACK_CLOSED: {
            {
                std::lock_guard<std::mutex> lock(instance_->clients_mutex_);
                auto it = std::find(instance_->clients_.begin(), instance_->clients_.end(), wsi);
                if (it != instance_->clients_.end()) {
                    instance_->clients_.erase(it);
                }
                std::cout << "Client disconnected (total: " << instance_->clients_.size() << ")\n";
            }
            // Clean up message queue for this client
            {
                std::lock_guard<std::mutex> lock(instance_->queue_mutex_);
                instance_->message_queues_.erase(wsi);
            }
            break;
        }

        case LWS_CALLBACK_SERVER_WRITEABLE: {
            // Send queued messages
            std::string message;
            {
                std::lock_guard<std::mutex> lock(instance_->queue_mutex_);
                auto it = instance_->message_queues_.find(wsi);
                if (it != instance_->message_queues_.end() && !it->second.empty()) {
                    message = std::move(it->second.front());
                    it->second.pop();
                }
            }

            if (!message.empty()) {
                // LWS requires pre-padding
                std::vector<unsigned char> buf(LWS_PRE + message.size());
                std::memcpy(buf.data() + LWS_PRE, message.data(), message.size());
                lws_write(wsi, buf.data() + LWS_PRE, message.size(), LWS_WRITE_TEXT);

                // If there are more messages, request another writeable callback
                {
                    std::lock_guard<std::mutex> lock(instance_->queue_mutex_);
                    auto it = instance_->message_queues_.find(wsi);
                    if (it != instance_->message_queues_.end() && !it->second.empty()) {
                        lws_callback_on_writable(wsi);
                    }
                }
            }
            break;
        }

        case LWS_CALLBACK_RECEIVE: {
            std::string message(static_cast<char*>(in), len);
            instance_->handle_message(wsi, message);
            break;
        }

        default:
            break;
    }

    return 0;
}
