/**
 * @file main.cpp
 * @brief JaguarEngine WebSocket Server
 *
 * Real-time simulation server that bridges JaguarEngine with web clients
 * via WebSocket protocol. Provides entity state updates, simulation control,
 * and command handling.
 */

#include "jaguar/jaguar.h"
#include "jaguar/core/coordinates.h"
#include "websocket_server.h"
#include <iostream>
#include <thread>
#include <atomic>
#include <chrono>
#include <csignal>

// Global shutdown flag
std::atomic<bool> g_running{true};

void signal_handler(int signal) {
    std::cout << "\nReceived signal " << signal << ", shutting down...\n";
    g_running = false;
}

int main(int argc, char* argv[]) {
    // Parse command line arguments
    uint16_t port = 8081;
    double time_scale = 1.0;
    double tick_rate = 20.0;  // Hz

    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--port" && i + 1 < argc) {
            port = static_cast<uint16_t>(std::stoi(argv[++i]));
        } else if (arg == "--time-scale" && i + 1 < argc) {
            time_scale = std::stod(argv[++i]);
        } else if (arg == "--tick-rate" && i + 1 < argc) {
            tick_rate = std::stod(argv[++i]);
        } else if (arg == "--help") {
            std::cout << "JaguarEngine WebSocket Server\n\n"
                      << "Usage: " << argv[0] << " [options]\n\n"
                      << "Options:\n"
                      << "  --port <port>        WebSocket port (default: 8081)\n"
                      << "  --time-scale <scale> Time scale multiplier (default: 1.0)\n"
                      << "  --tick-rate <hz>     Simulation tick rate (default: 20)\n"
                      << "  --help               Show this help\n";
            return 0;
        }
    }

    // Setup signal handlers
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    std::cout << "========================================\n";
    std::cout << "   JaguarEngine WebSocket Server\n";
    std::cout << "========================================\n";
    std::cout << "Version: " << jaguar::GetVersionString() << "\n\n";

    // Initialize engine
    jaguar::Engine engine;
    if (!engine.initialize()) {
        std::cerr << "Failed to initialize JaguarEngine\n";
        return 1;
    }
    std::cout << "Engine initialized successfully\n";

    // Create WebSocket server
    WebSocketServer server(engine, port);
    if (!server.start()) {
        std::cerr << "Failed to start WebSocket server\n";
        return 1;
    }
    std::cout << "WebSocket server listening on ws://localhost:" << port << "\n\n";

    // Create some initial entities for demonstration
    // NOTE: Position and velocity are in ECEF (Earth-Centered, Earth-Fixed) coordinates.
    //       The physics engine works in ECEF frame:
    //       - Position: ECEF (meters)
    //       - Velocity: ECEF (m/s)
    //       - Orientation: Body-to-ECEF quaternion
    auto create_initial_entities = [&engine]() {
        using namespace jaguar;
        using namespace jaguar::coord;

        // Helper to create entity state from geodetic position and NED velocity
        auto create_state_from_geodetic = [](
            Real lat_deg, Real lon_deg, Real alt_m,
            Real vel_north_ms, Real vel_east_ms, Real vel_down_ms,
            Real mass_kg
        ) -> physics::EntityState {
            physics::EntityState state;

            // Convert geodetic position to ECEF
            GeodeticPosition lla = GeodeticPosition::from_degrees(lat_deg, lon_deg, alt_m);
            state.position = lla_to_ecef(lla);

            // Convert NED velocity to ECEF velocity
            Vec3 vel_ned{vel_north_ms, vel_east_ms, vel_down_ms};
            state.velocity = ned_to_ecef(vel_ned, lla);

            // Initialize orientation as identity (body aligned with ECEF)
            // For proper simulation, this should be set based on heading/attitude
            state.orientation = ned_quat_to_ecef_quat(Quat::Identity(), lla);

            state.mass = mass_kg;

            // Initialize diagonal inertia tensor (simplified)
            Real inertia_scale = mass_kg * 10.0;  // Rough approximation
            state.inertia = Mat3x3::Diagonal(inertia_scale, inertia_scale, inertia_scale);

            return state;
        };

        // Air entities (F-16 cruises at ~250 m/s)
        for (int i = 0; i < 3; i++) {
            auto id = engine.create_entity("F-16 Eagle #" + std::to_string(i + 1), Domain::Air);
            if (id != INVALID_ENTITY_ID) {
                Real lat = 37.5665 + (i - 1) * 0.3;
                Real lon = 126.9780 + (i - 1) * 0.5;
                Real alt = 5000.0 + i * 2000.0;
                Real speed = 150.0 + i * 50;  // m/s eastward

                auto state = create_state_from_geodetic(lat, lon, alt, 0, speed, 0, 12000);
                engine.set_entity_state(id, state);
            }
        }

        // Land entities (M1 Abrams max speed ~70 km/h ≈ 20 m/s)
        for (int i = 0; i < 2; i++) {
            auto id = engine.create_entity("M1 Abrams #" + std::to_string(i + 1), Domain::Land);
            if (id != INVALID_ENTITY_ID) {
                Real lat = 37.4 + i * 0.05;
                Real lon = 127.0 + i * 0.1;
                Real alt = 100.0;
                Real speed = 15.0;  // m/s eastward

                auto state = create_state_from_geodetic(lat, lon, alt, 0, speed, 0, 60000);
                engine.set_entity_state(id, state);
            }
        }

        // Sea entities (DDG-51 Burke max speed ~30 knots ≈ 15 m/s)
        for (int i = 0; i < 2; i++) {
            auto id = engine.create_entity("DDG-51 Burke #" + std::to_string(i + 1), Domain::Sea);
            if (id != INVALID_ENTITY_ID) {
                Real lat = 37.0 + i * 0.1;
                Real lon = 126.5 + i * 0.2;
                Real alt = 0.0;  // Sea level
                Real speed = 8.0;  // m/s eastward

                auto state = create_state_from_geodetic(lat, lon, alt, 0, speed, 0, 9000000);
                engine.set_entity_state(id, state);
            }
        }

        // Space entity (ISS orbital velocity ~7660 m/s)
        auto space_id = engine.create_entity("ISS", Domain::Space);
        if (space_id != INVALID_ENTITY_ID) {
            Real lat = 37.5;
            Real lon = 127.0;
            Real alt = 408000.0;  // ~408 km altitude
            Real speed = 7660.0;  // m/s eastward (simplified - real ISS has complex orbit)

            auto state = create_state_from_geodetic(lat, lon, alt, 0, speed, 0, 420000);
            engine.set_entity_state(space_id, state);
        }
    };

    create_initial_entities();
    std::cout << "Created initial entities\n";

    // Capture initial entity states for reset functionality
    server.capture_initial_states();

    // Set time scale
    engine.set_time_scale(time_scale);
    std::cout << "Time scale: " << time_scale << "x\n";
    std::cout << "Tick rate: " << tick_rate << " Hz\n\n";

    std::cout << "Server ready. Press Ctrl+C to stop.\n";
    std::cout << "========================================\n\n";

    // Main simulation loop
    const auto tick_interval = std::chrono::microseconds(
        static_cast<int64_t>(1000000.0 / tick_rate)
    );
    auto last_tick = std::chrono::steady_clock::now();
    auto last_broadcast = last_tick;
    const auto broadcast_interval = std::chrono::milliseconds(50);  // 20 Hz broadcast

    while (g_running) {
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration<double>(now - last_tick).count();

        // Process WebSocket messages FIRST so commands take effect immediately
        server.poll();

        // Step simulation if running
        // NOTE: Only pass elapsed time - the engine handles time_scale internally
        if (server.get_simulation_status() == SimulationStatus::Running) {
            server.apply_autopilot_guidance();  // Apply waypoint navigation
            engine.step(elapsed);
        }
        last_tick = now;

        // Broadcast world state at regular intervals (after commands are processed)
        if (now - last_broadcast >= broadcast_interval) {
            server.broadcast_world_state();
            last_broadcast = now;
        }

        // Sleep until next tick
        auto tick_end = std::chrono::steady_clock::now();
        auto tick_duration = tick_end - now;
        if (tick_duration < tick_interval) {
            std::this_thread::sleep_for(tick_interval - tick_duration);
        }
    }

    // Cleanup
    std::cout << "\nShutting down...\n";
    server.stop();
    engine.shutdown();
    std::cout << "Server stopped.\n";

    return 0;
}
