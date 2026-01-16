/**
 * @file main.cpp
 * @brief Simple flight simulation example
 */

#include "jaguar/jaguar.h"
#include <iostream>
#include <iomanip>

int main() {
    std::cout << "JaguarEngine Simple Flight Example\n";
    std::cout << "Version: " << jaguar::GetVersionString() << "\n\n";

    // Create engine
    jaguar::Engine engine;
    if (!engine.initialize()) {
        std::cerr << "Failed to initialize engine\n";
        return 1;
    }

    // Create an aircraft entity
    auto aircraft_id = engine.create_entity("F-16", jaguar::Domain::Air);
    if (aircraft_id == jaguar::INVALID_ENTITY_ID) {
        std::cerr << "Failed to create aircraft entity\n";
        return 1;
    }

    // Set initial state
    jaguar::physics::EntityState state;
    state.position = jaguar::Vec3{0, 0, -10000};  // 10km altitude
    state.velocity = jaguar::Vec3{250, 0, 0};     // 250 m/s forward
    state.mass = 12000;  // kg
    engine.set_entity_state(aircraft_id, state);

    // Run simulation
    const double dt = 0.01;  // 100 Hz
    const double duration = 10.0;  // 10 seconds

    std::cout << "Running simulation for " << duration << " seconds...\n\n";
    std::cout << std::fixed << std::setprecision(2);
    std::cout << "Time(s)  X(m)       Y(m)       Z(m)       Vx(m/s)\n";
    std::cout << "-------  ---------  ---------  ---------  --------\n";

    double t = 0;
    while (t < duration) {
        engine.step(dt);
        t = engine.get_time();

        // Print state every second
        if (static_cast<int>(t * 100) % 100 == 0) {
            auto s = engine.get_entity_state(aircraft_id);
            std::cout << std::setw(7) << t << "  "
                      << std::setw(9) << s.position.x << "  "
                      << std::setw(9) << s.position.y << "  "
                      << std::setw(9) << s.position.z << "  "
                      << std::setw(8) << s.velocity.x << "\n";
        }
    }

    std::cout << "\nSimulation complete.\n";

    engine.shutdown();
    return 0;
}
