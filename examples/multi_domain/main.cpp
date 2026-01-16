/**
 * @file main.cpp
 * @brief Multi-domain simulation example
 */

#include "jaguar/jaguar.h"
#include <iostream>
#include <iomanip>

int main() {
    std::cout << "JaguarEngine Multi-Domain Example\n";
    std::cout << "Version: " << jaguar::GetVersionString() << "\n\n";

    jaguar::Engine engine;
    if (!engine.initialize()) {
        std::cerr << "Failed to initialize engine\n";
        return 1;
    }

    // Create entities in different domains
    auto aircraft = engine.create_entity("F-16", jaguar::Domain::Air);
    auto tank = engine.create_entity("M1A2", jaguar::Domain::Land);
    auto ship = engine.create_entity("DDG-51", jaguar::Domain::Sea);
    auto satellite = engine.create_entity("GPS-IIR", jaguar::Domain::Space);

    std::cout << "Created entities:\n";
    std::cout << "  Aircraft (Air):    ID " << aircraft << "\n";
    std::cout << "  Tank (Land):       ID " << tank << "\n";
    std::cout << "  Ship (Sea):        ID " << ship << "\n";
    std::cout << "  Satellite (Space): ID " << satellite << "\n\n";

    // Set initial states
    {
        jaguar::physics::EntityState s;
        s.position = {0, 0, -10000};  // 10km altitude
        s.velocity = {250, 0, 0};
        s.mass = 12000;
        engine.set_entity_state(aircraft, s);
    }
    {
        jaguar::physics::EntityState s;
        s.position = {1000, 0, 0};  // On ground
        s.velocity = {10, 0, 0};    // Moving
        s.mass = 60000;
        engine.set_entity_state(tank, s);
    }
    {
        jaguar::physics::EntityState s;
        s.position = {0, 5000, 0};  // At sea
        s.velocity = {15, 0, 0};
        s.mass = 9000000;  // 9000 tons
        engine.set_entity_state(ship, s);
    }
    {
        jaguar::physics::EntityState s;
        s.position = {0, 0, -400000};  // 400km altitude (LEO)
        s.velocity = {0, 7700, 0};     // Orbital velocity
        s.mass = 2000;
        engine.set_entity_state(satellite, s);
    }

    // Run simulation
    std::cout << "Running multi-domain simulation...\n\n";

    const double dt = 0.01;
    engine.run_for(5.0);  // 5 seconds

    std::cout << "Final states:\n\n";

    auto print_state = [&](const char* name, jaguar::EntityId id) {
        auto s = engine.get_entity_state(id);
        std::cout << name << ":\n";
        std::cout << "  Position: (" << s.position.x << ", "
                  << s.position.y << ", " << s.position.z << ") m\n";
        std::cout << "  Velocity: (" << s.velocity.x << ", "
                  << s.velocity.y << ", " << s.velocity.z << ") m/s\n\n";
    };

    print_state("Aircraft", aircraft);
    print_state("Tank", tank);
    print_state("Ship", ship);
    print_state("Satellite", satellite);

    std::cout << "Multi-domain simulation complete.\n";

    engine.shutdown();
    return 0;
}
