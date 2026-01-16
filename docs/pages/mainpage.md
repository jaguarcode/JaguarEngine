# JaguarEngine Documentation {#mainpage}

## Multi-Domain Physics Simulation Engine

JaguarEngine is a high-fidelity, multi-domain physics simulation engine designed for defense and aerospace applications. It provides accurate modeling of entities across Air, Land, Sea, and Space domains with seamless interoperability.

### Key Features

- **Multi-Domain Physics**: Unified simulation across Air (6-DOF flight dynamics), Land (terramechanics), Sea (hydrodynamics), and Space (orbital mechanics)
- **High Fidelity Models**: Industry-standard physics models including JSBSim-compatible aerodynamics, Bekker-Wong terramechanics, ITTC-compliant hydrodynamics, and SGP4/SDP4 orbital propagation
- **Scripting Support**: Python and Lua bindings for rapid prototyping and integration
- **Network Interfaces**: DIS (IEEE 1278) protocol support for distributed simulation
- **Validation**: Comprehensive V&V test suite against NASA, NRMM, ITTC, and SGP4 reference data

### Quick Start

@code{.cpp}
#include <jaguar/interface/api.h>

int main() {
    jaguar::Engine engine;
    engine.initialize();

    // Create an aircraft entity
    auto aircraft = engine.create_entity("f16", jaguar::Domain::Air);

    // Set initial state
    auto state = engine.get_entity_state(aircraft);
    state.position = jaguar::Vec3(0, 0, -1000);  // 1000m altitude (NED)
    state.velocity = jaguar::Vec3(200, 0, 0);    // 200 m/s
    engine.set_entity_state(aircraft, state);

    // Run simulation
    for (int i = 0; i < 1000; i++) {
        engine.step(0.01);  // 100 Hz
    }

    engine.shutdown();
    return 0;
}
@endcode

### Documentation Sections

- @subpage getting_started "Getting Started Guide"
- @subpage cpp_api "C++ API Reference"
- @subpage python_api "Python API Reference"
- @subpage lua_api "Lua API Reference"
- @subpage network_interface "Network Interfaces (DIS/HLA)"
- @subpage physics_models "Physics Models"
- @subpage validation "Validation & Verification"

### Version Information

- **Version**: 0.4.0
- **License**: Proprietary
- **Platforms**: Windows, Linux, macOS

### Contact

For support and inquiries, please contact the development team.
