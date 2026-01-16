# Contributing to JaguarEngine

Thank you for your interest in contributing to JaguarEngine! This guide will help you get started.

## Getting Started

### Prerequisites

- C++20 compatible compiler (GCC 11+, Clang 14+, MSVC 2022)
- CMake 3.25+
- Git

### Setting Up Development Environment

```bash
# Fork and clone the repository
git clone https://github.com/jaguarcode/JaguarEngine.git
cd JaguarEngine

# Create a development branch
git checkout -b feature/your-feature-name

# Build with tests enabled
mkdir build && cd build
cmake .. -DJAGUAR_BUILD_TESTS=ON
make -j$(nproc)

# Run tests
./jaguar_unit_tests
```

---

## Development Workflow

### 1. Create an Issue

Before starting work, create or find an issue describing the change:

- Bug reports should include reproduction steps
- Feature requests should explain the use case
- Check existing issues to avoid duplicates

### 2. Make Changes

Follow these guidelines:

- **Code Style**: Match existing code formatting
- **Naming**: Use descriptive names (`calculate_drag_force`, not `calcDF`)
- **Comments**: Add comments for complex logic
- **Tests**: Add tests for new functionality

### 3. Test Your Changes

```bash
# Run all tests
./jaguar_unit_tests

# Run specific test suite
./jaguar_unit_tests --gtest_filter=TestAir*

# Run benchmarks (optional)
./jaguar_benchmarks
```

### 4. Submit a Pull Request

```bash
# Push your branch
git push origin feature/your-feature-name
```

Then create a pull request on GitHub with:

- Clear description of changes
- Link to related issue
- Test results

---

## Code Style Guidelines

### C++ Conventions

```cpp
// Namespaces: lowercase
namespace jaguar::domain::air {

// Classes: PascalCase
class AerodynamicsModel {
public:
    // Methods: snake_case
    void compute_forces(const EntityState& state);

    // Getters: get_prefix
    Real get_cl() const;

    // Setters: set_prefix
    void set_reference_area(Real area);

private:
    // Members: trailing underscore
    Real reference_area_;
    Real cl_;
};

// Constants: UPPER_CASE
constexpr Real MAX_MACH = 3.0;

// Type aliases: PascalCase
using EntityId = uint64_t;

} // namespace jaguar::domain::air
```

### Header Organization

```cpp
#pragma once

// Standard library
#include <string>
#include <vector>

// Third-party
#include <Eigen/Dense>

// Project headers
#include "jaguar/core/types.h"
#include "jaguar/physics/entity.h"

namespace jaguar {
// ...
}
```

### Documentation

```cpp
/// @brief Computes aerodynamic forces and moments
/// @param state Current entity state (position, velocity, orientation)
/// @param env Environmental conditions (atmosphere, terrain)
/// @param dt Time step in seconds
/// @param[out] forces Accumulated forces and torques
/// @throws std::runtime_error if state is invalid
void compute_forces(
    const EntityState& state,
    const Environment& env,
    Real dt,
    EntityForces& forces);
```

---

## Testing Guidelines

### Writing Tests

```cpp
#include <gtest/gtest.h>
#include "jaguar/domain/air.h"

class TestAerodynamics : public ::testing::Test {
protected:
    void SetUp() override {
        aero_.set_reference_area(30.0);
        aero_.set_reference_chord(3.0);
    }

    jaguar::domain::air::AerodynamicsModel aero_;
};

TEST_F(TestAerodynamics, ComputeLiftCoefficient) {
    // Arrange
    jaguar::physics::EntityState state;
    state.velocity = {100, 0, 5};  // 5° angle of attack

    // Act
    jaguar::physics::EntityForces forces;
    aero_.compute_forces(state, env_, 0.01, forces);

    // Assert
    EXPECT_NEAR(aero_.get_cl(), 0.5, 0.01);
}
```

### Test Categories

| Category | Location | Purpose |
|----------|----------|---------|
| Unit | `tests/unit/` | Individual component testing |
| Integration | `tests/integration/` | Component interaction |
| Validation | `tests/validation/` | Physics accuracy |
| Benchmark | `tests/benchmarks/` | Performance measurement |

---

## Pull Request Guidelines

### PR Checklist

- [ ] Code follows project style guidelines
- [ ] All existing tests pass
- [ ] New tests added for new functionality
- [ ] Documentation updated if needed
- [ ] No compiler warnings
- [ ] Commit messages are clear

### Commit Messages

```
component: Short description (50 chars max)

Longer explanation if needed. Wrap at 72 characters.
Explain what and why, not how.

Fixes #123
```

**Examples:**

```
air: Add stall model to aerodynamics

Implements post-stall aerodynamic behavior based on
flat plate theory for angles of attack > 25 degrees.

Fixes #42
```

```
tests: Add sea domain buoyancy validation

Validates buoyancy calculation against analytical
solution for simple hull shapes.
```

---

## Architecture Guidelines

### Adding a New Domain

1. Create header in `include/jaguar/domain/`
2. Implement in `src/domain/`
3. Add force generators
4. Create unit tests
5. Add example
6. Update documentation

### Adding a Force Generator

```cpp
// 1. Inherit from IForceGenerator
class MyForceModel : public physics::IForceGenerator {
public:
    void compute_forces(
        const EntityState& state,
        const Environment& env,
        Real dt,
        EntityForces& forces) override;
};

// 2. Add to appropriate domain
// 3. Write tests
// 4. Document parameters
```

---

## Documentation

### Updating Docs

Documentation lives in `docs/web/`. To preview changes:

```bash
# Install MkDocs
pip install mkdocs-material

# Serve locally
mkdocs serve

# View at http://localhost:8000
```

### API Documentation

Use Doxygen comments in headers. Generate with:

```bash
doxygen docs/Doxyfile
```

---

## Getting Help

- **Questions**: Open a [Discussion](https://github.com/jaguarcode/JaguarEngine/discussions)
- **Bugs**: Open an [Issue](https://github.com/jaguarcode/JaguarEngine/issues)
- **Security**: Email <behonz@gmail.com>

---

## License

By contributing, you agree that your contributions will be licensed under the project's license.
