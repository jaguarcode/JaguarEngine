# Contributing to JaguarEngine

Thank you for your interest in contributing to JaguarEngine! This guide outlines the process and guidelines for contributing to the project.

## Table of Contents

1. [Code of Conduct](#code-of-conduct)
2. [Getting Started](#getting-started)
3. [Development Setup](#development-setup)
4. [Coding Standards](#coding-standards)
5. [Testing Guidelines](#testing-guidelines)
6. [Documentation](#documentation)
7. [Submitting Changes](#submitting-changes)
8. [Review Process](#review-process)

## Code of Conduct

### Our Standards

- Be respectful and inclusive
- Provide constructive feedback
- Focus on technical merit
- Accept criticism gracefully
- Prioritize project goals

### Expected Behavior

- Professional communication
- Collaborative problem-solving
- Respectful disagreement
- Support for newcomers

## Getting Started

### Finding Work

1. **Good First Issues**: Look for issues labeled `good-first-issue`
2. **Bug Fixes**: Check `bug` labeled issues
3. **Feature Requests**: Review `enhancement` labeled issues
4. **Documentation**: Help improve docs with `documentation` label

### Before You Start

1. Check existing issues and pull requests
2. For major changes, open an issue first to discuss
3. Fork the repository
4. Create a feature branch

## Development Setup

### Prerequisites

```bash
# Required tools
- CMake 3.20+
- C++20 compiler (GCC 11+, Clang 14+, MSVC 2022+)
- Git

# Optional for full development
- Doxygen (documentation)
- clang-format (code formatting)
- clang-tidy (static analysis)
- lcov (coverage reports)
```

### Build Setup

```bash
# Clone your fork
git clone https://github.com/YOUR_USERNAME/JaguarEngine.git
cd JaguarEngine

# Create build directory
cmake -B build \
    -DCMAKE_BUILD_TYPE=Debug \
    -DJAGUAR_BUILD_TESTS=ON \
    -DJAGUAR_BUILD_EXAMPLES=ON \
    -DJAGUAR_ENABLE_COVERAGE=ON

# Build
cmake --build build -j$(nproc)

# Run tests
ctest --test-dir build --output-on-failure
```

### IDE Setup

#### VS Code
```json
// .vscode/settings.json
{
    "cmake.buildDirectory": "${workspaceFolder}/build",
    "cmake.configureSettings": {
        "CMAKE_BUILD_TYPE": "Debug",
        "JAGUAR_BUILD_TESTS": "ON"
    },
    "C_Cpp.default.configurationProvider": "ms-vscode.cmake-tools"
}
```

#### CLion
- Open project directory
- Configure CMake profile with test flags
- Set build type to Debug

## Coding Standards

### C++ Style Guide

We follow a consistent style based on the Google C++ Style Guide with modifications.

#### Naming Conventions

```cpp
// Namespaces: lowercase with underscores
namespace jaguar::domain::air { }

// Classes/Structs: PascalCase
class AerodynamicsModel { };
struct EntityState { };

// Functions/Methods: snake_case
void compute_forces();
Real get_altitude() const;

// Variables: snake_case
Real air_density;
Vec3 position_ecef;

// Member variables: trailing underscore
class Example {
    Real value_;
    Vec3 position_;
};

// Constants: UPPER_SNAKE_CASE or k prefix
constexpr Real G0 = 9.80665;
constexpr Real kMaxIterations = 100;

// Enum values: PascalCase
enum class Domain { Air, Land, Sea, Space };

// Template parameters: PascalCase
template<typename ValueType>
class Container { };
```

#### Formatting

```cpp
// Braces: Allman style for functions, K&R for control flow
void function()
{
    if (condition) {
        // code
    } else {
        // code
    }

    for (int i = 0; i < n; ++i) {
        // code
    }
}

// Indentation: 4 spaces (no tabs)
class Example {
    void method()
    {
        if (condition) {
            statement;
        }
    }
};

// Line length: 100 characters max
// Exception: URLs, include paths

// Pointer/reference alignment: left-aligned
int* ptr;
const std::string& ref;
```

#### Headers

```cpp
#pragma once  // Use pragma once, not include guards

// Include order (with blank lines between groups):
// 1. Corresponding header (for .cpp files)
// 2. C system headers
// 3. C++ standard library
// 4. Third-party libraries
// 5. Project headers

#include "jaguar/domain/air.h"  // Corresponding header

#include <cmath>                // C system

#include <vector>               // C++ standard
#include <memory>

#include <Eigen/Dense>          // Third-party

#include "jaguar/core/types.h"  // Project
#include "jaguar/physics/entity.h"
```

#### Comments

```cpp
/**
 * @brief Brief description of class/function
 *
 * Detailed description if needed.
 *
 * @param param_name Description of parameter
 * @return Description of return value
 */
Real compute_lift(Real alpha, Real velocity) const;

// Single-line comments for implementation details
Real result = 0.0;  // Initialize accumulator

// TODO: Description of future work
// FIXME: Description of known issue
// NOTE: Important implementation note
```

### clang-format

Use the provided `.clang-format` configuration:

```bash
# Format single file
clang-format -i src/file.cpp

# Format all files
find src include -name "*.cpp" -o -name "*.h" | xargs clang-format -i
```

### clang-tidy

Run static analysis:

```bash
# Generate compile_commands.json
cmake -B build -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

# Run clang-tidy
clang-tidy -p build src/file.cpp
```

## Testing Guidelines

### Test Structure

```cpp
#include <gtest/gtest.h>
#include "jaguar/domain/air.h"

namespace jaguar::test {

// Test fixture for related tests
class AerodynamicsTest : public ::testing::Test {
protected:
    void SetUp() override
    {
        model_.set_reference_area(27.87);
        model_.set_reference_chord(3.45);
        model_.set_reference_span(9.45);
    }

    domain::air::AerodynamicsModel model_;
};

// Test case naming: Test<Feature>_<Scenario>_<ExpectedResult>
TEST_F(AerodynamicsTest, ComputeForces_ZeroAlpha_NoLift)
{
    // Arrange
    physics::EntityState state;
    state.velocity = {100.0, 0.0, 0.0};

    environment::Environment env;
    env.atmosphere.density = 1.225;

    // Act
    physics::EntityForces forces;
    model_.compute_forces(state, env, 0.01, forces);

    // Assert
    EXPECT_NEAR(forces.force.z, 0.0, 1.0);  // Allow 1N tolerance
}

TEST_F(AerodynamicsTest, ComputeForces_PositiveAlpha_PositiveLift)
{
    // Arrange
    physics::EntityState state;
    state.velocity = {100.0, 0.0, 10.0};  // ~5.7 deg alpha

    environment::Environment env;
    env.atmosphere.density = 1.225;

    // Act
    physics::EntityForces forces;
    model_.compute_forces(state, env, 0.01, forces);

    // Assert
    EXPECT_GT(-forces.force.z, 0.0);  // Lift is negative Z in body frame
}

} // namespace jaguar::test
```

### Test Categories

1. **Unit Tests**: Test individual classes/functions
2. **Integration Tests**: Test component interactions
3. **Physics Tests**: Validate physics calculations against known values
4. **Performance Tests**: Benchmark critical paths

### Running Tests

```bash
# Run all tests
ctest --test-dir build --output-on-failure

# Run specific test
./build/tests/test_aerodynamics

# Run with filter
./build/tests/test_aerodynamics --gtest_filter="*ComputeForces*"

# Generate coverage report
cmake --build build --target coverage
```

### Test Coverage

- Aim for >80% line coverage
- 100% coverage on critical physics calculations
- All public API methods must have tests

## Documentation

### Code Documentation

```cpp
/**
 * @file aerodynamics.h
 * @brief Aerodynamic force models for aircraft
 */

/**
 * @class AerodynamicsModel
 * @brief 6-DOF aerodynamics model with coefficient tables
 *
 * Implements aerodynamic force and moment calculations using
 * coefficient tables or default analytical models. Supports
 * both subsonic and supersonic flight regimes.
 *
 * ## Usage
 * @code
 * AerodynamicsModel aero;
 * aero.set_reference_area(27.87);
 * aero.set_reference_chord(3.45);
 *
 * physics::EntityForces forces;
 * aero.compute_forces(state, env, dt, forces);
 * @endcode
 *
 * @see PropulsionModel
 * @see FlightControlSystem
 */
class AerodynamicsModel : public physics::IAerodynamicsModel {
public:
    /**
     * @brief Compute aerodynamic forces and moments
     *
     * Calculates lift, drag, and moment coefficients based on
     * flight conditions, then computes body-axis forces.
     *
     * @param state Current entity state (position, velocity, attitude)
     * @param env Environment conditions (atmosphere, wind)
     * @param dt Time step (seconds) - used for rate calculations
     * @param[out] forces Accumulated forces and moments
     *
     * @pre state.velocity.norm() > 0
     * @post forces contains added aerodynamic forces
     */
    void compute_forces(const physics::EntityState& state,
                        const environment::Environment& env,
                        Real dt,
                        physics::EntityForces& forces) override;
};
```

### Documentation Files

Update relevant documentation when making changes:

- API changes: Update `docs/API_REFERENCE.md`
- New features: Add to appropriate module docs in `docs/modules/`
- Examples: Update `docs/EXAMPLES.md`
- Configuration: Update `docs/CONFIGURATION.md`

### Building Documentation

```bash
# Generate Doxygen documentation
cmake --build build --target docs

# View documentation
open build/docs/html/index.html
```

## Submitting Changes

### Branch Naming

```
feature/short-description    # New features
fix/issue-number-description # Bug fixes
docs/description             # Documentation only
refactor/description         # Code refactoring
test/description             # Test additions/changes
```

### Commit Messages

Follow the Conventional Commits format:

```
<type>(<scope>): <subject>

<body>

<footer>
```

**Types:**
- `feat`: New feature
- `fix`: Bug fix
- `docs`: Documentation
- `style`: Formatting
- `refactor`: Code restructuring
- `test`: Tests
- `chore`: Maintenance

**Examples:**
```
feat(air): add stall model to aerodynamics

Implements progressive stall model with hysteresis for more
realistic post-stall behavior.

- Added stall alpha detection
- Implemented coefficient modification
- Added unit tests

Closes #123
```

```
fix(physics): correct quaternion normalization

Fixed numerical drift in quaternion integration that caused
attitude errors over long simulations.

Fixes #456
```

### Pull Request Process

1. **Create PR**: Use the PR template
2. **Description**: Explain what and why
3. **Tests**: Ensure all tests pass
4. **Documentation**: Update relevant docs
5. **Review**: Address feedback promptly

### PR Template

```markdown
## Description
Brief description of changes.

## Type of Change
- [ ] Bug fix
- [ ] New feature
- [ ] Breaking change
- [ ] Documentation update

## Testing
- [ ] Unit tests added/updated
- [ ] Integration tests pass
- [ ] Manual testing performed

## Checklist
- [ ] Code follows style guidelines
- [ ] Self-review completed
- [ ] Documentation updated
- [ ] No new warnings
```

## Review Process

### For Contributors

1. Respond to feedback promptly
2. Explain your reasoning
3. Make requested changes
4. Re-request review when ready

### For Reviewers

1. Be constructive and specific
2. Explain the "why"
3. Acknowledge good work
4. Focus on important issues first

### Approval Criteria

- All tests pass
- Code follows style guidelines
- Documentation is updated
- No unresolved comments
- At least one approval from maintainer

## Physics Model Contributions

### Adding New Physics Models

1. **Interface**: Implement appropriate interface (`IAerodynamicsModel`, etc.)
2. **Documentation**: Document equations and references
3. **Validation**: Include validation against known data
4. **Tests**: Comprehensive unit tests
5. **Example**: Update examples if needed

### Physics Validation Requirements

```cpp
/**
 * @brief Validation test against published data
 *
 * Reference: Anderson, "Fundamentals of Aerodynamics", Table 5.3
 */
TEST(AeroValidation, NACA0012_CL_Alpha)
{
    // Known values from wind tunnel data
    const std::vector<std::pair<Real, Real>> reference = {
        {0.0, 0.0},
        {5.0, 0.55},
        {10.0, 1.05},
        {15.0, 1.40},
    };

    for (const auto& [alpha_deg, cl_expected] : reference) {
        Real cl = model.compute_cl(alpha_deg * DEG_TO_RAD);
        EXPECT_NEAR(cl, cl_expected, 0.05);  // 5% tolerance
    }
}
```

## Getting Help

- **Questions**: Open a discussion
- **Bugs**: Open an issue with reproduction steps
- **Features**: Open an issue to discuss first
- **Security**: Contact maintainers directly

## License

By contributing, you agree that your contributions will be licensed under the project's license.

---

Thank you for contributing to JaguarEngine!
