# Installation Guide

This guide covers building JaguarEngine from source on various platforms.

## System Requirements

### Minimum Requirements
- **CPU**: x86_64 or ARM64 processor
- **RAM**: 4 GB (8 GB recommended for development)
- **Disk**: 1 GB free space
- **OS**: Linux, macOS 12+, Windows 10+

### Compiler Requirements
- **C++ Standard**: C++20
- **GCC**: 11.0 or later
- **Clang**: 14.0 or later (AppleClang 15+)
- **MSVC**: Visual Studio 2022 (17.0+)

### Build Tools
- **CMake**: 3.25 or later
- **Make/Ninja**: Any recent version

## Dependencies

### Required (Auto-fetched)
These dependencies are automatically downloaded by CMake if not found:

| Library | Version | Purpose |
|---------|---------|---------|
| Eigen3 | 3.4.0 | Linear algebra, matrix operations |
| pugixml | 1.14 | XML configuration parsing |
| GoogleTest | 1.14.0 | Unit testing framework |
| Google Benchmark | 1.8.3 | Performance benchmarking |

### Optional Dependencies

| Library | Version | Purpose | CMake Option |
|---------|---------|---------|--------------|
| GDAL | 3.0+ | Geospatial terrain loading | Auto-detected |
| pybind11 | 2.11+ | Python bindings | `JAGUAR_BUILD_PYTHON` |
| sol2 | 3.3+ | Lua bindings | `JAGUAR_BUILD_LUA` |
| OpenDIS | - | DIS protocol | `JAGUAR_ENABLE_DIS` |
| HLA RTI | - | HLA protocol | `JAGUAR_ENABLE_HLA` |

## Quick Build

### Linux / macOS

```bash
# Clone repository
git clone https://github.com/jaguarcode/JaguarEngine.git
cd JaguarEngine

# Create build directory
mkdir build && cd build

# Configure
cmake ..

# Build (use all available cores)
make -j$(nproc)

# Run tests
./jaguar_unit_tests
```

### Windows (Visual Studio)

```powershell
# Clone repository
git clone https://github.com/jaguarcode/JaguarEngine.git
cd JaguarEngine

# Create build directory
mkdir build
cd build

# Configure for Visual Studio 2022
cmake .. -G "Visual Studio 17 2022" -A x64

# Build
cmake --build . --config Release

# Run tests
.\Release\jaguar_unit_tests.exe
```

## Detailed Platform Instructions

### Ubuntu / Debian

```bash
# Install build tools
sudo apt update
sudo apt install -y build-essential cmake git

# Optional: Install GDAL for terrain support
sudo apt install -y libgdal-dev

# Optional: Install Python development for bindings
sudo apt install -y python3-dev python3-pip

# Build
mkdir build && cd build
cmake ..
make -j$(nproc)
```

### Fedora / RHEL

```bash
# Install build tools
sudo dnf install -y gcc-c++ cmake git

# Optional: Install GDAL
sudo dnf install -y gdal-devel

# Build
mkdir build && cd build
cmake ..
make -j$(nproc)
```

### macOS

```bash
# Install Xcode Command Line Tools
xcode-select --install

# Install CMake via Homebrew
brew install cmake

# Optional: Install GDAL
brew install gdal

# Build
mkdir build && cd build
cmake ..
make -j$(sysctl -n hw.ncpu)
```

### Windows

1. Install [Visual Studio 2022](https://visualstudio.microsoft.com/) with C++ workload
2. Install [CMake](https://cmake.org/download/)
3. Install [Git](https://git-scm.com/)

```powershell
# From Developer PowerShell for VS 2022
cd JaguarEngine
mkdir build && cd build
cmake .. -G "Visual Studio 17 2022" -A x64
cmake --build . --config Release --parallel
```

## Build Configuration

### CMake Options

```bash
# Example with all options
cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DJAGUAR_BUILD_TESTS=ON \
    -DJAGUAR_BUILD_BENCHMARKS=ON \
    -DJAGUAR_BUILD_EXAMPLES=ON \
    -DJAGUAR_BUILD_PYTHON=OFF \
    -DJAGUAR_BUILD_LUA=OFF \
    -DJAGUAR_ENABLE_DIS=OFF \
    -DJAGUAR_ENABLE_HLA=OFF \
    -DJAGUAR_ENABLE_SIMD=ON
```

### Build Types

| Build Type | Description | Use Case |
|------------|-------------|----------|
| `Debug` | No optimization, full debug symbols | Development, debugging |
| `Release` | Full optimization (-O3) | Production, benchmarking |
| `RelWithDebInfo` | Optimization with debug info | Profiling |
| `MinSizeRel` | Optimize for size | Embedded systems |

```bash
cmake .. -DCMAKE_BUILD_TYPE=Release
```

### Option Reference

| Option | Default | Description |
|--------|---------|-------------|
| `JAGUAR_BUILD_TESTS` | ON | Build GoogleTest-based unit tests |
| `JAGUAR_BUILD_BENCHMARKS` | ON | Build Google Benchmark suite |
| `JAGUAR_BUILD_EXAMPLES` | ON | Build example applications |
| `JAGUAR_BUILD_PYTHON` | OFF | Build pybind11 Python module |
| `JAGUAR_BUILD_LUA` | OFF | Build sol2 Lua bindings |
| `JAGUAR_ENABLE_DIS` | OFF | Enable DIS network protocol |
| `JAGUAR_ENABLE_HLA` | OFF | Enable HLA federation |
| `JAGUAR_ENABLE_SIMD` | ON | Enable AVX2/FMA optimizations |

## Installing

### System-wide Installation

```bash
cd build
sudo cmake --install . --prefix /usr/local
```

### Custom Installation Path

```bash
cmake --install . --prefix ~/local/jaguar
```

### CMake Package Export

After installation, use in other CMake projects:

```cmake
find_package(Jaguar REQUIRED)
target_link_libraries(myapp PRIVATE Jaguar::jaguar)
```

## Verifying Installation

### Run Unit Tests

```bash
cd build
./jaguar_unit_tests
```

Expected output:
```
[==========] Running 237 tests from 41 test suites.
...
[  PASSED  ] 237 tests.
```

### Run Examples

```bash
# Simple flight simulation
./example_simple_flight

# Multi-domain simulation
./example_multi_domain
```

### Check Build Configuration

```bash
cmake -L ..   # List all options
cmake -LA ..  # List all options with advanced
```

## Troubleshooting

### Common Issues

#### Eigen3 Not Found
```
CMake Warning: Eigen3 not found, will use FetchContent
```
This is normal - Eigen3 is automatically downloaded if not installed.

#### GDAL Not Found
```
CMake Warning: GDAL not found. Terrain features will be limited.
```
Install GDAL if you need terrain loading:
- Ubuntu: `sudo apt install libgdal-dev`
- macOS: `brew install gdal`
- Windows: Use OSGeo4W installer

#### Compiler Too Old
```
error: 'span' is not a member of 'std'
```
Upgrade to a C++20 compatible compiler:
- GCC 11+ (Ubuntu 22.04+ default)
- Clang 14+
- MSVC 2022

#### Build Fails on ARM Mac (Rosetta)
```
error: -march=native requires native target
```
The build system automatically detects Rosetta and disables native optimizations.

### Getting Help

1. Check existing [GitHub Issues](https://github.com/jaguarcode/JaguarEngine/issues)
2. Search the [documentation](docs/)
3. Open a new issue with:
   - OS and version
   - Compiler version (`g++ --version`)
   - CMake version (`cmake --version`)
   - Full error output

## Next Steps

- Read the [Quick Start](../README.md#quick-start)
- Explore [Examples](EXAMPLES.md)
- Review [API Reference](API_REFERENCE.md)
- Study [Architecture](ARCHITECTURE.md)
