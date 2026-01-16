# Installation Guide

This guide covers building JaguarEngine from source on various platforms.

## System Requirements

### Minimum Requirements

| Component | Requirement |
|-----------|-------------|
| **CPU** | x86_64 or ARM64 processor |
| **RAM** | 4 GB (8 GB recommended for development) |
| **Disk** | 1 GB free space |
| **OS** | Linux, macOS 12+, Windows 10+ |

### Compiler Requirements

JaguarEngine requires a C++20 compatible compiler:

| Compiler | Minimum Version |
|----------|-----------------|
| GCC | 11.0+ |
| Clang | 14.0+ |
| AppleClang | 15.0+ |
| MSVC | Visual Studio 2022 (17.0+) |

### Build Tools

- **CMake**: 3.25 or later
- **Make/Ninja**: Any recent version

---

## Dependencies

### Required (Auto-fetched)

These dependencies are automatically downloaded by CMake if not found on your system:

| Library | Version | Purpose |
|---------|---------|---------|
| **Eigen3** | 3.4.0 | Linear algebra, matrix operations |
| **pugixml** | 1.14 | XML configuration parsing |
| **GoogleTest** | 1.14.0 | Unit testing framework |
| **Google Benchmark** | 1.8.3 | Performance benchmarking |

### Optional Dependencies

| Library | Version | Purpose | CMake Option |
|---------|---------|---------|--------------|
| GDAL | 3.0+ | Geospatial terrain loading | Auto-detected |
| pybind11 | 2.11+ | Python bindings | `JAGUAR_BUILD_PYTHON` |
| sol2 | 3.3+ | Lua bindings | `JAGUAR_BUILD_LUA` |
| OpenDIS | - | DIS protocol | `JAGUAR_ENABLE_DIS` |
| HLA RTI | - | HLA protocol | `JAGUAR_ENABLE_HLA` |

---

## Quick Installation

### Linux / macOS

```bash
# Clone repository
git clone https://github.com/jaguarcode/JaguarEngine.git
cd JaguarEngine

# Create build directory
mkdir build && cd build

# Configure (dependencies auto-fetched)
cmake ..

# Build (use all available cores)
make -j$(nproc)

# Run tests to verify installation
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

---

## Platform-Specific Instructions

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
cd JaguarEngine
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
cd JaguarEngine
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
cd JaguarEngine
mkdir build && cd build
cmake ..
make -j$(sysctl -n hw.ncpu)
```

### Windows

1. Install [Visual Studio 2022](https://visualstudio.microsoft.com/) with the **C++ Desktop Development** workload
2. Install [CMake](https://cmake.org/download/) (add to PATH)
3. Install [Git](https://git-scm.com/)

```powershell
# From Developer PowerShell for VS 2022
cd JaguarEngine
mkdir build && cd build
cmake .. -G "Visual Studio 17 2022" -A x64
cmake --build . --config Release --parallel
```

---

## Build Options

### CMake Configuration Options

```bash
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

---

## Installing JaguarEngine

### System-wide Installation

```bash
cd build
sudo cmake --install . --prefix /usr/local
```

### Custom Installation Path

```bash
cmake --install . --prefix ~/local/jaguar
```

### Using in Your CMake Project

After installation, use JaguarEngine in your CMake projects:

```cmake
# CMakeLists.txt
cmake_minimum_required(VERSION 3.25)
project(MySimulation)

find_package(Jaguar REQUIRED)

add_executable(my_simulation main.cpp)
target_link_libraries(my_simulation PRIVATE Jaguar::jaguar)
```

---

## Verifying Installation

### Run Unit Tests

```bash
cd build
./jaguar_unit_tests
```

**Expected output:**

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
cmake -LA ..  # List all options including advanced
```

---

## Troubleshooting

### Common Issues

#### Eigen3 Not Found

```
CMake Warning: Eigen3 not found, will use FetchContent
```

**Solution:** This is normal behavior. Eigen3 is automatically downloaded if not installed system-wide.

#### GDAL Not Found

```
CMake Warning: GDAL not found. Terrain features will be limited.
```

**Solution:** Install GDAL if you need terrain loading:

- Ubuntu: `sudo apt install libgdal-dev`
- macOS: `brew install gdal`
- Windows: Use [OSGeo4W](https://trac.osgeo.org/osgeo4w/) installer

#### Compiler Too Old

```
error: 'span' is not a member of 'std'
```

**Solution:** Upgrade to a C++20 compatible compiler:

- GCC 11+ (Ubuntu 22.04+ default)
- Clang 14+
- MSVC 2022

#### Build Fails on ARM Mac (Rosetta)

```
error: -march=native requires native target
```

**Solution:** The build system automatically detects Rosetta and disables native optimizations. Ensure you're using the latest CMakeLists.txt.

### Getting Help

1. Check existing [GitHub Issues](https://github.com/jaguarcode/JaguarEngine/issues)
2. Search the [documentation](https://jaguarcode.github.io/JaguarEngine/)
3. Open a new issue with:
   - OS and version
   - Compiler version (`g++ --version`)
   - CMake version (`cmake --version`)
   - Full error output

---

## Next Steps

- [Quick Start Tutorial](quickstart.md) - Your first simulation
- [Examples Guide](../tutorials/examples.md) - Code examples for all domains
- [API Reference](../api/overview.md) - Complete API documentation
