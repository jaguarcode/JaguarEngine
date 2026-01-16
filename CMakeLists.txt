cmake_minimum_required(VERSION 3.25)

project(JaguarEngine
    VERSION 0.4.0
    DESCRIPTION "Next-Generation Multi-Domain Physics Simulation Platform"
    LANGUAGES CXX
)

# ============================================================================
# Project Options
# ============================================================================
option(JAGUAR_BUILD_TESTS "Build test suite" ON)
option(JAGUAR_BUILD_BENCHMARKS "Build benchmarks" ON)
option(JAGUAR_BUILD_EXAMPLES "Build example applications" ON)
option(JAGUAR_BUILD_PYTHON "Build Python bindings" OFF)
option(JAGUAR_BUILD_LUA "Build Lua bindings" OFF)
option(JAGUAR_ENABLE_DIS "Enable DIS network support" OFF)
option(JAGUAR_ENABLE_HLA "Enable HLA network support" OFF)
option(JAGUAR_ENABLE_SIMD "Enable SIMD optimizations" ON)

# ============================================================================
# C++ Standard & Compiler Settings
# ============================================================================
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Export compile commands for IDE support
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Default to Release build if not specified
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type" FORCE)
endif()

# ============================================================================
# CMake Module Path
# ============================================================================
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# ============================================================================
# Compiler Warnings
# ============================================================================
add_library(jaguar_compiler_flags INTERFACE)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang|AppleClang")
    target_compile_options(jaguar_compiler_flags INTERFACE
        -Wall
        -Wextra
        -Wpedantic
        -Wconversion
        -Wshadow
        -Wnon-virtual-dtor
        -Wold-style-cast
        -Wcast-align
        -Wunused
        -Woverloaded-virtual
        -Wnull-dereference
        -Wformat=2
        $<$<CONFIG:Release>:-O3>
    )

    # Architecture-specific optimizations (disabled for cross-compilation compatibility)
    # Note: -march=native and -mcpu=native cause issues under Rosetta
    if(JAGUAR_ENABLE_SIMD)
        # Only enable SIMD on native x86_64 systems
        if(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "x86_64" AND CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
            target_compile_options(jaguar_compiler_flags INTERFACE
                -mavx2
                -mfma
            )
        endif()
    endif()
elseif(MSVC)
    target_compile_options(jaguar_compiler_flags INTERFACE
        /W4
        /permissive-
        $<$<CONFIG:Release>:/O2>
    )

    if(JAGUAR_ENABLE_SIMD)
        target_compile_options(jaguar_compiler_flags INTERFACE
            /arch:AVX2
        )
    endif()
endif()

# ============================================================================
# Dependencies
# ============================================================================

# Eigen (Linear Algebra)
find_package(Eigen3 3.4 QUIET)
if(NOT Eigen3_FOUND)
    message(STATUS "Eigen3 not found, will use FetchContent")
    include(FetchContent)
    FetchContent_Declare(
        eigen
        GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
        GIT_TAG 3.4.0
        GIT_SHALLOW TRUE
    )
    FetchContent_MakeAvailable(eigen)
endif()

# pugixml (XML Parsing)
find_package(pugixml QUIET)
if(NOT pugixml_FOUND)
    include(FetchContent)
    FetchContent_Declare(
        pugixml
        GIT_REPOSITORY https://github.com/zeux/pugixml.git
        GIT_TAG v1.14
        GIT_SHALLOW TRUE
    )
    FetchContent_MakeAvailable(pugixml)
endif()

# GDAL (Geospatial Data) - Optional but recommended
find_package(GDAL 3.0 QUIET)
if(GDAL_FOUND)
    message(STATUS "GDAL found: ${GDAL_VERSION}")
else()
    message(WARNING "GDAL not found. Terrain features will be limited.")
endif()

# ============================================================================
# Core Library
# ============================================================================
add_library(jaguar_core STATIC
    # Core
    src/core/engine_exec.cpp
    src/core/property_manager.cpp
    src/core/config_loader.cpp
    src/core/time_manager.cpp
    # Memory
    src/core/memory/pool_allocator.cpp
    # Threading
    src/core/threading/thread_pool.cpp
    # SIMD
    src/core/simd.cpp
    # Math
    src/core/math/vector.cpp
    src/core/math/quaternion.cpp
    src/core/math/matrix.cpp
    src/core/math/coordinates.cpp
)

target_include_directories(jaguar_core
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(jaguar_core
    PUBLIC
        Eigen3::Eigen
    PRIVATE
        jaguar_compiler_flags
        pugixml
)

# ============================================================================
# Physics Library
# ============================================================================
add_library(jaguar_physics STATIC
    src/physics/entity_manager.cpp
    src/physics/physics_system.cpp
    src/physics/force_generators.cpp
    src/physics/integrators/rk4.cpp
    src/physics/integrators/abm4.cpp
    src/physics/integrators/adaptive.cpp
)

target_link_libraries(jaguar_physics
    PUBLIC
        jaguar_core
    PRIVATE
        jaguar_compiler_flags
)

# ============================================================================
# Domain Libraries
# ============================================================================

# Air Domain
add_library(jaguar_domain_air STATIC
    src/domain/air/aerodynamics.cpp
    src/domain/air/propulsion.cpp
    src/domain/air/flight_control.cpp
)

target_link_libraries(jaguar_domain_air
    PUBLIC
        jaguar_physics
    PRIVATE
        jaguar_compiler_flags
)

# Land Domain
add_library(jaguar_domain_land STATIC
    src/domain/land/terramechanics.cpp
    src/domain/land/suspension.cpp
)

target_link_libraries(jaguar_domain_land
    PUBLIC
        jaguar_physics
    PRIVATE
        jaguar_compiler_flags
)

# Sea Domain
add_library(jaguar_domain_sea STATIC
    src/domain/sea/hydrodynamics.cpp
    src/domain/sea/buoyancy.cpp
)

target_link_libraries(jaguar_domain_sea
    PUBLIC
        jaguar_physics
    PRIVATE
        jaguar_compiler_flags
)

# Space Domain
add_library(jaguar_domain_space STATIC
    src/domain/space/sgp4.cpp
    src/domain/space/gravity.cpp
)

target_link_libraries(jaguar_domain_space
    PUBLIC
        jaguar_physics
    PRIVATE
        jaguar_compiler_flags
)

# ============================================================================
# Environment Library
# ============================================================================
add_library(jaguar_environment STATIC
    src/environment/terrain/terrain_manager.cpp
    src/environment/terrain/gdal_loader.cpp
    src/environment/atmosphere/standard_atmosphere.cpp
    src/environment/ocean/wave_spectrum.cpp
)

target_link_libraries(jaguar_environment
    PUBLIC
        jaguar_core
    PRIVATE
        jaguar_compiler_flags
        $<$<BOOL:${GDAL_FOUND}>:GDAL::GDAL>
)

if(GDAL_FOUND)
    target_compile_definitions(jaguar_environment PRIVATE JAGUAR_HAS_GDAL)
endif()

# ============================================================================
# Main Engine Library (combines all modules)
# ============================================================================
add_library(jaguar INTERFACE)

target_link_libraries(jaguar INTERFACE
    jaguar_core
    jaguar_physics
    jaguar_domain_air
    jaguar_domain_land
    jaguar_domain_sea
    jaguar_domain_space
    jaguar_environment
)

# ============================================================================
# Network Libraries (Optional)
# ============================================================================
if(JAGUAR_ENABLE_DIS)
    add_library(jaguar_network_dis STATIC
        src/interface/network/dis_adapter.cpp
    )
    target_link_libraries(jaguar_network_dis
        PUBLIC jaguar_core
        PRIVATE jaguar_compiler_flags
    )
    # TODO: Link OpenDIS library
endif()

if(JAGUAR_ENABLE_HLA)
    add_library(jaguar_network_hla STATIC
        src/interface/network/hla_adapter.cpp
    )
    target_link_libraries(jaguar_network_hla
        PUBLIC jaguar_core
        PRIVATE jaguar_compiler_flags
    )
    # TODO: Link HLA RTI library
endif()

# ============================================================================
# Python Bindings (Optional)
# ============================================================================
if(JAGUAR_BUILD_PYTHON)
    find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
    find_package(pybind11 CONFIG QUIET)

    if(NOT pybind11_FOUND)
        include(FetchContent)
        FetchContent_Declare(
            pybind11
            GIT_REPOSITORY https://github.com/pybind/pybind11.git
            GIT_TAG v2.11.1
            GIT_SHALLOW TRUE
        )
        FetchContent_MakeAvailable(pybind11)
    endif()

    pybind11_add_module(pyjaguar
        src/interface/script/python_bindings.cpp
    )
    target_link_libraries(pyjaguar PRIVATE jaguar)
endif()

# ============================================================================
# Tests
# ============================================================================
if(JAGUAR_BUILD_TESTS)
    enable_testing()

    include(FetchContent)
    FetchContent_Declare(
        googletest
        GIT_REPOSITORY https://github.com/google/googletest.git
        GIT_TAG v1.14.0
        GIT_SHALLOW TRUE
    )
    # Prevent overriding the parent project's compiler/linker settings on Windows
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)

    include(GoogleTest)

    # Unit tests
    add_executable(jaguar_unit_tests
        tests/unit/test_main.cpp
        tests/unit/test_math.cpp
        tests/unit/test_physics.cpp
        tests/unit/test_force.cpp
        tests/unit/test_coordinates.cpp
        tests/unit/test_atmosphere.cpp
        tests/unit/test_air_domain.cpp
        tests/unit/test_land_domain.cpp
        tests/unit/test_sea_domain.cpp
        tests/unit/test_space_domain.cpp
        tests/unit/test_simd.cpp
    )
    target_link_libraries(jaguar_unit_tests PRIVATE
        jaguar
        GTest::gtest
        GTest::gtest_main
    )
    gtest_discover_tests(jaguar_unit_tests)
endif()

# ============================================================================
# Benchmarks
# ============================================================================
if(JAGUAR_BUILD_BENCHMARKS)
    include(FetchContent)
    FetchContent_Declare(
        benchmark
        GIT_REPOSITORY https://github.com/google/benchmark.git
        GIT_TAG v1.8.3
        GIT_SHALLOW TRUE
    )
    set(BENCHMARK_ENABLE_TESTING OFF CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(benchmark)

    add_executable(jaguar_benchmarks
        tests/benchmarks/bench_main.cpp
        # Add benchmark files here
    )
    target_link_libraries(jaguar_benchmarks PRIVATE
        jaguar
        benchmark::benchmark
        benchmark::benchmark_main
    )
endif()

# ============================================================================
# Examples
# ============================================================================
if(JAGUAR_BUILD_EXAMPLES)
    add_executable(example_simple_flight
        examples/simple_flight/main.cpp
    )
    target_link_libraries(example_simple_flight PRIVATE jaguar)

    add_executable(example_multi_domain
        examples/multi_domain/main.cpp
    )
    target_link_libraries(example_multi_domain PRIVATE jaguar)
endif()

# ============================================================================
# Installation
# ============================================================================
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

install(TARGETS
    jaguar_core
    jaguar_physics
    jaguar_domain_air
    jaguar_domain_land
    jaguar_domain_sea
    jaguar_domain_space
    jaguar_environment
    jaguar
    jaguar_compiler_flags
    EXPORT JaguarTargets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(EXPORT JaguarTargets
    FILE JaguarTargets.cmake
    NAMESPACE Jaguar::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/Jaguar
)

configure_package_config_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/JaguarConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/JaguarConfig.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/Jaguar
)

write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/JaguarConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)

install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/JaguarConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/JaguarConfigVersion.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/Jaguar
)

# ============================================================================
# Summary
# ============================================================================
message(STATUS "")
message(STATUS "JaguarEngine Configuration Summary")
message(STATUS "===================================")
message(STATUS "Version:           ${PROJECT_VERSION}")
message(STATUS "Build type:        ${CMAKE_BUILD_TYPE}")
message(STATUS "C++ Compiler:      ${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}")
message(STATUS "")
message(STATUS "Options:")
message(STATUS "  Tests:           ${JAGUAR_BUILD_TESTS}")
message(STATUS "  Benchmarks:      ${JAGUAR_BUILD_BENCHMARKS}")
message(STATUS "  Examples:        ${JAGUAR_BUILD_EXAMPLES}")
message(STATUS "  Python bindings: ${JAGUAR_BUILD_PYTHON}")
message(STATUS "  Lua bindings:    ${JAGUAR_BUILD_LUA}")
message(STATUS "  DIS support:     ${JAGUAR_ENABLE_DIS}")
message(STATUS "  HLA support:     ${JAGUAR_ENABLE_HLA}")
message(STATUS "  SIMD:            ${JAGUAR_ENABLE_SIMD}")
message(STATUS "")
message(STATUS "Dependencies:")
message(STATUS "  Eigen3:          ${Eigen3_FOUND}")
message(STATUS "  GDAL:            ${GDAL_FOUND}")
message(STATUS "")

# ============================================================================
# CPack Configuration - SDK Packaging
# ============================================================================
set(CPACK_PACKAGE_NAME "JaguarEngine")
set(CPACK_PACKAGE_VENDOR "JaguarEngine Development Team")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Multi-Domain Physics Simulation Engine")
set(CPACK_PACKAGE_DESCRIPTION_FILE "${CMAKE_CURRENT_SOURCE_DIR}/README.md")
set(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_SOURCE_DIR}/LICENSE")
set(CPACK_PACKAGE_VERSION_MAJOR ${PROJECT_VERSION_MAJOR})
set(CPACK_PACKAGE_VERSION_MINOR ${PROJECT_VERSION_MINOR})
set(CPACK_PACKAGE_VERSION_PATCH ${PROJECT_VERSION_PATCH})
set(CPACK_PACKAGE_VERSION ${PROJECT_VERSION})
set(CPACK_PACKAGE_INSTALL_DIRECTORY "JaguarEngine-${PROJECT_VERSION}")
set(CPACK_PACKAGE_CONTACT "support@jaguarengine.dev")
set(CPACK_PACKAGE_HOMEPAGE_URL "https://github.com/jaguarcode/JaguarEngine")

# Package file name format
set(CPACK_PACKAGE_FILE_NAME "${CPACK_PACKAGE_NAME}-${CPACK_PACKAGE_VERSION}-${CMAKE_SYSTEM_NAME}-${CMAKE_SYSTEM_PROCESSOR}")

# Source package configuration
set(CPACK_SOURCE_PACKAGE_FILE_NAME "${CPACK_PACKAGE_NAME}-${CPACK_PACKAGE_VERSION}-source")
set(CPACK_SOURCE_IGNORE_FILES
    "/\\\\.git/"
    "/\\\\.github/"
    "/\\\\.vscode/"
    "/\\\\.idea/"
    "/build/"
    "/cmake-build-.*/"
    "\\\\.DS_Store$"
    "~$"
)

# ============================================================================
# Component-Based Installation
# ============================================================================
set(CPACK_COMPONENTS_ALL
    core
    development
    python
    lua
    documentation
    examples
)

# Component descriptions
set(CPACK_COMPONENT_CORE_DISPLAY_NAME "Core Libraries")
set(CPACK_COMPONENT_CORE_DESCRIPTION "Core JaguarEngine runtime libraries for simulation")
set(CPACK_COMPONENT_CORE_REQUIRED TRUE)

set(CPACK_COMPONENT_DEVELOPMENT_DISPLAY_NAME "Development Files")
set(CPACK_COMPONENT_DEVELOPMENT_DESCRIPTION "Header files and CMake configuration for C++ development")
set(CPACK_COMPONENT_DEVELOPMENT_DEPENDS core)

set(CPACK_COMPONENT_PYTHON_DISPLAY_NAME "Python Bindings")
set(CPACK_COMPONENT_PYTHON_DESCRIPTION "Python module (pyjaguar) for Python 3.8+ integration")
set(CPACK_COMPONENT_PYTHON_DEPENDS core)
set(CPACK_COMPONENT_PYTHON_DISABLED TRUE)  # Optional by default

set(CPACK_COMPONENT_LUA_DISPLAY_NAME "Lua Bindings")
set(CPACK_COMPONENT_LUA_DESCRIPTION "Lua module for Lua 5.4+ scripting support")
set(CPACK_COMPONENT_LUA_DEPENDS core)
set(CPACK_COMPONENT_LUA_DISABLED TRUE)  # Optional by default

set(CPACK_COMPONENT_DOCUMENTATION_DISPLAY_NAME "Documentation")
set(CPACK_COMPONENT_DOCUMENTATION_DESCRIPTION "API documentation, user guides, and examples")
set(CPACK_COMPONENT_DOCUMENTATION_DISABLED TRUE)  # Optional by default

set(CPACK_COMPONENT_EXAMPLES_DISPLAY_NAME "Example Applications")
set(CPACK_COMPONENT_EXAMPLES_DESCRIPTION "Sample code and example simulations")
set(CPACK_COMPONENT_EXAMPLES_DEPENDS core)
set(CPACK_COMPONENT_EXAMPLES_DISABLED TRUE)  # Optional by default

# Component groups
set(CPACK_COMPONENT_CORE_GROUP "Runtime")
set(CPACK_COMPONENT_DEVELOPMENT_GROUP "Development")
set(CPACK_COMPONENT_PYTHON_GROUP "Scripting")
set(CPACK_COMPONENT_LUA_GROUP "Scripting")
set(CPACK_COMPONENT_DOCUMENTATION_GROUP "Documentation")
set(CPACK_COMPONENT_EXAMPLES_GROUP "Documentation")

set(CPACK_COMPONENT_GROUP_RUNTIME_DESCRIPTION "Runtime libraries required for simulation")
set(CPACK_COMPONENT_GROUP_DEVELOPMENT_DESCRIPTION "Files needed for C++ development")
set(CPACK_COMPONENT_GROUP_SCRIPTING_DESCRIPTION "Language bindings for scripting integration")
set(CPACK_COMPONENT_GROUP_DOCUMENTATION_DESCRIPTION "Documentation and examples")

# ============================================================================
# Platform-Specific Installer Configuration
# ============================================================================
if(WIN32)
    # NSIS Installer (Windows)
    set(CPACK_GENERATOR "NSIS;ZIP")
    set(CPACK_NSIS_DISPLAY_NAME "JaguarEngine ${PROJECT_VERSION}")
    set(CPACK_NSIS_PACKAGE_NAME "JaguarEngine")
    set(CPACK_NSIS_HELP_LINK "https://github.com/jaguarcode/JaguarEngine")
    set(CPACK_NSIS_URL_INFO_ABOUT "https://github.com/jaguarcode/JaguarEngine")
    set(CPACK_NSIS_CONTACT "${CPACK_PACKAGE_CONTACT}")
    set(CPACK_NSIS_MODIFY_PATH ON)
    set(CPACK_NSIS_ENABLE_UNINSTALL_BEFORE_INSTALL ON)
    set(CPACK_NSIS_INSTALL_ROOT "$PROGRAMFILES64")

    # Start menu shortcuts
    set(CPACK_NSIS_CREATE_ICONS_EXTRA
        "CreateShortCut '$SMPROGRAMS\\\\$STARTMENU_FOLDER\\\\JaguarEngine Documentation.lnk' '$INSTDIR\\\\share\\\\doc\\\\JaguarEngine\\\\html\\\\index.html'"
    )
    set(CPACK_NSIS_DELETE_ICONS_EXTRA
        "Delete '$SMPROGRAMS\\\\$STARTMENU_FOLDER\\\\JaguarEngine Documentation.lnk'"
    )

    # Environment variable setup
    set(CPACK_NSIS_EXTRA_INSTALL_COMMANDS "
        WriteRegStr HKLM 'SYSTEM\\\\CurrentControlSet\\\\Control\\\\Session Manager\\\\Environment' 'JAGUAR_HOME' '$INSTDIR'
    ")
    set(CPACK_NSIS_EXTRA_UNINSTALL_COMMANDS "
        DeleteRegValue HKLM 'SYSTEM\\\\CurrentControlSet\\\\Control\\\\Session Manager\\\\Environment' 'JAGUAR_HOME'
    ")

elseif(APPLE)
    # macOS DragNDrop and productbuild
    set(CPACK_GENERATOR "DragNDrop;TGZ")
    set(CPACK_DMG_VOLUME_NAME "JaguarEngine ${PROJECT_VERSION}")
    set(CPACK_DMG_FORMAT "UDBZ")  # bzip2 compressed
    set(CPACK_DMG_DS_STORE_SETUP_SCRIPT "${CMAKE_CURRENT_SOURCE_DIR}/cmake/macos/DS_Store_setup.scpt")
    set(CPACK_DMG_BACKGROUND_IMAGE "${CMAKE_CURRENT_SOURCE_DIR}/cmake/macos/background.png")

    # Bundle configuration
    set(CPACK_BUNDLE_NAME "JaguarEngine")
    set(CPACK_BUNDLE_PLIST "${CMAKE_CURRENT_SOURCE_DIR}/cmake/macos/Info.plist.in")
    set(CPACK_BUNDLE_ICON "${CMAKE_CURRENT_SOURCE_DIR}/cmake/macos/JaguarEngine.icns")

    # Code signing (requires developer certificate)
    set(CPACK_PRODUCTBUILD_IDENTITY_NAME "" CACHE STRING "macOS code signing identity")
    if(CPACK_PRODUCTBUILD_IDENTITY_NAME)
        set(CPACK_PRODUCTBUILD_SIGN ON)
    endif()

else()
    # Linux - DEB and RPM packages
    set(CPACK_GENERATOR "DEB;RPM;TGZ")

    # Debian package configuration
    set(CPACK_DEBIAN_PACKAGE_NAME "jaguarengine")
    set(CPACK_DEBIAN_PACKAGE_MAINTAINER "${CPACK_PACKAGE_CONTACT}")
    set(CPACK_DEBIAN_PACKAGE_SECTION "science")
    set(CPACK_DEBIAN_PACKAGE_PRIORITY "optional")
    set(CPACK_DEBIAN_PACKAGE_HOMEPAGE "${CPACK_PACKAGE_HOMEPAGE_URL}")
    set(CPACK_DEBIAN_PACKAGE_DEPENDS "libeigen3-dev (>= 3.4)")
    set(CPACK_DEBIAN_PACKAGE_SUGGESTS "libgdal-dev, python3-dev")
    set(CPACK_DEBIAN_FILE_NAME DEB-DEFAULT)
    set(CPACK_DEBIAN_PACKAGE_SHLIBDEPS ON)
    set(CPACK_DEBIAN_COMPRESSION_TYPE "xz")

    # Component-specific Debian packages
    set(CPACK_DEB_COMPONENT_INSTALL ON)
    set(CPACK_DEBIAN_CORE_PACKAGE_NAME "jaguarengine")
    set(CPACK_DEBIAN_DEVELOPMENT_PACKAGE_NAME "jaguarengine-dev")
    set(CPACK_DEBIAN_PYTHON_PACKAGE_NAME "python3-jaguarengine")
    set(CPACK_DEBIAN_LUA_PACKAGE_NAME "lua-jaguarengine")
    set(CPACK_DEBIAN_DOCUMENTATION_PACKAGE_NAME "jaguarengine-doc")
    set(CPACK_DEBIAN_EXAMPLES_PACKAGE_NAME "jaguarengine-examples")

    # RPM package configuration
    set(CPACK_RPM_PACKAGE_NAME "jaguarengine")
    set(CPACK_RPM_PACKAGE_LICENSE "Proprietary")
    set(CPACK_RPM_PACKAGE_GROUP "Development/Libraries")
    set(CPACK_RPM_PACKAGE_URL "${CPACK_PACKAGE_HOMEPAGE_URL}")
    set(CPACK_RPM_PACKAGE_REQUIRES "eigen3-devel >= 3.4")
    set(CPACK_RPM_PACKAGE_SUGGESTS "gdal-devel, python3-devel")
    set(CPACK_RPM_FILE_NAME RPM-DEFAULT)
    set(CPACK_RPM_PACKAGE_AUTOREQ ON)
    set(CPACK_RPM_COMPRESSION_TYPE "xz")

    # Component-specific RPM packages
    set(CPACK_RPM_COMPONENT_INSTALL ON)
    set(CPACK_RPM_CORE_PACKAGE_NAME "jaguarengine")
    set(CPACK_RPM_DEVELOPMENT_PACKAGE_NAME "jaguarengine-devel")
    set(CPACK_RPM_PYTHON_PACKAGE_NAME "python3-jaguarengine")
    set(CPACK_RPM_LUA_PACKAGE_NAME "lua-jaguarengine")
    set(CPACK_RPM_DOCUMENTATION_PACKAGE_NAME "jaguarengine-doc")
    set(CPACK_RPM_EXAMPLES_PACKAGE_NAME "jaguarengine-examples")
endif()

# ============================================================================
# Additional Installation Rules for Components
# ============================================================================

# Install documentation (if built)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/docs/api/html")
    install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/docs/api/html"
        DESTINATION "${CMAKE_INSTALL_DOCDIR}"
        COMPONENT documentation
    )
endif()

# Install user guides
install(FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/README.md"
    DESTINATION "${CMAKE_INSTALL_DOCDIR}"
    COMPONENT documentation
    OPTIONAL
)

# Install examples source
if(JAGUAR_BUILD_EXAMPLES)
    install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/examples/"
        DESTINATION "${CMAKE_INSTALL_DOCDIR}/examples"
        COMPONENT examples
        FILES_MATCHING
        PATTERN "*.cpp"
        PATTERN "*.h"
        PATTERN "*.hpp"
        PATTERN "*.py"
        PATTERN "*.lua"
        PATTERN "CMakeLists.txt"
    )
endif()

# Install Python module to component
if(JAGUAR_BUILD_PYTHON)
    install(TARGETS pyjaguar
        LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/site-packages"
        COMPONENT python
    )

    # Install Python examples and tests
    install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/"
        DESTINATION "${CMAKE_INSTALL_DOCDIR}/python-examples"
        COMPONENT python
        FILES_MATCHING PATTERN "*.py"
    )
endif()

# Install Lua module to component
if(JAGUAR_BUILD_LUA)
    # Lua bindings library would go here
    install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/examples/lua/"
        DESTINATION "${CMAKE_INSTALL_DOCDIR}/lua-examples"
        COMPONENT lua
        FILES_MATCHING PATTERN "*.lua"
    )
endif()

# ============================================================================
# Include CPack (must be last)
# ============================================================================
include(CPack)

# ============================================================================
# Post-Configuration Summary
# ============================================================================
message(STATUS "Packaging Configuration:")
message(STATUS "  Package name:     ${CPACK_PACKAGE_NAME}")
message(STATUS "  Package version:  ${CPACK_PACKAGE_VERSION}")
message(STATUS "  Generator:        ${CPACK_GENERATOR}")
message(STATUS "")
