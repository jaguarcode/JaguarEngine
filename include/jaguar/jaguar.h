#pragma once
/**
 * @file jaguar.h
 * @brief Main include file for JaguarEngine
 *
 * JaguarEngine - Next-Generation Multi-Domain Physics Simulation Platform
 * Copyright (c) 2026
 *
 * Include this single header to access all public JaguarEngine APIs.
 */

#include "jaguar/core/types.h"
#include "jaguar/core/memory.h"
#include "jaguar/core/property.h"
#include "jaguar/core/simd.h"

#include "jaguar/physics/entity.h"
#include "jaguar/physics/solver.h"
#include "jaguar/physics/force.h"

#include "jaguar/domain/air.h"
#include "jaguar/domain/land.h"
#include "jaguar/domain/sea.h"
#include "jaguar/domain/space.h"

#include "jaguar/environment/terrain.h"
#include "jaguar/environment/atmosphere.h"
#include "jaguar/environment/ocean.h"

#include "jaguar/interface/api.h"
#include "jaguar/interface/config.h"

/**
 * @namespace jaguar
 * @brief Root namespace for all JaguarEngine components
 */
namespace jaguar {

/**
 * @brief Library version information
 */
constexpr int VERSION_MAJOR = 0;
constexpr int VERSION_MINOR = 1;
constexpr int VERSION_PATCH = 0;

/**
 * @brief Get version string
 * @return Version string in format "major.minor.patch"
 */
constexpr const char* GetVersionString() noexcept {
    return "0.1.0";
}

} // namespace jaguar
