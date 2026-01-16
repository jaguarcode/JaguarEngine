#pragma once
/**
 * @file ocean.h
 * @brief Ocean environment and wave models
 */

#include "jaguar/core/types.h"
#include "jaguar/domain/sea.h"
#include <memory>

namespace jaguar::environment {

/**
 * @brief Ocean state at a point
 */
struct OceanState {
    Real water_depth{1000.0};     ///< Water depth (m)
    Real surface_elevation{0.0};  ///< Wave surface elevation (m)
    Vec3 current{0, 0, 0};        ///< Ocean current (m/s)
    Real temperature{15.0};       ///< Water temperature (°C)
    Real salinity{35.0};          ///< Salinity (ppt)
    Real density{1025.0};         ///< Water density (kg/m³)
};

/**
 * @brief Ocean environment manager
 */
class OceanManager {
public:
    OceanManager();
    ~OceanManager();

    /**
     * @brief Set sea state
     */
    void set_sea_state(const domain::sea::SeaState& state);

    /**
     * @brief Get sea state
     */
    const domain::sea::SeaState& get_sea_state() const;

    /**
     * @brief Get ocean state at position
     */
    OceanState get_state(Real lat, Real lon, Real time) const;

    /**
     * @brief Get wave elevation at position
     */
    Real get_wave_elevation(Real x, Real y, Real time) const;

    /**
     * @brief Get wave slope at position
     */
    Vec3 get_wave_slope(Real x, Real y, Real time) const;

    /**
     * @brief Get wave particle velocity
     */
    Vec3 get_particle_velocity(Real x, Real y, Real z, Real time) const;

    /**
     * @brief Set ocean current
     */
    void set_current(const Vec3& current);

    /**
     * @brief Get ocean current at position
     */
    Vec3 get_current(Real lat, Real lon, Real depth) const;

    /**
     * @brief Get water density at position
     */
    Real get_density(Real lat, Real lon, Real depth) const;

    /**
     * @brief Update wave model (call each frame)
     */
    void update(Real dt);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;

    /**
     * @brief Compute seawater density using UNESCO equation of state
     */
    Real compute_seawater_density(Real temperature, Real salinity) const;
};

} // namespace jaguar::environment
