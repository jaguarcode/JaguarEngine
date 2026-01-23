#pragma once
/**
 * @file api.h
 * @brief Public C++ API for JaguarEngine
 */

#include "jaguar/core/types.h"
#include "jaguar/physics/entity.h"
#include <memory>

namespace jaguar {

// Forward declarations
class EngineImpl;

/**
 * @brief Main engine facade - primary interface for users
 */
class Engine {
public:
    Engine();
    ~Engine();

    // Non-copyable
    Engine(const Engine&) = delete;
    Engine& operator=(const Engine&) = delete;

    // Movable
    Engine(Engine&&) noexcept;
    Engine& operator=(Engine&&) noexcept;

    // ========================================================================
    // Lifecycle
    // ========================================================================

    /**
     * @brief Initialize engine with configuration file
     * @param config_path Path to XML configuration file
     * @return true if initialization successful
     */
    bool initialize(const std::string& config_path);

    /**
     * @brief Initialize with default configuration
     */
    bool initialize();

    /**
     * @brief Shutdown engine and release resources
     */
    void shutdown();

    /**
     * @brief Check if engine is initialized
     */
    bool is_initialized() const;

    // ========================================================================
    // Simulation Control
    // ========================================================================

    /**
     * @brief Run simulation continuously
     */
    void run();

    /**
     * @brief Run for specified duration
     * @param duration_sec Simulation time to advance
     */
    void run_for(Real duration_sec);

    /**
     * @brief Advance simulation by one time step
     * @param dt Time step in seconds
     */
    void step(Real dt);

    /**
     * @brief Pause simulation
     */
    void pause();

    /**
     * @brief Resume simulation
     */
    void resume();

    /**
     * @brief Stop simulation
     */
    void stop();

    /**
     * @brief Get current simulation state
     */
    SimulationState get_state() const;

    // ========================================================================
    // Time Management
    // ========================================================================

    /**
     * @brief Get current simulation time
     */
    Real get_time() const;

    /**
     * @brief Get time scale (1.0 = real-time)
     */
    Real get_time_scale() const;

    /**
     * @brief Set time scale
     */
    void set_time_scale(Real scale);

    /**
     * @brief Set fixed time step
     */
    void set_fixed_time_step(Real dt);

    // ========================================================================
    // Entity Management
    // ========================================================================

    /**
     * @brief Create entity from XML definition
     * @param xml_path Path to entity XML file
     * @return Entity ID, or INVALID_ENTITY_ID on failure
     */
    EntityId create_entity(const std::string& xml_path);

    /**
     * @brief Create entity with specified domain
     */
    EntityId create_entity(const std::string& name, Domain domain);

    /**
     * @brief Destroy entity
     */
    void destroy_entity(EntityId id);

    /**
     * @brief Check if entity exists
     */
    bool entity_exists(EntityId id) const;

    /**
     * @brief Get entity state
     */
    physics::EntityState get_entity_state(EntityId id) const;

    /**
     * @brief Set entity state
     */
    void set_entity_state(EntityId id, const physics::EntityState& state);

    // ========================================================================
    // Property Access
    // ========================================================================

    /**
     * @brief Get property value
     */
    Real get_property(const std::string& path) const;

    /**
     * @brief Set property value
     */
    void set_property(const std::string& path, Real value);

    /**
     * @brief Get property for specific entity
     */
    Real get_property(EntityId id, const std::string& property) const;

    /**
     * @brief Set property for specific entity
     */
    void set_property(EntityId id, const std::string& property, Real value);

    // ========================================================================
    // Subsystem Access
    // ========================================================================

    /**
     * @brief Get entity manager
     */
    physics::EntityManager& get_entity_manager();
    const physics::EntityManager& get_entity_manager() const;

private:
    std::unique_ptr<EngineImpl> impl_;
};

// ============================================================================
// C-style API for FFI compatibility
// ============================================================================

extern "C" {

/**
 * @brief Create engine instance
 */
Engine* jaguar_create_engine();

/**
 * @brief Destroy engine instance
 */
void jaguar_destroy_engine(Engine* engine);

/**
 * @brief Initialize engine
 */
bool jaguar_initialize(Engine* engine, const char* config_path);

/**
 * @brief Step simulation
 */
void jaguar_step(Engine* engine, double dt);

/**
 * @brief Get simulation time
 */
double jaguar_get_time(Engine* engine);

/**
 * @brief Create entity
 */
EntityId jaguar_create_entity(Engine* engine, const char* xml_path);

/**
 * @brief Destroy entity
 */
void jaguar_destroy_entity(Engine* engine, EntityId id);

/**
 * @brief Get entity position
 */
void jaguar_get_position(Engine* engine, EntityId id, double* x, double* y, double* z);

/**
 * @brief Set entity position
 */
void jaguar_set_position(Engine* engine, EntityId id, double x, double y, double z);

/**
 * @brief Get property value
 */
double jaguar_get_property(Engine* engine, const char* path);

/**
 * @brief Set property value
 */
void jaguar_set_property(Engine* engine, const char* path, double value);

} // extern "C"

} // namespace jaguar
