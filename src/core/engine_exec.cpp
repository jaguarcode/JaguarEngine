/**
 * @file engine_exec.cpp
 * @brief PhysicsEngineExec implementation
 *
 * Main simulation engine implementation with full physics integration.
 */

#include "jaguar/interface/api.h"
#include "jaguar/interface/config.h"
#include "jaguar/core/time.h"
#include "jaguar/core/property.h"
#include "jaguar/physics/solver.h"
#include "jaguar/physics/force.h"
#include "jaguar/environment/environment.h"
#include <atomic>

namespace jaguar {

// ============================================================================
// Engine Implementation
// ============================================================================

class EngineImpl {
public:
    EngineImpl() = default;
    ~EngineImpl() = default;

    bool initialize(const std::string& config_path)
    {
        try {
            config::EngineConfig cfg = config::EngineConfig::load(config_path);
            return initialize_with_config(cfg);
        } catch (const std::exception&) {
            // Fall back to defaults if config fails to load
            return initialize_internal();
        }
    }

    bool initialize()
    {
        return initialize_internal();
    }

    void shutdown()
    {
        state_ = SimulationState::Uninitialized;
        force_registry_.clear();
        initialized_ = false;
    }

    bool is_initialized() const { return initialized_; }

    void step(Real dt)
    {
        if (!initialized_ || state_ == SimulationState::Paused) {
            return;
        }

        // Update time management
        time_manager_.advance(dt);

        // Use fixed timestep sub-stepping if configured
        Real fixed_dt = time_manager_.get_fixed_dt();
        if (fixed_dt > 0.0) {
            // Fixed timestep mode with sub-stepping
            while (time_manager_.has_fixed_step()) {
                execute_physics_step(fixed_dt);
                time_manager_.consume_fixed_step();
            }
        } else {
            // Variable timestep mode
            execute_physics_step(dt * time_manager_.get_time_scale());
        }
    }

    void run()
    {
        state_ = SimulationState::Running;
        running_ = true;

        constexpr Real TARGET_DT = 0.01;  // 100 Hz default

        while (running_ && state_ == SimulationState::Running) {
            step(TARGET_DT);
        }
    }

    void run_for(Real duration_sec)
    {
        const Real dt = time_manager_.get_fixed_dt() > 0.0 ?
                        time_manager_.get_fixed_dt() : 0.01;
        Real elapsed = 0.0;

        state_ = SimulationState::Running;
        while (elapsed < duration_sec && state_ == SimulationState::Running) {
            step(dt);
            elapsed += dt;
        }
    }

    void pause()
    {
        if (state_ == SimulationState::Running) {
            state_ = SimulationState::Paused;
            time_manager_.pause();
        }
    }

    void resume()
    {
        if (state_ == SimulationState::Paused) {
            state_ = SimulationState::Running;
            time_manager_.resume();
        }
    }

    void stop()
    {
        running_ = false;
        state_ = SimulationState::Stopped;
        time_manager_.pause();
    }

    SimulationState get_state() const { return state_; }

    Real get_time() const { return time_manager_.get_time(); }

    Real get_time_scale() const { return time_manager_.get_time_scale(); }

    void set_time_scale(Real scale) { time_manager_.set_time_scale(scale); }

    void set_fixed_time_step(Real dt) { time_manager_.set_fixed_dt(dt); }

    physics::EntityManager& get_entity_manager() { return entity_manager_; }
    const physics::EntityManager& get_entity_manager() const { return entity_manager_; }

    physics::PhysicsSystem& get_physics_system() { return physics_system_; }
    const physics::PhysicsSystem& get_physics_system() const { return physics_system_; }

    physics::ForceGeneratorRegistry& get_force_registry() { return force_registry_; }
    const physics::ForceGeneratorRegistry& get_force_registry() const { return force_registry_; }

    environment::EnvironmentService& get_environment() { return environment_; }
    const environment::EnvironmentService& get_environment() const { return environment_; }

    core::TimeManager& get_time_manager() { return time_manager_; }
    const core::TimeManager& get_time_manager() const { return time_manager_; }

    core::PropertyManager& get_property_manager() { return property_manager_; }
    const core::PropertyManager& get_property_manager() const { return property_manager_; }

private:
    bool initialize_with_config(const config::EngineConfig& cfg)
    {
        // Initialize time manager with configured timestep
        if (!time_manager_.initialize(cfg.time_step)) {
            return false;
        }

        // Initialize environment service
        if (!environment_.initialize()) {
            return false;
        }

        // Register default force generators
        setup_default_force_generators();

        // Set RK4 as default integrator
        physics_system_.set_propagator(std::make_unique<physics::RK4Integrator>());

        // Store config properties for runtime access
        property_manager_.set_real("config/time_step", cfg.time_step);
        property_manager_.set_real("config/max_entities", static_cast<Real>(cfg.max_entities));
        property_manager_.set_bool("config/terrain_enabled", cfg.terrain_enabled);
        property_manager_.set_bool("config/weather_enabled", cfg.weather_enabled);

        initialized_ = true;
        state_ = SimulationState::Initialized;
        return true;
    }

    bool initialize_internal()
    {
        // Initialize time manager (default 100Hz fixed timestep)
        if (!time_manager_.initialize(0.01)) {
            return false;
        }

        // Initialize environment service
        if (!environment_.initialize()) {
            return false;
        }

        // Register default force generators
        setup_default_force_generators();

        // Set RK4 as default integrator
        physics_system_.set_propagator(std::make_unique<physics::RK4Integrator>());

        initialized_ = true;
        state_ = SimulationState::Initialized;
        return true;
    }

    void setup_default_force_generators()
    {
        // Register standard gravity model
        force_registry_.register_generator(physics::force::create_simple_gravity());
    }

    void execute_physics_step(Real dt)
    {
        // ====================================================================
        // Main Simulation Loop
        // ====================================================================

        // 1. Pre-Update Phase
        //    - Update environment time
        //    - Any pre-physics calculations
        environment_.set_time(time_manager_.get_time());
        environment_.update(dt);

        // 2. Clear Forces Phase
        //    - Reset accumulated forces on all entities
        auto& storage = entity_manager_.get_state_storage();
        storage.clear_forces();

        // 3. Force Calculation Phase
        //    - Query environment for each entity
        //    - Compute forces from all registered generators
        compute_all_forces(dt);

        // 4. Integration Phase
        //    - Propagate entity states using physics system
        physics_system_.update(entity_manager_, dt);

        // 5. Post-Update Phase
        //    - Constraint resolution (future)
        //    - Collision detection/response (future)
        //    - Event callbacks (future)
    }

    void compute_all_forces(Real dt)
    {
        auto& storage = entity_manager_.get_state_storage();
        auto enabled_generators = force_registry_.get_enabled_generators();

        // Process each active entity
        entity_manager_.for_each([&](physics::Entity& entity) {
            if (!entity.active) return;

            // Get entity state
            physics::EntityState state = storage.get_state(entity.state_index);

            // Query environment at entity position
            environment::Environment env = environment_.query(state.position, time_manager_.get_time());

            // Accumulate forces from all generators
            physics::EntityForces& forces = storage.forces(entity.state_index);

            for (auto* generator : enabled_generators) {
                // Check if generator applies to this entity's domain
                Domain gen_domain = generator->domain();
                if (gen_domain == Domain::Generic ||
                    gen_domain == entity.primary_domain) {
                    generator->compute_forces(state, env, dt, forces);
                }
            }

            // Apply entity-specific force generators based on component mask
            compute_entity_specific_forces(entity, state, env, dt, forces);
        });
    }

    void compute_entity_specific_forces(
        const physics::Entity& entity,
        const physics::EntityState& state,
        const environment::Environment& env,
        Real dt,
        physics::EntityForces& forces)
    {
        using namespace physics::ComponentBits;

        // Check component mask and apply appropriate models
        // These would typically be attached per-entity via configuration

        if (entity.components & Gravity) {
            // Gravity already handled by global generator
            // Entity-specific gravity overrides could go here
        }

        if (entity.components & Aerodynamics) {
            // Entity-specific aerodynamics model
            // Would be looked up from entity's component storage
        }

        if (entity.components & Propulsion) {
            // Entity-specific propulsion model
        }

        if (entity.components & Terramechanics) {
            // Ground vehicle terrain interaction
        }

        if (entity.components & Hydrodynamics) {
            // Ship hull forces
        }

        if (entity.components & Buoyancy) {
            // Buoyancy calculations
        }

        // Suppress unused parameter warnings for future expansion
        (void)state;
        (void)env;
        (void)dt;
        (void)forces;
    }

    // Subsystems
    core::TimeManager time_manager_;
    core::PropertyManager property_manager_;
    physics::EntityManager entity_manager_;
    physics::PhysicsSystem physics_system_;
    physics::ForceGeneratorRegistry force_registry_;
    environment::EnvironmentService environment_;

    // State
    bool initialized_{false};
    std::atomic<bool> running_{false};
    SimulationState state_{SimulationState::Uninitialized};
};

// ============================================================================
// Engine Public API
// ============================================================================

Engine::Engine() : impl_(std::make_unique<EngineImpl>()) {}
Engine::~Engine() = default;
Engine::Engine(Engine&&) noexcept = default;
Engine& Engine::operator=(Engine&&) noexcept = default;

bool Engine::initialize(const std::string& config_path)
{
    return impl_->initialize(config_path);
}

bool Engine::initialize()
{
    return impl_->initialize();
}

void Engine::shutdown()
{
    impl_->shutdown();
}

bool Engine::is_initialized() const
{
    return impl_->is_initialized();
}

void Engine::run()
{
    impl_->run();
}

void Engine::run_for(Real duration_sec)
{
    impl_->run_for(duration_sec);
}

void Engine::step(Real dt)
{
    impl_->step(dt);
}

void Engine::pause()
{
    impl_->pause();
}

void Engine::resume()
{
    impl_->resume();
}

void Engine::stop()
{
    impl_->stop();
}

SimulationState Engine::get_state() const
{
    return impl_->get_state();
}

Real Engine::get_time() const
{
    return impl_->get_time();
}

Real Engine::get_time_scale() const
{
    return impl_->get_time_scale();
}

void Engine::set_time_scale(Real scale)
{
    impl_->set_time_scale(scale);
}

void Engine::set_fixed_time_step(Real dt)
{
    impl_->set_fixed_time_step(dt);
}

EntityId Engine::create_entity(const std::string& /*xml_path*/)
{
    // TODO: Load entity from XML configuration
    return INVALID_ENTITY_ID;
}

EntityId Engine::create_entity(const std::string& name, Domain domain)
{
    return impl_->get_entity_manager().create_entity(name, domain);
}

void Engine::destroy_entity(EntityId id)
{
    impl_->get_entity_manager().destroy_entity(id);
}

bool Engine::entity_exists(EntityId id) const
{
    return impl_->get_entity_manager().exists(id);
}

physics::EntityState Engine::get_entity_state(EntityId id) const
{
    return impl_->get_entity_manager().get_state(id);
}

void Engine::set_entity_state(EntityId id, const physics::EntityState& state)
{
    impl_->get_entity_manager().set_state(id, state);
}

Real Engine::get_property(const std::string& path) const
{
    return impl_->get_property_manager().get_real(path);
}

void Engine::set_property(const std::string& path, Real value)
{
    impl_->get_property_manager().set_real(path, value);
}

Real Engine::get_property(EntityId id, const std::string& property) const
{
    // Construct entity property path: "entities/<id>/<property>"
    std::string path = "entities/" + std::to_string(id) + "/" + property;
    return impl_->get_property_manager().get_real(path);
}

void Engine::set_property(EntityId id, const std::string& property, Real value)
{
    // Construct entity property path: "entities/<id>/<property>"
    std::string path = "entities/" + std::to_string(id) + "/" + property;
    impl_->get_property_manager().set_real(path, value);
}

physics::EntityManager& Engine::get_entity_manager()
{
    return impl_->get_entity_manager();
}

const physics::EntityManager& Engine::get_entity_manager() const
{
    return impl_->get_entity_manager();
}

// ============================================================================
// C API Implementation
// ============================================================================

extern "C" {

Engine* jaguar_create_engine()
{
    return new Engine();
}

void jaguar_destroy_engine(Engine* engine)
{
    delete engine;
}

bool jaguar_initialize(Engine* engine, const char* config_path)
{
    if (!engine) return false;
    if (config_path) {
        return engine->initialize(config_path);
    }
    return engine->initialize();
}

void jaguar_step(Engine* engine, double dt)
{
    if (engine) engine->step(dt);
}

double jaguar_get_time(Engine* engine)
{
    return engine ? engine->get_time() : 0.0;
}

EntityId jaguar_create_entity(Engine* engine, const char* xml_path)
{
    if (!engine || !xml_path) return INVALID_ENTITY_ID;
    return engine->create_entity(xml_path);
}

void jaguar_destroy_entity(Engine* engine, EntityId id)
{
    if (engine) engine->destroy_entity(id);
}

void jaguar_get_position(Engine* engine, EntityId id, double* x, double* y, double* z)
{
    if (!engine || !x || !y || !z) return;
    auto state = engine->get_entity_state(id);
    *x = state.position.x;
    *y = state.position.y;
    *z = state.position.z;
}

void jaguar_set_position(Engine* engine, EntityId id, double x, double y, double z)
{
    if (!engine) return;
    auto state = engine->get_entity_state(id);
    state.position = Vec3{x, y, z};
    engine->set_entity_state(id, state);
}

double jaguar_get_property(Engine* engine, const char* path)
{
    if (!engine || !path) return 0.0;
    return engine->get_property(path);
}

void jaguar_set_property(Engine* engine, const char* path, double value)
{
    if (engine && path) engine->set_property(path, value);
}

} // extern "C"

} // namespace jaguar
