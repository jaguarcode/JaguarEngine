#!/usr/bin/env lua
--[[
    Flight Simulation Example using JaguarEngine Lua Bindings

    This example demonstrates:
    1. Creating an engine and initializing simulation
    2. Creating aircraft entities with different domains
    3. Setting initial states (position, velocity, orientation)
    4. Running the simulation and monitoring state changes
    5. Using the event system for collision and threshold monitoring

    Build JaguarEngine with Lua bindings:
        cmake -B build -DJAGUAR_BUILD_LUA=ON
        cmake --build build
]]

-- Add build directory to package.path
package.cpath = package.cpath .. ";./build/?.so;./build/?.dll"

-- Try to load JaguarEngine
local status, jag = pcall(require, "jaguar")
if not status then
    print("Failed to load jaguar module: " .. tostring(jag))
    print("Build with: cmake -B build -DJAGUAR_BUILD_LUA=ON && cmake --build build")
    os.exit(1)
end

-- Print banner
print(string.rep("=", 60))
print("JaguarEngine Lua Flight Simulation Example")
print(string.format("Version: %s", JAGUAR_VERSION))
print(string.rep("=", 60))

-- ========================================================================
-- 1. Create and Initialize Engine
-- ========================================================================
print("\n[1] Creating and initializing engine...")

local engine = Engine()
if not engine:initialize() then
    print("Failed to initialize engine!")
    os.exit(1)
end

print(string.format("    Engine state: %s", tostring(engine:get_state())))
print(string.format("    Initial time: %.3f s", engine:get_time()))

-- ========================================================================
-- 2. Create Aircraft Entity
-- ========================================================================
print("\n[2] Creating aircraft entity...")

local aircraft_id = engine:create_entity("F16_Alpha", Domain.Air)
if aircraft_id == 0 then  -- INVALID_ENTITY_ID
    print("Failed to create aircraft!")
    os.exit(1)
end

print(string.format("    Created aircraft with ID: %d", aircraft_id))

-- ========================================================================
-- 3. Set Initial State
-- ========================================================================
print("\n[3] Setting initial state...")

local initial_state = EntityState()

-- Position: 10km altitude, at origin (NED: down is negative)
initial_state.position = Vec3(0.0, 0.0, -10000.0)

-- Velocity: 200 m/s forward (approximately Mach 0.6)
initial_state.velocity = Vec3(200.0, 0.0, 0.0)

-- Orientation: Level flight, heading north
initial_state.orientation = Quat.identity()

-- Mass: typical F-16 empty weight
initial_state.mass = 8500.0  -- kg

-- Inertia tensor (simplified)
initial_state.inertia = Mat3x3.inertia(12000.0, 75000.0, 85000.0)

engine:set_entity_state(aircraft_id, initial_state)

local state = engine:get_entity_state(aircraft_id)
print(string.format("    Position: (%.1f, %.1f, %.1f) m",
    state.position.x, state.position.y, state.position.z))
print(string.format("    Velocity: (%.1f, %.1f, %.1f) m/s",
    state.velocity.x, state.velocity.y, state.velocity.z))
print(string.format("    Mass: %.1f kg", state.mass))

-- ========================================================================
-- 4. Set Up Event Monitoring (Optional)
-- ========================================================================
print("\n[4] Setting up event monitoring...")

-- Event log table
local event_log = {}

-- Create event dispatcher
local dispatcher = EventDispatcher()

-- Subscribe to all events
local handler_id = dispatcher:subscribe_all(function(event_data)
    table.insert(event_log, {
        time = event_data.timestamp,
        type = event_data.type,
        source = event_data.source_entity
    })
    return true  -- Continue propagation
end, "flight_monitor", 0)

print(string.format("    Event handler registered: %d", handler_id))

-- ========================================================================
-- 5. Run Simulation
-- ========================================================================
print("\n[5] Running simulation...")

local dt = 0.01  -- 10ms time step (100 Hz)
local sim_duration = 10.0  -- 10 seconds
local num_steps = math.floor(sim_duration / dt)

-- Data recording
local times = {}
local positions_x = {}
local positions_z = {}
local velocities = {}

print(string.format("    Time step: %.1f ms", dt * 1000))
print(string.format("    Duration: %.1f s", sim_duration))
print(string.format("    Total steps: %d", num_steps))
print()

for i = 1, num_steps do
    -- Step simulation
    engine:step(dt)

    -- Record state every 100 steps (1 second intervals)
    if i % 100 == 0 then
        local state = engine:get_entity_state(aircraft_id)
        local t = engine:get_time()

        table.insert(times, t)
        table.insert(positions_x, state.position.x)
        table.insert(positions_z, -state.position.z)  -- Convert to altitude
        table.insert(velocities, state.velocity:length())

        -- Print progress
        print(string.format("    t=%6.2fs | x=%10.1fm | alt=%8.1fm | v=%6.1fm/s",
            t, state.position.x, -state.position.z, state.velocity:length()))
    end
end

-- ========================================================================
-- 6. Display Results
-- ========================================================================
print("\n[6] Final state:")

local final_state = engine:get_entity_state(aircraft_id)
print(string.format("    Position: (%.1f, %.1f, %.1f) m",
    final_state.position.x, final_state.position.y, final_state.position.z))
print(string.format("    Velocity: (%.1f, %.1f, %.1f) m/s",
    final_state.velocity.x, final_state.velocity.y, final_state.velocity.z))
print(string.format("    Distance traveled: %.1f m", final_state.position.x))
print(string.format("    Simulation time: %.3f s", engine:get_time()))

if #event_log > 0 then
    print(string.format("\n    Events logged: %d", #event_log))
end

-- ========================================================================
-- 7. Cleanup
-- ========================================================================
print("\n[7] Shutting down...")

engine:remove_entity(aircraft_id)
engine:shutdown()

print("    Done!")
print(string.rep("=", 60))

-- ========================================================================
-- Bonus: Demonstrate Core Types
-- ========================================================================
print("\n" .. string.rep("=", 60))
print("Core Types Demonstration")
print(string.rep("=", 60))

-- Vec3 operations
print("\nVec3 Operations:")
local v1 = Vec3(1, 2, 3)
local v2 = Vec3(4, 5, 6)
print(string.format("  v1 = %s", tostring(v1)))
print(string.format("  v2 = %s", tostring(v2)))
print(string.format("  v1 + v2 = %s", tostring(v1 + v2)))
print(string.format("  v1 . v2 = %.2f (dot)", v1:dot(v2)))
print(string.format("  v1 x v2 = %s (cross)", tostring(v1:cross(v2))))
print(string.format("  |v1| = %.4f", v1:length()))
print(string.format("  normalized = %s", tostring(v1:normalized())))

-- Quaternion operations
print("\nQuaternion Operations:")
local q = Quat.from_euler(0.1, 0.2, 0.3)
print(string.format("  q from euler(0.1, 0.2, 0.3) = %s", tostring(q)))
local roll, pitch, yaw = q:to_euler()
print(string.format("  back to euler: roll=%.3f, pitch=%.3f, yaw=%.3f", roll, pitch, yaw))

-- Rotate a vector
local v = Vec3(1, 0, 0)
local rotated = q:rotate(v)
print(string.format("  rotate Vec3(1,0,0) = %s", tostring(rotated)))

-- Mat3x3 operations
print("\nMat3x3 Operations:")
local m = Mat3x3.identity()
print(string.format("  Identity matrix trace: %.1f", m:trace()))
local inertia = Mat3x3.inertia(100, 200, 300)
print(string.format("  Inertia matrix determinant: %.1f", inertia:determinant()))

-- Constants
print("\nPhysical Constants:")
print(string.format("  PI = %.10f", constants.PI))
print(string.format("  G0 (standard gravity) = %.5f m/s^2", constants.G0))
print(string.format("  Speed of light = %.0f m/s", constants.C))
print(string.format("  Earth radius (equator) = %.1f m", constants.R_EARTH_EQUATOR))

-- Unit conversions
print("\nUnit Conversions:")
print(string.format("  180 deg = %.6f rad", deg_to_rad(180)))
print(string.format("  PI rad = %.1f deg", rad_to_deg(constants.PI)))
print(string.format("  1 ft = %.4f m", ft_to_m(1)))
print(string.format("  100 kt = %.2f m/s", kt_to_ms(100)))

print("\n" .. string.rep("=", 60))
print("All Lua bindings demonstrated successfully!")
print(string.rep("=", 60))
