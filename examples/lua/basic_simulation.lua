#!/usr/bin/env lua
--[[
JaguarEngine Basic Simulation Example (Lua)

Demonstrates:
- Engine initialization
- Entity creation
- State manipulation
- Simulation stepping
- Property access
]]

print("JaguarEngine Lua Example")
print(string.rep("=", 40))
print(jaguar_info())
print()

-- =============================================================================
-- Math Types Demo
-- =============================================================================
print("Math Types Demo")
print(string.rep("-", 40))

-- Vec3 operations
local v1 = Vec3(1.0, 2.0, 3.0)
local v2 = Vec3(4.0, 5.0, 6.0)

print(string.format("v1 = %s", tostring(v1)))
print(string.format("v2 = %s", tostring(v2)))
print(string.format("v1 + v2 = %s", tostring(v1 + v2)))
print(string.format("v1 dot v2 = %.4f", v1:dot(v2)))
print(string.format("v1 cross v2 = %s", tostring(v1:cross(v2))))
print(string.format("|v1| = %.4f", v1:length()))
print()

-- Quaternion operations
local q1 = Quat.from_euler(0.0, 0.0, deg_to_rad(45))  -- 45 deg yaw
print(string.format("Quaternion (45° yaw): %s", tostring(q1)))

local rotated = q1:rotate(Vec3(1, 0, 0))
print(string.format("Unit X rotated 45° yaw: (%.4f, %.4f, %.4f)",
                    rotated.x, rotated.y, rotated.z))
print()

-- Alternative vec3 factory
local v3 = vec3(7.0, 8.0, 9.0)
print(string.format("Using vec3() factory: %s", tostring(v3)))
print()

-- =============================================================================
-- Engine Initialization
-- =============================================================================
print("Engine Initialization")
print(string.rep("-", 40))

local engine = Engine()
local success = engine:initialize()

if not success then
    print("Failed to initialize engine")
    return
end

print("Engine initialized successfully")
print(string.format("Engine state: %s", tostring(engine:get_state())))
print()

-- =============================================================================
-- Entity Creation
-- =============================================================================
print("Entity Creation")
print(string.rep("-", 40))

-- Create entities in different domains
local aircraft = engine:create_entity("f16", Domain.Air)
local vehicle = engine:create_entity("m1a2", Domain.Land)
local ship = engine:create_entity("destroyer", Domain.Sea)
local satellite = engine:create_entity("gps01", Domain.Space)

print(string.format("Created aircraft: ID=%d", aircraft))
print(string.format("Created vehicle: ID=%d", vehicle))
print(string.format("Created ship: ID=%d", ship))
print(string.format("Created satellite: ID=%d", satellite))
print(string.format("Total entities: %d", engine:entity_count()))
print()

-- Alternative string-based domain (convenient in scripts)
local aircraft2 = engine:create_entity("f22", "Air")
print(string.format("Created F-22 using string domain: ID=%d", aircraft2))
print()

-- =============================================================================
-- State Manipulation
-- =============================================================================
print("State Manipulation")
print(string.rep("-", 40))

-- Get and modify aircraft state
local state = engine:get_entity_state(aircraft)
print(string.format("Initial aircraft state: %s", tostring(state)))

-- Set initial position (1000m altitude)
state.position = Vec3(0.0, 0.0, -1000.0)  -- NED: negative Z = up
state.velocity = Vec3(200.0, 0.0, 0.0)     -- 200 m/s forward
state.mass = 12000.0  -- 12,000 kg
state.inertia = Mat3x3.inertia(9496, 55814, 63100)

engine:set_entity_state(aircraft, state)
print(string.format("Modified aircraft state: %s", tostring(engine:get_entity_state(aircraft))))
print()

-- =============================================================================
-- Simulation Loop
-- =============================================================================
print("Running Simulation")
print(string.rep("-", 40))

local dt = 0.01  -- 100 Hz
local total_time = 1.0  -- 1 second
local steps = math.floor(total_time / dt)

print(string.format("Time step: %.3f s", dt))
print(string.format("Total simulation time: %.1f s", total_time))
print(string.format("Number of steps: %d", steps))
print()

for i = 0, steps - 1 do
    engine:step(dt)

    -- Print progress every 10 steps
    if i % 10 == 0 then
        state = engine:get_entity_state(aircraft)
        print(string.format("t=%6.2fs | pos=(%8.1f, %8.1f, %8.1f)",
                            engine:get_time(),
                            state.position.x,
                            state.position.y,
                            state.position.z))
    end
end

print()
print(string.format("Final simulation time: %.3f s", engine:get_time()))
print()

-- =============================================================================
-- Physical Constants
-- =============================================================================
print("Physical Constants")
print(string.rep("-", 40))
print(string.format("Gravitational constant G = %.4e m³/kg/s²", constants.G))
print(string.format("Earth radius = %.0f m", constants.R_EARTH_EQUATOR))
print(string.format("Standard gravity = %.5f m/s²", constants.G0))
print(string.format("Sea level density = %.3f kg/m³", constants.RHO0))
print(string.format("Pi = %.15f", constants.PI))
print()

-- =============================================================================
-- Unit Conversions
-- =============================================================================
print("Unit Conversions")
print(string.rep("-", 40))
print(string.format("100 ft = %.2f m", ft_to_m(100)))
print(string.format("100 m = %.2f ft", m_to_ft(100)))
print(string.format("100 knots = %.2f m/s", kt_to_ms(100)))
print(string.format("45 degrees = %.4f rad", deg_to_rad(45)))
print(string.format("1 NM = %.0f m", nm_to_m(1)))
print()

-- =============================================================================
-- Table Conversion (Lua-friendly)
-- =============================================================================
print("Table Conversion Demo")
print(string.rep("-", 40))

-- Convert Vec3 to table
local v_table = v1:to_table()
print(string.format("Vec3 as table: {x=%.1f, y=%.1f, z=%.1f}",
                    v_table.x, v_table.y, v_table.z))

-- Create Vec3 from table (named keys)
local v_from_named = Vec3.from_table({x=10.0, y=20.0, z=30.0})
print(string.format("Vec3 from named table: %s", tostring(v_from_named)))

-- Create Vec3 from table (array indices)
local v_from_array = Vec3.from_table({100.0, 200.0, 300.0})
print(string.format("Vec3 from array table: %s", tostring(v_from_array)))
print()

-- =============================================================================
-- Utility Functions
-- =============================================================================
print("Utility Functions")
print(string.rep("-", 40))

-- Clamp
print(string.format("clamp(15, 0, 10) = %.0f", clamp(15, 0, 10)))

-- Lerp
print(string.format("lerp(0, 100, 0.25) = %.1f", lerp(0, 100, 0.25)))

-- Angle wrapping
local big_angle = deg_to_rad(400)
print(string.format("wrap_angle(400°) = %.2f° (%.4f rad)",
                    rad_to_deg(wrap_angle(big_angle)),
                    wrap_angle(big_angle)))
print()

-- =============================================================================
-- Cleanup
-- =============================================================================
engine:shutdown()
print("Engine shutdown complete")
print()
print("Simulation completed successfully!")
