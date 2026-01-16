# Lua API Reference {#lua_api}

JaguarEngine provides Lua bindings through the `jaguar` module, enabling lightweight scripting and rapid prototyping.

## Installation

Build JaguarEngine with Lua support:

```bash
cmake -DJAGUAR_BUILD_LUA=ON ..
cmake --build .
```

The Lua bindings will be available in the build directory.

## Module Overview

```lua
local jag = require("jaguar")

-- Version information
print("JaguarEngine version: " .. jag.version())
```

## Classes

### Vec3

3D vector class for positions, velocities, and forces.

```lua
-- Construction
local v1 = jag.Vec3()              -- Zero vector
local v2 = jag.Vec3(1.0, 2.0, 3.0) -- From components

-- Properties
print(v1.x, v1.y, v1.z)

-- Operations
local v3 = v1 + v2                 -- Addition
local v4 = v1 - v2                 -- Subtraction
local v5 = v1 * 2.0                -- Scalar multiplication
local v6 = v1 / 2.0                -- Scalar division

-- Methods
local dot_product = v1:dot(v2)
local cross_product = v1:cross(v2)
local length = v1:length()
local length_sq = v1:length_squared()
local normalized = v1:normalized()

-- String representation
print(tostring(v1))                -- "Vec3(x, y, z)"
```

### Quat

Quaternion class for orientations and rotations.

```lua
-- Construction
local q1 = jag.Quat.identity()                  -- Identity quaternion
local q2 = jag.Quat.from_euler(roll, pitch, yaw)  -- From Euler angles (rad)
local q3 = jag.Quat.from_axis_angle(axis, angle)  -- From axis-angle

-- Properties
print(q1.w, q1.x, q1.y, q1.z)

-- Operations
local q4 = q1 * q2                 -- Quaternion multiplication
local rotated_vec = q1:rotate(v)   -- Rotate a vector

-- Methods
local q_conj = q1:conjugate()
local q_inv = q1:inverse()
local roll, pitch, yaw = q1:to_euler()  -- To Euler angles
```

### Mat3x3

3x3 matrix class for inertia tensors and rotations.

```lua
-- Construction
local m1 = jag.Mat3x3.identity()
local m2 = jag.Mat3x3.zero()
local m3 = jag.Mat3x3.inertia(Ixx, Iyy, Izz)  -- Diagonal inertia

-- Element access
local val = m1:get(0, 0)           -- Get element
m1:set(0, 0, value)                -- Set element

-- Operations
local m4 = m1 * m2                 -- Matrix multiplication
local v = m1 * vec                 -- Matrix-vector multiplication

-- Methods
local m_t = m1:transpose()
local det = m1:determinant()
local m_inv = m1:inverse()
local tr = m1:trace()
```

### Domain

Enumeration for simulation domains.

```lua
jag.Domain.Air               -- Aircraft, missiles
jag.Domain.Land              -- Ground vehicles
jag.Domain.Sea               -- Ships, submarines
jag.Domain.Space             -- Satellites, spacecraft
jag.Domain.Unknown           -- Unspecified
```

### CoordinateFrame

Enumeration for coordinate frames.

```lua
jag.CoordinateFrame.ECEF     -- Earth-Centered Earth-Fixed
jag.CoordinateFrame.ECI      -- Earth-Centered Inertial
jag.CoordinateFrame.NED      -- North-East-Down
jag.CoordinateFrame.ENU      -- East-North-Up
jag.CoordinateFrame.Body     -- Body-fixed frame
jag.CoordinateFrame.Wind     -- Wind-aligned frame
jag.CoordinateFrame.Stability  -- Stability axis frame
```

### SimulationState

Enumeration for simulation states.

```lua
jag.SimulationState.Uninitialized
jag.SimulationState.Initialized
jag.SimulationState.Running
jag.SimulationState.Paused
jag.SimulationState.Stopped
```

### EntityState

Structure containing entity kinematic state.

```lua
local state = jag.EntityState()

-- Position and velocity (Vec3)
state.position = jag.Vec3(x, y, z)
state.velocity = jag.Vec3(vx, vy, vz)
state.acceleration = jag.Vec3(ax, ay, az)

-- Orientation and angular rates
state.orientation = jag.Quat.from_euler(r, p, y)
state.angular_velocity = jag.Vec3(p, q, r)
state.angular_acceleration = jag.Vec3(p_dot, q_dot, r_dot)

-- Mass properties
state.mass = 12000.0  -- kg
state.inertia = jag.Mat3x3.inertia(Ixx, Iyy, Izz)
state.cg_offset = jag.Vec3(0, 0, 0)

-- Metadata
state.coordinate_frame = jag.CoordinateFrame.NED
state.time = 0.0
```

### EntityForces

Structure containing forces and moments acting on an entity.

```lua
local forces = jag.EntityForces()

-- Force components
forces.total_force = jag.Vec3(fx, fy, fz)
forces.total_moment = jag.Vec3(mx, my, mz)
forces.gravity = jag.Vec3(0, 0, mg)
forces.aerodynamic = jag.Vec3(lift, drag, side)
forces.thrust = jag.Vec3(Tx, Ty, Tz)
```

### Engine

Main simulation engine class.

```lua
-- Create engine
local engine = jag.Engine()

-- Lifecycle
local success = engine:initialize()           -- Initialize with defaults
local success = engine:initialize("config.xml")  -- Initialize with config
engine:shutdown()                             -- Release resources

-- Simulation control
engine:step(dt)                               -- Advance by dt seconds
engine:run()                                  -- Run continuously
engine:pause()                                -- Pause simulation
engine:resume()                               -- Resume simulation
engine:stop()                                 -- Stop simulation

-- State queries
local state = engine:get_state()              -- Get SimulationState
local time = engine:get_time()                -- Get simulation time
local dt = engine:get_dt()                    -- Get time step

-- Entity management
local entity_id = engine:create_entity("name", jag.Domain.Air)
engine:remove_entity(entity_id)
local entity_state = engine:get_entity_state(entity_id)
engine:set_entity_state(entity_id, state)
local forces = engine:get_entity_forces(entity_id)
local count = engine:entity_count()
local entities = engine:get_all_entities()
```

## Constants Module

Physical constants are available in the `jag.constants` table:

```lua
-- Universal constants
jag.constants.G          -- Gravitational constant (m³/kg/s²)
jag.constants.C          -- Speed of light (m/s)
jag.constants.PI         -- Pi
jag.constants.TWO_PI     -- 2*Pi
jag.constants.HALF_PI    -- Pi/2
jag.constants.DEG_TO_RAD -- Degrees to radians conversion
jag.constants.RAD_TO_DEG -- Radians to degrees conversion

-- Earth parameters
jag.constants.MU_EARTH        -- Earth gravitational parameter (m³/s²)
jag.constants.R_EARTH_EQUATOR -- Earth equatorial radius (m)
jag.constants.R_EARTH_POLAR   -- Earth polar radius (m)
jag.constants.OMEGA_EARTH     -- Earth rotation rate (rad/s)
jag.constants.J2              -- Earth J2 coefficient
jag.constants.FLATTENING      -- Earth flattening

-- Atmosphere
jag.constants.G0        -- Standard gravity (m/s²)
jag.constants.RHO0      -- Sea level density (kg/m³)
jag.constants.P0        -- Sea level pressure (Pa)
jag.constants.T0        -- Sea level temperature (K)
jag.constants.GAMMA_AIR -- Air specific heat ratio
jag.constants.R_AIR     -- Air gas constant (J/kg/K)
```

## Unit Conversion Functions

```lua
-- Length conversions
local meters = jag.ft_to_m(feet)
local feet = jag.m_to_ft(meters)
local meters = jag.nm_to_m(nautical_miles)

-- Speed conversions
local ms = jag.kt_to_ms(knots)
local knots = jag.ms_to_kt(ms)

-- Angle conversions
local radians = jag.deg_to_rad(degrees)
local degrees = jag.rad_to_deg(radians)
```

## Example: Complete Simulation

```lua
-- basic_simulation.lua
-- Example simulation using JaguarEngine Lua bindings

local jag = require("jaguar")

-- Helper function to print state
local function print_state(time, state)
    local pos = state.position
    local vel = state.velocity
    local speed = vel:length()
    print(string.format(
        "t=%.2f  pos=(%.1f, %.1f, %.1f)  speed=%.1f m/s",
        time, pos.x, pos.y, pos.z, speed
    ))
end

-- Main simulation function
local function run_simulation()
    print("JaguarEngine Lua Simulation")
    print("===========================")

    -- Initialize engine
    local engine = jag.Engine()
    if not engine:initialize() then
        print("ERROR: Failed to initialize engine")
        return
    end

    -- Create aircraft entity
    local aircraft = engine:create_entity("f16", jag.Domain.Air)
    print("Created aircraft entity: " .. aircraft)

    -- Configure initial state
    local state = engine:get_entity_state(aircraft)
    state.position = jag.Vec3(0, 0, -1000)  -- 1km altitude (NED)
    state.velocity = jag.Vec3(200, 0, 0)    -- 200 m/s
    state.mass = 12000                       -- 12 tons
    state.inertia = jag.Mat3x3.inertia(9496, 55814, 63100)
    state.orientation = jag.Quat.identity()
    engine:set_entity_state(aircraft, state)

    print("\nInitial conditions:")
    print_state(0, state)

    -- Simulation parameters
    local dt = 0.01        -- 100 Hz
    local duration = 10.0  -- 10 seconds

    -- Collect data for analysis
    local times = {}
    local positions = {}
    local speeds = {}

    -- Run simulation
    print("\nRunning simulation...")
    while engine:get_time() < duration do
        engine:step(dt)

        -- Get updated state
        local current = engine:get_entity_state(aircraft)
        local time = engine:get_time()

        -- Store data (every 0.5 seconds)
        if math.floor(time * 2) > #times then
            table.insert(times, time)
            table.insert(positions, {
                x = current.position.x,
                y = current.position.y,
                z = current.position.z
            })
            table.insert(speeds, current.velocity:length())
        end
    end

    -- Print final state
    print("\nFinal state:")
    print_state(engine:get_time(), engine:get_entity_state(aircraft))

    -- Print trajectory summary
    print("\nTrajectory Summary:")
    print("-------------------")
    for i, t in ipairs(times) do
        print(string.format(
            "t=%.1fs: N=%.1fm E=%.1fm Alt=%.1fm Speed=%.1fm/s",
            t, positions[i].x, positions[i].y, -positions[i].z, speeds[i]
        ))
    end

    -- Cleanup
    engine:shutdown()
    print("\nSimulation complete!")
end

-- Run the simulation
run_simulation()
```

## Multi-Entity Simulation

```lua
local jag = require("jaguar")

local function multi_entity_simulation()
    local engine = jag.Engine()
    engine:initialize()

    -- Create multiple entities
    local aircraft = engine:create_entity("f16", jag.Domain.Air)
    local tank = engine:create_entity("m1a2", jag.Domain.Land)
    local ship = engine:create_entity("ddg", jag.Domain.Sea)
    local satellite = engine:create_entity("gps", jag.Domain.Space)

    -- Configure aircraft
    local air_state = engine:get_entity_state(aircraft)
    air_state.position = jag.Vec3(0, 0, -5000)  -- 5km altitude
    air_state.velocity = jag.Vec3(250, 0, 0)
    engine:set_entity_state(aircraft, air_state)

    -- Configure tank
    local land_state = engine:get_entity_state(tank)
    land_state.position = jag.Vec3(1000, 500, 0)  -- Ground level
    land_state.velocity = jag.Vec3(15, 0, 0)      -- 15 m/s
    engine:set_entity_state(tank, land_state)

    -- Configure ship
    local sea_state = engine:get_entity_state(ship)
    sea_state.position = jag.Vec3(5000, 2000, 0)  -- Sea level
    sea_state.velocity = jag.Vec3(10, 5, 0)       -- 10 knots
    engine:set_entity_state(ship, sea_state)

    -- Configure satellite (LEO)
    local space_state = engine:get_entity_state(satellite)
    space_state.position = jag.Vec3(0, 0, -400000)  -- 400km altitude
    space_state.velocity = jag.Vec3(7660, 0, 0)     -- Orbital velocity
    engine:set_entity_state(satellite, space_state)

    print("Entity count: " .. engine:entity_count())

    -- Simulate
    for i = 1, 100 do
        engine:step(0.1)
    end

    print("Simulation time: " .. engine:get_time() .. " seconds")
    engine:shutdown()
end

multi_entity_simulation()
```

## Error Handling

```lua
local jag = require("jaguar")

local function safe_simulation()
    local engine = jag.Engine()

    -- Check initialization
    if not engine:initialize() then
        error("Failed to initialize engine")
    end

    -- Use pcall for protected calls
    local success, entity = pcall(function()
        return engine:create_entity("test", jag.Domain.Air)
    end)

    if not success then
        print("Error creating entity: " .. tostring(entity))
        engine:shutdown()
        return
    end

    -- Safe state access
    local state = engine:get_entity_state(entity)
    if state then
        print("Entity position: " .. tostring(state.position))
    end

    engine:shutdown()
end

safe_simulation()
```

