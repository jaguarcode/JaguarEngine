# Lua API Reference

Lua scripting bindings for JaguarEngine using sol2.

## Overview

JaguarEngine provides Lua bindings for scripting simulations, creating scenarios, and automating entity behaviors. Lua scripts can control entities, query environment data, and implement custom logic.

## Installation

### Build from Source

```bash
cd JaguarEngine

# Configure with Lua bindings enabled
cmake -B build -DJAGUAR_BUILD_LUA=ON

# Build
cmake --build build --parallel

# The module will be at build/jaguar.so (or jaguar.dll on Windows)
```

**Requirements:**
- Lua 5.4+ (bundled if not found on system)
- sol2 (auto-fetched by CMake)

### Loading the Module

```lua
-- Add build directory to cpath
package.cpath = package.cpath .. ";./build/?.so;./build/?.dll"

-- Load the module
local jag = require("jaguar")

-- All types are now in global scope: Vec3, Quat, Engine, etc.
```

## Basic Usage

```lua
-- Load the module
require("jaguar")

-- Create and initialize engine
local engine = Engine()
engine:initialize()

-- Create an entity
local aircraft = engine:create_entity("F-16", Domain.Air)

-- Set state
local state = EntityState()
state.position = Vec3(0, 0, -5000)
state.velocity = Vec3(200, 0, 0)
state.mass = 12000
engine:set_entity_state(aircraft, state)

-- Run simulation
for i = 1, 10000 do
    engine:step(0.01)
end

engine:shutdown()
```

## Core Types

### Vec3

```lua
-- Construction (both syntaxes work)
local v = Vec3()              -- (0, 0, 0)
local v = Vec3(1, 2, 3)       -- (1, 2, 3)
local v = Vec3.new(1, 2, 3)   -- Alternative syntax

-- Properties
local x, y, z = v.x, v.y, v.z

-- Methods
local length = v:length()
local length_sq = v:length_squared()
local unit = v:normalized()
local dot = v:dot(other)
local cross = v:cross(other)

-- Static factory methods
local zero = Vec3.zero()
local unit_x = Vec3.unit_x()
local unit_y = Vec3.unit_y()
local unit_z = Vec3.unit_z()

-- Operators
local v3 = v1 + v2
local v3 = v1 - v2
local v3 = v * 2.0
local v3 = v / 2.0
local neg = -v

-- String conversion
print(v)  -- "Vec3(1.000000, 2.000000, 3.000000)"

-- Table conversion
local t = v:to_table()       -- {x=1, y=2, z=3}
local v = Vec3.from_table(t) -- Back to Vec3
```

### Quat

```lua
-- Construction
local q = Quat()                           -- Identity
local q = Quat(w, x, y, z)
local q = Quat.identity()                  -- Factory method

-- From rotation
local q = Quat.from_euler(roll, pitch, yaw)
local q = Quat.from_axis_angle(axis, angle)

-- Properties
local w, x, y, z = q.w, q.x, q.y, q.z

-- Methods
local q_norm = q:normalized()
local q_conj = q:conjugate()
local q_inv = q:inverse()
local v_rot = q:rotate(v)
local roll, pitch, yaw = q:to_euler()
local mat = q:to_rotation_matrix()
local n = q:norm()
local n_sq = q:norm_squared()

-- Operators
local q3 = q1 * q2   -- Rotation composition
local v_rot = q * v  -- Rotate vector

-- Table conversion
local t = q:to_table()       -- {w=1, x=0, y=0, z=0}
local q = Quat.from_table(t) -- Back to Quat
```

### Mat3x3

```lua
-- Construction
local m = Mat3x3()              -- Zero matrix
local m = Mat3x3.identity()     -- Identity matrix
local m = Mat3x3.zero()         -- Zero matrix
local m = Mat3x3.diagonal(a, b, c)
local m = Mat3x3.inertia(ixx, iyy, izz)
local m = Mat3x3.inertia(ixx, iyy, izz, ixy, ixz, iyz)

-- Element access (0-indexed)
local value = m:get(row, col)
m:set(row, col, value)

-- Methods
local mt = m:transpose()
local det = m:determinant()
local minv = m:inverse()
local tr = m:trace()

-- Operators
local v_out = m * v_in     -- Matrix-vector multiply
local m_out = m1 + m2      -- Matrix addition
local m_scaled = m * 2.0   -- Scalar multiply
```

## Engine API

```lua
-- Initialization
local engine = Engine()
engine:initialize()
-- or with config file
engine:initialize("config/simulation.xml")

-- Entity management
local id = engine:create_entity("name", Domain.Air)
local id = engine:create_entity("name", "air")  -- String domain also works
engine:destroy_entity(id)
local exists = engine:entity_exists(id)

-- State access
local state = engine:get_entity_state(id)
engine:set_entity_state(id, state)

-- Simulation control
engine:step(dt)           -- Single step
engine:run()              -- Run until stopped
engine:run_for(duration)  -- Run for duration seconds
engine:pause()
engine:resume()
engine:stop()

-- Time access
local t = engine:get_time()
local scale = engine:get_time_scale()
engine:set_time_scale(2.0)     -- 2x speed
engine:set_fixed_time_step(0.01)

-- State query
local state = engine:get_state()  -- SimulationState enum
local init = engine:is_initialized()

-- Property access
local val = engine:get_property("time.scale")
local val = engine:get_property(entity_id, "fuel.remaining")
engine:set_property("time.scale", 2.0)
engine:set_property(entity_id, "throttle", 0.8)

-- Cleanup
engine:shutdown()
```

## Enumerations

```lua
-- Domain enum
Domain.Air
Domain.Land
Domain.Sea
Domain.Space
Domain.Generic

-- CoordinateFrame enum
CoordinateFrame.ECEF
CoordinateFrame.ECI
CoordinateFrame.NED
CoordinateFrame.ENU
CoordinateFrame.Body

-- SimulationState enum
SimulationState.Uninitialized
SimulationState.Initialized
SimulationState.Running
SimulationState.Paused
SimulationState.Stopped
SimulationState.Error
```

## EntityState

```lua
local state = EntityState()

-- Position and velocity
state.position = Vec3(x, y, z)           -- m (ECEF)
state.velocity = Vec3(vx, vy, vz)        -- m/s
state.acceleration = Vec3(ax, ay, az)    -- m/s²

-- Orientation
state.orientation = Quat.from_euler(roll, pitch, yaw)
state.angular_velocity = Vec3(p, q, r)   -- rad/s
state.angular_accel = Vec3(pdot, qdot, rdot)  -- rad/s²

-- Mass properties
state.mass = 12000                        -- kg
state.inertia = Mat3x3.inertia(ixx, iyy, izz)  -- kg·m²

-- String representation
print(state)  -- "EntityState(pos=(x, y, z), mass=12000)"
```

## EntityForces

```lua
local forces = EntityForces()

-- Access properties
local f = forces.force    -- Vec3, N
local t = forces.torque   -- Vec3, N·m

-- Modification
forces:clear()
forces:add_force(Vec3(fx, fy, fz))
forces:add_torque(Vec3(tx, ty, tz))
forces:add_force_at_point(force, point, cg)  -- Applies force and computes torque

-- String representation
print(forces)  -- "EntityForces(force=(fx, fy, fz))"
```

## Environment

```lua
local env = engine:get_environment(entity_id)

-- Position
local lat = env.latitude      -- rad
local lon = env.longitude     -- rad
local alt = env.altitude      -- m

-- Atmosphere
local temp = env.atmosphere.temperature     -- K
local pressure = env.atmosphere.pressure    -- Pa
local density = env.atmosphere.density      -- kg/m³
local wind = env.atmosphere.wind            -- Vec3, m/s (NED)

-- Terrain
local elev = env.terrain.elevation          -- m
local normal = env.terrain.normal           -- Vec3
local slope = env.terrain.slope_angle       -- rad

-- Ocean
local wave_height = env.ocean.surface_elevation
local over_water = env.over_water
```

## Domain Models

### Air Domain

```lua
local air = jaguar.domain.air

-- Aerodynamics
local aero = air.AerodynamicsModel()
aero:set_reference_area(28.0)
aero:set_reference_chord(3.5)
aero:set_reference_span(10.0)
aero:set_elevator(elevator_deg)
aero:compute_forces(state, env, dt, forces)

local cl = aero:get_cl()
local alpha = aero:get_alpha()

-- Propulsion
local prop = air.PropulsionModel()
prop:set_max_thrust(75000)
prop:set_fuel_capacity(3000)
prop:set_throttle(0.8)
prop:start()
prop:compute_forces(state, env, dt, forces)
```

### Land Domain

```lua
local land = jaguar.domain.land

-- Soil properties
local soil = land.SoilProperties.DrySand()
-- or customize
local custom_soil = land.SoilProperties()
custom_soil.k_c = 5.0
custom_soil.k_phi = 1000.0

-- Terramechanics
local terra = land.TerramechanicsModel()
terra:set_contact_area(0.63, 4.6)
terra:set_vehicle_weight(62000 * jaguar.G0)
terra:set_soil(soil)
terra:compute_forces(state, env, dt, forces)

local sinkage = terra:get_sinkage()
```

### Sea Domain

```lua
local sea = jaguar.domain.sea

-- Sea state
local sea_state = sea.SeaState.FromNATOSeaState(4)

-- Buoyancy
local buoyancy = sea.BuoyancyModel()
buoyancy:set_displaced_volume(8390)
buoyancy:set_metacentric_height(2.5)
buoyancy:compute_forces(state, env, dt, forces)

local draft = buoyancy:get_draft()
local heel = buoyancy:get_heel()

-- Hydrodynamics
local hydro = sea.HydrodynamicsModel()
hydro:set_hull_coefficients(-0.04, -0.01, -0.4, 0.05, -0.1, -0.05)
hydro:set_rudder_angle(rudder_rad)
hydro:set_propeller_rpm(120)
```

### Space Domain

```lua
local space = jaguar.domain.space

-- Orbital elements
local orbit = space.OrbitalElements()
orbit.semi_major_axis = jaguar.EARTH_RADIUS + 420000
orbit.eccentricity = 0.0001
orbit.inclination = 51.6 * jaguar.DEG_TO_RAD

local pos, vel = orbit:to_cartesian()
local period = orbit:period()

-- TLE parsing
local tle = space.TLE()
tle:parse(line1, line2)

-- SGP4 propagator
local sgp4 = space.SGP4Propagator()
sgp4:initialize(tle)

for minutes = 0, 24*60, 10 do
    local pos_km, vel_kms = sgp4:propagate(minutes)
end

-- Gravity model
local gravity = space.GravityModel()
gravity:set_fidelity(space.GravityFidelity.J2)
```

## Coordinate Transforms

```lua
local transforms = jaguar.transforms

-- Geodetic <-> ECEF
local ecef = transforms.geodetic_to_ecef(lat_rad, lon_rad, alt_m)
local lat, lon, alt = transforms.ecef_to_geodetic(ecef)

-- ECEF <-> ECI
local eci = transforms.ecef_to_eci(ecef, julian_date)
local ecef = transforms.eci_to_ecef(eci, julian_date)

-- Julian date
local jd = transforms.julian_date(2024, 1, 1, 12, 0, 0.0)
```

## Constants

```lua
-- Mathematical
jaguar.PI
jaguar.TWO_PI
jaguar.DEG_TO_RAD
jaguar.RAD_TO_DEG

-- Physical
jaguar.G0              -- Standard gravity (9.80665 m/s²)
jaguar.EARTH_RADIUS    -- WGS84 equatorial radius
jaguar.EARTH_MU        -- Earth gravitational parameter

-- Enumerations
jaguar.Domain.Air
jaguar.Domain.Land
jaguar.Domain.Sea
jaguar.Domain.Space
```

## Callbacks and Hooks

```lua
-- Register update callback
function on_step(dt, time)
    -- Called each simulation step
    local state = engine:get_entity_state(my_aircraft)
    -- Update controls based on state
end

jaguar.register_callback("step", on_step)

-- Register entity event
function on_entity_created(entity_id, name, domain)
    print("Created: " .. name)
end

jaguar.register_callback("entity_created", on_entity_created)
```

## Example: Simple Scenario

```lua
-- Initialize
local engine = jaguar.engine
engine:initialize()

-- Create blue force
local blue = {}
for i = 1, 4 do
    local id = engine:create_entity("Blue-" .. i, jaguar.Domain.Air)
    local state = jaguar.EntityState()
    state.position = jaguar.Vec3(0, (i-1) * 500, -8000)
    state.velocity = jaguar.Vec3(250, 0, 0)
    state.mass = 12000
    engine:set_entity_state(id, state)
    table.insert(blue, id)
end

-- Create red force
local red = {}
for i = 1, 4 do
    local id = engine:create_entity("Red-" .. i, jaguar.Domain.Air)
    local state = jaguar.EntityState()
    state.position = jaguar.Vec3(50000, (i-1) * 500, -8000)
    state.velocity = jaguar.Vec3(-250, 0, 0)
    state.mass = 12000
    engine:set_entity_state(id, state)
    table.insert(red, id)
end

-- Simulation loop
local dt = 0.01
local duration = 120.0  -- 2 minutes
local time = 0

while time < duration do
    -- Update all entities
    for _, id in ipairs(blue) do
        update_aircraft(id, dt)
    end
    for _, id in ipairs(red) do
        update_aircraft(id, dt)
    end

    engine:step(dt)
    time = time + dt

    -- Log every second
    if math.floor(time) ~= math.floor(time - dt) then
        print(string.format("Time: %.1f s", time))
    end
end

engine:shutdown()
```

## See Also

- [API Overview](overview.md) - Complete API index
- [Python API](python.md) - Python bindings
- [Configuration](configuration.md) - Configuration reference

