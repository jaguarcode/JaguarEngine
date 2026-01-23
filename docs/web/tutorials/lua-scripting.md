# Lua Scripting Tutorial

This tutorial demonstrates how to use JaguarEngine's Lua bindings for simulation scripting, scenario automation, and embedded scripting applications.

## Prerequisites

- JaguarEngine built with Lua bindings (`-DJAGUAR_BUILD_LUA=ON`)
- Lua 5.4+ (bundled with JaguarEngine if not found on system)

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

### Loading the Module

```lua
-- Add build directory to cpath
package.cpath = package.cpath .. ";./build/?.so;./build/?.dll"

-- Load the module
local jag = require("jaguar")

-- All types are now in global scope: Vec3, Quat, Engine, etc.
```

### Verify Installation

```lua
require("jaguar")

print("JaguarEngine Lua bindings loaded!")
print("Domains available:")
for k, v in pairs(Domain) do
    print("  " .. k)
end
```

---

## Tutorial 1: Hello World - Minimal Simulation

The simplest possible JaguarEngine Lua program:

```lua
require("jaguar")

-- Create and initialize engine
local engine = Engine()
engine:initialize()

-- Create an entity
local entity = engine:create_entity("Test", Domain.Air)

-- Set initial state
local state = EntityState()
state.position = Vec3(0, 0, -1000)  -- 1 km altitude
state.velocity = Vec3(100, 0, 0)    -- 100 m/s forward
state.mass = 1000
engine:set_entity_state(entity, state)

-- Run simulation
for i = 1, 1000 do  -- 10 seconds at 100 Hz
    engine:step(0.01)
end

-- Get final state
local final = engine:get_entity_state(entity)
print(string.format("Final position: (%.1f, %.1f, %.1f)",
    final.position.x, final.position.y, final.position.z))

engine:shutdown()
```

---

## Tutorial 2: Vec3 and Quat Operations

Master the core math types:

```lua
require("jaguar")

print("=== Vec3 Operations ===\n")

-- Construction (both syntaxes work)
local v1 = Vec3(1, 2, 3)        -- Constructor syntax
local v2 = Vec3.new(4, 5, 6)    -- Factory method
local zero = Vec3.zero()         -- Static factory

print(string.format("v1 = %s", tostring(v1)))
print(string.format("v2 = %s", tostring(v2)))

-- Arithmetic operators
local sum = v1 + v2
local diff = v1 - v2
local scaled = v1 * 2.0
local divided = v2 / 2.0
local negated = -v1

print(string.format("v1 + v2 = %s", tostring(sum)))
print(string.format("v1 - v2 = %s", tostring(diff)))
print(string.format("v1 * 2 = %s", tostring(scaled)))
print(string.format("-v1 = %s", tostring(negated)))

-- Methods
print(string.format("v1 length = %.3f", v1:length()))
print(string.format("v1 normalized = %s", tostring(v1:normalized())))
print(string.format("v1 dot v2 = %.3f", v1:dot(v2)))
print(string.format("v1 cross v2 = %s", tostring(v1:cross(v2))))

-- Table conversion (useful for serialization)
local t = v1:to_table()
print(string.format("As table: x=%.1f, y=%.1f, z=%.1f", t.x, t.y, t.z))

local from_table = Vec3.from_table({x = 10, y = 20, z = 30})
print(string.format("From table: %s", tostring(from_table)))

print("\n=== Quaternion Operations ===\n")

-- Construction
local q1 = Quat()                              -- Identity
local q2 = Quat.from_euler(0, 0.1, 0)          -- Pitch 0.1 rad
local q3 = Quat.from_axis_angle(Vec3.unit_z(), math.pi / 4)  -- 45° yaw

print(string.format("Identity: %s", tostring(q1)))
print(string.format("Pitch 0.1 rad: %s", tostring(q2)))
print(string.format("45° yaw: %s", tostring(q3)))

-- Rotation composition
local combined = q2 * q3
print(string.format("Combined rotation: %s", tostring(combined)))

-- Rotate a vector
local forward = Vec3(1, 0, 0)
local rotated = q3:rotate(forward)
print(string.format("Forward after 45° yaw: %s", tostring(rotated)))

-- Extract Euler angles
local roll, pitch, yaw = q3:to_euler()
print(string.format("Euler angles: roll=%.2f, pitch=%.2f, yaw=%.2f",
    roll, pitch, yaw))
```

---

## Tutorial 3: Aircraft Simulation

A complete aircraft simulation with aerodynamics and propulsion:

```lua
require("jaguar")

print("F-16 Flight Simulation (Lua)\n")

-- Access domain models
local air = jaguar.domain.air

-- Initialize engine
local engine = Engine()
engine:initialize()

-- Create aircraft entity
local aircraft = engine:create_entity("F-16", Domain.Air)

-- Configure aerodynamics
local aero = air.AerodynamicsModel()
aero:set_reference_area(27.87)    -- m²
aero:set_reference_chord(3.45)    -- m
aero:set_reference_span(9.45)     -- m

-- Configure propulsion
local prop = air.PropulsionModel()
prop:set_max_thrust(131000.0)       -- N
prop:set_fuel_capacity(3200.0)      -- kg
prop:set_specific_fuel_consumption(2.5e-5)
prop:start()
prop:set_throttle(0.85)             -- 85% power

-- Set initial state (10 km altitude, 250 m/s)
local state = EntityState()
state.position = Vec3(0, 0, -10000)
state.velocity = Vec3(250, 0, 0)
state.orientation = Quat.from_euler(0, 0, 0)
state.mass = 12000
engine:set_entity_state(aircraft, state)

-- Print header
print(string.format("%7s | %9s | %10s | %5s | %8s",
    "Time(s)", "Alt(m)", "Speed(m/s)", "Mach", "Fuel(kg)"))
print(string.rep("-", 55))

-- Simulation loop
local dt = 0.01  -- 100 Hz
local forces = EntityForces()

for i = 0, 5999 do
    local t = i * dt

    -- Get current state and environment
    local s = engine:get_entity_state(aircraft)
    local env = engine:get_environment(aircraft)

    -- Compute forces
    forces:clear()
    aero:compute_forces(s, env, dt, forces)
    prop:compute_forces(s, env, dt, forces)

    -- Add gravity
    forces:add_force(Vec3(0, 0, s.mass * jaguar.G0))

    -- Apply and step
    engine:apply_forces(aircraft, forces)
    engine:step(dt)

    -- Print telemetry every 5 seconds
    if i % 500 == 0 then
        print(string.format("%7.1f | %9.0f | %10.1f | %5.2f | %8.1f",
            t, -s.position.z, s.velocity:length(),
            aero:get_mach(), prop:get_fuel_remaining()))
    end
end

engine:shutdown()
print("\nSimulation complete!")
```

---

## Tutorial 4: Multi-Entity Scenario

Simulate multiple entities interacting:

```lua
require("jaguar")

print("Multi-Entity Combat Scenario\n")

local engine = Engine()
engine:initialize()

-- Create formations
local function create_formation(base_name, count)
    local entities = {}
    for i = 1, count do
        local name = base_name .. "-" .. i
        local entity = engine:create_entity(name, Domain.Air)
        table.insert(entities, entity)
    end
    return entities
end

-- Create blue force (4 aircraft)
local blue = create_formation("Blue", 4)

-- Create red force (4 aircraft)
local red = create_formation("Red", 4)

-- Set blue force initial states (from west)
for i, aircraft in ipairs(blue) do
    local state = EntityState()
    state.position = Vec3(0, (i-1) * 500, -8000)
    state.velocity = Vec3(250, 0, 0)  -- Heading east
    state.mass = 12000
    engine:set_entity_state(aircraft, state)
end

-- Set red force initial states (from east)
for i, aircraft in ipairs(red) do
    local state = EntityState()
    state.position = Vec3(50000, (i-1) * 500, -8000)
    state.velocity = Vec3(-250, 0, 0)  -- Heading west
    state.mass = 12000
    engine:set_entity_state(aircraft, state)
end

print(string.format("Created %d Blue aircraft and %d Red aircraft",
    #blue, #red))
print("Starting simulation...\n")

-- Simulation loop
local dt = 0.01
local duration = 120.0  -- 2 minutes

for t = 0, duration, dt do
    engine:step(dt)

    -- Log every 10 seconds
    if math.floor(t) ~= math.floor(t - dt) and math.floor(t) % 10 == 0 then
        print(string.format("T = %.0fs", t))

        -- Calculate closest approach
        local min_dist = math.huge
        for _, b in ipairs(blue) do
            local blue_state = engine:get_entity_state(b)
            for _, r in ipairs(red) do
                local red_state = engine:get_entity_state(r)
                local diff = blue_state.position - red_state.position
                local dist = diff:length()
                if dist < min_dist then
                    min_dist = dist
                end
            end
        end

        print(string.format("  Closest blue-red distance: %.0f m", min_dist))
    end
end

print("\nSimulation complete!")
engine:shutdown()
```

---

## Tutorial 5: Space Domain - Orbital Mechanics

Simulate a satellite in orbit:

```lua
require("jaguar")

print("LEO Satellite Simulation\n")

local space = jaguar.domain.space

local engine = Engine()
engine:initialize()

local satellite = engine:create_entity("ISS", Domain.Space)

-- Define ISS-like orbit using orbital elements
local orbit = space.OrbitalElements()
orbit.semi_major_axis = jaguar.EARTH_RADIUS + 420000  -- 420 km altitude
orbit.eccentricity = 0.0001
orbit.inclination = 51.6 * jaguar.DEG_TO_RAD
orbit.raan = 0.0
orbit.arg_periapsis = 0.0
orbit.mean_anomaly = 0.0

-- Convert to Cartesian state
local pos, vel = orbit:to_cartesian()

local state = EntityState()
state.position = pos
state.velocity = vel
state.mass = 420000  -- ~ISS mass
engine:set_entity_state(satellite, state)

print(string.format("Initial orbit: %.0f km altitude",
    (orbit.semi_major_axis - jaguar.EARTH_RADIUS) / 1000))
print(string.format("Orbital period: %.1f minutes", orbit:period() / 60))
print("")

-- Configure gravity model
local gravity = space.GravityModel()
gravity:set_fidelity(space.GravityFidelity.J2)

print(string.format("%9s | %7s | %11s | %10s",
    "Time(min)", "Alt(km)", "Speed(km/s)", "Period(min)"))
print(string.rep("-", 50))

-- Simulation loop
local dt = 10.0  -- 10 second steps
local duration = 5400.0  -- ~1 orbit (90 min)
local forces = EntityForces()

for t = 0, duration, dt do
    local s = engine:get_entity_state(satellite)
    local env = engine:get_environment(satellite)

    forces:clear()

    -- Gravity (main force)
    gravity:compute_forces(s, env, dt, forces)

    engine:apply_forces(satellite, forces)
    engine:step(dt)

    -- Print every 10 minutes
    if math.floor(t) % 600 == 0 then
        local alt_km = (s.position:length() - jaguar.EARTH_RADIUS) / 1000
        local speed_kms = s.velocity:length() / 1000

        -- Compute current orbital period from energy
        local r = s.position:length()
        local v = s.velocity:length()
        local energy = v*v/2.0 - jaguar.EARTH_MU/r
        local a = -jaguar.EARTH_MU / (2.0 * energy)
        local period_min = 2.0 * jaguar.PI * math.sqrt(a*a*a / jaguar.EARTH_MU) / 60.0

        print(string.format("%9.2f | %7.1f | %11.3f | %10.2f",
            t/60, alt_km, speed_kms, period_min))
    end
end

engine:shutdown()
print("\nSimulation complete!")
```

---

## Tutorial 6: Coordinate Transforms

Working with geodetic coordinates:

```lua
require("jaguar")

local transforms = jaguar.transforms

print("=== Geodetic to ECEF Conversion ===\n")

-- Example locations
local locations = {
    { name = "New York", lat = 40.7128, lon = -74.0060, alt = 0 },
    { name = "London", lat = 51.5074, lon = -0.1278, alt = 0 },
    { name = "Tokyo", lat = 35.6762, lon = 139.6503, alt = 0 },
}

print(string.format("%-12s | %10s | %10s | %12s | %12s | %12s",
    "Location", "Lat", "Lon", "ECEF X (km)", "ECEF Y (km)", "ECEF Z (km)"))
print(string.rep("-", 80))

for _, loc in ipairs(locations) do
    local lat_rad = loc.lat * jaguar.DEG_TO_RAD
    local lon_rad = loc.lon * jaguar.DEG_TO_RAD

    local ecef = transforms.geodetic_to_ecef(lat_rad, lon_rad, loc.alt)

    print(string.format("%-12s | %10.4f | %10.4f | %12.1f | %12.1f | %12.1f",
        loc.name, loc.lat, loc.lon,
        ecef.x/1000, ecef.y/1000, ecef.z/1000))
end

print("\n=== ECEF to Geodetic (Round-trip) ===\n")

-- Test round-trip conversion
local test_lat = 45.0 * jaguar.DEG_TO_RAD
local test_lon = -90.0 * jaguar.DEG_TO_RAD
local test_alt = 10000  -- 10 km

local ecef = transforms.geodetic_to_ecef(test_lat, test_lon, test_alt)
local lat2, lon2, alt2 = transforms.ecef_to_geodetic(ecef)

print(string.format("Original:  lat=%.4f°, lon=%.4f°, alt=%.1f m",
    test_lat * jaguar.RAD_TO_DEG, test_lon * jaguar.RAD_TO_DEG, test_alt))
print(string.format("Round-trip: lat=%.4f°, lon=%.4f°, alt=%.1f m",
    lat2 * jaguar.RAD_TO_DEG, lon2 * jaguar.RAD_TO_DEG, alt2))
```

---

## Tutorial 7: Callback System

Register callbacks for simulation events:

```lua
require("jaguar")

print("Callback System Demo\n")

local engine = Engine()
engine:initialize()

-- Create test entity
local aircraft = engine:create_entity("Test", Domain.Air)

local state = EntityState()
state.position = Vec3(0, 0, -10000)
state.velocity = Vec3(200, 0, 0)
state.mass = 12000
engine:set_entity_state(aircraft, state)

-- Register step callback
local step_count = 0
local function on_step(dt, time)
    step_count = step_count + 1

    -- Do something every second
    if math.floor(time) ~= math.floor(time - dt) then
        local s = engine:get_entity_state(aircraft)
        print(string.format("T=%.0fs: altitude=%.0f m, speed=%.1f m/s",
            time, -s.position.z, s.velocity:length()))
    end
end

jaguar.register_callback("step", on_step)

-- Register entity event callback
local function on_entity_created(entity_id, name, domain)
    print(string.format("Entity created: %s (ID=%d, domain=%s)",
        name, entity_id, tostring(domain)))
end

jaguar.register_callback("entity_created", on_entity_created)

-- Create another entity to trigger callback
local missile = engine:create_entity("Missile", Domain.Air)

-- Run simulation
print("\nRunning simulation for 10 seconds...\n")
engine:run_for(10.0)

print(string.format("\nTotal steps executed: %d", step_count))

engine:shutdown()
```

---

## Tutorial 8: Configuration and Properties

Work with engine configuration and entity properties:

```lua
require("jaguar")

print("Configuration and Properties Demo\n")

local engine = Engine()
engine:initialize()

-- Engine properties
print("=== Engine Properties ===")
print(string.format("Time: %.2f s", engine:get_time()))
print(string.format("Time scale: %.1f", engine:get_time_scale()))
print(string.format("State: %s", tostring(engine:get_state())))

-- Modify time scale
engine:set_time_scale(2.0)
print(string.format("New time scale: %.1f", engine:get_time_scale()))

-- Set fixed time step
engine:set_fixed_time_step(0.01)

-- Create entity with properties
local aircraft = engine:create_entity("Test", Domain.Air)

local state = EntityState()
state.position = Vec3(0, 0, -5000)
state.velocity = Vec3(200, 0, 0)
state.mass = 10000
engine:set_entity_state(aircraft, state)

-- Access entity properties
print("\n=== Entity Properties ===")

-- Get environment data
local env = engine:get_environment(aircraft)
print(string.format("Latitude: %.4f°", env.latitude * jaguar.RAD_TO_DEG))
print(string.format("Longitude: %.4f°", env.longitude * jaguar.RAD_TO_DEG))
print(string.format("Altitude: %.1f m", env.altitude))
print(string.format("Temperature: %.1f K", env.atmosphere.temperature))
print(string.format("Pressure: %.0f Pa", env.atmosphere.pressure))
print(string.format("Density: %.4f kg/m³", env.atmosphere.density))

-- Simulation control
print("\n=== Simulation Control ===")

engine:step(0.01)
print(string.format("After step: time = %.2f s", engine:get_time()))

-- Run for duration
engine:run_for(1.0)
print(string.format("After run_for(1.0): time = %.2f s", engine:get_time()))

-- Pause and resume
engine:pause()
print(string.format("State after pause: %s", tostring(engine:get_state())))

engine:resume()
print(string.format("State after resume: %s", tostring(engine:get_state())))

engine:shutdown()
```

---

## Common Patterns

### Pattern: Reusable Scenario Framework

```lua
require("jaguar")

-- Scenario base class
Scenario = {}
Scenario.__index = Scenario

function Scenario.new(name)
    local self = setmetatable({}, Scenario)
    self.name = name
    self.engine = Engine()
    self.entities = {}
    self.time = 0
    return self
end

function Scenario:setup()
    self.engine:initialize()
    self:create_entities()
    print(string.format("Scenario '%s' initialized", self.name))
end

function Scenario:create_entities()
    -- Override in subclass
end

function Scenario:step(dt)
    self.engine:step(dt)
    self.time = self.time + dt
end

function Scenario:run(duration, dt)
    dt = dt or 0.01
    local steps = math.floor(duration / dt)

    for i = 1, steps do
        self:step(dt)
        self:on_step(i, steps)
    end

    print(string.format("Scenario completed at T = %.2fs", self.time))
end

function Scenario:on_step(step, total)
    -- Override for custom behavior
end

function Scenario:cleanup()
    self.engine:shutdown()
end

-- Example usage
local InterceptScenario = setmetatable({}, {__index = Scenario})
InterceptScenario.__index = InterceptScenario

function InterceptScenario.new()
    local self = Scenario.new("Intercept")
    setmetatable(self, InterceptScenario)
    return self
end

function InterceptScenario:create_entities()
    self.target = self.engine:create_entity("Target", Domain.Air)
    self.interceptor = self.engine:create_entity("Interceptor", Domain.Air)

    local ts = EntityState()
    ts.position = Vec3(0, 0, -8000)
    ts.velocity = Vec3(200, 0, 0)
    ts.mass = 10000
    self.engine:set_entity_state(self.target, ts)

    local is = EntityState()
    is.position = Vec3(30000, 5000, -10000)
    is.velocity = Vec3(-300, -50, 20)
    is.mass = 15000
    self.engine:set_entity_state(self.interceptor, is)
end

function InterceptScenario:on_step(step, total)
    if step % 1000 == 0 then
        local t_state = self.engine:get_entity_state(self.target)
        local i_state = self.engine:get_entity_state(self.interceptor)
        local range = (t_state.position - i_state.position):length()
        print(string.format("T=%.1fs, Range: %.0f m", self.time, range))
    end
end

-- Run
local scenario = InterceptScenario.new()
scenario:setup()
scenario:run(60.0)
scenario:cleanup()
```

### Pattern: Data Collection

```lua
require("jaguar")

-- Simple data logger
DataLogger = {}
DataLogger.__index = DataLogger

function DataLogger.new()
    local self = setmetatable({}, DataLogger)
    self.data = {}
    return self
end

function DataLogger:log(time, entity_name, state)
    table.insert(self.data, {
        time = time,
        entity = entity_name,
        x = state.position.x,
        y = state.position.y,
        z = state.position.z,
        vx = state.velocity.x,
        vy = state.velocity.y,
        vz = state.velocity.z,
    })
end

function DataLogger:export_csv(filename)
    local file = io.open(filename, "w")
    file:write("time,entity,x,y,z,vx,vy,vz\n")

    for _, record in ipairs(self.data) do
        file:write(string.format("%.3f,%s,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
            record.time, record.entity,
            record.x, record.y, record.z,
            record.vx, record.vy, record.vz))
    end

    file:close()
    print(string.format("Exported %d records to %s", #self.data, filename))
end
```

---

## Debugging Tips

### Debug Entity State

```lua
function debug_entity(engine, entity_id, name)
    local state = engine:get_entity_state(entity_id)
    local env = engine:get_environment(entity_id)

    print(string.format("\n=== %s Debug Info ===", name or "Entity"))
    print(string.format("Position: (%.2f, %.2f, %.2f)",
        state.position.x, state.position.y, state.position.z))
    print(string.format("Velocity: (%.2f, %.2f, %.2f)",
        state.velocity.x, state.velocity.y, state.velocity.z))
    print(string.format("Speed: %.2f m/s", state.velocity:length()))
    print(string.format("Mass: %.1f kg", state.mass))
    print(string.format("Altitude: %.1f m", env.altitude))
    print(string.format("Temperature: %.1f K", env.atmosphere.temperature))
    print(string.format("Density: %.4f kg/m³", env.atmosphere.density))
end
```

### Performance Measurement

```lua
function profile_simulation(engine, steps, dt)
    local start = os.clock()

    for i = 1, steps do
        engine:step(dt)
    end

    local elapsed = os.clock() - start
    local sim_time = steps * dt

    print(string.format("Simulated %.1fs in %.3fs real time", sim_time, elapsed))
    print(string.format("Real-time factor: %.1fx", sim_time / elapsed))
    print(string.format("Steps/second: %.0f", steps / elapsed))
end
```

---

## Next Steps

- [Python Scripting Tutorial](python-scripting.md) - Python scripting guide
- [Examples Guide](examples.md) - More example code
- [Lua API Reference](../api/lua.md) - Complete API documentation
- [Air Domain Tutorial](air-domain.md) - Aircraft simulation deep dive
