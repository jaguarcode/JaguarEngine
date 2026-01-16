# JaguarEngine Architecture Document

**Version**: 1.5.0
**Date**: 2026-01-16
**Status**: Phase 6 Complete - Phase 7 Planning

---

## 1. Executive Summary

JaguarEngine is a next-generation multi-domain physics simulation platform designed for defense modeling and simulation (M&S). Built upon the proven architectural patterns of JSBSim, the engine extends data-driven simulation capabilities to encompass Air, Land, Sea, and Space domains within a unified framework.

### 1.1 Design Goals

| Goal | Description | Priority |
|------|-------------|----------|
| **Multi-Domain Integration** | Unified simulation of air, land, sea, and space entities | Critical |
| **High Performance** | Real-time and faster-than-real-time execution | Critical |
| **Data-Driven Design** | XML-configurable entities without recompilation | High |
| **Extensibility** | Plugin architecture for custom physics models | High |
| **Interoperability** | DIS/HLA protocol support for LVC integration | High |
| **Fidelity** | Physics accuracy validated against NATO standards | Medium |

### 1.2 Key Architectural Decisions

1. **Data-Oriented Design (DOD)**: Structure of Arrays (SoA) memory layout for cache efficiency
2. **Component-Based Physics**: Modular force generators attached to entities
3. **Property System**: Hierarchical key-value access for runtime configuration (JSBSim-inspired)
4. **Coordinate System Hierarchy**: ECEF base with domain-specific local frames
5. **64-bit Precision**: Double-precision floating point for geodetic accuracy

---

## 2. System Architecture Overview

```
┌─────────────────────────────────────────────────────────────────────────┐
│                         EXTERNAL INTERFACES                              │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐    │
│  │   Python    │  │    Lua      │  │  DIS/HLA    │  │     CIG     │    │
│  │  (pybind11) │  │   (sol2)    │  │  Network    │  │ (UE/Unity)  │    │
│  └──────┬──────┘  └──────┬──────┘  └──────┬──────┘  └──────┬──────┘    │
└─────────┼────────────────┼────────────────┼────────────────┼────────────┘
          │                │                │                │
          └────────────────┴────────┬───────┴────────────────┘
                                    │
┌───────────────────────────────────┴────────────────────────────────────┐
│                         PUBLIC API LAYER                                │
│                      jaguar::Engine (Facade)                           │
└───────────────────────────────────┬────────────────────────────────────┘
                                    │
┌───────────────────────────────────┴────────────────────────────────────┐
│                    PHYSICS ENGINE EXECUTIVE                             │
│  ┌──────────────────────────────────────────────────────────────────┐  │
│  │                    PhysicsEngineExec                              │  │
│  │  ┌────────────┐ ┌────────────┐ ┌────────────┐ ┌────────────┐    │  │
│  │  │  Entity    │ │  Physics   │ │  Property  │ │   Time     │    │  │
│  │  │  Manager   │ │  System    │ │  Manager   │ │  Manager   │    │  │
│  │  └────────────┘ └────────────┘ └────────────┘ └────────────┘    │  │
│  └──────────────────────────────────────────────────────────────────┘  │
└───────────────────────────────────┬────────────────────────────────────┘
                                    │
┌───────────────────────────────────┴────────────────────────────────────┐
│                        DOMAIN PHYSICS LAYER                             │
│  ┌─────────────┐ ┌─────────────┐ ┌─────────────┐ ┌─────────────┐      │
│  │     AIR     │ │    LAND     │ │     SEA     │ │    SPACE    │      │
│  │ Aerodynamics│ │Terramechanics│ │Hydrodynamics│ │  Astrodynamics│    │
│  │ Propulsion  │ │ Suspension  │ │  Buoyancy   │ │   Gravity   │      │
│  │ FCS/Autopilot│ │   MBS      │ │    RAO      │ │  SGP4/SDP4  │      │
│  └─────────────┘ └─────────────┘ └─────────────┘ └─────────────┘      │
└───────────────────────────────────┬────────────────────────────────────┘
                                    │
┌───────────────────────────────────┴────────────────────────────────────┐
│                      ENVIRONMENT SERVICES                               │
│  ┌─────────────────┐ ┌─────────────────┐ ┌─────────────────┐          │
│  │     TERRAIN     │ │   ATMOSPHERE    │ │      OCEAN      │          │
│  │   Digital Twin  │ │  US Std 1976    │ │ Wave Spectrum   │          │
│  │   GDAL/GIS      │ │  JBH08 (Space)  │ │ PM/JONSWAP      │          │
│  │   Quadtree LOD  │ │  MODTRAN LUT    │ │ Sea State       │          │
│  └─────────────────┘ └─────────────────┘ └─────────────────┘          │
└───────────────────────────────────┬────────────────────────────────────┘
                                    │
┌───────────────────────────────────┴────────────────────────────────────┐
│                         CORE SERVICES                                   │
│  ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌──────────┐    │
│  │  Memory  │ │ Threading│ │   Math   │ │   I/O    │ │  Config  │    │
│  │  Pools   │ │Work-Steal│ │   SIMD   │ │  Async   │ │   XML    │    │
│  └──────────┘ └──────────┘ └──────────┘ └──────────┘ └──────────┘    │
└────────────────────────────────────────────────────────────────────────┘
```

---

## 3. Core Subsystems

### 3.1 PhysicsEngineExec (Master Orchestrator)

The central coordinator responsible for simulation lifecycle management.

**Responsibilities**:

- Initialize and configure all subsystems
- Execute the main simulation loop
- Coordinate time advancement across all entities
- Manage module registration and discovery

**Key Methods**:

```cpp
class PhysicsEngineExec {
public:
    bool Initialize(const std::string& config_path);
    void Run();                          // Main simulation loop
    void RunFor(Real duration_sec);      // Run for specific duration
    void Step(Real dt);                  // Single frame advance
    void Shutdown();

    EntityManager& GetEntityManager();
    PhysicsSystem& GetPhysicsSystem();
    PropertyManager& GetPropertyManager();
    TimeManager& GetTimeManager();
};
```

### 3.2 Entity Management System

**Design Philosophy**: Entities are lightweight identifiers; data lives in contiguous arrays.

```cpp
// Entity is just an ID with component tracking
struct Entity {
    EntityId id;              // Unique identifier
    ComponentMask components; // Bitfield of attached components
    Domain primary_domain;    // Primary physics domain
    PropertyNode* properties; // Property tree root
};

// EntityState stored in Structure of Arrays (SoA)
class EntityStateStorage {
private:
    // Cache-line aligned arrays for SIMD operations
    alignas(64) std::vector<Vec3d> positions_;      // ECEF meters
    alignas(64) std::vector<Vec3d> velocities_;     // ECEF m/s
    alignas(64) std::vector<Quatd> orientations_;   // Body→ECEF
    alignas(64) std::vector<Vec3d> angular_vels_;   // Body rad/s
    alignas(64) std::vector<Real>  masses_;         // kg
    alignas(64) std::vector<Mat3d> inertias_;       // Body kg·m²
    alignas(64) std::vector<Vec3d> forces_;         // Accumulated N
    alignas(64) std::vector<Vec3d> torques_;        // Accumulated N·m
};
```

### 3.3 Property System (JSBSim-Inspired)

Hierarchical configuration system enabling runtime parameter access.

**Path Structure**: `<domain>/<entity_type>/<subsystem>/<parameter>`

**Examples**:

```
aircraft/f16/aero/cl_alpha     → Lift curve slope
aircraft/f16/propulsion/thrust → Current thrust (lbf)
vehicle/m1a2/terra/sinkage     → Terrain sinkage (m)
vessel/ddg51/hydro/draft       → Current draft (m)
satellite/gps01/orbit/altitude → Orbital altitude (km)
```

**Implementation**:

```cpp
class PropertyManager {
public:
    // Binding for direct memory access (high performance)
    void Bind(const std::string& path, Real* ptr);
    void Bind(const std::string& path, Vec3* ptr);

    // Dynamic access (slower, for scripting)
    Real GetReal(const std::string& path) const;
    void SetReal(const std::string& path, Real value);

    // Lookup optimization via hash indexing
    PropertyId GetId(const std::string& path);  // Pre-resolve
    Real GetById(PropertyId id) const;          // Fast access
};
```

### 3.4 Time Management

**Simulation Modes**:

| Mode | Description | Use Case |
|------|-------------|----------|
| Real-Time | 1s sim = 1s wall | Training, HiL |
| Accelerated | N×s sim = 1s wall | Analysis, Monte Carlo |
| Stepped | Manual advance | Debugging, Scripting |

**Implementation**:

```cpp
class TimeManager {
public:
    Real GetSimTime() const;        // Current simulation time
    Real GetDeltaTime() const;      // Last frame dt
    Real GetWallTime() const;       // Real-world time

    void SetTimeScale(Real scale);  // 1.0 = real-time
    void Pause();
    void Resume();

    // Frame timing control
    void SetFixedTimeStep(Real dt); // e.g., 0.01s (100Hz)
    void SetVariableTimeStep(Real min_dt, Real max_dt);
};
```

---

## 4. Physics Domain Specifications

### 4.1 Air Domain (Aerodynamics & Flight Dynamics)

**Reference**: JSBSim FGFDMExec architecture

#### 4.1.1 Force Generators

| Component | Model | Inputs | Outputs |
|-----------|-------|--------|---------|
| Aerodynamics | Table lookup | α, β, M, δ, q̄, ω | F_aero, M_aero |
| Propulsion | Thrust curves | throttle, M, alt | F_thrust |
| Ground Reaction | Contact model | gear state, terrain | F_ground |

#### 4.1.2 Aerodynamic Coefficient Tables

Multi-dimensional interpolation for:

- C_L(α, M, δ_e)  — Lift coefficient
- C_D(α, M, C_L)  — Drag coefficient (via drag polar)
- C_m(α, M, δ_e)  — Pitching moment
- C_l(β, p̄, δ_a, δ_r) — Rolling moment
- C_n(β, r̄, δ_a, δ_r) — Yawing moment
- C_Y(β, δ_r) — Side force

#### 4.1.3 6DOF Equations of Motion

**Translational** (ECEF frame):

```
m·a = F_aero + F_thrust + F_gravity + F_ground
```

**Rotational** (Body frame):

```
I·ω̇ = M - ω × (I·ω)
```

**Quaternion Integration** (Gimbal-lock free):

```
q̇ = 0.5 · q ⊗ ω_body
```

#### 4.1.4 Missile-Specific Features

- **Seeker Model**: Configurable FOV, track rate, noise
- **Guidance Laws**: PNG, APN, optimal (configurable via XML)
- **Motor Profiles**: Thrust(t) curves with burn-out detection
- **Fin Actuators**: Rate limits, deflection limits, lag

### 4.2 Land Domain (Terramechanics)

**Reference**: Bekker-Wong soft soil theory, NATO NRMM

#### 4.2.1 Pressure-Sinkage Relationship

```
p = (k_c/b + k_φ) · z^n
```

Where:

- p = ground pressure (Pa)
- z = sinkage (m)
- b = contact width (m)
- k_c = cohesive modulus (kN/m^(n+1))
- k_φ = frictional modulus (kN/m^(n+2))
- n = deformation exponent

#### 4.2.2 Motion Resistance

```
R_c = b·l · ∫₀^z p(z) dz
```

#### 4.2.3 Soil Property Database

| Soil Type | k_c | k_φ | n | c (kPa) | φ (°) |
|-----------|-----|-----|---|---------|-------|
| Dry Sand | 0.99 | 1528 | 1.10 | 1.04 | 28 |
| Wet Sand | 5.27 | 1515 | 0.73 | 1.72 | 29 |
| Clay | 13.19 | 692 | 0.50 | 4.14 | 13 |
| Snow | 4.37 | 196 | 1.60 | 1.03 | 19.7 |

#### 4.2.4 Tracked Vehicle Model

**Components**:

- Sprocket: Drive force application
- Road wheels: Load distribution
- Idler: Track tension maintenance
- Track: Continuous belt approximation

**Suspension Model**:

```cpp
struct SuspensionUnit {
    Real spring_k;      // N/m
    Real damper_c;      // N·s/m
    Real preload;       // N
    Real travel_max;    // m
    Real current_pos;   // m (state)
};
```

### 4.3 Sea Domain (Hydrodynamics)

**Reference**: ITTC standards, MMG maneuvering model

#### 4.3.1 Hydrostatic Forces

**Buoyancy**:

```
F_b = ρ_water · g · V_displaced
```

**Restoring Moment**:

```
M_restore = ρ·g·∇ · GM · sin(φ)
```

Where GM = metacentric height

#### 4.3.2 Wave Loading

**Wave Spectrum Options**:

- Pierson-Moskowitz (fully developed sea)
- JONSWAP (fetch-limited)
- Bretschneider (parameterized)

**Response Amplitude Operator (RAO)**:

```
η_response(ω, θ) = RAO(ω, θ) · η_wave(ω, θ)
```

#### 4.3.3 Maneuvering Model (MMG)

**Surge**: X = X_H + X_P + X_R
**Sway**: Y = Y_H + Y_R
**Yaw**: N = N_H + N_R

Where:

- _H = hull forces (velocity dependent)
- _P = propeller forces
- _R = rudder forces

### 4.4 Space Domain (Astrodynamics)

**Reference**: Vallado "Fundamentals of Astrodynamics"

#### 4.4.1 Orbital Propagation

**SGP4/SDP4** (Analytical):

- Input: TLE (Two-Line Element)
- Perturbations: J2-J4, drag, lunar/solar gravity
- Output: ECI position/velocity
- Use: Catalog tracking, constellation simulation

**Numerical Propagation** (High fidelity):

- Integrator: RK7(8) or Adams-Bashforth-Moulton
- Gravity: EGM96/EGM2008 (degree/order configurable)
- Perturbations: Atmospheric drag, SRP, 3rd body

#### 4.4.2 Coordinate Frames

| Frame | Origin | Use |
|-------|--------|-----|
| ECI J2000 | Earth center | Orbital mechanics |
| ECEF | Earth center | Geodetic reference |
| LVLH | Spacecraft | Relative motion |
| Body | Spacecraft | Attitude dynamics |

#### 4.4.3 Atmosphere Model (JBH08)

```
ρ = f(altitude, F10.7, ap, day_of_year, local_time)
```

Inputs:

- F10.7: Solar flux index
- ap: Geomagnetic activity index

---

## 5. Environment Subsystem

### 5.1 Terrain Digital Twin

#### 5.1.1 Data Sources

| Format | Description | Resolution |
|--------|-------------|------------|
| DTED Level 0 | 30 arc-sec (~900m) | Strategic |
| DTED Level 1 | 3 arc-sec (~90m) | Tactical |
| DTED Level 2 | 1 arc-sec (~30m) | Precision |
| GeoTIFF | Custom DEM | Variable |
| Shapefile | Vector features | N/A |

#### 5.1.2 Terrain Paging Architecture

```
┌─────────────────────────────────────────┐
│           TerrainManager                 │
│  ┌─────────────────────────────────┐    │
│  │        Quadtree Index           │    │
│  │  ┌─────┬─────┬─────┬─────┐     │    │
│  │  │ NW  │ NE  │ SW  │ SE  │     │    │
│  │  └──┬──┴──┬──┴──┬──┴──┬──┘     │    │
│  │     └─────┴──┬──┴─────┘         │    │
│  │              │                  │    │
│  │         [LOD Chain]             │    │
│  └─────────────────────────────────┘    │
│                                         │
│  ┌─────────────────────────────────┐    │
│  │       Async Tile Loader         │    │
│  │  • GDAL I/O thread pool         │    │
│  │  • Priority queue (distance)    │    │
│  │  • Memory budget enforcement    │    │
│  └─────────────────────────────────┘    │
│                                         │
│  ┌─────────────────────────────────┐    │
│  │      Material Mapper            │    │
│  │  • Soil type → Physics params   │    │
│  │  • Dynamic weather effects      │    │
│  └─────────────────────────────────┘    │
└─────────────────────────────────────────┘
```

#### 5.1.3 Terrain Query API

```cpp
class TerrainManager {
public:
    // Height queries
    Real GetElevation(Real lat, Real lon) const;
    Real GetElevationECEF(const Vec3& ecef) const;

    // Normal/slope queries
    Vec3 GetSurfaceNormal(Real lat, Real lon) const;
    Real GetSlopeAngle(Real lat, Real lon) const;

    // Material queries
    SoilType GetSoilType(Real lat, Real lon) const;
    TerrainMaterial GetMaterial(Real lat, Real lon) const;

    // LOD control
    void SetFocusPoint(const Vec3& ecef);
    void SetDetailRadius(Real radius_m);
};
```

### 5.2 Atmosphere Model

#### 5.2.1 Standard Atmosphere (US Std 1976)

| Altitude (km) | Temperature (K) | Pressure (Pa) | Density (kg/m³) |
|---------------|-----------------|---------------|-----------------|
| 0 | 288.15 | 101325 | 1.225 |
| 11 | 216.65 | 22632 | 0.3639 |
| 20 | 216.65 | 5474.9 | 0.0880 |
| 32 | 228.65 | 868.02 | 0.0132 |

#### 5.2.2 Atmospheric Effects

| Effect | Model | Application |
|--------|-------|-------------|
| Refraction | ITU-R P.835 | Radar, optics |
| Rain attenuation | ITU-R P.838 | RF propagation |
| Fog/dust | MODTRAN LUT | EO/IR sensors |
| Wind | MIL-STD-210C profiles | Flight dynamics |

### 5.3 Ocean Model

#### 5.3.1 Sea State Table (NATO)

| Sea State | H_s (m) | T_p (s) | Description |
|-----------|---------|---------|-------------|
| 0 | 0 | - | Calm (glassy) |
| 1 | 0-0.1 | - | Calm (rippled) |
| 2 | 0.1-0.5 | 3.3 | Smooth |
| 3 | 0.5-1.25 | 5.0 | Slight |
| 4 | 1.25-2.5 | 6.1 | Moderate |
| 5 | 2.5-4.0 | 7.7 | Rough |
| 6 | 4.0-6.0 | 10.0 | Very rough |
| 7 | 6.0-9.0 | 12.5 | High |
| 8 | 9.0-14.0 | 15.0 | Very high |

---

## 6. Interaction Systems

### 6.1 Collision Detection

#### 6.1.1 Two-Phase Architecture

**Broad Phase** (O(n log n)):

- Algorithm: Dynamic AABB Tree or Sweep & Prune
- Output: Potentially colliding pairs

**Narrow Phase** (Per pair):

- Algorithm: GJK + EPA for convex hulls
- Output: Contact points, penetration depth, normal

#### 6.1.2 Collision Groups

| Group | Collides With |
|-------|---------------|
| Aircraft | Terrain, Missiles, Projectiles |
| Vehicle | Terrain, Vehicles, Projectiles |
| Vessel | Ocean floor, Vessels, Torpedoes |
| Missile | All |
| Projectile | Aircraft, Vehicle, Vessel |

### 6.2 Damage Model

#### 6.2.1 Weapon Effects

| Weapon Type | Damage Model |
|-------------|--------------|
| Kinetic | Penetration (RHA equivalent) |
| Blast | Overpressure + Impulse |
| Fragment | Pk = f(fragment density, area) |
| EMP | Probability of electronic kill |

#### 6.2.2 System Degradation

```cpp
enum class DamageState {
    Operational,    // 100% capability
    Degraded,       // Reduced performance
    Disabled,       // Non-functional
    Destroyed       // Entity removed
};

struct DamageableSystem {
    std::string name;
    Real health;           // 0.0 - 1.0
    Real vulnerability;    // Pk per hit
    DamageState state;
    std::function<void(Real)> degrade_callback;
};
```

---

## 7. Interface Layer

### 7.1 C++ Native API

```cpp
namespace jaguar {
    // Engine lifecycle
    Engine* CreateEngine();
    void DestroyEngine(Engine* engine);

    // Entity management
    EntityId CreateEntity(Engine* engine, const char* xml_path);
    void DestroyEntity(Engine* engine, EntityId id);

    // Simulation control
    void Initialize(Engine* engine, const char* config);
    void Step(Engine* engine, Real dt);
    void Run(Engine* engine);
    void Pause(Engine* engine);
    void Shutdown(Engine* engine);

    // State access
    void GetEntityState(Engine* engine, EntityId id, EntityState* out);
    void SetEntityState(Engine* engine, EntityId id, const EntityState* state);

    // Property access
    Real GetProperty(Engine* engine, const char* path);
    void SetProperty(Engine* engine, const char* path, Real value);
}
```

### 7.2 Network Interface (DIS/HLA)

#### 7.2.1 DIS Protocol (IEEE 1278)

**Supported PDUs**:

| PDU Type | Direction | Purpose |
|----------|-----------|---------|
| Entity State | TX/RX | Position, velocity, orientation |
| Fire | TX/RX | Weapon discharge |
| Detonation | TX/RX | Impact/explosion |
| Collision | TX | Physical contact |
| Start/Resume | RX | Simulation control |
| Stop/Freeze | RX | Simulation control |

#### 7.2.2 HLA Interface (IEEE 1516)

**Object Classes**:

- BaseEntity.PhysicalEntity
  - Platform.Aircraft
  - Platform.GroundVehicle
  - Platform.SurfaceVessel
  - Platform.Spacecraft
  - Munition

**Interaction Classes**:

- WeaponFire
- MunitionDetonation
- RadioSignal

### 7.3 Scripting Interface

#### 7.3.1 Python Binding (pybind11)

```python
import pyjaguar as jag

# Create engine
engine = jag.Engine()
engine.initialize("config.xml")

# Create entities
f16 = engine.create_entity("aircraft/f16.xml")
sam = engine.create_entity("missile/sam.xml")

# Run simulation
while engine.get_time() < 300.0:
    engine.step(0.01)

    # Access properties
    alt = engine.get_property(f16, "position/altitude_ft")
    print(f"F-16 altitude: {alt:.0f} ft")

engine.shutdown()
```

#### 7.3.2 Lua Binding (sol2)

```lua
local engine = jaguar.Engine.new()
engine:initialize("config.xml")

local aircraft = engine:create_entity("aircraft/f16.xml")

-- Main loop
while engine:get_time() < 300.0 do
    engine:step(0.01)

    local state = engine:get_state(aircraft)
    print(string.format("Position: %.1f, %.1f, %.1f",
        state.position.x, state.position.y, state.position.z))
end

engine:shutdown()
```

---

## 8. Configuration System

### 8.1 Engine Configuration (XML)

```xml
<?xml version="1.0" encoding="UTF-8"?>
<jaguar_config version="1.0">
    <simulation>
        <time_step>0.01</time_step>
        <max_entities>10000</max_entities>
        <coordinate_frame>ECEF</coordinate_frame>
    </simulation>

    <threading>
        <physics_threads>auto</physics_threads>
        <io_threads>2</io_threads>
        <work_stealing>true</work_stealing>
    </threading>

    <terrain>
        <provider>gdal</provider>
        <cache_size_mb>2048</cache_size_mb>
        <tile_size>256</tile_size>
        <data_paths>
            <path>/data/terrain/dted</path>
        </data_paths>
    </terrain>

    <atmosphere>
        <model>us_standard_1976</model>
        <enable_weather>true</enable_weather>
    </atmosphere>

    <network>
        <dis>
            <enabled>true</enabled>
            <port>3000</port>
            <site_id>1</site_id>
            <app_id>1</app_id>
        </dis>
    </network>
</jaguar_config>
```

### 8.2 Entity Definition (XML)

Example: F-16 Aircraft Definition

```xml
<?xml version="1.0" encoding="UTF-8"?>
<entity type="aircraft" name="F-16C">
    <metrics>
        <wingspan unit="ft">32.8</wingspan>
        <length unit="ft">49.5</length>
        <height unit="ft">16.7</height>
        <wing_area unit="ft2">300.0</wing_area>
    </metrics>

    <mass_balance>
        <empty_weight unit="lbs">19700</empty_weight>
        <fuel_capacity unit="lbs">6972</fuel_capacity>
        <cg_location unit="in">
            <x>180.0</x><y>0.0</y><z>0.0</z>
        </cg_location>
        <inertia unit="slug*ft2">
            <ixx>9496</ixx>
            <iyy>55814</iyy>
            <izz>63100</izz>
            <ixz>982</ixz>
        </inertia>
    </mass_balance>

    <aerodynamics>
        <axis name="LIFT">
            <function name="aero/cl">
                <table>
                    <independentVar>aero/alpha_rad</independentVar>
                    <tableData>
                        -0.20  -0.68
                        0.00   0.20
                        0.24   1.20
                        0.60   0.70
                    </tableData>
                </table>
            </function>
        </axis>
        <!-- Additional axes: DRAG, SIDE, ROLL, PITCH, YAW -->
    </aerodynamics>

    <propulsion>
        <engine type="turbofan" name="F110-GE-129">
            <max_thrust unit="lbf">29000</max_thrust>
            <afterburner_thrust unit="lbf">29000</afterburner_thrust>
            <!-- Thrust tables by Mach, altitude -->
        </engine>
    </propulsion>

    <flight_control>
        <channel name="pitch">
            <summer>
                <input>fcs/pitch-cmd</input>
                <input>fcs/pitch-trim</input>
                <clipto><min>-1.0</min><max>1.0</max></clipto>
            </summer>
            <aerosurface_scale>
                <input>fcs/pitch-sum</input>
                <range><min>-25</min><max>25</max></range>
                <output>fcs/elevator-deg</output>
            </aerosurface_scale>
        </channel>
    </flight_control>
</entity>
```

---

## 9. Performance Optimization

### 9.1 Memory Layout (SoA)

**Traditional OOP (AoS)**:

```cpp
// Array of Structures - poor cache utilization
struct Entity { Vec3 pos; Vec3 vel; Quat ori; Real mass; };
std::vector<Entity> entities;  // Data scattered
```

**Data-Oriented (SoA)**:

```cpp
// Structure of Arrays - excellent cache utilization
struct EntityStorage {
    std::vector<Vec3> positions;    // Contiguous
    std::vector<Vec3> velocities;   // Contiguous
    std::vector<Quat> orientations; // Contiguous
    std::vector<Real> masses;       // Contiguous
};
```

**Cache Performance**:

| Pattern | Cache Miss Rate | Relative Speed |
|---------|-----------------|----------------|
| AoS (random) | ~80% | 1.0x |
| AoS (sequential) | ~40% | 2.5x |
| SoA | ~5% | 8-15x |

### 9.2 SIMD Vectorization

```cpp
// Process 4 entities per SIMD instruction (AVX2)
void UpdatePositions_AVX2(
    float* __restrict positions,    // x0,y0,z0,x1,y1,z1,...
    const float* __restrict velocities,
    float dt,
    size_t count)
{
    __m256 dt_vec = _mm256_set1_ps(dt);

    for (size_t i = 0; i < count; i += 8) {
        __m256 pos = _mm256_load_ps(&positions[i]);
        __m256 vel = _mm256_load_ps(&velocities[i]);
        __m256 new_pos = _mm256_fmadd_ps(vel, dt_vec, pos);
        _mm256_store_ps(&positions[i], new_pos);
    }
}
```

### 9.3 Threading Model

```
┌─────────────────────────────────────────────────────────────┐
│                    Main Thread                               │
│  • Input processing                                         │
│  • Script execution                                         │
│  • Network I/O dispatch                                     │
│  • Frame synchronization                                    │
└───────────────────────────┬─────────────────────────────────┘
                            │ Job dispatch
┌───────────────────────────┴─────────────────────────────────┐
│                 Physics Thread Pool                          │
│  ┌─────────┐ ┌─────────┐ ┌─────────┐ ┌─────────┐           │
│  │Worker 0 │ │Worker 1 │ │Worker 2 │ │Worker N │           │
│  │ ┌─────┐ │ │ ┌─────┐ │ │ ┌─────┐ │ │ ┌─────┐ │           │
│  │ │Job Q│ │ │ │Job Q│ │ │ │Job Q│ │ │ │Job Q│ │           │
│  │ └──┬──┘ │ │ └──┬──┘ │ │ └──┬──┘ │ │ └──┬──┘ │           │
│  │    │    │ │    │    │ │    │    │ │    │    │           │
│  │   ←Work Stealing→   │ │   ←Work Stealing→   │           │
│  └─────────┘ └─────────┘ └─────────┘ └─────────┘           │
└─────────────────────────────────────────────────────────────┘
                            │
┌───────────────────────────┴─────────────────────────────────┐
│                 I/O Thread Pool                              │
│  • Terrain tile loading (GDAL)                              │
│  • Network PDU send/receive                                 │
│  • File I/O operations                                      │
└─────────────────────────────────────────────────────────────┘
```

---

## 10. Verification & Validation

### 10.1 Test Categories

| Category | Purpose | Framework |
|----------|---------|-----------|
| Unit | Component isolation | Google Test |
| Integration | Subsystem interaction | Google Test |
| Benchmark | Performance regression | Google Benchmark |
| Validation | Physics accuracy | NASA/AIAA cases |

### 10.2 Domain Validation References

| Domain | Benchmark | Source |
|--------|-----------|--------|
| Air | Standard Check-Cases | NASA TN-20160006944 |
| Land | NRMM Validation | NATO ET-148 |
| Land | TOP 02-2-602A | US Army |
| Sea | ITTC Standard Hull | ITTC |
| Space | SGP4 Test Cases | Space-Track.org |

### 10.3 Continuous Integration

```yaml
# .github/workflows/ci.yml
stages:
  - build
  - test
  - benchmark
  - docs

build:
  matrix:
    os: [ubuntu-22.04, windows-2022, macos-13]
    compiler: [gcc-12, clang-15, msvc-2022]
    config: [Debug, Release]
  steps:
    - cmake --preset ${{ matrix.config }}
    - cmake --build --preset ${{ matrix.config }}

test:
  steps:
    - ctest --preset unit
    - ctest --preset integration

benchmark:
  steps:
    - ./build/benchmarks/jaguar_bench --benchmark_format=json
    - compare with baseline (fail on >5% regression)
```

---

## 11. Development Roadmap

### Phase 1: Foundation (Core Engine) ✅ COMPLETE

**Objectives**:

- Establish C++ project structure with CMake
- Implement core memory management (pools, SoA)
- Create Property System (JSBSim-compatible)
- Implement 6DOF rigid body dynamics
- Basic time management and simulation loop

**Deliverables**:

- [x] CMakeLists.txt with dependency management
- [x] Core math library (Vec3, Quat, Mat3x3)
- [x] Entity storage with SoA layout
- [x] Property manager with hash indexing
- [x] RK4 integrator for state propagation
- [x] Unit test framework setup

### Phase 2: Domain Expansion (Air + Land) ✅ COMPLETE

**Objectives**:

- Integrate GDAL for terrain data
- Implement aerodynamics model (JSBSim-style tables)
- Implement Bekker-Wong terramechanics
- Add XML entity configuration parsing

**Deliverables**:

- [x] Terrain manager with quadtree paging
- [x] Aerodynamic coefficient interpolation
- [x] Propulsion model (thrust curves)
- [x] Terramechanics force generator
- [x] Suspension dynamics model
- [x] XML parser for entity definitions

### Phase 3: Full Integration (Sea + Space) ✅ COMPLETE

**Objectives**:

- Implement hydrodynamic forces
- Add wave spectrum generation
- Implement SGP4 orbital propagator
- Add atmospheric models (US Std, JB08)

**Deliverables**:

- [x] Buoyancy calculator with metacentric height
- [x] RAO-based ship motion model (6DOF response)
- [x] SGP4/SDP4 implementation
- [x] Gravity model (J2-J4)
- [x] Standard atmosphere implementation
- [x] Ocean wave spectrum generator (PM, JONSWAP)
- [x] JB08 space atmosphere model
- [x] Atmospheric drag model
- [x] ECI↔LVLH coordinate transforms

### Phase 4: Commercialization ✅ COMPLETE

**Objectives**:

- Implement DIS/HLA network interfaces
- Add Python/Lua scripting bindings
- Complete V&V test suite
- Performance optimization pass
- SDK packaging and documentation

**Deliverables**:

- [x] DIS PDU encoding/decoding (EntityState, Fire, Detonation)
- [x] pybind11 Python module (pyjaguar)
- [x] sol2 Lua bindings
- [x] NASA check-case validation (SGP4, Aerodynamics)
- [x] NRMM/ITTC validation results
- [x] SDK installer and documentation (CPack)
- [x] Doxygen API documentation

### Phase 5: Advanced Features ✅ COMPLETE

**Objectives**:

- Implement HLA federate interface for full LVC integration
- Add collision detection and response system
- Implement damage modeling system
- Enhance flight control systems (autopilot, FCS)
- Create CI/CD pipeline for automated testing

**Deliverables**:

- [x] HLA federate implementation (IEEE 1516-2010 with RPR FOM 2.0)
- [x] Collision detection (AABB broadphase + GJK/EPA narrowphase)
- [x] Damage model (kinetic, blast, fragment with JTCG/ME methodology)
- [x] Flight control system (SAS, CAS, autopilot channels)
- [x] Missile guidance laws (PNG, APN)
- [x] GitHub Actions CI/CD pipeline (multi-platform, security scanning)

### Phase 6: Enterprise & Integration ✅ COMPLETE

**Objectives**:

- Add CIG integration interface (Unreal/Unity)
- Implement distributed simulation orchestration
- Cloud deployment support (AWS, Azure)
- Enhanced terrain streaming for large-scale simulations
- Real-time performance monitoring and analytics

**Deliverables**:

- [x] Unreal Engine 5 plugin (`plugins/unreal/JaguarEngine/`)
- [x] Unity integration package (`plugins/unity/JaguarEngine/`)
- [x] Kubernetes deployment configurations (`deploy/kubernetes/`)
- [x] Cloud-native terrain streaming service
- [x] Performance telemetry dashboard (Prometheus + Grafana)
- [x] Multi-federation orchestration tools (`federation_orchestrator`)

### Phase 7: Advanced Capabilities (Planned)

**See**: [PHASE7_ROADMAP.md](./PHASE7_ROADMAP.md) for detailed specifications.

**Sub-Phases**:

| Phase | Name | Version | Status |
|-------|------|---------|--------|
| 7A | GPU Acceleration | v1.6.0 | Planned |
| 7B | Extended Reality | v1.7.0 | Planned |
| 7C | Cloud Burst | v1.8.0 | Planned |
| 7D | Digital Thread | v1.9.0 | Planned |
| 7E | Machine Learning | v2.0.0 | Planned |

**Key Features**:

- **GPU Acceleration**: CUDA/OpenCL physics computation for 10x performance
- **Extended Reality**: OpenXR integration for VR/AR training applications
- **Cloud Burst**: Dynamic scaling for 100,000+ entity exercises
- **Digital Thread**: Entity lifecycle management and predictive maintenance
- **Machine Learning**: ONNX-based neural autopilot and RL environments

---

## 12. Technology Stack

### 12.1 Core Dependencies

| Component | Library | Version | Purpose |
|-----------|---------|---------|---------|
| Build | CMake | 3.25+ | Cross-platform build |
| Compiler | C++20 | - | Language standard |
| Math | Eigen | 3.4+ | Linear algebra |
| GIS | GDAL | 3.6+ | Geospatial data |
| XML | pugixml | 1.13+ | Configuration parsing |
| JSON | nlohmann_json | 3.11+ | Data interchange |
| Testing | GoogleTest | 1.14+ | Unit testing |
| Benchmark | GoogleBenchmark | 1.8+ | Performance testing |

### 12.2 Optional Dependencies

| Component | Library | Version | Purpose |
|-----------|---------|---------|---------|
| Python | pybind11 | 2.11+ | Python bindings |
| Lua | sol2 | 3.3+ | Lua bindings |
| Network | OpenDIS | 7.1+ | DIS protocol |
| Network | Portico | 2.1+ | HLA RTI |
| Threading | TBB | 2021+ | Parallel algorithms |
| SIMD | xsimd | 11+ | Portable SIMD |

---

## Appendix A: Glossary

| Term | Definition |
|------|------------|
| 6DOF | Six Degrees of Freedom (x, y, z, roll, pitch, yaw) |
| AABB | Axis-Aligned Bounding Box |
| CIG | Computer Image Generator |
| DIS | Distributed Interactive Simulation |
| DOD | Data-Oriented Design |
| ECEF | Earth-Centered, Earth-Fixed coordinate frame |
| ECI | Earth-Centered Inertial coordinate frame |
| FDM | Flight Dynamics Model |
| GIS | Geographic Information System |
| HLA | High Level Architecture |
| LOD | Level of Detail |
| LVC | Live-Virtual-Constructive |
| MBS | Multi-Body System |
| NED | North-East-Down local coordinate frame |
| RAO | Response Amplitude Operator |
| RTI | Runtime Infrastructure (HLA) |
| SGP4 | Simplified General Perturbations (orbital model) |
| SIMD | Single Instruction Multiple Data |
| SoA | Structure of Arrays (memory layout) |
| TLE | Two-Line Element (orbital data format) |

---

## Appendix B: References

1. JSBSim Reference Manual, <https://jsbsim-team.github.io/jsbsim-reference-manual/>
2. NASA TN-20160006944, "Standard Check-cases for Six-DOF Flight Vehicle Simulations"
3. NATO ET-148, "Next-Generation NATO Reference Mobility Model"
4. US Army TOP 02-2-602A, "Acceleration; Maximum and Minimum Speeds"
5. ITU-R P.838, "Specific attenuation model for rain"
6. Vallado, D.A., "Fundamentals of Astrodynamics and Applications"
7. Bekker, M.G., "Introduction to Terrain-Vehicle Systems"
8. ITTC Recommended Procedures, "Seakeeping Experiments"
