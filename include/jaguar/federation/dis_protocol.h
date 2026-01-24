#pragma once
/**
 * @file dis_protocol.h
 * @brief IEEE 1278.1 Distributed Interactive Simulation (DIS) protocol implementation
 *
 * This file implements the DIS protocol for real-time distributed military simulation.
 * DIS enables interoperability between heterogeneous simulations, live instrumented
 * ranges, and operational systems through standardized Protocol Data Units (PDUs).
 *
 * Key features:
 * - Complete PDU type definitions (Entity State, Fire, Detonation, etc.)
 * - Dead reckoning algorithms for network bandwidth reduction
 * - Geocentric coordinate transformations (ECEF <-> LLA)
 * - Entity type enumeration system (SISO-REF-010)
 * - Big-endian encoding/decoding for network transmission
 * - Time synchronization for distributed exercises
 *
 * Standards compliance:
 * - IEEE 1278.1-2012 (DIS version 7)
 * - SISO-REF-010 (Entity Type enumerations)
 * - SISO-REF-015 (Dead Reckoning)
 *
 * @see https://www.sisostandards.org/
 */

#include "jaguar/core/types.h"
#include <vector>
#include <array>
#include <string>
#include <memory>
#include <optional>
#include <cstring>
#include <cstdint>

namespace jaguar::federation {

// ============================================================================
// DIS Protocol Constants
// ============================================================================

/// DIS protocol version (version 7)
constexpr UInt8 DIS_VERSION = 7;

/// Exercise identifier (0 = all exercises)
constexpr UInt8 DIS_EXERCISE_ID_ALL = 0;

/// Maximum PDU size in bytes (8KB)
constexpr SizeT DIS_MAX_PDU_SIZE = 8192;

/// Maximum marking (entity name) length
constexpr SizeT DIS_MARKING_LENGTH = 11;

/// Maximum articulation parameters per entity
constexpr SizeT DIS_MAX_ARTICULATION_PARAMS = 16;

/// DIS heartbeat interval (seconds) - typical value
constexpr Real DIS_HEARTBEAT_INTERVAL = 5.0;

/// Dead reckoning threshold (meters) - typical value
constexpr Real DIS_DR_THRESHOLD_POSITION = 1.0;

/// Dead reckoning threshold (degrees) - typical value
constexpr Real DIS_DR_THRESHOLD_ORIENTATION = 3.0;

// ============================================================================
// PDU Type Enumeration
// ============================================================================

/**
 * @brief DIS Protocol Data Unit (PDU) types
 *
 * These define the message types that can be exchanged in a DIS exercise.
 * Each PDU type has a specific structure and purpose.
 */
enum class PDUType : UInt8 {
    // Entity Information/Interaction Family
    EntityState = 1,           ///< Entity state update (position, velocity, etc.)
    Fire = 2,                  ///< Weapon fire event
    Detonation = 3,            ///< Munition detonation or impact
    Collision = 4,             ///< Entity collision event
    ServiceRequest = 5,        ///< Request for services (repair, resupply)
    ResupplyOffer = 6,         ///< Offer to provide resupply
    ResupplyReceived = 7,      ///< Acknowledgment of resupply
    ResupplyCancel = 8,        ///< Cancel resupply operation
    RepairComplete = 9,        ///< Repair operation completed
    RepairResponse = 10,       ///< Response to repair request

    // Simulation Management Family
    CreateEntity = 11,         ///< Create new entity in simulation
    RemoveEntity = 12,         ///< Remove entity from simulation
    StartResume = 13,          ///< Start or resume simulation
    StopFreeze = 14,           ///< Stop or freeze simulation
    Acknowledge = 15,          ///< Acknowledge receipt of PDU
    ActionRequest = 16,        ///< Request action from entity
    ActionResponse = 17,       ///< Response to action request
    DataQuery = 18,            ///< Query for data
    SetData = 19,              ///< Set data values
    Data = 20,                 ///< Data response
    EventReport = 21,          ///< Report simulation event
    Comment = 22,              ///< Free-form comment/message

    // Distributed Emission Regeneration Family
    ElectromagneticEmission = 23,  ///< Electromagnetic emission (radar, etc.)
    Designator = 24,           ///< Laser designator
    Transmitter = 25,          ///< Radio transmitter
    Signal = 26,               ///< Signal (waveform data)
    Receiver = 27,             ///< Radio receiver

    // Radio Communications Family
    Transmitter_RadioComms = 28,   ///< Radio transmitter (voice/data)
    Signal_RadioComms = 29,        ///< Radio signal (voice/data)
    Receiver_RadioComms = 30,      ///< Radio receiver (voice/data)

    // Entity Management Family
    IsGroupOf = 48,            ///< Entity is group of entities
    TransferOwnership = 49,    ///< Transfer entity ownership
    IsPartOf = 50,             ///< Entity is part of another

    // Minefield Family
    Minefield_State = 51,      ///< Minefield state
    Minefield_Query = 52,      ///< Query minefield
    Minefield_Data = 53,       ///< Minefield data
    Minefield_ResponseNACK = 54, ///< Minefield negative acknowledgment

    // Synthetic Environment Family
    EnvironmentalProcess = 67, ///< Environmental process (weather, etc.)
    GriddedData = 68,          ///< Gridded environmental data
    PointObjectState = 69,     ///< Point object state
    LinearObjectState = 70,    ///< Linear object state
    ArealObjectState = 71,     ///< Areal object state

    // Simulation Management with Reliability Family
    CreateEntity_R = 72,       ///< Create entity (reliable)
    RemoveEntity_R = 73,       ///< Remove entity (reliable)
    StartResume_R = 74,        ///< Start/resume (reliable)
    StopFreeze_R = 75,         ///< Stop/freeze (reliable)
    Acknowledge_R = 76,        ///< Acknowledge (reliable)
    ActionRequest_R = 77,      ///< Action request (reliable)
    ActionResponse_R = 78,     ///< Action response (reliable)
    DataQuery_R = 79,          ///< Data query (reliable)
    SetData_R = 80,            ///< Set data (reliable)
    Data_R = 81,               ///< Data (reliable)
    EventReport_R = 82,        ///< Event report (reliable)
    Comment_R = 83,            ///< Comment (reliable)
    RecordQuery_R = 84,        ///< Record query (reliable)
    SetRecord_R = 85,          ///< Set record (reliable)
    Record_R = 86,             ///< Record (reliable)

    // Information Operations Family
    IOAction = 129,            ///< Information operations action
    IOReport = 130,            ///< Information operations report

    // Extended Entity Information Family
    Attribute = 131,           ///< Entity attribute
    Announce = 132,            ///< Announcement

    // Unknown/Custom
    Unknown = 255              ///< Unknown or custom PDU type
};

/**
 * @brief Convert PDU type to string
 */
inline const char* pdu_type_to_string(PDUType type) {
    switch (type) {
        case PDUType::EntityState: return "EntityState";
        case PDUType::Fire: return "Fire";
        case PDUType::Detonation: return "Detonation";
        case PDUType::Collision: return "Collision";
        case PDUType::ServiceRequest: return "ServiceRequest";
        case PDUType::ResupplyOffer: return "ResupplyOffer";
        case PDUType::ResupplyReceived: return "ResupplyReceived";
        case PDUType::RepairComplete: return "RepairComplete";
        case PDUType::CreateEntity: return "CreateEntity";
        case PDUType::RemoveEntity: return "RemoveEntity";
        case PDUType::StartResume: return "StartResume";
        case PDUType::StopFreeze: return "StopFreeze";
        case PDUType::Acknowledge: return "Acknowledge";
        case PDUType::ElectromagneticEmission: return "ElectromagneticEmission";
        case PDUType::Designator: return "Designator";
        case PDUType::Transmitter: return "Transmitter";
        case PDUType::Signal: return "Signal";
        case PDUType::Receiver: return "Receiver";
        case PDUType::IsGroupOf: return "IsGroupOf";
        case PDUType::TransferOwnership: return "TransferOwnership";
        case PDUType::IsPartOf: return "IsPartOf";
        case PDUType::EnvironmentalProcess: return "EnvironmentalProcess";
        case PDUType::Attribute: return "Attribute";
        default: return "Unknown";
    }
}

// ============================================================================
// Dead Reckoning Algorithm Enumeration
// ============================================================================

/**
 * @brief Dead reckoning algorithm identifiers
 *
 * Dead reckoning extrapolates entity position between state updates
 * to reduce network bandwidth. Different algorithms use different
 * orders of derivative information.
 */
enum class DeadReckoningAlgorithm : UInt8 {
    /// No dead reckoning - static entity
    DRM_Static = 0,

    /// Dead Reckoning Model - Fixed, Position, World coordinates
    /// Uses position only, assumes constant position
    DRM_FPW = 1,

    /// Dead Reckoning Model - Rotating, Position, World coordinates
    /// Uses position and orientation, constant angular velocity
    DRM_RPW = 2,

    /// Dead Reckoning Model - Rotating, Velocity, World coordinates
    /// Uses position, velocity, and angular velocity
    DRM_RVW = 3,

    /// Dead Reckoning Model - Fixed, Velocity, World coordinates
    /// Uses position and velocity (no rotation)
    DRM_FVW = 4,

    /// Dead Reckoning Model - Fixed, Position, Body coordinates
    /// Position in body frame
    DRM_FPB = 5,

    /// Dead Reckoning Model - Rotating, Position, Body coordinates
    /// Position and orientation in body frame
    DRM_RPB = 6,

    /// Dead Reckoning Model - Rotating, Velocity, Body coordinates
    /// Full 6DOF in body frame
    DRM_RVB = 7,

    /// Dead Reckoning Model - Fixed, Velocity, Body coordinates
    /// Velocity in body frame (no rotation)
    DRM_FVB = 8,

    /// Dead Reckoning Model - Rotating, Acceleration, World coordinates
    /// Uses acceleration for higher accuracy
    DRM_RVW_HighRes = 9
};

/**
 * @brief Convert dead reckoning algorithm to string
 */
inline const char* dr_algorithm_to_string(DeadReckoningAlgorithm alg) {
    switch (alg) {
        case DeadReckoningAlgorithm::DRM_Static: return "Static";
        case DeadReckoningAlgorithm::DRM_FPW: return "FPW";
        case DeadReckoningAlgorithm::DRM_RPW: return "RPW";
        case DeadReckoningAlgorithm::DRM_RVW: return "RVW";
        case DeadReckoningAlgorithm::DRM_FVW: return "FVW";
        case DeadReckoningAlgorithm::DRM_FPB: return "FPB";
        case DeadReckoningAlgorithm::DRM_RPB: return "RPB";
        case DeadReckoningAlgorithm::DRM_RVB: return "RVB";
        case DeadReckoningAlgorithm::DRM_FVB: return "FVB";
        case DeadReckoningAlgorithm::DRM_RVW_HighRes: return "RVW_HighRes";
        default: return "Unknown";
    }
}

// ============================================================================
// Force ID Enumeration
// ============================================================================

/**
 * @brief Force/Side identification
 */
enum class ForceId : UInt8 {
    Other = 0,
    Friendly = 1,
    Opposing = 2,
    Neutral = 3,
    Friendly2 = 4,  // Coalition partner
    Opposing2 = 5,  // Hostile 2
    Neutral2 = 6,
    Friendly3 = 7,
    Opposing3 = 8,
    Neutral3 = 9,
    Friendly4 = 10,
    Opposing4 = 11,
    Neutral4 = 12
};

// ============================================================================
// Entity Kind Enumeration (SISO-REF-010)
// ============================================================================

/**
 * @brief Entity kind (top-level classification)
 */
enum class EntityKind : UInt8 {
    Other = 0,
    Platform = 1,           // Aircraft, tanks, ships, etc.
    Munition = 2,           // Missiles, bombs, bullets
    Lifeform = 3,           // Personnel
    Environmental = 4,      // Weather, obstacles
    CulturalFeature = 5,    // Buildings, bridges
    Supply = 6,             // Supplies, cargo
    Radio = 7,              // Radio equipment
    Expendable = 8,         // Chaff, flares
    SensorEmitter = 9       // Radar, sensors
};

/**
 * @brief Platform domain (for EntityKind::Platform)
 */
enum class PlatformDomain : UInt8 {
    Other = 0,
    Land = 1,
    Air = 2,
    Surface = 3,        // Surface ship
    Subsurface = 4,     // Submarine
    Space = 5
};

// ============================================================================
// Detonation Result
// ============================================================================

/**
 * @brief Result of munition detonation
 */
enum class DetonationResult : UInt8 {
    Other = 0,
    EntityImpact = 1,           // Direct hit on entity
    EntityProximateDetonation = 2,  // Near-miss
    GroundImpact = 3,
    GroundProximateDetonation = 4,
    Detonation = 5,             // In-flight detonation
    None = 6,                   // No detonation (dud)
    HE_Hit_Small = 7,           // High explosive hit (small)
    HE_Hit_Medium = 8,
    HE_Hit_Large = 9,
    ArmorPiercingHit = 10,
    DirtBlastSmall = 11,
    DirtBlastMedium = 12,
    DirtBlastLarge = 13,
    WaterBlastSmall = 14,
    WaterBlastMedium = 15,
    WaterBlastLarge = 16,
    AirHit = 17,
    BuildingHitSmall = 18,
    BuildingHitMedium = 19,
    BuildingHitLarge = 20,
    MineClearingLineCharge = 21,
    EnvironmentObjectImpact = 22,
    EnvironmentObjectProximateDetonation = 23,
    WaterImpact = 24,
    AirBurst = 25,
    KillWithFragmentType1 = 26,
    KillWithFragmentType2 = 27,
    KillWithFragmentType3 = 28,
    KillWithoutFragment = 29,
    Miss = 30
};

// ============================================================================
// DIS Entity Identifier
// ============================================================================

/**
 * @brief Unique identifier for an entity in a DIS exercise
 *
 * Composed of site ID, application ID, and entity number.
 * Together these form a globally unique identifier.
 */
struct EntityIdentifier {
    UInt16 site{0};             ///< Site identifier (installation/facility)
    UInt16 application{0};      ///< Application ID (specific simulation)
    UInt16 entity{0};           ///< Entity number within application

    constexpr EntityIdentifier() noexcept = default;
    constexpr EntityIdentifier(UInt16 s, UInt16 a, UInt16 e) noexcept
        : site(s), application(a), entity(e) {}

    /// Check if valid
    constexpr bool is_valid() const noexcept {
        return site != 0 || application != 0 || entity != 0;
    }

    /// Equality comparison
    constexpr bool operator==(const EntityIdentifier& other) const noexcept {
        return site == other.site && application == other.application && entity == other.entity;
    }

    constexpr bool operator!=(const EntityIdentifier& other) const noexcept {
        return !(*this == other);
    }

    /// Hash for use in maps
    struct Hash {
        std::size_t operator()(const EntityIdentifier& id) const noexcept {
            return (std::size_t(id.site) << 32) |
                   (std::size_t(id.application) << 16) |
                   std::size_t(id.entity);
        }
    };
};

/// Invalid entity identifier constant
constexpr EntityIdentifier INVALID_ENTITY_IDENTIFIER{0, 0, 0};

// ============================================================================
// Event Identifier
// ============================================================================

/**
 * @brief Identifier for simulation events (fire, detonation, etc.)
 */
struct EventIdentifier {
    UInt16 site{0};             ///< Site identifier
    UInt16 application{0};      ///< Application identifier
    UInt16 event_number{0};     ///< Event number (unique per application)

    constexpr EventIdentifier() noexcept = default;
    constexpr EventIdentifier(UInt16 s, UInt16 a, UInt16 e) noexcept
        : site(s), application(a), event_number(e) {}

    constexpr bool operator==(const EventIdentifier& other) const noexcept {
        return site == other.site &&
               application == other.application &&
               event_number == other.event_number;
    }
};

// ============================================================================
// Entity Type
// ============================================================================

/**
 * @brief Hierarchical entity type classification (SISO-REF-010)
 *
 * Describes what kind of entity this is using a 7-level hierarchy:
 * Kind -> Domain -> Country -> Category -> Subcategory -> Specific -> Extra
 *
 * Examples:
 * - F-16C: Platform, Air, USA, Fighter/Air Defense, F-16, F-16C
 * - M1A2 Abrams: Platform, Land, USA, Tank, M1 Abrams, M1A2
 * - AIM-120: Munition, Air, USA, Guided, AAM, AIM-120
 */
struct EntityType {
    EntityKind kind{EntityKind::Other};    ///< Top-level kind
    UInt8 domain{0};                       ///< Domain (Air, Land, Sea, etc.)
    UInt16 country{0};                     ///< Country code (ISO 3166 or SISO)
    UInt8 category{0};                     ///< Category within domain
    UInt8 subcategory{0};                  ///< Subcategory
    UInt8 specific{0};                     ///< Specific type
    UInt8 extra{0};                        ///< Extra (variant, configuration)

    constexpr EntityType() noexcept = default;

    constexpr EntityType(EntityKind k, UInt8 d, UInt16 c,
                         UInt8 cat, UInt8 sub, UInt8 spec, UInt8 ex = 0) noexcept
        : kind(k), domain(d), country(c),
          category(cat), subcategory(sub), specific(spec), extra(ex) {}

    constexpr bool operator==(const EntityType& other) const noexcept {
        return kind == other.kind && domain == other.domain &&
               country == other.country && category == other.category &&
               subcategory == other.subcategory && specific == other.specific &&
               extra == other.extra;
    }

    /// Create platform entity type
    static constexpr EntityType create_platform(PlatformDomain domain,
                                                 UInt16 country,
                                                 UInt8 category,
                                                 UInt8 subcategory = 0,
                                                 UInt8 specific = 0,
                                                 UInt8 extra = 0) noexcept {
        return EntityType(EntityKind::Platform, static_cast<UInt8>(domain),
                          country, category, subcategory, specific, extra);
    }

    /// Create munition entity type
    static constexpr EntityType create_munition(UInt16 country,
                                                UInt8 category,
                                                UInt8 subcategory = 0,
                                                UInt8 specific = 0) noexcept {
        return EntityType(EntityKind::Munition, 0, country,
                          category, subcategory, specific, 0);
    }
};

// ============================================================================
// Geodetic Coordinates (LLA)
// ============================================================================

/**
 * @brief Latitude, Longitude, Altitude coordinates (geodetic WGS84)
 */
struct GeodeticCoordinates {
    Real latitude{0.0};     ///< Latitude in radians (-π/2 to π/2)
    Real longitude{0.0};    ///< Longitude in radians (-π to π)
    Real altitude{0.0};     ///< Altitude above WGS84 ellipsoid (meters)

    constexpr GeodeticCoordinates() noexcept = default;
    constexpr GeodeticCoordinates(Real lat, Real lon, Real alt) noexcept
        : latitude(lat), longitude(lon), altitude(alt) {}

    /// Create from degrees
    static GeodeticCoordinates from_degrees(Real lat_deg, Real lon_deg, Real alt_m) noexcept {
        return GeodeticCoordinates(
            lat_deg * constants::DEG_TO_RAD,
            lon_deg * constants::DEG_TO_RAD,
            alt_m
        );
    }

    /// Get latitude in degrees
    Real latitude_degrees() const noexcept { return latitude * constants::RAD_TO_DEG; }

    /// Get longitude in degrees
    Real longitude_degrees() const noexcept { return longitude * constants::RAD_TO_DEG; }
};

// ============================================================================
// Euler Angles
// ============================================================================

/**
 * @brief Euler angles for entity orientation (DIS convention)
 *
 * DIS uses a right-handed coordinate system with angles in radians:
 * - Psi (ψ): Heading/yaw - rotation about Z axis (0 = North, π/2 = East)
 * - Theta (θ): Pitch - rotation about Y axis (positive = nose up)
 * - Phi (φ): Roll - rotation about X axis (positive = right wing down)
 */
struct EulerAngles {
    Real psi{0.0};      ///< Yaw/heading (radians, 0 to 2π)
    Real theta{0.0};    ///< Pitch (radians, -π/2 to π/2)
    Real phi{0.0};      ///< Roll (radians, -π to π)

    constexpr EulerAngles() noexcept = default;
    constexpr EulerAngles(Real psi_, Real theta_, Real phi_) noexcept
        : psi(psi_), theta(theta_), phi(phi_) {}

    /// Create from degrees
    static constexpr EulerAngles from_degrees(Real psi_deg, Real theta_deg, Real phi_deg) noexcept {
        return EulerAngles(
            psi_deg * constants::DEG_TO_RAD,
            theta_deg * constants::DEG_TO_RAD,
            phi_deg * constants::DEG_TO_RAD
        );
    }

    /// Get yaw in degrees
    Real psi_degrees() const noexcept { return psi * constants::RAD_TO_DEG; }

    /// Get pitch in degrees
    Real theta_degrees() const noexcept { return theta * constants::RAD_TO_DEG; }

    /// Get roll in degrees
    Real phi_degrees() const noexcept { return phi * constants::RAD_TO_DEG; }
};

// ============================================================================
// Dead Reckoning Parameters
// ============================================================================

/**
 * @brief Dead reckoning parameters for entity extrapolation
 *
 * Contains the algorithm and parameters needed to extrapolate
 * entity position and orientation between state updates.
 */
struct DeadReckoningParameters {
    DeadReckoningAlgorithm algorithm{DeadReckoningAlgorithm::DRM_Static};
    UInt8 other_parameters[15]{0};  ///< Reserved for future use
    Vec3 linear_acceleration{0.0, 0.0, 0.0};  ///< Linear acceleration (m/s²) in ECEF
    Vec3 angular_velocity{0.0, 0.0, 0.0};     ///< Angular velocity (rad/s) in body frame

    constexpr DeadReckoningParameters() noexcept = default;

    /// Create with algorithm only
    explicit constexpr DeadReckoningParameters(DeadReckoningAlgorithm alg) noexcept
        : algorithm(alg) {}

    /// Create with full parameters
    constexpr DeadReckoningParameters(DeadReckoningAlgorithm alg,
                                      const Vec3& lin_accel,
                                      const Vec3& ang_vel) noexcept
        : algorithm(alg), linear_acceleration(lin_accel), angular_velocity(ang_vel) {}
};

// ============================================================================
// Entity Marking
// ============================================================================

/**
 * @brief Entity marking/name (callsign, tail number, etc.)
 */
struct EntityMarking {
    UInt8 character_set{1};  ///< Character set (1 = ASCII)
    char characters[DIS_MARKING_LENGTH]{0};  ///< Marking string (null-terminated)

    EntityMarking() noexcept = default;

    explicit EntityMarking(const std::string& marking) noexcept {
        set_marking(marking);
    }

    /// Set marking from string (truncates if too long)
    void set_marking(const std::string& marking) noexcept {
        std::memset(characters, 0, DIS_MARKING_LENGTH);
        SizeT len = std::min(marking.size(), DIS_MARKING_LENGTH - 1);
        std::memcpy(characters, marking.c_str(), len);
    }

    /// Get marking as string
    std::string get_marking() const noexcept {
        return std::string(characters,
                          std::min(std::strlen(characters), DIS_MARKING_LENGTH));
    }
};

// ============================================================================
// Articulation Parameter
// ============================================================================

/**
 * @brief Articulation/attachment parameter (turret rotation, etc.)
 */
struct ArticulationParameter {
    UInt8 parameter_type_designator{0};  ///< Type designator
    UInt8 change_indicator{0};           ///< Change indicator (incremented on change)
    UInt16 attachment_id{0};             ///< Attachment ID
    UInt32 parameter_type{0};            ///< Parameter type code
    Real parameter_value{0.0};           ///< Parameter value (units depend on type)

    constexpr ArticulationParameter() noexcept = default;
};

// ============================================================================
// Entity Appearance
// ============================================================================

/**
 * @brief Entity appearance bit field (32 bits)
 *
 * Encodes visual appearance: damage, smoke, flames, lights, etc.
 * Bit meanings depend on entity kind and domain.
 */
struct EntityAppearance {
    UInt32 bits{0};

    constexpr EntityAppearance() noexcept = default;
    explicit constexpr EntityAppearance(UInt32 b) noexcept : bits(b) {}

    // Common appearance bits (bit 0-15 are common across types)

    /// Paint scheme (bits 0-1): 0=Uniform, 1=Camouflage
    constexpr UInt8 paint_scheme() const noexcept { return bits & 0x3; }
    constexpr void set_paint_scheme(UInt8 scheme) noexcept {
        bits = (bits & ~0x3) | (scheme & 0x3);
    }

    /// Damage (bits 3-4): 0=None, 1=Slight, 2=Moderate, 3=Destroyed
    constexpr UInt8 damage() const noexcept { return (bits >> 3) & 0x3; }
    constexpr void set_damage(UInt8 dmg) noexcept {
        bits = (bits & ~(0x3 << 3)) | ((dmg & 0x3) << 3);
    }

    /// Smoke (bits 5-6): 0=None, 1=Small, 2=Medium, 3=Large
    constexpr UInt8 smoke() const noexcept { return (bits >> 5) & 0x3; }
    constexpr void set_smoke(UInt8 smk) noexcept {
        bits = (bits & ~(0x3 << 5)) | ((smk & 0x3) << 5);
    }

    /// Trailing effects (bits 7-8): 0=None, 1=Small, 2=Medium, 3=Large
    constexpr UInt8 trailing_effects() const noexcept { return (bits >> 7) & 0x3; }

    /// Hatch state (bit 9): 0=Closed, 1=Open
    constexpr bool hatch_open() const noexcept { return (bits >> 9) & 0x1; }

    /// Lights on (bit 12): 0=Off, 1=On
    constexpr bool lights_on() const noexcept { return (bits >> 12) & 0x1; }
    constexpr void set_lights(bool on) noexcept {
        if (on) bits |= (1 << 12);
        else bits &= ~(1 << 12);
    }

    /// Flaming (bit 15): 0=No flames, 1=Flames
    constexpr bool flaming() const noexcept { return (bits >> 15) & 0x1; }
    constexpr void set_flaming(bool on) noexcept {
        if (on) bits |= (1 << 15);
        else bits &= ~(1 << 15);
    }
};

// ============================================================================
// Entity Capabilities
// ============================================================================

/**
 * @brief Entity capabilities bit field (32 bits)
 */
struct EntityCapabilities {
    UInt32 bits{0};

    constexpr EntityCapabilities() noexcept = default;
    explicit constexpr EntityCapabilities(UInt32 b) noexcept : bits(b) {}

    /// AmmunitionSupply (bit 0)
    constexpr bool has_ammunition_supply() const noexcept { return bits & 0x1; }

    /// FuelSupply (bit 1)
    constexpr bool has_fuel_supply() const noexcept { return (bits >> 1) & 0x1; }

    /// Recovery (bit 2)
    constexpr bool has_recovery() const noexcept { return (bits >> 2) & 0x1; }

    /// Repair (bit 3)
    constexpr bool has_repair() const noexcept { return (bits >> 3) & 0x1; }
};

// ============================================================================
// PDU Header
// ============================================================================

/**
 * @brief Common header for all DIS PDUs
 *
 * Every DIS message starts with this 12-byte header.
 */
struct PDUHeader {
    UInt8 protocol_version{DIS_VERSION};  ///< Protocol version (7)
    UInt8 exercise_id{0};                 ///< Exercise identifier
    PDUType pdu_type{PDUType::Unknown};   ///< PDU type
    UInt8 protocol_family{1};             ///< Protocol family
    UInt32 timestamp{0};                  ///< Timestamp (units of 1 hour / 2^31)
    UInt16 length{0};                     ///< PDU length in bytes
    UInt16 padding{0};                    ///< Padding (for 32-bit alignment)

    constexpr PDUHeader() noexcept = default;

    explicit constexpr PDUHeader(PDUType type) noexcept
        : pdu_type(type) {}
};

// ============================================================================
// Entity State PDU
// ============================================================================

/**
 * @brief Entity State PDU (Type 1)
 *
 * The most common PDU type. Broadcasts the complete state of an entity
 * including position, velocity, orientation, and appearance.
 */
struct EntityStatePDU {
    PDUHeader header;

    // Entity identification
    EntityIdentifier entity_id;
    ForceId force_id{ForceId::Other};
    UInt8 num_articulation_params{0};

    // Entity type
    EntityType entity_type;
    EntityType alternative_entity_type;  // Alternate representation

    // Linear motion (ECEF coordinates)
    Vec3 entity_linear_velocity{0.0, 0.0, 0.0};  // m/s
    Vec3 entity_location{0.0, 0.0, 0.0};         // ECEF meters

    // Orientation (Euler angles)
    EulerAngles entity_orientation;

    // Visual appearance
    EntityAppearance appearance;

    // Dead reckoning
    DeadReckoningParameters dead_reckoning_params;

    // Entity marking
    EntityMarking entity_marking;

    // Capabilities
    EntityCapabilities capabilities;

    // Variable-length articulation parameters
    std::vector<ArticulationParameter> articulation_params;

    EntityStatePDU() noexcept {
        header.pdu_type = PDUType::EntityState;
        header.protocol_family = 1;  // Entity Information/Interaction
    }
};

// ============================================================================
// Burst Descriptor
// ============================================================================

/**
 * @brief Describes munition burst characteristics
 */
struct BurstDescriptor {
    EntityType munition_type;
    UInt16 warhead{0};              ///< Warhead type
    UInt16 fuse{0};                 ///< Fuse type
    UInt16 quantity{1};             ///< Number of rounds
    UInt16 rate{0};                 ///< Rate of fire (rounds/min)

    constexpr BurstDescriptor() noexcept = default;
};

// ============================================================================
// Fire PDU
// ============================================================================

/**
 * @brief Fire PDU (Type 2)
 *
 * Indicates that a weapon has been fired. Includes firing entity,
 * target entity, munition type, and initial conditions.
 */
struct FirePDU {
    PDUHeader header;

    EntityIdentifier firing_entity_id;
    EntityIdentifier target_entity_id;
    EntityIdentifier munition_id;       ///< ID of the munition entity
    EventIdentifier event_id;           ///< Unique event identifier

    UInt32 fire_mission_index{0};       ///< Fire mission index
    Vec3 location_in_world;             ///< Fire location (ECEF meters)
    BurstDescriptor burst_descriptor;
    Vec3 velocity;                      ///< Initial munition velocity (m/s)
    Real range{0.0};                    ///< Range to target (meters)

    FirePDU() noexcept {
        header.pdu_type = PDUType::Fire;
        header.protocol_family = 1;
    }
};

// ============================================================================
// Detonation PDU
// ============================================================================

/**
 * @brief Detonation PDU (Type 3)
 *
 * Indicates that a munition has detonated or impacted.
 * Includes location, velocity, and result (hit/miss/etc.).
 */
struct DetonationPDU {
    PDUHeader header;

    EntityIdentifier firing_entity_id;
    EntityIdentifier target_entity_id;
    EntityIdentifier munition_id;
    EventIdentifier event_id;

    Vec3 velocity;                      ///< Velocity at detonation (m/s)
    Vec3 location_in_world;             ///< Detonation location (ECEF meters)
    BurstDescriptor burst_descriptor;

    Vec3 location_in_entity;            ///< Impact location relative to target
    DetonationResult detonation_result;
    UInt8 num_articulation_params{0};
    UInt16 padding{0};

    std::vector<ArticulationParameter> articulation_params;

    DetonationPDU() noexcept {
        header.pdu_type = PDUType::Detonation;
        header.protocol_family = 1;
    }
};

// ============================================================================
// Collision PDU
// ============================================================================

/**
 * @brief Collision PDU (Type 4)
 *
 * Reports collision between entities or entity-environment collision.
 */
struct CollisionPDU {
    PDUHeader header;

    EntityIdentifier issuing_entity_id;
    EntityIdentifier colliding_entity_id;
    EventIdentifier event_id;

    UInt8 collision_type{0};            ///< 0=Elastic, 1=Inelastic
    UInt8 padding{0};

    Vec3 velocity;                      ///< Velocity of issuing entity (m/s)
    Real mass{0.0};                     ///< Mass of issuing entity (kg)
    Vec3 location;                      ///< Collision location (ECEF meters)

    CollisionPDU() noexcept {
        header.pdu_type = PDUType::Collision;
        header.protocol_family = 1;
    }
};

// ============================================================================
// Start/Resume PDU
// ============================================================================

/**
 * @brief Start/Resume PDU (Type 13)
 *
 * Used to start or resume a simulation exercise.
 */
struct StartResumePDU {
    PDUHeader header;

    EntityIdentifier originating_entity_id;
    EntityIdentifier receiving_entity_id;

    UInt32 real_world_time{0};          ///< Real-world time (seconds since midnight)
    UInt32 simulation_time{0};          ///< Simulation time (seconds since start)
    UInt32 request_id{0};               ///< Request identifier

    StartResumePDU() noexcept {
        header.pdu_type = PDUType::StartResume;
        header.protocol_family = 4;  // Simulation Management
    }
};

// ============================================================================
// Stop/Freeze PDU
// ============================================================================

/**
 * @brief Stop/Freeze PDU (Type 14)
 *
 * Used to stop or freeze a simulation exercise.
 */
struct StopFreezePDU {
    PDUHeader header;

    EntityIdentifier originating_entity_id;
    EntityIdentifier receiving_entity_id;

    UInt32 real_world_time{0};
    UInt32 reason{0};                   ///< Reason code
    UInt32 frozen_behavior{0};          ///< Behavior while frozen
    UInt32 request_id{0};

    StopFreezePDU() noexcept {
        header.pdu_type = PDUType::StopFreeze;
        header.protocol_family = 4;
    }
};

// ============================================================================
// Coordinate Transformation Functions
// ============================================================================

/**
 * @brief Convert geodetic (LLA) to geocentric ECEF coordinates
 *
 * Uses WGS84 ellipsoid parameters.
 *
 * @param lla Geodetic coordinates (latitude, longitude, altitude)
 * @return ECEF coordinates in meters
 */
Vec3 lla_to_geocentric(const GeodeticCoordinates& lla) noexcept;

/**
 * @brief Convert geocentric ECEF to geodetic (LLA) coordinates
 *
 * Uses iterative algorithm for accuracy.
 *
 * @param ecef ECEF coordinates in meters
 * @return Geodetic coordinates
 */
GeodeticCoordinates geocentric_to_lla(const Vec3& ecef) noexcept;

/**
 * @brief Convert Euler angles to orientation quaternion
 *
 * @param euler Euler angles (psi, theta, phi)
 * @return Orientation quaternion
 */
Quat euler_to_orientation(const EulerAngles& euler) noexcept;

/**
 * @brief Convert orientation quaternion to Euler angles
 *
 * @param q Orientation quaternion
 * @return Euler angles (psi, theta, phi)
 */
EulerAngles orientation_to_euler(const Quat& q) noexcept;

// ============================================================================
// Dead Reckoning Calculator
// ============================================================================

/**
 * @brief Dead reckoning extrapolation calculator
 *
 * Implements DIS dead reckoning algorithms to predict entity
 * position and orientation between state updates.
 */
class DeadReckoningCalculator {
public:
    DeadReckoningCalculator() = default;

    /**
     * @brief Extrapolate entity position using dead reckoning
     *
     * @param state Current entity state (at last update time)
     * @param elapsed_time Time since last update (seconds)
     * @return Extrapolated state
     */
    EntityStatePDU extrapolate_position(const EntityStatePDU& state,
                                        Real elapsed_time) const noexcept;

    /**
     * @brief Check if entity state should be updated
     *
     * Determines if the error between actual and predicted position/orientation
     * exceeds thresholds, indicating a new Entity State PDU should be sent.
     *
     * @param current Current actual state
     * @param predicted Predicted state from dead reckoning
     * @param position_threshold Position error threshold (meters)
     * @param orientation_threshold Orientation error threshold (radians)
     * @return True if update should be sent
     */
    bool should_send_update(const EntityStatePDU& current,
                           const EntityStatePDU& predicted,
                           Real position_threshold = DIS_DR_THRESHOLD_POSITION,
                           Real orientation_threshold = DIS_DR_THRESHOLD_ORIENTATION *
                                                        constants::DEG_TO_RAD) const noexcept;

    /**
     * @brief Calculate position error between two states
     *
     * @return Distance in meters
     */
    Real calculate_position_error(const EntityStatePDU& a,
                                  const EntityStatePDU& b) const noexcept;

    /**
     * @brief Calculate orientation error between two states
     *
     * @return Angle difference in radians
     */
    Real calculate_orientation_error(const EntityStatePDU& a,
                                     const EntityStatePDU& b) const noexcept;

private:
    /// Extrapolate using DRM_FPW (Fixed, Position, World)
    EntityStatePDU extrapolate_fpw(const EntityStatePDU& state, Real dt) const noexcept;

    /// Extrapolate using DRM_RPW (Rotating, Position, World)
    EntityStatePDU extrapolate_rpw(const EntityStatePDU& state, Real dt) const noexcept;

    /// Extrapolate using DRM_RVW (Rotating, Velocity, World)
    EntityStatePDU extrapolate_rvw(const EntityStatePDU& state, Real dt) const noexcept;

    /// Extrapolate using DRM_FVW (Fixed, Velocity, World)
    EntityStatePDU extrapolate_fvw(const EntityStatePDU& state, Real dt) const noexcept;
};

// ============================================================================
// DIS Codec Interface
// ============================================================================

/**
 * @brief Interface for encoding/decoding DIS PDUs to/from byte buffers
 *
 * DIS uses big-endian (network byte order) for all multi-byte values.
 */
class IDISCodec {
public:
    virtual ~IDISCodec() = default;

    /**
     * @brief Encode Entity State PDU to byte buffer
     *
     * @param pdu PDU to encode
     * @param buffer Output buffer (must be at least DIS_MAX_PDU_SIZE)
     * @return Number of bytes written, or 0 on error
     */
    virtual SizeT encode(const EntityStatePDU& pdu, UInt8* buffer) = 0;

    /**
     * @brief Decode Entity State PDU from byte buffer
     *
     * @param buffer Input buffer
     * @param length Buffer length
     * @return Decoded PDU, or empty if error
     */
    virtual std::optional<EntityStatePDU> decode_entity_state(const UInt8* buffer,
                                                               SizeT length) = 0;

    /**
     * @brief Encode Fire PDU to byte buffer
     */
    virtual SizeT encode(const FirePDU& pdu, UInt8* buffer) = 0;

    /**
     * @brief Decode Fire PDU from byte buffer
     */
    virtual std::optional<FirePDU> decode_fire(const UInt8* buffer, SizeT length) = 0;

    /**
     * @brief Encode Detonation PDU to byte buffer
     */
    virtual SizeT encode(const DetonationPDU& pdu, UInt8* buffer) = 0;

    /**
     * @brief Decode Detonation PDU from byte buffer
     */
    virtual std::optional<DetonationPDU> decode_detonation(const UInt8* buffer,
                                                           SizeT length) = 0;

    /**
     * @brief Decode PDU header only (to determine type)
     */
    virtual std::optional<PDUHeader> decode_header(const UInt8* buffer, SizeT length) = 0;
};

// ============================================================================
// DIS Network Interface
// ============================================================================

/**
 * @brief Interface for sending/receiving DIS PDUs over network
 *
 * DIS typically uses UDP multicast on port 3000 for broadcast,
 * or unicast for point-to-point communication.
 */
class IDISNetwork {
public:
    virtual ~IDISNetwork() = default;

    /**
     * @brief Initialize network (bind socket, join multicast group)
     *
     * @param multicast_address Multicast group address (e.g., "239.1.2.3")
     * @param port Port number (default 3000)
     * @param interface_address Local interface to bind (empty = any)
     * @return True on success
     */
    virtual bool initialize(const std::string& multicast_address,
                           UInt16 port = 3000,
                           const std::string& interface_address = "") = 0;

    /**
     * @brief Shutdown network
     */
    virtual void shutdown() = 0;

    /**
     * @brief Send Entity State PDU
     */
    virtual bool send_pdu(const EntityStatePDU& pdu) = 0;

    /**
     * @brief Send Fire PDU
     */
    virtual bool send_pdu(const FirePDU& pdu) = 0;

    /**
     * @brief Send Detonation PDU
     */
    virtual bool send_pdu(const DetonationPDU& pdu) = 0;

    /**
     * @brief Receive next PDU (non-blocking)
     *
     * @param buffer Buffer to receive into
     * @param buffer_size Size of buffer
     * @return Number of bytes received, or 0 if no data available
     */
    virtual SizeT receive_pdu(UInt8* buffer, SizeT buffer_size) = 0;

    /**
     * @brief Join DIS exercise
     *
     * Sets the site and application IDs for this simulation.
     *
     * @param site_id Site identifier
     * @param application_id Application identifier
     */
    virtual void join_exercise(UInt16 site_id, UInt16 application_id) = 0;

    /**
     * @brief Leave exercise (send Remove Entity PDUs)
     */
    virtual void leave_exercise() = 0;

    /**
     * @brief Get local site ID
     */
    virtual UInt16 get_site_id() const = 0;

    /**
     * @brief Get local application ID
     */
    virtual UInt16 get_application_id() const = 0;

    /**
     * @brief Get next entity number (auto-increment)
     */
    virtual UInt16 allocate_entity_id() = 0;
};

// ============================================================================
// Factory Functions
// ============================================================================

/**
 * @brief Create standard DIS codec (big-endian)
 */
std::unique_ptr<IDISCodec> create_dis_codec();

/**
 * @brief Create DIS network handler (UDP multicast)
 */
std::unique_ptr<IDISNetwork> create_dis_network();

// ============================================================================
// Utility Functions
// ============================================================================

/**
 * @brief Get current DIS timestamp
 *
 * DIS timestamps are in units of hours / 2^31, with the LSB
 * representing approximately 1.676 microseconds.
 *
 * @return Current timestamp
 */
UInt32 get_dis_timestamp() noexcept;

/**
 * @brief Convert DIS timestamp to seconds
 */
Real dis_timestamp_to_seconds(UInt32 timestamp) noexcept;

/**
 * @brief Convert seconds to DIS timestamp
 */
UInt32 seconds_to_dis_timestamp(Real seconds) noexcept;

/**
 * @brief Calculate distance between two ECEF positions
 *
 * @return Distance in meters
 */
inline Real distance_ecef(const Vec3& a, const Vec3& b) noexcept {
    return (a - b).length();
}

/**
 * @brief Calculate great circle distance between two LLA positions
 *
 * Uses haversine formula for accuracy.
 *
 * @return Distance in meters
 */
Real distance_lla(const GeodeticCoordinates& a,
                 const GeodeticCoordinates& b) noexcept;

} // namespace jaguar::federation
