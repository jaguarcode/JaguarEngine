#pragma once
/**
 * @file dis_fire_pdu.h
 * @brief DIS Fire and Detonation PDU handlers
 *
 * This file provides specialized handlers for Fire PDU (Type 2) and Detonation PDU (Type 3)
 * in the DIS protocol. These PDUs are part of the Warfare Family and handle weapon fire
 * events and munition detonations/impacts.
 *
 * Fire PDUs are sent when a weapon is fired and include:
 * - Firing entity identification
 * - Target entity identification
 * - Munition type and characteristics
 * - Initial location and velocity
 * - Fire mission parameters
 *
 * Detonation PDUs are sent when a munition detonates or impacts and include:
 * - Detonation location and velocity
 * - Detonation result (hit, miss, type of impact)
 * - Target damage assessment
 * - Burst characteristics
 *
 * Standards compliance:
 * - IEEE 1278.1-2012 (DIS version 7)
 * - IEEE 1278.1a-1998 (DIS version 6)
 * - SISO-REF-010 (Entity Type enumerations)
 *
 * @see dis_protocol.h for base PDU types and structures
 */

#include "jaguar/federation/dis_protocol.h"
#include <optional>
#include <vector>

namespace jaguar::federation {

// ============================================================================
// Fire PDU Handler
// ============================================================================

/**
 * @brief Handler for Fire PDU encoding and decoding
 *
 * Provides methods to serialize/deserialize Fire PDUs to/from network byte order.
 * Includes validation of PDU fields and support for both DIS v6 and v7.
 */
class FirePDUHandler {
public:
    FirePDUHandler() = default;
    ~FirePDUHandler() = default;

    /**
     * @brief Serialize Fire PDU to network byte order
     *
     * Encodes a Fire PDU into a byte buffer using big-endian (network) byte order.
     * The PDU header length field is automatically updated.
     *
     * @param pdu Fire PDU to encode
     * @param buffer Output buffer (must be at least get_fire_pdu_size() bytes)
     * @param buffer_size Size of output buffer
     * @return Number of bytes written, or 0 on error
     */
    SizeT serialize(const FirePDU& pdu, UInt8* buffer, SizeT buffer_size);

    /**
     * @brief Deserialize Fire PDU from network byte order
     *
     * Decodes a Fire PDU from a byte buffer. Validates PDU type and length.
     *
     * @param buffer Input buffer containing encoded PDU
     * @param length Length of input buffer
     * @return Decoded Fire PDU, or std::nullopt if invalid
     */
    std::optional<FirePDU> deserialize(const UInt8* buffer, SizeT length);

    /**
     * @brief Validate Fire PDU fields
     *
     * Checks that all required fields are valid:
     * - Valid entity identifiers (non-zero)
     * - Valid event identifier
     * - Range >= 0
     * - Velocity magnitude reasonable
     *
     * @param pdu Fire PDU to validate
     * @return True if PDU is valid
     */
    bool validate(const FirePDU& pdu) const;

    /**
     * @brief Get size of Fire PDU in bytes
     *
     * Fire PDU has fixed size:
     * - Header: 12 bytes
     * - Entity IDs: 18 bytes (3 * 6)
     * - Event ID: 6 bytes
     * - Fire mission: 4 bytes
     * - Location: 24 bytes (3 * 8 doubles)
     * - Burst descriptor: 16 bytes
     * - Velocity: 12 bytes (3 * 4 floats)
     * - Range: 4 bytes
     * Total: 96 bytes
     *
     * @return Fixed size of Fire PDU (96 bytes)
     */
    static constexpr SizeT get_fire_pdu_size() { return 96; }

    /**
     * @brief Create Fire PDU from weapon fire event
     *
     * Helper method to construct a Fire PDU from common parameters.
     *
     * @param firing_entity Entity that fired the weapon
     * @param target_entity Target entity (may be invalid for unguided munitions)
     * @param munition_type Type of munition fired
     * @param location Fire location in ECEF coordinates (meters)
     * @param velocity Initial munition velocity (m/s)
     * @param range Range to target (meters, 0 if unknown)
     * @return Constructed Fire PDU
     */
    FirePDU create_fire_pdu(
        const EntityIdentifier& firing_entity,
        const EntityIdentifier& target_entity,
        const EntityType& munition_type,
        const Vec3& location,
        const Vec3& velocity,
        Real range = 0.0
    );

    /**
     * @brief Set burst descriptor for Fire PDU
     *
     * Configures munition characteristics including warhead and fuse types.
     *
     * @param pdu Fire PDU to modify
     * @param munition_type Munition entity type
     * @param warhead Warhead type code (SISO-REF-010)
     * @param fuse Fuse type code (SISO-REF-010)
     * @param quantity Number of rounds in burst
     * @param rate Rate of fire (rounds/minute)
     */
    void set_burst_descriptor(
        FirePDU& pdu,
        const EntityType& munition_type,
        UInt16 warhead,
        UInt16 fuse,
        UInt16 quantity = 1,
        UInt16 rate = 0
    );

private:
    UInt16 next_event_number_{1};  ///< Auto-incrementing event number
};

// ============================================================================
// Detonation PDU Handler
// ============================================================================

/**
 * @brief Handler for Detonation PDU encoding and decoding
 *
 * Provides methods to serialize/deserialize Detonation PDUs to/from network byte order.
 * Includes validation of PDU fields and support for both DIS v6 and v7.
 */
class DetonationPDUHandler {
public:
    DetonationPDUHandler() = default;
    ~DetonationPDUHandler() = default;

    /**
     * @brief Serialize Detonation PDU to network byte order
     *
     * Encodes a Detonation PDU into a byte buffer using big-endian byte order.
     * Handles variable-length articulation parameters.
     *
     * @param pdu Detonation PDU to encode
     * @param buffer Output buffer (must be sufficient for PDU + articulation params)
     * @param buffer_size Size of output buffer
     * @return Number of bytes written, or 0 on error
     */
    SizeT serialize(const DetonationPDU& pdu, UInt8* buffer, SizeT buffer_size);

    /**
     * @brief Deserialize Detonation PDU from network byte order
     *
     * Decodes a Detonation PDU from a byte buffer, including variable-length
     * articulation parameters.
     *
     * @param buffer Input buffer containing encoded PDU
     * @param length Length of input buffer
     * @return Decoded Detonation PDU, or std::nullopt if invalid
     */
    std::optional<DetonationPDU> deserialize(const UInt8* buffer, SizeT length);

    /**
     * @brief Validate Detonation PDU fields
     *
     * Checks that all required fields are valid:
     * - Valid entity identifiers
     * - Valid event identifier
     * - Valid detonation result
     * - Reasonable velocity magnitude
     * - Articulation parameter count matches array size
     *
     * @param pdu Detonation PDU to validate
     * @return True if PDU is valid
     */
    bool validate(const DetonationPDU& pdu) const;

    /**
     * @brief Get minimum size of Detonation PDU in bytes
     *
     * Detonation PDU base size (without articulation parameters):
     * - Header: 12 bytes
     * - Entity IDs: 18 bytes (3 * 6)
     * - Event ID: 6 bytes
     * - Velocity: 12 bytes (3 * 4 floats)
     * - Location: 24 bytes (3 * 8 doubles)
     * - Burst descriptor: 16 bytes
     * - Location in entity: 12 bytes (3 * 4 floats)
     * - Result/count/padding: 4 bytes
     * Total base: 104 bytes
     * Plus: 16 bytes per articulation parameter
     *
     * @param num_articulation_params Number of articulation parameters
     * @return Size of Detonation PDU in bytes
     */
    static constexpr SizeT get_detonation_pdu_size(UInt8 num_articulation_params = 0) {
        return 104 + (num_articulation_params * 16);
    }

    /**
     * @brief Create Detonation PDU from munition impact event
     *
     * Helper method to construct a Detonation PDU from common parameters.
     *
     * @param firing_entity Entity that fired the munition
     * @param target_entity Target entity (may be invalid for ground impacts)
     * @param munition_id Munition entity identifier
     * @param event_id Event identifier (matches Fire PDU)
     * @param location Detonation location in ECEF coordinates (meters)
     * @param velocity Velocity at detonation (m/s)
     * @param result Detonation result (hit, miss, etc.)
     * @return Constructed Detonation PDU
     */
    DetonationPDU create_detonation_pdu(
        const EntityIdentifier& firing_entity,
        const EntityIdentifier& target_entity,
        const EntityIdentifier& munition_id,
        const EventIdentifier& event_id,
        const Vec3& location,
        const Vec3& velocity,
        DetonationResult result
    );

    /**
     * @brief Set burst descriptor for Detonation PDU
     *
     * Configures munition characteristics matching the Fire PDU.
     *
     * @param pdu Detonation PDU to modify
     * @param munition_type Munition entity type
     * @param warhead Warhead type code
     * @param fuse Fuse type code
     * @param quantity Number of rounds
     * @param rate Rate of fire
     */
    void set_burst_descriptor(
        DetonationPDU& pdu,
        const EntityType& munition_type,
        UInt16 warhead,
        UInt16 fuse,
        UInt16 quantity = 1,
        UInt16 rate = 0
    );

    /**
     * @brief Set impact location relative to target entity
     *
     * For entity impacts, specifies where the munition hit relative to
     * the target's coordinate system (body frame).
     *
     * @param pdu Detonation PDU to modify
     * @param location_in_entity Impact location in target body frame (meters)
     */
    void set_location_in_entity(DetonationPDU& pdu, const Vec3& location_in_entity);

    /**
     * @brief Add articulation parameter to Detonation PDU
     *
     * Articulation parameters can describe damage effects on target entity
     * (e.g., turret destroyed, engine disabled).
     *
     * @param pdu Detonation PDU to modify
     * @param param Articulation parameter to add
     * @return True if added successfully (max 255 parameters)
     */
    bool add_articulation_parameter(DetonationPDU& pdu, const ArticulationParameter& param);
};

// ============================================================================
// Utility Functions
// ============================================================================

/**
 * @brief Calculate ballistic trajectory endpoint
 *
 * Estimates where a munition will impact given initial conditions.
 * Uses simplified ballistic model (does not account for drag).
 *
 * @param fire_location Initial position (ECEF meters)
 * @param fire_velocity Initial velocity (m/s ECEF)
 * @param time_of_flight Time to detonation (seconds)
 * @return Estimated detonation location
 */
Vec3 calculate_ballistic_endpoint(
    const Vec3& fire_location,
    const Vec3& fire_velocity,
    Real time_of_flight
);

/**
 * @brief Calculate time of flight for ballistic trajectory
 *
 * Estimates time for munition to reach target range.
 *
 * @param range Range to target (meters)
 * @param muzzle_velocity Initial velocity magnitude (m/s)
 * @param elevation_angle Elevation angle (radians)
 * @return Estimated time of flight (seconds)
 */
Real calculate_time_of_flight(
    Real range,
    Real muzzle_velocity,
    Real elevation_angle = 0.0
);

/**
 * @brief Match Fire PDU to Detonation PDU
 *
 * Checks if a Detonation PDU corresponds to a given Fire PDU by comparing
 * event identifiers and munition IDs.
 *
 * @param fire Fire PDU
 * @param detonation Detonation PDU
 * @return True if detonation matches fire event
 */
bool match_fire_to_detonation(const FirePDU& fire, const DetonationPDU& detonation);

/**
 * @brief Get detonation result as string
 *
 * @param result Detonation result enumeration
 * @return Human-readable string
 */
const char* detonation_result_to_string(DetonationResult result);

/**
 * @brief Check if detonation was a hit on entity
 *
 * @param result Detonation result
 * @return True if result indicates entity impact
 */
bool is_entity_hit(DetonationResult result);

/**
 * @brief Check if detonation was a ground impact
 *
 * @param result Detonation result
 * @return True if result indicates ground impact
 */
bool is_ground_impact(DetonationResult result);

/**
 * @brief Calculate damage assessment from detonation
 *
 * Provides a simple damage score based on detonation result.
 * Higher values indicate more effective hits.
 *
 * @param result Detonation result
 * @return Damage score (0.0 = no damage, 1.0 = maximum damage)
 */
Real calculate_damage_score(DetonationResult result);

} // namespace jaguar::federation
