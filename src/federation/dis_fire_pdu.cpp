/**
 * @file dis_fire_pdu.cpp
 * @brief Implementation of DIS Fire and Detonation PDU handlers
 */

#include "jaguar/federation/dis_fire_pdu.h"
#include <cmath>
#include <cstring>
#include <algorithm>

namespace jaguar::federation {

namespace {
    // Helper functions for byte-order conversion (big-endian network order)

    inline UInt16 swap_endian16(UInt16 val) {
        return ((val & 0xFF) << 8) | ((val >> 8) & 0xFF);
    }

    inline UInt32 swap_endian32(UInt32 val) {
        return ((val & 0xFF) << 24) |
               ((val & 0xFF00) << 8) |
               ((val & 0xFF0000) >> 8) |
               ((val >> 24) & 0xFF);
    }

    inline void write_be16(UInt8* buffer, UInt16 value) {
        #if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
            value = swap_endian16(value);
        #endif
        std::memcpy(buffer, &value, sizeof(UInt16));
    }

    inline void write_be32(UInt8* buffer, UInt32 value) {
        #if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
            value = swap_endian32(value);
        #endif
        std::memcpy(buffer, &value, sizeof(UInt32));
    }

    inline UInt16 read_be16(const UInt8* buffer) {
        UInt16 value;
        std::memcpy(&value, buffer, sizeof(UInt16));
        #if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
            value = swap_endian16(value);
        #endif
        return value;
    }

    inline UInt32 read_be32(const UInt8* buffer) {
        UInt32 value;
        std::memcpy(&value, buffer, sizeof(UInt32));
        #if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
            value = swap_endian32(value);
        #endif
        return value;
    }

    inline void write_be_float(UInt8* buffer, Real value) {
        float f = static_cast<float>(value);
        UInt32 bits;
        std::memcpy(&bits, &f, sizeof(float));
        write_be32(buffer, bits);
    }

    inline Real read_be_float(const UInt8* buffer) {
        UInt32 bits = read_be32(buffer);
        float f;
        std::memcpy(&f, &bits, sizeof(float));
        return static_cast<Real>(f);
    }

    inline void write_be_double(UInt8* buffer, Real value) {
        double d = static_cast<double>(value);
        UInt64 bits;
        std::memcpy(&bits, &d, sizeof(double));
        #if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
            UInt64 swapped = 0;
            for (int i = 0; i < 8; ++i) {
                swapped |= ((bits >> (i * 8)) & 0xFF) << ((7 - i) * 8);
            }
            bits = swapped;
        #endif
        std::memcpy(buffer, &bits, sizeof(UInt64));
    }

    inline Real read_be_double(const UInt8* buffer) {
        UInt64 bits;
        std::memcpy(&bits, buffer, sizeof(UInt64));
        #if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
            UInt64 swapped = 0;
            for (int i = 0; i < 8; ++i) {
                swapped |= ((bits >> (i * 8)) & 0xFF) << ((7 - i) * 8);
            }
            bits = swapped;
        #endif
        double d;
        std::memcpy(&d, &bits, sizeof(double));
        return static_cast<Real>(d);
    }

    // PDU Header encoding/decoding
    SizeT encode_header(const PDUHeader& header, UInt8* buffer) {
        buffer[0] = header.protocol_version;
        buffer[1] = header.exercise_id;
        buffer[2] = static_cast<UInt8>(header.pdu_type);
        buffer[3] = header.protocol_family;
        write_be32(buffer + 4, header.timestamp);
        write_be16(buffer + 8, header.length);
        write_be16(buffer + 10, header.padding);
        return 12;
    }

    std::optional<PDUHeader> decode_header(const UInt8* buffer, SizeT length) {
        if (length < 12) return std::nullopt;

        PDUHeader header;
        header.protocol_version = buffer[0];
        header.exercise_id = buffer[1];
        header.pdu_type = static_cast<PDUType>(buffer[2]);
        header.protocol_family = buffer[3];
        header.timestamp = read_be32(buffer + 4);
        header.length = read_be16(buffer + 8);
        header.padding = read_be16(buffer + 10);

        return header;
    }

    // Entity Identifier encoding/decoding
    SizeT encode_entity_id(const EntityIdentifier& id, UInt8* buffer) {
        write_be16(buffer, id.site);
        write_be16(buffer + 2, id.application);
        write_be16(buffer + 4, id.entity);
        return 6;
    }

    EntityIdentifier decode_entity_id(const UInt8* buffer) {
        EntityIdentifier id;
        id.site = read_be16(buffer);
        id.application = read_be16(buffer + 2);
        id.entity = read_be16(buffer + 4);
        return id;
    }

    // Event Identifier encoding/decoding
    SizeT encode_event_id(const EventIdentifier& id, UInt8* buffer) {
        write_be16(buffer, id.site);
        write_be16(buffer + 2, id.application);
        write_be16(buffer + 4, id.event_number);
        return 6;
    }

    EventIdentifier decode_event_id(const UInt8* buffer) {
        EventIdentifier id;
        id.site = read_be16(buffer);
        id.application = read_be16(buffer + 2);
        id.event_number = read_be16(buffer + 4);
        return id;
    }

    // Entity Type encoding/decoding
    SizeT encode_entity_type(const EntityType& type, UInt8* buffer) {
        buffer[0] = static_cast<UInt8>(type.kind);
        buffer[1] = type.domain;
        write_be16(buffer + 2, type.country);
        buffer[4] = type.category;
        buffer[5] = type.subcategory;
        buffer[6] = type.specific;
        buffer[7] = type.extra;
        return 8;
    }

    EntityType decode_entity_type(const UInt8* buffer) {
        EntityType type;
        type.kind = static_cast<EntityKind>(buffer[0]);
        type.domain = buffer[1];
        type.country = read_be16(buffer + 2);
        type.category = buffer[4];
        type.subcategory = buffer[5];
        type.specific = buffer[6];
        type.extra = buffer[7];
        return type;
    }

    // Burst Descriptor encoding/decoding
    SizeT encode_burst_descriptor(const BurstDescriptor& burst, UInt8* buffer) {
        encode_entity_type(burst.munition_type, buffer);
        write_be16(buffer + 8, burst.warhead);
        write_be16(buffer + 10, burst.fuse);
        write_be16(buffer + 12, burst.quantity);
        write_be16(buffer + 14, burst.rate);
        return 16;
    }

    BurstDescriptor decode_burst_descriptor(const UInt8* buffer) {
        BurstDescriptor burst;
        burst.munition_type = decode_entity_type(buffer);
        burst.warhead = read_be16(buffer + 8);
        burst.fuse = read_be16(buffer + 10);
        burst.quantity = read_be16(buffer + 12);
        burst.rate = read_be16(buffer + 14);
        return burst;
    }

    // Articulation Parameter encoding/decoding
    SizeT encode_articulation(const ArticulationParameter& param, UInt8* buffer) {
        buffer[0] = param.parameter_type_designator;
        buffer[1] = param.change_indicator;
        write_be16(buffer + 2, param.attachment_id);
        write_be32(buffer + 4, param.parameter_type);
        write_be_double(buffer + 8, param.parameter_value);
        return 16;
    }

    ArticulationParameter decode_articulation(const UInt8* buffer) {
        ArticulationParameter param;
        param.parameter_type_designator = buffer[0];
        param.change_indicator = buffer[1];
        param.attachment_id = read_be16(buffer + 2);
        param.parameter_type = read_be32(buffer + 4);
        param.parameter_value = read_be_double(buffer + 8);
        return param;
    }

    // Velocity magnitude check for validation
    constexpr Real MAX_REASONABLE_VELOCITY = 10000.0;  // 10 km/s (hypersonic)
}

// ============================================================================
// FirePDUHandler Implementation
// ============================================================================

SizeT FirePDUHandler::serialize(const FirePDU& pdu, UInt8* buffer, SizeT buffer_size) {
    if (buffer_size < get_fire_pdu_size()) {
        return 0;  // Buffer too small
    }

    UInt8* ptr = buffer;

    // Encode PDU header
    ptr += encode_header(pdu.header, ptr);

    // Encode entity identifiers
    ptr += encode_entity_id(pdu.firing_entity_id, ptr);
    ptr += encode_entity_id(pdu.target_entity_id, ptr);
    ptr += encode_entity_id(pdu.munition_id, ptr);

    // Encode event identifier
    ptr += encode_event_id(pdu.event_id, ptr);

    // Fire mission index
    write_be32(ptr, pdu.fire_mission_index);
    ptr += 4;

    // Location in world (ECEF coordinates - doubles)
    write_be_double(ptr, pdu.location_in_world.x); ptr += 8;
    write_be_double(ptr, pdu.location_in_world.y); ptr += 8;
    write_be_double(ptr, pdu.location_in_world.z); ptr += 8;

    // Burst descriptor
    ptr += encode_burst_descriptor(pdu.burst_descriptor, ptr);

    // Velocity (floats)
    write_be_float(ptr, pdu.velocity.x); ptr += 4;
    write_be_float(ptr, pdu.velocity.y); ptr += 4;
    write_be_float(ptr, pdu.velocity.z); ptr += 4;

    // Range
    write_be_float(ptr, pdu.range);
    ptr += 4;

    // Update PDU length in header
    SizeT total_length = ptr - buffer;
    write_be16(buffer + 8, static_cast<UInt16>(total_length));

    return total_length;
}

std::optional<FirePDU> FirePDUHandler::deserialize(const UInt8* buffer, SizeT length) {
    if (length < get_fire_pdu_size()) {
        return std::nullopt;  // Buffer too small
    }

    // Decode and validate header
    auto header_opt = decode_header(buffer, length);
    if (!header_opt || header_opt->pdu_type != PDUType::Fire) {
        return std::nullopt;
    }

    FirePDU pdu;
    pdu.header = *header_opt;

    const UInt8* ptr = buffer + 12;  // Skip header

    // Decode entity identifiers
    pdu.firing_entity_id = decode_entity_id(ptr); ptr += 6;
    pdu.target_entity_id = decode_entity_id(ptr); ptr += 6;
    pdu.munition_id = decode_entity_id(ptr); ptr += 6;

    // Decode event identifier
    pdu.event_id = decode_event_id(ptr); ptr += 6;

    // Fire mission index
    pdu.fire_mission_index = read_be32(ptr); ptr += 4;

    // Location in world
    pdu.location_in_world.x = read_be_double(ptr); ptr += 8;
    pdu.location_in_world.y = read_be_double(ptr); ptr += 8;
    pdu.location_in_world.z = read_be_double(ptr); ptr += 8;

    // Burst descriptor
    pdu.burst_descriptor = decode_burst_descriptor(ptr); ptr += 16;

    // Velocity
    pdu.velocity.x = read_be_float(ptr); ptr += 4;
    pdu.velocity.y = read_be_float(ptr); ptr += 4;
    pdu.velocity.z = read_be_float(ptr); ptr += 4;

    // Range
    pdu.range = read_be_float(ptr); ptr += 4;

    return pdu;
}

bool FirePDUHandler::validate(const FirePDU& pdu) const {
    // Check firing entity is valid
    if (!pdu.firing_entity_id.is_valid()) {
        return false;
    }

    // Munition ID should be valid
    if (!pdu.munition_id.is_valid()) {
        return false;
    }

    // Event ID should have valid site and application
    if (pdu.event_id.site == 0 && pdu.event_id.application == 0) {
        return false;
    }

    // Range should be non-negative
    if (pdu.range < 0.0) {
        return false;
    }

    // Velocity magnitude should be reasonable
    Real velocity_mag = pdu.velocity.length();
    if (velocity_mag > MAX_REASONABLE_VELOCITY) {
        return false;
    }

    // Burst descriptor should have at least 1 round
    if (pdu.burst_descriptor.quantity == 0) {
        return false;
    }

    return true;
}

FirePDU FirePDUHandler::create_fire_pdu(
    const EntityIdentifier& firing_entity,
    const EntityIdentifier& target_entity,
    const EntityType& munition_type,
    const Vec3& location,
    const Vec3& velocity,
    Real range) {

    FirePDU pdu;

    // Set header
    pdu.header.pdu_type = PDUType::Fire;
    pdu.header.protocol_family = 1;  // Entity Information/Interaction
    pdu.header.protocol_version = DIS_VERSION;
    pdu.header.timestamp = get_dis_timestamp();

    // Set entity identifiers
    pdu.firing_entity_id = firing_entity;
    pdu.target_entity_id = target_entity;
    pdu.munition_id = firing_entity;  // Munition inherits firing entity's site/app
    pdu.munition_id.entity = 0;  // Will be set by caller

    // Set event identifier
    pdu.event_id.site = firing_entity.site;
    pdu.event_id.application = firing_entity.application;
    pdu.event_id.event_number = next_event_number_++;

    // Set location and velocity
    pdu.location_in_world = location;
    pdu.velocity = velocity;
    pdu.range = range;

    // Set burst descriptor with munition type
    pdu.burst_descriptor.munition_type = munition_type;
    pdu.burst_descriptor.quantity = 1;
    pdu.burst_descriptor.rate = 0;

    return pdu;
}

void FirePDUHandler::set_burst_descriptor(
    FirePDU& pdu,
    const EntityType& munition_type,
    UInt16 warhead,
    UInt16 fuse,
    UInt16 quantity,
    UInt16 rate) {

    pdu.burst_descriptor.munition_type = munition_type;
    pdu.burst_descriptor.warhead = warhead;
    pdu.burst_descriptor.fuse = fuse;
    pdu.burst_descriptor.quantity = quantity;
    pdu.burst_descriptor.rate = rate;
}

// ============================================================================
// DetonationPDUHandler Implementation
// ============================================================================

SizeT DetonationPDUHandler::serialize(const DetonationPDU& pdu, UInt8* buffer, SizeT buffer_size) {
    SizeT required_size = get_detonation_pdu_size(pdu.num_articulation_params);
    if (buffer_size < required_size) {
        return 0;  // Buffer too small
    }

    UInt8* ptr = buffer;

    // Encode PDU header
    ptr += encode_header(pdu.header, ptr);

    // Encode entity identifiers
    ptr += encode_entity_id(pdu.firing_entity_id, ptr);
    ptr += encode_entity_id(pdu.target_entity_id, ptr);
    ptr += encode_entity_id(pdu.munition_id, ptr);

    // Encode event identifier
    ptr += encode_event_id(pdu.event_id, ptr);

    // Velocity at detonation (floats)
    write_be_float(ptr, pdu.velocity.x); ptr += 4;
    write_be_float(ptr, pdu.velocity.y); ptr += 4;
    write_be_float(ptr, pdu.velocity.z); ptr += 4;

    // Location in world (ECEF coordinates - doubles)
    write_be_double(ptr, pdu.location_in_world.x); ptr += 8;
    write_be_double(ptr, pdu.location_in_world.y); ptr += 8;
    write_be_double(ptr, pdu.location_in_world.z); ptr += 8;

    // Burst descriptor
    ptr += encode_burst_descriptor(pdu.burst_descriptor, ptr);

    // Location in entity coordinates (floats)
    write_be_float(ptr, pdu.location_in_entity.x); ptr += 4;
    write_be_float(ptr, pdu.location_in_entity.y); ptr += 4;
    write_be_float(ptr, pdu.location_in_entity.z); ptr += 4;

    // Detonation result, articulation count, padding
    *ptr++ = static_cast<UInt8>(pdu.detonation_result);
    *ptr++ = pdu.num_articulation_params;
    write_be16(ptr, pdu.padding);
    ptr += 2;

    // Encode articulation parameters
    for (const auto& param : pdu.articulation_params) {
        ptr += encode_articulation(param, ptr);
    }

    // Update PDU length in header
    SizeT total_length = ptr - buffer;
    write_be16(buffer + 8, static_cast<UInt16>(total_length));

    return total_length;
}

std::optional<DetonationPDU> DetonationPDUHandler::deserialize(const UInt8* buffer, SizeT length) {
    if (length < get_detonation_pdu_size(0)) {
        return std::nullopt;  // Buffer too small for base PDU
    }

    // Decode and validate header
    auto header_opt = decode_header(buffer, length);
    if (!header_opt || header_opt->pdu_type != PDUType::Detonation) {
        return std::nullopt;
    }

    DetonationPDU pdu;
    pdu.header = *header_opt;

    const UInt8* ptr = buffer + 12;  // Skip header

    // Decode entity identifiers
    pdu.firing_entity_id = decode_entity_id(ptr); ptr += 6;
    pdu.target_entity_id = decode_entity_id(ptr); ptr += 6;
    pdu.munition_id = decode_entity_id(ptr); ptr += 6;

    // Decode event identifier
    pdu.event_id = decode_event_id(ptr); ptr += 6;

    // Velocity at detonation
    pdu.velocity.x = read_be_float(ptr); ptr += 4;
    pdu.velocity.y = read_be_float(ptr); ptr += 4;
    pdu.velocity.z = read_be_float(ptr); ptr += 4;

    // Location in world
    pdu.location_in_world.x = read_be_double(ptr); ptr += 8;
    pdu.location_in_world.y = read_be_double(ptr); ptr += 8;
    pdu.location_in_world.z = read_be_double(ptr); ptr += 8;

    // Burst descriptor
    pdu.burst_descriptor = decode_burst_descriptor(ptr); ptr += 16;

    // Location in entity
    pdu.location_in_entity.x = read_be_float(ptr); ptr += 4;
    pdu.location_in_entity.y = read_be_float(ptr); ptr += 4;
    pdu.location_in_entity.z = read_be_float(ptr); ptr += 4;

    // Detonation result, articulation count, padding
    pdu.detonation_result = static_cast<DetonationResult>(*ptr++);
    pdu.num_articulation_params = *ptr++;
    pdu.padding = read_be16(ptr); ptr += 2;

    // Decode articulation parameters
    for (UInt8 i = 0; i < pdu.num_articulation_params; ++i) {
        if (ptr + 16 > buffer + length) {
            return std::nullopt;  // Incomplete articulation data
        }
        pdu.articulation_params.push_back(decode_articulation(ptr));
        ptr += 16;
    }

    return pdu;
}

bool DetonationPDUHandler::validate(const DetonationPDU& pdu) const {
    // Check firing entity is valid
    if (!pdu.firing_entity_id.is_valid()) {
        return false;
    }

    // Munition ID should be valid
    if (!pdu.munition_id.is_valid()) {
        return false;
    }

    // Event ID should have valid site and application
    if (pdu.event_id.site == 0 && pdu.event_id.application == 0) {
        return false;
    }

    // Velocity magnitude should be reasonable
    Real velocity_mag = pdu.velocity.length();
    if (velocity_mag > MAX_REASONABLE_VELOCITY) {
        return false;
    }

    // Articulation parameter count should match array size
    if (pdu.num_articulation_params != pdu.articulation_params.size()) {
        return false;
    }

    // Detonation result should be valid (< 31)
    if (static_cast<UInt8>(pdu.detonation_result) > 30) {
        return false;
    }

    return true;
}

DetonationPDU DetonationPDUHandler::create_detonation_pdu(
    const EntityIdentifier& firing_entity,
    const EntityIdentifier& target_entity,
    const EntityIdentifier& munition_id,
    const EventIdentifier& event_id,
    const Vec3& location,
    const Vec3& velocity,
    DetonationResult result) {

    DetonationPDU pdu;

    // Set header
    pdu.header.pdu_type = PDUType::Detonation;
    pdu.header.protocol_family = 1;  // Entity Information/Interaction
    pdu.header.protocol_version = DIS_VERSION;
    pdu.header.timestamp = get_dis_timestamp();

    // Set entity identifiers
    pdu.firing_entity_id = firing_entity;
    pdu.target_entity_id = target_entity;
    pdu.munition_id = munition_id;

    // Set event identifier (should match Fire PDU)
    pdu.event_id = event_id;

    // Set location and velocity
    pdu.location_in_world = location;
    pdu.velocity = velocity;

    // Set detonation result
    pdu.detonation_result = result;

    // Initialize with no articulation parameters
    pdu.num_articulation_params = 0;
    pdu.padding = 0;

    return pdu;
}

void DetonationPDUHandler::set_burst_descriptor(
    DetonationPDU& pdu,
    const EntityType& munition_type,
    UInt16 warhead,
    UInt16 fuse,
    UInt16 quantity,
    UInt16 rate) {

    pdu.burst_descriptor.munition_type = munition_type;
    pdu.burst_descriptor.warhead = warhead;
    pdu.burst_descriptor.fuse = fuse;
    pdu.burst_descriptor.quantity = quantity;
    pdu.burst_descriptor.rate = rate;
}

void DetonationPDUHandler::set_location_in_entity(DetonationPDU& pdu, const Vec3& location_in_entity) {
    pdu.location_in_entity = location_in_entity;
}

bool DetonationPDUHandler::add_articulation_parameter(DetonationPDU& pdu, const ArticulationParameter& param) {
    if (pdu.articulation_params.size() >= 255) {
        return false;  // Maximum articulation parameters reached
    }

    pdu.articulation_params.push_back(param);
    pdu.num_articulation_params = static_cast<UInt8>(pdu.articulation_params.size());

    return true;
}

// ============================================================================
// Utility Functions Implementation
// ============================================================================

Vec3 calculate_ballistic_endpoint(
    const Vec3& fire_location,
    const Vec3& fire_velocity,
    Real time_of_flight) {

    // Simplified ballistic model: p = p0 + v*t + 0.5*g*t^2
    // Assumes constant gravity vector pointing toward Earth center

    // Calculate gravity acceleration vector (pointing toward origin)
    Vec3 position_normalized = fire_location.normalized();
    constexpr Real g = 9.80665;  // Standard gravity (m/s^2)
    Vec3 gravity = -position_normalized * g;

    // Calculate endpoint
    Vec3 endpoint = fire_location +
                    fire_velocity * time_of_flight +
                    gravity * (0.5 * time_of_flight * time_of_flight);

    return endpoint;
}

Real calculate_time_of_flight(Real range, Real muzzle_velocity, Real elevation_angle) {
    if (muzzle_velocity <= 0.0 || range <= 0.0) {
        return 0.0;
    }

    // Simplified ballistic equation for time of flight
    // Assumes flat Earth approximation for small ranges
    constexpr Real g = 9.80665;

    Real v_horizontal = muzzle_velocity * std::cos(elevation_angle);

    if (v_horizontal <= 0.0) {
        return 0.0;
    }

    // Time = range / horizontal_velocity
    Real time = range / v_horizontal;

    return time;
}

bool match_fire_to_detonation(const FirePDU& fire, const DetonationPDU& detonation) {
    // Check if event identifiers match
    if (fire.event_id.site != detonation.event_id.site ||
        fire.event_id.application != detonation.event_id.application ||
        fire.event_id.event_number != detonation.event_id.event_number) {
        return false;
    }

    // Check if munition IDs match
    if (fire.munition_id != detonation.munition_id) {
        return false;
    }

    return true;
}

const char* detonation_result_to_string(DetonationResult result) {
    switch (result) {
        case DetonationResult::Other: return "Other";
        case DetonationResult::EntityImpact: return "EntityImpact";
        case DetonationResult::EntityProximateDetonation: return "EntityProximateDetonation";
        case DetonationResult::GroundImpact: return "GroundImpact";
        case DetonationResult::GroundProximateDetonation: return "GroundProximateDetonation";
        case DetonationResult::Detonation: return "Detonation";
        case DetonationResult::None: return "None";
        case DetonationResult::HE_Hit_Small: return "HE_Hit_Small";
        case DetonationResult::HE_Hit_Medium: return "HE_Hit_Medium";
        case DetonationResult::HE_Hit_Large: return "HE_Hit_Large";
        case DetonationResult::ArmorPiercingHit: return "ArmorPiercingHit";
        case DetonationResult::DirtBlastSmall: return "DirtBlastSmall";
        case DetonationResult::DirtBlastMedium: return "DirtBlastMedium";
        case DetonationResult::DirtBlastLarge: return "DirtBlastLarge";
        case DetonationResult::WaterBlastSmall: return "WaterBlastSmall";
        case DetonationResult::WaterBlastMedium: return "WaterBlastMedium";
        case DetonationResult::WaterBlastLarge: return "WaterBlastLarge";
        case DetonationResult::AirHit: return "AirHit";
        case DetonationResult::BuildingHitSmall: return "BuildingHitSmall";
        case DetonationResult::BuildingHitMedium: return "BuildingHitMedium";
        case DetonationResult::BuildingHitLarge: return "BuildingHitLarge";
        case DetonationResult::MineClearingLineCharge: return "MineClearingLineCharge";
        case DetonationResult::EnvironmentObjectImpact: return "EnvironmentObjectImpact";
        case DetonationResult::EnvironmentObjectProximateDetonation: return "EnvironmentObjectProximateDetonation";
        case DetonationResult::WaterImpact: return "WaterImpact";
        case DetonationResult::AirBurst: return "AirBurst";
        case DetonationResult::KillWithFragmentType1: return "KillWithFragmentType1";
        case DetonationResult::KillWithFragmentType2: return "KillWithFragmentType2";
        case DetonationResult::KillWithFragmentType3: return "KillWithFragmentType3";
        case DetonationResult::KillWithoutFragment: return "KillWithoutFragment";
        case DetonationResult::Miss: return "Miss";
        default: return "Unknown";
    }
}

bool is_entity_hit(DetonationResult result) {
    return result == DetonationResult::EntityImpact ||
           result == DetonationResult::EntityProximateDetonation ||
           result == DetonationResult::HE_Hit_Small ||
           result == DetonationResult::HE_Hit_Medium ||
           result == DetonationResult::HE_Hit_Large ||
           result == DetonationResult::ArmorPiercingHit ||
           result == DetonationResult::KillWithFragmentType1 ||
           result == DetonationResult::KillWithFragmentType2 ||
           result == DetonationResult::KillWithFragmentType3 ||
           result == DetonationResult::KillWithoutFragment;
}

bool is_ground_impact(DetonationResult result) {
    return result == DetonationResult::GroundImpact ||
           result == DetonationResult::GroundProximateDetonation ||
           result == DetonationResult::DirtBlastSmall ||
           result == DetonationResult::DirtBlastMedium ||
           result == DetonationResult::DirtBlastLarge;
}

Real calculate_damage_score(DetonationResult result) {
    switch (result) {
        // Direct hits - maximum damage
        case DetonationResult::EntityImpact:
        case DetonationResult::ArmorPiercingHit:
        case DetonationResult::KillWithFragmentType1:
        case DetonationResult::KillWithFragmentType2:
        case DetonationResult::KillWithFragmentType3:
        case DetonationResult::KillWithoutFragment:
            return 1.0;

        // High explosive hits - graduated damage
        case DetonationResult::HE_Hit_Large:
            return 0.9;
        case DetonationResult::HE_Hit_Medium:
            return 0.7;
        case DetonationResult::HE_Hit_Small:
            return 0.5;

        // Proximity detonations - moderate damage
        case DetonationResult::EntityProximateDetonation:
            return 0.4;

        // Building hits - depends on size
        case DetonationResult::BuildingHitLarge:
            return 0.8;
        case DetonationResult::BuildingHitMedium:
            return 0.6;
        case DetonationResult::BuildingHitSmall:
            return 0.4;

        // Environmental impacts - minimal to no damage on target
        case DetonationResult::GroundImpact:
        case DetonationResult::GroundProximateDetonation:
        case DetonationResult::WaterImpact:
        case DetonationResult::AirBurst:
        case DetonationResult::EnvironmentObjectImpact:
        case DetonationResult::EnvironmentObjectProximateDetonation:
            return 0.1;

        // Misses and duds - no damage
        case DetonationResult::Miss:
        case DetonationResult::None:
            return 0.0;

        default:
            return 0.0;
    }
}

} // namespace jaguar::federation
