/**
 * @file dis_protocol.cpp
 * @brief Implementation of IEEE 1278.1 DIS protocol
 */

#include "jaguar/federation/dis_protocol.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <chrono>
#include <queue>
#include <mutex>

namespace jaguar::federation {

// ============================================================================
// WGS84 Ellipsoid Constants
// ============================================================================

namespace {
    // WGS84 ellipsoid parameters
    constexpr Real WGS84_A = 6378137.0;              // Semi-major axis (meters)
    constexpr Real WGS84_F = 1.0 / 298.257223563;    // Flattening
    constexpr Real WGS84_B = WGS84_A * (1.0 - WGS84_F); // Semi-minor axis
    constexpr Real WGS84_E2 = 2.0 * WGS84_F - WGS84_F * WGS84_F; // Eccentricity squared

    // Time constants
    constexpr Real DIS_TIMESTAMP_SCALE = 3600.0 / static_cast<Real>(1ULL << 31);

    // Helper function to swap endianness for 16-bit values
    inline UInt16 swap_endian16(UInt16 val) {
        return ((val & 0xFF) << 8) | ((val >> 8) & 0xFF);
    }

    // Helper function to swap endianness for 32-bit values
    inline UInt32 swap_endian32(UInt32 val) {
        return ((val & 0xFF) << 24) |
               ((val & 0xFF00) << 8) |
               ((val & 0xFF0000) >> 8) |
               ((val >> 24) & 0xFF);
    }

    // Helper to write big-endian 16-bit value
    inline void write_be16(UInt8* buffer, UInt16 value) {
        #if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
            value = swap_endian16(value);
        #endif
        std::memcpy(buffer, &value, sizeof(UInt16));
    }

    // Helper to write big-endian 32-bit value
    inline void write_be32(UInt8* buffer, UInt32 value) {
        #if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
            value = swap_endian32(value);
        #endif
        std::memcpy(buffer, &value, sizeof(UInt32));
    }

    // Helper to read big-endian 16-bit value
    inline UInt16 read_be16(const UInt8* buffer) {
        UInt16 value;
        std::memcpy(&value, buffer, sizeof(UInt16));
        #if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
            value = swap_endian16(value);
        #endif
        return value;
    }

    // Helper to read big-endian 32-bit value
    inline UInt32 read_be32(const UInt8* buffer) {
        UInt32 value;
        std::memcpy(&value, buffer, sizeof(UInt32));
        #if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
            value = swap_endian32(value);
        #endif
        return value;
    }

    // Helper to write IEEE 754 float as big-endian
    inline void write_be_float(UInt8* buffer, Real value) {
        float f = static_cast<float>(value);
        UInt32 bits;
        std::memcpy(&bits, &f, sizeof(float));
        write_be32(buffer, bits);
    }

    // Helper to read IEEE 754 float from big-endian
    inline Real read_be_float(const UInt8* buffer) {
        UInt32 bits = read_be32(buffer);
        float f;
        std::memcpy(&f, &bits, sizeof(float));
        return static_cast<Real>(f);
    }

    // Helper to write IEEE 754 double as big-endian
    inline void write_be_double(UInt8* buffer, Real value) {
        double d = static_cast<double>(value);
        UInt64 bits;
        std::memcpy(&bits, &d, sizeof(double));
        // Swap bytes for little-endian systems
        #if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
            UInt64 swapped = 0;
            for (int i = 0; i < 8; ++i) {
                swapped |= ((bits >> (i * 8)) & 0xFF) << ((7 - i) * 8);
            }
            bits = swapped;
        #endif
        std::memcpy(buffer, &bits, sizeof(UInt64));
    }

    // Helper to read IEEE 754 double from big-endian
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
}

// ============================================================================
// Coordinate Transformation Functions
// ============================================================================

Vec3 lla_to_geocentric(const GeodeticCoordinates& lla) noexcept {
    Real sin_lat = std::sin(lla.latitude);
    Real cos_lat = std::cos(lla.latitude);
    Real sin_lon = std::sin(lla.longitude);
    Real cos_lon = std::cos(lla.longitude);

    // Calculate radius of curvature in prime vertical
    Real N = WGS84_A / std::sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat);

    // Calculate ECEF coordinates
    Real x = (N + lla.altitude) * cos_lat * cos_lon;
    Real y = (N + lla.altitude) * cos_lat * sin_lon;
    Real z = (N * (1.0 - WGS84_E2) + lla.altitude) * sin_lat;

    return Vec3(x, y, z);
}

GeodeticCoordinates geocentric_to_lla(const Vec3& ecef) noexcept {
    Real x = ecef.x;
    Real y = ecef.y;
    Real z = ecef.z;

    // Calculate longitude directly
    Real longitude = std::atan2(y, x);

    // Iterative algorithm for latitude
    Real p = std::sqrt(x * x + y * y);
    Real latitude = std::atan2(z, p * (1.0 - WGS84_E2));

    // Iterate to improve latitude accuracy
    for (int i = 0; i < 5; ++i) {
        Real sin_lat = std::sin(latitude);
        Real N = WGS84_A / std::sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat);
        Real altitude = p / std::cos(latitude) - N;
        latitude = std::atan2(z, p * (1.0 - WGS84_E2 * N / (N + altitude)));
    }

    Real sin_lat = std::sin(latitude);
    Real N = WGS84_A / std::sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat);
    Real altitude = p / std::cos(latitude) - N;

    return GeodeticCoordinates(latitude, longitude, altitude);
}

Quat euler_to_orientation(const EulerAngles& euler) noexcept {
    // DIS uses psi (yaw), theta (pitch), phi (roll) - Z-Y-X rotation order
    Real half_psi = euler.psi * 0.5;
    Real half_theta = euler.theta * 0.5;
    Real half_phi = euler.phi * 0.5;

    Real cos_psi = std::cos(half_psi);
    Real sin_psi = std::sin(half_psi);
    Real cos_theta = std::cos(half_theta);
    Real sin_theta = std::sin(half_theta);
    Real cos_phi = std::cos(half_phi);
    Real sin_phi = std::sin(half_phi);

    // Z-Y-X rotation order (yaw-pitch-roll)
    Real w = cos_psi * cos_theta * cos_phi + sin_psi * sin_theta * sin_phi;
    Real x = cos_psi * cos_theta * sin_phi - sin_psi * sin_theta * cos_phi;
    Real y = cos_psi * sin_theta * cos_phi + sin_psi * cos_theta * sin_phi;
    Real z = sin_psi * cos_theta * cos_phi - cos_psi * sin_theta * sin_phi;

    return Quat(w, x, y, z);
}

EulerAngles orientation_to_euler(const Quat& q) noexcept {
    // Convert quaternion to Euler angles (Z-Y-X order)
    Real w = q.w, x = q.x, y = q.y, z = q.z;

    // Roll (phi)
    Real sin_roll = 2.0 * (w * x + y * z);
    Real cos_roll = 1.0 - 2.0 * (x * x + y * y);
    Real phi = std::atan2(sin_roll, cos_roll);

    // Pitch (theta)
    Real sin_pitch = 2.0 * (w * y - z * x);
    sin_pitch = std::max(-1.0, std::min(1.0, sin_pitch)); // Clamp
    Real theta = std::asin(sin_pitch);

    // Yaw (psi)
    Real sin_yaw = 2.0 * (w * z + x * y);
    Real cos_yaw = 1.0 - 2.0 * (y * y + z * z);
    Real psi = std::atan2(sin_yaw, cos_yaw);

    return EulerAngles(psi, theta, phi);
}

// ============================================================================
// Dead Reckoning Calculator Implementation
// ============================================================================

EntityStatePDU DeadReckoningCalculator::extrapolate_position(
    const EntityStatePDU& state, Real elapsed_time) const noexcept {

    switch (state.dead_reckoning_params.algorithm) {
        case DeadReckoningAlgorithm::DRM_Static:
            return state;
        case DeadReckoningAlgorithm::DRM_FPW:
            return extrapolate_fpw(state, elapsed_time);
        case DeadReckoningAlgorithm::DRM_RPW:
            return extrapolate_rpw(state, elapsed_time);
        case DeadReckoningAlgorithm::DRM_RVW:
            return extrapolate_rvw(state, elapsed_time);
        case DeadReckoningAlgorithm::DRM_FVW:
            return extrapolate_fvw(state, elapsed_time);
        default:
            // Unsupported algorithm - return as-is
            return state;
    }
}

EntityStatePDU DeadReckoningCalculator::extrapolate_fpw(
    const EntityStatePDU& state, Real dt) const noexcept {
    // Fixed, Position, World - no motion
    return state;
}

EntityStatePDU DeadReckoningCalculator::extrapolate_rpw(
    const EntityStatePDU& state, Real dt) const noexcept {
    // Rotating, Position, World - rotate orientation only
    EntityStatePDU extrapolated = state;

    const Vec3& omega = state.dead_reckoning_params.angular_velocity;
    Real angle = omega.length() * dt;

    if (angle > 1e-6) {
        Vec3 axis = omega.normalized();
        Quat rotation = Quat::from_axis_angle(axis, angle);
        Quat current = euler_to_orientation(state.entity_orientation);
        Quat new_orientation = rotation * current;
        extrapolated.entity_orientation = orientation_to_euler(new_orientation);
    }

    return extrapolated;
}

EntityStatePDU DeadReckoningCalculator::extrapolate_rvw(
    const EntityStatePDU& state, Real dt) const noexcept {
    // Rotating, Velocity, World - linear motion + rotation
    EntityStatePDU extrapolated = state;

    // Linear motion: p = p0 + v*dt
    extrapolated.entity_location = state.entity_location + state.entity_linear_velocity * dt;

    // Rotation
    const Vec3& omega = state.dead_reckoning_params.angular_velocity;
    Real angle = omega.length() * dt;

    if (angle > 1e-6) {
        Vec3 axis = omega.normalized();
        Quat rotation = Quat::from_axis_angle(axis, angle);
        Quat current = euler_to_orientation(state.entity_orientation);
        Quat new_orientation = rotation * current;
        extrapolated.entity_orientation = orientation_to_euler(new_orientation);
    }

    return extrapolated;
}

EntityStatePDU DeadReckoningCalculator::extrapolate_fvw(
    const EntityStatePDU& state, Real dt) const noexcept {
    // Fixed, Velocity, World - linear motion only
    EntityStatePDU extrapolated = state;

    // Linear motion with acceleration: p = p0 + v*dt + 0.5*a*dt^2
    const Vec3& accel = state.dead_reckoning_params.linear_acceleration;
    extrapolated.entity_location = state.entity_location +
                                   state.entity_linear_velocity * dt +
                                   accel * (0.5 * dt * dt);

    // Update velocity: v = v0 + a*dt
    extrapolated.entity_linear_velocity = state.entity_linear_velocity + accel * dt;

    return extrapolated;
}

bool DeadReckoningCalculator::should_send_update(
    const EntityStatePDU& current,
    const EntityStatePDU& predicted,
    Real position_threshold,
    Real orientation_threshold) const noexcept {

    Real pos_error = calculate_position_error(current, predicted);
    Real orient_error = calculate_orientation_error(current, predicted);

    return (pos_error > position_threshold) || (orient_error > orientation_threshold);
}

Real DeadReckoningCalculator::calculate_position_error(
    const EntityStatePDU& a, const EntityStatePDU& b) const noexcept {
    return (a.entity_location - b.entity_location).length();
}

Real DeadReckoningCalculator::calculate_orientation_error(
    const EntityStatePDU& a, const EntityStatePDU& b) const noexcept {
    Quat qa = euler_to_orientation(a.entity_orientation);
    Quat qb = euler_to_orientation(b.entity_orientation);

    // Calculate angle between quaternions
    Real dot = qa.w * qb.w + qa.x * qb.x + qa.y * qb.y + qa.z * qb.z;
    dot = std::max(-1.0, std::min(1.0, dot)); // Clamp
    return std::acos(std::abs(dot)) * 2.0;
}

// ============================================================================
// DIS Codec Implementation
// ============================================================================

class DISCodecImpl : public IDISCodec {
public:
    DISCodecImpl() = default;
    ~DISCodecImpl() override = default;

    SizeT encode(const EntityStatePDU& pdu, UInt8* buffer) override {
        UInt8* ptr = buffer;

        // Encode PDU header
        ptr += encode_header(pdu.header, ptr);

        // Encode entity identifier
        ptr += encode_entity_id(pdu.entity_id, ptr);

        // Force ID and articulation count
        *ptr++ = static_cast<UInt8>(pdu.force_id);
        *ptr++ = pdu.num_articulation_params;

        // Entity type
        ptr += encode_entity_type(pdu.entity_type, ptr);
        ptr += encode_entity_type(pdu.alternative_entity_type, ptr);

        // Linear velocity (3 floats)
        write_be_float(ptr, pdu.entity_linear_velocity.x); ptr += 4;
        write_be_float(ptr, pdu.entity_linear_velocity.y); ptr += 4;
        write_be_float(ptr, pdu.entity_linear_velocity.z); ptr += 4;

        // Location (3 doubles)
        write_be_double(ptr, pdu.entity_location.x); ptr += 8;
        write_be_double(ptr, pdu.entity_location.y); ptr += 8;
        write_be_double(ptr, pdu.entity_location.z); ptr += 8;

        // Orientation (3 floats)
        write_be_float(ptr, pdu.entity_orientation.psi); ptr += 4;
        write_be_float(ptr, pdu.entity_orientation.theta); ptr += 4;
        write_be_float(ptr, pdu.entity_orientation.phi); ptr += 4;

        // Appearance
        write_be32(ptr, pdu.appearance.bits); ptr += 4;

        // Dead reckoning parameters
        ptr += encode_dead_reckoning(pdu.dead_reckoning_params, ptr);

        // Entity marking
        ptr += encode_marking(pdu.entity_marking, ptr);

        // Capabilities
        write_be32(ptr, pdu.capabilities.bits); ptr += 4;

        // Articulation parameters
        for (const auto& param : pdu.articulation_params) {
            ptr += encode_articulation(param, ptr);
        }

        // Update PDU length in header
        SizeT total_length = ptr - buffer;
        write_be16(buffer + 8, static_cast<UInt16>(total_length));

        return total_length;
    }

    std::optional<EntityStatePDU> decode_entity_state(const UInt8* buffer, SizeT length) override {
        if (length < 144) return std::nullopt; // Minimum EntityState PDU size

        EntityStatePDU pdu;
        const UInt8* ptr = buffer;

        // Decode header
        auto header = decode_header(buffer, length);
        if (!header || header->pdu_type != PDUType::EntityState) {
            return std::nullopt;
        }
        pdu.header = *header;
        ptr += 12;

        // Decode entity identifier
        pdu.entity_id = decode_entity_id(ptr);
        ptr += 6;

        // Force ID and articulation count
        pdu.force_id = static_cast<ForceId>(*ptr++);
        pdu.num_articulation_params = *ptr++;

        // Entity types
        pdu.entity_type = decode_entity_type(ptr); ptr += 8;
        pdu.alternative_entity_type = decode_entity_type(ptr); ptr += 8;

        // Linear velocity
        pdu.entity_linear_velocity.x = read_be_float(ptr); ptr += 4;
        pdu.entity_linear_velocity.y = read_be_float(ptr); ptr += 4;
        pdu.entity_linear_velocity.z = read_be_float(ptr); ptr += 4;

        // Location
        pdu.entity_location.x = read_be_double(ptr); ptr += 8;
        pdu.entity_location.y = read_be_double(ptr); ptr += 8;
        pdu.entity_location.z = read_be_double(ptr); ptr += 8;

        // Orientation
        pdu.entity_orientation.psi = read_be_float(ptr); ptr += 4;
        pdu.entity_orientation.theta = read_be_float(ptr); ptr += 4;
        pdu.entity_orientation.phi = read_be_float(ptr); ptr += 4;

        // Appearance
        pdu.appearance.bits = read_be32(ptr); ptr += 4;

        // Dead reckoning
        pdu.dead_reckoning_params = decode_dead_reckoning(ptr); ptr += 40;

        // Entity marking
        pdu.entity_marking = decode_marking(ptr); ptr += 12;

        // Capabilities
        pdu.capabilities.bits = read_be32(ptr); ptr += 4;

        // Articulation parameters
        for (UInt8 i = 0; i < pdu.num_articulation_params; ++i) {
            if (ptr + 16 > buffer + length) break;
            pdu.articulation_params.push_back(decode_articulation(ptr));
            ptr += 16;
        }

        return pdu;
    }

    SizeT encode(const FirePDU& pdu, UInt8* buffer) override {
        UInt8* ptr = buffer;

        ptr += encode_header(pdu.header, ptr);
        ptr += encode_entity_id(pdu.firing_entity_id, ptr);
        ptr += encode_entity_id(pdu.target_entity_id, ptr);
        ptr += encode_entity_id(pdu.munition_id, ptr);
        ptr += encode_event_id(pdu.event_id, ptr);

        write_be32(ptr, pdu.fire_mission_index); ptr += 4;

        write_be_double(ptr, pdu.location_in_world.x); ptr += 8;
        write_be_double(ptr, pdu.location_in_world.y); ptr += 8;
        write_be_double(ptr, pdu.location_in_world.z); ptr += 8;

        ptr += encode_burst_descriptor(pdu.burst_descriptor, ptr);

        write_be_float(ptr, pdu.velocity.x); ptr += 4;
        write_be_float(ptr, pdu.velocity.y); ptr += 4;
        write_be_float(ptr, pdu.velocity.z); ptr += 4;

        write_be_float(ptr, pdu.range); ptr += 4;

        SizeT total_length = ptr - buffer;
        write_be16(buffer + 8, static_cast<UInt16>(total_length));

        return total_length;
    }

    std::optional<FirePDU> decode_fire(const UInt8* buffer, SizeT length) override {
        if (length < 96) return std::nullopt;

        FirePDU pdu;
        const UInt8* ptr = buffer;

        auto header = decode_header(buffer, length);
        if (!header || header->pdu_type != PDUType::Fire) return std::nullopt;
        pdu.header = *header;
        ptr += 12;

        pdu.firing_entity_id = decode_entity_id(ptr); ptr += 6;
        pdu.target_entity_id = decode_entity_id(ptr); ptr += 6;
        pdu.munition_id = decode_entity_id(ptr); ptr += 6;
        pdu.event_id = decode_event_id(ptr); ptr += 6;

        pdu.fire_mission_index = read_be32(ptr); ptr += 4;

        pdu.location_in_world.x = read_be_double(ptr); ptr += 8;
        pdu.location_in_world.y = read_be_double(ptr); ptr += 8;
        pdu.location_in_world.z = read_be_double(ptr); ptr += 8;

        pdu.burst_descriptor = decode_burst_descriptor(ptr); ptr += 16;

        pdu.velocity.x = read_be_float(ptr); ptr += 4;
        pdu.velocity.y = read_be_float(ptr); ptr += 4;
        pdu.velocity.z = read_be_float(ptr); ptr += 4;

        pdu.range = read_be_float(ptr); ptr += 4;

        return pdu;
    }

    SizeT encode(const DetonationPDU& pdu, UInt8* buffer) override {
        UInt8* ptr = buffer;

        ptr += encode_header(pdu.header, ptr);
        ptr += encode_entity_id(pdu.firing_entity_id, ptr);
        ptr += encode_entity_id(pdu.target_entity_id, ptr);
        ptr += encode_entity_id(pdu.munition_id, ptr);
        ptr += encode_event_id(pdu.event_id, ptr);

        write_be_float(ptr, pdu.velocity.x); ptr += 4;
        write_be_float(ptr, pdu.velocity.y); ptr += 4;
        write_be_float(ptr, pdu.velocity.z); ptr += 4;

        write_be_double(ptr, pdu.location_in_world.x); ptr += 8;
        write_be_double(ptr, pdu.location_in_world.y); ptr += 8;
        write_be_double(ptr, pdu.location_in_world.z); ptr += 8;

        ptr += encode_burst_descriptor(pdu.burst_descriptor, ptr);

        write_be_float(ptr, pdu.location_in_entity.x); ptr += 4;
        write_be_float(ptr, pdu.location_in_entity.y); ptr += 4;
        write_be_float(ptr, pdu.location_in_entity.z); ptr += 4;

        *ptr++ = static_cast<UInt8>(pdu.detonation_result);
        *ptr++ = pdu.num_articulation_params;
        write_be16(ptr, pdu.padding); ptr += 2;

        for (const auto& param : pdu.articulation_params) {
            ptr += encode_articulation(param, ptr);
        }

        SizeT total_length = ptr - buffer;
        write_be16(buffer + 8, static_cast<UInt16>(total_length));

        return total_length;
    }

    std::optional<DetonationPDU> decode_detonation(const UInt8* buffer, SizeT length) override {
        if (length < 104) return std::nullopt;

        DetonationPDU pdu;
        const UInt8* ptr = buffer;

        auto header = decode_header(buffer, length);
        if (!header || header->pdu_type != PDUType::Detonation) return std::nullopt;
        pdu.header = *header;
        ptr += 12;

        pdu.firing_entity_id = decode_entity_id(ptr); ptr += 6;
        pdu.target_entity_id = decode_entity_id(ptr); ptr += 6;
        pdu.munition_id = decode_entity_id(ptr); ptr += 6;
        pdu.event_id = decode_event_id(ptr); ptr += 6;

        pdu.velocity.x = read_be_float(ptr); ptr += 4;
        pdu.velocity.y = read_be_float(ptr); ptr += 4;
        pdu.velocity.z = read_be_float(ptr); ptr += 4;

        pdu.location_in_world.x = read_be_double(ptr); ptr += 8;
        pdu.location_in_world.y = read_be_double(ptr); ptr += 8;
        pdu.location_in_world.z = read_be_double(ptr); ptr += 8;

        pdu.burst_descriptor = decode_burst_descriptor(ptr); ptr += 16;

        pdu.location_in_entity.x = read_be_float(ptr); ptr += 4;
        pdu.location_in_entity.y = read_be_float(ptr); ptr += 4;
        pdu.location_in_entity.z = read_be_float(ptr); ptr += 4;

        pdu.detonation_result = static_cast<DetonationResult>(*ptr++);
        pdu.num_articulation_params = *ptr++;
        pdu.padding = read_be16(ptr); ptr += 2;

        for (UInt8 i = 0; i < pdu.num_articulation_params; ++i) {
            if (ptr + 16 > buffer + length) break;
            pdu.articulation_params.push_back(decode_articulation(ptr));
            ptr += 16;
        }

        return pdu;
    }

    std::optional<PDUHeader> decode_header(const UInt8* buffer, SizeT length) override {
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

private:
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

    SizeT encode_dead_reckoning(const DeadReckoningParameters& params, UInt8* buffer) {
        buffer[0] = static_cast<UInt8>(params.algorithm);
        std::memcpy(buffer + 1, params.other_parameters, 15);

        write_be_float(buffer + 16, params.linear_acceleration.x);
        write_be_float(buffer + 20, params.linear_acceleration.y);
        write_be_float(buffer + 24, params.linear_acceleration.z);

        write_be_float(buffer + 28, params.angular_velocity.x);
        write_be_float(buffer + 32, params.angular_velocity.y);
        write_be_float(buffer + 36, params.angular_velocity.z);

        return 40;
    }

    DeadReckoningParameters decode_dead_reckoning(const UInt8* buffer) {
        DeadReckoningParameters params;
        params.algorithm = static_cast<DeadReckoningAlgorithm>(buffer[0]);
        std::memcpy(params.other_parameters, buffer + 1, 15);

        params.linear_acceleration.x = read_be_float(buffer + 16);
        params.linear_acceleration.y = read_be_float(buffer + 20);
        params.linear_acceleration.z = read_be_float(buffer + 24);

        params.angular_velocity.x = read_be_float(buffer + 28);
        params.angular_velocity.y = read_be_float(buffer + 32);
        params.angular_velocity.z = read_be_float(buffer + 36);

        return params;
    }

    SizeT encode_marking(const EntityMarking& marking, UInt8* buffer) {
        buffer[0] = marking.character_set;
        std::memcpy(buffer + 1, marking.characters, DIS_MARKING_LENGTH);
        return 12;
    }

    EntityMarking decode_marking(const UInt8* buffer) {
        EntityMarking marking;
        marking.character_set = buffer[0];
        std::memcpy(marking.characters, buffer + 1, DIS_MARKING_LENGTH);
        return marking;
    }

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
};

// ============================================================================
// DIS Network Implementation (Stub)
// ============================================================================

class DISNetworkImpl : public IDISNetwork {
public:
    DISNetworkImpl()
        : codec_(std::make_unique<DISCodecImpl>()),
          site_id_(0),
          application_id_(0),
          next_entity_number_(1) {}

    ~DISNetworkImpl() override = default;

    bool initialize(const std::string& multicast_address,
                   UInt16 port,
                   const std::string& interface_address) override {
        std::lock_guard<std::mutex> lock(mutex_);

        multicast_address_ = multicast_address;
        port_ = port;
        interface_address_ = interface_address;
        initialized_ = true;

        // TODO: Actual socket initialization would go here
        // For now, stub implementation uses in-memory queue

        return true;
    }

    void shutdown() override {
        std::lock_guard<std::mutex> lock(mutex_);

        send_queue_.clear();
        receive_queue_.clear();
        initialized_ = false;
    }

    bool send_pdu(const EntityStatePDU& pdu) override {
        std::lock_guard<std::mutex> lock(mutex_);

        if (!initialized_) return false;

        UInt8 buffer[DIS_MAX_PDU_SIZE];
        SizeT length = codec_->encode(pdu, buffer);

        if (length > 0) {
            std::vector<UInt8> packet(buffer, buffer + length);
            send_queue_.push_back(packet);
            return true;
        }

        return false;
    }

    bool send_pdu(const FirePDU& pdu) override {
        std::lock_guard<std::mutex> lock(mutex_);

        if (!initialized_) return false;

        UInt8 buffer[DIS_MAX_PDU_SIZE];
        SizeT length = codec_->encode(pdu, buffer);

        if (length > 0) {
            std::vector<UInt8> packet(buffer, buffer + length);
            send_queue_.push_back(packet);
            return true;
        }

        return false;
    }

    bool send_pdu(const DetonationPDU& pdu) override {
        std::lock_guard<std::mutex> lock(mutex_);

        if (!initialized_) return false;

        UInt8 buffer[DIS_MAX_PDU_SIZE];
        SizeT length = codec_->encode(pdu, buffer);

        if (length > 0) {
            std::vector<UInt8> packet(buffer, buffer + length);
            send_queue_.push_back(packet);
            return true;
        }

        return false;
    }

    SizeT receive_pdu(UInt8* buffer, SizeT buffer_size) override {
        std::lock_guard<std::mutex> lock(mutex_);

        if (!initialized_ || receive_queue_.empty()) {
            return 0;
        }

        const auto& packet = receive_queue_.front();
        SizeT copy_size = std::min(packet.size(), buffer_size);
        std::memcpy(buffer, packet.data(), copy_size);

        receive_queue_.pop_front();

        return copy_size;
    }

    void join_exercise(UInt16 site_id, UInt16 application_id) override {
        std::lock_guard<std::mutex> lock(mutex_);
        site_id_ = site_id;
        application_id_ = application_id;
        next_entity_number_ = 1;
    }

    void leave_exercise() override {
        std::lock_guard<std::mutex> lock(mutex_);
        // In real implementation, send RemoveEntity PDUs for all entities
        site_id_ = 0;
        application_id_ = 0;
        next_entity_number_ = 1;
    }

    UInt16 get_site_id() const override {
        return site_id_;
    }

    UInt16 get_application_id() const override {
        return application_id_;
    }

    UInt16 allocate_entity_id() override {
        std::lock_guard<std::mutex> lock(mutex_);
        return next_entity_number_++;
    }

private:
    std::unique_ptr<DISCodecImpl> codec_;
    std::string multicast_address_;
    UInt16 port_;
    std::string interface_address_;
    bool initialized_{false};

    UInt16 site_id_;
    UInt16 application_id_;
    UInt16 next_entity_number_;

    // Stub implementation: in-memory queues for testing
    std::deque<std::vector<UInt8>> send_queue_;
    std::deque<std::vector<UInt8>> receive_queue_;

    mutable std::mutex mutex_;
};

// ============================================================================
// Utility Functions
// ============================================================================

UInt32 get_dis_timestamp() noexcept {
    using namespace std::chrono;

    auto now = system_clock::now();
    auto duration = now.time_since_epoch();
    auto hours = duration_cast<std::chrono::hours>(duration);
    auto remainder = duration - hours;

    // Convert remainder to DIS timestamp units
    double seconds = std::chrono::duration_cast<std::chrono::duration<double>>(remainder).count();
    UInt32 timestamp = static_cast<UInt32>(seconds / DIS_TIMESTAMP_SCALE);

    return timestamp;
}

Real dis_timestamp_to_seconds(UInt32 timestamp) noexcept {
    return static_cast<Real>(timestamp) * DIS_TIMESTAMP_SCALE;
}

UInt32 seconds_to_dis_timestamp(Real seconds) noexcept {
    return static_cast<UInt32>(seconds / DIS_TIMESTAMP_SCALE);
}

Real distance_lla(const GeodeticCoordinates& a, const GeodeticCoordinates& b) noexcept {
    // Haversine formula for great circle distance
    Real dlat = b.latitude - a.latitude;
    Real dlon = b.longitude - a.longitude;

    Real sin_dlat_2 = std::sin(dlat * 0.5);
    Real sin_dlon_2 = std::sin(dlon * 0.5);

    Real aa = sin_dlat_2 * sin_dlat_2 +
             std::cos(a.latitude) * std::cos(b.latitude) *
             sin_dlon_2 * sin_dlon_2;

    Real c = 2.0 * std::atan2(std::sqrt(aa), std::sqrt(1.0 - aa));

    // Use average of altitudes for radius
    Real avg_altitude = (a.altitude + b.altitude) * 0.5;
    Real radius = WGS84_A + avg_altitude;

    return radius * c;
}

// ============================================================================
// Factory Functions
// ============================================================================

std::unique_ptr<IDISCodec> create_dis_codec() {
    return std::make_unique<DISCodecImpl>();
}

std::unique_ptr<IDISNetwork> create_dis_network() {
    return std::make_unique<DISNetworkImpl>();
}

} // namespace jaguar::federation
