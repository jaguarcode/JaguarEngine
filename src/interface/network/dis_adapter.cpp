/**
 * @file dis_adapter.cpp
 * @brief DIS protocol adapter implementation
 *
 * Implements IEEE 1278 DIS protocol for networked simulation.
 */

#include "jaguar/interface/dis.h"
#include "jaguar/core/types.h"

#include <unordered_map>
#include <mutex>
#include <cstring>
#include <cmath>
#include <chrono>

#ifdef _WIN32
    #include <winsock2.h>
    #include <ws2tcpip.h>
    #pragma comment(lib, "ws2_32.lib")
#else
    #include <sys/socket.h>
    #include <netinet/in.h>
    #include <arpa/inet.h>
    #include <unistd.h>
    #include <fcntl.h>
    #include <errno.h>
#endif

namespace jaguar::interface::dis {

// ============================================================================
// Platform Abstraction
// ============================================================================

#ifdef _WIN32
using SocketType = SOCKET;
constexpr SocketType INVALID_SOCKET_VALUE = INVALID_SOCKET;
#else
using SocketType = int;
constexpr SocketType INVALID_SOCKET_VALUE = -1;
#endif

inline void close_socket(SocketType sock) {
#ifdef _WIN32
    closesocket(sock);
#else
    close(sock);
#endif
}

inline bool set_nonblocking(SocketType sock) {
#ifdef _WIN32
    u_long mode = 1;
    return ioctlsocket(sock, FIONBIO, &mode) == 0;
#else
    int flags = fcntl(sock, F_GETFL, 0);
    return fcntl(sock, F_SETFL, flags | O_NONBLOCK) == 0;
#endif
}

// ============================================================================
// Hash Functions
// ============================================================================

struct EntityIdentifierHash {
    size_t operator()(const EntityIdentifier& id) const {
        return std::hash<uint64_t>{}(
            (static_cast<uint64_t>(id.site_id) << 32) |
            (static_cast<uint64_t>(id.application_id) << 16) |
            static_cast<uint64_t>(id.entity_id));
    }
};

// ============================================================================
// PDU Serialization
// ============================================================================

namespace serialize {

// Network byte order helpers
inline uint16_t hton16(uint16_t val) { return htons(val); }
inline uint32_t hton32(uint32_t val) { return htonl(val); }
inline uint16_t ntoh16(uint16_t val) { return ntohs(val); }
inline uint32_t ntoh32(uint32_t val) { return ntohl(val); }

inline uint64_t hton64(uint64_t val) {
    return (static_cast<uint64_t>(htonl(val & 0xFFFFFFFF)) << 32) |
           htonl(val >> 32);
}

inline uint64_t ntoh64(uint64_t val) {
    return (static_cast<uint64_t>(ntohl(val & 0xFFFFFFFF)) << 32) |
           ntohl(val >> 32);
}

inline float hton_float(float val) {
    uint32_t* p = reinterpret_cast<uint32_t*>(&val);
    uint32_t n = hton32(*p);
    return *reinterpret_cast<float*>(&n);
}

inline float ntoh_float(float val) {
    uint32_t* p = reinterpret_cast<uint32_t*>(&val);
    uint32_t n = ntoh32(*p);
    return *reinterpret_cast<float*>(&n);
}

inline double hton_double(double val) {
    uint64_t* p = reinterpret_cast<uint64_t*>(&val);
    uint64_t n = hton64(*p);
    return *reinterpret_cast<double*>(&n);
}

inline double ntoh_double(double val) {
    uint64_t* p = reinterpret_cast<uint64_t*>(&val);
    uint64_t n = ntoh64(*p);
    return *reinterpret_cast<double*>(&n);
}

/**
 * @brief Serialize Entity State PDU to buffer
 */
size_t serialize_entity_state(const EntityStatePdu& pdu, uint8_t* buffer, size_t max_size) {
    if (max_size < 144) return 0;  // Minimum size without articulation params

    size_t offset = 0;

    // Header (12 bytes)
    buffer[offset++] = static_cast<uint8_t>(pdu.header.protocol_version);
    buffer[offset++] = pdu.header.exercise_id;
    buffer[offset++] = static_cast<uint8_t>(pdu.header.pdu_type);
    buffer[offset++] = static_cast<uint8_t>(pdu.header.protocol_family);

    uint32_t timestamp = hton32(pdu.header.timestamp);
    std::memcpy(buffer + offset, &timestamp, 4);
    offset += 4;

    uint16_t pdu_length = hton16(pdu.header.pdu_length);
    std::memcpy(buffer + offset, &pdu_length, 2);
    offset += 2;

    uint16_t padding = 0;
    std::memcpy(buffer + offset, &padding, 2);
    offset += 2;

    // Entity ID (6 bytes)
    uint16_t site_id = hton16(pdu.entity_id.site_id);
    uint16_t app_id = hton16(pdu.entity_id.application_id);
    uint16_t ent_id = hton16(pdu.entity_id.entity_id);
    std::memcpy(buffer + offset, &site_id, 2); offset += 2;
    std::memcpy(buffer + offset, &app_id, 2); offset += 2;
    std::memcpy(buffer + offset, &ent_id, 2); offset += 2;

    // Force ID and articulation params count (2 bytes)
    buffer[offset++] = static_cast<uint8_t>(pdu.force_id);
    buffer[offset++] = pdu.num_articulation_params;

    // Entity Type (8 bytes)
    buffer[offset++] = pdu.entity_type.kind;
    buffer[offset++] = pdu.entity_type.domain;
    uint16_t country = hton16(pdu.entity_type.country);
    std::memcpy(buffer + offset, &country, 2); offset += 2;
    buffer[offset++] = pdu.entity_type.category;
    buffer[offset++] = pdu.entity_type.subcategory;
    buffer[offset++] = pdu.entity_type.specific;
    buffer[offset++] = pdu.entity_type.extra;

    // Alternative Entity Type (8 bytes)
    buffer[offset++] = pdu.alternative_entity_type.kind;
    buffer[offset++] = pdu.alternative_entity_type.domain;
    country = hton16(pdu.alternative_entity_type.country);
    std::memcpy(buffer + offset, &country, 2); offset += 2;
    buffer[offset++] = pdu.alternative_entity_type.category;
    buffer[offset++] = pdu.alternative_entity_type.subcategory;
    buffer[offset++] = pdu.alternative_entity_type.specific;
    buffer[offset++] = pdu.alternative_entity_type.extra;

    // Linear Velocity (12 bytes)
    float vx = hton_float(pdu.linear_velocity.x);
    float vy = hton_float(pdu.linear_velocity.y);
    float vz = hton_float(pdu.linear_velocity.z);
    std::memcpy(buffer + offset, &vx, 4); offset += 4;
    std::memcpy(buffer + offset, &vy, 4); offset += 4;
    std::memcpy(buffer + offset, &vz, 4); offset += 4;

    // Location (24 bytes - double precision)
    double lx = hton_double(pdu.location.x);
    double ly = hton_double(pdu.location.y);
    double lz = hton_double(pdu.location.z);
    std::memcpy(buffer + offset, &lx, 8); offset += 8;
    std::memcpy(buffer + offset, &ly, 8); offset += 8;
    std::memcpy(buffer + offset, &lz, 8); offset += 8;

    // Orientation (12 bytes)
    float psi = hton_float(pdu.orientation.psi);
    float theta = hton_float(pdu.orientation.theta);
    float phi = hton_float(pdu.orientation.phi);
    std::memcpy(buffer + offset, &psi, 4); offset += 4;
    std::memcpy(buffer + offset, &theta, 4); offset += 4;
    std::memcpy(buffer + offset, &phi, 4); offset += 4;

    // Appearance (4 bytes)
    uint32_t appearance = hton32(pdu.appearance);
    std::memcpy(buffer + offset, &appearance, 4); offset += 4;

    // Dead Reckoning Parameters (40 bytes)
    buffer[offset++] = static_cast<uint8_t>(pdu.dead_reckoning.algorithm);
    std::memcpy(buffer + offset, pdu.dead_reckoning.other_params, 15);
    offset += 15;

    float ax = hton_float(pdu.dead_reckoning.linear_acceleration.x);
    float ay = hton_float(pdu.dead_reckoning.linear_acceleration.y);
    float az = hton_float(pdu.dead_reckoning.linear_acceleration.z);
    std::memcpy(buffer + offset, &ax, 4); offset += 4;
    std::memcpy(buffer + offset, &ay, 4); offset += 4;
    std::memcpy(buffer + offset, &az, 4); offset += 4;

    float avx = hton_float(pdu.dead_reckoning.angular_velocity.x);
    float avy = hton_float(pdu.dead_reckoning.angular_velocity.y);
    float avz = hton_float(pdu.dead_reckoning.angular_velocity.z);
    std::memcpy(buffer + offset, &avx, 4); offset += 4;
    std::memcpy(buffer + offset, &avy, 4); offset += 4;
    std::memcpy(buffer + offset, &avz, 4); offset += 4;

    // Entity Marking (12 bytes)
    buffer[offset++] = pdu.marking.character_set;
    std::memcpy(buffer + offset, pdu.marking.characters, 11);
    offset += 11;

    // Capabilities (4 bytes)
    uint32_t capabilities = hton32(pdu.capabilities);
    std::memcpy(buffer + offset, &capabilities, 4); offset += 4;

    // Articulation parameters (variable)
    for (const auto& ap : pdu.articulation_params) {
        if (offset + 16 > max_size) break;

        buffer[offset++] = ap.parameter_type_designator;
        buffer[offset++] = ap.change_indicator;
        uint16_t attach_id = hton16(ap.articulation_attachment_id);
        std::memcpy(buffer + offset, &attach_id, 2); offset += 2;

        uint32_t param_type = hton32(ap.parameter_type);
        std::memcpy(buffer + offset, &param_type, 4); offset += 4;

        uint64_t param_value = hton64(ap.parameter_value);
        std::memcpy(buffer + offset, &param_value, 8); offset += 8;
    }

    return offset;
}

/**
 * @brief Deserialize Entity State PDU from buffer
 */
bool deserialize_entity_state(const uint8_t* buffer, size_t size, EntityStatePdu& pdu) {
    if (size < 144) return false;

    size_t offset = 0;

    // Header
    pdu.header.protocol_version = static_cast<ProtocolVersion>(buffer[offset++]);
    pdu.header.exercise_id = buffer[offset++];
    pdu.header.pdu_type = static_cast<PduType>(buffer[offset++]);
    pdu.header.protocol_family = static_cast<ProtocolFamily>(buffer[offset++]);

    uint32_t timestamp;
    std::memcpy(&timestamp, buffer + offset, 4);
    pdu.header.timestamp = ntoh32(timestamp);
    offset += 4;

    uint16_t pdu_length;
    std::memcpy(&pdu_length, buffer + offset, 2);
    pdu.header.pdu_length = ntoh16(pdu_length);
    offset += 2;

    offset += 2; // padding

    // Entity ID
    uint16_t site_id, app_id, ent_id;
    std::memcpy(&site_id, buffer + offset, 2); offset += 2;
    std::memcpy(&app_id, buffer + offset, 2); offset += 2;
    std::memcpy(&ent_id, buffer + offset, 2); offset += 2;
    pdu.entity_id.site_id = ntoh16(site_id);
    pdu.entity_id.application_id = ntoh16(app_id);
    pdu.entity_id.entity_id = ntoh16(ent_id);

    // Force ID and articulation count
    pdu.force_id = static_cast<ForceId>(buffer[offset++]);
    pdu.num_articulation_params = buffer[offset++];

    // Entity Type
    pdu.entity_type.kind = buffer[offset++];
    pdu.entity_type.domain = buffer[offset++];
    uint16_t country;
    std::memcpy(&country, buffer + offset, 2);
    pdu.entity_type.country = ntoh16(country);
    offset += 2;
    pdu.entity_type.category = buffer[offset++];
    pdu.entity_type.subcategory = buffer[offset++];
    pdu.entity_type.specific = buffer[offset++];
    pdu.entity_type.extra = buffer[offset++];

    // Alternative Entity Type
    pdu.alternative_entity_type.kind = buffer[offset++];
    pdu.alternative_entity_type.domain = buffer[offset++];
    std::memcpy(&country, buffer + offset, 2);
    pdu.alternative_entity_type.country = ntoh16(country);
    offset += 2;
    pdu.alternative_entity_type.category = buffer[offset++];
    pdu.alternative_entity_type.subcategory = buffer[offset++];
    pdu.alternative_entity_type.specific = buffer[offset++];
    pdu.alternative_entity_type.extra = buffer[offset++];

    // Linear Velocity
    float vx, vy, vz;
    std::memcpy(&vx, buffer + offset, 4); offset += 4;
    std::memcpy(&vy, buffer + offset, 4); offset += 4;
    std::memcpy(&vz, buffer + offset, 4); offset += 4;
    pdu.linear_velocity.x = ntoh_float(vx);
    pdu.linear_velocity.y = ntoh_float(vy);
    pdu.linear_velocity.z = ntoh_float(vz);

    // Location
    double lx, ly, lz;
    std::memcpy(&lx, buffer + offset, 8); offset += 8;
    std::memcpy(&ly, buffer + offset, 8); offset += 8;
    std::memcpy(&lz, buffer + offset, 8); offset += 8;
    pdu.location.x = ntoh_double(lx);
    pdu.location.y = ntoh_double(ly);
    pdu.location.z = ntoh_double(lz);

    // Orientation
    float psi, theta, phi;
    std::memcpy(&psi, buffer + offset, 4); offset += 4;
    std::memcpy(&theta, buffer + offset, 4); offset += 4;
    std::memcpy(&phi, buffer + offset, 4); offset += 4;
    pdu.orientation.psi = ntoh_float(psi);
    pdu.orientation.theta = ntoh_float(theta);
    pdu.orientation.phi = ntoh_float(phi);

    // Appearance
    uint32_t appearance;
    std::memcpy(&appearance, buffer + offset, 4);
    pdu.appearance = ntoh32(appearance);
    offset += 4;

    // Dead Reckoning
    pdu.dead_reckoning.algorithm = static_cast<DeadReckoningAlgorithm>(buffer[offset++]);
    std::memcpy(pdu.dead_reckoning.other_params, buffer + offset, 15);
    offset += 15;

    float ax, ay, az;
    std::memcpy(&ax, buffer + offset, 4); offset += 4;
    std::memcpy(&ay, buffer + offset, 4); offset += 4;
    std::memcpy(&az, buffer + offset, 4); offset += 4;
    pdu.dead_reckoning.linear_acceleration.x = ntoh_float(ax);
    pdu.dead_reckoning.linear_acceleration.y = ntoh_float(ay);
    pdu.dead_reckoning.linear_acceleration.z = ntoh_float(az);

    float avx, avy, avz;
    std::memcpy(&avx, buffer + offset, 4); offset += 4;
    std::memcpy(&avy, buffer + offset, 4); offset += 4;
    std::memcpy(&avz, buffer + offset, 4); offset += 4;
    pdu.dead_reckoning.angular_velocity.x = ntoh_float(avx);
    pdu.dead_reckoning.angular_velocity.y = ntoh_float(avy);
    pdu.dead_reckoning.angular_velocity.z = ntoh_float(avz);

    // Entity Marking
    pdu.marking.character_set = buffer[offset++];
    std::memcpy(pdu.marking.characters, buffer + offset, 11);
    offset += 11;

    // Capabilities
    uint32_t capabilities;
    std::memcpy(&capabilities, buffer + offset, 4);
    pdu.capabilities = ntoh32(capabilities);
    offset += 4;

    // Articulation parameters
    pdu.articulation_params.clear();
    for (uint8_t i = 0; i < pdu.num_articulation_params && offset + 16 <= size; ++i) {
        ArticulationParameter ap;
        ap.parameter_type_designator = buffer[offset++];
        ap.change_indicator = buffer[offset++];

        uint16_t attach_id;
        std::memcpy(&attach_id, buffer + offset, 2);
        ap.articulation_attachment_id = ntoh16(attach_id);
        offset += 2;

        uint32_t param_type;
        std::memcpy(&param_type, buffer + offset, 4);
        ap.parameter_type = ntoh32(param_type);
        offset += 4;

        uint64_t param_value;
        std::memcpy(&param_value, buffer + offset, 8);
        ap.parameter_value = ntoh64(param_value);
        offset += 8;

        pdu.articulation_params.push_back(ap);
    }

    return true;
}

/**
 * @brief Serialize Fire PDU to buffer
 */
size_t serialize_fire(const FirePdu& pdu, uint8_t* buffer, size_t max_size) {
    if (max_size < 96) return 0;

    size_t offset = 0;

    // Header (12 bytes)
    buffer[offset++] = static_cast<uint8_t>(pdu.header.protocol_version);
    buffer[offset++] = pdu.header.exercise_id;
    buffer[offset++] = static_cast<uint8_t>(pdu.header.pdu_type);
    buffer[offset++] = static_cast<uint8_t>(pdu.header.protocol_family);

    uint32_t timestamp = hton32(pdu.header.timestamp);
    std::memcpy(buffer + offset, &timestamp, 4);
    offset += 4;

    uint16_t pdu_length = hton16(pdu.header.pdu_length);
    std::memcpy(buffer + offset, &pdu_length, 2);
    offset += 2;

    uint16_t padding = 0;
    std::memcpy(buffer + offset, &padding, 2);
    offset += 2;

    // Firing Entity ID (6 bytes)
    uint16_t site_id = hton16(pdu.firing_entity_id.site_id);
    uint16_t app_id = hton16(pdu.firing_entity_id.application_id);
    uint16_t ent_id = hton16(pdu.firing_entity_id.entity_id);
    std::memcpy(buffer + offset, &site_id, 2); offset += 2;
    std::memcpy(buffer + offset, &app_id, 2); offset += 2;
    std::memcpy(buffer + offset, &ent_id, 2); offset += 2;

    // Target Entity ID (6 bytes)
    site_id = hton16(pdu.target_entity_id.site_id);
    app_id = hton16(pdu.target_entity_id.application_id);
    ent_id = hton16(pdu.target_entity_id.entity_id);
    std::memcpy(buffer + offset, &site_id, 2); offset += 2;
    std::memcpy(buffer + offset, &app_id, 2); offset += 2;
    std::memcpy(buffer + offset, &ent_id, 2); offset += 2;

    // Munition ID (6 bytes)
    site_id = hton16(pdu.munition_id.site_id);
    app_id = hton16(pdu.munition_id.application_id);
    ent_id = hton16(pdu.munition_id.entity_id);
    std::memcpy(buffer + offset, &site_id, 2); offset += 2;
    std::memcpy(buffer + offset, &app_id, 2); offset += 2;
    std::memcpy(buffer + offset, &ent_id, 2); offset += 2;

    // Event ID (6 bytes)
    site_id = hton16(pdu.event_id.site_id);
    app_id = hton16(pdu.event_id.application_id);
    uint16_t event_num = hton16(pdu.event_id.event_number);
    std::memcpy(buffer + offset, &site_id, 2); offset += 2;
    std::memcpy(buffer + offset, &app_id, 2); offset += 2;
    std::memcpy(buffer + offset, &event_num, 2); offset += 2;

    // Fire Mission Index (4 bytes)
    uint32_t fire_mission = hton32(pdu.fire_mission_index);
    std::memcpy(buffer + offset, &fire_mission, 4); offset += 4;

    // Location (24 bytes)
    double lx = hton_double(pdu.location_in_world.x);
    double ly = hton_double(pdu.location_in_world.y);
    double lz = hton_double(pdu.location_in_world.z);
    std::memcpy(buffer + offset, &lx, 8); offset += 8;
    std::memcpy(buffer + offset, &ly, 8); offset += 8;
    std::memcpy(buffer + offset, &lz, 8); offset += 8;

    // Burst Descriptor (16 bytes)
    buffer[offset++] = pdu.burst_descriptor.munition.kind;
    buffer[offset++] = pdu.burst_descriptor.munition.domain;
    uint16_t country = hton16(pdu.burst_descriptor.munition.country);
    std::memcpy(buffer + offset, &country, 2); offset += 2;
    buffer[offset++] = pdu.burst_descriptor.munition.category;
    buffer[offset++] = pdu.burst_descriptor.munition.subcategory;
    buffer[offset++] = pdu.burst_descriptor.munition.specific;
    buffer[offset++] = pdu.burst_descriptor.munition.extra;

    uint16_t warhead = hton16(pdu.burst_descriptor.warhead);
    uint16_t fuse = hton16(pdu.burst_descriptor.fuse);
    uint16_t quantity = hton16(pdu.burst_descriptor.quantity);
    uint16_t rate = hton16(pdu.burst_descriptor.rate);
    std::memcpy(buffer + offset, &warhead, 2); offset += 2;
    std::memcpy(buffer + offset, &fuse, 2); offset += 2;
    std::memcpy(buffer + offset, &quantity, 2); offset += 2;
    std::memcpy(buffer + offset, &rate, 2); offset += 2;

    // Velocity (12 bytes)
    float vx = hton_float(pdu.velocity.x);
    float vy = hton_float(pdu.velocity.y);
    float vz = hton_float(pdu.velocity.z);
    std::memcpy(buffer + offset, &vx, 4); offset += 4;
    std::memcpy(buffer + offset, &vy, 4); offset += 4;
    std::memcpy(buffer + offset, &vz, 4); offset += 4;

    // Range (4 bytes)
    float range = hton_float(pdu.range);
    std::memcpy(buffer + offset, &range, 4); offset += 4;

    return offset;
}

/**
 * @brief Serialize Detonation PDU to buffer
 */
size_t serialize_detonation(const DetonationPdu& pdu, uint8_t* buffer, size_t max_size) {
    if (max_size < 104) return 0;

    size_t offset = 0;

    // Header (12 bytes)
    buffer[offset++] = static_cast<uint8_t>(pdu.header.protocol_version);
    buffer[offset++] = pdu.header.exercise_id;
    buffer[offset++] = static_cast<uint8_t>(pdu.header.pdu_type);
    buffer[offset++] = static_cast<uint8_t>(pdu.header.protocol_family);

    uint32_t timestamp = hton32(pdu.header.timestamp);
    std::memcpy(buffer + offset, &timestamp, 4);
    offset += 4;

    uint16_t pdu_length = hton16(pdu.header.pdu_length);
    std::memcpy(buffer + offset, &pdu_length, 2);
    offset += 2;

    uint16_t padding = 0;
    std::memcpy(buffer + offset, &padding, 2);
    offset += 2;

    // Entity IDs (18 bytes)
    uint16_t site_id = hton16(pdu.firing_entity_id.site_id);
    uint16_t app_id = hton16(pdu.firing_entity_id.application_id);
    uint16_t ent_id = hton16(pdu.firing_entity_id.entity_id);
    std::memcpy(buffer + offset, &site_id, 2); offset += 2;
    std::memcpy(buffer + offset, &app_id, 2); offset += 2;
    std::memcpy(buffer + offset, &ent_id, 2); offset += 2;

    site_id = hton16(pdu.target_entity_id.site_id);
    app_id = hton16(pdu.target_entity_id.application_id);
    ent_id = hton16(pdu.target_entity_id.entity_id);
    std::memcpy(buffer + offset, &site_id, 2); offset += 2;
    std::memcpy(buffer + offset, &app_id, 2); offset += 2;
    std::memcpy(buffer + offset, &ent_id, 2); offset += 2;

    site_id = hton16(pdu.munition_id.site_id);
    app_id = hton16(pdu.munition_id.application_id);
    ent_id = hton16(pdu.munition_id.entity_id);
    std::memcpy(buffer + offset, &site_id, 2); offset += 2;
    std::memcpy(buffer + offset, &app_id, 2); offset += 2;
    std::memcpy(buffer + offset, &ent_id, 2); offset += 2;

    // Event ID (6 bytes)
    site_id = hton16(pdu.event_id.site_id);
    app_id = hton16(pdu.event_id.application_id);
    uint16_t event_num = hton16(pdu.event_id.event_number);
    std::memcpy(buffer + offset, &site_id, 2); offset += 2;
    std::memcpy(buffer + offset, &app_id, 2); offset += 2;
    std::memcpy(buffer + offset, &event_num, 2); offset += 2;

    // Velocity (12 bytes)
    float vx = hton_float(pdu.velocity.x);
    float vy = hton_float(pdu.velocity.y);
    float vz = hton_float(pdu.velocity.z);
    std::memcpy(buffer + offset, &vx, 4); offset += 4;
    std::memcpy(buffer + offset, &vy, 4); offset += 4;
    std::memcpy(buffer + offset, &vz, 4); offset += 4;

    // Location in world (24 bytes)
    double lx = hton_double(pdu.location_in_world.x);
    double ly = hton_double(pdu.location_in_world.y);
    double lz = hton_double(pdu.location_in_world.z);
    std::memcpy(buffer + offset, &lx, 8); offset += 8;
    std::memcpy(buffer + offset, &ly, 8); offset += 8;
    std::memcpy(buffer + offset, &lz, 8); offset += 8;

    // Burst Descriptor (16 bytes)
    buffer[offset++] = pdu.burst_descriptor.munition.kind;
    buffer[offset++] = pdu.burst_descriptor.munition.domain;
    uint16_t country = hton16(pdu.burst_descriptor.munition.country);
    std::memcpy(buffer + offset, &country, 2); offset += 2;
    buffer[offset++] = pdu.burst_descriptor.munition.category;
    buffer[offset++] = pdu.burst_descriptor.munition.subcategory;
    buffer[offset++] = pdu.burst_descriptor.munition.specific;
    buffer[offset++] = pdu.burst_descriptor.munition.extra;

    uint16_t warhead = hton16(pdu.burst_descriptor.warhead);
    uint16_t fuse = hton16(pdu.burst_descriptor.fuse);
    uint16_t quantity = hton16(pdu.burst_descriptor.quantity);
    uint16_t rate = hton16(pdu.burst_descriptor.rate);
    std::memcpy(buffer + offset, &warhead, 2); offset += 2;
    std::memcpy(buffer + offset, &fuse, 2); offset += 2;
    std::memcpy(buffer + offset, &quantity, 2); offset += 2;
    std::memcpy(buffer + offset, &rate, 2); offset += 2;

    // Location in entity (12 bytes)
    float lex = hton_float(pdu.location_in_entity.x);
    float ley = hton_float(pdu.location_in_entity.y);
    float lez = hton_float(pdu.location_in_entity.z);
    std::memcpy(buffer + offset, &lex, 4); offset += 4;
    std::memcpy(buffer + offset, &ley, 4); offset += 4;
    std::memcpy(buffer + offset, &lez, 4); offset += 4;

    // Detonation result and articulation count (4 bytes)
    buffer[offset++] = pdu.detonation_result;
    buffer[offset++] = pdu.num_articulation_params;
    std::memcpy(buffer + offset, &padding, 2); offset += 2;

    return offset;
}

} // namespace serialize

// ============================================================================
// DIS Adapter Implementation
// ============================================================================

struct DISAdapter::Impl {
    DISConfig config;
    SocketType send_socket{INVALID_SOCKET_VALUE};
    SocketType recv_socket{INVALID_SOCKET_VALUE};
    sockaddr_in send_addr{};
    sockaddr_in recv_addr{};

    bool connected{false};

    // Callbacks
    EntityStateCallback entity_state_callback;
    FireCallback fire_callback;
    DetonationCallback detonation_callback;

    // Entity mappings
    std::unordered_map<EntityId, EntityIdentifier> engine_to_dis;
    std::unordered_map<EntityIdentifier, EntityId, EntityIdentifierHash> dis_to_engine;

    // Last sent states for dead reckoning comparison
    struct LastState {
        physics::EntityState state;
        std::chrono::steady_clock::time_point time;
    };
    std::unordered_map<EntityId, LastState> last_states;

    // Remote entity states for dead reckoning
    struct RemoteEntityState {
        EntityStatePdu pdu;
        std::chrono::steady_clock::time_point receive_time;
    };
    std::unordered_map<EntityIdentifier, RemoteEntityState, EntityIdentifierHash> remote_states;

    // Statistics
    std::atomic<uint64_t> pdus_sent{0};
    std::atomic<uint64_t> pdus_received{0};
    std::atomic<uint64_t> bytes_sent{0};
    std::atomic<uint64_t> bytes_received{0};

    // Thread safety
    std::mutex state_mutex;

    // Receive buffer
    std::vector<uint8_t> recv_buffer;

    Impl() : recv_buffer(4096) {}
};

DISAdapter::DISAdapter() : impl_(std::make_unique<Impl>()) {}

DISAdapter::DISAdapter(const DISConfig& config) : impl_(std::make_unique<Impl>()) {
    impl_->config = config;
}

DISAdapter::~DISAdapter() {
    shutdown();
}

bool DISAdapter::initialize() {
#ifdef _WIN32
    WSADATA wsa_data;
    if (WSAStartup(MAKEWORD(2, 2), &wsa_data) != 0) {
        return false;
    }
#endif

    // Create send socket (UDP)
    impl_->send_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (impl_->send_socket == INVALID_SOCKET_VALUE) {
        return false;
    }

    // Enable broadcast
    int broadcast_enable = 1;
    setsockopt(impl_->send_socket, SOL_SOCKET, SO_BROADCAST,
               reinterpret_cast<char*>(&broadcast_enable), sizeof(broadcast_enable));

    // Set up send address
    impl_->send_addr.sin_family = AF_INET;
    impl_->send_addr.sin_port = htons(impl_->config.port);
    inet_pton(AF_INET, impl_->config.broadcast_address.c_str(),
              &impl_->send_addr.sin_addr);

    if (impl_->config.enable_receive) {
        // Create receive socket
        impl_->recv_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (impl_->recv_socket == INVALID_SOCKET_VALUE) {
            close_socket(impl_->send_socket);
            impl_->send_socket = INVALID_SOCKET_VALUE;
            return false;
        }

        // Allow address reuse
        int reuse = 1;
        setsockopt(impl_->recv_socket, SOL_SOCKET, SO_REUSEADDR,
                   reinterpret_cast<char*>(&reuse), sizeof(reuse));

        // Set non-blocking
        set_nonblocking(impl_->recv_socket);

        // Bind to receive address
        impl_->recv_addr.sin_family = AF_INET;
        impl_->recv_addr.sin_port = htons(impl_->config.port);
        impl_->recv_addr.sin_addr.s_addr = INADDR_ANY;

        if (bind(impl_->recv_socket, reinterpret_cast<sockaddr*>(&impl_->recv_addr),
                 sizeof(impl_->recv_addr)) < 0) {
            close_socket(impl_->send_socket);
            close_socket(impl_->recv_socket);
            impl_->send_socket = INVALID_SOCKET_VALUE;
            impl_->recv_socket = INVALID_SOCKET_VALUE;
            return false;
        }

        // Join multicast group
        ip_mreq mreq{};
        inet_pton(AF_INET, impl_->config.broadcast_address.c_str(), &mreq.imr_multiaddr);
        mreq.imr_interface.s_addr = INADDR_ANY;
        setsockopt(impl_->recv_socket, IPPROTO_IP, IP_ADD_MEMBERSHIP,
                   reinterpret_cast<char*>(&mreq), sizeof(mreq));
    }

    impl_->connected = true;
    return true;
}

void DISAdapter::shutdown() {
    if (impl_->send_socket != INVALID_SOCKET_VALUE) {
        close_socket(impl_->send_socket);
        impl_->send_socket = INVALID_SOCKET_VALUE;
    }
    if (impl_->recv_socket != INVALID_SOCKET_VALUE) {
        close_socket(impl_->recv_socket);
        impl_->recv_socket = INVALID_SOCKET_VALUE;
    }
    impl_->connected = false;

#ifdef _WIN32
    WSACleanup();
#endif
}

bool DISAdapter::is_connected() const {
    return impl_->connected;
}

const DISConfig& DISAdapter::get_config() const {
    return impl_->config;
}

void DISAdapter::set_config(const DISConfig& config) {
    impl_->config = config;
}

bool DISAdapter::send_entity_state(const EntityStatePdu& pdu) {
    if (!impl_->connected || !impl_->config.enable_send) return false;

    uint8_t buffer[1500];
    size_t size = serialize::serialize_entity_state(pdu, buffer, sizeof(buffer));

    if (size == 0) return false;

    ssize_t sent = sendto(impl_->send_socket, reinterpret_cast<char*>(buffer), size, 0,
                          reinterpret_cast<sockaddr*>(&impl_->send_addr),
                          sizeof(impl_->send_addr));

    if (sent > 0) {
        impl_->pdus_sent++;
        impl_->bytes_sent += sent;
        return true;
    }
    return false;
}

bool DISAdapter::send_fire(const FirePdu& pdu) {
    if (!impl_->connected || !impl_->config.enable_send) return false;

    uint8_t buffer[1500];
    size_t size = serialize::serialize_fire(pdu, buffer, sizeof(buffer));

    if (size == 0) return false;

    ssize_t sent = sendto(impl_->send_socket, reinterpret_cast<char*>(buffer), size, 0,
                          reinterpret_cast<sockaddr*>(&impl_->send_addr),
                          sizeof(impl_->send_addr));

    if (sent > 0) {
        impl_->pdus_sent++;
        impl_->bytes_sent += sent;
        return true;
    }
    return false;
}

bool DISAdapter::send_detonation(const DetonationPdu& pdu) {
    if (!impl_->connected || !impl_->config.enable_send) return false;

    uint8_t buffer[1500];
    size_t size = serialize::serialize_detonation(pdu, buffer, sizeof(buffer));

    if (size == 0) return false;

    ssize_t sent = sendto(impl_->send_socket, reinterpret_cast<char*>(buffer), size, 0,
                          reinterpret_cast<sockaddr*>(&impl_->send_addr),
                          sizeof(impl_->send_addr));

    if (sent > 0) {
        impl_->pdus_sent++;
        impl_->bytes_sent += sent;
        return true;
    }
    return false;
}

bool DISAdapter::send_entity_state(
    EntityId entity_id,
    const physics::EntityState& state,
    const EntityIdentifier& dis_id,
    const EntityType& type) {

    Vec3 origin_ecef{0, 0, 0};  // TODO: Make configurable
    EntityStatePdu pdu = convert::state_to_pdu(state, dis_id, type, origin_ecef);

    // Update last state
    {
        std::lock_guard<std::mutex> lock(impl_->state_mutex);
        impl_->last_states[entity_id] = {state, std::chrono::steady_clock::now()};
    }

    return send_entity_state(pdu);
}

void DISAdapter::update() {
    if (!impl_->connected || !impl_->config.enable_receive) return;
    if (impl_->recv_socket == INVALID_SOCKET_VALUE) return;

    sockaddr_in from_addr{};
    socklen_t from_len = sizeof(from_addr);

    // Process all available PDUs
    while (true) {
        ssize_t received = recvfrom(impl_->recv_socket,
                                    reinterpret_cast<char*>(impl_->recv_buffer.data()),
                                    impl_->recv_buffer.size(), 0,
                                    reinterpret_cast<sockaddr*>(&from_addr), &from_len);

        if (received <= 0) break;

        impl_->pdus_received++;
        impl_->bytes_received += received;

        // Parse header to determine PDU type
        if (received < 12) continue;  // Minimum header size

        PduType pdu_type = static_cast<PduType>(impl_->recv_buffer[2]);

        switch (pdu_type) {
            case PduType::EntityState: {
                EntityStatePdu pdu;
                if (serialize::deserialize_entity_state(impl_->recv_buffer.data(),
                                                        received, pdu)) {
                    // Store remote state
                    {
                        std::lock_guard<std::mutex> lock(impl_->state_mutex);
                        impl_->remote_states[pdu.entity_id] = {
                            pdu, std::chrono::steady_clock::now()
                        };
                    }

                    // Invoke callback
                    if (impl_->entity_state_callback) {
                        impl_->entity_state_callback(pdu);
                    }
                }
                break;
            }
            case PduType::Fire: {
                // TODO: Deserialize Fire PDU
                if (impl_->fire_callback) {
                    FirePdu pdu;
                    impl_->fire_callback(pdu);
                }
                break;
            }
            case PduType::Detonation: {
                // TODO: Deserialize Detonation PDU
                if (impl_->detonation_callback) {
                    DetonationPdu pdu;
                    impl_->detonation_callback(pdu);
                }
                break;
            }
            default:
                // Ignore other PDU types
                break;
        }
    }
}

void DISAdapter::on_entity_state(EntityStateCallback callback) {
    impl_->entity_state_callback = std::move(callback);
}

void DISAdapter::on_fire(FireCallback callback) {
    impl_->fire_callback = std::move(callback);
}

void DISAdapter::on_detonation(DetonationCallback callback) {
    impl_->detonation_callback = std::move(callback);
}

void DISAdapter::register_entity(EntityId engine_id, const EntityIdentifier& dis_id) {
    std::lock_guard<std::mutex> lock(impl_->state_mutex);
    impl_->engine_to_dis[engine_id] = dis_id;
    impl_->dis_to_engine[dis_id] = engine_id;
}

void DISAdapter::unregister_entity(EntityId engine_id) {
    std::lock_guard<std::mutex> lock(impl_->state_mutex);
    auto it = impl_->engine_to_dis.find(engine_id);
    if (it != impl_->engine_to_dis.end()) {
        impl_->dis_to_engine.erase(it->second);
        impl_->engine_to_dis.erase(it);
    }
    impl_->last_states.erase(engine_id);
}

EntityIdentifier DISAdapter::get_dis_id(EntityId engine_id) const {
    std::lock_guard<std::mutex> lock(impl_->state_mutex);
    auto it = impl_->engine_to_dis.find(engine_id);
    return it != impl_->engine_to_dis.end() ? it->second : EntityIdentifier{};
}

EntityId DISAdapter::get_engine_id(const EntityIdentifier& dis_id) const {
    std::lock_guard<std::mutex> lock(impl_->state_mutex);
    auto it = impl_->dis_to_engine.find(dis_id);
    return it != impl_->dis_to_engine.end() ? it->second : INVALID_ENTITY_ID;
}

bool DISAdapter::needs_update(EntityId entity_id, const physics::EntityState& current_state) const {
    std::lock_guard<std::mutex> lock(impl_->state_mutex);

    auto it = impl_->last_states.find(entity_id);
    if (it == impl_->last_states.end()) {
        return true;  // Never sent, needs update
    }

    const auto& last = it->second;

    // Time since last update
    auto now = std::chrono::steady_clock::now();
    double elapsed = std::chrono::duration<double>(now - last.time).count();

    // Heartbeat check
    if (elapsed >= impl_->config.heartbeat_interval) {
        return true;
    }

    // Dead reckoning threshold check
    Vec3 predicted_pos = last.state.position + last.state.velocity * elapsed;
    Vec3 diff = current_state.position - predicted_pos;
    double distance = diff.length();

    return distance > impl_->config.dead_reckoning_threshold;
}

physics::EntityState DISAdapter::get_dead_reckoned_state(
    const EntityIdentifier& dis_id,
    Real current_time) const {

    std::lock_guard<std::mutex> lock(impl_->state_mutex);

    auto it = impl_->remote_states.find(dis_id);
    if (it == impl_->remote_states.end()) {
        return physics::EntityState{};
    }

    const auto& remote = it->second;
    Vec3 origin_ecef{0, 0, 0};

    // Convert PDU to state
    physics::EntityState state = convert::pdu_to_state(remote.pdu, origin_ecef);

    // Calculate elapsed time
    auto now = std::chrono::steady_clock::now();
    double dt = std::chrono::duration<double>(now - remote.receive_time).count();

    // Apply dead reckoning based on algorithm
    switch (remote.pdu.dead_reckoning.algorithm) {
        case DeadReckoningAlgorithm::Static:
        case DeadReckoningAlgorithm::DRM_FPW:
            // Fixed position - no extrapolation
            break;

        case DeadReckoningAlgorithm::DRM_FVW:
            // Fixed velocity world
            state.position = state.position + state.velocity * dt;
            break;

        case DeadReckoningAlgorithm::DRM_RVW:
            // Rotation velocity world
            state.position = state.position + state.velocity * dt;
            // TODO: Integrate angular velocity
            break;

        default:
            // Default to linear extrapolation
            state.position = state.position + state.velocity * dt;
            break;
    }

    return state;
}

uint64_t DISAdapter::get_pdus_sent() const { return impl_->pdus_sent; }
uint64_t DISAdapter::get_pdus_received() const { return impl_->pdus_received; }
uint64_t DISAdapter::get_bytes_sent() const { return impl_->bytes_sent; }
uint64_t DISAdapter::get_bytes_received() const { return impl_->bytes_received; }

void DISAdapter::reset_statistics() {
    impl_->pdus_sent = 0;
    impl_->pdus_received = 0;
    impl_->bytes_sent = 0;
    impl_->bytes_received = 0;
}

// ============================================================================
// Conversion Utilities
// ============================================================================

namespace convert {

WorldCoordinates ned_to_ecef(const Vec3& ned, const Vec3& origin_ecef) {
    // Simplified conversion (for small displacements near origin)
    // Full implementation would use proper geodetic transforms
    WorldCoordinates ecef;
    ecef.x = origin_ecef.x + ned.x;  // North -> X
    ecef.y = origin_ecef.y + ned.y;  // East -> Y
    ecef.z = origin_ecef.z - ned.z;  // Down -> -Z
    return ecef;
}

Vec3 ecef_to_ned(const WorldCoordinates& ecef, const Vec3& origin_ecef) {
    Vec3 ned;
    ned.x = ecef.x - origin_ecef.x;
    ned.y = ecef.y - origin_ecef.y;
    ned.z = -(ecef.z - origin_ecef.z);
    return ned;
}

EulerAngles quat_to_euler(const Quat& q) {
    EulerAngles euler;
    Real roll, pitch, yaw;
    q.to_euler(roll, pitch, yaw);
    euler.phi = static_cast<float>(roll);
    euler.theta = static_cast<float>(pitch);
    euler.psi = static_cast<float>(yaw);
    return euler;
}

Quat euler_to_quat(const EulerAngles& euler) {
    return Quat::from_euler(euler.phi, euler.theta, euler.psi);
}

uint8_t domain_to_dis(Domain domain) {
    switch (domain) {
        case Domain::Land: return 1;
        case Domain::Air: return 2;
        case Domain::Sea: return 3;
        case Domain::Space: return 5;
        default: return 0;
    }
}

Domain dis_to_domain(uint8_t dis_domain) {
    switch (dis_domain) {
        case 1: return Domain::Land;
        case 2: return Domain::Air;
        case 3: return Domain::Sea;
        case 5: return Domain::Space;
        default: return Domain::Generic;
    }
}

EntityStatePdu state_to_pdu(
    const physics::EntityState& state,
    const EntityIdentifier& entity_id,
    const EntityType& entity_type,
    const Vec3& origin_ecef) {

    EntityStatePdu pdu;

    pdu.entity_id = entity_id;
    pdu.entity_type = entity_type;

    // Convert position
    pdu.location = ned_to_ecef(state.position, origin_ecef);

    // Convert velocity
    pdu.linear_velocity.x = static_cast<float>(state.velocity.x);
    pdu.linear_velocity.y = static_cast<float>(state.velocity.y);
    pdu.linear_velocity.z = static_cast<float>(state.velocity.z);

    // Convert orientation
    pdu.orientation = quat_to_euler(state.orientation);

    // Set dead reckoning
    pdu.dead_reckoning.algorithm = DeadReckoningAlgorithm::DRM_FVW;
    pdu.dead_reckoning.linear_acceleration.x = static_cast<float>(state.acceleration.x);
    pdu.dead_reckoning.linear_acceleration.y = static_cast<float>(state.acceleration.y);
    pdu.dead_reckoning.linear_acceleration.z = static_cast<float>(state.acceleration.z);
    pdu.dead_reckoning.angular_velocity.x = static_cast<float>(state.angular_velocity.x);
    pdu.dead_reckoning.angular_velocity.y = static_cast<float>(state.angular_velocity.y);
    pdu.dead_reckoning.angular_velocity.z = static_cast<float>(state.angular_velocity.z);

    // Calculate timestamp
    auto now = std::chrono::system_clock::now();
    auto epoch = now.time_since_epoch();
    auto seconds = std::chrono::duration_cast<std::chrono::seconds>(epoch);
    pdu.header.timestamp = static_cast<uint32_t>(seconds.count() % 0x7FFFFFFF);

    return pdu;
}

physics::EntityState pdu_to_state(
    const EntityStatePdu& pdu,
    const Vec3& origin_ecef) {

    physics::EntityState state;

    // Convert position
    state.position = ecef_to_ned(pdu.location, origin_ecef);

    // Convert velocity
    state.velocity.x = pdu.linear_velocity.x;
    state.velocity.y = pdu.linear_velocity.y;
    state.velocity.z = pdu.linear_velocity.z;

    // Convert acceleration
    state.acceleration.x = pdu.dead_reckoning.linear_acceleration.x;
    state.acceleration.y = pdu.dead_reckoning.linear_acceleration.y;
    state.acceleration.z = pdu.dead_reckoning.linear_acceleration.z;

    // Convert orientation
    state.orientation = euler_to_quat(pdu.orientation);

    // Convert angular velocity
    state.angular_velocity.x = pdu.dead_reckoning.angular_velocity.x;
    state.angular_velocity.y = pdu.dead_reckoning.angular_velocity.y;
    state.angular_velocity.z = pdu.dead_reckoning.angular_velocity.z;

    return state;
}

} // namespace convert

} // namespace jaguar::interface::dis
