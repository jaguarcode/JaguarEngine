#pragma once
/**
 * @file dis_socket.h
 * @brief UDP socket wrapper for DIS (Distributed Interactive Simulation) protocol
 *
 * Provides a cross-platform UDP socket implementation optimized for DIS network traffic.
 * Supports unicast, broadcast, and multicast modes with MTU-aware fragmentation for
 * large PDUs. Thread-safe for concurrent send/receive operations.
 *
 * Key features:
 * - Cross-platform socket abstraction (POSIX/Winsock)
 * - Non-blocking receive with configurable timeout
 * - MTU-aware fragmentation for large PDUs (>1500 bytes)
 * - Multicast group management
 * - Broadcast support for discovery
 * - Configurable buffer sizes
 * - Multiple endpoint support (send to multiple receivers)
 * - DIS-specific error codes
 * - Thread-safe operations
 *
 * DIS typically uses:
 * - UDP port 3000 (default)
 * - Multicast groups in 224.0.0.0/4 range
 * - MTU of 1500 bytes (Ethernet) or 1492 (PPPoE)
 *
 * @see IEEE 1278.1-2012 (DIS Standard)
 */

#include "jaguar/core/types.h"
#include <string>
#include <string_view>
#include <vector>
#include <memory>
#include <chrono>
#include <optional>
#include <mutex>
#include <cstdint>

namespace jaguar::federation {

//==============================================================================
// DIS Socket Constants
//==============================================================================

/// Default DIS port (IEEE 1278.1)
constexpr UInt16 DIS_DEFAULT_PORT = 3000;

/// Default DIS multicast address
constexpr const char* DIS_DEFAULT_MULTICAST = "224.0.1.10";

/// Typical Ethernet MTU (maximum transmission unit)
constexpr SizeT DIS_MTU_ETHERNET = 1500;

/// Typical PPPoE MTU
constexpr SizeT DIS_MTU_PPPOE = 1492;

/// IPv4 header size
constexpr SizeT IP_HEADER_SIZE = 20;

/// UDP header size
constexpr SizeT UDP_HEADER_SIZE = 8;

/// Maximum UDP payload size for typical Ethernet (MTU - IP - UDP headers)
constexpr SizeT DIS_MAX_PAYLOAD_SIZE = DIS_MTU_ETHERNET - IP_HEADER_SIZE - UDP_HEADER_SIZE;

/// Default socket buffer size
constexpr SizeT DIS_DEFAULT_BUFFER_SIZE = 65536;

/// Maximum endpoints for multi-send
constexpr SizeT DIS_MAX_ENDPOINTS = 64;

//==============================================================================
// DIS Socket Error Codes
//==============================================================================

/**
 * @brief DIS-specific socket error codes
 */
enum class DisSocketError : UInt32 {
    Success = 0,

    // Initialization errors
    NotInitialized,
    AlreadyInitialized,
    SocketCreationFailed,
    BindFailed,
    InvalidConfiguration,

    // Network errors
    SendFailed,
    ReceiveFailed,
    WouldBlock,
    Timeout,
    ConnectionReset,
    NetworkUnreachable,

    // Multicast errors
    MulticastJoinFailed,
    MulticastLeaveFailed,
    InvalidMulticastAddress,

    // Address errors
    InvalidAddress,
    AddressInUse,
    AddressNotAvailable,

    // Buffer errors
    BufferTooSmall,
    BufferOverflow,
    FragmentationRequired,
    PayloadTooLarge,

    // Configuration errors
    InvalidSocketOption,
    SetOptionFailed,
    GetOptionFailed,

    // Platform errors
    PlatformNotSupported,
    PermissionDenied,

    // General errors
    InternalError,
    Unknown
};

/**
 * @brief Convert error code to string
 */
const char* dis_socket_error_to_string(DisSocketError error) noexcept;

//==============================================================================
// Network Endpoint
//==============================================================================

/**
 * @brief Network endpoint (IP address and port)
 */
struct DisEndpoint {
    std::string address;    ///< IP address (IPv4 dotted-quad)
    UInt16 port{0};         ///< Port number

    DisEndpoint() = default;
    DisEndpoint(std::string_view addr, UInt16 p) : address(addr), port(p) {}

    /// Parse from string (e.g., "192.168.1.100:3000")
    static std::optional<DisEndpoint> parse(std::string_view str) noexcept;

    /// Convert to string
    std::string to_string() const;

    /// Check if this is a multicast address
    bool is_multicast() const noexcept;

    /// Check if this is a broadcast address
    bool is_broadcast() const noexcept;

    /// Check if this is a loopback address
    bool is_loopback() const noexcept;

    /// Equality comparison
    bool operator==(const DisEndpoint& other) const noexcept;
    bool operator!=(const DisEndpoint& other) const noexcept { return !(*this == other); }
};

//==============================================================================
// Socket Configuration
//==============================================================================

/**
 * @brief DIS socket configuration
 */
struct DisSocketConfig {
    /// Local endpoint to bind to
    DisEndpoint bind_endpoint{"0.0.0.0", DIS_DEFAULT_PORT};

    /// Receive buffer size (OS socket buffer)
    SizeT receive_buffer_size{DIS_DEFAULT_BUFFER_SIZE};

    /// Send buffer size (OS socket buffer)
    SizeT send_buffer_size{DIS_DEFAULT_BUFFER_SIZE};

    /// Enable broadcast
    bool broadcast_enabled{false};

    /// Allow address reuse (SO_REUSEADDR)
    bool reuse_address{true};

    /// Allow port reuse (SO_REUSEPORT on Linux)
    bool reuse_port{false};

    /// Multicast TTL (time-to-live, hop count)
    UInt8 multicast_ttl{32};

    /// Receive own multicast packets
    bool multicast_loopback{true};

    /// Network interface for multicast (empty = default)
    std::string multicast_interface;

    /// Receive timeout (0 = non-blocking)
    std::chrono::milliseconds receive_timeout{0};

    /// Send timeout (0 = non-blocking)
    std::chrono::milliseconds send_timeout{0};

    /// MTU size for fragmentation awareness
    SizeT mtu{DIS_MTU_ETHERNET};

    /// Enable automatic fragmentation handling
    bool auto_fragment{true};

    /// Default configuration for DIS
    static DisSocketConfig dis_default() noexcept {
        DisSocketConfig config;
        config.bind_endpoint = DisEndpoint("0.0.0.0", DIS_DEFAULT_PORT);
        config.broadcast_enabled = false;
        config.multicast_ttl = 32;
        config.multicast_loopback = true;
        return config;
    }

    /// Configuration for DIS multicast
    static DisSocketConfig dis_multicast(const std::string& multicast_group = DIS_DEFAULT_MULTICAST) noexcept {
        DisSocketConfig config = dis_default();
        // Note: bind to INADDR_ANY, not multicast group
        config.bind_endpoint = DisEndpoint("0.0.0.0", DIS_DEFAULT_PORT);
        config.reuse_address = true;
        config.reuse_port = true;
        return config;
    }
};

//==============================================================================
// Receive Result
//==============================================================================

/**
 * @brief Result of receive operation
 */
struct DisReceiveResult {
    SizeT bytes_received{0};        ///< Number of bytes received
    DisEndpoint source;             ///< Source endpoint
    bool truncated{false};          ///< True if message was truncated
    std::chrono::steady_clock::time_point timestamp; ///< Receive timestamp
};

//==============================================================================
// Socket Statistics
//==============================================================================

/**
 * @brief Socket I/O statistics
 */
struct DisSocketStats {
    UInt64 packets_sent{0};
    UInt64 packets_received{0};
    UInt64 bytes_sent{0};
    UInt64 bytes_received{0};
    UInt64 send_errors{0};
    UInt64 receive_errors{0};
    UInt64 fragments_sent{0};       ///< Number of fragmented sends
    UInt64 multicast_packets{0};
    UInt64 broadcast_packets{0};
    std::chrono::steady_clock::time_point last_send;
    std::chrono::steady_clock::time_point last_receive;

    /// Reset all statistics
    void reset() noexcept {
        packets_sent = packets_received = 0;
        bytes_sent = bytes_received = 0;
        send_errors = receive_errors = 0;
        fragments_sent = multicast_packets = broadcast_packets = 0;
    }

    /// Calculate throughput (bytes per second)
    Real throughput_sent(std::chrono::duration<Real> period) const noexcept {
        if (period.count() <= 0) return 0.0;
        return static_cast<Real>(bytes_sent) / period.count();
    }

    Real throughput_received(std::chrono::duration<Real> period) const noexcept {
        if (period.count() <= 0) return 0.0;
        return static_cast<Real>(bytes_received) / period.count();
    }
};

//==============================================================================
// DIS Socket
//==============================================================================

/**
 * @brief UDP socket for DIS protocol communication
 *
 * Thread-safe UDP socket implementation optimized for DIS traffic.
 * Supports unicast, multicast, and broadcast modes. Handles MTU-aware
 * fragmentation for large PDUs automatically.
 *
 * Usage:
 * @code
 * DisSocket socket;
 * auto config = DisSocketConfig::dis_multicast();
 * socket.initialize(config);
 * socket.join_multicast_group(DisEndpoint(DIS_DEFAULT_MULTICAST, DIS_DEFAULT_PORT));
 *
 * // Send PDU
 * std::vector<UInt8> pdu_data = ...;
 * socket.send_to(pdu_data.data(), pdu_data.size(),
 *                DisEndpoint(DIS_DEFAULT_MULTICAST, DIS_DEFAULT_PORT));
 *
 * // Receive PDU
 * std::vector<UInt8> buffer(DIS_MAX_PDU_SIZE);
 * DisReceiveResult result;
 * auto error = socket.receive_from(buffer.data(), buffer.size(), result);
 * @endcode
 */
class DisSocket {
public:
    DisSocket();
    ~DisSocket();

    // Non-copyable, movable
    DisSocket(const DisSocket&) = delete;
    DisSocket& operator=(const DisSocket&) = delete;
    DisSocket(DisSocket&& other) noexcept;
    DisSocket& operator=(DisSocket&& other) noexcept;

    //==========================================================================
    // Lifecycle
    //==========================================================================

    /**
     * @brief Initialize socket with configuration
     * @param config Socket configuration
     * @return Error code (Success or error)
     */
    DisSocketError initialize(const DisSocketConfig& config);

    /**
     * @brief Close socket and release resources
     */
    void close();

    /**
     * @brief Check if socket is open
     */
    bool is_open() const noexcept;

    /**
     * @brief Check if socket is initialized
     */
    bool is_initialized() const noexcept { return is_open(); }

    //==========================================================================
    // Multicast
    //==========================================================================

    /**
     * @brief Join multicast group
     * @param group Multicast group endpoint
     * @param interface_address Local interface address (empty = default)
     * @return Error code
     */
    DisSocketError join_multicast_group(const DisEndpoint& group,
                                        const std::string& interface_address = "");

    /**
     * @brief Leave multicast group
     * @param group Multicast group endpoint
     * @return Error code
     */
    DisSocketError leave_multicast_group(const DisEndpoint& group);

    /**
     * @brief Leave all multicast groups
     */
    void leave_all_multicast_groups();

    //==========================================================================
    // Send Operations
    //==========================================================================

    /**
     * @brief Send data to single destination
     * @param data Data buffer
     * @param length Data length
     * @param destination Destination endpoint
     * @return Error code
     */
    DisSocketError send_to(const UInt8* data, SizeT length, const DisEndpoint& destination);

    /**
     * @brief Send data to multiple destinations
     * @param data Data buffer
     * @param length Data length
     * @param destinations Array of destination endpoints
     * @param num_destinations Number of destinations
     * @return Error code (first error encountered, or Success)
     */
    DisSocketError send_to_multiple(const UInt8* data, SizeT length,
                                     const DisEndpoint* destinations,
                                     SizeT num_destinations);

    /**
     * @brief Send data with automatic fragmentation if needed
     *
     * If data exceeds MTU, this will fragment the payload and send
     * multiple packets. Note: DIS does not define a standard fragmentation
     * protocol, so this should be avoided when possible.
     *
     * @param data Data buffer
     * @param length Data length
     * @param destination Destination endpoint
     * @return Error code
     */
    DisSocketError send_fragmented(const UInt8* data, SizeT length,
                                    const DisEndpoint& destination);

    //==========================================================================
    // Receive Operations
    //==========================================================================

    /**
     * @brief Receive data (non-blocking or with timeout)
     * @param buffer Receive buffer
     * @param buffer_size Buffer size
     * @param[out] result Receive result (source, size, etc.)
     * @return Error code
     */
    DisSocketError receive_from(UInt8* buffer, SizeT buffer_size, DisReceiveResult& result);

    /**
     * @brief Check if data is available for reading
     * @param timeout Maximum time to wait (0 = immediate check)
     * @return True if data is available
     */
    bool poll_readable(std::chrono::milliseconds timeout = std::chrono::milliseconds{0});

    //==========================================================================
    // Configuration
    //==========================================================================

    /**
     * @brief Get local bound endpoint
     */
    DisEndpoint get_local_endpoint() const;

    /**
     * @brief Set receive timeout
     */
    DisSocketError set_receive_timeout(std::chrono::milliseconds timeout);

    /**
     * @brief Set send timeout
     */
    DisSocketError set_send_timeout(std::chrono::milliseconds timeout);

    /**
     * @brief Set multicast TTL
     */
    DisSocketError set_multicast_ttl(UInt8 ttl);

    /**
     * @brief Enable/disable broadcast
     */
    DisSocketError set_broadcast(bool enabled);

    /**
     * @brief Get MTU size
     */
    SizeT get_mtu() const noexcept;

    /**
     * @brief Set MTU size
     */
    void set_mtu(SizeT mtu) noexcept;

    //==========================================================================
    // Statistics
    //==========================================================================

    /**
     * @brief Get socket statistics
     */
    const DisSocketStats& get_stats() const noexcept;

    /**
     * @brief Reset statistics
     */
    void reset_stats() noexcept;

    //==========================================================================
    // Utilities
    //==========================================================================

    /**
     * @brief Get last error message (platform-specific)
     */
    std::string get_last_error_message() const;

    /**
     * @brief Get native socket handle (for advanced usage)
     * @return Platform-specific socket handle (SOCKET on Windows, int on POSIX)
     */
    intptr_t get_native_handle() const noexcept;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
    mutable std::mutex mutex_;  // Thread safety
};

//==============================================================================
// Utility Functions
//==============================================================================

/**
 * @brief Get list of local network interfaces
 * @return Vector of (interface_name, address) pairs
 */
std::vector<std::pair<std::string, std::string>> get_network_interfaces();

/**
 * @brief Get default network interface address
 */
std::optional<std::string> get_default_interface_address();

/**
 * @brief Check if address is valid IPv4
 */
bool is_valid_ipv4(std::string_view address) noexcept;

/**
 * @brief Check if port is available for binding
 */
bool is_port_available(UInt16 port);

/**
 * @brief Convert host byte order to network byte order (16-bit)
 */
inline UInt16 htons_safe(UInt16 value) noexcept {
    const UInt8* bytes = reinterpret_cast<const UInt8*>(&value);
    return (static_cast<UInt16>(bytes[0]) << 8) | bytes[1];
}

/**
 * @brief Convert host byte order to network byte order (32-bit)
 */
inline UInt32 htonl_safe(UInt32 value) noexcept {
    const UInt8* bytes = reinterpret_cast<const UInt8*>(&value);
    return (static_cast<UInt32>(bytes[0]) << 24) |
           (static_cast<UInt32>(bytes[1]) << 16) |
           (static_cast<UInt32>(bytes[2]) << 8) |
           bytes[3];
}

/**
 * @brief Convert network byte order to host byte order (16-bit)
 */
inline UInt16 ntohs_safe(UInt16 value) noexcept { return htons_safe(value); }

/**
 * @brief Convert network byte order to host byte order (32-bit)
 */
inline UInt32 ntohl_safe(UInt32 value) noexcept { return htonl_safe(value); }

} // namespace jaguar::federation
