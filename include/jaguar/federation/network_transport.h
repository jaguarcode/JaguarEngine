// Copyright JaguarEngine Team. All Rights Reserved.
//
// Network Transport Layer for Federation Protocols
//
// Provides a unified network abstraction for DIS and HLA protocols,
// supporting UDP unicast/multicast, TCP connections, and reliable
// message delivery. Implements zero-copy buffer management and
// asynchronous I/O for high-performance networking.

#pragma once

#include "jaguar/core/types.h"
#include <cstdint>
#include <string>
#include <string_view>
#include <vector>
#include <memory>
#include <functional>
#include <optional>
#include <chrono>
#include <span>

namespace jaguar::federation::network {

//==============================================================================
// Network Constants
//==============================================================================

/// Default DIS port (IEEE 1278.1)
constexpr UInt16 DEFAULT_DIS_PORT = 3000;

/// Default HLA RTI port
constexpr UInt16 DEFAULT_HLA_PORT = 8989;

/// Maximum UDP payload size (typical MTU - headers)
constexpr SizeT MAX_UDP_PAYLOAD = 65507;

/// Default receive buffer size
constexpr SizeT DEFAULT_RECV_BUFFER_SIZE = 65536;

/// Maximum multicast TTL
constexpr UInt8 MAX_MULTICAST_TTL = 255;

/// Default multicast TTL
constexpr UInt8 DEFAULT_MULTICAST_TTL = 32;

//==============================================================================
// Network Result Codes
//==============================================================================

/**
 * @brief Result codes for network operations
 */
enum class NetworkResult : UInt32 {
    Success = 0,

    // Socket errors
    SocketCreationFailed,
    BindFailed,
    ConnectFailed,
    ListenFailed,
    AcceptFailed,
    AlreadyConnected,
    NotConnected,

    // Address errors
    InvalidAddress,
    AddressInUse,
    AddressNotAvailable,

    // Multicast errors
    MulticastJoinFailed,
    MulticastLeaveFailed,
    InvalidMulticastGroup,

    // I/O errors
    SendFailed,
    ReceiveFailed,
    WouldBlock,
    Timeout,
    ConnectionReset,
    ConnectionClosed,

    // Buffer errors
    BufferTooSmall,
    BufferOverflow,

    // Configuration errors
    InvalidConfiguration,
    InvalidOption,

    // General errors
    NotInitialized,
    AlreadyInitialized,
    OperationAborted,
    InternalError
};

/**
 * @brief Convert NetworkResult to string
 */
const char* network_result_to_string(NetworkResult result);

//==============================================================================
// Network Address Types
//==============================================================================

/**
 * @brief IP address version
 */
enum class IPVersion : UInt8 {
    IPv4,
    IPv6,
    Any  // Accept either version
};

/**
 * @brief Socket address abstraction
 */
struct SocketAddress {
    std::string host;
    UInt16 port{0};
    IPVersion version{IPVersion::IPv4};

    SocketAddress() = default;
    SocketAddress(std::string_view h, UInt16 p, IPVersion v = IPVersion::IPv4)
        : host(h), port(p), version(v) {}

    /// Parse from string (e.g., "192.168.1.1:3000" or "[::1]:3000")
    static std::optional<SocketAddress> parse(std::string_view address);

    /// Convert to string
    std::string to_string() const;

    /// Check if this is a multicast address
    bool is_multicast() const;

    /// Check if this is a loopback address
    bool is_loopback() const;

    /// Check if this is a broadcast address
    bool is_broadcast() const;

    /// Equality comparison
    bool operator==(const SocketAddress& other) const;
    bool operator!=(const SocketAddress& other) const { return !(*this == other); }
};

/// Any address constant (bind to all interfaces)
inline SocketAddress ANY_ADDRESS("0.0.0.0", 0, IPVersion::IPv4);

/// Loopback address constant
inline SocketAddress LOOPBACK_ADDRESS("127.0.0.1", 0, IPVersion::IPv4);

//==============================================================================
// Network Statistics
//==============================================================================

/**
 * @brief Network I/O statistics
 */
struct NetworkStats {
    UInt64 bytes_sent{0};
    UInt64 bytes_received{0};
    UInt64 packets_sent{0};
    UInt64 packets_received{0};
    UInt64 send_errors{0};
    UInt64 receive_errors{0};
    UInt64 dropped_packets{0};
    std::chrono::steady_clock::time_point last_send_time;
    std::chrono::steady_clock::time_point last_receive_time;

    /// Reset all statistics
    void reset() {
        bytes_sent = bytes_received = 0;
        packets_sent = packets_received = 0;
        send_errors = receive_errors = dropped_packets = 0;
    }

    /// Calculate bytes per second (send)
    Real bytes_per_second_sent(std::chrono::duration<Real> period) const {
        if (period.count() <= 0) return 0.0;
        return static_cast<Real>(bytes_sent) / period.count();
    }

    /// Calculate bytes per second (receive)
    Real bytes_per_second_received(std::chrono::duration<Real> period) const {
        if (period.count() <= 0) return 0.0;
        return static_cast<Real>(bytes_received) / period.count();
    }
};

//==============================================================================
// Buffer Types
//==============================================================================

/**
 * @brief Scatter-gather I/O buffer descriptor
 */
struct IOBuffer {
    UInt8* data{nullptr};
    SizeT length{0};

    IOBuffer() = default;
    IOBuffer(UInt8* d, SizeT len) : data(d), length(len) {}

    /// Create from vector
    static IOBuffer from_vector(std::vector<UInt8>& vec) {
        return IOBuffer(vec.data(), vec.size());
    }
};

/**
 * @brief Receive result with source address
 */
struct ReceiveResult {
    SizeT bytes_received{0};
    SocketAddress source;
    bool truncated{false};  // True if message was larger than buffer
};

//==============================================================================
// UDP Socket Interface
//==============================================================================

/**
 * @brief UDP socket configuration
 */
struct UDPSocketConfig {
    SocketAddress bind_address;              // Local address to bind
    SizeT receive_buffer_size{DEFAULT_RECV_BUFFER_SIZE};
    SizeT send_buffer_size{DEFAULT_RECV_BUFFER_SIZE};
    bool broadcast_enabled{false};           // Allow broadcast
    bool reuse_address{true};                // Allow address reuse
    bool reuse_port{false};                  // Allow port reuse (Linux)
    UInt8 multicast_ttl{DEFAULT_MULTICAST_TTL};
    bool multicast_loopback{true};           // Receive own multicast
    std::optional<std::string> multicast_interface;  // Interface for multicast
    std::chrono::milliseconds receive_timeout{0};    // 0 = non-blocking
    std::chrono::milliseconds send_timeout{0};

    /// Default configuration for DIS
    static UDPSocketConfig dis_default(UInt16 port = DEFAULT_DIS_PORT) {
        UDPSocketConfig config;
        config.bind_address = SocketAddress("0.0.0.0", port);
        config.multicast_ttl = 32;
        config.multicast_loopback = true;
        return config;
    }
};

/**
 * @brief UDP socket interface for connectionless networking
 *
 * Supports unicast, broadcast, and multicast communication.
 */
class IUDPSocket {
public:
    virtual ~IUDPSocket() = default;

    //==========================================================================
    // Lifecycle
    //==========================================================================

    /**
     * @brief Initialize socket with configuration
     * @param config Socket configuration
     * @return Success or error code
     */
    virtual NetworkResult initialize(const UDPSocketConfig& config) = 0;

    /**
     * @brief Close socket and release resources
     */
    virtual void close() = 0;

    /**
     * @brief Check if socket is open
     */
    virtual bool is_open() const = 0;

    //==========================================================================
    // Multicast
    //==========================================================================

    /**
     * @brief Join multicast group
     * @param group Multicast group address
     * @param interface_address Local interface (empty = default)
     * @return Success or error code
     */
    virtual NetworkResult join_multicast_group(
        const SocketAddress& group,
        const std::string& interface_address = "") = 0;

    /**
     * @brief Leave multicast group
     * @param group Multicast group address
     * @return Success or error code
     */
    virtual NetworkResult leave_multicast_group(const SocketAddress& group) = 0;

    //==========================================================================
    // Send/Receive
    //==========================================================================

    /**
     * @brief Send data to destination
     * @param data Data buffer
     * @param length Data length
     * @param destination Destination address
     * @return Bytes sent or error code (negative)
     */
    virtual Int64 send_to(
        const UInt8* data,
        SizeT length,
        const SocketAddress& destination) = 0;

    /**
     * @brief Send data using scatter-gather I/O
     * @param buffers Array of I/O buffers
     * @param buffer_count Number of buffers
     * @param destination Destination address
     * @return Bytes sent or error code (negative)
     */
    virtual Int64 send_to_gather(
        const IOBuffer* buffers,
        SizeT buffer_count,
        const SocketAddress& destination) = 0;

    /**
     * @brief Receive data (non-blocking)
     * @param buffer Receive buffer
     * @param buffer_size Buffer size
     * @param[out] result Receive result with source address
     * @return Success or error code
     */
    virtual NetworkResult receive_from(
        UInt8* buffer,
        SizeT buffer_size,
        ReceiveResult& result) = 0;

    /**
     * @brief Check if data is available for reading
     * @param timeout Maximum time to wait (0 = immediate check)
     * @return True if data is available
     */
    virtual bool poll_readable(std::chrono::milliseconds timeout = {}) = 0;

    //==========================================================================
    // Configuration
    //==========================================================================

    /**
     * @brief Get local bound address
     */
    virtual SocketAddress get_local_address() const = 0;

    /**
     * @brief Set receive timeout
     */
    virtual NetworkResult set_receive_timeout(std::chrono::milliseconds timeout) = 0;

    /**
     * @brief Set send timeout
     */
    virtual NetworkResult set_send_timeout(std::chrono::milliseconds timeout) = 0;

    /**
     * @brief Set multicast TTL
     */
    virtual NetworkResult set_multicast_ttl(UInt8 ttl) = 0;

    //==========================================================================
    // Statistics
    //==========================================================================

    /**
     * @brief Get network statistics
     */
    virtual const NetworkStats& get_stats() const = 0;

    /**
     * @brief Reset network statistics
     */
    virtual void reset_stats() = 0;
};

//==============================================================================
// TCP Socket Interface
//==============================================================================

/**
 * @brief TCP connection configuration
 */
struct TCPConnectionConfig {
    SocketAddress remote_address;            // Remote endpoint
    std::chrono::milliseconds connect_timeout{30000};
    std::chrono::milliseconds receive_timeout{0};   // 0 = blocking
    std::chrono::milliseconds send_timeout{0};
    SizeT receive_buffer_size{DEFAULT_RECV_BUFFER_SIZE};
    SizeT send_buffer_size{DEFAULT_RECV_BUFFER_SIZE};
    bool tcp_nodelay{true};                  // Disable Nagle's algorithm
    bool keep_alive{true};                   // Enable keep-alive
    std::chrono::seconds keep_alive_idle{60};
    std::chrono::seconds keep_alive_interval{10};
    Int32 keep_alive_count{3};
};

/**
 * @brief TCP server configuration
 */
struct TCPServerConfig {
    SocketAddress bind_address;
    Int32 backlog{128};                      // Listen backlog
    bool reuse_address{true};
    SizeT receive_buffer_size{DEFAULT_RECV_BUFFER_SIZE};
    SizeT send_buffer_size{DEFAULT_RECV_BUFFER_SIZE};
};

/**
 * @brief TCP connection interface
 */
class ITCPConnection {
public:
    virtual ~ITCPConnection() = default;

    /**
     * @brief Connect to remote endpoint
     * @param config Connection configuration
     * @return Success or error code
     */
    virtual NetworkResult connect(const TCPConnectionConfig& config) = 0;

    /**
     * @brief Close connection
     * @param graceful True for graceful close (wait for pending data)
     */
    virtual void close(bool graceful = true) = 0;

    /**
     * @brief Check if connected
     */
    virtual bool is_connected() const = 0;

    /**
     * @brief Send data
     * @param data Data buffer
     * @param length Data length
     * @return Bytes sent or error code (negative)
     */
    virtual Int64 send(const UInt8* data, SizeT length) = 0;

    /**
     * @brief Send all data (blocking)
     * @return Success or error code
     */
    virtual NetworkResult send_all(const UInt8* data, SizeT length) = 0;

    /**
     * @brief Receive data
     * @param buffer Receive buffer
     * @param buffer_size Buffer size
     * @return Bytes received or error code (negative)
     */
    virtual Int64 receive(UInt8* buffer, SizeT buffer_size) = 0;

    /**
     * @brief Receive exact amount of data (blocking)
     * @return Success or error code
     */
    virtual NetworkResult receive_exact(UInt8* buffer, SizeT length) = 0;

    /**
     * @brief Check if data is available for reading
     */
    virtual bool poll_readable(std::chrono::milliseconds timeout = {}) = 0;

    /**
     * @brief Get remote address
     */
    virtual SocketAddress get_remote_address() const = 0;

    /**
     * @brief Get local address
     */
    virtual SocketAddress get_local_address() const = 0;

    /**
     * @brief Get network statistics
     */
    virtual const NetworkStats& get_stats() const = 0;
};

/**
 * @brief TCP server/listener interface
 */
class ITCPServer {
public:
    virtual ~ITCPServer() = default;

    /**
     * @brief Start listening for connections
     * @param config Server configuration
     * @return Success or error code
     */
    virtual NetworkResult listen(const TCPServerConfig& config) = 0;

    /**
     * @brief Stop listening and close server
     */
    virtual void close() = 0;

    /**
     * @brief Check if server is listening
     */
    virtual bool is_listening() const = 0;

    /**
     * @brief Accept incoming connection
     * @param timeout Maximum time to wait
     * @return Connection or nullptr if none available
     */
    virtual std::unique_ptr<ITCPConnection> accept(
        std::chrono::milliseconds timeout = {}) = 0;

    /**
     * @brief Check if connection is pending
     */
    virtual bool poll_pending(std::chrono::milliseconds timeout = {}) = 0;

    /**
     * @brief Get local bound address
     */
    virtual SocketAddress get_local_address() const = 0;
};

//==============================================================================
// Message Framing
//==============================================================================

/**
 * @brief Length-prefixed message framing for TCP
 *
 * Provides message boundaries over stream-oriented TCP.
 * Format: [4-byte length (network byte order)][message data]
 */
class MessageFramer {
public:
    MessageFramer() = default;

    /**
     * @brief Frame a message with length prefix
     * @param message Message data
     * @param length Message length
     * @param[out] output Output buffer (must be length + 4 bytes)
     */
    void frame_message(const UInt8* message, SizeT length, UInt8* output);

    /**
     * @brief Add data to receive buffer and extract complete messages
     * @param data Received data
     * @param length Data length
     * @param[out] messages Extracted complete messages
     * @return Number of messages extracted
     */
    SizeT process_received_data(
        const UInt8* data,
        SizeT length,
        std::vector<std::vector<UInt8>>& messages);

    /**
     * @brief Check if partial message is buffered
     */
    bool has_partial_message() const { return !receive_buffer_.empty(); }

    /**
     * @brief Clear receive buffer
     */
    void clear() { receive_buffer_.clear(); }

private:
    std::vector<UInt8> receive_buffer_;
};

//==============================================================================
// Reliable UDP
//==============================================================================

/**
 * @brief Reliable UDP configuration
 */
struct ReliableUDPConfig {
    UDPSocketConfig socket_config;
    std::chrono::milliseconds ack_timeout{100};
    std::chrono::milliseconds retry_interval{200};
    Int32 max_retries{5};
    SizeT max_pending_messages{1000};
    SizeT window_size{32};  // Sliding window size
};

/**
 * @brief Sequence number type for reliable messaging
 */
using SequenceNumber = UInt32;

/**
 * @brief Acknowledgment information
 */
struct AckInfo {
    SequenceNumber sequence;
    std::chrono::steady_clock::time_point sent_time;
    std::chrono::steady_clock::time_point ack_time;
    Int32 retry_count{0};
};

/**
 * @brief Reliable UDP interface with acknowledgments and retransmission
 */
class IReliableUDP {
public:
    virtual ~IReliableUDP() = default;

    /**
     * @brief Initialize reliable UDP
     * @param config Configuration
     * @return Success or error code
     */
    virtual NetworkResult initialize(const ReliableUDPConfig& config) = 0;

    /**
     * @brief Close and release resources
     */
    virtual void close() = 0;

    /**
     * @brief Send message reliably (with retransmission)
     * @param data Message data
     * @param length Message length
     * @param destination Destination address
     * @param[out] sequence Assigned sequence number
     * @return Success or error code
     */
    virtual NetworkResult send_reliable(
        const UInt8* data,
        SizeT length,
        const SocketAddress& destination,
        SequenceNumber& sequence) = 0;

    /**
     * @brief Send message unreliably (fire-and-forget)
     * @param data Message data
     * @param length Message length
     * @param destination Destination address
     * @return Success or error code
     */
    virtual NetworkResult send_unreliable(
        const UInt8* data,
        SizeT length,
        const SocketAddress& destination) = 0;

    /**
     * @brief Process pending acknowledgments and retransmissions
     * @return Number of retransmissions performed
     */
    virtual SizeT process_pending() = 0;

    /**
     * @brief Receive message
     * @param buffer Receive buffer
     * @param buffer_size Buffer size
     * @param[out] result Receive result
     * @param[out] is_reliable True if message was sent reliably
     * @return Success or error code
     */
    virtual NetworkResult receive(
        UInt8* buffer,
        SizeT buffer_size,
        ReceiveResult& result,
        bool& is_reliable) = 0;

    /**
     * @brief Check if all reliable messages have been acknowledged
     */
    virtual bool all_acknowledged() const = 0;

    /**
     * @brief Get number of pending acknowledgments
     */
    virtual SizeT pending_ack_count() const = 0;
};

//==============================================================================
// Factory Functions
//==============================================================================

/**
 * @brief Create UDP socket
 */
std::unique_ptr<IUDPSocket> create_udp_socket();

/**
 * @brief Create TCP connection
 */
std::unique_ptr<ITCPConnection> create_tcp_connection();

/**
 * @brief Create TCP server
 */
std::unique_ptr<ITCPServer> create_tcp_server();

/**
 * @brief Create reliable UDP
 */
std::unique_ptr<IReliableUDP> create_reliable_udp();

//==============================================================================
// Utility Functions
//==============================================================================

/**
 * @brief Get list of local network interfaces
 * @return Vector of (interface_name, address) pairs
 */
std::vector<std::pair<std::string, SocketAddress>> get_local_interfaces();

/**
 * @brief Resolve hostname to address
 * @param hostname Hostname to resolve
 * @param version IP version preference
 * @return Resolved addresses (may be empty on failure)
 */
std::vector<SocketAddress> resolve_hostname(
    std::string_view hostname,
    IPVersion version = IPVersion::Any);

/**
 * @brief Check if port is available for binding
 * @param port Port number
 * @param udp True for UDP, false for TCP
 * @return True if port is available
 */
bool is_port_available(UInt16 port, bool udp = true);

/**
 * @brief Get an available ephemeral port
 * @return Available port number or 0 on failure
 */
UInt16 get_available_port();

/**
 * @brief Convert bytes to network byte order (big endian)
 */
inline UInt16 htons_safe(UInt16 value) {
    const UInt8* bytes = reinterpret_cast<const UInt8*>(&value);
    return (static_cast<UInt16>(bytes[0]) << 8) | bytes[1];
}

inline UInt32 htonl_safe(UInt32 value) {
    const UInt8* bytes = reinterpret_cast<const UInt8*>(&value);
    return (static_cast<UInt32>(bytes[0]) << 24) |
           (static_cast<UInt32>(bytes[1]) << 16) |
           (static_cast<UInt32>(bytes[2]) << 8) |
           bytes[3];
}

/**
 * @brief Convert bytes from network byte order (big endian)
 */
inline UInt16 ntohs_safe(UInt16 value) { return htons_safe(value); }
inline UInt32 ntohl_safe(UInt32 value) { return htonl_safe(value); }

} // namespace jaguar::federation::network
