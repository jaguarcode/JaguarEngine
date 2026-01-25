// Copyright JaguarEngine Team. All Rights Reserved.
//
// DIS UDP Socket Implementation
// Cross-platform UDP socket for DIS protocol communication

#include "jaguar/federation/dis_socket.h"
#include <cstring>
#include <algorithm>
#include <sstream>

// Platform-specific includes
#ifdef _WIN32
    #include <winsock2.h>
    #include <ws2tcpip.h>
    #pragma comment(lib, "ws2_32.lib")
    using socket_t = SOCKET;
    constexpr socket_t INVALID_SOCKET_VALUE = INVALID_SOCKET;
    constexpr int SOCKET_ERROR_VALUE = SOCKET_ERROR;
    #define GET_SOCKET_ERROR() WSAGetLastError()

    inline void close_socket(socket_t sock) { closesocket(sock); }
#else
    #include <sys/types.h>
    #include <sys/socket.h>
    #include <netinet/in.h>
    #include <arpa/inet.h>
    #include <unistd.h>
    #include <fcntl.h>
    #include <errno.h>
    #include <poll.h>
    #include <ifaddrs.h>
    #include <net/if.h>
    using socket_t = int;
    constexpr socket_t INVALID_SOCKET_VALUE = -1;
    constexpr int SOCKET_ERROR_VALUE = -1;
    #define GET_SOCKET_ERROR() errno

    inline void close_socket(socket_t sock) { ::close(sock); }
#endif

namespace jaguar::federation {

//==============================================================================
// Platform-Specific Utilities
//==============================================================================

#ifdef _WIN32
// Windows initialization helper
class WinsockInitializer {
public:
    WinsockInitializer() {
        WSADATA wsa_data;
        WSAStartup(MAKEWORD(2, 2), &wsa_data);
    }
    ~WinsockInitializer() {
        WSACleanup();
    }
};

static WinsockInitializer g_winsock_initializer;
#endif

//==============================================================================
// Error Code to String
//==============================================================================

const char* dis_socket_error_to_string(DisSocketError error) noexcept {
    switch (error) {
        case DisSocketError::Success: return "Success";
        case DisSocketError::NotInitialized: return "Socket not initialized";
        case DisSocketError::AlreadyInitialized: return "Socket already initialized";
        case DisSocketError::SocketCreationFailed: return "Failed to create socket";
        case DisSocketError::BindFailed: return "Failed to bind socket";
        case DisSocketError::InvalidConfiguration: return "Invalid configuration";
        case DisSocketError::SendFailed: return "Send operation failed";
        case DisSocketError::ReceiveFailed: return "Receive operation failed";
        case DisSocketError::WouldBlock: return "Operation would block";
        case DisSocketError::Timeout: return "Operation timed out";
        case DisSocketError::ConnectionReset: return "Connection reset by peer";
        case DisSocketError::NetworkUnreachable: return "Network unreachable";
        case DisSocketError::MulticastJoinFailed: return "Failed to join multicast group";
        case DisSocketError::MulticastLeaveFailed: return "Failed to leave multicast group";
        case DisSocketError::InvalidMulticastAddress: return "Invalid multicast address";
        case DisSocketError::InvalidAddress: return "Invalid address";
        case DisSocketError::AddressInUse: return "Address already in use";
        case DisSocketError::AddressNotAvailable: return "Address not available";
        case DisSocketError::BufferTooSmall: return "Buffer too small";
        case DisSocketError::BufferOverflow: return "Buffer overflow";
        case DisSocketError::FragmentationRequired: return "Fragmentation required";
        case DisSocketError::PayloadTooLarge: return "Payload too large";
        case DisSocketError::InvalidSocketOption: return "Invalid socket option";
        case DisSocketError::SetOptionFailed: return "Failed to set socket option";
        case DisSocketError::GetOptionFailed: return "Failed to get socket option";
        case DisSocketError::PlatformNotSupported: return "Platform not supported";
        case DisSocketError::PermissionDenied: return "Permission denied";
        case DisSocketError::InternalError: return "Internal error";
        case DisSocketError::Unknown: return "Unknown error";
        default: return "Unrecognized error";
    }
}

//==============================================================================
// DisEndpoint Implementation
//==============================================================================

std::optional<DisEndpoint> DisEndpoint::parse(std::string_view str) noexcept {
    auto colon_pos = str.find(':');
    if (colon_pos == std::string_view::npos) {
        return std::nullopt;
    }

    try {
        std::string addr(str.substr(0, colon_pos));
        std::string port_str(str.substr(colon_pos + 1));
        UInt16 port = static_cast<UInt16>(std::stoul(port_str));
        return DisEndpoint(addr, port);
    } catch (...) {
        return std::nullopt;
    }
}

std::string DisEndpoint::to_string() const {
    std::ostringstream oss;
    oss << address << ":" << port;
    return oss.str();
}

bool DisEndpoint::is_multicast() const noexcept {
    struct in_addr addr;
    if (inet_pton(AF_INET, address.c_str(), &addr) != 1) {
        return false;
    }
    UInt32 ip = ntohl(addr.s_addr);
    // Multicast range: 224.0.0.0 to 239.255.255.255
    return (ip >= 0xE0000000) && (ip <= 0xEFFFFFFF);
}

bool DisEndpoint::is_broadcast() const noexcept {
    return address == "255.255.255.255";
}

bool DisEndpoint::is_loopback() const noexcept {
    struct in_addr addr;
    if (inet_pton(AF_INET, address.c_str(), &addr) != 1) {
        return false;
    }
    UInt32 ip = ntohl(addr.s_addr);
    // Loopback range: 127.0.0.0/8
    return (ip & 0xFF000000) == 0x7F000000;
}

bool DisEndpoint::operator==(const DisEndpoint& other) const noexcept {
    return address == other.address && port == other.port;
}

//==============================================================================
// DisSocket Implementation
//==============================================================================

struct DisSocket::Impl {
    socket_t socket{INVALID_SOCKET_VALUE};
    DisSocketConfig config;
    DisSocketStats stats;
    std::vector<DisEndpoint> multicast_groups;
    bool initialized{false};

    ~Impl() {
        if (socket != INVALID_SOCKET_VALUE) {
            close_socket(socket);
        }
    }
};

DisSocket::DisSocket() : impl_(std::make_unique<Impl>()) {}

DisSocket::~DisSocket() {
    close();
}

DisSocket::DisSocket(DisSocket&& other) noexcept
    : impl_(std::move(other.impl_)), mutex_() {}

DisSocket& DisSocket::operator=(DisSocket&& other) noexcept {
    if (this != &other) {
        std::lock_guard<std::mutex> lock1(mutex_);
        std::lock_guard<std::mutex> lock2(other.mutex_);
        close();
        impl_ = std::move(other.impl_);
    }
    return *this;
}

DisSocketError DisSocket::initialize(const DisSocketConfig& config) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (impl_->initialized) {
        return DisSocketError::AlreadyInitialized;
    }

    // Create socket
    impl_->socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (impl_->socket == INVALID_SOCKET_VALUE) {
        return DisSocketError::SocketCreationFailed;
    }

    // Set socket options
    if (config.reuse_address) {
        int reuse = 1;
        if (setsockopt(impl_->socket, SOL_SOCKET, SO_REUSEADDR,
                       reinterpret_cast<const char*>(&reuse), sizeof(reuse)) == SOCKET_ERROR_VALUE) {
            close_socket(impl_->socket);
            impl_->socket = INVALID_SOCKET_VALUE;
            return DisSocketError::SetOptionFailed;
        }
    }

#ifndef _WIN32
    if (config.reuse_port) {
        int reuse = 1;
        if (setsockopt(impl_->socket, SOL_SOCKET, SO_REUSEPORT,
                       &reuse, sizeof(reuse)) == SOCKET_ERROR_VALUE) {
            // SO_REUSEPORT not supported on all platforms, continue anyway
        }
    }
#endif

    // Set receive buffer size
    if (config.receive_buffer_size > 0) {
        int buffer_size = static_cast<int>(config.receive_buffer_size);
        setsockopt(impl_->socket, SOL_SOCKET, SO_RCVBUF,
                   reinterpret_cast<const char*>(&buffer_size), sizeof(buffer_size));
    }

    // Set send buffer size
    if (config.send_buffer_size > 0) {
        int buffer_size = static_cast<int>(config.send_buffer_size);
        setsockopt(impl_->socket, SOL_SOCKET, SO_SNDBUF,
                   reinterpret_cast<const char*>(&buffer_size), sizeof(buffer_size));
    }

    // Enable broadcast if requested
    if (config.broadcast_enabled) {
        int broadcast = 1;
        if (setsockopt(impl_->socket, SOL_SOCKET, SO_BROADCAST,
                       reinterpret_cast<const char*>(&broadcast), sizeof(broadcast)) == SOCKET_ERROR_VALUE) {
            close_socket(impl_->socket);
            impl_->socket = INVALID_SOCKET_VALUE;
            return DisSocketError::SetOptionFailed;
        }
    }

    // Set multicast TTL
    int ttl = config.multicast_ttl;
    setsockopt(impl_->socket, IPPROTO_IP, IP_MULTICAST_TTL,
               reinterpret_cast<const char*>(&ttl), sizeof(ttl));

    // Set multicast loopback
    int loopback = config.multicast_loopback ? 1 : 0;
    setsockopt(impl_->socket, IPPROTO_IP, IP_MULTICAST_LOOP,
               reinterpret_cast<const char*>(&loopback), sizeof(loopback));

    // Set non-blocking mode if receive timeout is 0
    if (config.receive_timeout.count() == 0) {
#ifdef _WIN32
        u_long mode = 1;
        ioctlsocket(impl_->socket, FIONBIO, &mode);
#else
        int flags = fcntl(impl_->socket, F_GETFL, 0);
        fcntl(impl_->socket, F_SETFL, flags | O_NONBLOCK);
#endif
    } else {
        // Set receive timeout
#ifdef _WIN32
        DWORD timeout = static_cast<DWORD>(config.receive_timeout.count());
#else
        struct timeval timeout;
        timeout.tv_sec = config.receive_timeout.count() / 1000;
        timeout.tv_usec = (config.receive_timeout.count() % 1000) * 1000;
#endif
        setsockopt(impl_->socket, SOL_SOCKET, SO_RCVTIMEO,
                   reinterpret_cast<const char*>(&timeout), sizeof(timeout));
    }

    // Set send timeout
    if (config.send_timeout.count() > 0) {
#ifdef _WIN32
        DWORD timeout = static_cast<DWORD>(config.send_timeout.count());
#else
        struct timeval timeout;
        timeout.tv_sec = config.send_timeout.count() / 1000;
        timeout.tv_usec = (config.send_timeout.count() % 1000) * 1000;
#endif
        setsockopt(impl_->socket, SOL_SOCKET, SO_SNDTIMEO,
                   reinterpret_cast<const char*>(&timeout), sizeof(timeout));
    }

    // Bind socket
    struct sockaddr_in bind_addr;
    std::memset(&bind_addr, 0, sizeof(bind_addr));
    bind_addr.sin_family = AF_INET;
    bind_addr.sin_port = htons(config.bind_endpoint.port);

    if (inet_pton(AF_INET, config.bind_endpoint.address.c_str(), &bind_addr.sin_addr) != 1) {
        close_socket(impl_->socket);
        impl_->socket = INVALID_SOCKET_VALUE;
        return DisSocketError::InvalidAddress;
    }

    if (bind(impl_->socket, reinterpret_cast<struct sockaddr*>(&bind_addr),
             sizeof(bind_addr)) == SOCKET_ERROR_VALUE) {
        int error = GET_SOCKET_ERROR();
        close_socket(impl_->socket);
        impl_->socket = INVALID_SOCKET_VALUE;

#ifdef _WIN32
        if (error == WSAEADDRINUSE) return DisSocketError::AddressInUse;
#else
        if (error == EADDRINUSE) return DisSocketError::AddressInUse;
        if (error == EACCES) return DisSocketError::PermissionDenied;
#endif
        return DisSocketError::BindFailed;
    }

    impl_->config = config;
    impl_->initialized = true;
    return DisSocketError::Success;
}

void DisSocket::close() {
    std::lock_guard<std::mutex> lock(mutex_);

    if (impl_->socket != INVALID_SOCKET_VALUE) {
        // Leave all multicast groups
        for (const auto& group : impl_->multicast_groups) {
            leave_multicast_group(group);
        }
        impl_->multicast_groups.clear();

        close_socket(impl_->socket);
        impl_->socket = INVALID_SOCKET_VALUE;
        impl_->initialized = false;
    }
}

bool DisSocket::is_open() const noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    return impl_->initialized && impl_->socket != INVALID_SOCKET_VALUE;
}

DisSocketError DisSocket::join_multicast_group(const DisEndpoint& group,
                                                const std::string& interface_address) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!impl_->initialized) {
        return DisSocketError::NotInitialized;
    }

    if (!group.is_multicast()) {
        return DisSocketError::InvalidMulticastAddress;
    }

    struct ip_mreq mreq;
    std::memset(&mreq, 0, sizeof(mreq));

    if (inet_pton(AF_INET, group.address.c_str(), &mreq.imr_multiaddr) != 1) {
        return DisSocketError::InvalidAddress;
    }

    if (interface_address.empty() || interface_address == "0.0.0.0") {
        mreq.imr_interface.s_addr = htonl(INADDR_ANY);
    } else {
        if (inet_pton(AF_INET, interface_address.c_str(), &mreq.imr_interface) != 1) {
            return DisSocketError::InvalidAddress;
        }
    }

    if (setsockopt(impl_->socket, IPPROTO_IP, IP_ADD_MEMBERSHIP,
                   reinterpret_cast<const char*>(&mreq), sizeof(mreq)) == SOCKET_ERROR_VALUE) {
        return DisSocketError::MulticastJoinFailed;
    }

    impl_->multicast_groups.push_back(group);
    return DisSocketError::Success;
}

DisSocketError DisSocket::leave_multicast_group(const DisEndpoint& group) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!impl_->initialized) {
        return DisSocketError::NotInitialized;
    }

    struct ip_mreq mreq;
    std::memset(&mreq, 0, sizeof(mreq));

    if (inet_pton(AF_INET, group.address.c_str(), &mreq.imr_multiaddr) != 1) {
        return DisSocketError::InvalidAddress;
    }

    mreq.imr_interface.s_addr = htonl(INADDR_ANY);

    if (setsockopt(impl_->socket, IPPROTO_IP, IP_DROP_MEMBERSHIP,
                   reinterpret_cast<const char*>(&mreq), sizeof(mreq)) == SOCKET_ERROR_VALUE) {
        return DisSocketError::MulticastLeaveFailed;
    }

    // Remove from tracked groups
    impl_->multicast_groups.erase(
        std::remove(impl_->multicast_groups.begin(), impl_->multicast_groups.end(), group),
        impl_->multicast_groups.end()
    );

    return DisSocketError::Success;
}

void DisSocket::leave_all_multicast_groups() {
    std::lock_guard<std::mutex> lock(mutex_);

    auto groups = impl_->multicast_groups; // Copy
    for (const auto& group : groups) {
        leave_multicast_group(group);
    }
}

DisSocketError DisSocket::send_to(const UInt8* data, SizeT length, const DisEndpoint& destination) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!impl_->initialized) {
        return DisSocketError::NotInitialized;
    }

    if (length > impl_->config.mtu - IP_HEADER_SIZE - UDP_HEADER_SIZE) {
        if (impl_->config.auto_fragment) {
            // Delegate to fragmented send
            return send_fragmented(data, length, destination);
        }
        return DisSocketError::PayloadTooLarge;
    }

    struct sockaddr_in dest_addr;
    std::memset(&dest_addr, 0, sizeof(dest_addr));
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(destination.port);

    if (inet_pton(AF_INET, destination.address.c_str(), &dest_addr.sin_addr) != 1) {
        return DisSocketError::InvalidAddress;
    }

    int sent = sendto(impl_->socket, reinterpret_cast<const char*>(data), static_cast<int>(length), 0,
                      reinterpret_cast<struct sockaddr*>(&dest_addr), sizeof(dest_addr));

    if (sent == SOCKET_ERROR_VALUE) {
        impl_->stats.send_errors++;
        int error = GET_SOCKET_ERROR();

#ifdef _WIN32
        if (error == WSAEWOULDBLOCK) return DisSocketError::WouldBlock;
        if (error == WSAEMSGSIZE) return DisSocketError::PayloadTooLarge;
        if (error == WSAENETUNREACH) return DisSocketError::NetworkUnreachable;
#else
        if (error == EWOULDBLOCK || error == EAGAIN) return DisSocketError::WouldBlock;
        if (error == EMSGSIZE) return DisSocketError::PayloadTooLarge;
        if (error == ENETUNREACH) return DisSocketError::NetworkUnreachable;
#endif
        return DisSocketError::SendFailed;
    }

    impl_->stats.packets_sent++;
    impl_->stats.bytes_sent += sent;
    impl_->stats.last_send = std::chrono::steady_clock::now();

    if (destination.is_multicast()) {
        impl_->stats.multicast_packets++;
    } else if (destination.is_broadcast()) {
        impl_->stats.broadcast_packets++;
    }

    return DisSocketError::Success;
}

DisSocketError DisSocket::send_to_multiple(const UInt8* data, SizeT length,
                                            const DisEndpoint* destinations,
                                            SizeT num_destinations) {
    DisSocketError last_error = DisSocketError::Success;

    for (SizeT i = 0; i < num_destinations; ++i) {
        auto error = send_to(data, length, destinations[i]);
        if (error != DisSocketError::Success) {
            last_error = error;
        }
    }

    return last_error;
}

DisSocketError DisSocket::send_fragmented(const UInt8* data, SizeT length,
                                          const DisEndpoint& destination) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!impl_->initialized) {
        return DisSocketError::NotInitialized;
    }

    SizeT max_fragment_size = impl_->config.mtu - IP_HEADER_SIZE - UDP_HEADER_SIZE;
    SizeT num_fragments = (length + max_fragment_size - 1) / max_fragment_size;

    for (SizeT i = 0; i < num_fragments; ++i) {
        SizeT offset = i * max_fragment_size;
        SizeT fragment_length = std::min(max_fragment_size, length - offset);

        auto error = send_to(data + offset, fragment_length, destination);
        if (error != DisSocketError::Success) {
            return error;
        }
    }

    impl_->stats.fragments_sent += num_fragments;
    return DisSocketError::Success;
}

DisSocketError DisSocket::receive_from(UInt8* buffer, SizeT buffer_size, DisReceiveResult& result) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!impl_->initialized) {
        return DisSocketError::NotInitialized;
    }

    struct sockaddr_in src_addr;
    socklen_t addr_len = sizeof(src_addr);
    std::memset(&src_addr, 0, sizeof(src_addr));

    int received = recvfrom(impl_->socket, reinterpret_cast<char*>(buffer),
                            static_cast<int>(buffer_size), 0,
                            reinterpret_cast<struct sockaddr*>(&src_addr), &addr_len);

    if (received == SOCKET_ERROR_VALUE) {
        impl_->stats.receive_errors++;
        int error = GET_SOCKET_ERROR();

#ifdef _WIN32
        if (error == WSAEWOULDBLOCK) return DisSocketError::WouldBlock;
        if (error == WSAECONNRESET) return DisSocketError::ConnectionReset;
        if (error == WSAETIMEDOUT) return DisSocketError::Timeout;
#else
        if (error == EWOULDBLOCK || error == EAGAIN) return DisSocketError::WouldBlock;
        if (error == ECONNRESET) return DisSocketError::ConnectionReset;
        if (error == ETIMEDOUT) return DisSocketError::Timeout;
#endif
        return DisSocketError::ReceiveFailed;
    }

    // Convert source address
    char addr_str[INET_ADDRSTRLEN];
    inet_ntop(AF_INET, &src_addr.sin_addr, addr_str, sizeof(addr_str));

    result.bytes_received = static_cast<SizeT>(received);
    result.source = DisEndpoint(addr_str, ntohs(src_addr.sin_port));
    result.truncated = false; // UDP doesn't indicate truncation in recvfrom
    result.timestamp = std::chrono::steady_clock::now();

    impl_->stats.packets_received++;
    impl_->stats.bytes_received += received;
    impl_->stats.last_receive = result.timestamp;

    return DisSocketError::Success;
}

bool DisSocket::poll_readable(std::chrono::milliseconds timeout) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!impl_->initialized) {
        return false;
    }

#ifdef _WIN32
    fd_set read_fds;
    FD_ZERO(&read_fds);
    FD_SET(impl_->socket, &read_fds);

    struct timeval tv;
    tv.tv_sec = timeout.count() / 1000;
    tv.tv_usec = (timeout.count() % 1000) * 1000;

    int result = select(0, &read_fds, nullptr, nullptr, &tv);
    return result > 0;
#else
    struct pollfd pfd;
    pfd.fd = impl_->socket;
    pfd.events = POLLIN;
    pfd.revents = 0;

    int result = poll(&pfd, 1, static_cast<int>(timeout.count()));
    return result > 0 && (pfd.revents & POLLIN);
#endif
}

DisEndpoint DisSocket::get_local_endpoint() const {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!impl_->initialized) {
        return DisEndpoint();
    }

    struct sockaddr_in addr;
    socklen_t addr_len = sizeof(addr);
    std::memset(&addr, 0, sizeof(addr));

    if (getsockname(impl_->socket, reinterpret_cast<struct sockaddr*>(&addr), &addr_len) == SOCKET_ERROR_VALUE) {
        return DisEndpoint();
    }

    char addr_str[INET_ADDRSTRLEN];
    inet_ntop(AF_INET, &addr.sin_addr, addr_str, sizeof(addr_str));

    return DisEndpoint(addr_str, ntohs(addr.sin_port));
}

DisSocketError DisSocket::set_receive_timeout(std::chrono::milliseconds timeout) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!impl_->initialized) {
        return DisSocketError::NotInitialized;
    }

#ifdef _WIN32
    DWORD tv = static_cast<DWORD>(timeout.count());
#else
    struct timeval tv;
    tv.tv_sec = timeout.count() / 1000;
    tv.tv_usec = (timeout.count() % 1000) * 1000;
#endif

    if (setsockopt(impl_->socket, SOL_SOCKET, SO_RCVTIMEO,
                   reinterpret_cast<const char*>(&tv), sizeof(tv)) == SOCKET_ERROR_VALUE) {
        return DisSocketError::SetOptionFailed;
    }

    impl_->config.receive_timeout = timeout;
    return DisSocketError::Success;
}

DisSocketError DisSocket::set_send_timeout(std::chrono::milliseconds timeout) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!impl_->initialized) {
        return DisSocketError::NotInitialized;
    }

#ifdef _WIN32
    DWORD tv = static_cast<DWORD>(timeout.count());
#else
    struct timeval tv;
    tv.tv_sec = timeout.count() / 1000;
    tv.tv_usec = (timeout.count() % 1000) * 1000;
#endif

    if (setsockopt(impl_->socket, SOL_SOCKET, SO_SNDTIMEO,
                   reinterpret_cast<const char*>(&tv), sizeof(tv)) == SOCKET_ERROR_VALUE) {
        return DisSocketError::SetOptionFailed;
    }

    impl_->config.send_timeout = timeout;
    return DisSocketError::Success;
}

DisSocketError DisSocket::set_multicast_ttl(UInt8 ttl) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!impl_->initialized) {
        return DisSocketError::NotInitialized;
    }

    int ttl_val = ttl;
    if (setsockopt(impl_->socket, IPPROTO_IP, IP_MULTICAST_TTL,
                   reinterpret_cast<const char*>(&ttl_val), sizeof(ttl_val)) == SOCKET_ERROR_VALUE) {
        return DisSocketError::SetOptionFailed;
    }

    impl_->config.multicast_ttl = ttl;
    return DisSocketError::Success;
}

DisSocketError DisSocket::set_broadcast(bool enabled) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!impl_->initialized) {
        return DisSocketError::NotInitialized;
    }

    int broadcast = enabled ? 1 : 0;
    if (setsockopt(impl_->socket, SOL_SOCKET, SO_BROADCAST,
                   reinterpret_cast<const char*>(&broadcast), sizeof(broadcast)) == SOCKET_ERROR_VALUE) {
        return DisSocketError::SetOptionFailed;
    }

    impl_->config.broadcast_enabled = enabled;
    return DisSocketError::Success;
}

SizeT DisSocket::get_mtu() const noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    return impl_->config.mtu;
}

void DisSocket::set_mtu(SizeT mtu) noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    impl_->config.mtu = mtu;
}

const DisSocketStats& DisSocket::get_stats() const noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    return impl_->stats;
}

void DisSocket::reset_stats() noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    impl_->stats.reset();
}

std::string DisSocket::get_last_error_message() const {
    int error = GET_SOCKET_ERROR();

#ifdef _WIN32
    char* msg_buf = nullptr;
    FormatMessageA(FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS,
                   nullptr, error, MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
                   reinterpret_cast<LPSTR>(&msg_buf), 0, nullptr);
    std::string message = msg_buf ? msg_buf : "Unknown error";
    LocalFree(msg_buf);
    return message;
#else
    return std::strerror(error);
#endif
}

intptr_t DisSocket::get_native_handle() const noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    return static_cast<intptr_t>(impl_->socket);
}

//==============================================================================
// Utility Functions
//==============================================================================

std::vector<std::pair<std::string, std::string>> get_network_interfaces() {
    std::vector<std::pair<std::string, std::string>> interfaces;

#ifndef _WIN32
    struct ifaddrs* if_addrs = nullptr;
    if (getifaddrs(&if_addrs) != 0) {
        return interfaces;
    }

    for (struct ifaddrs* ifa = if_addrs; ifa != nullptr; ifa = ifa->ifa_next) {
        if (ifa->ifa_addr == nullptr || ifa->ifa_addr->sa_family != AF_INET) {
            continue;
        }

        struct sockaddr_in* addr = reinterpret_cast<struct sockaddr_in*>(ifa->ifa_addr);
        char addr_str[INET_ADDRSTRLEN];
        inet_ntop(AF_INET, &addr->sin_addr, addr_str, sizeof(addr_str));

        interfaces.emplace_back(ifa->ifa_name, addr_str);
    }

    freeifaddrs(if_addrs);
#endif

    return interfaces;
}

std::optional<std::string> get_default_interface_address() {
    auto interfaces = get_network_interfaces();
    for (const auto& [name, address] : interfaces) {
        // Skip loopback
        if (address.substr(0, 3) != "127") {
            return address;
        }
    }
    return std::nullopt;
}

bool is_valid_ipv4(std::string_view address) noexcept {
    struct in_addr addr;
    return inet_pton(AF_INET, std::string(address).c_str(), &addr) == 1;
}

bool is_port_available(UInt16 port) {
    socket_t test_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (test_socket == INVALID_SOCKET_VALUE) {
        return false;
    }

    struct sockaddr_in addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(port);

    bool available = bind(test_socket, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) != SOCKET_ERROR_VALUE;
    close_socket(test_socket);

    return available;
}

} // namespace jaguar::federation
