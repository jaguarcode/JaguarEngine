/**
 * @file network_transport.cpp
 * @brief Network transport layer implementation
 */

#include "jaguar/federation/network_transport.h"
#include <cstring>
#include <algorithm>
#include <sstream>
#include <regex>

#ifdef _WIN32
    #include <winsock2.h>
    #include <ws2tcpip.h>
    #pragma comment(lib, "ws2_32.lib")
    using socket_t = SOCKET;
    #define INVALID_SOCKET_VALUE INVALID_SOCKET
    #define SOCKET_ERROR_VALUE SOCKET_ERROR
    #define close_socket closesocket
    #define socklen_t int
#else
    #include <sys/socket.h>
    #include <sys/types.h>
    #include <netinet/in.h>
    #include <netinet/tcp.h>
    #include <arpa/inet.h>
    #include <unistd.h>
    #include <fcntl.h>
    #include <poll.h>
    #include <netdb.h>
    #include <ifaddrs.h>
    #include <errno.h>
    using socket_t = int;
    #define INVALID_SOCKET_VALUE (-1)
    #define SOCKET_ERROR_VALUE (-1)
    #define close_socket ::close
#endif

namespace jaguar::federation::network {

//==============================================================================
// NetworkResult String Conversion
//==============================================================================

const char* network_result_to_string(NetworkResult result) {
    switch (result) {
        case NetworkResult::Success: return "Success";
        case NetworkResult::SocketCreationFailed: return "SocketCreationFailed";
        case NetworkResult::BindFailed: return "BindFailed";
        case NetworkResult::ConnectFailed: return "ConnectFailed";
        case NetworkResult::ListenFailed: return "ListenFailed";
        case NetworkResult::AcceptFailed: return "AcceptFailed";
        case NetworkResult::AlreadyConnected: return "AlreadyConnected";
        case NetworkResult::NotConnected: return "NotConnected";
        case NetworkResult::InvalidAddress: return "InvalidAddress";
        case NetworkResult::AddressInUse: return "AddressInUse";
        case NetworkResult::AddressNotAvailable: return "AddressNotAvailable";
        case NetworkResult::MulticastJoinFailed: return "MulticastJoinFailed";
        case NetworkResult::MulticastLeaveFailed: return "MulticastLeaveFailed";
        case NetworkResult::InvalidMulticastGroup: return "InvalidMulticastGroup";
        case NetworkResult::SendFailed: return "SendFailed";
        case NetworkResult::ReceiveFailed: return "ReceiveFailed";
        case NetworkResult::WouldBlock: return "WouldBlock";
        case NetworkResult::Timeout: return "Timeout";
        case NetworkResult::ConnectionReset: return "ConnectionReset";
        case NetworkResult::ConnectionClosed: return "ConnectionClosed";
        case NetworkResult::BufferTooSmall: return "BufferTooSmall";
        case NetworkResult::BufferOverflow: return "BufferOverflow";
        case NetworkResult::InvalidConfiguration: return "InvalidConfiguration";
        case NetworkResult::InvalidOption: return "InvalidOption";
        case NetworkResult::NotInitialized: return "NotInitialized";
        case NetworkResult::AlreadyInitialized: return "AlreadyInitialized";
        case NetworkResult::OperationAborted: return "OperationAborted";
        case NetworkResult::InternalError: return "InternalError";
        default: return "Unknown";
    }
}

//==============================================================================
// SocketAddress Implementation
//==============================================================================

std::optional<SocketAddress> SocketAddress::parse(std::string_view address) {
    std::string addr_str(address);

    // Check for IPv6 format: [host]:port
    if (addr_str.front() == '[') {
        auto close_bracket = addr_str.find(']');
        if (close_bracket == std::string::npos) {
            return std::nullopt;
        }

        std::string host = addr_str.substr(1, close_bracket - 1);

        if (close_bracket + 1 < addr_str.size() && addr_str[close_bracket + 1] == ':') {
            try {
                UInt16 port = static_cast<UInt16>(std::stoul(addr_str.substr(close_bracket + 2)));
                return SocketAddress(host, port, IPVersion::IPv6);
            } catch (...) {
                return std::nullopt;
            }
        } else {
            return SocketAddress(host, 0, IPVersion::IPv6);
        }
    }

    // IPv4 format: host:port
    auto colon = addr_str.rfind(':');
    if (colon != std::string::npos) {
        std::string host = addr_str.substr(0, colon);
        try {
            UInt16 port = static_cast<UInt16>(std::stoul(addr_str.substr(colon + 1)));
            return SocketAddress(host, port, IPVersion::IPv4);
        } catch (...) {
            return std::nullopt;
        }
    }

    // Just host, no port
    return SocketAddress(addr_str, 0, IPVersion::IPv4);
}

std::string SocketAddress::to_string() const {
    std::ostringstream oss;
    if (version == IPVersion::IPv6) {
        oss << "[" << host << "]:" << port;
    } else {
        oss << host << ":" << port;
    }
    return oss.str();
}

bool SocketAddress::is_multicast() const {
    if (version == IPVersion::IPv6) {
        return !host.empty() && host[0] == 'f' && host[1] == 'f';
    }

    // IPv4 multicast: 224.0.0.0 - 239.255.255.255
    if (host.empty()) return false;

    struct in_addr addr;
    if (inet_pton(AF_INET, host.c_str(), &addr) != 1) {
        return false;
    }

    UInt32 ip = ntohl(addr.s_addr);
    return (ip & 0xF0000000) == 0xE0000000;  // 224.0.0.0/4
}

bool SocketAddress::is_loopback() const {
    if (version == IPVersion::IPv6) {
        return host == "::1" || host == "0:0:0:0:0:0:0:1";
    }
    return host == "127.0.0.1" || host.substr(0, 4) == "127.";
}

bool SocketAddress::is_broadcast() const {
    return host == "255.255.255.255";
}

bool SocketAddress::operator==(const SocketAddress& other) const {
    return host == other.host && port == other.port && version == other.version;
}

//==============================================================================
// UDP Socket Implementation
//==============================================================================

class UDPSocketImpl : public IUDPSocket {
public:
    UDPSocketImpl() = default;

    ~UDPSocketImpl() override {
        close();
    }

    NetworkResult initialize(const UDPSocketConfig& config) override {
        if (socket_ != INVALID_SOCKET_VALUE) {
            return NetworkResult::AlreadyInitialized;
        }

        config_ = config;

        // Create socket
        int af = (config.bind_address.version == IPVersion::IPv6) ? AF_INET6 : AF_INET;
        socket_ = socket(af, SOCK_DGRAM, IPPROTO_UDP);
        if (socket_ == INVALID_SOCKET_VALUE) {
            return NetworkResult::SocketCreationFailed;
        }

        // Set socket options
        if (config.reuse_address) {
            int opt = 1;
            setsockopt(socket_, SOL_SOCKET, SO_REUSEADDR,
                       reinterpret_cast<const char*>(&opt), sizeof(opt));
        }

#ifdef SO_REUSEPORT
        if (config.reuse_port) {
            int opt = 1;
            setsockopt(socket_, SOL_SOCKET, SO_REUSEPORT,
                       reinterpret_cast<const char*>(&opt), sizeof(opt));
        }
#endif

        if (config.broadcast_enabled) {
            int opt = 1;
            setsockopt(socket_, SOL_SOCKET, SO_BROADCAST,
                       reinterpret_cast<const char*>(&opt), sizeof(opt));
        }

        // Set buffer sizes
        if (config.receive_buffer_size > 0) {
            int size = static_cast<int>(config.receive_buffer_size);
            setsockopt(socket_, SOL_SOCKET, SO_RCVBUF,
                       reinterpret_cast<const char*>(&size), sizeof(size));
        }

        if (config.send_buffer_size > 0) {
            int size = static_cast<int>(config.send_buffer_size);
            setsockopt(socket_, SOL_SOCKET, SO_SNDBUF,
                       reinterpret_cast<const char*>(&size), sizeof(size));
        }

        // Set multicast options
        if (config.multicast_ttl != DEFAULT_MULTICAST_TTL) {
            set_multicast_ttl(config.multicast_ttl);
        }

        if (!config.multicast_loopback) {
            UInt8 loop = 0;
            setsockopt(socket_, IPPROTO_IP, IP_MULTICAST_LOOP,
                       reinterpret_cast<const char*>(&loop), sizeof(loop));
        }

        // Bind socket
        if (af == AF_INET) {
            struct sockaddr_in addr;
            std::memset(&addr, 0, sizeof(addr));
            addr.sin_family = AF_INET;
            addr.sin_port = htons(config.bind_address.port);

            if (config.bind_address.host.empty() || config.bind_address.host == "0.0.0.0") {
                addr.sin_addr.s_addr = INADDR_ANY;
            } else {
                inet_pton(AF_INET, config.bind_address.host.c_str(), &addr.sin_addr);
            }

            if (bind(socket_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
                close();
                return NetworkResult::BindFailed;
            }
        } else {
            struct sockaddr_in6 addr;
            std::memset(&addr, 0, sizeof(addr));
            addr.sin6_family = AF_INET6;
            addr.sin6_port = htons(config.bind_address.port);

            if (config.bind_address.host.empty() || config.bind_address.host == "::") {
                addr.sin6_addr = in6addr_any;
            } else {
                inet_pton(AF_INET6, config.bind_address.host.c_str(), &addr.sin6_addr);
            }

            if (bind(socket_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
                close();
                return NetworkResult::BindFailed;
            }
        }

        // Set non-blocking if no timeout
        if (config.receive_timeout.count() == 0) {
#ifdef _WIN32
            u_long mode = 1;
            ioctlsocket(socket_, FIONBIO, &mode);
#else
            int flags = fcntl(socket_, F_GETFL, 0);
            fcntl(socket_, F_SETFL, flags | O_NONBLOCK);
#endif
        }

        return NetworkResult::Success;
    }

    void close() override {
        if (socket_ != INVALID_SOCKET_VALUE) {
            close_socket(socket_);
            socket_ = INVALID_SOCKET_VALUE;
        }
    }

    bool is_open() const override {
        return socket_ != INVALID_SOCKET_VALUE;
    }

    NetworkResult join_multicast_group(
        const SocketAddress& group,
        const std::string& interface_address) override {

        if (!is_open()) return NetworkResult::NotInitialized;
        if (!group.is_multicast()) return NetworkResult::InvalidMulticastGroup;

        struct ip_mreq mreq;
        inet_pton(AF_INET, group.host.c_str(), &mreq.imr_multiaddr);

        if (interface_address.empty()) {
            mreq.imr_interface.s_addr = INADDR_ANY;
        } else {
            inet_pton(AF_INET, interface_address.c_str(), &mreq.imr_interface);
        }

        if (setsockopt(socket_, IPPROTO_IP, IP_ADD_MEMBERSHIP,
                       reinterpret_cast<const char*>(&mreq), sizeof(mreq)) < 0) {
            return NetworkResult::MulticastJoinFailed;
        }

        return NetworkResult::Success;
    }

    NetworkResult leave_multicast_group(const SocketAddress& group) override {
        if (!is_open()) return NetworkResult::NotInitialized;
        if (!group.is_multicast()) return NetworkResult::InvalidMulticastGroup;

        struct ip_mreq mreq;
        inet_pton(AF_INET, group.host.c_str(), &mreq.imr_multiaddr);
        mreq.imr_interface.s_addr = INADDR_ANY;

        if (setsockopt(socket_, IPPROTO_IP, IP_DROP_MEMBERSHIP,
                       reinterpret_cast<const char*>(&mreq), sizeof(mreq)) < 0) {
            return NetworkResult::MulticastLeaveFailed;
        }

        return NetworkResult::Success;
    }

    Int64 send_to(
        const UInt8* data,
        SizeT length,
        const SocketAddress& destination) override {

        if (!is_open()) return -1;

        struct sockaddr_in addr;
        std::memset(&addr, 0, sizeof(addr));
        addr.sin_family = AF_INET;
        addr.sin_port = htons(destination.port);
        inet_pton(AF_INET, destination.host.c_str(), &addr.sin_addr);

        Int64 sent = sendto(socket_,
                            reinterpret_cast<const char*>(data),
                            static_cast<int>(length),
                            0,
                            reinterpret_cast<struct sockaddr*>(&addr),
                            sizeof(addr));

        if (sent > 0) {
            stats_.bytes_sent += sent;
            stats_.packets_sent++;
            stats_.last_send_time = std::chrono::steady_clock::now();
        } else {
            stats_.send_errors++;
        }

        return sent;
    }

    Int64 send_to_gather(
        const IOBuffer* buffers,
        SizeT buffer_count,
        const SocketAddress& destination) override {

        // Simple implementation: concatenate buffers
        std::vector<UInt8> combined;
        for (SizeT i = 0; i < buffer_count; ++i) {
            combined.insert(combined.end(),
                           buffers[i].data,
                           buffers[i].data + buffers[i].length);
        }
        return send_to(combined.data(), combined.size(), destination);
    }

    NetworkResult receive_from(
        UInt8* buffer,
        SizeT buffer_size,
        ReceiveResult& result) override {

        if (!is_open()) return NetworkResult::NotInitialized;

        struct sockaddr_in addr;
        socklen_t addr_len = sizeof(addr);

        Int64 received = recvfrom(socket_,
                                  reinterpret_cast<char*>(buffer),
                                  static_cast<int>(buffer_size),
                                  0,
                                  reinterpret_cast<struct sockaddr*>(&addr),
                                  &addr_len);

        if (received < 0) {
#ifdef _WIN32
            int err = WSAGetLastError();
            if (err == WSAEWOULDBLOCK) {
                return NetworkResult::WouldBlock;
            }
#else
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                return NetworkResult::WouldBlock;
            }
#endif
            stats_.receive_errors++;
            return NetworkResult::ReceiveFailed;
        }

        if (received == 0) {
            return NetworkResult::ConnectionClosed;
        }

        result.bytes_received = static_cast<SizeT>(received);
        result.truncated = false;

        char host_buf[INET_ADDRSTRLEN];
        inet_ntop(AF_INET, &addr.sin_addr, host_buf, sizeof(host_buf));
        result.source = SocketAddress(host_buf, ntohs(addr.sin_port));

        stats_.bytes_received += received;
        stats_.packets_received++;
        stats_.last_receive_time = std::chrono::steady_clock::now();

        return NetworkResult::Success;
    }

    bool poll_readable(std::chrono::milliseconds timeout) override {
        if (!is_open()) return false;

#ifdef _WIN32
        fd_set read_fds;
        FD_ZERO(&read_fds);
        FD_SET(socket_, &read_fds);

        struct timeval tv;
        tv.tv_sec = static_cast<long>(timeout.count() / 1000);
        tv.tv_usec = static_cast<long>((timeout.count() % 1000) * 1000);

        return select(0, &read_fds, nullptr, nullptr, &tv) > 0;
#else
        struct pollfd pfd;
        pfd.fd = socket_;
        pfd.events = POLLIN;

        return poll(&pfd, 1, static_cast<int>(timeout.count())) > 0;
#endif
    }

    SocketAddress get_local_address() const override {
        if (!is_open()) return SocketAddress();

        struct sockaddr_in addr;
        socklen_t addr_len = sizeof(addr);

        if (getsockname(socket_, reinterpret_cast<struct sockaddr*>(&addr), &addr_len) < 0) {
            return SocketAddress();
        }

        char host_buf[INET_ADDRSTRLEN];
        inet_ntop(AF_INET, &addr.sin_addr, host_buf, sizeof(host_buf));
        return SocketAddress(host_buf, ntohs(addr.sin_port));
    }

    NetworkResult set_receive_timeout(std::chrono::milliseconds timeout) override {
        if (!is_open()) return NetworkResult::NotInitialized;

#ifdef _WIN32
        DWORD tv = static_cast<DWORD>(timeout.count());
        setsockopt(socket_, SOL_SOCKET, SO_RCVTIMEO,
                   reinterpret_cast<const char*>(&tv), sizeof(tv));
#else
        struct timeval tv;
        tv.tv_sec = timeout.count() / 1000;
        tv.tv_usec = (timeout.count() % 1000) * 1000;
        setsockopt(socket_, SOL_SOCKET, SO_RCVTIMEO,
                   reinterpret_cast<const char*>(&tv), sizeof(tv));
#endif
        return NetworkResult::Success;
    }

    NetworkResult set_send_timeout(std::chrono::milliseconds timeout) override {
        if (!is_open()) return NetworkResult::NotInitialized;

#ifdef _WIN32
        DWORD tv = static_cast<DWORD>(timeout.count());
        setsockopt(socket_, SOL_SOCKET, SO_SNDTIMEO,
                   reinterpret_cast<const char*>(&tv), sizeof(tv));
#else
        struct timeval tv;
        tv.tv_sec = timeout.count() / 1000;
        tv.tv_usec = (timeout.count() % 1000) * 1000;
        setsockopt(socket_, SOL_SOCKET, SO_SNDTIMEO,
                   reinterpret_cast<const char*>(&tv), sizeof(tv));
#endif
        return NetworkResult::Success;
    }

    NetworkResult set_multicast_ttl(UInt8 ttl) override {
        if (!is_open()) return NetworkResult::NotInitialized;

        int ttl_val = ttl;
        if (setsockopt(socket_, IPPROTO_IP, IP_MULTICAST_TTL,
                       reinterpret_cast<const char*>(&ttl_val), sizeof(ttl_val)) < 0) {
            return NetworkResult::InvalidOption;
        }
        return NetworkResult::Success;
    }

    const NetworkStats& get_stats() const override {
        return stats_;
    }

    void reset_stats() override {
        stats_.reset();
    }

private:
    socket_t socket_{INVALID_SOCKET_VALUE};
    UDPSocketConfig config_;
    NetworkStats stats_;
};

//==============================================================================
// TCP Connection Implementation
//==============================================================================

class TCPConnectionImpl : public ITCPConnection {
public:
    TCPConnectionImpl() = default;

    explicit TCPConnectionImpl(socket_t sock, const SocketAddress& remote)
        : socket_(sock), remote_address_(remote), connected_(true) {}

    ~TCPConnectionImpl() override {
        close(false);
    }

    NetworkResult connect(const TCPConnectionConfig& config) override {
        if (connected_) return NetworkResult::AlreadyConnected;

        remote_address_ = config.remote_address;

        // Create socket
        int af = (config.remote_address.version == IPVersion::IPv6) ? AF_INET6 : AF_INET;
        socket_ = socket(af, SOCK_STREAM, IPPROTO_TCP);
        if (socket_ == INVALID_SOCKET_VALUE) {
            return NetworkResult::SocketCreationFailed;
        }

        // Set TCP_NODELAY
        if (config.tcp_nodelay) {
            int opt = 1;
            setsockopt(socket_, IPPROTO_TCP, TCP_NODELAY,
                       reinterpret_cast<const char*>(&opt), sizeof(opt));
        }

        // Set buffer sizes
        if (config.receive_buffer_size > 0) {
            int size = static_cast<int>(config.receive_buffer_size);
            setsockopt(socket_, SOL_SOCKET, SO_RCVBUF,
                       reinterpret_cast<const char*>(&size), sizeof(size));
        }

        if (config.send_buffer_size > 0) {
            int size = static_cast<int>(config.send_buffer_size);
            setsockopt(socket_, SOL_SOCKET, SO_SNDBUF,
                       reinterpret_cast<const char*>(&size), sizeof(size));
        }

        // Set keep-alive
        if (config.keep_alive) {
            int opt = 1;
            setsockopt(socket_, SOL_SOCKET, SO_KEEPALIVE,
                       reinterpret_cast<const char*>(&opt), sizeof(opt));
        }

        // Connect
        struct sockaddr_in addr;
        std::memset(&addr, 0, sizeof(addr));
        addr.sin_family = AF_INET;
        addr.sin_port = htons(config.remote_address.port);
        inet_pton(AF_INET, config.remote_address.host.c_str(), &addr.sin_addr);

        if (::connect(socket_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
            close(false);
            return NetworkResult::ConnectFailed;
        }

        connected_ = true;
        return NetworkResult::Success;
    }

    void close(bool graceful) override {
        if (socket_ != INVALID_SOCKET_VALUE) {
            if (graceful) {
                shutdown(socket_, 2);  // SHUT_RDWR
            }
            close_socket(socket_);
            socket_ = INVALID_SOCKET_VALUE;
        }
        connected_ = false;
    }

    bool is_connected() const override {
        return connected_;
    }

    Int64 send(const UInt8* data, SizeT length) override {
        if (!connected_) return -1;

        Int64 sent = ::send(socket_, reinterpret_cast<const char*>(data),
                            static_cast<int>(length), 0);

        if (sent > 0) {
            stats_.bytes_sent += sent;
            stats_.packets_sent++;
        } else {
            stats_.send_errors++;
        }

        return sent;
    }

    NetworkResult send_all(const UInt8* data, SizeT length) override {
        if (!connected_) return NetworkResult::NotConnected;

        SizeT total_sent = 0;
        while (total_sent < length) {
            Int64 sent = send(data + total_sent, length - total_sent);
            if (sent < 0) {
                return NetworkResult::SendFailed;
            }
            if (sent == 0) {
                return NetworkResult::ConnectionClosed;
            }
            total_sent += sent;
        }
        return NetworkResult::Success;
    }

    Int64 receive(UInt8* buffer, SizeT buffer_size) override {
        if (!connected_) return -1;

        Int64 received = recv(socket_, reinterpret_cast<char*>(buffer),
                              static_cast<int>(buffer_size), 0);

        if (received > 0) {
            stats_.bytes_received += received;
            stats_.packets_received++;
        } else if (received == 0) {
            connected_ = false;
        } else {
            stats_.receive_errors++;
        }

        return received;
    }

    NetworkResult receive_exact(UInt8* buffer, SizeT length) override {
        if (!connected_) return NetworkResult::NotConnected;

        SizeT total_received = 0;
        while (total_received < length) {
            Int64 received = receive(buffer + total_received, length - total_received);
            if (received < 0) {
                return NetworkResult::ReceiveFailed;
            }
            if (received == 0) {
                return NetworkResult::ConnectionClosed;
            }
            total_received += received;
        }
        return NetworkResult::Success;
    }

    bool poll_readable(std::chrono::milliseconds timeout) override {
        if (!connected_) return false;

#ifdef _WIN32
        fd_set read_fds;
        FD_ZERO(&read_fds);
        FD_SET(socket_, &read_fds);

        struct timeval tv;
        tv.tv_sec = static_cast<long>(timeout.count() / 1000);
        tv.tv_usec = static_cast<long>((timeout.count() % 1000) * 1000);

        return select(0, &read_fds, nullptr, nullptr, &tv) > 0;
#else
        struct pollfd pfd;
        pfd.fd = socket_;
        pfd.events = POLLIN;

        return poll(&pfd, 1, static_cast<int>(timeout.count())) > 0;
#endif
    }

    SocketAddress get_remote_address() const override {
        return remote_address_;
    }

    SocketAddress get_local_address() const override {
        if (!connected_) return SocketAddress();

        struct sockaddr_in addr;
        socklen_t addr_len = sizeof(addr);

        if (getsockname(socket_, reinterpret_cast<struct sockaddr*>(&addr), &addr_len) < 0) {
            return SocketAddress();
        }

        char host_buf[INET_ADDRSTRLEN];
        inet_ntop(AF_INET, &addr.sin_addr, host_buf, sizeof(host_buf));
        return SocketAddress(host_buf, ntohs(addr.sin_port));
    }

    const NetworkStats& get_stats() const override {
        return stats_;
    }

private:
    socket_t socket_{INVALID_SOCKET_VALUE};
    SocketAddress remote_address_;
    bool connected_{false};
    NetworkStats stats_;
};

//==============================================================================
// TCP Server Implementation
//==============================================================================

class TCPServerImpl : public ITCPServer {
public:
    TCPServerImpl() = default;

    ~TCPServerImpl() override {
        close();
    }

    NetworkResult listen(const TCPServerConfig& config) override {
        if (socket_ != INVALID_SOCKET_VALUE) {
            return NetworkResult::AlreadyInitialized;
        }

        config_ = config;

        // Create socket
        int af = (config.bind_address.version == IPVersion::IPv6) ? AF_INET6 : AF_INET;
        socket_ = socket(af, SOCK_STREAM, IPPROTO_TCP);
        if (socket_ == INVALID_SOCKET_VALUE) {
            return NetworkResult::SocketCreationFailed;
        }

        // Set reuse address
        if (config.reuse_address) {
            int opt = 1;
            setsockopt(socket_, SOL_SOCKET, SO_REUSEADDR,
                       reinterpret_cast<const char*>(&opt), sizeof(opt));
        }

        // Bind
        struct sockaddr_in addr;
        std::memset(&addr, 0, sizeof(addr));
        addr.sin_family = AF_INET;
        addr.sin_port = htons(config.bind_address.port);

        if (config.bind_address.host.empty() || config.bind_address.host == "0.0.0.0") {
            addr.sin_addr.s_addr = INADDR_ANY;
        } else {
            inet_pton(AF_INET, config.bind_address.host.c_str(), &addr.sin_addr);
        }

        if (bind(socket_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
            close();
            return NetworkResult::BindFailed;
        }

        // Listen
        if (::listen(socket_, config.backlog) < 0) {
            close();
            return NetworkResult::ListenFailed;
        }

        listening_ = true;
        return NetworkResult::Success;
    }

    void close() override {
        if (socket_ != INVALID_SOCKET_VALUE) {
            close_socket(socket_);
            socket_ = INVALID_SOCKET_VALUE;
        }
        listening_ = false;
    }

    bool is_listening() const override {
        return listening_;
    }

    std::unique_ptr<ITCPConnection> accept(std::chrono::milliseconds timeout) override {
        if (!listening_) return nullptr;

        if (timeout.count() > 0 && !poll_pending(timeout)) {
            return nullptr;
        }

        struct sockaddr_in client_addr;
        socklen_t addr_len = sizeof(client_addr);

        socket_t client_socket = ::accept(socket_,
                                          reinterpret_cast<struct sockaddr*>(&client_addr),
                                          &addr_len);

        if (client_socket == INVALID_SOCKET_VALUE) {
            return nullptr;
        }

        char host_buf[INET_ADDRSTRLEN];
        inet_ntop(AF_INET, &client_addr.sin_addr, host_buf, sizeof(host_buf));
        SocketAddress remote(host_buf, ntohs(client_addr.sin_port));

        return std::make_unique<TCPConnectionImpl>(client_socket, remote);
    }

    bool poll_pending(std::chrono::milliseconds timeout) override {
        if (!listening_) return false;

#ifdef _WIN32
        fd_set read_fds;
        FD_ZERO(&read_fds);
        FD_SET(socket_, &read_fds);

        struct timeval tv;
        tv.tv_sec = static_cast<long>(timeout.count() / 1000);
        tv.tv_usec = static_cast<long>((timeout.count() % 1000) * 1000);

        return select(0, &read_fds, nullptr, nullptr, &tv) > 0;
#else
        struct pollfd pfd;
        pfd.fd = socket_;
        pfd.events = POLLIN;

        return poll(&pfd, 1, static_cast<int>(timeout.count())) > 0;
#endif
    }

    SocketAddress get_local_address() const override {
        if (!listening_) return SocketAddress();

        struct sockaddr_in addr;
        socklen_t addr_len = sizeof(addr);

        if (getsockname(socket_, reinterpret_cast<struct sockaddr*>(&addr), &addr_len) < 0) {
            return SocketAddress();
        }

        char host_buf[INET_ADDRSTRLEN];
        inet_ntop(AF_INET, &addr.sin_addr, host_buf, sizeof(host_buf));
        return SocketAddress(host_buf, ntohs(addr.sin_port));
    }

private:
    socket_t socket_{INVALID_SOCKET_VALUE};
    TCPServerConfig config_;
    bool listening_{false};
};

//==============================================================================
// Message Framer Implementation
//==============================================================================

void MessageFramer::frame_message(const UInt8* message, SizeT length, UInt8* output) {
    // Write length in network byte order (big endian)
    UInt32 net_length = htonl_safe(static_cast<UInt32>(length));
    std::memcpy(output, &net_length, 4);
    std::memcpy(output + 4, message, length);
}

SizeT MessageFramer::process_received_data(
    const UInt8* data,
    SizeT length,
    std::vector<std::vector<UInt8>>& messages) {

    // Append to receive buffer
    receive_buffer_.insert(receive_buffer_.end(), data, data + length);

    SizeT message_count = 0;

    // Extract complete messages
    while (receive_buffer_.size() >= 4) {
        // Read length prefix
        UInt32 net_length;
        std::memcpy(&net_length, receive_buffer_.data(), 4);
        UInt32 msg_length = ntohl_safe(net_length);

        if (receive_buffer_.size() < 4 + msg_length) {
            break;  // Incomplete message
        }

        // Extract message
        std::vector<UInt8> message(receive_buffer_.begin() + 4,
                                   receive_buffer_.begin() + 4 + msg_length);
        messages.push_back(std::move(message));

        // Remove from buffer
        receive_buffer_.erase(receive_buffer_.begin(),
                              receive_buffer_.begin() + 4 + msg_length);

        message_count++;
    }

    return message_count;
}

//==============================================================================
// Reliable UDP Implementation (Stub)
//==============================================================================

class ReliableUDPImpl : public IReliableUDP {
public:
    ReliableUDPImpl() = default;

    ~ReliableUDPImpl() override {
        close();
    }

    NetworkResult initialize(const ReliableUDPConfig& config) override {
        config_ = config;
        socket_ = create_udp_socket();
        return socket_->initialize(config.socket_config);
    }

    void close() override {
        if (socket_) {
            socket_->close();
            socket_.reset();
        }
    }

    NetworkResult send_reliable(
        const UInt8* data,
        SizeT length,
        const SocketAddress& destination,
        SequenceNumber& sequence) override {

        // Stub implementation - just send unreliably for now
        sequence = next_sequence_++;
        return send_unreliable(data, length, destination);
    }

    NetworkResult send_unreliable(
        const UInt8* data,
        SizeT length,
        const SocketAddress& destination) override {

        if (!socket_ || !socket_->is_open()) {
            return NetworkResult::NotInitialized;
        }

        Int64 sent = socket_->send_to(data, length, destination);
        return (sent > 0) ? NetworkResult::Success : NetworkResult::SendFailed;
    }

    SizeT process_pending() override {
        // Stub - no retransmission logic yet
        return 0;
    }

    NetworkResult receive(
        UInt8* buffer,
        SizeT buffer_size,
        ReceiveResult& result,
        bool& is_reliable) override {

        if (!socket_ || !socket_->is_open()) {
            return NetworkResult::NotInitialized;
        }

        is_reliable = false;  // Stub - all messages unreliable for now
        return socket_->receive_from(buffer, buffer_size, result);
    }

    bool all_acknowledged() const override {
        return pending_acks_.empty();
    }

    SizeT pending_ack_count() const override {
        return pending_acks_.size();
    }

private:
    std::unique_ptr<IUDPSocket> socket_;
    ReliableUDPConfig config_;
    SequenceNumber next_sequence_{0};
    std::vector<AckInfo> pending_acks_;
};

//==============================================================================
// Factory Functions
//==============================================================================

std::unique_ptr<IUDPSocket> create_udp_socket() {
    return std::make_unique<UDPSocketImpl>();
}

std::unique_ptr<ITCPConnection> create_tcp_connection() {
    return std::make_unique<TCPConnectionImpl>();
}

std::unique_ptr<ITCPServer> create_tcp_server() {
    return std::make_unique<TCPServerImpl>();
}

std::unique_ptr<IReliableUDP> create_reliable_udp() {
    return std::make_unique<ReliableUDPImpl>();
}

//==============================================================================
// Utility Functions
//==============================================================================

std::vector<std::pair<std::string, SocketAddress>> get_local_interfaces() {
    std::vector<std::pair<std::string, SocketAddress>> result;

#ifndef _WIN32
    struct ifaddrs* ifaddrs;
    if (getifaddrs(&ifaddrs) == 0) {
        for (auto* ifa = ifaddrs; ifa != nullptr; ifa = ifa->ifa_next) {
            if (ifa->ifa_addr && ifa->ifa_addr->sa_family == AF_INET) {
                auto* addr = reinterpret_cast<struct sockaddr_in*>(ifa->ifa_addr);
                char host_buf[INET_ADDRSTRLEN];
                inet_ntop(AF_INET, &addr->sin_addr, host_buf, sizeof(host_buf));
                result.emplace_back(ifa->ifa_name, SocketAddress(host_buf, 0));
            }
        }
        freeifaddrs(ifaddrs);
    }
#endif

    return result;
}

std::vector<SocketAddress> resolve_hostname(
    std::string_view hostname,
    IPVersion version) {

    std::vector<SocketAddress> result;

    struct addrinfo hints;
    std::memset(&hints, 0, sizeof(hints));

    switch (version) {
        case IPVersion::IPv4: hints.ai_family = AF_INET; break;
        case IPVersion::IPv6: hints.ai_family = AF_INET6; break;
        case IPVersion::Any: hints.ai_family = AF_UNSPEC; break;
    }
    hints.ai_socktype = SOCK_STREAM;

    struct addrinfo* res;
    std::string host(hostname);
    if (getaddrinfo(host.c_str(), nullptr, &hints, &res) == 0) {
        for (auto* p = res; p != nullptr; p = p->ai_next) {
            char host_buf[INET6_ADDRSTRLEN];
            if (p->ai_family == AF_INET) {
                auto* addr = reinterpret_cast<struct sockaddr_in*>(p->ai_addr);
                inet_ntop(AF_INET, &addr->sin_addr, host_buf, sizeof(host_buf));
                result.emplace_back(host_buf, 0, IPVersion::IPv4);
            } else if (p->ai_family == AF_INET6) {
                auto* addr = reinterpret_cast<struct sockaddr_in6*>(p->ai_addr);
                inet_ntop(AF_INET6, &addr->sin6_addr, host_buf, sizeof(host_buf));
                result.emplace_back(host_buf, 0, IPVersion::IPv6);
            }
        }
        freeaddrinfo(res);
    }

    return result;
}

bool is_port_available(UInt16 port, bool udp) {
    int type = udp ? SOCK_DGRAM : SOCK_STREAM;
    socket_t sock = socket(AF_INET, type, 0);
    if (sock == INVALID_SOCKET_VALUE) {
        return false;
    }

    struct sockaddr_in addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    addr.sin_addr.s_addr = INADDR_ANY;

    int result = bind(sock, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr));
    close_socket(sock);

    return result == 0;
}

UInt16 get_available_port() {
    socket_t sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock == INVALID_SOCKET_VALUE) {
        return 0;
    }

    struct sockaddr_in addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = 0;  // Let OS assign port
    addr.sin_addr.s_addr = INADDR_ANY;

    if (bind(sock, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
        close_socket(sock);
        return 0;
    }

    socklen_t addr_len = sizeof(addr);
    if (getsockname(sock, reinterpret_cast<struct sockaddr*>(&addr), &addr_len) < 0) {
        close_socket(sock);
        return 0;
    }

    UInt16 port = ntohs(addr.sin_port);
    close_socket(sock);

    return port;
}

} // namespace jaguar::federation::network
