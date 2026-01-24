/**
 * @file network_transport_tests.cpp
 * @brief Unit tests for network transport layer
 */

#include <gtest/gtest.h>
#include "jaguar/federation/network_transport.h"
#include <thread>
#include <chrono>

using namespace jaguar;
using namespace jaguar::federation::network;

// ============================================================================
// NetworkResult Tests
// ============================================================================

TEST(NetworkResultTest, SuccessValue) {
    EXPECT_EQ(static_cast<UInt32>(NetworkResult::Success), 0);
}

TEST(NetworkResultTest, ToString) {
    EXPECT_STREQ(network_result_to_string(NetworkResult::Success), "Success");
    EXPECT_STREQ(network_result_to_string(NetworkResult::SocketCreationFailed), "SocketCreationFailed");
    EXPECT_STREQ(network_result_to_string(NetworkResult::BindFailed), "BindFailed");
    EXPECT_STREQ(network_result_to_string(NetworkResult::ConnectFailed), "ConnectFailed");
    EXPECT_STREQ(network_result_to_string(NetworkResult::SendFailed), "SendFailed");
    EXPECT_STREQ(network_result_to_string(NetworkResult::WouldBlock), "WouldBlock");
    EXPECT_STREQ(network_result_to_string(NetworkResult::Timeout), "Timeout");
    EXPECT_STREQ(network_result_to_string(NetworkResult::InternalError), "InternalError");
}

// ============================================================================
// SocketAddress Tests
// ============================================================================

TEST(SocketAddressTest, DefaultConstruction) {
    SocketAddress addr;
    EXPECT_TRUE(addr.host.empty());
    EXPECT_EQ(addr.port, 0);
    EXPECT_EQ(addr.version, IPVersion::IPv4);
}

TEST(SocketAddressTest, ParameterizedConstruction) {
    SocketAddress addr("192.168.1.1", 8080, IPVersion::IPv4);
    EXPECT_EQ(addr.host, "192.168.1.1");
    EXPECT_EQ(addr.port, 8080);
    EXPECT_EQ(addr.version, IPVersion::IPv4);
}

TEST(SocketAddressTest, ParseIPv4) {
    auto addr = SocketAddress::parse("192.168.1.100:3000");
    ASSERT_TRUE(addr.has_value());
    EXPECT_EQ(addr->host, "192.168.1.100");
    EXPECT_EQ(addr->port, 3000);
    EXPECT_EQ(addr->version, IPVersion::IPv4);
}

TEST(SocketAddressTest, ParseIPv6) {
    auto addr = SocketAddress::parse("[::1]:8989");
    ASSERT_TRUE(addr.has_value());
    EXPECT_EQ(addr->host, "::1");
    EXPECT_EQ(addr->port, 8989);
    EXPECT_EQ(addr->version, IPVersion::IPv6);
}

TEST(SocketAddressTest, ParseHostOnly) {
    auto addr = SocketAddress::parse("localhost");
    ASSERT_TRUE(addr.has_value());
    EXPECT_EQ(addr->host, "localhost");
    EXPECT_EQ(addr->port, 0);
}

TEST(SocketAddressTest, ToString) {
    SocketAddress ipv4("192.168.1.1", 3000, IPVersion::IPv4);
    EXPECT_EQ(ipv4.to_string(), "192.168.1.1:3000");

    SocketAddress ipv6("::1", 8989, IPVersion::IPv6);
    EXPECT_EQ(ipv6.to_string(), "[::1]:8989");
}

TEST(SocketAddressTest, IsMulticast) {
    // Multicast range: 224.0.0.0 - 239.255.255.255
    SocketAddress multicast("239.1.2.3", 3000);
    EXPECT_TRUE(multicast.is_multicast());

    SocketAddress unicast("192.168.1.1", 3000);
    EXPECT_FALSE(unicast.is_multicast());

    SocketAddress lower_bound("224.0.0.0", 3000);
    EXPECT_TRUE(lower_bound.is_multicast());

    SocketAddress upper_bound("239.255.255.255", 3000);
    EXPECT_TRUE(upper_bound.is_multicast());

    SocketAddress not_multicast("223.255.255.255", 3000);
    EXPECT_FALSE(not_multicast.is_multicast());
}

TEST(SocketAddressTest, IsLoopback) {
    SocketAddress loopback("127.0.0.1", 3000);
    EXPECT_TRUE(loopback.is_loopback());

    SocketAddress loopback2("127.0.1.1", 3000);
    EXPECT_TRUE(loopback2.is_loopback());

    SocketAddress non_loopback("192.168.1.1", 3000);
    EXPECT_FALSE(non_loopback.is_loopback());

    SocketAddress ipv6_loopback("::1", 3000, IPVersion::IPv6);
    EXPECT_TRUE(ipv6_loopback.is_loopback());
}

TEST(SocketAddressTest, IsBroadcast) {
    SocketAddress broadcast("255.255.255.255", 3000);
    EXPECT_TRUE(broadcast.is_broadcast());

    SocketAddress non_broadcast("192.168.1.255", 3000);
    EXPECT_FALSE(non_broadcast.is_broadcast());
}

TEST(SocketAddressTest, Equality) {
    SocketAddress addr1("192.168.1.1", 3000);
    SocketAddress addr2("192.168.1.1", 3000);
    SocketAddress addr3("192.168.1.2", 3000);
    SocketAddress addr4("192.168.1.1", 3001);

    EXPECT_EQ(addr1, addr2);
    EXPECT_NE(addr1, addr3);
    EXPECT_NE(addr1, addr4);
}

// ============================================================================
// NetworkStats Tests
// ============================================================================

TEST(NetworkStatsTest, DefaultConstruction) {
    NetworkStats stats;
    EXPECT_EQ(stats.bytes_sent, 0);
    EXPECT_EQ(stats.bytes_received, 0);
    EXPECT_EQ(stats.packets_sent, 0);
    EXPECT_EQ(stats.packets_received, 0);
    EXPECT_EQ(stats.send_errors, 0);
    EXPECT_EQ(stats.receive_errors, 0);
}

TEST(NetworkStatsTest, Reset) {
    NetworkStats stats;
    stats.bytes_sent = 1000;
    stats.bytes_received = 2000;
    stats.packets_sent = 10;
    stats.packets_received = 20;

    stats.reset();

    EXPECT_EQ(stats.bytes_sent, 0);
    EXPECT_EQ(stats.bytes_received, 0);
    EXPECT_EQ(stats.packets_sent, 0);
    EXPECT_EQ(stats.packets_received, 0);
}

TEST(NetworkStatsTest, BytesPerSecond) {
    NetworkStats stats;
    stats.bytes_sent = 1000;
    stats.bytes_received = 2000;

    auto period = std::chrono::duration<Real>(2.0);
    EXPECT_NEAR(stats.bytes_per_second_sent(period), 500.0, 0.001);
    EXPECT_NEAR(stats.bytes_per_second_received(period), 1000.0, 0.001);

    // Zero period should return 0
    auto zero_period = std::chrono::duration<Real>(0.0);
    EXPECT_DOUBLE_EQ(stats.bytes_per_second_sent(zero_period), 0.0);
}

// ============================================================================
// IOBuffer Tests
// ============================================================================

TEST(IOBufferTest, DefaultConstruction) {
    IOBuffer buffer;
    EXPECT_EQ(buffer.data, nullptr);
    EXPECT_EQ(buffer.length, 0);
}

TEST(IOBufferTest, ParameterizedConstruction) {
    UInt8 data[10] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
    IOBuffer buffer(data, 10);
    EXPECT_EQ(buffer.data, data);
    EXPECT_EQ(buffer.length, 10);
}

TEST(IOBufferTest, FromVector) {
    std::vector<UInt8> vec = {1, 2, 3, 4, 5};
    IOBuffer buffer = IOBuffer::from_vector(vec);
    EXPECT_EQ(buffer.data, vec.data());
    EXPECT_EQ(buffer.length, 5);
}

// ============================================================================
// UDPSocketConfig Tests
// ============================================================================

TEST(UDPSocketConfigTest, DISDefault) {
    auto config = UDPSocketConfig::dis_default();
    EXPECT_EQ(config.bind_address.port, DEFAULT_DIS_PORT);
    EXPECT_EQ(config.multicast_ttl, 32);
    EXPECT_TRUE(config.multicast_loopback);
}

TEST(UDPSocketConfigTest, CustomPort) {
    auto config = UDPSocketConfig::dis_default(5000);
    EXPECT_EQ(config.bind_address.port, 5000);
}

// ============================================================================
// UDP Socket Factory Test
// ============================================================================

TEST(UDPSocketTest, CreateSocket) {
    auto socket = create_udp_socket();
    EXPECT_NE(socket, nullptr);
    EXPECT_FALSE(socket->is_open());
}

TEST(UDPSocketTest, InitializeAndClose) {
    auto socket = create_udp_socket();

    UDPSocketConfig config;
    config.bind_address = SocketAddress("127.0.0.1", 0);  // Ephemeral port
    config.reuse_address = true;

    auto result = socket->initialize(config);
    EXPECT_EQ(result, NetworkResult::Success);
    EXPECT_TRUE(socket->is_open());

    auto local_addr = socket->get_local_address();
    EXPECT_GT(local_addr.port, 0);  // Should have been assigned a port

    socket->close();
    EXPECT_FALSE(socket->is_open());
}

TEST(UDPSocketTest, DoubleInitialize) {
    auto socket = create_udp_socket();

    UDPSocketConfig config;
    config.bind_address = SocketAddress("127.0.0.1", 0);

    auto result = socket->initialize(config);
    EXPECT_EQ(result, NetworkResult::Success);

    // Second initialize should fail
    result = socket->initialize(config);
    EXPECT_EQ(result, NetworkResult::AlreadyInitialized);

    socket->close();
}

TEST(UDPSocketTest, SendReceiveLoopback) {
    auto sender = create_udp_socket();
    auto receiver = create_udp_socket();

    UDPSocketConfig recv_config;
    recv_config.bind_address = SocketAddress("127.0.0.1", 0);
    EXPECT_EQ(receiver->initialize(recv_config), NetworkResult::Success);

    auto recv_addr = receiver->get_local_address();
    recv_addr.host = "127.0.0.1";

    UDPSocketConfig send_config;
    send_config.bind_address = SocketAddress("127.0.0.1", 0);
    EXPECT_EQ(sender->initialize(send_config), NetworkResult::Success);

    // Send message
    std::vector<UInt8> message = {0x01, 0x02, 0x03, 0x04, 0x05};
    Int64 sent = sender->send_to(message.data(), message.size(), recv_addr);
    EXPECT_EQ(sent, 5);

    // Small delay for network
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    // Receive message
    UInt8 buffer[256];
    ReceiveResult result;
    auto status = receiver->receive_from(buffer, sizeof(buffer), result);
    EXPECT_EQ(status, NetworkResult::Success);
    EXPECT_EQ(result.bytes_received, 5);
    EXPECT_EQ(std::memcmp(buffer, message.data(), 5), 0);

    sender->close();
    receiver->close();
}

TEST(UDPSocketTest, PollReadable) {
    auto socket = create_udp_socket();

    UDPSocketConfig config;
    config.bind_address = SocketAddress("127.0.0.1", 0);
    EXPECT_EQ(socket->initialize(config), NetworkResult::Success);

    // Should not be readable immediately
    EXPECT_FALSE(socket->poll_readable(std::chrono::milliseconds(1)));

    socket->close();
}

TEST(UDPSocketTest, SetTimeouts) {
    auto socket = create_udp_socket();

    UDPSocketConfig config;
    config.bind_address = SocketAddress("127.0.0.1", 0);
    EXPECT_EQ(socket->initialize(config), NetworkResult::Success);

    EXPECT_EQ(socket->set_receive_timeout(std::chrono::milliseconds(100)),
              NetworkResult::Success);
    EXPECT_EQ(socket->set_send_timeout(std::chrono::milliseconds(100)),
              NetworkResult::Success);

    socket->close();
}

TEST(UDPSocketTest, Statistics) {
    auto socket = create_udp_socket();

    UDPSocketConfig config;
    config.bind_address = SocketAddress("127.0.0.1", 0);
    socket->initialize(config);

    const auto& stats = socket->get_stats();
    EXPECT_EQ(stats.bytes_sent, 0);
    EXPECT_EQ(stats.packets_sent, 0);

    // Send some data
    std::vector<UInt8> data = {1, 2, 3};
    socket->send_to(data.data(), data.size(), SocketAddress("127.0.0.1", 12345));

    EXPECT_GT(socket->get_stats().bytes_sent, 0);
    EXPECT_EQ(socket->get_stats().packets_sent, 1);

    socket->reset_stats();
    EXPECT_EQ(socket->get_stats().bytes_sent, 0);

    socket->close();
}

// ============================================================================
// TCP Connection Factory Test
// ============================================================================

TEST(TCPConnectionTest, CreateConnection) {
    auto conn = create_tcp_connection();
    EXPECT_NE(conn, nullptr);
    EXPECT_FALSE(conn->is_connected());
}

// ============================================================================
// TCP Server Factory Test
// ============================================================================

TEST(TCPServerTest, CreateServer) {
    auto server = create_tcp_server();
    EXPECT_NE(server, nullptr);
    EXPECT_FALSE(server->is_listening());
}

TEST(TCPServerTest, ListenAndClose) {
    auto server = create_tcp_server();

    TCPServerConfig config;
    config.bind_address = SocketAddress("127.0.0.1", 0);
    config.reuse_address = true;

    auto result = server->listen(config);
    EXPECT_EQ(result, NetworkResult::Success);
    EXPECT_TRUE(server->is_listening());

    auto local_addr = server->get_local_address();
    EXPECT_GT(local_addr.port, 0);

    server->close();
    EXPECT_FALSE(server->is_listening());
}

TEST(TCPServerTest, AcceptTimeout) {
    auto server = create_tcp_server();

    TCPServerConfig config;
    config.bind_address = SocketAddress("127.0.0.1", 0);

    server->listen(config);

    // Should timeout with no connection
    auto conn = server->accept(std::chrono::milliseconds(10));
    EXPECT_EQ(conn, nullptr);

    server->close();
}

// ============================================================================
// TCP Client-Server Integration Test
// ============================================================================

TEST(TCPIntegrationTest, ClientServerConnection) {
    auto server = create_tcp_server();

    TCPServerConfig server_config;
    server_config.bind_address = SocketAddress("127.0.0.1", 0);
    EXPECT_EQ(server->listen(server_config), NetworkResult::Success);

    auto server_addr = server->get_local_address();
    server_addr.host = "127.0.0.1";

    // Start client in a thread
    std::thread client_thread([&server_addr]() {
        auto client = create_tcp_connection();

        TCPConnectionConfig config;
        config.remote_address = server_addr;
        auto result = client->connect(config);
        EXPECT_EQ(result, NetworkResult::Success);

        // Send data
        std::vector<UInt8> message = {0x48, 0x45, 0x4C, 0x4C, 0x4F};  // "HELLO"
        client->send_all(message.data(), message.size());

        client->close();
    });

    // Accept connection
    auto conn = server->accept(std::chrono::milliseconds(1000));
    ASSERT_NE(conn, nullptr);
    EXPECT_TRUE(conn->is_connected());

    // Receive data
    UInt8 buffer[256];
    Int64 received = conn->receive(buffer, sizeof(buffer));
    EXPECT_EQ(received, 5);
    EXPECT_EQ(std::memcmp(buffer, "HELLO", 5), 0);

    client_thread.join();
    conn->close();
    server->close();
}

// ============================================================================
// Message Framer Tests
// ============================================================================

TEST(MessageFramerTest, FrameMessage) {
    MessageFramer framer;

    std::vector<UInt8> message = {0x01, 0x02, 0x03, 0x04};
    std::vector<UInt8> output(message.size() + 4);

    framer.frame_message(message.data(), message.size(), output.data());

    // Check length prefix (big endian)
    EXPECT_EQ(output[0], 0x00);
    EXPECT_EQ(output[1], 0x00);
    EXPECT_EQ(output[2], 0x00);
    EXPECT_EQ(output[3], 0x04);

    // Check message data
    EXPECT_EQ(output[4], 0x01);
    EXPECT_EQ(output[5], 0x02);
    EXPECT_EQ(output[6], 0x03);
    EXPECT_EQ(output[7], 0x04);
}

TEST(MessageFramerTest, ProcessSingleMessage) {
    MessageFramer framer;

    // Create framed message
    std::vector<UInt8> message = {0xAA, 0xBB, 0xCC};
    std::vector<UInt8> framed(message.size() + 4);
    framer.frame_message(message.data(), message.size(), framed.data());

    // Process
    std::vector<std::vector<UInt8>> messages;
    SizeT count = framer.process_received_data(framed.data(), framed.size(), messages);

    EXPECT_EQ(count, 1);
    EXPECT_EQ(messages.size(), 1);
    EXPECT_EQ(messages[0], message);
    EXPECT_FALSE(framer.has_partial_message());
}

TEST(MessageFramerTest, ProcessMultipleMessages) {
    MessageFramer framer;

    // Create two framed messages
    std::vector<UInt8> msg1 = {0x01, 0x02};
    std::vector<UInt8> msg2 = {0x03, 0x04, 0x05};

    std::vector<UInt8> framed1(msg1.size() + 4);
    std::vector<UInt8> framed2(msg2.size() + 4);

    framer.frame_message(msg1.data(), msg1.size(), framed1.data());
    framer.frame_message(msg2.data(), msg2.size(), framed2.data());

    // Concatenate
    std::vector<UInt8> combined;
    combined.insert(combined.end(), framed1.begin(), framed1.end());
    combined.insert(combined.end(), framed2.begin(), framed2.end());

    // Process
    std::vector<std::vector<UInt8>> messages;
    SizeT count = framer.process_received_data(combined.data(), combined.size(), messages);

    EXPECT_EQ(count, 2);
    EXPECT_EQ(messages[0], msg1);
    EXPECT_EQ(messages[1], msg2);
}

TEST(MessageFramerTest, PartialMessage) {
    MessageFramer framer;

    // Create framed message
    std::vector<UInt8> message = {0x01, 0x02, 0x03, 0x04, 0x05};
    std::vector<UInt8> framed(message.size() + 4);
    framer.frame_message(message.data(), message.size(), framed.data());

    // Send only first half
    std::vector<std::vector<UInt8>> messages;
    SizeT count = framer.process_received_data(framed.data(), 5, messages);

    EXPECT_EQ(count, 0);
    EXPECT_TRUE(framer.has_partial_message());

    // Send rest
    count = framer.process_received_data(framed.data() + 5, 4, messages);

    EXPECT_EQ(count, 1);
    EXPECT_EQ(messages[0], message);
    EXPECT_FALSE(framer.has_partial_message());
}

TEST(MessageFramerTest, Clear) {
    MessageFramer framer;

    // Add partial message
    std::vector<UInt8> partial = {0x00, 0x00, 0x00, 0x05, 0x01};
    std::vector<std::vector<UInt8>> messages;
    framer.process_received_data(partial.data(), partial.size(), messages);

    EXPECT_TRUE(framer.has_partial_message());

    framer.clear();
    EXPECT_FALSE(framer.has_partial_message());
}

// ============================================================================
// Reliable UDP Factory Test
// ============================================================================

TEST(ReliableUDPTest, CreateReliableUDP) {
    auto rudp = create_reliable_udp();
    EXPECT_NE(rudp, nullptr);
}

TEST(ReliableUDPTest, InitializeAndClose) {
    auto rudp = create_reliable_udp();

    ReliableUDPConfig config;
    config.socket_config.bind_address = SocketAddress("127.0.0.1", 0);

    auto result = rudp->initialize(config);
    EXPECT_EQ(result, NetworkResult::Success);
    EXPECT_TRUE(rudp->all_acknowledged());
    EXPECT_EQ(rudp->pending_ack_count(), 0);

    rudp->close();
}

// ============================================================================
// Utility Function Tests
// ============================================================================

TEST(UtilityTest, IsPortAvailable) {
    // Port 0 is never available for explicit binding
    // Test with ephemeral port range
    UInt16 test_port = get_available_port();
    if (test_port != 0) {
        EXPECT_TRUE(is_port_available(test_port, true));
    }
}

TEST(UtilityTest, GetAvailablePort) {
    UInt16 port = get_available_port();
    // Should return a valid ephemeral port
    EXPECT_GT(port, 0);
    EXPECT_GE(port, 1024);  // Should be above well-known ports
}

TEST(UtilityTest, ResolveLocalhost) {
    auto addrs = resolve_hostname("localhost", IPVersion::IPv4);
    // localhost should resolve to 127.0.0.1
    bool found_loopback = false;
    for (const auto& addr : addrs) {
        if (addr.host == "127.0.0.1") {
            found_loopback = true;
            break;
        }
    }
    EXPECT_TRUE(found_loopback);
}

TEST(UtilityTest, ByteOrderConversion) {
    // Test 16-bit
    UInt16 val16 = 0x1234;
    UInt16 net16 = htons_safe(val16);
    EXPECT_EQ(ntohs_safe(net16), val16);

    // Test 32-bit
    UInt32 val32 = 0x12345678;
    UInt32 net32 = htonl_safe(val32);
    EXPECT_EQ(ntohl_safe(net32), val32);
}

// ============================================================================
// Constants Tests
// ============================================================================

TEST(ConstantsTest, DefaultPorts) {
    EXPECT_EQ(DEFAULT_DIS_PORT, 3000);
    EXPECT_EQ(DEFAULT_HLA_PORT, 8989);
}

TEST(ConstantsTest, MaxUDPPayload) {
    EXPECT_EQ(MAX_UDP_PAYLOAD, 65507);
}

TEST(ConstantsTest, MulticastTTL) {
    EXPECT_EQ(MAX_MULTICAST_TTL, 255);
    EXPECT_EQ(DEFAULT_MULTICAST_TTL, 32);
}
