/****************************************************************************
 *
 * Copyright (c) 2024, libmav development team
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name libmav nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <future>
#include "doctest.h"
#include "mav/Message.h"
#include "mav/Network.h"
#include "mav/TCPClient.h"
#include "mav/TCPServer.h"

using namespace mav;

#define PORT 13975

TEST_CASE("TCP server client") {
    MessageSet message_set;
    message_set.addFromXMLString(R"(
        <mavlink>
            <messages>
                <message id="0" name="HEARTBEAT">
                    <field type="uint8_t" name="type">Type of the MAV (quadrotor, helicopter, etc., up to 15 types, defined in MAV_TYPE ENUM)</field>
                    <field type="uint8_t" name="autopilot">Autopilot type / class. defined in MAV_AUTOPILOT ENUM</field>
                    <field type="uint8_t" name="base_mode">System mode bitfield, see MAV_MODE_FLAGS ENUM in mavlink/include/mavlink_types.h</field>
                    <field type="uint32_t" name="custom_mode">A bitfield for use for autopilot-specific flags.</field>
                    <field type="uint8_t" name="system_status">System status flag, see MAV_STATE ENUM</field>
                    <field type="uint8_t" name="mavlink_version">MAVLink version, not writable by user, gets added by protocol because of magic data type: uint8_t_mavlink_version</field>
                </message>
            </messages>
        </mavlink>
    )");
    message_set.addFromXMLString(R""""(
    <mavlink>
        <messages>
            <message id="9916" name="TEST_MESSAGE">
                <field type="char[25]" name="message">description</field>
            </message>
        </messages>
    </mavlink>
    )"""");

    REQUIRE(message_set.contains("TEST_MESSAGE"));
    REQUIRE_EQ(message_set.size(), 2);

    SUBCASE("Can connect TCP server client") {
        // setup server
        mav::TCPServer server_physical(PORT);
        mav::NetworkRuntime server_runtime(message_set, server_physical);

        std::promise<void> connection_called_promise;
        auto connection_called_future = connection_called_promise.get_future();
        server_runtime.onConnection([&connection_called_promise](const std::shared_ptr<mav::Connection>&) {
            connection_called_promise.set_value();
        });

        // setup client
        auto heartbeat_opt = message_set.create("HEARTBEAT");
        REQUIRE(heartbeat_opt.has_value());
        auto heartbeat = heartbeat_opt.value();
        auto result = heartbeat.set({
                                        {"type", static_cast<uint8_t>(1)},
                                        {"autopilot", static_cast<uint8_t>(2)},
                                        {"base_mode", static_cast<uint8_t>(3)},
                                        {"custom_mode", static_cast<uint32_t>(4)},
                                        {"system_status", static_cast<uint8_t>(5)},
                                        {"mavlink_version", static_cast<uint8_t>(6)}});
        REQUIRE_EQ(result, mav::MessageResult::Success);

        mav::TCPClient client_physical("localhost", PORT);
        mav::NetworkRuntime client_runtime(message_set, heartbeat, client_physical);

        CHECK((connection_called_future.wait_for(std::chrono::seconds(2)) != std::future_status::timeout));

        auto server = server_runtime.awaitConnection(100);
        server->send(heartbeat);

        auto client = client_runtime.awaitConnection(100);

        SUBCASE("Can send message from server to client over TCP") {
            auto client_expectation = client->expect("TEST_MESSAGE");
            auto test_msg_opt = message_set.create("TEST_MESSAGE");
            REQUIRE(test_msg_opt.has_value());
            auto test_msg = test_msg_opt.value();
            auto result = test_msg.setString("message", "hello client");
            REQUIRE_EQ(result, mav::MessageResult::Success);
            server->send(test_msg);
            auto client_message = client->receive(client_expectation, 100);
            std::string message;
            auto get_result = client_message.getString("message", message);
            CHECK_EQ(get_result, mav::MessageResult::Success);
            CHECK_EQ(message, "hello client");
        }

        SUBCASE("Can send message from client to server over TCP") {
            auto server_expectation = server->expect("TEST_MESSAGE");
            auto test_msg_opt = message_set.create("TEST_MESSAGE");
            REQUIRE(test_msg_opt.has_value());
            auto test_msg = test_msg_opt.value();
            auto result = test_msg.setString("message", "hello server");
            REQUIRE_EQ(result, mav::MessageResult::Success);
            client->send(test_msg);
            auto server_message = server->receive(server_expectation, 100);
            std::string message;
            auto get_result = server_message.getString("message", message);
            CHECK_EQ(get_result, mav::MessageResult::Success);
            CHECK_EQ(message, "hello server");
        }


        SUBCASE("Can connect two TCP clients that both can send messages to server") {
            // setup client 2
            auto heartbeat2_opt = message_set.create("HEARTBEAT");
            REQUIRE(heartbeat2_opt.has_value());
            auto heartbeat2 = heartbeat2_opt.value();
            auto result2 = heartbeat2.set({
                                            {"type", static_cast<uint8_t>(1)},
                                            {"autopilot", static_cast<uint8_t>(2)},
                                            {"base_mode", static_cast<uint8_t>(3)},
                                            {"custom_mode", static_cast<uint32_t>(4)},
                                            {"system_status", static_cast<uint8_t>(5)},
                                            {"mavlink_version", static_cast<uint8_t>(6)}});
            REQUIRE_EQ(result2, mav::MessageResult::Success);

            std::promise<std::shared_ptr<mav::Connection>> connection2_promise;
            server_runtime.onConnection([&connection2_promise](const std::shared_ptr<mav::Connection> &connection) {
                connection2_promise.set_value(connection);
            });

            mav::TCPClient client2_physical("127.0.0.1", PORT);
            mav::NetworkRuntime client2_runtime(message_set, heartbeat2, client2_physical);

            auto connection2_future = connection2_promise.get_future();
            connection2_future.wait_for(std::chrono::milliseconds (100));
            auto server2 = connection2_future.get();
            server2->send(heartbeat2);
            auto client2 = client2_runtime.awaitConnection(100);

            auto server_expectation1 = server->expect("TEST_MESSAGE");
            auto server_expectation2 = server2->expect("TEST_MESSAGE");

            auto test_msg1_opt = message_set.create("TEST_MESSAGE");
            REQUIRE(test_msg1_opt.has_value());
            auto test_msg1 = test_msg1_opt.value();
            auto result1 = test_msg1.setString("message", "hello from client 1");
            REQUIRE_EQ(result1, mav::MessageResult::Success);
            client->send(test_msg1);
            
            auto test_msg2_opt = message_set.create("TEST_MESSAGE");
            REQUIRE(test_msg2_opt.has_value());
            auto test_msg2 = test_msg2_opt.value();
            auto set_result2 = test_msg2.setString("message", "hello from client 2");
            REQUIRE_EQ(set_result2, mav::MessageResult::Success);
            client2->send(test_msg2);

            auto server_message1 = server->receive(server_expectation1, 100);
            auto server_message2 = server2->receive(server_expectation2, 100);

            CHECK_NE(server_message1.source(), server_message2.source());
            std::string message1, message2;
            auto get_result1 = server_message1.getString("message", message1);
            auto get_result2 = server_message2.getString("message", message2);
            CHECK_EQ(get_result1, mav::MessageResult::Success);
            CHECK_EQ(get_result2, mav::MessageResult::Success);
            CHECK_EQ(message1, "hello from client 1");
            CHECK_EQ(message2, "hello from client 2");
        }

    }
}

