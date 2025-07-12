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
#include "doctest.h"
#include "mav/BufferParser.h"
#include "mav/MessageSet.h"
#include "mav/Message.h"
#include <vector>
#include <cstring>

using namespace mav;

TEST_CASE("BufferParser basic functionality") {
    MessageSet message_set;
    auto result = message_set.addFromXMLString(R""""(
<mavlink>
    <messages>
        <message id="1" name="TEST_MESSAGE">
            <field type="uint8_t" name="target_system">System ID</field>
            <field type="uint8_t" name="target_component">Component ID</field>
            <field type="uint32_t" name="test_field">Test field</field>
        </message>
    </messages>
</mavlink>
)"""");
    REQUIRE(result == MessageSetResult::Success);

    BufferParser parser(message_set);
    
    SUBCASE("Parse empty buffer") {
        uint8_t empty_buffer[1] = {0};
        size_t bytes_consumed = 0;
        auto message = parser.parseMessage(empty_buffer, 0, bytes_consumed);
        CHECK_FALSE(message.has_value());
        CHECK(bytes_consumed == 0);
    }

    SUBCASE("Parse buffer without magic byte") {
        uint8_t no_magic_buffer[] = {0x01, 0x02, 0x03, 0x04, 0x05};
        size_t bytes_consumed = 0;
        auto message = parser.parseMessage(no_magic_buffer, sizeof(no_magic_buffer), bytes_consumed);
        CHECK_FALSE(message.has_value());
        CHECK(bytes_consumed == sizeof(no_magic_buffer));
    }

    SUBCASE("Parse buffer with magic byte but insufficient data") {
        uint8_t insufficient_buffer[] = {0xFD, 0x01, 0x02}; // Magic + 2 bytes (need 10 for header)
        size_t bytes_consumed = 0;
        auto message = parser.parseMessage(insufficient_buffer, sizeof(insufficient_buffer), bytes_consumed);
        CHECK_FALSE(message.has_value());
        CHECK(bytes_consumed == 0); // Should stop at magic byte
    }
}

TEST_CASE("BufferParser message creation and parsing") {
    MessageSet message_set;
    auto result = message_set.addFromXMLString(R""""(
<mavlink>
    <messages>
        <message id="100" name="SIMPLE_MESSAGE">
            <field type="uint8_t" name="target_system">System ID</field>
            <field type="uint8_t" name="target_component">Component ID</field>
            <field type="uint16_t" name="value">Test value</field>
        </message>
    </messages>
</mavlink>
)"""");
    REQUIRE(result == MessageSetResult::Success);

    SUBCASE("Create, serialize, and parse back a message") {
        // Create a message
        auto message_opt = message_set.create("SIMPLE_MESSAGE");
        REQUIRE(message_opt.has_value());
        auto message = message_opt.value();

        // Set field values
        message.set("target_system", static_cast<uint8_t>(1));
        message.set("target_component", static_cast<uint8_t>(2));
        message.set("value", static_cast<uint16_t>(42));

        // Finalize the message
        mav::Identifier sender{123, 45};
        auto size_opt = message.finalize(0, sender);
        REQUIRE(size_opt.has_value());
        auto size = size_opt.value();

        // Get the serialized data
        const uint8_t* wire_data = message.data();
        REQUIRE(wire_data != nullptr);

        // Parse it back using BufferParser
        BufferParser parser(message_set);
        size_t bytes_consumed = 0;
        auto parsed_message = parser.parseMessage(wire_data, size, bytes_consumed);
        
        REQUIRE(parsed_message.has_value());
        CHECK(bytes_consumed == size);
        CHECK(parsed_message->name() == "SIMPLE_MESSAGE");
        CHECK(parsed_message->id() == 100);

        // Verify field values
        uint8_t target_system;
        uint8_t target_component;
        uint16_t value;
        
        CHECK(parsed_message->get("target_system", target_system) == MessageResult::Success);
        CHECK(parsed_message->get("target_component", target_component) == MessageResult::Success);
        CHECK(parsed_message->get("value", value) == MessageResult::Success);
        
        CHECK(target_system == 1);
        CHECK(target_component == 2);
        CHECK(value == 42);
    }
}

TEST_CASE("BufferParser CRC validation") {
    MessageSet message_set;
    auto result = message_set.addFromXMLString(R""""(
<mavlink>
    <messages>
        <message id="200" name="CRC_TEST_MESSAGE">
            <field type="uint8_t" name="test_field">Test field</field>
        </message>
    </messages>
</mavlink>
)"""");
    REQUIRE(result == MessageSetResult::Success);

    SUBCASE("Invalid CRC should be rejected") {
        // Create a valid message first
        auto message_opt = message_set.create("CRC_TEST_MESSAGE");
        REQUIRE(message_opt.has_value());
        auto message = message_opt.value();
        
        message.set("test_field", static_cast<uint8_t>(123));
        
        mav::Identifier sender{1, 1};
        auto size_opt = message.finalize(0, sender);
        REQUIRE(size_opt.has_value());
        auto size = size_opt.value();
        
        const uint8_t* wire_data = message.data();
        REQUIRE(wire_data != nullptr);
        
        // Corrupt the CRC by modifying the last 2 bytes
        std::vector<uint8_t> corrupted_data(wire_data, wire_data + size);
        corrupted_data[size - 2] ^= 0xFF; // Flip bits in CRC
        corrupted_data[size - 1] ^= 0xFF;
        
        // Try to parse the corrupted message
        BufferParser parser(message_set);
        size_t bytes_consumed = 0;
        auto parsed_message = parser.parseMessage(corrupted_data.data(), size, bytes_consumed);
        
        // Should fail due to CRC error
        CHECK_FALSE(parsed_message.has_value());
        CHECK(bytes_consumed == size); // Should consume the entire corrupted message
    }
}

TEST_CASE("BufferParser unknown message handling") {
    MessageSet message_set;
    // Empty message set - no messages defined
    
    SUBCASE("Unknown message ID should be skipped") {
        // Create a buffer with a valid MAVLink v2 header but unknown message ID
        uint8_t unknown_message[] = {
            0xFD,       // Magic byte
            0x04,       // Payload length
            0x00,       // Incompatible flags
            0x00,       // Compatible flags  
            0x01,       // Sequence
            0x01,       // System ID
            0x01,       // Component ID
            0xFF, 0xFF, 0xFF, // Message ID (unknown - 0xFFFFFF)
            0x00, 0x00, 0x00, 0x00, // Payload (4 bytes)
            0x00, 0x00  // CRC (will be wrong anyway)
        };
        
        BufferParser parser(message_set);
        size_t bytes_consumed = 0;
        auto parsed_message = parser.parseMessage(unknown_message, sizeof(unknown_message), bytes_consumed);
        
        CHECK_FALSE(parsed_message.has_value());
        CHECK(bytes_consumed == sizeof(unknown_message)); // Should consume the entire unknown message
    }
}

TEST_CASE("BufferParser multiple messages in buffer") {
    MessageSet message_set;
    auto result = message_set.addFromXMLString(R""""(
<mavlink>
    <messages>
        <message id="50" name="MULTI_TEST_MSG">
            <field type="uint8_t" name="counter">Counter value</field>
        </message>
    </messages>
</mavlink>
)"""");
    REQUIRE(result == MessageSetResult::Success);

    SUBCASE("Parse multiple messages from single buffer") {
        // Create two messages
        auto msg1_opt = message_set.create("MULTI_TEST_MSG");
        auto msg2_opt = message_set.create("MULTI_TEST_MSG");
        REQUIRE(msg1_opt.has_value());
        REQUIRE(msg2_opt.has_value());
        
        auto msg1 = msg1_opt.value();
        auto msg2 = msg2_opt.value();
        
        msg1.set("counter", static_cast<uint8_t>(10));
        msg2.set("counter", static_cast<uint8_t>(20));
        
        mav::Identifier sender{1, 1};
        auto size1_opt = msg1.finalize(0, sender);
        auto size2_opt = msg2.finalize(1, sender);
        REQUIRE(size1_opt.has_value());
        REQUIRE(size2_opt.has_value());
        
        const uint8_t* wire1 = msg1.data();
        const uint8_t* wire2 = msg2.data();
        REQUIRE(wire1 != nullptr);
        REQUIRE(wire2 != nullptr);
        
        // Combine both messages into a single buffer
        std::vector<uint8_t> combined_buffer;
        combined_buffer.insert(combined_buffer.end(), wire1, wire1 + size1_opt.value());
        combined_buffer.insert(combined_buffer.end(), wire2, wire2 + size2_opt.value());
        
        // Parse first message
        BufferParser parser(message_set);
        size_t bytes_consumed = 0;
        auto parsed1 = parser.parseMessage(combined_buffer.data(), combined_buffer.size(), bytes_consumed);
        
        REQUIRE(parsed1.has_value());
        CHECK(bytes_consumed == size1_opt.value());
        
        uint8_t counter1;
        CHECK(parsed1->get("counter", counter1) == MessageResult::Success);
        CHECK(counter1 == 10);
        
        // Parse second message
        size_t bytes_consumed2 = 0;
        auto parsed2 = parser.parseMessage(
            combined_buffer.data() + bytes_consumed, 
            combined_buffer.size() - bytes_consumed, 
            bytes_consumed2);
        
        REQUIRE(parsed2.has_value());
        CHECK(bytes_consumed2 == size2_opt.value());
        
        uint8_t counter2;
        CHECK(parsed2->get("counter", counter2) == MessageResult::Success);
        CHECK(counter2 == 20);
    }
}