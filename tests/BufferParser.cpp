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
#include "mav/utils.h"
#include <vector>
#include <cstring>

using namespace mav;

// Build a valid MAVLink v1 wire packet.
// v1 layout: [0xFE, len, seq, sysid, compid, msgid(8-bit), payload..., crc_lo, crc_hi]
// CRC covers bytes [1..5+len] (everything except the magic byte) plus crc_extra.
static std::vector<uint8_t> buildV1Packet(
    uint8_t sysid, uint8_t compid, uint8_t msgid, uint8_t seq,
    const std::vector<uint8_t>& payload, uint8_t crc_extra)
{
    std::vector<uint8_t> pkt;
    pkt.push_back(0xFE);
    pkt.push_back(static_cast<uint8_t>(payload.size()));
    pkt.push_back(seq);
    pkt.push_back(sysid);
    pkt.push_back(compid);
    pkt.push_back(msgid);
    pkt.insert(pkt.end(), payload.begin(), payload.end());

    CRC crc;
    // Accumulate bytes starting after the magic byte
    for (size_t i = 1; i < pkt.size(); ++i) {
        crc.accumulate(pkt[i]);
    }
    crc.accumulate(crc_extra);

    const uint16_t c = crc.crc16();
    pkt.push_back(static_cast<uint8_t>(c & 0xFF));
    pkt.push_back(static_cast<uint8_t>((c >> 8) & 0xFF));
    return pkt;
}

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
// ── MAVLink v1 tests ──────────────────────────────────────────────────────────

static const char* V1_TEST_XML = R""""(
<mavlink>
    <messages>
        <message id="42" name="V1_TEST_MSG">
            <field type="uint8_t" name="field_a">Field A</field>
            <field type="uint8_t" name="field_b">Field B</field>
        </message>
    </messages>
</mavlink>
)"""";

TEST_CASE("BufferParser MAVLink v1 basic parsing") {
    MessageSet message_set;
    REQUIRE(message_set.addFromXMLString(V1_TEST_XML) == MessageSetResult::Success);

    // Obtain the crc_extra from the definition (same for v1 and v2)
    auto def_opt = message_set.getMessageDefinition(42);
    REQUIRE(def_opt);
    const uint8_t crc_extra = def_opt.get().crcExtra();

    BufferParser parser(message_set);

    SUBCASE("Parse a valid v1 packet") {
        // field_a=0xAA, field_b=0xBB; payload is in wire order (both uint8_t, no reordering)
        auto pkt = buildV1Packet(1, 1, 42, 0, {0xAA, 0xBB}, crc_extra);

        size_t consumed = 0;
        auto msg = parser.parseMessage(pkt.data(), pkt.size(), consumed);

        REQUIRE(msg.has_value());
        CHECK(consumed == pkt.size());
        CHECK(msg->name() == "V1_TEST_MSG");
        CHECK(msg->id() == 42);

        // Verify header fields are correctly extracted
        CHECK(msg->header().systemId() == 1);
        CHECK(msg->header().componentId() == 1);

        // Verify payload fields
        uint8_t a, b;
        CHECK(msg->get("field_a", a) == MessageResult::Success);
        CHECK(msg->get("field_b", b) == MessageResult::Success);
        CHECK(a == 0xAA);
        CHECK(b == 0xBB);
    }

    SUBCASE("v1 packet with bad CRC is rejected") {
        auto pkt = buildV1Packet(1, 1, 42, 0, {0x01, 0x02}, crc_extra);
        // Corrupt the CRC bytes
        pkt[pkt.size() - 2] ^= 0xFF;
        pkt[pkt.size() - 1] ^= 0xFF;

        size_t consumed = 0;
        auto msg = parser.parseMessage(pkt.data(), pkt.size(), consumed);

        CHECK_FALSE(msg.has_value());
        CHECK(consumed == pkt.size()); // Entire packet is consumed/skipped
    }

    SUBCASE("v1 packet with unknown message ID is skipped") {
        // msgid=99 is not in the MessageSet
        auto pkt = buildV1Packet(1, 1, 99, 0, {0xDE, 0xAD}, 0x00);

        size_t consumed = 0;
        auto msg = parser.parseMessage(pkt.data(), pkt.size(), consumed);

        CHECK_FALSE(msg.has_value());
        CHECK(consumed == pkt.size());
    }

    SUBCASE("Incomplete v1 header returns nullopt and waits") {
        // Only 4 bytes — not even a full 6-byte v1 header
        uint8_t truncated[] = {0xFE, 0x02, 0x00, 0x01};

        size_t consumed = 0;
        auto msg = parser.parseMessage(truncated, sizeof(truncated), consumed);

        CHECK_FALSE(msg.has_value());
        CHECK(consumed == 0); // Should stop at magic byte, not advance past it
    }

    SUBCASE("v1 header present but payload incomplete") {
        // Full header but only 1 of 2 payload bytes present
        auto pkt = buildV1Packet(1, 1, 42, 0, {0xAA, 0xBB}, crc_extra);
        // Trim to header + 1 payload byte (omit second payload byte and CRC)
        const size_t partial_size = 6 + 1; // V1_HEADER_SIZE + 1

        size_t consumed = 0;
        auto msg = parser.parseMessage(pkt.data(), partial_size, consumed);

        CHECK_FALSE(msg.has_value());
        CHECK(consumed == 0); // Hold at magic byte waiting for the rest
    }

    SUBCASE("Garbage bytes before v1 packet are skipped") {
        auto pkt = buildV1Packet(1, 1, 42, 0, {0x11, 0x22}, crc_extra);
        std::vector<uint8_t> buf = {0x00, 0x11, 0x22, 0x33}; // garbage
        buf.insert(buf.end(), pkt.begin(), pkt.end());

        size_t consumed = 0;
        auto msg = parser.parseMessage(buf.data(), buf.size(), consumed);

        REQUIRE(msg.has_value());
        CHECK(msg->name() == "V1_TEST_MSG");
        CHECK(consumed == 4 + pkt.size()); // 4 garbage bytes + full packet
    }
}

TEST_CASE("BufferParser mixed v1 and v2 messages") {
    MessageSet message_set;
    auto r1 = message_set.addFromXMLString(V1_TEST_XML);
    auto r2 = message_set.addFromXMLString(R""""(
<mavlink>
    <messages>
        <message id="100" name="V2_TEST_MSG">
            <field type="uint16_t" name="counter">Counter</field>
        </message>
    </messages>
</mavlink>
)"""");
    REQUIRE(r1 == MessageSetResult::Success);
    REQUIRE(r2 == MessageSetResult::Success);

    auto v1_def = message_set.getMessageDefinition(42);
    REQUIRE(v1_def);
    auto v1_pkt = buildV1Packet(1, 1, 42, 0, {0xAA, 0xBB}, v1_def.get().crcExtra());

    // Build a v2 message
    auto v2_msg_opt = message_set.create("V2_TEST_MSG");
    REQUIRE(v2_msg_opt.has_value());
    auto v2_msg = v2_msg_opt.value();
    v2_msg.set("counter", static_cast<uint16_t>(1234));
    auto v2_size_opt = v2_msg.finalize(1, mav::Identifier{1, 1});
    REQUIRE(v2_size_opt.has_value());
    const uint8_t* v2_wire = v2_msg.data();
    const size_t v2_size = static_cast<size_t>(v2_size_opt.value());

    SUBCASE("v1 followed by v2") {
        std::vector<uint8_t> buf(v1_pkt);
        buf.insert(buf.end(), v2_wire, v2_wire + v2_size);

        BufferParser parser(message_set);

        size_t c1 = 0;
        auto msg1 = parser.parseMessage(buf.data(), buf.size(), c1);
        REQUIRE(msg1.has_value());
        CHECK(msg1->name() == "V1_TEST_MSG");
        CHECK(c1 == v1_pkt.size());

        size_t c2 = 0;
        auto msg2 = parser.parseMessage(buf.data() + c1, buf.size() - c1, c2);
        REQUIRE(msg2.has_value());
        CHECK(msg2->name() == "V2_TEST_MSG");
        CHECK(c2 == v2_size);

        uint16_t counter;
        CHECK(msg2->get("counter", counter) == MessageResult::Success);
        CHECK(counter == 1234);
    }

    SUBCASE("v2 followed by v1") {
        std::vector<uint8_t> buf(v2_wire, v2_wire + v2_size);
        buf.insert(buf.end(), v1_pkt.begin(), v1_pkt.end());

        BufferParser parser(message_set);

        size_t c1 = 0;
        auto msg1 = parser.parseMessage(buf.data(), buf.size(), c1);
        REQUIRE(msg1.has_value());
        CHECK(msg1->name() == "V2_TEST_MSG");
        CHECK(c1 == v2_size);

        size_t c2 = 0;
        auto msg2 = parser.parseMessage(buf.data() + c1, buf.size() - c1, c2);
        REQUIRE(msg2.has_value());
        CHECK(msg2->name() == "V1_TEST_MSG");
        CHECK(c2 == v1_pkt.size());

        uint8_t a, b;
        CHECK(msg2->get("field_a", a) == MessageResult::Success);
        CHECK(msg2->get("field_b", b) == MessageResult::Success);
        CHECK(a == 0xAA);
        CHECK(b == 0xBB);
    }
}
