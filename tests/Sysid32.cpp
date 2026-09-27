/****************************************************************************
 *
 * Copyright (c) 2023, libmav development team
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
#include <string>
#include <vector>

using namespace mav;

// 32 bit system IDs (IFLAG_SYSID32) and extended header targeting
// (IFLAG_TARGET32).
//
// The golden frames below are copied verbatim from pymavlink's
// tests/test_sysid32.py, as merged in ArduPilot/pymavlink#1229. Wire compatibility with
// every other MAVLink implementation rests on these matching byte for byte,
// so they are the point of this file: everything else here is a round trip
// against them.
//
// All frames are COMMAND_LONG packed from
// (sysid, compid=11, target, target_component=250, command=300,
//  confirmation=1, param1..param7 = 1.0 .. 7.0) at seq 0.

namespace {

constexpr uint32_t sysid_small = 42;
constexpr uint32_t sysid_big = 0x0A000001; // 10.0.0.1
constexpr uint32_t target_small = 7;
constexpr uint32_t target_big = 0x0A000002; // 10.0.0.2

constexpr uint8_t compid = 11;
constexpr uint8_t target_component = 250;
constexpr uint16_t command = 300;
constexpr uint8_t confirmation = 1;

const char command_long_xml[] = R""""(
<?xml version="1.0"?>
<mavlink>
  <version>3</version>
  <messages>
    <message id="76" name="COMMAND_LONG">
      <field type="uint8_t" name="target_system">System which should execute the command</field>
      <field type="uint8_t" name="target_component">Component which should execute the command, 0 for all components</field>
      <field type="uint16_t" name="command">Command ID (of command to send).</field>
      <field type="uint8_t" name="confirmation">0: First transmission of this command.</field>
      <field type="float" name="param1">Parameter 1 (for the specific command).</field>
      <field type="float" name="param2">Parameter 2 (for the specific command).</field>
      <field type="float" name="param3">Parameter 3 (for the specific command).</field>
      <field type="float" name="param4">Parameter 4 (for the specific command).</field>
      <field type="float" name="param5">Parameter 5 (for the specific command).</field>
      <field type="float" name="param6">Parameter 6 (for the specific command).</field>
      <field type="float" name="param7">Parameter 7 (for the specific command).</field>
    </message>
  </messages>
</mavlink>
)"""";

// no extended fields, targets fit in 8 bits
const char* golden_small_small =
    "fd210000002a0b4c00000000803f0000004000004040000080400000a0400000c0400000e0402c0107fa01ce94";
// IFLAG_TARGET32 only
const char* golden_small_big =
    "fd210400002a0b4c00000200000a0000803f0000004000004040000080400000a0400000c0400000e0402c01fffa018df2";
// IFLAG_SYSID32 only
const char* golden_big_small =
    "fd210200000100000a0b4c00000000803f0000004000004040000080400000a0400000c0400000e0402c0107fa019d0f";
// IFLAG_SYSID32 | IFLAG_TARGET32
const char* golden_big_big =
    "fd210600000100000a0b4c00000200000a0000803f0000004000004040000080400000a0400000c0400000e0402c01fffa0140e5";
// IFLAG_SIGNED | IFLAG_SYSID32 | IFLAG_TARGET32, key bytes([42]*32),
// link id 3, timestamp 1000
const char* golden_signed =
    "fd210700000100000a0b4c00000200000a0000803f0000004000004040000080400000a0"
    "400000c0400000e0402c01fffa01d03903e80300000000349dea1b799d";

std::vector<uint8_t> fromHex(const std::string& hex) {
    std::vector<uint8_t> out;
    out.reserve(hex.size() / 2);
    for (size_t i = 0; i + 1 < hex.size(); i += 2) {
        const auto hi = hex[i];
        const auto lo = hex[i + 1];
        auto nibble = [](char c) -> uint8_t {
            if (c >= '0' && c <= '9') {
                return static_cast<uint8_t>(c - '0');
            }
            return static_cast<uint8_t>(c - 'a' + 10);
        };
        out.push_back(static_cast<uint8_t>((nibble(hi) << 4) | nibble(lo)));
    }
    return out;
}

std::string toHex(const uint8_t* data, size_t size) {
    static const char* digits = "0123456789abcdef";
    std::string out;
    out.reserve(size * 2);
    for (size_t i = 0; i < size; i++) {
        out.push_back(digits[data[i] >> 4]);
        out.push_back(digits[data[i] & 0x0F]);
    }
    return out;
}

MessageSet makeMessageSet() {
    MessageSet message_set;
    const auto result = message_set.addFromXMLString(command_long_xml);
    REQUIRE_EQ(result, MessageSetResult::Success);
    return message_set;
}

// Build the COMMAND_LONG used for all golden frames, minus the finalize step.
Message makeCommandLong(MessageSet& message_set, uint32_t target) {
    auto message_opt = message_set.create("COMMAND_LONG");
    REQUIRE(message_opt.has_value());
    auto message = message_opt.value();

    // Targets above 255 do not fit the payload field and travel in the
    // extended header instead.
    if (target > 255) {
        message.setExtendedTarget(target);
    } else {
        message.set("target_system", static_cast<uint8_t>(target));
    }
    message.set("target_component", target_component);
    message.set("command", command);
    message.set("confirmation", confirmation);
    message.set("param1", 1.0f);
    message.set("param2", 2.0f);
    message.set("param3", 3.0f);
    message.set("param4", 4.0f);
    message.set("param5", 5.0f);
    message.set("param6", 6.0f);
    message.set("param7", 7.0f);
    return message;
}

} // namespace

TEST_CASE("32 bit system IDs serialize to the C implementation's bytes") {
    auto message_set = makeMessageSet();

    struct Case {
        std::string name;
        uint32_t sysid;
        uint32_t target;
        const char* golden;
    };

    const std::vector<Case> cases = {
        {"8 bit sysid, 8 bit target", sysid_small, target_small, golden_small_small},
        {"8 bit sysid, 32 bit target", sysid_small, target_big, golden_small_big},
        {"32 bit sysid, 8 bit target", sysid_big, target_small, golden_big_small},
        {"32 bit sysid, 32 bit target", sysid_big, target_big, golden_big_big},
    };

    for (const auto& c : cases) {
        CAPTURE(c.name);
        auto message = makeCommandLong(message_set, c.target);
        auto size_opt = message.finalize(0, Identifier{c.sysid, compid});
        REQUIRE(size_opt.has_value());
        CHECK_EQ(toHex(message.data(), size_opt.value()), std::string{c.golden});
        CHECK_EQ(message.finalizedSize(), size_opt.value());
    }
}

TEST_CASE("32 bit system IDs parse from the C implementation's bytes") {
    auto message_set = makeMessageSet();
    BufferParser parser{message_set};

    SUBCASE("8 bit sysid, 8 bit target") {
        const auto frame = fromHex(golden_small_small);
        size_t consumed = 0;
        auto message_opt = parser.parseMessage(frame.data(), frame.size(), consumed);
        REQUIRE(message_opt.has_value());
        auto& message = message_opt.value();

        CHECK_EQ(consumed, frame.size());
        CHECK_EQ(message.header().incompatFlags(), 0);
        CHECK_EQ(message.header().size(), 10);
        CHECK_EQ(message.header().systemId(), sysid_small);
        CHECK_EQ(message.header().componentId(), compid);

        // No extended header, so the target is in the payload as usual.
        CHECK_EQ(message.extendedTargetSystemId(), 0);
        uint8_t payload_target = 0;
        CHECK_EQ(message.get("target_system", payload_target), MessageResult::Success);
        CHECK_EQ(payload_target, target_small);
    }

    SUBCASE("8 bit sysid, 32 bit target") {
        const auto frame = fromHex(golden_small_big);
        size_t consumed = 0;
        auto message_opt = parser.parseMessage(frame.data(), frame.size(), consumed);
        REQUIRE(message_opt.has_value());
        auto& message = message_opt.value();

        CHECK_EQ(consumed, frame.size());
        CHECK_EQ(message.header().incompatFlags(), IFLAG_TARGET32);
        CHECK_EQ(message.header().size(), 14);
        CHECK_EQ(message.header().systemId(), sysid_small);
        CHECK_EQ(message.header().componentId(), compid);
        CHECK_EQ(message.extendedTargetSystemId(), target_big);

        // The payload target_system reads as the sentinel when the real target
        // is in the header. Callers must consult extendedTargetSystemId() first.
        uint8_t payload_target = 0;
        CHECK_EQ(message.get("target_system", payload_target), MessageResult::Success);
        CHECK_EQ(payload_target, TARGET_SYSTEM_SENTINEL);

        // The target component is not part of the extended header.
        uint8_t payload_target_component = 0;
        CHECK_EQ(
            message.get("target_component", payload_target_component), MessageResult::Success);
        CHECK_EQ(payload_target_component, target_component);
    }

    SUBCASE("32 bit sysid, 8 bit target") {
        const auto frame = fromHex(golden_big_small);
        size_t consumed = 0;
        auto message_opt = parser.parseMessage(frame.data(), frame.size(), consumed);
        REQUIRE(message_opt.has_value());
        auto& message = message_opt.value();

        CHECK_EQ(consumed, frame.size());
        CHECK_EQ(message.header().incompatFlags(), IFLAG_SYSID32);
        CHECK_EQ(message.header().size(), 13);
        CHECK_EQ(message.header().systemId(), sysid_big);
        CHECK_EQ(message.header().componentId(), compid);
        CHECK_EQ(message.extendedTargetSystemId(), 0);

        uint8_t payload_target = 0;
        CHECK_EQ(message.get("target_system", payload_target), MessageResult::Success);
        CHECK_EQ(payload_target, target_small);
    }

    SUBCASE("32 bit sysid, 32 bit target") {
        const auto frame = fromHex(golden_big_big);
        size_t consumed = 0;
        auto message_opt = parser.parseMessage(frame.data(), frame.size(), consumed);
        REQUIRE(message_opt.has_value());
        auto& message = message_opt.value();

        CHECK_EQ(consumed, frame.size());
        CHECK_EQ(message.header().incompatFlags(), IFLAG_SYSID32 | IFLAG_TARGET32);
        CHECK_EQ(message.header().size(), 17);
        CHECK_EQ(message.header().systemId(), sysid_big);
        CHECK_EQ(message.header().componentId(), compid);
        CHECK_EQ(message.extendedTargetSystemId(), target_big);
    }

    SUBCASE("payload survives the widest header") {
        const auto frame = fromHex(golden_big_big);
        size_t consumed = 0;
        auto message_opt = parser.parseMessage(frame.data(), frame.size(), consumed);
        REQUIRE(message_opt.has_value());
        auto& message = message_opt.value();

        // Field offsets are payload relative, so a 17 byte header must not
        // shift what the fields read.
        uint16_t read_command = 0;
        CHECK_EQ(message.get("command", read_command), MessageResult::Success);
        CHECK_EQ(read_command, command);

        uint8_t read_confirmation = 0;
        CHECK_EQ(message.get("confirmation", read_confirmation), MessageResult::Success);
        CHECK_EQ(read_confirmation, confirmation);

        uint8_t read_target_component = 0;
        CHECK_EQ(message.get("target_component", read_target_component), MessageResult::Success);
        CHECK_EQ(read_target_component, target_component);

        for (int i = 1; i <= 7; i++) {
            float value = 0.0f;
            CHECK_EQ(message.get("param" + std::to_string(i), value), MessageResult::Success);
            CHECK_EQ(value, static_cast<float>(i));
        }
    }
}

TEST_CASE("Signed frames with extended headers match the C implementation") {
    auto message_set = makeMessageSet();
    std::array<uint8_t, MessageDefinition::KEY_SIZE> key{};
    key.fill(42);

    auto message = makeCommandLong(message_set, target_big);
    auto size_opt = message.finalize(0, Identifier{sysid_big, compid}, key, 1000, 3);
    REQUIRE(size_opt.has_value());
    CHECK_EQ(toHex(message.data(), size_opt.value()), std::string{golden_signed});

    // The hash covers the extended header, so validation has to agree.
    auto valid_opt = message.validate(key);
    REQUIRE(valid_opt.has_value());
    CHECK(valid_opt.value());
}

TEST_CASE("Unknown incompat flags are rejected rather than mis-parsed") {
    auto message_set = makeMessageSet();
    BufferParser parser{message_set};

    auto frame = fromHex(golden_small_small);
    // 0x08 is not a flag we know, so we cannot tell where the payload starts.
    frame[2] = 0x08;

    size_t consumed = 0;
    auto message_opt = parser.parseMessage(frame.data(), frame.size(), consumed);
    CHECK(!message_opt.has_value());
    // Only the magic byte is dropped, so a real frame starting later in the
    // buffer is still found on the next pass.
    CHECK_EQ(consumed, 1);
}

TEST_CASE("A 32 bit frame is found after leading garbage and before a second frame") {
    auto message_set = makeMessageSet();
    BufferParser parser{message_set};

    const auto frame = fromHex(golden_big_big);
    std::vector<uint8_t> buffer{0x11, 0x22, 0x33};
    buffer.insert(buffer.end(), frame.begin(), frame.end());
    buffer.insert(buffer.end(), frame.begin(), frame.end());

    size_t consumed = 0;
    auto first = parser.parseMessage(buffer.data(), buffer.size(), consumed);
    REQUIRE(first.has_value());
    CHECK_EQ(consumed, 3 + frame.size());
    CHECK_EQ(first.value().header().systemId(), sysid_big);

    size_t consumed_second = 0;
    auto second = parser.parseMessage(
        buffer.data() + consumed, buffer.size() - consumed, consumed_second);
    REQUIRE(second.has_value());
    CHECK_EQ(consumed_second, frame.size());
    CHECK_EQ(second.value().header().systemId(), sysid_big);
}

TEST_CASE("An incomplete extended header waits for more data") {
    auto message_set = makeMessageSet();
    BufferParser parser{message_set};

    const auto frame = fromHex(golden_big_big);

    // Anything shorter than the full 17 byte header cannot be sized yet.
    for (size_t len = 1; len < 17; len++) {
        CAPTURE(len);
        size_t consumed = 0;
        auto message_opt = parser.parseMessage(frame.data(), len, consumed);
        CHECK(!message_opt.has_value());
        CHECK_EQ(consumed, 0);
    }

    // Nor can a complete header with a partial payload.
    size_t consumed = 0;
    auto message_opt = parser.parseMessage(frame.data(), frame.size() - 1, consumed);
    CHECK(!message_opt.has_value());
    CHECK_EQ(consumed, 0);
}

TEST_CASE("8 bit senders are unaffected") {
    auto message_set = makeMessageSet();
    BufferParser parser{message_set};

    // A system ID that fits in 8 bits must not set IFLAG_SYSID32, otherwise
    // peers that predate this feature stop being able to parse us at all.
    auto message = makeCommandLong(message_set, target_small);
    auto size_opt = message.finalize(0, Identifier{255, compid});
    REQUIRE(size_opt.has_value());
    CHECK_EQ(message.header().incompatFlags(), 0);
    CHECK_EQ(message.header().size(), 10);

    size_t consumed = 0;
    auto parsed = parser.parseMessage(message.data(), size_opt.value(), consumed);
    REQUIRE(parsed.has_value());
    CHECK_EQ(parsed.value().header().systemId(), 255);
}

TEST_CASE("A received extended target survives being sent on") {
    auto message_set = makeMessageSet();
    BufferParser parser{message_set};

    const auto frame = fromHex(golden_small_big);
    size_t consumed = 0;
    auto message_opt = parser.parseMessage(frame.data(), frame.size(), consumed);
    REQUIRE(message_opt.has_value());
    auto message = message_opt.value();

    // Re-finalizing, e.g. when forwarding, must not turn the sentinel in the
    // payload into a target of system 255.
    auto size_opt = message.finalize(0, Identifier{sysid_small, compid});
    REQUIRE(size_opt.has_value());
    CHECK_EQ(toHex(message.data(), size_opt.value()), std::string{golden_small_big});
    CHECK_EQ(message.extendedTargetSystemId(), target_big);
}

TEST_CASE("Retargeting a received message to an 8 bit system drops the extended target") {
    auto message_set = makeMessageSet();
    BufferParser parser{message_set};

    const auto frame = fromHex(golden_small_big);
    size_t consumed = 0;
    auto message_opt = parser.parseMessage(frame.data(), frame.size(), consumed);
    REQUIRE(message_opt.has_value());
    auto message = message_opt.value();

    CHECK_EQ(message.set("target_system", static_cast<uint8_t>(target_small)), MessageResult::Success);
    auto size_opt = message.finalize(0, Identifier{sysid_small, compid});
    REQUIRE(size_opt.has_value());
    CHECK_EQ(toHex(message.data(), size_opt.value()), std::string{golden_small_small});
    CHECK_EQ(message.extendedTargetSystemId(), 0);
}
