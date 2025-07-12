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

#include <filesystem>
#include "doctest.h"
#include "mav/MessageSet.h"
#include "minimal.h"
#include <cstdio>
#include <fstream>

using namespace mav;

std::string dump_minimal_xml() {
    // write temporary file
    auto temp_path = std::filesystem::temp_directory_path();
    auto xml_file = temp_path / "minimal.xml";
    std::ofstream out(xml_file);
    out << minimal_xml;
    out.close();
    return xml_file;
}

TEST_CASE("Message set creation") {

    auto file_name = dump_minimal_xml();
    MessageSet message_set;
    auto result = message_set.addFromXML(file_name);
    std::remove(file_name.c_str());

    CHECK_EQ(result, MessageSetResult::Success);
    CHECK_EQ(message_set.size(), 2);

    SUBCASE("Can not parse incomplete XML") {
        // This is not a valid XML file - in no-exceptions mode, rapidxml will abort on malformed XML
        // so we test for valid but empty XML instead
        auto empty_result = message_set.addFromXMLString("");
        CHECK_EQ(empty_result, MessageSetResult::XmlParseError);
        
        // XML file is missing the closing tag - will also cause rapidxml to abort
        auto incomplete_result = message_set.addFromXMLString("<mavlink>");
        CHECK_EQ(incomplete_result, MessageSetResult::XmlParseError);
    }

    SUBCASE("Can not add message with unknown field type") {
        auto unknown_field_result = message_set.addFromXMLString(R""""(
<mavlink>
    <messages>
        <message id="15321" name="ONLY_MESSAGE">
            <field type="unknown" name="field">Field</field>
        </message>
    </messages>
</mavlink>
)"""");
        CHECK_EQ(unknown_field_result, MessageSetResult::XmlParseError);
    }

    SUBCASE("Can add valid, partial XML") {
        // This is a valid XML file, but it does not contain any messages or enums
        auto empty_result = message_set.addFromXMLString("<mavlink></mavlink>");
        CHECK_EQ(empty_result, MessageSetResult::Success);
        
        // only messages
        auto message_result = message_set.addFromXMLString(R""""(
<mavlink>
    <messages>
        <message id="15321" name="ONLY_MESSAGE">
            <field type="uint8_t" name="field">Field</field>
        </message>
    </messages>
</mavlink>
)"""");
        CHECK_EQ(message_result, MessageSetResult::Success);
        CHECK(message_set.contains("ONLY_MESSAGE"));
        
        auto message_opt = message_set.create("ONLY_MESSAGE");
        REQUIRE(message_opt.has_value());
        auto message = message_opt.value();
        CHECK_EQ(message.id(), 15321);
        CHECK_EQ(message.name(), "ONLY_MESSAGE");

        // only enums
        auto enum_result = message_set.addFromXMLString(R""""(
<mavlink>
    <enums>
        <enum name="TEST_ENUM">
            <entry name="TEST_VALUE" value="1"/>
        </enum>
    </enums>
</mavlink>
)"""");
        CHECK_EQ(enum_result, MessageSetResult::Success);
        
        auto enum_opt = message_set.getEnum("TEST_VALUE");
        REQUIRE(enum_opt.has_value());
        CHECK_EQ(enum_opt.value(), 1);
    }

    SUBCASE("Can parse multiple XML parts") {
        // Test that we can add multiple XML strings to the same message set
        auto result1 = message_set.addFromXMLString(R""""(
<mavlink>
    <messages>
        <message id="999" name="FIRST_MESSAGE">
            <field type="uint16_t" name="data">Data field</field>
        </message>
    </messages>
</mavlink>
)"""");
        CHECK_EQ(result1, MessageSetResult::Success);

        auto result2 = message_set.addFromXMLString(R""""(
<mavlink>
    <messages>
        <message id="1000" name="SECOND_MESSAGE">
            <field type="uint32_t" name="value">Value field</field>
        </message>
    </messages>
</mavlink>
)"""");
        CHECK_EQ(result2, MessageSetResult::Success);

        CHECK(message_set.contains("FIRST_MESSAGE"));
        CHECK(message_set.contains("SECOND_MESSAGE"));
        CHECK(message_set.contains(999));
        CHECK(message_set.contains(1000));
    }

    SUBCASE("Can handle various enum formats") {
        auto hex_result = message_set.addFromXMLString(R""""(
<mavlink>
    <enums>
        <enum name="HEX_ENUM">
            <entry name="HEX_VALUE" value="0xFF"/>
        </enum>
    </enums>
</mavlink>
)"""");
        CHECK_EQ(hex_result, MessageSetResult::Success);
        
        auto hex_enum = message_set.getEnum("HEX_VALUE");
        REQUIRE(hex_enum.has_value());
        CHECK_EQ(hex_enum.value(), 255);

        auto binary_result = message_set.addFromXMLString(R""""(
<mavlink>
    <enums>
        <enum name="BINARY_ENUM">
            <entry name="BINARY_VALUE" value="0b1010"/>
        </enum>
    </enums>
</mavlink>
)"""");
        CHECK_EQ(binary_result, MessageSetResult::Success);
        
        auto binary_enum = message_set.getEnum("BINARY_VALUE");
        REQUIRE(binary_enum.has_value());
        CHECK_EQ(binary_enum.value(), 10);

        auto power_result = message_set.addFromXMLString(R""""(
<mavlink>
    <enums>
        <enum name="POWER_ENUM">
            <entry name="POWER_VALUE" value="2**3"/>
        </enum>
    </enums>
</mavlink>
)"""");
        CHECK_EQ(power_result, MessageSetResult::Success);
        
        auto power_enum = message_set.getEnum("POWER_VALUE");
        REQUIRE(power_enum.has_value());
        CHECK_EQ(power_enum.value(), 8);
    }

    SUBCASE("Can handle invalid enum values") {
        auto invalid_result = message_set.addFromXMLString(R""""(
<mavlink>
    <enums>
        <enum name="INVALID_ENUM">
            <entry name="INVALID_VALUE" value="notanumber"/>
        </enum>
    </enums>
</mavlink>
)"""");
        CHECK_EQ(invalid_result, MessageSetResult::XmlParseError);
    }

    SUBCASE("Can handle array field types") {
        auto array_result = message_set.addFromXMLString(R""""(
<mavlink>
    <messages>
        <message id="2000" name="ARRAY_MESSAGE">
            <field type="uint8_t[10]" name="array_field">Array field</field>
            <field type="char[20]" name="string_field">String field</field>
        </message>
    </messages>
</mavlink>
)"""");
        CHECK_EQ(array_result, MessageSetResult::Success);
        
        auto message_opt = message_set.create("ARRAY_MESSAGE");
        REQUIRE(message_opt.has_value());
        auto message = message_opt.value();
        
        // Test setting array values
        std::vector<uint8_t> test_array = {1, 2, 3, 4, 5};
        auto set_result = message.set("array_field", test_array);
        CHECK_EQ(set_result, MessageResult::Success);
        
        // Test setting string
        auto string_result = message.setString("string_field", "test");
        CHECK_EQ(string_result, MessageResult::Success);
        
        // Test getting values back
        std::vector<uint8_t> retrieved_array;
        auto get_result = message.get("array_field", retrieved_array);
        CHECK_EQ(get_result, MessageResult::Success);
        CHECK_EQ(retrieved_array.size(), 10); // Should be resized to field size
        CHECK_EQ(retrieved_array[0], 1);
        CHECK_EQ(retrieved_array[4], 5);
        
        std::string retrieved_string;
        auto string_get_result = message.getString("string_field", retrieved_string);
        CHECK_EQ(string_get_result, MessageResult::Success);
        CHECK_EQ(retrieved_string, "test");
    }

    SUBCASE("Message creation and field access") {
        auto message_opt = message_set.create("HEARTBEAT");
        REQUIRE(message_opt.has_value());
        auto message = message_opt.value();
        
        // Test field setting and getting
        auto set_result = message.set("type", static_cast<uint8_t>(42));
        CHECK_EQ(set_result, MessageResult::Success);
        
        uint8_t retrieved_value;
        auto get_result = message.get("type", retrieved_value);
        CHECK_EQ(get_result, MessageResult::Success);
        CHECK_EQ(retrieved_value, 42);
        
        // Test invalid field access
        auto invalid_set = message.set("nonexistent_field", static_cast<uint8_t>(1));
        CHECK_EQ(invalid_set, MessageResult::FieldNotFound);
        
        uint8_t dummy;
        auto invalid_get = message.get("nonexistent_field", dummy);
        CHECK_EQ(invalid_get, MessageResult::FieldNotFound);
    }

    SUBCASE("Message finalization") {
        auto message_opt = message_set.create("HEARTBEAT");
        REQUIRE(message_opt.has_value());
        auto message = message_opt.value();
        
        // Set some field values
        message.set("type", static_cast<uint8_t>(1));
        message.set("autopilot", static_cast<uint8_t>(2));
        
        // Test finalization
        Identifier sender(1, 1);
        auto finalize_result = message.finalize(0, sender);
        REQUIRE(finalize_result.has_value());
        CHECK_GT(finalize_result.value(), 0);
    }

    SUBCASE("Enum access") {
        auto enum_opt = message_set.getEnum("MAV_TYPE_GENERIC");
        REQUIRE(enum_opt.has_value());
        CHECK_EQ(enum_opt.value(), 0);
        
        // Test nonexistent enum
        auto invalid_enum = message_set.getEnum("NONEXISTENT_ENUM");
        CHECK_FALSE(invalid_enum.has_value());
    }

    SUBCASE("Message ID lookup") {
        auto id_opt = message_set.idForMessage("HEARTBEAT");
        REQUIRE(id_opt.has_value());
        CHECK_EQ(id_opt.value(), 0);
        
        // Test nonexistent message
        auto invalid_id = message_set.idForMessage("NONEXISTENT_MESSAGE");
        CHECK_FALSE(invalid_id.has_value());
    }
}

TEST_CASE("Message set edge cases") {
    MessageSet message_set;
    
    SUBCASE("Empty message set operations") {
        CHECK_EQ(message_set.size(), 0);
        CHECK_FALSE(message_set.contains("ANY_MESSAGE"));
        CHECK_FALSE(message_set.contains(999));
        
        auto no_message = message_set.create("NONEXISTENT");
        CHECK_FALSE(no_message.has_value());
        
        auto no_enum = message_set.getEnum("NONEXISTENT");
        CHECK_FALSE(no_enum.has_value());
        
        auto no_id = message_set.idForMessage("NONEXISTENT");
        CHECK_FALSE(no_id.has_value());
    }
    
    SUBCASE("Malformed XML handling") {
        // In no-exceptions mode, completely malformed XML will cause abort
        // Test edge cases that don't cause abort but return parse errors
        auto result = message_set.addFromXMLString("not xml at all");
        CHECK_EQ(result, MessageSetResult::XmlParseError);
    }
}