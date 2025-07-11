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

// Note: This test file is designed to be compiled with -fno-exceptions
// to verify the no-exceptions API works correctly
#define DOCTEST_CONFIG_NO_EXCEPTIONS_BUT_WITH_ALL_ASSERTS
#include "doctest.h"
#include "mav/MessageSet.h"
#include "mav/MessageFieldIterator.h"

using namespace mav;

TEST_CASE("No-exceptions API functionality") {
    
    SUBCASE("MessageSet creation and XML parsing") {
        MessageSet message_set;
        
        std::string simple_xml = R"(
            <mavlink>
                <enums>
                    <enum name="TEST_ENUM">
                        <entry name="TEST_VALUE" value="1"/>
                    </enum>
                </enums>
                <messages>
                    <message id="1" name="TEST_MESSAGE">
                        <field type="uint8_t" name="test_field"/>
                    </message>
                </messages>
            </mavlink>
        )";
        
        // Test XML parsing - should return Success
        auto xml_result = message_set.addFromXMLString(simple_xml);
        CHECK(xml_result == MessageSetResult::Success);
    }
    
    SUBCASE("Message creation") {
        MessageSet message_set;
        std::string simple_xml = R"(
            <mavlink>
                <messages>
                    <message id="1" name="TEST_MESSAGE">
                        <field type="uint8_t" name="test_field"/>
                    </message>
                </messages>
            </mavlink>
        )";
        
        message_set.addFromXMLString(simple_xml);
        
        // Test message creation - should return a valid message
        auto message_opt = message_set.create("TEST_MESSAGE");
        CHECK(message_opt.has_value());
        
        // Test invalid message creation - should return nullopt
        auto invalid_message_opt = message_set.create("INVALID_MESSAGE");
        CHECK_FALSE(invalid_message_opt.has_value());
    }
    
    SUBCASE("Field operations") {
        MessageSet message_set;
        std::string simple_xml = R"(
            <mavlink>
                <messages>
                    <message id="1" name="TEST_MESSAGE">
                        <field type="uint8_t" name="test_field"/>
                        <field type="char[10]" name="string_field"/>
                    </message>
                </messages>
            </mavlink>
        )";
        
        message_set.addFromXMLString(simple_xml);
        auto message = message_set.create("TEST_MESSAGE").value();
        
        // Test setting a field - should return Success
        auto set_result = message.set("test_field", uint8_t(42));
        CHECK(set_result == MessageResult::Success);
        
        // Test getting a field - should return Success
        uint8_t value;
        auto get_result = message.get("test_field", value);
        CHECK(get_result == MessageResult::Success);
        CHECK(value == 42);
        
        // Test setting invalid field - should return FieldNotFound
        auto invalid_set = message.set("invalid_field", uint8_t(1));
        CHECK(invalid_set == MessageResult::FieldNotFound);
        
        // Test getting invalid field - should return FieldNotFound
        uint8_t dummy_value;
        auto invalid_get = message.get("invalid_field", dummy_value);
        CHECK(invalid_get == MessageResult::FieldNotFound);
        
        // Test string operations
        auto string_set_result = message.setString("string_field", "hello");
        CHECK(string_set_result == MessageResult::Success);
        
        std::string string_value;
        auto string_get_result = message.getString("string_field", string_value);
        CHECK(string_get_result == MessageResult::Success);
        CHECK(string_value == "hello");
    }
    
    SUBCASE("Enum operations") {
        MessageSet message_set;
        std::string simple_xml = R"(
            <mavlink>
                <enums>
                    <enum name="TEST_ENUM">
                        <entry name="TEST_VALUE" value="42"/>
                    </enum>
                </enums>
            </mavlink>
        )";
        
        message_set.addFromXMLString(simple_xml);
        
        // Test enum getting - should return a valid value
        auto enum_opt = message_set.getEnum("TEST_VALUE");
        CHECK(enum_opt.has_value());
        CHECK(enum_opt.value() == 42);
        
        // Test invalid enum - should return nullopt
        auto invalid_enum_opt = message_set.getEnum("INVALID_ENUM");
        CHECK_FALSE(invalid_enum_opt.has_value());
    }
    
    SUBCASE("Message finalization") {
        MessageSet message_set;
        std::string simple_xml = R"(
            <mavlink>
                <messages>
                    <message id="1" name="TEST_MESSAGE">
                        <field type="uint8_t" name="test_field"/>
                    </message>
                </messages>
            </mavlink>
        )";
        
        message_set.addFromXMLString(simple_xml);
        auto message = message_set.create("TEST_MESSAGE").value();
        message.set("test_field", uint8_t(42));
        
        // Test finalization - should return a valid size
        Identifier sender(1, 1);
        auto finalize_result = message.finalize(0, sender);
        CHECK(finalize_result.has_value());
        CHECK(finalize_result.value() > 0);
    }
    
    SUBCASE("Error conditions") {
        MessageSet message_set;
        std::string simple_xml = R"(
            <mavlink>
                <messages>
                    <message id="1" name="TEST_MESSAGE">
                        <field type="uint8_t" name="test_field"/>
                    </message>
                </messages>
            </mavlink>
        )";
        message_set.addFromXMLString(simple_xml);
        
        // Test invalid message creation
        auto invalid_message = message_set.create("NON_EXISTENT");
        CHECK_FALSE(invalid_message.has_value());
        
        // Test invalid enum access
        auto invalid_enum = message_set.getEnum("NON_EXISTENT_ENUM");
        CHECK_FALSE(invalid_enum.has_value());
        
        // Test invalid message ID lookup
        auto invalid_id = message_set.idForMessage("NON_EXISTENT");
        CHECK_FALSE(invalid_id.has_value());
        
        // Test field errors on valid message
        auto message = message_set.create("TEST_MESSAGE").value();
        
        // Test setting invalid field
        auto invalid_field_set = message.set("invalid_field", uint8_t(1));
        CHECK(invalid_field_set == MessageResult::FieldNotFound);
        
        // Test getting invalid field
        uint8_t dummy;
        auto invalid_field_get = message.get("invalid_field", dummy);
        CHECK(invalid_field_get == MessageResult::FieldNotFound);
        
        // Test string operation on non-string field
        auto invalid_string_set = message.setString("test_field", "hello");
        CHECK(invalid_string_set == MessageResult::TypeMismatch);
        
        std::string dummy_string;
        auto invalid_string_get = message.getString("test_field", dummy_string);
        CHECK(invalid_string_get == MessageResult::TypeMismatch);
        
        // Test array bounds
        auto out_of_bounds = message.set("test_field", uint8_t(1), 999);
        CHECK(out_of_bounds == MessageResult::OutOfRange);
    }
    
    SUBCASE("XML parsing with valid but empty content") {
        MessageSet message_set;
        
        // Test valid but minimal XML
        std::string empty_xml = "<mavlink></mavlink>";
        auto result = message_set.addFromXMLString(empty_xml);
        CHECK(result == MessageSetResult::Success);
        
        // Should have no messages or enums
        auto no_message = message_set.create("ANY_MESSAGE");
        CHECK_FALSE(no_message.has_value());
        
        auto no_enum = message_set.getEnum("ANY_ENUM");
        CHECK_FALSE(no_enum.has_value());
    }
    
    SUBCASE("MessageFieldIterator functionality") {
        MessageSet message_set;
        std::string simple_xml = R"(
            <mavlink>
                <messages>
                    <message id="1" name="TEST_MESSAGE">
                        <field type="uint8_t" name="field1"/>
                        <field type="uint16_t" name="field2"/>
                    </message>
                </messages>
            </mavlink>
        )";
        
        message_set.addFromXMLString(simple_xml);
        auto message = message_set.create("TEST_MESSAGE").value();
        message.set("field1", uint8_t(10));
        message.set("field2", uint16_t(20));
        
        // Test iterator over message fields
        int field_count = 0;
        for (const auto& field_pair : FieldIterate(message)) {
            field_count++;
            CHECK(field_pair.first.size() > 0); // Field name should not be empty
        }
        CHECK(field_count == 2); // Should have exactly 2 fields
    }
    
    // Note: We don't test completely invalid XML (like "invalid xml") here because 
    // in no-exceptions mode, rapidxml will call our error handler which aborts the program. 
    // This is the expected behavior - malformed XML in no-exceptions mode terminates the program.
}