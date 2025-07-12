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
#include "mav/MessageSet.h"
#include "mav/Message.h"

using namespace mav;

TEST_CASE("Message operations") {
    MessageSet message_set;
    auto result = message_set.addFromXMLString(R""""(
<mavlink>
    <messages>
        <message id="9915" name="BIG_MESSAGE">
            <field type="uint8_t" name="uint8_field">description</field>
            <field type="int8_t" name="int8_field">description</field>
            <field type="uint16_t" name="uint16_field">description</field>
            <field type="int16_t" name="int16_field">description</field>
            <field type="uint32_t" name="uint32_field">description</field>
            <field type="int32_t" name="int32_field">description</field>
            <field type="uint64_t" name="uint64_field">description</field>
            <field type="int64_t" name="int64_field">description</field>
            <field type="double" name="double_field">description</field>
            <field type="float" name="float_field">description</field>
            <field type="char[20]" name="char_arr_field">description</field>
            <field type="float[3]" name="float_arr_field">description</field>
            <field type="int32_t[3]" name="int32_arr_field">description</field>
        </message>
        <message id="9916" name="UINT8_ONLY_MESSAGE">
            <field type="uint8_t" name="field1">description</field>
            <field type="uint8_t" name="field2">description</field>
            <field type="uint8_t" name="field3">description</field>
            <field type="uint8_t" name="field4">description</field>
        </message>
        <message id="9917" name="ARRAY_ONLY_MESSAGE">
            <field type="char[6]" name="field1">description</field>
            <field type="uint8_t[3]" name="field2">description</field>
            <field type="uint16_t[3]" name="field3">description</field>
            <field type="uint32_t[3]" name="field4">description</field>
            <field type="uint64_t[3]" name="field5">description</field>
            <field type="int8_t[3]" name="field6">description</field>
            <field type="int16_t[3]" name="field7">description</field>
            <field type="int32_t[3]" name="field8">description</field>
            <field type="int64_t[3]" name="field9">description</field>
            <field type="float[3]" name="field10">description</field>
            <field type="double[3]" name="field11">description</field>
        </message>
    </messages>
</mavlink>
)"""");

    REQUIRE_EQ(result, MessageSetResult::Success);
    REQUIRE(message_set.contains("BIG_MESSAGE"));
    REQUIRE_EQ(message_set.size(), 3);

    auto message_opt = message_set.create("BIG_MESSAGE");
    REQUIRE(message_opt.has_value());
    auto message = message_opt.value();
    
    auto id_opt = message_set.idForMessage("BIG_MESSAGE");
    REQUIRE(id_opt.has_value());
    CHECK_EQ(message.id(), id_opt.value());
    CHECK_EQ(message.name(), "BIG_MESSAGE");

    SUBCASE("Can set and get values with set / get API") {
        CHECK_EQ(message.set("uint8_field", static_cast<uint8_t>(0x12)), MessageResult::Success);
        CHECK_EQ(message.set("int8_field", static_cast<int8_t>(0x12)), MessageResult::Success);
        CHECK_EQ(message.set("uint16_field", static_cast<uint16_t>(0x1234)), MessageResult::Success);
        CHECK_EQ(message.set("int16_field", static_cast<int16_t>(0x1234)), MessageResult::Success);
        CHECK_EQ(message.set("uint32_field", static_cast<uint32_t>(0x12345678)), MessageResult::Success);
        CHECK_EQ(message.set("int32_field", static_cast<int32_t>(0x12345678)), MessageResult::Success);
        CHECK_EQ(message.set("uint64_field", static_cast<uint64_t>(0x1234567890ABCDEF)), MessageResult::Success);
        CHECK_EQ(message.set("int64_field", static_cast<int64_t>(0x1234567890ABCDEF)), MessageResult::Success);
        CHECK_EQ(message.set("double_field", 0.123456789), MessageResult::Success);
        CHECK_EQ(message.set("float_field", 0.123456789f), MessageResult::Success);
        CHECK_EQ(message.setString("char_arr_field", "Hello World!"), MessageResult::Success);
        CHECK_EQ(message.set("float_arr_field", std::vector<float>{1.0, 2.0, 3.0}), MessageResult::Success);
        CHECK_EQ(message.set("int32_arr_field", std::vector<int32_t>{1, 2, 3}), MessageResult::Success);
        
        uint8_t uint8_val;
        CHECK_EQ(message.get("uint8_field", uint8_val), MessageResult::Success);
        CHECK_EQ(uint8_val, 0x12);
        
        int8_t int8_val;
        CHECK_EQ(message.get("int8_field", int8_val), MessageResult::Success);
        CHECK_EQ(int8_val, 0x12);
        
        uint16_t uint16_val;
        CHECK_EQ(message.get("uint16_field", uint16_val), MessageResult::Success);
        CHECK_EQ(uint16_val, 0x1234);
        
        int16_t int16_val;
        CHECK_EQ(message.get("int16_field", int16_val), MessageResult::Success);
        CHECK_EQ(int16_val, 0x1234);
        
        uint32_t uint32_val;
        CHECK_EQ(message.get("uint32_field", uint32_val), MessageResult::Success);
        CHECK_EQ(uint32_val, 0x12345678);
        
        int32_t int32_val;
        CHECK_EQ(message.get("int32_field", int32_val), MessageResult::Success);
        CHECK_EQ(int32_val, 0x12345678);
        
        uint64_t uint64_val;
        CHECK_EQ(message.get("uint64_field", uint64_val), MessageResult::Success);
        CHECK_EQ(uint64_val, 0x1234567890ABCDEF);
        
        int64_t int64_val;
        CHECK_EQ(message.get("int64_field", int64_val), MessageResult::Success);
        CHECK_EQ(int64_val, 0x1234567890ABCDEF);
        
        double double_val;
        CHECK_EQ(message.get("double_field", double_val), MessageResult::Success);
        CHECK_EQ(double_val, doctest::Approx(0.123456789));
        
        float float_val;
        CHECK_EQ(message.get("float_field", float_val), MessageResult::Success);
        CHECK_EQ(float_val, doctest::Approx(0.123456789f));
        
        std::string string_val;
        CHECK_EQ(message.getString("char_arr_field", string_val), MessageResult::Success);
        CHECK_EQ(string_val, "Hello World!");
        
        std::vector<float> float_arr_val;
        CHECK_EQ(message.get("float_arr_field", float_arr_val), MessageResult::Success);
        CHECK_EQ(float_arr_val, std::vector<float>{1.0, 2.0, 3.0});
        
        std::vector<int32_t> int32_arr_val;
        CHECK_EQ(message.get("int32_arr_field", int32_arr_val), MessageResult::Success);
        CHECK_EQ(int32_arr_val, std::vector<int32_t>{1, 2, 3});
    }

    SUBCASE("Have fields truncated by zero-elision") {
        CHECK_EQ(message.set("int64_field", static_cast<int64_t>(34)), MessageResult::Success); // since largest field, will be at the end of the message
        
        int64_t val;
        CHECK_EQ(message.get("int64_field", val), MessageResult::Success);
        CHECK_EQ(val, 34);
        
        auto finalize_result = message.finalize(1, {2, 3});
        REQUIRE(finalize_result.has_value());
        
        CHECK_EQ(message.get("int64_field", val), MessageResult::Success);
        CHECK_EQ(val, 34);
    }

    SUBCASE("Can set and get values with assignment API") {
        CHECK_EQ(message["uint8_field"] = static_cast<uint8_t>(0x12), MessageResult::Success);
        CHECK_EQ(message["int8_field"] = static_cast<int8_t>(0x12), MessageResult::Success);
        CHECK_EQ(message["uint16_field"] = static_cast<uint16_t>(0x1234), MessageResult::Success);
        CHECK_EQ(message["int16_field"] = static_cast<int16_t>(0x1234), MessageResult::Success);
        CHECK_EQ(message["uint32_field"] = static_cast<uint32_t>(0x12345678), MessageResult::Success);
        CHECK_EQ(message["int32_field"] = static_cast<int32_t>(0x12345678), MessageResult::Success);
        CHECK_EQ(message["uint64_field"] = static_cast<uint64_t>(0x1234567890ABCDEF), MessageResult::Success);
        CHECK_EQ(message["int64_field"] = static_cast<int64_t>(0x1234567890ABCDEF), MessageResult::Success);
        CHECK_EQ(message["double_field"] = 0.123456789, MessageResult::Success);
        CHECK_EQ(message["float_field"] = 0.123456789f, MessageResult::Success);
        CHECK_EQ(message.setString("char_arr_field", "Hello World!"), MessageResult::Success);
        CHECK_EQ(message["float_arr_field"] = std::vector<float>{1.0, 2.0, 3.0}, MessageResult::Success);
        CHECK_EQ(message["int32_arr_field"] = std::vector<int32_t>{1, 2, 3}, MessageResult::Success);

        uint8_t uint8_val;
        CHECK_EQ(message["uint8_field"].get(uint8_val), MessageResult::Success);
        CHECK_EQ(uint8_val, 0x12);
        
        int8_t int8_val;
        CHECK_EQ(message["int8_field"].get(int8_val), MessageResult::Success);
        CHECK_EQ(int8_val, 0x12);
        
        uint16_t uint16_val;
        CHECK_EQ(message["uint16_field"].get(uint16_val), MessageResult::Success);
        CHECK_EQ(uint16_val, 0x1234);
        
        int16_t int16_val;
        CHECK_EQ(message["int16_field"].get(int16_val), MessageResult::Success);
        CHECK_EQ(int16_val, 0x1234);
        
        uint32_t uint32_val;
        CHECK_EQ(message["uint32_field"].get(uint32_val), MessageResult::Success);
        CHECK_EQ(uint32_val, 0x12345678);
        
        int32_t int32_val;
        CHECK_EQ(message["int32_field"].get(int32_val), MessageResult::Success);
        CHECK_EQ(int32_val, 0x12345678);
        
        uint64_t uint64_val;
        CHECK_EQ(message["uint64_field"].get(uint64_val), MessageResult::Success);
        CHECK_EQ(uint64_val, 0x1234567890ABCDEF);
        
        int64_t int64_val;
        CHECK_EQ(message["int64_field"].get(int64_val), MessageResult::Success);
        CHECK_EQ(int64_val, 0x1234567890ABCDEF);
        
        double double_val;
        CHECK_EQ(message["double_field"].get(double_val), MessageResult::Success);
        CHECK_EQ(double_val, doctest::Approx(0.123456789));
        
        float float_val;
        CHECK_EQ(message["float_field"].get(float_val), MessageResult::Success);
        CHECK_EQ(float_val, doctest::Approx(0.123456789f));
        
        std::string string_val;
        CHECK_EQ(message.getString("char_arr_field", string_val), MessageResult::Success);
        CHECK_EQ(string_val, "Hello World!");
        
        std::vector<float> float_arr_val;
        CHECK_EQ(message.get("float_arr_field", float_arr_val), MessageResult::Success);
        CHECK_EQ(float_arr_val, std::vector<float>{1.0, 2.0, 3.0});
        
        std::vector<int32_t> int32_arr_val;
        CHECK_EQ(message.get("int32_arr_field", int32_arr_val), MessageResult::Success);
        CHECK_EQ(int32_arr_val, std::vector<int32_t>{1, 2, 3});
    }

    SUBCASE("Can set values with initializer list API") {
        auto init_result = message.set({
                            {"uint8_field",     static_cast<uint8_t>(0x12)},
                            {"int8_field",      static_cast<int8_t>(0x12)},
                            {"uint16_field",    static_cast<uint16_t>(0x1234)},
                            {"int16_field",     static_cast<int16_t>(0x1234)},
                            {"uint32_field",    static_cast<uint32_t>(0x12345678)},
                            {"int32_field",     static_cast<int32_t>(0x12345678)},
                            {"uint64_field",    static_cast<uint64_t>(0x1234567890ABCDEF)},
                            {"int64_field",     static_cast<int64_t>(0x1234567890ABCDEF)},
                            {"double_field",    0.123456789},
                            {"float_field",     0.123456789f},
                            {"char_arr_field",  std::string("Hello World!")},
                            {"float_arr_field", std::vector<float>{1.0, 2.0, 3.0}},
                            {"int32_arr_field", std::vector<int32_t>{1, 2, 3}}});
        CHECK_EQ(init_result, MessageResult::Success);

        uint8_t uint8_val;
        CHECK_EQ(message["uint8_field"].get(uint8_val), MessageResult::Success);
        CHECK_EQ(uint8_val, 0x12);
        
        int8_t int8_val;
        CHECK_EQ(message["int8_field"].get(int8_val), MessageResult::Success);
        CHECK_EQ(int8_val, 0x12);
        
        // Test a few more fields to verify the initializer list worked
        uint32_t uint32_val;
        CHECK_EQ(message.get("uint32_field", uint32_val), MessageResult::Success);
        CHECK_EQ(uint32_val, 0x12345678);
        
        std::string string_val;
        CHECK_EQ(message.getString("char_arr_field", string_val), MessageResult::Success);
        CHECK_EQ(string_val, "Hello World!");
    }

    SUBCASE("Can assign std::string to char array field") {
        std::string str = "Hello World!";
        CHECK_EQ(message.setString("char_arr_field", str), MessageResult::Success);
        
        std::string retrieved_str;
        CHECK_EQ(message.getString("char_arr_field", retrieved_str), MessageResult::Success);
        CHECK_EQ(retrieved_str, "Hello World!");
    }

    SUBCASE("Assign independent chars to char array field") {
        CHECK_EQ(message.setString("char_arr_field", "012345"), MessageResult::Success);
        CHECK_EQ(message.set("char_arr_field", 'a', 0), MessageResult::Success);
        
        char char_val;
        CHECK_EQ(message.get("char_arr_field", char_val, 0), MessageResult::Success);
        CHECK_EQ(char_val, 'a');
        
        std::string string_val;
        CHECK_EQ(message.getString("char_arr_field", string_val), MessageResult::Success);
        CHECK_EQ(string_val, "a12345");

        CHECK_EQ(message.set("char_arr_field", 'b', 1), MessageResult::Success);
        CHECK_EQ(message.getString("char_arr_field", string_val), MessageResult::Success);
        CHECK_EQ(string_val, "ab2345");

        CHECK_EQ(message["char_arr_field"][2] = 'c', MessageResult::Success);
        CHECK_EQ(message.getString("char_arr_field", string_val), MessageResult::Success);
        CHECK_EQ(string_val, "abc345");

        CHECK_EQ(message["char_arr_field"][3] = 'd', MessageResult::Success);
        char item3;
        CHECK_EQ(message["char_arr_field"][3].get(item3), MessageResult::Success);
        CHECK_EQ(item3, 'd');

        CHECK_EQ(message["char_arr_field"][4] = 'e', MessageResult::Success);
        CHECK_EQ(message.getString("char_arr_field", string_val), MessageResult::Success);
        CHECK_EQ(string_val, "abcde5");
    }

    SUBCASE("Assigning too long string to char array field returns error") {
        std::string str = "This is a very long string that will not fit in the char array field";
        auto result = message.setString("char_arr_field", str + str);
        CHECK_EQ(result, MessageResult::OutOfRange);
    }

    SUBCASE("Assigning a too long vector to an array field returns error") {
        std::vector<float> vec(100);
        auto result = message.set("float_arr_field", vec);
        CHECK_EQ(result, MessageResult::OutOfRange);
    }

    SUBCASE("Pack integer into float field") {
        CHECK_EQ(message.setAsFloatPack("float_field", static_cast<uint32_t>(0x12345678)), MessageResult::Success);
        
        uint32_t unpacked_val;
        CHECK_EQ(message.getAsFloatUnpack("float_field", unpacked_val), MessageResult::Success);
        CHECK_EQ(unpacked_val, 0x12345678);

        CHECK_EQ(message["float_field"].floatPack<uint32_t>(0x23456789), MessageResult::Success);
        CHECK_EQ(message["float_field"].floatUnpack<uint32_t>(unpacked_val), MessageResult::Success);
        CHECK_EQ(unpacked_val, 0x23456789);

        // Test type mismatches
        std::string dummy_string;
        CHECK_EQ(message["float_field"].floatUnpack<std::string>(dummy_string), MessageResult::TypeMismatch);
        
        std::vector<int> dummy_vec;
        CHECK_EQ(message["float_field"].floatUnpack<std::vector<int>>(dummy_vec), MessageResult::TypeMismatch);
    }

    SUBCASE("Set and get a single field in array outside of range") {
        auto set_result = message.set("float_arr_field", 1.0f, 100);
        CHECK_EQ(set_result, MessageResult::OutOfRange);
        
        float dummy_val;
        auto get_result = message.get("float_arr_field", dummy_val, 100);
        CHECK_EQ(get_result, MessageResult::OutOfRange);
    }

    SUBCASE("Set and get a string to a non-char field") {
        auto set_result = message.setString("float_field", "Hello World!");
        CHECK_EQ(set_result, MessageResult::TypeMismatch);
        
        std::string dummy_string;
        auto get_result = message.getString("float_field", dummy_string);
        CHECK_EQ(get_result, MessageResult::TypeMismatch);
    }

    SUBCASE("String at the end of message") {
        CHECK_EQ(message.set("int8_field", static_cast<int8_t>(0x00)), MessageResult::Success);
        CHECK_EQ(message.set("uint16_field", static_cast<uint16_t>(0x0)), MessageResult::Success);
        CHECK_EQ(message.set("int16_field", static_cast<int16_t>(0x0)), MessageResult::Success);
        CHECK_EQ(message.set("uint32_field", static_cast<uint32_t>(0x0)), MessageResult::Success);
        CHECK_EQ(message.set("int32_field", static_cast<int32_t>(0x0)), MessageResult::Success);
        CHECK_EQ(message.set("uint64_field", static_cast<uint64_t>(0x0)), MessageResult::Success);
        CHECK_EQ(message.set("int64_field", static_cast<int64_t>(0x0)), MessageResult::Success);
        CHECK_EQ(message.set("double_field", 0.0), MessageResult::Success);
        CHECK_EQ(message.set("float_field", 0.0f), MessageResult::Success);
        CHECK_EQ(message.setString("char_arr_field", "Hello World!"), MessageResult::Success);
        CHECK_EQ(message.set("float_arr_field", std::vector<float>{0.0, 0.0, 0.0}), MessageResult::Success);
        CHECK_EQ(message.set("int32_arr_field", std::vector<int32_t>{0, 0, 0}), MessageResult::Success);

        auto finalize_result = message.finalize(5, {6, 7});
        REQUIRE(finalize_result.has_value());
        
        std::string string_val;
        CHECK_EQ(message.getString("char_arr_field", string_val), MessageResult::Success);
        CHECK_EQ(string_val, "Hello World!");

        // check that it stays correct even after modifying the fields
        CHECK_EQ(message.set("uint32_field", static_cast<uint32_t>(0x1)), MessageResult::Success);
        CHECK_EQ(message.getString("char_arr_field", string_val), MessageResult::Success);
        CHECK_EQ(string_val, "Hello World!");

        CHECK_EQ(message.setString("char_arr_field", "Hello Worldo!"), MessageResult::Success);
        CHECK_EQ(message.getString("char_arr_field", string_val), MessageResult::Success);
        CHECK_EQ(string_val, "Hello Worldo!");
    }

    SUBCASE("Zero elision works on single byte at end of array") {
        auto this_test_message_opt = message_set.create("UINT8_ONLY_MESSAGE");
        REQUIRE(this_test_message_opt.has_value());
        auto this_test_message = this_test_message_opt.value();
        
        CHECK_EQ(this_test_message.set("field1", static_cast<uint8_t>(111)), MessageResult::Success);
        CHECK_EQ(this_test_message.set("field2", static_cast<uint8_t>(0)), MessageResult::Success);
        CHECK_EQ(this_test_message.set("field3", static_cast<uint8_t>(0)), MessageResult::Success);
        CHECK_EQ(this_test_message.set("field4", static_cast<uint8_t>(0)), MessageResult::Success);

        auto wire_size = this_test_message.finalize(5, {6, 7});
        REQUIRE(wire_size.has_value());
        CHECK_EQ(wire_size.value(), 13);
        
        uint8_t val;
        CHECK_EQ(this_test_message.get("field1", val), MessageResult::Success);
        CHECK_EQ(val, 111);
        CHECK_EQ(this_test_message.get("field2", val), MessageResult::Success);
        CHECK_EQ(val, 0);
        CHECK_EQ(this_test_message.get("field3", val), MessageResult::Success);
        CHECK_EQ(val, 0);
        CHECK_EQ(this_test_message.get("field4", val), MessageResult::Success);
        CHECK_EQ(val, 0);
    }

    SUBCASE("Test all array conversions") {
        auto this_test_message_opt = message_set.create("ARRAY_ONLY_MESSAGE");
        REQUIRE(this_test_message_opt.has_value());
        auto this_test_message = this_test_message_opt.value();
        
        CHECK_EQ(this_test_message.setString("field1", "Hello"), MessageResult::Success);
        CHECK_EQ(this_test_message["field2"] = std::vector<uint8_t>{1, 2, 3}, MessageResult::Success);
        CHECK_EQ(this_test_message["field3"] = std::vector<uint16_t>{4, 5, 6}, MessageResult::Success);
        CHECK_EQ(this_test_message["field4"] = std::vector<uint32_t>{7, 8, 9}, MessageResult::Success);
        CHECK_EQ(this_test_message["field5"] = std::vector<uint64_t>{10, 11, 12}, MessageResult::Success);
        CHECK_EQ(this_test_message["field6"] = std::vector<int8_t>{13, 14, 15}, MessageResult::Success);
        CHECK_EQ(this_test_message["field7"] = std::vector<int16_t>{16, 17, 18}, MessageResult::Success);
        CHECK_EQ(this_test_message["field8"] = std::vector<int32_t>{19, 20, 21}, MessageResult::Success);
        CHECK_EQ(this_test_message["field9"] = std::vector<int64_t>{22, 23, 24}, MessageResult::Success);
        CHECK_EQ(this_test_message["field10"] = std::vector<float>{25, 26, 27}, MessageResult::Success);
        CHECK_EQ(this_test_message["field11"] = std::vector<double>{28, 29, 30}, MessageResult::Success);

        std::string string_val;
        CHECK_EQ(this_test_message.getString("field1", string_val), MessageResult::Success);
        CHECK_EQ(string_val, "Hello");
        
        std::vector<uint8_t> uint8_vec;
        CHECK_EQ(this_test_message.get("field2", uint8_vec), MessageResult::Success);
        CHECK_EQ(uint8_vec, std::vector<uint8_t>{1, 2, 3});
        
        std::vector<uint16_t> uint16_vec;
        CHECK_EQ(this_test_message.get("field3", uint16_vec), MessageResult::Success);
        CHECK_EQ(uint16_vec, std::vector<uint16_t>{4, 5, 6});
        
        // Test a few more to verify they work
        std::vector<uint64_t> uint64_vec;
        CHECK_EQ(this_test_message.get("field5", uint64_vec), MessageResult::Success);
        CHECK_EQ(uint64_vec, std::vector<uint64_t>{10, 11, 12});
        
        std::vector<double> double_vec;
        CHECK_EQ(this_test_message.get("field11", double_vec), MessageResult::Success);
        CHECK_EQ(double_vec, std::vector<double>{28, 29, 30});
    }

    SUBCASE("Can get as native type variant") {
        CHECK_EQ(message.setFromNativeTypeVariant("uint8_field", {static_cast<uint8_t>(1)}), MessageResult::Success);
        CHECK_EQ(message.setFromNativeTypeVariant("int8_field", {static_cast<int8_t>(2)}), MessageResult::Success);
        CHECK_EQ(message.setFromNativeTypeVariant("uint16_field", {static_cast<uint16_t>(3)}), MessageResult::Success);
        CHECK_EQ(message.setFromNativeTypeVariant("int16_field", {static_cast<int16_t>(4)}), MessageResult::Success);
        CHECK_EQ(message.setFromNativeTypeVariant("uint32_field", {static_cast<uint32_t>(5)}), MessageResult::Success);
        CHECK_EQ(message.setFromNativeTypeVariant("int32_field", {static_cast<int32_t>(6)}), MessageResult::Success);
        CHECK_EQ(message.setFromNativeTypeVariant("uint64_field", {static_cast<uint64_t>(7)}), MessageResult::Success);
        CHECK_EQ(message.setFromNativeTypeVariant("int64_field", {static_cast<int64_t>(8)}), MessageResult::Success);
        CHECK_EQ(message.setFromNativeTypeVariant("double_field", {9.0}), MessageResult::Success);
        CHECK_EQ(message.setFromNativeTypeVariant("float_field", {10.0f}), MessageResult::Success);
        CHECK_EQ(message.setFromNativeTypeVariant("char_arr_field", {std::string("Hello World!")}), MessageResult::Success);
        CHECK_EQ(message.setFromNativeTypeVariant("float_arr_field", {std::vector<float>{1.0, 2.0, 3.0}}), MessageResult::Success);
        CHECK_EQ(message.setFromNativeTypeVariant("int32_arr_field", {std::vector<int32_t>{4, 5, 6}}), MessageResult::Success);
        
        uint8_t uint8_val;
        CHECK_EQ(message.get("uint8_field", uint8_val), MessageResult::Success);
        CHECK_EQ(uint8_val, 1);
        
        int8_t int8_val;
        CHECK_EQ(message.get("int8_field", int8_val), MessageResult::Success);
        CHECK_EQ(int8_val, 2);
        
        // Test variant extraction
        auto variant_opt = message.getAsNativeTypeInVariant("uint8_field");
        REQUIRE(variant_opt.has_value());
        CHECK(std::holds_alternative<uint8_t>(variant_opt.value()));
        
        variant_opt = message.getAsNativeTypeInVariant("int8_field");
        REQUIRE(variant_opt.has_value());
        CHECK(std::holds_alternative<int8_t>(variant_opt.value()));
        
        variant_opt = message.getAsNativeTypeInVariant("char_arr_field");
        REQUIRE(variant_opt.has_value());
        CHECK(std::holds_alternative<std::string>(variant_opt.value()));
        
        variant_opt = message.getAsNativeTypeInVariant("float_arr_field");
        REQUIRE(variant_opt.has_value());
        CHECK(std::holds_alternative<std::vector<float>>(variant_opt.value()));

        auto this_test_message_opt = message_set.create("ARRAY_ONLY_MESSAGE");
        REQUIRE(this_test_message_opt.has_value());
        auto this_test_message = this_test_message_opt.value();
        
        variant_opt = this_test_message.getAsNativeTypeInVariant("field1");
        REQUIRE(variant_opt.has_value());
        CHECK(std::holds_alternative<std::string>(variant_opt.value()));
        
        variant_opt = this_test_message.getAsNativeTypeInVariant("field2");
        REQUIRE(variant_opt.has_value());
        CHECK(std::holds_alternative<std::vector<uint8_t>>(variant_opt.value()));
    }

    SUBCASE("Test toString") {
        CHECK_EQ(message["uint8_field"] = static_cast<uint8_t>(1), MessageResult::Success);
        CHECK_EQ(message["int8_field"] = static_cast<int8_t>(-2), MessageResult::Success);
        CHECK_EQ(message["uint16_field"] = static_cast<uint16_t>(3), MessageResult::Success);
        CHECK_EQ(message["int16_field"] = static_cast<int16_t>(-4), MessageResult::Success);
        CHECK_EQ(message["uint32_field"] = static_cast<uint32_t>(5), MessageResult::Success);
        CHECK_EQ(message["int32_field"] = static_cast<int32_t>(-6), MessageResult::Success);
        CHECK_EQ(message["uint64_field"] = static_cast<uint64_t>(7), MessageResult::Success);
        CHECK_EQ(message["int64_field"] = static_cast<int64_t>(8), MessageResult::Success);
        CHECK_EQ(message["double_field"] = 9.0, MessageResult::Success);
        CHECK_EQ(message["float_field"] = 10.0f, MessageResult::Success);
        CHECK_EQ(message.setString("char_arr_field", "Hello World!"), MessageResult::Success);
        CHECK_EQ(message["float_arr_field"] = std::vector<float>{1.0, 2.0, 3.0}, MessageResult::Success);
        CHECK_EQ(message["int32_arr_field"] = std::vector<int32_t>{4, 5, 6}, MessageResult::Success);
        
        std::string str = message.toString();
        CHECK(str.find("Message ID 9915 (BIG_MESSAGE)") != std::string::npos);
        CHECK(str.find("Hello World!") != std::string::npos);
        CHECK(str.find("uint8_field: 1") != std::string::npos);
    }

    SUBCASE("Sign a packet") {
        std::array<uint8_t, 32> key;
        for (int i = 0 ; i < 32; i++) key[i] = i;

        uint64_t timestamp = 770479200;

        // Attempt to access signature before signed
        auto const_message_opt = message_set.create("UINT8_ONLY_MESSAGE");
        REQUIRE(const_message_opt.has_value());
        auto const_message = const_message_opt.value();
        
        auto sig_opt = message.signature();
        CHECK_FALSE(sig_opt.has_value());
        
        auto const_sig_opt = const_message.signature();
        CHECK_FALSE(const_sig_opt.has_value());

        auto wire_size = message.finalize(5, {6, 7}, key, timestamp);
        REQUIRE(wire_size.has_value());

        CHECK_EQ(wire_size.value(), 26);
        CHECK(message.header().isSigned());
        
        auto final_sig_opt = message.signature();
        REQUIRE(final_sig_opt.has_value());
        auto signature = final_sig_opt.value();
        CHECK_NE(signature.signature(), 0);
        CHECK_EQ(signature.timestamp(), timestamp);
        
        auto validate_result = message.validate(key);
        REQUIRE(validate_result.has_value());
        CHECK(validate_result.value());
    }
}