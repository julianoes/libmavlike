/**
 * @file mavsdk_integration_tests.cpp
 * @brief Tests demonstrating MAVSDK integration patterns with non-throwing libmav API
 * 
 * This test file shows how to use the new non-throwing methods in libmav
 * for integration with systems that don't use exceptions (like MAVSDK).
 */

#include "doctest.h"
#include "mav/MessageSet.h"
#include "mav/Message.h"
#include <vector>
#include <cstring>

using namespace mav;

// Helper class demonstrating MAVSDK integration patterns
class MAVSDKMessageHandler {
private:
    MessageSet message_set_;
    
public:
    enum class Result {
        SUCCESS,
        XML_LOAD_FAILED,
        MESSAGE_NOT_FOUND,
        FIELD_NOT_FOUND,
        TYPE_MISMATCH,
        OUT_OF_RANGE,
        ENUM_NOT_FOUND,
        SERIALIZATION_FAILED
    };
    
    Result initializeFromXMLString(const std::string& xml_string) {
        try {
            message_set_.addFromXMLString(xml_string);
            return Result::SUCCESS;
        } catch (const std::exception&) {
            return Result::XML_LOAD_FAILED;
        }
    }
    
    Result createHeartbeatMessage(std::vector<uint8_t>& output) {
        auto message_opt = message_set_.tryCreate("HEARTBEAT");
        if (!message_opt) {
            return Result::MESSAGE_NOT_FOUND;
        }
        
        auto message = message_opt.value();
        
        // Set fields safely using the non-throwing API
        auto type_enum = message_set_.tryGetEnum("MAV_TYPE_ONBOARD_CONTROLLER");
        if (!type_enum) {
            return Result::ENUM_NOT_FOUND;
        }
        
        if (message.trySet("type", static_cast<uint8_t>(type_enum.value())) != Message::SetResult::SUCCESS) {
            return Result::FIELD_NOT_FOUND;
        }
        
        if (message.trySet("autopilot", static_cast<uint8_t>(0)) != Message::SetResult::SUCCESS) {
            return Result::FIELD_NOT_FOUND;
        }
        
        if (message.trySet("base_mode", static_cast<uint8_t>(0)) != Message::SetResult::SUCCESS) {
            return Result::FIELD_NOT_FOUND;
        }
        
        if (message.trySet("custom_mode", static_cast<uint32_t>(0)) != Message::SetResult::SUCCESS) {
            return Result::FIELD_NOT_FOUND;
        }
        
        if (message.trySet("system_status", static_cast<uint8_t>(4)) != Message::SetResult::SUCCESS) {
            return Result::FIELD_NOT_FOUND;
        }
        
        if (message.trySet("mavlink_version", static_cast<uint8_t>(2)) != Message::SetResult::SUCCESS) {
            return Result::FIELD_NOT_FOUND;
        }
        
        // Serialize the message safely
        Identifier sender{1, 1};
        auto length_opt = message.tryFinalize(0, sender);
        if (!length_opt) {
            return Result::SERIALIZATION_FAILED;
        }
        
        // Copy binary data to output
        output.resize(length_opt.value());
        std::memcpy(output.data(), message.data(), length_opt.value());
        
        return Result::SUCCESS;
    }
    
    Result createParamRequestMessage(const std::string& param_name, 
                                   uint8_t target_system, 
                                   uint8_t target_component, 
                                   std::vector<uint8_t>& output) {
        auto message_opt = message_set_.tryCreate("PARAM_REQUEST_READ");
        if (!message_opt) {
            return Result::MESSAGE_NOT_FOUND;
        }
        
        auto message = message_opt.value();
        
        // Set numeric fields
        if (message.trySet("target_system", target_system) != Message::SetResult::SUCCESS) {
            return Result::FIELD_NOT_FOUND;
        }
        
        if (message.trySet("target_component", target_component) != Message::SetResult::SUCCESS) {
            return Result::FIELD_NOT_FOUND;
        }
        
        if (message.trySet("param_index", static_cast<int16_t>(-1)) != Message::SetResult::SUCCESS) {
            return Result::FIELD_NOT_FOUND;
        }
        
        // Set string field safely
        if (message.trySetString("param_id", param_name) != Message::SetResult::SUCCESS) {
            return Result::FIELD_NOT_FOUND;
        }
        
        // Serialize safely
        Identifier sender{1, 1};
        auto length_opt = message.tryFinalize(1, sender);
        if (!length_opt) {
            return Result::SERIALIZATION_FAILED;
        }
        
        output.resize(length_opt.value());
        std::memcpy(output.data(), message.data(), length_opt.value());
        
        return Result::SUCCESS;
    }
};

TEST_CASE("MAVSDK Integration - Basic message creation") {
    const std::string xml_def = R""""(
<mavlink>
    <enums>
        <enum name="MAV_TYPE">
            <entry value="2" name="MAV_TYPE_ONBOARD_CONTROLLER">
                <description>Onboard companion controller</description>
            </entry>
        </enum>
    </enums>
    <messages>
        <message id="0" name="HEARTBEAT">
            <field type="uint8_t" name="type">Type of the system</field>
            <field type="uint8_t" name="autopilot">Autopilot type / class</field>
            <field type="uint8_t" name="base_mode">System mode bitmap</field>
            <field type="uint32_t" name="custom_mode">Custom mode</field>
            <field type="uint8_t" name="system_status">System status flag</field>
            <field type="uint8_t" name="mavlink_version">MAVLink version</field>
        </message>
        <message id="20" name="PARAM_REQUEST_READ">
            <field type="uint8_t" name="target_system">System ID</field>
            <field type="uint8_t" name="target_component">Component ID</field>
            <field type="char[16]" name="param_id">Parameter id</field>
            <field type="int16_t" name="param_index">Parameter index</field>
        </message>
    </messages>
</mavlink>
)"""";

    MAVSDKMessageHandler handler;
    
    SUBCASE("Initialize message handler") {
        auto result = handler.initializeFromXMLString(xml_def);
        CHECK(result == MAVSDKMessageHandler::Result::SUCCESS);
    }
    
    SUBCASE("Create HEARTBEAT message without exceptions") {
        REQUIRE(handler.initializeFromXMLString(xml_def) == MAVSDKMessageHandler::Result::SUCCESS);
        
        std::vector<uint8_t> heartbeat_data;
        auto result = handler.createHeartbeatMessage(heartbeat_data);
        
        CHECK(result == MAVSDKMessageHandler::Result::SUCCESS);
        CHECK_FALSE(heartbeat_data.empty());
        CHECK(heartbeat_data[0] == 0xFD); // MAVLink v2 magic byte
        CHECK(heartbeat_data.size() > 10); // At least header + some payload
    }
    
    SUBCASE("Create PARAM_REQUEST_READ message without exceptions") {
        REQUIRE(handler.initializeFromXMLString(xml_def) == MAVSDKMessageHandler::Result::SUCCESS);
        
        std::vector<uint8_t> param_data;
        auto result = handler.createParamRequestMessage("SYS_AUTOSTART", 1, 1, param_data);
        
        CHECK(result == MAVSDKMessageHandler::Result::SUCCESS);
        CHECK_FALSE(param_data.empty());
        CHECK(param_data[0] == 0xFD); // MAVLink v2 magic byte
    }
    
    SUBCASE("Handle missing message gracefully") {
        REQUIRE(handler.initializeFromXMLString(xml_def) == MAVSDKMessageHandler::Result::SUCCESS);
        
        MessageSet message_set;
        message_set.addFromXMLString(xml_def);
        
        auto message_opt = message_set.tryCreate("NONEXISTENT_MESSAGE");
        CHECK_FALSE(message_opt.has_value());
    }
    
    SUBCASE("Handle missing enum gracefully") {
        REQUIRE(handler.initializeFromXMLString(xml_def) == MAVSDKMessageHandler::Result::SUCCESS);
        
        MessageSet message_set;
        message_set.addFromXMLString(xml_def);
        
        auto enum_opt = message_set.tryGetEnum("NONEXISTENT_ENUM");
        CHECK_FALSE(enum_opt.has_value());
    }
}

TEST_CASE("MAVSDK Integration - Error handling patterns") {
    MessageSet message_set;
    message_set.addFromXMLString(R""""(
<mavlink>
    <messages>
        <message id="100" name="TEST_MESSAGE">
            <field type="uint8_t" name="uint8_field">Test field</field>
            <field type="char[8]" name="string_field">String field</field>
        </message>
    </messages>
</mavlink>
)"""");

    SUBCASE("Field setting error codes") {
        auto message_opt = message_set.tryCreate("TEST_MESSAGE");
        REQUIRE(message_opt.has_value());
        auto message = message_opt.value();
        
        // Test different error conditions
        CHECK(message.trySet("nonexistent_field", 42) == Message::SetResult::FIELD_NOT_FOUND);
        CHECK(message.trySet("uint8_field", 3.14f) == Message::SetResult::TYPE_MISMATCH);
        CHECK(message.trySetString("string_field", "way_too_long_string") == Message::SetResult::OUT_OF_RANGE);
        CHECK(message.trySetString("uint8_field", "test") == Message::SetResult::TYPE_MISMATCH);
    }
    
    SUBCASE("Field getting error codes") {
        auto message_opt = message_set.tryCreate("TEST_MESSAGE");
        REQUIRE(message_opt.has_value());
        auto message = message_opt.value();
        
        uint8_t dummy_val;
        std::string dummy_str;
        
        CHECK(message.tryGet("nonexistent_field", dummy_val) == Message::GetResult::FIELD_NOT_FOUND);
        float dummy_float = 3.14f;
        CHECK(message.tryGet("uint8_field", dummy_float) == Message::GetResult::TYPE_MISMATCH);
        CHECK(message.tryGetString("nonexistent_field", dummy_str) == Message::GetResult::FIELD_NOT_FOUND);
        CHECK(message.tryGetString("uint8_field", dummy_str) == Message::GetResult::TYPE_MISMATCH);
    }
}

TEST_CASE("MAVSDK Integration - Performance characteristics") {
    MessageSet message_set;
    message_set.addFromXMLString(R""""(
<mavlink>
    <messages>
        <message id="0" name="HEARTBEAT">
            <field type="uint8_t" name="type">Type of the system</field>
            <field type="uint8_t" name="autopilot">Autopilot type / class</field>
        </message>
    </messages>
</mavlink>
)"""");

    SUBCASE("Multiple message creation and serialization") {
        const int iterations = 100;
        
        for (int i = 0; i < iterations; ++i) {
            auto message_opt = message_set.tryCreate("HEARTBEAT");
            if (message_opt) {
                auto message = message_opt.value();
                auto set_result1 = message.trySet("type", static_cast<uint8_t>(1));
                auto set_result2 = message.trySet("autopilot", static_cast<uint8_t>(0));
                
                Identifier sender{1, 1};
                auto result = message.tryFinalize(i, sender);
                
                // All operations should succeed
                CHECK(set_result1 == Message::SetResult::SUCCESS);
                CHECK(set_result2 == Message::SetResult::SUCCESS);
                CHECK(result.has_value());
            }
        }
    }
    
    SUBCASE("Round-trip field operations") {
        auto message_opt = message_set.tryCreate("HEARTBEAT");
        REQUIRE(message_opt.has_value());
        auto message = message_opt.value();
        
        // Set values
        CHECK(message.trySet("type", static_cast<uint8_t>(42)) == Message::SetResult::SUCCESS);
        CHECK(message.trySet("autopilot", static_cast<uint8_t>(123)) == Message::SetResult::SUCCESS);
        
        // Get values back
        uint8_t type_val, autopilot_val;
        CHECK(message.tryGet("type", type_val) == Message::GetResult::SUCCESS);
        CHECK(message.tryGet("autopilot", autopilot_val) == Message::GetResult::SUCCESS);
        
        // Verify round-trip
        CHECK(type_val == 42);
        CHECK(autopilot_val == 123);
    }
}