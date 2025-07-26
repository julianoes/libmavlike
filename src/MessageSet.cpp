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

#include "mav/MessageSet.h"
#include <cmath>
#include <climits>

// tinyxml2 provides robust error handling without requiring error handlers

namespace mav {

    // FileLoader implementation
    FileLoader::FileLoader(const char *filename) : m_valid(false) {
        std::ifstream stream(filename, std::ios::binary);
        if (!stream) {
            return;
        }
        stream.unsetf(std::ios::skipws);

        stream.seekg(0, std::ios::end);
        auto size = static_cast<std::size_t>(stream.tellg());
        stream.seekg(0);

        m_data.resize(size + 1);
        stream.read(&m_data.front(), static_cast<std::streamsize>(size));
        m_data[size] = 0;
        m_valid = true;
    }

    FileLoader::FileLoader(std::istream &stream) : m_valid(false) {
        if (stream.fail() || stream.bad()) {
            return;
        }
        stream.unsetf(std::ios::skipws);
        m_data.assign(std::istreambuf_iterator<char>(stream), std::istreambuf_iterator<char>());
        if (stream.fail() || stream.bad()) {
            return;
        }
        m_data.push_back(0);
        m_valid = true;
    }

    char *FileLoader::data() {
        return &m_data.front();
    }

    const char *FileLoader::data() const {
        return &m_data.front();
    }

    std::size_t FileLoader::size() const {
        return m_data.empty() ? 0 : m_data.size() - 1;
    }

    bool FileLoader::isValid() const {
        return m_valid;
    }

    // XMLParser implementation
    XMLParser::XMLParser(std::shared_ptr<FileLoader> source_file,
                        std::unique_ptr<tinyxml2::XMLDocument> document,
                        const std::string &root_xml_folder_path,
                        bool recursive_open_files) :
            _source_file(std::move(source_file)),
            _document(std::move(document)),
            _root_xml_folder_path(root_xml_folder_path),
            _recursive_open_files(recursive_open_files) {}
    std::optional<uint64_t> XMLParser::_strict_stoul(const std::string &str, int base) {
        if (str.empty()) {
            return std::nullopt;
        }

        uint64_t result = 0;
        for (char c : str) {
            int digit_value = -1;
            
            if (c >= '0' && c <= '9') {
                digit_value = c - '0';
            } else if (base == 16 && c >= 'a' && c <= 'f') {
                digit_value = c - 'a' + 10;
            } else if (base == 16 && c >= 'A' && c <= 'F') {
                digit_value = c - 'A' + 10;
            }
            
            if (digit_value < 0 || digit_value >= base) {
                return std::nullopt;
            }
            
            // Check for overflow
            if (result > (UINT64_MAX - static_cast<uint64_t>(digit_value)) / static_cast<uint64_t>(base)) {
                return std::nullopt;
            }
            
            result = result * static_cast<uint64_t>(base) + static_cast<uint64_t>(digit_value);
        }
        
        return result;
    }

    std::optional<uint64_t> XMLParser::_parseEnumValue(const std::string &str) {
        // Check for binary format: 0b or 0B
        if (str.size() >= 2 && (str.substr(0, 2) == "0b" || str.substr(0, 2) == "0B")) {
            return _strict_stoul(str.substr(2), 2);
        }

        // Check for hexadecimal format: 0x or 0X
        if (str.size() >= 2 && (str.substr(0, 2) == "0x" || str.substr(0, 2) == "0X")) {
            return _strict_stoul(str.substr(2), 16);
        }

        // Check for exponential format: 2**
        size_t expPos = str.find("**");
        if (expPos != std::string::npos) {
            auto base_opt = _strict_stoul(str.substr(0, expPos));
            if (!base_opt || base_opt.value() != 2) {
                return std::nullopt;
            }
            auto exponent_opt = _strict_stoul(str.substr(expPos + 2));
            if (!exponent_opt || exponent_opt.value() > 63) {
                return std::nullopt;
            }
            return static_cast<uint64_t>(std::pow(base_opt.value(), exponent_opt.value()));
        }

        // If none of the above, assume decimal format
        return _strict_stoul(str);
    }

    bool XMLParser::_isPrefix(std::string_view prefix, std::string_view full) noexcept {
        return prefix == full.substr(0, prefix.size());
    }

    std::optional<FieldType> XMLParser::_parseFieldType(const std::string &field_type_string) {
        int size = 1;
        size_t array_notation_start_idx = field_type_string.find('[');
        std::string base_type_substr;
        if (array_notation_start_idx != std::string::npos) {
            auto size_substr = field_type_string.substr(
                    array_notation_start_idx + 1, field_type_string.length() - array_notation_start_idx - 2);
            auto size_opt = _strict_stoul(size_substr);
            if (!size_opt || size_opt.value() > INT_MAX) {
                return std::nullopt;
            }
            size = static_cast<int>(size_opt.value());
        }

        if (_isPrefix("uint8_t", field_type_string)) {
            return FieldType{FieldType::BaseType::UINT8, size};
        } else if (_isPrefix("uint16_t", field_type_string)) {
            return FieldType{FieldType::BaseType::UINT16, size};
        } else if (_isPrefix("uint32_t", field_type_string)) {
            return FieldType{FieldType::BaseType::UINT32, size};
        } else if (_isPrefix("uint64_t", field_type_string)) {
            return FieldType{FieldType::BaseType::UINT64, size};
        } else if (_isPrefix("int8_t", field_type_string)) {
            return FieldType{FieldType::BaseType::INT8, size};
        } else if (_isPrefix("int16_t", field_type_string)) {
            return FieldType{FieldType::BaseType::INT16, size};
        } else if (_isPrefix("int32_t", field_type_string)) {
            return FieldType{FieldType::BaseType::INT32, size};
        } else if (_isPrefix("int64_t", field_type_string)) {
            return FieldType{FieldType::BaseType::INT64, size};
        } else if (_isPrefix("char", field_type_string)) {
            return FieldType{FieldType::BaseType::CHAR, size};
        } else if (_isPrefix("float", field_type_string)) {
            return FieldType{FieldType::BaseType::FLOAT, size};
        } else if (_isPrefix("double", field_type_string)) {
            return FieldType{FieldType::BaseType::DOUBLE, size};
        }
        return std::nullopt;
    }

    std::optional<XMLParser> XMLParser::forFile(const std::string &file_name, bool recursive_open_includes) {
        auto file = std::make_shared<FileLoader>(file_name.c_str());
        if (!file->isValid()) {
            return std::nullopt;
        }
        
        auto doc = std::make_unique<tinyxml2::XMLDocument>();
        
        // Parse file content using tinyxml2's robust error handling
        tinyxml2::XMLError error = doc->Parse(file->data(), file->size());
        if (error != tinyxml2::XML_SUCCESS) {
            // tinyxml2 handled the malformed XML gracefully - return error
            return std::nullopt;
        }

        return XMLParser{file, std::move(doc), std::filesystem::path{file_name}.parent_path().string(), recursive_open_includes};
    }

    std::optional<XMLParser> XMLParser::forXMLString(const std::string &xml_string, bool recursive_open_includes) {
        // Create a FileLoader from the XML string
        auto istream = std::istringstream(xml_string);
        auto file = std::make_shared<FileLoader>(istream);
        if (!file->isValid()) {
            return std::nullopt;
        }
        
        auto doc = std::make_unique<tinyxml2::XMLDocument>();
        
        // Parse XML string using tinyxml2's robust error handling
        tinyxml2::XMLError error = doc->Parse(file->data(), file->size());
        if (error != tinyxml2::XML_SUCCESS) {
            // tinyxml2 handled the malformed XML gracefully - return error
            return std::nullopt;
        }
        
        return XMLParser{file, std::move(doc), "", recursive_open_includes};
    }

    ParseResult XMLParser::parse(std::map<std::string, uint64_t> &out_enum,
               std::map<std::string, std::shared_ptr<const MessageDefinition>> &out_messages,
               std::map<int, std::shared_ptr<const MessageDefinition>> &out_message_ids) const {

        auto root_node = _document->FirstChildElement("mavlink");
        if (!root_node) {
            return ParseResult::InvalidXml;
        }
        if (_recursive_open_files) {
            for (auto include_element = root_node->FirstChildElement("include");
                 include_element != nullptr;
                 include_element = include_element->NextSiblingElement("include")) {

                const char* include_text = include_element->GetText();
                if (!include_text) {
                    return ParseResult::InvalidXml;
                }
                const std::string include_name = include_text;
                auto sub_parser_opt = XMLParser::forFile(
                        (std::filesystem::path{_root_xml_folder_path} / include_name).string(), true);
                if (!sub_parser_opt) {
                    return ParseResult::FileNotFound;
                }
                auto result = sub_parser_opt.value().parse(out_enum, out_messages, out_message_ids);
                if (result != ParseResult::Success) {
                    return result;
                }
            }
        }

        auto enums_node = root_node->FirstChildElement("enums");
        if (enums_node) {
            for (auto enum_node = enums_node->FirstChildElement();
                enum_node != nullptr;
                enum_node = enum_node->NextSiblingElement()) {

                for (auto entry = enum_node->FirstChildElement();
                    entry != nullptr;
                    entry = entry->NextSiblingElement()) {
                    if (std::string_view("entry") == entry->Name()) {
                        const char* entry_name = entry->Attribute("name");
                        const char* value_str = entry->Attribute("value");
                        if (!entry_name || !value_str) {
                            return ParseResult::InvalidXml;
                        }
                        auto enum_value_opt = _parseEnumValue(value_str);
                        if (!enum_value_opt) {
                            return ParseResult::EnumParseError;
                        }
                        out_enum[entry_name] = enum_value_opt.value();
                    }
                }
            }
        }

        auto messages_node = root_node->FirstChildElement("messages");
        if (messages_node) {
            for (auto message = messages_node->FirstChildElement();
                 message != nullptr;
                 message = message->NextSiblingElement()) {

                const char* message_name_attr = message->Attribute("name");
                const char* message_id_attr = message->Attribute("id");
                if (!message_name_attr || !message_id_attr) {
                    return ParseResult::InvalidXml;
                }
                
                const std::string message_name = message_name_attr;
                auto message_id_opt = _strict_stoul(message_id_attr);
                if (!message_id_opt || message_id_opt.value() > INT_MAX) {
                    return ParseResult::InvalidXml;
                }
                int message_id = static_cast<int>(message_id_opt.value());
                
                MessageDefinitionBuilder builder{message_name, message_id};

                std::string description;

                bool in_extension_fields = false;
                for (auto field = message->FirstChildElement();
                     field != nullptr;
                     field = field->NextSiblingElement()) {

                    if (std::string_view{"description"} == field->Name()) {
                        const char* desc_text = field->GetText();
                        if (desc_text) {
                            description = desc_text;
                        }
                    } else if (std::string_view{"extensions"} == field->Name()) {
                        in_extension_fields = true;
                    } else if (std::string_view{"field"} == field->Name()) {
                        // parse the field
                        const char* field_type_attr = field->Attribute("type");
                        const char* field_name_attr = field->Attribute("name");
                        if (!field_type_attr || !field_name_attr) {
                            return ParseResult::InvalidXml;
                        }
                        
                        auto field_type_opt = _parseFieldType(field_type_attr);
                        if (!field_type_opt) {
                            return ParseResult::FieldTypeError;
                        }

                        if (!in_extension_fields) {
                            builder.addField(field_name_attr, field_type_opt.value());
                        } else {
                            builder.addExtensionField(field_name_attr, field_type_opt.value());
                        }
                    }
                }
                auto definition = std::make_shared<const MessageDefinition>(builder.build());
                out_messages.emplace(message_name, definition);
                out_message_ids.emplace(definition->id(), definition);
            }
        }
        return ParseResult::Success;
    }

    // MessageSet implementation
    MessageSetResult MessageSet::addFromXML(const std::string &file_path, bool recursive_open_includes) {
        auto parser_opt = XMLParser::forFile(file_path, recursive_open_includes);
        if (!parser_opt) {
            return MessageSetResult::FileError;
        }
        auto result = parser_opt.value().parse(_enums, _messages, _message_ids);
        if (result != ParseResult::Success) {
            return MessageSetResult::XmlParseError;
        }
        return MessageSetResult::Success;
    }

    MessageSetResult MessageSet::addFromXMLString(const std::string &xml_string, bool recursive_open_includes) {
        auto parser_opt = XMLParser::forXMLString(xml_string, recursive_open_includes);
        if (!parser_opt) {
            return MessageSetResult::XmlParseError;
        }
        auto result = parser_opt.value().parse(_enums, _messages, _message_ids);
        if (result != ParseResult::Success) {
            return MessageSetResult::XmlParseError;
        }
        return MessageSetResult::Success;
    }

    OptionalReference<const MessageDefinition> MessageSet::getMessageDefinition(const std::string &message_name) const {
        auto message_definition = _messages.find(message_name);
        if (message_definition == _messages.end()) {
            return std::nullopt;
        }
        return *(message_definition->second);
    }

    OptionalReference<const MessageDefinition> MessageSet::getMessageDefinition(int message_id) const {
        auto message_definition = _message_ids.find(message_id);
        if (message_definition == _message_ids.end()) {
            return std::nullopt;
        }
        return *(message_definition->second);
    }

    std::optional<Message> MessageSet::create(const std::string &message_name) const {
        auto message_definition = getMessageDefinition(message_name);
        if (!message_definition) {
            return std::nullopt;
        }
        return Message{message_definition.get()};
    }

    std::optional<Message> MessageSet::create(int message_id) const {
        auto message_definition = getMessageDefinition(message_id);
        if (!message_definition) {
            return std::nullopt;
        }
        return Message{message_definition.get()};
    }

    std::optional<uint64_t> MessageSet::getEnum(const std::string &key) const {
        auto res = _enums.find(key);
        if (res == _enums.end()) {
            return std::nullopt;
        }
        return res->second;
    }

    std::optional<int> MessageSet::idForMessage(const std::string &message_name) const {
        auto message_definition = _messages.find(message_name);
        if (message_definition == _messages.end()) {
            return std::nullopt;
        }
        return message_definition->second->id();
    }

    bool MessageSet::contains(const std::string &message_name) const {
        return _messages.find(message_name) != _messages.end();
    }

    bool MessageSet::contains(int message_id) const {
        return _message_ids.find(message_id) != _message_ids.end();
    }

    size_t MessageSet::size() const {
        return _messages.size();
    }

} // namespace mav