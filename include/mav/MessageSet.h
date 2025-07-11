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
#ifndef MAV_MESSAGESET_H
#define MAV_MESSAGESET_H

#include <utility>
#include <memory>
#include <optional>
#include <cmath>
#include <fstream>
#include <sstream>
#include <climits>
#include <cstdlib>

#include "MessageDefinition.h"
#include "Message.h"

// Enable no-exceptions mode for rapidxml
#define RAPIDXML_NO_EXCEPTIONS
#include <rapidxml/rapidxml.hpp>
#include <rapidxml/rapidxml_utils.hpp>

#ifdef _LIBCPP_VERSION
#if _LIBCPP_VERSION < 11000
#define _NO_STD_FILESYSTEM
#endif
#endif
#ifndef _NO_STD_FILESYSTEM
#include <filesystem>
#endif

// Required error handler for RAPIDXML_NO_EXCEPTIONS mode
namespace rapidxml {
    void parse_error_handler(const char *what, void *where) {
        // In no-exceptions mode, we can't throw. We'll abort to indicate the error.
        // In a real application, you might want to log the error or handle it differently.
        (void)what;   // Suppress unused parameter warning
        (void)where;  // Suppress unused parameter warning
        std::abort();
    }
}

namespace mav {


    class FileLoader {
    private:
        std::vector<char> m_data;
        bool m_valid;

    public:
        explicit FileLoader(const char *filename) : m_valid(false) {
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

        explicit FileLoader(std::istream &stream) : m_valid(false) {
            stream.unsetf(std::ios::skipws);
            m_data.assign(std::istreambuf_iterator<char>(stream), std::istreambuf_iterator<char>());
            if (stream.fail() || stream.bad()) {
                return;
            }
            m_data.push_back(0);
            m_valid = true;
        }

        char *data() {
            return &m_data.front();
        }

        const char *data() const {
            return &m_data.front();
        }

        std::size_t size() const {
            return m_data.empty() ? 0 : m_data.size() - 1;
        }

        bool isValid() const {
            return m_valid;
        }
    };

    class XMLParser {
    private:

        std::shared_ptr<FileLoader> _source_file;
        std::shared_ptr<rapidxml::xml_document<>> _document;
        std::string _root_xml_folder_path;
        bool _recursive_open_files;

        XMLParser(
                std::shared_ptr<FileLoader> source_file,
                std::shared_ptr<rapidxml::xml_document<>> document,
                const std::string &root_xml_folder_path,
                bool recursive_open_files) :
                _source_file(std::move(source_file)),
                _document(std::move(document)),
                _root_xml_folder_path(root_xml_folder_path),
                _recursive_open_files(recursive_open_files) {}

        
        static std::optional<uint64_t> _strict_stoul(const std::string &str, int base=10) {
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
                if (result > (UINT64_MAX - digit_value) / base) {
                    return std::nullopt;
                }
                
                result = result * base + digit_value;
            }
            
            return result;
        }

        static std::optional<uint64_t> _parseEnumValue(const std::string &str) {
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


        static bool _isPrefix(std::string_view prefix, std::string_view full) noexcept {
            return prefix == full.substr(0, prefix.size());
        }

        static std::optional<FieldType> _parseFieldType(const std::string &field_type_string) {
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

    public:
#ifndef _NO_STD_FILESYSTEM
        static std::optional<XMLParser> forFile(const std::string &file_name, bool recursive_open_includes) {
            auto file = std::make_shared<FileLoader>(file_name.c_str());
            if (!file->isValid()) {
                return std::nullopt;
            }
            auto doc = std::make_shared<rapidxml::xml_document<>>();
            doc->parse<0>(file->data());

            return XMLParser{file, doc, std::filesystem::path{file_name}.parent_path().string(), recursive_open_includes};
        }
#endif // _NO_STD_FILESYSTEM

        static std::optional<XMLParser> forXMLString(const std::string &xml_string, bool recursive_open_includes) {
            // pass by value on purpose, rapidxml mutates the string on parse
            auto istream = std::istringstream(xml_string);
            auto file = std::make_shared<FileLoader>(istream);
            if (!file->isValid()) {
                return std::nullopt;
            }
            auto doc = std::make_shared<rapidxml::xml_document<>>();
            doc->parse<0>(file->data());
            return XMLParser{file, doc, "", recursive_open_includes};
        }


        ParseResult parse(std::map<std::string, uint64_t> &out_enum,
                   std::map<std::string, std::shared_ptr<const MessageDefinition>> &out_messages,
                   std::map<int, std::shared_ptr<const MessageDefinition>> &out_message_ids) const {

            auto root_node = _document->first_node("mavlink");
            if (!root_node) {
                return ParseResult::InvalidXml;
            }
#ifndef _NO_STD_FILESYSTEM
            if (_recursive_open_files) {
                for (auto include_element = root_node->first_node("include");
                     include_element != nullptr;
                     include_element = include_element->next_sibling("include")) {

                    const std::string include_name = include_element->value();
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
#endif // _NO_STD_FILESYSTEM

            auto enums_node = root_node->first_node("enums");
            if (enums_node) {
                for (auto enum_node = enums_node->first_node();
                    enum_node != nullptr;
                    enum_node = enum_node->next_sibling()) {

                    for (auto entry = enum_node->first_node();
                        entry != nullptr;
                        entry = entry->next_sibling()) {
                        if (std::string_view("entry") == entry->name()) {
                            auto entry_name = entry->first_attribute("name")->value();
                            auto value_str = entry->first_attribute("value")->value();
                            auto enum_value_opt = _parseEnumValue(value_str);
                            if (!enum_value_opt) {
                                return ParseResult::EnumParseError;
                            }
                            out_enum[entry_name] = enum_value_opt.value();
                        }
                    }
                }
            }

            auto messages_node = root_node->first_node("messages");
            if (messages_node) {
                for (auto message = messages_node->first_node();
                     message != nullptr;
                     message = message->next_sibling()) {

                    const std::string message_name = message->first_attribute("name")->value();
                    auto message_id_opt = _strict_stoul(message->first_attribute("id")->value());
                    if (!message_id_opt || message_id_opt.value() > INT_MAX) {
                        return ParseResult::InvalidXml;
                    }
                    int message_id = static_cast<int>(message_id_opt.value());
                    
                    MessageDefinitionBuilder builder{message_name, message_id};

                    std::string description;

                    bool in_extension_fields = false;
                    for (auto field = message->first_node();
                         field != nullptr;
                         field = field->next_sibling()) {

                        if (std::string_view{"description"} == field->name()) {
                            description = field->value();
                        } else if (std::string_view{"extensions"} == field->name()) {
                            in_extension_fields = true;
                        } else if (std::string_view{"field"} == field->name()) {
                            // parse the field
                            auto field_type_opt = _parseFieldType(field->first_attribute("type")->value());
                            if (!field_type_opt) {
                                return ParseResult::FieldTypeError;
                            }
                            auto field_name = field->first_attribute("name")->value();

                            if (!in_extension_fields) {
                                builder.addField(field_name, field_type_opt.value());
                            } else {
                                builder.addExtensionField(field_name, field_type_opt.value());
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
    };


    class MessageSet {
    private:
        std::map<std::string, uint64_t> _enums;
        std::map<std::string, std::shared_ptr<const MessageDefinition>> _messages;
        std::map<int, std::shared_ptr<const MessageDefinition>> _message_ids;

    public:
        MessageSet() = default;

#ifndef _NO_STD_FILESYSTEM        
        MessageSetResult addFromXML(const std::string &file_path, bool recursive_open_includes = true) {
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
#endif // _NO_STD_FILESYSTEM

        MessageSetResult addFromXMLString(const std::string &xml_string, bool recursive_open_includes = false) {
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


        [[nodiscard]] OptionalReference<const MessageDefinition> getMessageDefinition(const std::string &message_name) const {
            auto message_definition = _messages.find(message_name);
            if (message_definition == _messages.end()) {
                return std::nullopt;
            }
            return *(message_definition->second);
        }

        [[nodiscard]] OptionalReference<const MessageDefinition> getMessageDefinition(int message_id) const {
            auto message_definition = _message_ids.find(message_id);
            if (message_definition == _message_ids.end()) {
                return std::nullopt;
            }
            return *(message_definition->second);
        }

        [[nodiscard]] std::optional<Message> create(const std::string &message_name) const {
            auto message_definition = getMessageDefinition(message_name);
            if (!message_definition) {
                return std::nullopt;
            }
            return Message{message_definition.get()};
        }

        [[nodiscard]] std::optional<Message> create(int message_id) const {
            auto message_definition = getMessageDefinition(message_id);
            if (!message_definition) {
                return std::nullopt;
            }
            return Message{message_definition.get()};
        }

        [[nodiscard]] std::optional<uint64_t> getEnum(const std::string &key) const {
            auto res = _enums.find(key);
            if (res == _enums.end()) {
                return std::nullopt;
            }
            return res->second;
        }

        [[nodiscard]] std::optional<int> idForMessage(const std::string &message_name) const {
            auto message_definition = _messages.find(message_name);
            if (message_definition == _messages.end()) {
                return std::nullopt;
            }
            return message_definition->second->id();
        }

        [[nodiscard]] bool contains(const std::string &message_name) const {
            return _messages.find(message_name) != _messages.end();
        }

        [[nodiscard]] bool contains(int message_id) const {
            return _message_ids.find(message_id) != _message_ids.end();
        }


        [[nodiscard]] size_t size() const {
            return _messages.size();
        }
    };
}




#endif //MAV_MESSAGESET_H
