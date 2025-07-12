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
#include <map>
#include <vector>
#include <string>
#include <string_view>
#include <fstream>

#include "MessageDefinition.h"
#include "Message.h"

// Use tinyxml2 for robust XML parsing with proper error handling
#include <tinyxml2.h>

#include <filesystem>

namespace mav {

    // Class declarations with minimal interface
    class FileLoader {
    private:
        std::vector<char> m_data;
        bool m_valid;

    public:
        explicit FileLoader(const char *filename);
        explicit FileLoader(std::istream &stream);
        char *data();
        const char *data() const;
        std::size_t size() const;
        bool isValid() const;
    };

    class XMLParser {
    private:
        std::shared_ptr<FileLoader> _source_file;
        std::unique_ptr<tinyxml2::XMLDocument> _document;
        std::string _root_xml_folder_path;
        bool _recursive_open_files;

        XMLParser(std::shared_ptr<FileLoader> source_file,
                 std::unique_ptr<tinyxml2::XMLDocument> document,
                 const std::string &root_xml_folder_path,
                 bool recursive_open_files);

        static std::optional<uint64_t> _strict_stoul(const std::string &str, int base=10);
        static std::optional<uint64_t> _parseEnumValue(const std::string &str);
        static bool _isPrefix(std::string_view prefix, std::string_view full) noexcept;
        static std::optional<FieldType> _parseFieldType(const std::string &field_type_string);

    public:
        static std::optional<XMLParser> forFile(const std::string &file_name, bool recursive_open_includes);
        static std::optional<XMLParser> forXMLString(const std::string &xml_string, bool recursive_open_includes);
        ParseResult parse(std::map<std::string, uint64_t> &out_enum,
                   std::map<std::string, std::shared_ptr<const MessageDefinition>> &out_messages,
                   std::map<int, std::shared_ptr<const MessageDefinition>> &out_message_ids) const;
    };

    class MessageSet {
    private:
        std::map<std::string, uint64_t> _enums;
        std::map<std::string, std::shared_ptr<const MessageDefinition>> _messages;
        std::map<int, std::shared_ptr<const MessageDefinition>> _message_ids;

    public:
        MessageSet() = default;

        MessageSetResult addFromXML(const std::string &file_path, bool recursive_open_includes = true);

        MessageSetResult addFromXMLString(const std::string &xml_string, bool recursive_open_includes = false);

        [[nodiscard]] OptionalReference<const MessageDefinition> getMessageDefinition(const std::string &message_name) const;

        [[nodiscard]] OptionalReference<const MessageDefinition> getMessageDefinition(int message_id) const;

        [[nodiscard]] std::optional<Message> create(const std::string &message_name) const;

        [[nodiscard]] std::optional<Message> create(int message_id) const;

        [[nodiscard]] std::optional<uint64_t> getEnum(const std::string &key) const;

        [[nodiscard]] std::optional<int> idForMessage(const std::string &message_name) const;

        [[nodiscard]] bool contains(const std::string &message_name) const;

        [[nodiscard]] bool contains(int message_id) const;

        [[nodiscard]] size_t size() const;
    };
}

#endif //MAV_MESSAGESET_H