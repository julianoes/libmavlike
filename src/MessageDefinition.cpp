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

#include "mav/MessageDefinition.h"
#include <sstream>
#include <algorithm>

namespace mav {

    std::string ConnectionPartner::toString() const {
        if (isUart()) {
            std::stringstream ss;
            ss << "UART " << std::hex << address();
            return ss.str();
        } else {
            std::stringstream ss;
            for (int i = 0; i < 4; i++) {
                ss << ((address() >> (8 * i)) & 0xFF);
                if (i < 3) {
                    ss << ".";
                }
            }
            ss << ":" << port();
            return ss.str();
        }
    }

    int FieldType::baseSize() const noexcept {
        switch(base_type) {
            case BaseType::CHAR: return 1;
            case BaseType::UINT8: return 1;
            case BaseType::UINT16: return 2;
            case BaseType::UINT32: return 4;
            case BaseType::UINT64: return 8;
            case BaseType::INT8: return 1;
            case BaseType::INT16: return 2;
            case BaseType::INT32: return 4;
            case BaseType::INT64: return 8;
            case BaseType::FLOAT: return 4;
            case BaseType::DOUBLE: return 8;
        }
        return 0;
    }

    const char* FieldType::crcNameString() const noexcept {
        switch(base_type) {
            case BaseType::CHAR: return "char";
            case BaseType::UINT8: return "uint8_t";
            case BaseType::UINT16: return "uint16_t";
            case BaseType::UINT32: return "uint32_t";
            case BaseType::UINT64: return "uint64_t";
            case BaseType::INT8: return "int8_t";
            case BaseType::INT16: return "int16_t";
            case BaseType::INT32: return "int32_t";
            case BaseType::INT64: return "int64_t";
            case BaseType::FLOAT: return "float";
            case BaseType::DOUBLE: return "double";
        }
        return "";
    }

    std::vector<std::string> MessageDefinition::fieldNames() const {
        std::vector<std::string> keys;
        for(auto const& item: _fields) {
            keys.push_back(item.first);
        }
        return keys;
    }

    MessageDefinition MessageDefinitionBuilder::build() {
        // As to mavlink spec, all main fields are sorted by their data type size
        std::stable_sort(_fields.begin(), _fields.end(),
                         [](const NamedField &a, const NamedField &b) -> bool {
                             return a.type.baseSize() > b.type.baseSize();
                         });

        int offset = MessageDefinition::HEADER_SIZE;
        CRC crc_extra;
        crc_extra.accumulate(_result._name);
        crc_extra.accumulate(" ");

        for (const auto &field : _fields) {
            const auto &type = field.type;
            _result._fields.insert({field.name, {type, offset}});
            offset = offset + (type.baseSize() * type.size);
            crc_extra.accumulate(type.crcNameString());
            crc_extra.accumulate(" ");
            crc_extra.accumulate(field.name);
            crc_extra.accumulate(" ");
            // in case this is an array (more than 1 element), we also have to add the array size
            if (type.size > 1) {
                crc_extra.accumulate(static_cast<uint8_t>(type.size));
            }
        }
        _result._crc_extra = crc_extra.crc8();

        _result._not_extended_payload_length = offset - MessageDefinition::HEADER_SIZE;

        for (const auto &field : _extension_fields) {
            const auto &type = field.type;
            _result._fields.insert({field.name, {type, offset}});
            offset = offset + (type.baseSize() * type.size);
        }
        _result._max_payload_length = offset - MessageDefinition::HEADER_SIZE;
        _result._max_buffer_length = offset + MessageDefinition::CHECKSUM_SIZE + MessageDefinition::SIGNATURE_SIZE;
        return _result;
    }

} // namespace mav
