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

#include "mav/Message.h"
#include <sstream>
#include <picosha2.h>

namespace mav {

    std::string Message::toString() const {
        std::stringstream  ss;
        ss << "Message ID " << id() << " (" << name() << ") \n";
        for (const auto &field_key : _message_definition->fieldNames()) {
            ss << "  " << field_key << ": ";
            auto variant_opt = getAsNativeTypeInVariant(field_key);
            if (variant_opt) {
                std::visit([&ss](auto&& arg) {
                    if constexpr (is_string<decltype(arg)>::value) {
                        ss << "\"" << arg << "\"";
                    } else if constexpr (is_any<std::decay_t<decltype(arg)>, uint8_t, int8_t>::value) {
                        // static cast to int to avoid printing as a char
                        ss << static_cast<int>(arg);
                    } else if constexpr (is_iterable<decltype(arg)>::value) {
                        for (auto it = arg.begin(); it != arg.end(); it++) {
                            if (it != arg.begin())
                                ss << ", ";
                            ss << *it;
                        }
                    } else {
                        ss << arg;
                    }
                }, variant_opt.value());
            } else {
                ss << "<error reading field>";
            }
            ss << "\n";
        }
        return ss.str();
    }

    std::optional<NativeVariantType> Message::getAsNativeTypeInVariant(const std::string &field_key) const noexcept {
        auto field_opt = _message_definition->getField(field_key);
        if (!field_opt) {
            return std::nullopt;
        }
        auto field = field_opt.value();
        
        if (field.type.size <= 1) {
            switch (field.type.base_type) {
                case FieldType::BaseType::CHAR: {
                    char value;
                    if (get<char>(field_key, value) == MessageResult::Success) return value;
                    break;
                }
                case FieldType::BaseType::UINT8: {
                    uint8_t value;
                    if (get<uint8_t>(field_key, value) == MessageResult::Success) return value;
                    break;
                }
                case FieldType::BaseType::UINT16: {
                    uint16_t value;
                    if (get<uint16_t>(field_key, value) == MessageResult::Success) return value;
                    break;
                }
                case FieldType::BaseType::UINT32: {
                    uint32_t value;
                    if (get<uint32_t>(field_key, value) == MessageResult::Success) return value;
                    break;
                }
                case FieldType::BaseType::UINT64: {
                    uint64_t value;
                    if (get<uint64_t>(field_key, value) == MessageResult::Success) return value;
                    break;
                }
                case FieldType::BaseType::INT8: {
                    int8_t value;
                    if (get<int8_t>(field_key, value) == MessageResult::Success) return value;
                    break;
                }
                case FieldType::BaseType::INT16: {
                    int16_t value;
                    if (get<int16_t>(field_key, value) == MessageResult::Success) return value;
                    break;
                }
                case FieldType::BaseType::INT32: {
                    int32_t value;
                    if (get<int32_t>(field_key, value) == MessageResult::Success) return value;
                    break;
                }
                case FieldType::BaseType::INT64: {
                    int64_t value;
                    if (get<int64_t>(field_key, value) == MessageResult::Success) return value;
                    break;
                }
                case FieldType::BaseType::FLOAT: {
                    float value;
                    if (get<float>(field_key, value) == MessageResult::Success) return value;
                    break;
                }
                case FieldType::BaseType::DOUBLE: {
                    double value;
                    if (get<double>(field_key, value) == MessageResult::Success) return value;
                    break;
                }
            }
        } else {
            switch (field.type.base_type) {
                case FieldType::BaseType::CHAR: {
                    std::string value;
                    if (getString(field_key, value) == MessageResult::Success) return value;
                    break;
                }
                case FieldType::BaseType::UINT8: {
                    std::vector<uint8_t> value;
                    if (get<std::vector<uint8_t>>(field_key, value) == MessageResult::Success) return value;
                    break;
                }
                case FieldType::BaseType::UINT16: {
                    std::vector<uint16_t> value;
                    if (get<std::vector<uint16_t>>(field_key, value) == MessageResult::Success) return value;
                    break;
                }
                case FieldType::BaseType::UINT32: {
                    std::vector<uint32_t> value;
                    if (get<std::vector<uint32_t>>(field_key, value) == MessageResult::Success) return value;
                    break;
                }
                case FieldType::BaseType::UINT64: {
                    std::vector<uint64_t> value;
                    if (get<std::vector<uint64_t>>(field_key, value) == MessageResult::Success) return value;
                    break;
                }
                case FieldType::BaseType::INT8: {
                    std::vector<int8_t> value;
                    if (get<std::vector<int8_t>>(field_key, value) == MessageResult::Success) return value;
                    break;
                }
                case FieldType::BaseType::INT16: {
                    std::vector<int16_t> value;
                    if (get<std::vector<int16_t>>(field_key, value) == MessageResult::Success) return value;
                    break;
                }
                case FieldType::BaseType::INT32: {
                    std::vector<int32_t> value;
                    if (get<std::vector<int32_t>>(field_key, value) == MessageResult::Success) return value;
                    break;
                }
                case FieldType::BaseType::INT64: {
                    std::vector<int64_t> value;
                    if (get<std::vector<int64_t>>(field_key, value) == MessageResult::Success) return value;
                    break;
                }
                case FieldType::BaseType::FLOAT: {
                    std::vector<float> value;
                    if (get<std::vector<float>>(field_key, value) == MessageResult::Success) return value;
                    break;
                }
                case FieldType::BaseType::DOUBLE: {
                    std::vector<double> value;
                    if (get<std::vector<double>>(field_key, value) == MessageResult::Success) return value;
                    break;
                }
            }
        }
        return std::nullopt;
    }

    MessageResult Message::setString(const std::string &field_key, const std::string &v) noexcept {
        auto field_opt = _message_definition->getField(field_key);
        if (!field_opt) {
            return MessageResult::FieldNotFound;
        }
        auto field = field_opt.value();
        
        if (field.type.base_type != FieldType::BaseType::CHAR) {
            return MessageResult::TypeMismatch;
        }
        if (static_cast<int>(v.size()) > field.type.size) {
            return MessageResult::OutOfRange;
        }
        int i = 0;
        for (char c : v) {
            _writeSingle(field, c, i);
            i++;
        }
        // write a terminating zero only if there is still enough space
        if (i < field.type.size) {
            _writeSingle(field, '\0', i);
            i++;
        }
        return MessageResult::Success;
    }

    MessageResult Message::getString(const std::string &field_key, std::string& out_value) const noexcept {
        auto field_opt = _message_definition->getField(field_key);
        if (!field_opt) {
            return MessageResult::FieldNotFound;
        }
        auto field = field_opt.value();
        
        if (field.type.base_type != FieldType::BaseType::CHAR) {
            return MessageResult::TypeMismatch;
        }
        int max_string_length = isFinalized() ?
                std::min(field.type.size, _crc_offset - field.offset) : field.type.size;
        int real_string_length = strnlen(_backing_memory.data() + field.offset, max_string_length);

        out_value = std::string{reinterpret_cast<const char*>(_backing_memory.data() + field.offset),
                           static_cast<std::string::size_type>(real_string_length)};
        return MessageResult::Success;
    }

    std::optional<bool> Message::validate(const std::array<uint8_t, MessageDefinition::KEY_SIZE>& key) const noexcept {
        auto linkId_opt = _getSignatureLinkId();
        auto timestamp_opt = _getSignatureTimestamp();
        auto signature_opt = _getSignatureSignature();
        
        if (!linkId_opt || !timestamp_opt || !signature_opt) {
            return std::nullopt;
        }
        
        return signature_opt.value() == _computeSignatureHash48(key, linkId_opt.value(), timestamp_opt.value());
    }

    std::optional<uint32_t> Message::finalize(uint8_t seq, const Identifier &sender) noexcept {
        static const std::array<uint8_t, MessageDefinition::KEY_SIZE> null_key = {};
        return finalize(seq, sender, null_key, 0, 0);
    }

    std::optional<uint32_t> Message::finalize(uint8_t seq, const Identifier &sender,
                                            const std::array<uint8_t, MessageDefinition::KEY_SIZE>& key,
                                            const uint64_t& timestamp, const uint8_t linkId) noexcept {
        if (isFinalized()) {
            _unFinalize();
        }

        bool sign = (timestamp > 0);
        auto last_nonzero = std::find_if(_backing_memory.rend() -
                MessageDefinition::HEADER_SIZE - _message_definition->maxPayloadSize(),
                _backing_memory.rend(), [](const auto &v) {
            return v != 0;
        });

        int payload_size = std::max(
                static_cast<int>(std::distance(last_nonzero, _backing_memory.rend()))
                        - MessageDefinition::HEADER_SIZE, 1);

        header().magic() = 0xFD;
        header().len() = static_cast<uint8_t>(payload_size);
        header().incompatFlags() = sign ? 0x01 : 0x00;
        header().compatFlags() = 0;
        header().seq() = seq;
        if (header().systemId() == 0) {
            header().systemId() = static_cast<uint8_t>(sender.system_id);
        }
        if (header().componentId() == 0) {
            header().componentId() = static_cast<uint8_t>(sender.component_id);
        }
        header().msgId() = _message_definition->id();

        CRC crc;
        crc.accumulate(_backing_memory.begin() + 1, _backing_memory.begin() +
            MessageDefinition::HEADER_SIZE + payload_size);
        crc.accumulate(_message_definition->crcExtra());
        _crc_offset = MessageDefinition::HEADER_SIZE + payload_size;
        serialize(crc.crc16(), _backing_memory.data() + _crc_offset);

        int signature_size = 0;
        if (sign) {
            // Set signature data directly without using throwing signature() accessors
            _backing_memory[static_cast<size_t>(MessageDefinition::HEADER_SIZE + payload_size + MessageDefinition::CHECKSUM_SIZE)] = linkId;
            
            // Set timestamp
            uint8_t* timestamp_ptr = &_backing_memory[static_cast<size_t>(MessageDefinition::HEADER_SIZE + payload_size + 
                MessageDefinition::CHECKSUM_SIZE + MessageDefinition::SIGNATURE_LINK_ID_SIZE)];
            serialize(timestamp & 0xFFFFFFFFFFFF, timestamp_ptr);
            
            // Compute and set signature
            uint64_t computed_signature = _computeSignatureHash48(key, linkId, timestamp);
            uint8_t* signature_ptr = &_backing_memory[static_cast<size_t>(MessageDefinition::HEADER_SIZE + payload_size + 
                MessageDefinition::CHECKSUM_SIZE + MessageDefinition::SIGNATURE_LINK_ID_SIZE + MessageDefinition::SIGNATURE_TIMESTAMP_SIZE)];
            serialize(computed_signature & 0xFFFFFFFFFFFF, signature_ptr);
            
            signature_size = MessageDefinition::SIGNATURE_SIZE;
        }

        return MessageDefinition::HEADER_SIZE + payload_size + MessageDefinition::CHECKSUM_SIZE + signature_size;
    }

    uint64_t Message::_computeSignatureHash48(const std::array<uint8_t, MessageDefinition::KEY_SIZE>& key, 
                                             uint8_t linkId, uint64_t timestamp) const {
        // signature = sha256_48(secret_key + header + payload + CRC + link-ID + timestamp)
        picosha2::hash256_one_by_one hasher;
        // secret_key
        hasher.process(key.begin(), key.begin() + MessageDefinition::KEY_SIZE);
        // header + payload + CRC
        hasher.process(_backing_memory.begin(), _backing_memory.begin() + 
                MessageDefinition::HEADER_SIZE + header().len() + MessageDefinition::CHECKSUM_SIZE);
        // link-ID
        hasher.process(&linkId, &linkId + MessageDefinition::SIGNATURE_LINK_ID_SIZE);
        // timestamp
        std::array<uint8_t, sizeof(timestamp)> timestampSerialized;
        serialize(timestamp, timestampSerialized.data());
        hasher.process(timestampSerialized.begin(), timestampSerialized.begin() + MessageDefinition::SIGNATURE_TIMESTAMP_SIZE);

        hasher.finish();
        std::vector<unsigned char> hash(picosha2::k_digest_size);
        hasher.get_hash_bytes(hash.begin(), hash.end());
        return deserialize<uint64_t>(hash.data(), MessageDefinition::SIGNATURE_SIGNATURE_SIZE);
    }

} // namespace mav