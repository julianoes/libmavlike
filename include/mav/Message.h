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

#ifndef MAV_DYNAMICMESSAGE_H
#define MAV_DYNAMICMESSAGE_H

#include <memory>
#include <array>
#include <utility>
#include <variant>
#include "MessageDefinition.h"
#include "utils.h"
#include <picosha2.h>

namespace mav {

    using NativeVariantType = std::variant<
            std::string,
            std::vector<uint64_t>,
            std::vector<int64_t>,
            std::vector<uint32_t>,
            std::vector<int32_t>,
            std::vector<uint16_t>,
            std::vector<int16_t>,
            std::vector<uint8_t>,
            std::vector<int8_t>,
            std::vector<double>,
            std::vector<float>,
            uint64_t,
            int64_t,
            uint32_t,
            int32_t,
            uint16_t,
            int16_t,
            uint8_t,
            int8_t,
            char,
            double,
            float
    >;


    // forward declared MessageSet
    class MessageSet;

    class Message {
        friend MessageSet;
    private:
        ConnectionPartner _source_partner{};
        std::array<uint8_t, MessageDefinition::MAX_MESSAGE_SIZE> _backing_memory{};
        const MessageDefinition* _message_definition{nullptr};
        int _crc_offset{-1};

        explicit Message(const MessageDefinition &message_definition) :
            _message_definition(&message_definition) {
        }

        Message(const MessageDefinition &message_definition, ConnectionPartner source_partner, int crc_offset,
                std::array<uint8_t, MessageDefinition::MAX_MESSAGE_SIZE> &&backing_memory) :
                _source_partner(source_partner),
                _backing_memory(std::move(backing_memory)),
                _message_definition(&message_definition),
                _crc_offset(crc_offset) {}

        inline bool isFinalized() const {
            return _crc_offset >= 0;
        }

        inline void _unFinalize() {
            if (_crc_offset >= 0) {
                std::fill(_backing_memory.begin() + _crc_offset,
                          _backing_memory.begin() + _backing_memory.size(), 0);
                _crc_offset = -1;
            }
        }

        template <typename T>
        void _writeSingle(const Field &field, const T &v, int in_field_offset = 0) {
            // any write will potentially change the crc offset, so we invalidate it
            _unFinalize();
            // make sure that we only have simplistic base types here
            static_assert(is_any<std::decay_t<T>, short, int, long, unsigned int, unsigned long,
                    char, uint8_t, int8_t, uint16_t, int16_t, uint32_t, int32_t, uint64_t, int64_t, float, double
            >::value, "Can not set this data type to a mavlink message field.");
            // We serialize to the data type given in the field definition, not the data type used in the API.
            // This allows to use compatible data types in the API, but have them serialized to the correct data type.
            int offset = field.offset + in_field_offset;
            uint8_t* target = _backing_memory.data() + offset;

            switch (field.type.base_type) {
                case FieldType::BaseType::CHAR: return serialize(static_cast<char>(v), target);
                case FieldType::BaseType::UINT8: return serialize(static_cast<uint8_t>(v), target);
                case FieldType::BaseType::UINT16: return serialize(static_cast<uint16_t>(v), target);
                case FieldType::BaseType::UINT32: return serialize(static_cast<uint32_t>(v), target);
                case FieldType::BaseType::UINT64: return serialize(static_cast<uint64_t>(v), target);
                case FieldType::BaseType::INT8: return serialize(static_cast<int8_t>(v), target);
                case FieldType::BaseType::INT16: return serialize(static_cast<int16_t>(v), target);
                case FieldType::BaseType::INT32: return serialize(static_cast<int32_t>(v), target);
                case FieldType::BaseType::INT64: return serialize(static_cast<int64_t>(v), target);
                case FieldType::BaseType::FLOAT: return serialize(static_cast<float>(v), target);
                case FieldType::BaseType::DOUBLE: return serialize(static_cast<double>(v), target);
            }
        }

        template <typename T>
        inline T _readSingle(const Field &field, int in_field_offset = 0) const {
            int data_offset = field.offset + in_field_offset;
            int max_size = isFinalized() ? _crc_offset - data_offset : field.type.baseSize();
            const uint8_t* b_ptr = _backing_memory.data() + data_offset;
            switch (field.type.base_type) {
                case FieldType::BaseType::CHAR: return static_cast<T>(deserialize<char>(b_ptr, max_size));
                case FieldType::BaseType::UINT8: return static_cast<T>(deserialize<uint8_t>(b_ptr, max_size));
                case FieldType::BaseType::UINT16: return static_cast<T>(deserialize<uint16_t>(b_ptr, max_size));
                case FieldType::BaseType::UINT32: return static_cast<T>(deserialize<uint32_t>(b_ptr, max_size));
                case FieldType::BaseType::UINT64: return static_cast<T>(deserialize<uint64_t>(b_ptr, max_size));
                case FieldType::BaseType::INT8: return static_cast<T>(deserialize<int8_t>(b_ptr, max_size));
                case FieldType::BaseType::INT16: return static_cast<T>(deserialize<int16_t>(b_ptr, max_size));
                case FieldType::BaseType::INT32: return static_cast<T>(deserialize<int32_t>(b_ptr, max_size));
                case FieldType::BaseType::INT64: return static_cast<T>(deserialize<int64_t>(b_ptr, max_size));
                case FieldType::BaseType::FLOAT: return static_cast<T>(deserialize<float>(b_ptr, max_size));
                case FieldType::BaseType::DOUBLE: return static_cast<T>(deserialize<double>(b_ptr, max_size));
            }
            return T{}; // return default value instead of throwing
        }

        // Safe signature access methods that don't throw
        std::optional<uint8_t> _getSignatureLinkId() const {
            if (!isFinalized()) {
                return std::nullopt;
            }
            return _backing_memory[MessageDefinition::HEADER_SIZE + header().len() + MessageDefinition::CHECKSUM_SIZE];
        }

        std::optional<uint64_t> _getSignatureTimestamp() const {
            if (!isFinalized()) {
                return std::nullopt;
            }
            const uint8_t* timestamp_ptr = &_backing_memory[MessageDefinition::HEADER_SIZE + header().len() + 
                MessageDefinition::CHECKSUM_SIZE + MessageDefinition::SIGNATURE_LINK_ID_SIZE];
            return deserialize<uint64_t>(timestamp_ptr, MessageDefinition::SIGNATURE_TIMESTAMP_SIZE) & 0xFFFFFFFFFFFF;
        }

        std::optional<uint64_t> _getSignatureSignature() const {
            if (!isFinalized()) {
                return std::nullopt;
            }
            const uint8_t* signature_ptr = &_backing_memory[MessageDefinition::HEADER_SIZE + header().len() + 
                MessageDefinition::CHECKSUM_SIZE + MessageDefinition::SIGNATURE_LINK_ID_SIZE + MessageDefinition::SIGNATURE_TIMESTAMP_SIZE];
            return deserialize<uint64_t>(signature_ptr, MessageDefinition::SIGNATURE_SIGNATURE_SIZE) & 0xFFFFFFFFFFFF;
        }

        uint64_t _computeSignatureHash48(const std::array<uint8_t, MessageDefinition::KEY_SIZE>& key, 
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
            serialize(timestamp, timestampSerialized.begin());
            hasher.process(timestampSerialized.begin(), timestampSerialized.begin() + MessageDefinition::SIGNATURE_TIMESTAMP_SIZE);

            hasher.finish();
            std::vector<unsigned char> hash(picosha2::k_digest_size);
            hasher.get_hash_bytes(hash.begin(), hash.end());
            return deserialize<uint64_t>(hash.data(), MessageDefinition::SIGNATURE_SIGNATURE_SIZE);
        }

    public:

        static inline Message _instantiateFromMemory(const MessageDefinition &definition, ConnectionPartner source_partner,
                                          int crc_offset, std::array<uint8_t, MessageDefinition::MAX_MESSAGE_SIZE> &&backing_memory) {
            return Message{definition, source_partner, crc_offset, std::move(backing_memory)};
        }

        using _InitPairType = std::pair<const std::string, NativeVariantType>;

        template <typename MessageType>
        class _accessorType {
        private:
            const std::string &_field_name;
            MessageType &_message;
            int _array_index;
        public:
            _accessorType(const std::string &field_name, MessageType &message, int array_index) :
                _field_name(field_name), _message(message), _array_index(array_index) {}

            template <typename T>
            MessageResult operator=(const T& val) {
                return _message.template set<T>(_field_name, val, _array_index);
            }

            template <typename T>
            MessageResult get(T& out_value) const {
                return _message.template get<T>(_field_name, out_value, _array_index);
            }

            _accessorType operator[](int array_index) const {
                return _accessorType{_field_name, _message, array_index};
            }

            template <typename T>
            MessageResult floatPack(T value) const {
                return _message.template setAsFloatPack<T>(_field_name, value, _array_index);
            }

            template <typename T>
            MessageResult floatUnpack(T& out_value) const {
                return _message.template getAsFloatUnpack<T>(_field_name, out_value, _array_index);
            }
        };

        [[nodiscard]] const MessageDefinition& type() const {
            return *_message_definition;
        }

        [[nodiscard]] int id() const {
            return _message_definition->id();
        }

        [[nodiscard]] const std::string& name() const {
            return _message_definition->name();
        }

        [[nodiscard]] const Header<const uint8_t*> header() const {
            return Header<const uint8_t*>(_backing_memory.data());
        }

        [[nodiscard]] Header<uint8_t*> header() {
            return Header<uint8_t*>(_backing_memory.data());
        }

        [[nodiscard]] std::optional<Signature<const uint8_t*>> signature() const {
            if (!isFinalized()) {
                return std::nullopt;
            }
            return Signature<const uint8_t*>(&_backing_memory[MessageDefinition::HEADER_SIZE + header().len() + MessageDefinition::CHECKSUM_SIZE]);
        }

        [[nodiscard]] std::optional<Signature<uint8_t*>> signature() {
            if (!isFinalized()) {
                return std::nullopt;
            }
            return Signature<uint8_t*>(&_backing_memory[MessageDefinition::HEADER_SIZE + header().len() + MessageDefinition::CHECKSUM_SIZE]);
        }

        [[nodiscard]] const ConnectionPartner& source() const {
            return _source_partner;
        }

        MessageResult setFromNativeTypeVariant(const std::string &field_key, const NativeVariantType &v) {
            MessageResult result = MessageResult::Success;
            std::visit([this, &field_key, &result](auto&& arg) {
                result = this->set(field_key, arg);
            }, v);
            return result;
        }

        MessageResult set(std::initializer_list<_InitPairType> init) {
            for (const auto &pair : init) {
                const auto &key = pair.first;
                auto result = setFromNativeTypeVariant(key, pair.second);
                if (result != MessageResult::Success) {
                    return result;
                }
            }
            return MessageResult::Success;
        }

        template <typename T>
        MessageResult set(const std::string &field_key, const T &v, int array_index = 0) {
            auto field_opt = _message_definition->getField(field_key);
            if (!field_opt) {
                return MessageResult::FieldNotFound;
            }
            auto field = field_opt.value();

            if constexpr(is_string<T>::value) {
                (void)array_index; // unused
                return setString(field_key, v);
            }

            else if constexpr(is_iterable<T>::value) {
                (void)array_index; // unused
                auto begin = std::begin(v);
                auto end = std::end(v);
                if ((end - begin) > field.type.size) {
                    return MessageResult::OutOfRange;
                }

                for (int i = 0; begin != end; (void)++begin, ++i) {
                    _writeSingle(field, *begin, (i * field.type.baseSize()));
                }
            } else {
                if (array_index < 0 || array_index >= field.type.size) {
                    return MessageResult::OutOfRange;
                }

                _writeSingle(field, v, array_index * field.type.baseSize());
            }
            return MessageResult::Success;
        }

        template <typename T>
        MessageResult setAsFloatPack(const std::string &field_key, const T &v, int array_index = 0) {
            if constexpr(is_string<T>::value) {
                return MessageResult::TypeMismatch;
            } else if constexpr(is_iterable<T>::value) {
                return MessageResult::TypeMismatch;
            } else {
                return set<float>(field_key, floatPack<T>(v), array_index);
            }
        }


        MessageResult setString(const std::string &field_key, const std::string &v) {
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


        template <typename T>
        MessageResult get(const std::string &field_key, T& out_value, int array_index = 0) const {
            if constexpr(is_string<T>::value) {
                return getString(field_key, out_value);
            } else if constexpr(is_iterable<T>::value) {
                auto field_opt = _message_definition->getField(field_key);
                if (!field_opt) {
                    return MessageResult::FieldNotFound;
                }
                auto field = field_opt.value();

                // handle std::vector: Dynamically resize for convenience
                if constexpr(std::is_same<T, std::vector<typename T::value_type>>::value) {
                    out_value.resize(field.type.size);
                }

                if (static_cast<int>(out_value.size()) < field.type.size) {
                    return MessageResult::OutOfRange;
                }

                for (int i=0; i<field.type.size; i++) {
                    out_value[i] = _readSingle<typename T::value_type>(field, i * field.type.baseSize());
                }
                return MessageResult::Success;
            } else {
                auto field_opt = _message_definition->getField(field_key);
                if (!field_opt) {
                    return MessageResult::FieldNotFound;
                }
                auto field = field_opt.value();
                
                if (array_index < 0 || array_index >= field.type.size) {
                    return MessageResult::OutOfRange;
                }
                out_value = _readSingle<T>(field, array_index * field.type.baseSize());
                return MessageResult::Success;
            }
        }

        template <typename T>
        MessageResult getAsFloatUnpack(const std::string &field_key, T& out_value, int array_index = 0) const {
            if constexpr(is_string<T>::value) {
                (void)array_index; // unused
                return MessageResult::TypeMismatch;
            } else if constexpr(is_iterable<T>::value) {
                (void)array_index; // unused
                return MessageResult::TypeMismatch;
            } else {
                float float_value;
                auto result = get<float>(field_key, float_value, array_index);
                if (result != MessageResult::Success) {
                    return result;
                }
                out_value = floatUnpack<T>(float_value);
                return MessageResult::Success;
            }
        }

        MessageResult getString(const std::string &field_key, std::string& out_value) const {
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


        _accessorType<const Message> operator[](const std::string &field_name) const {
            return _accessorType<const Message>{field_name, *this, 0};
        }

        _accessorType<Message> operator[](const std::string &field_name) {
            return _accessorType<Message>{field_name, *this, 0};
        }

        std::optional<NativeVariantType> getAsNativeTypeInVariant(const std::string &field_key) const {
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

        [[nodiscard]] std::string toString() const {
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

        std::optional<bool> validate(const std::array<uint8_t, MessageDefinition::KEY_SIZE>& key) const {
            auto linkId_opt = _getSignatureLinkId();
            auto timestamp_opt = _getSignatureTimestamp();
            auto signature_opt = _getSignatureSignature();
            
            if (!linkId_opt || !timestamp_opt || !signature_opt) {
                return std::nullopt;
            }
            
            return signature_opt.value() == _computeSignatureHash48(key, linkId_opt.value(), timestamp_opt.value());
        }

        std::optional<uint32_t> finalize(uint8_t seq, const Identifier &sender) {
            static const std::array<uint8_t, MessageDefinition::KEY_SIZE> null_key = {};
            return finalize(seq, sender, null_key, 0, 0);
        }

        std::optional<uint32_t> finalize(uint8_t seq, const Identifier &sender,
                                        const std::array<uint8_t, MessageDefinition::KEY_SIZE>& key,
                                        const uint64_t& timestamp, const uint8_t linkId = 0) {
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
                _backing_memory[MessageDefinition::HEADER_SIZE + payload_size + MessageDefinition::CHECKSUM_SIZE] = linkId;
                
                // Set timestamp
                uint8_t* timestamp_ptr = &_backing_memory[MessageDefinition::HEADER_SIZE + payload_size + 
                    MessageDefinition::CHECKSUM_SIZE + MessageDefinition::SIGNATURE_LINK_ID_SIZE];
                serialize(timestamp & 0xFFFFFFFFFFFF, timestamp_ptr);
                
                // Compute and set signature
                uint64_t computed_signature = _computeSignatureHash48(key, linkId, timestamp);
                uint8_t* signature_ptr = &_backing_memory[MessageDefinition::HEADER_SIZE + payload_size + 
                    MessageDefinition::CHECKSUM_SIZE + MessageDefinition::SIGNATURE_LINK_ID_SIZE + MessageDefinition::SIGNATURE_TIMESTAMP_SIZE];
                serialize(computed_signature & 0xFFFFFFFFFFFF, signature_ptr);
                
                signature_size = MessageDefinition::SIGNATURE_SIZE;
            }

            return MessageDefinition::HEADER_SIZE + payload_size + MessageDefinition::CHECKSUM_SIZE + signature_size;
        }

        [[nodiscard]] const uint8_t* data() const {
            return _backing_memory.data();
        };
    };

} // namespace libmavlink`

#endif //MAV_DYNAMICMESSAGE_H