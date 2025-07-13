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

        inline bool isFinalized() const noexcept {
            return _crc_offset >= 0;
        }

        inline void _unFinalize() noexcept {
            if (_crc_offset >= 0) {
                std::fill(_backing_memory.begin() + _crc_offset,
                          _backing_memory.begin() + _backing_memory.size(), 0);
                _crc_offset = -1;
            }
        }

        template <typename T>
        void _writeSingle(const Field &field, const T &v, int in_field_offset = 0) noexcept {
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
        std::optional<uint8_t> _getSignatureLinkId() const noexcept {
            if (!isFinalized()) {
                return std::nullopt;
            }
            return _backing_memory[MessageDefinition::HEADER_SIZE + header().len() + MessageDefinition::CHECKSUM_SIZE];
        }

        std::optional<uint64_t> _getSignatureTimestamp() const noexcept {
            if (!isFinalized()) {
                return std::nullopt;
            }
            const uint8_t* timestamp_ptr = &_backing_memory[MessageDefinition::HEADER_SIZE + header().len() + 
                MessageDefinition::CHECKSUM_SIZE + MessageDefinition::SIGNATURE_LINK_ID_SIZE];
            return deserialize<uint64_t>(timestamp_ptr, MessageDefinition::SIGNATURE_TIMESTAMP_SIZE) & 0xFFFFFFFFFFFF;
        }

        std::optional<uint64_t> _getSignatureSignature() const noexcept {
            if (!isFinalized()) {
                return std::nullopt;
            }
            const uint8_t* signature_ptr = &_backing_memory[MessageDefinition::HEADER_SIZE + header().len() + 
                MessageDefinition::CHECKSUM_SIZE + MessageDefinition::SIGNATURE_LINK_ID_SIZE + MessageDefinition::SIGNATURE_TIMESTAMP_SIZE];
            return deserialize<uint64_t>(signature_ptr, MessageDefinition::SIGNATURE_SIGNATURE_SIZE) & 0xFFFFFFFFFFFF;
        }

        uint64_t _computeSignatureHash48(const std::array<uint8_t, MessageDefinition::KEY_SIZE>& key, 
                                         uint8_t linkId, uint64_t timestamp) const;

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

        [[nodiscard]] const MessageDefinition& type() const noexcept {
            return *_message_definition;
        }

        [[nodiscard]] int id() const noexcept {
            return _message_definition->id();
        }

        [[nodiscard]] const std::string& name() const noexcept {
            return _message_definition->name();
        }

        [[nodiscard]] const Header<const uint8_t*> header() const noexcept {
            return Header<const uint8_t*>(_backing_memory.data());
        }

        [[nodiscard]] Header<uint8_t*> header() noexcept {
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

        MessageResult setFromNativeTypeVariant(const std::string &field_key, const NativeVariantType &v) noexcept {
            MessageResult result = MessageResult::Success;
            std::visit([this, &field_key, &result](auto&& arg) {
                result = this->set(field_key, arg);
            }, v);
            return result;
        }

        MessageResult set(std::initializer_list<_InitPairType> init) noexcept {
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
        MessageResult set(const std::string &field_key, const T &v, int array_index = 0) noexcept {
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
        MessageResult setAsFloatPack(const std::string &field_key, const T &v, int array_index = 0) noexcept {
            if constexpr(is_string<T>::value) {
                return MessageResult::TypeMismatch;
            } else if constexpr(is_iterable<T>::value) {
                return MessageResult::TypeMismatch;
            } else {
                return set<float>(field_key, floatPack<T>(v), array_index);
            }
        }


        MessageResult setString(const std::string &field_key, const std::string &v) noexcept;


        template <typename T>
        MessageResult get(const std::string &field_key, T& out_value, int array_index = 0) const noexcept {
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
                    out_value.resize(static_cast<size_t>(field.type.size));
                }

                if (static_cast<int>(out_value.size()) < field.type.size) {
                    return MessageResult::OutOfRange;
                }

                for (int i=0; i<field.type.size; i++) {
                    out_value[static_cast<size_t>(i)] = _readSingle<typename T::value_type>(field, i * field.type.baseSize());
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
        MessageResult getAsFloatUnpack(const std::string &field_key, T& out_value, int array_index = 0) const noexcept {
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

        MessageResult getString(const std::string &field_key, std::string& out_value) const noexcept;


        _accessorType<const Message> operator[](const std::string &field_name) const {
            return _accessorType<const Message>{field_name, *this, 0};
        }

        _accessorType<Message> operator[](const std::string &field_name) {
            return _accessorType<Message>{field_name, *this, 0};
        }

        std::optional<NativeVariantType> getAsNativeTypeInVariant(const std::string &field_key) const noexcept;

        [[nodiscard]] std::string toString() const;

        std::optional<bool> validate(const std::array<uint8_t, MessageDefinition::KEY_SIZE>& key) const noexcept;

        std::optional<uint32_t> finalize(uint8_t seq, const Identifier &sender) noexcept;

        std::optional<uint32_t> finalize(uint8_t seq, const Identifier &sender,
                                        const std::array<uint8_t, MessageDefinition::KEY_SIZE>& key,
                                        const uint64_t& timestamp, const uint8_t linkId = 0) noexcept;

        [[nodiscard]] const uint8_t* data() const noexcept {
            return _backing_memory.data();
        };

        [[nodiscard]] uint32_t finalizedSize() const noexcept {
            if (!isFinalized()) {
                return 0;
            }
            
            // Calculate size from MAVLink header
            const uint8_t* data_ptr = _backing_memory.data();
            uint32_t data_size = 0;
            
            if (data_ptr[0] == 0xFD) { // MAVLink v2
                data_size = data_ptr[1] + 12; // payload length + header (10) + checksum (2)
                if (data_ptr[2] & 0x01) { // MAVLINK_IFLAG_SIGNED
                    data_size += 13; // signature
                }
            } else if (data_ptr[0] == 0xFE) { // MAVLink v1
                data_size = data_ptr[1] + 8; // payload length + header (6) + checksum (2)
            }
            
            return data_size;
        };

        // Helper methods for clean payload access
        [[nodiscard]] const uint8_t* getPayloadData() const noexcept {
            // Payload data starts after the MAVLink v2 header (10 bytes)
            return _backing_memory.data() + MessageDefinition::HEADER_SIZE;
        };

        [[nodiscard]] uint8_t getPayloadLength() const noexcept {
            return static_cast<uint8_t>(_message_definition->maxPayloadSize());
        };

        // Convenience method that matches the original interface
        [[nodiscard]] std::pair<const uint8_t*, size_t> getPayloadView() const noexcept {
            return {getPayloadData(), static_cast<size_t>(getPayloadLength())};
        };
    };

} // namespace mav

#endif //MAV_DYNAMICMESSAGE_H