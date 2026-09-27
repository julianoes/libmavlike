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

#ifndef MAV_MESSAGEDEFINITION_H
#define MAV_MESSAGEDEFINITION_H

#include <map>
#include <vector>
#include <algorithm>

#include "utils.h"


namespace mav {

    constexpr int ANY_ID = -1;
    constexpr int LIBMAV_DEFAULT_ID = 97;

    // Error code enums for no-exceptions API
    enum class MessageSetResult {
        Success,
        MessageNotFound,
        EnumNotFound,
        XmlParseError,
        FileError
    };

    enum class MessageResult {
        Success,
        FieldNotFound,
        TypeMismatch,
        OutOfRange,
        InvalidData,
        NotFinalized
    };

    enum class ParseResult {
        Success,
        InvalidXml,
        FileNotFound,
        EnumParseError,
        FieldTypeError
    };


    struct ConnectionPartner {
        uint32_t _address;
        uint16_t _port;
        bool _is_uart;

    public:
        ConnectionPartner() = default;

        ConnectionPartner(uint32_t address_, uint16_t port_, bool is_uart_) :
        _address(address_), _port(port_), _is_uart(is_uart_) {}

        bool operator==(const ConnectionPartner& other) const noexcept {
            return _address == other._address && _port == other._port && _is_uart == other._is_uart;
        }

        bool operator!=(const ConnectionPartner& other) const noexcept {
            return !(*this == other);
        }

        [[nodiscard]] uint32_t address() const noexcept {
            return _address;
        }

        [[nodiscard]] uint16_t port() const noexcept {
            return _port;
        }

        [[nodiscard]] bool isUart() const noexcept {
            return _is_uart;
        }

        [[nodiscard]] bool isBroadcast() const noexcept {
            return _address == 0 && _port == 0;
        }

        [[nodiscard]] std::string toString() const;
    };

    struct _ConnectionPartnerHash {
        std::size_t operator()(const ConnectionPartner& k) const noexcept {
            return (k.isUart() << 17) | (k.address() << 16) | k.port();
        }
    };


    // MAVLink 2 incompat_flags
    constexpr uint8_t IFLAG_SIGNED = 0x01;
    // 4 byte sender system ID
    constexpr uint8_t IFLAG_SYSID32 = 0x02;
    // 4 byte target system ID appended to the header
    constexpr uint8_t IFLAG_TARGET32 = 0x04;
    constexpr uint8_t IFLAG_ALL_KNOWN = IFLAG_SIGNED | IFLAG_SYSID32 | IFLAG_TARGET32;

    // Including the magic byte
    constexpr int V2_BASE_HEADER_SIZE = 10;
    constexpr int V2_SYSID32_HEADER_EXTRA = 3;
    constexpr int V2_TARGET32_HEADER_EXTRA = 4;
    constexpr int V2_MAX_HEADER_SIZE =
            V2_BASE_HEADER_SIZE + V2_SYSID32_HEADER_EXTRA + V2_TARGET32_HEADER_EXTRA;

    // Payload target_system with IFLAG_TARGET32. 255 is still a valid system ID.
    constexpr uint8_t TARGET_SYSTEM_SENTINEL = 255;

    class Identifier {
    public:
        // 32 bit system IDs plus ANY_ID (-1)
        const int64_t system_id;
        const int64_t component_id;

        Identifier(int64_t system_id_, int64_t component_id_) : system_id(system_id_), component_id(component_id_) {}

        bool operator==(const Identifier &o) const noexcept {
            return system_id == o.system_id && component_id == o.component_id;
        }

        bool operator!=(const Identifier &o) const noexcept {
            return system_id != o.system_id || component_id != o.component_id;
        }

        bool filter(const Identifier &o) const noexcept {
            return (system_id == ANY_ID || system_id == o.system_id) && (component_id == ANY_ID || component_id == o.component_id);
        }
    };

    template <typename BackingMemoryPointerType>
    class Header {
    private:
        BackingMemoryPointerType _backing_memory;

        class _MsgId {
        private:
            BackingMemoryPointerType _ptr;
        public:
            explicit _MsgId(BackingMemoryPointerType ptr) : _ptr(ptr) {}

            operator int() const {
                // Not aligned, and the offset moves with IFLAG_SYSID32.
                return static_cast<int>(
                        static_cast<uint32_t>(_ptr[0]) |
                        (static_cast<uint32_t>(_ptr[1]) << 8) |
                        (static_cast<uint32_t>(_ptr[2]) << 16));
            }

            _MsgId& operator=(int v) {
                _ptr[0] = static_cast<uint8_t>(v & 0xFF);
                _ptr[1] = static_cast<uint8_t>((v >> 8) & 0xFF);
                _ptr[2] = static_cast<uint8_t>((v >> 16) & 0xFF);
                return *this;
            }
        };

    public:
        explicit Header(BackingMemoryPointerType backing_memory) : _backing_memory(backing_memory) {}

        inline uint8_t& magic(){
            return _backing_memory[0];
        }

        [[nodiscard]] inline uint8_t magic() const {
            return _backing_memory[0];
        }

        inline uint8_t& len() {
            return _backing_memory[1];
        }

        [[nodiscard]] inline uint8_t len() const {
            return _backing_memory[1];
        }

        inline uint8_t& incompatFlags() {
            return _backing_memory[2];
        }

        [[nodiscard]] inline uint8_t incompatFlags() const {
            return _backing_memory[2];
        }

        [[nodiscard]] inline bool isSigned() const {
            return incompatFlags() & 0x01;
        }

        inline uint8_t& compatFlags() {
            return _backing_memory[3];
        }

        [[nodiscard]] inline uint8_t compatFlags() const {
            return _backing_memory[3];
        }

        inline uint8_t& seq() {
            return _backing_memory[4];
        }

        [[nodiscard]] inline uint8_t seq() const {
            return _backing_memory[4];
        }

        [[nodiscard]] inline bool hasWideSystemId() const {
            return (incompatFlags() & IFLAG_SYSID32) != 0;
        }

        [[nodiscard]] inline bool hasWideTarget() const {
            return (incompatFlags() & IFLAG_TARGET32) != 0;
        }

        [[nodiscard]] inline int systemIdSize() const {
            return hasWideSystemId() ? 4 : 1;
        }

        // Including the magic byte
        [[nodiscard]] inline int size() const {
            return V2_BASE_HEADER_SIZE + (hasWideSystemId() ? V2_SYSID32_HEADER_EXTRA : 0) +
                   (hasWideTarget() ? V2_TARGET32_HEADER_EXTRA : 0);
        }

        [[nodiscard]] inline uint32_t systemId() const {
            if (!hasWideSystemId()) {
                return _backing_memory[5];
            }
            return static_cast<uint32_t>(_backing_memory[5]) |
                   (static_cast<uint32_t>(_backing_memory[6]) << 8) |
                   (static_cast<uint32_t>(_backing_memory[7]) << 16) |
                   (static_cast<uint32_t>(_backing_memory[8]) << 24);
        }

        // Requires IFLAG_SYSID32 to already be set as needed.
        inline void setSystemId(uint32_t v) {
            _backing_memory[5] = static_cast<uint8_t>(v & 0xFF);
            if (hasWideSystemId()) {
                _backing_memory[6] = static_cast<uint8_t>((v >> 8) & 0xFF);
                _backing_memory[7] = static_cast<uint8_t>((v >> 16) & 0xFF);
                _backing_memory[8] = static_cast<uint8_t>((v >> 24) & 0xFF);
            }
        }

        [[nodiscard]] inline uint8_t componentId() const {
            return _backing_memory[5 + systemIdSize()];
        }

        inline void setComponentId(uint8_t v) {
            _backing_memory[5 + systemIdSize()] = v;
        }

        inline _MsgId msgId() {
            return _MsgId(_backing_memory + 6 + systemIdSize());
        }

        [[nodiscard]] inline _MsgId msgId() const {
            return _MsgId(_backing_memory + 6 + systemIdSize());
        }

        [[nodiscard]] inline uint32_t targetSystemId() const {
            if (!hasWideTarget()) {
                return 0;
            }
            const auto ofs = 9 + systemIdSize();
            return static_cast<uint32_t>(_backing_memory[ofs]) |
                   (static_cast<uint32_t>(_backing_memory[ofs + 1]) << 8) |
                   (static_cast<uint32_t>(_backing_memory[ofs + 2]) << 16) |
                   (static_cast<uint32_t>(_backing_memory[ofs + 3]) << 24);
        }

        // Requires IFLAG_TARGET32 to already be set.
        inline void setTargetSystemId(uint32_t system_id) {
            const auto ofs = 9 + systemIdSize();
            _backing_memory[ofs] = static_cast<uint8_t>(system_id & 0xFF);
            _backing_memory[ofs + 1] = static_cast<uint8_t>((system_id >> 8) & 0xFF);
            _backing_memory[ofs + 2] = static_cast<uint8_t>((system_id >> 16) & 0xFF);
            _backing_memory[ofs + 3] = static_cast<uint8_t>((system_id >> 24) & 0xFF);
        }

        [[nodiscard]] inline Identifier source() const {
            return {systemId(), componentId()};
        }
    };

    template <typename BackingMemoryPointerType>
    class Signature {
      private:
        BackingMemoryPointerType _backing_memory;

        // Both timestamp and signature use 6-byte fields
        class _SignatureField {
          private:
            BackingMemoryPointerType _ptr;
          public:
            explicit _SignatureField(BackingMemoryPointerType ptr) : _ptr(ptr) {}

            operator uint64_t() const {
                return (*static_cast<const uint64_t*>(static_cast<const void*>(_ptr))) & 0xFFFFFFFFFFFF;
            }

            _SignatureField& operator=(uint64_t v) {
                _ptr[0] = static_cast<uint8_t>(v & 0xFF);
                _ptr[1] = static_cast<uint8_t>((v >> 8) & 0xFF);
                _ptr[2] = static_cast<uint8_t>((v >> 16) & 0xFF);
                _ptr[3] = static_cast<uint8_t>((v >> 24) & 0xFF);
                _ptr[4] = static_cast<uint8_t>((v >> 32) & 0xFF);
                _ptr[5] = static_cast<uint8_t>((v >> 40) & 0xFF);
                return *this;
            }
        };

      public:
        explicit Signature(BackingMemoryPointerType backing_memory) : _backing_memory(backing_memory) {}

        inline uint8_t& linkId(){
            return _backing_memory[0];
        }

        [[nodiscard]] inline uint8_t linkId() const {
            return _backing_memory[0];
        }

        inline _SignatureField timestamp() {
            return _SignatureField(_backing_memory + 1);
        }

        [[nodiscard]] inline const _SignatureField timestamp() const {
            return _SignatureField(_backing_memory + 1);
        }

        inline _SignatureField signature() {
            return _SignatureField(_backing_memory + 7);
        }

        [[nodiscard]] inline const _SignatureField signature() const {
            return _SignatureField(_backing_memory + 7);
        }
    };

    class FieldType {
    public:
        enum class BaseType {
            CHAR,
            UINT8,
            UINT16,
            UINT32,
            UINT64,
            INT8,
            INT16,
            INT32,
            INT64,
            FLOAT,
            DOUBLE
        };

        BaseType base_type;
        int size;

        [[nodiscard]] int baseSize() const noexcept;

        [[nodiscard]] const char* crcNameString() const noexcept;


        FieldType(FieldType::BaseType base_type_, int size_) : base_type(base_type_), size(size_) {}
    };

    struct Field {
        FieldType type;
        int offset;
    };

    // Forward declared builder class
    class MessageDefinitionBuilder;


    class MessageDefinition {
        friend MessageDefinitionBuilder;
    private:
        std::string _name;
        int _id;
        int _max_buffer_length;
        int _max_payload_length;
        int _not_extended_payload_length;
        uint8_t _crc_extra = 0;
        std::map<std::string, Field> _fields;


        MessageDefinition(const std::string &name, int id) : _name(name), _id(id)
        {}

    public:
        static constexpr int MAX_PAYLOAD_SIZE = 255;
        // Without extensions, see Header::size().
        static constexpr int HEADER_SIZE = V2_BASE_HEADER_SIZE;
        static constexpr int MAX_HEADER_SIZE = V2_MAX_HEADER_SIZE;
        static constexpr int CHECKSUM_SIZE = 2;
        static constexpr int SIGNATURE_LINK_ID_SIZE = 1;
        static constexpr int SIGNATURE_TIMESTAMP_SIZE = 6;
        static constexpr int SIGNATURE_SIGNATURE_SIZE = 6;
        static constexpr int SIGNATURE_SIZE = SIGNATURE_LINK_ID_SIZE + SIGNATURE_TIMESTAMP_SIZE + SIGNATURE_SIGNATURE_SIZE;
        static constexpr int MAX_MESSAGE_SIZE = MAX_PAYLOAD_SIZE + MAX_HEADER_SIZE + CHECKSUM_SIZE + SIGNATURE_SIZE;
        static constexpr int KEY_SIZE = 32;

        [[nodiscard]] inline const std::string& name() const noexcept {
            return _name;
        }

        [[nodiscard]] inline int id() const noexcept {
            return _id;
        }

        [[nodiscard]] inline int maxBufferLength() const noexcept {
            return _max_buffer_length;
        }

        [[nodiscard]] inline int maxPayloadSize() const noexcept {
            return _max_payload_length;
        }

        [[nodiscard]] inline int notExtendedPayloadSize() const noexcept {
            return _not_extended_payload_length;
        }

        [[nodiscard]] inline uint8_t crcExtra() const noexcept {
            return _crc_extra;
        }

        [[nodiscard]] std::optional<Field> getField(const std::string &field_key) const noexcept {
            auto it = _fields.find(field_key);
            if (it == _fields.end()) {
                return std::nullopt;
            }
            return it->second;
        }

        [[nodiscard]] bool containsField(const std::string &field_key) const noexcept {
            auto it = _fields.find(field_key);
            if (it == _fields.end()) {
                return false;
            }
            return true;
        }

        [[nodiscard]] const std::map<std::string, Field>& fieldDefinitions() const noexcept {
            return _fields;
        }

        [[nodiscard]] std::vector<std::string> fieldNames() const;
    };


    class MessageDefinitionBuilder {
    private:

        struct NamedField {
            std::string name;
            FieldType type;
        };

        MessageDefinition _result;

        std::vector<NamedField> _fields;
        std::vector<NamedField> _extension_fields;

    public:
        MessageDefinitionBuilder(const std::string &name, int id) : _result(name, id) {}

        MessageDefinitionBuilder& addField(const std::string &name, const FieldType &type) {
            _fields.emplace_back(NamedField{name, type});
            return *this;
        }

        MessageDefinitionBuilder& addExtensionField(const std::string &name, const FieldType &type) {
            _extension_fields.emplace_back(NamedField{name, type});
            return *this;
        }

        MessageDefinition build();
    };
}

#endif //MAV_MESSAGEDEFINITION_H
