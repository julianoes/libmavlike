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

#include "mav/BufferParser.h"
#include "mav/Message.h"
#include "mav/MessageSet.h"
#include "mav/MessageDefinition.h"
#include "mav/utils.h"
#include <cstring>
#include <array>

namespace mav {

    BufferParser::BufferParser(const MessageSet& message_set) noexcept : 
        _message_set(message_set) {}

    std::optional<Message> BufferParser::parseMessage(
        const uint8_t* buffer, 
        size_t buffer_size, 
        size_t& bytes_consumed) const noexcept {

        bytes_consumed = 0;

        // Look for MAVLink v2 magic byte
        size_t start_pos = 0;
        while (start_pos < buffer_size && buffer[start_pos] != 0xFD) {
            start_pos++;
        }

        if (start_pos >= buffer_size) {
            bytes_consumed = buffer_size; // Consumed all bytes looking for magic
            return std::nullopt;
        }

        // Check if we have enough bytes for complete header
        if (start_pos + MessageDefinition::HEADER_SIZE > buffer_size) {
            bytes_consumed = start_pos; // Consumed bytes before magic byte, not including it
            return std::nullopt;
        }

        // Parse header
        std::array<uint8_t, MessageDefinition::MAX_MESSAGE_SIZE> backing_memory{};
        std::memcpy(backing_memory.data(), buffer + start_pos, MessageDefinition::HEADER_SIZE);
        
        Header header{backing_memory.data()};
        const bool message_is_signed = header.isSigned();
        const int wire_length = MessageDefinition::HEADER_SIZE + header.len() + 
                              MessageDefinition::CHECKSUM_SIZE +
                              (message_is_signed ? MessageDefinition::SIGNATURE_SIZE : 0);

        // Check if we have the complete message
        if (start_pos + static_cast<size_t>(wire_length) > buffer_size) {
            bytes_consumed = start_pos; // Consumed bytes up to magic byte
            return std::nullopt;
        }

        // Copy the complete message
        std::memcpy(backing_memory.data(), buffer + start_pos, static_cast<size_t>(wire_length));
        
        const int crc_offset = MessageDefinition::HEADER_SIZE + header.len();

        // Get message definition
        auto definition_opt = _message_set.getMessageDefinition(header.msgId());
        if (!definition_opt) {
            // Unknown message, skip it
            bytes_consumed = start_pos + static_cast<size_t>(wire_length);
            return std::nullopt;
        }
        auto& definition = definition_opt.get();

        // Validate CRC
        CRC crc;
        crc.accumulate(backing_memory.begin() + 1, backing_memory.begin() + crc_offset);
        crc.accumulate(definition.crcExtra());
        auto crc_received = deserialize<uint16_t>(backing_memory.data() + crc_offset, sizeof(uint16_t));

        if (crc_received != crc.crc16()) {
            // CRC error, skip this message
            bytes_consumed = start_pos + static_cast<size_t>(wire_length);
            return std::nullopt;
        }

        // Create Message object
        // For BufferParser, we use a default ConnectionPartner since we don't have network info
        ConnectionPartner partner{0, 0, false}; // Default address=0, port=0, not UART
        bytes_consumed = start_pos + static_cast<size_t>(wire_length);
        
        return Message::_instantiateFromMemory(definition, partner, crc_offset, std::move(backing_memory));
    }

} // namespace mav