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

#ifndef MAV_BUFFERPARSER_H
#define MAV_BUFFERPARSER_H

#include <optional>
#include <cstdint>
#include <cstddef>

namespace mav {

    // Forward declarations
    class Message;
    class MessageSet;

    /**
     * @brief Parser for extracting MAVLink messages from raw byte buffers
     * 
     * This class provides functionality to parse MAVLink v2 messages from
     * raw byte data without requiring network interface abstractions.
     * Based on the StreamParser from Network.h but simplified for buffer parsing.
     */
    class BufferParser {
    private:
        const MessageSet& _message_set;

    public:
        explicit BufferParser(const MessageSet& message_set) noexcept;

        /**
         * @brief Parse a single MAVLink message from a byte buffer
         * 
         * @param buffer Pointer to the buffer containing MAVLink data
         * @param buffer_size Size of the buffer in bytes
         * @param bytes_consumed Output parameter indicating how many bytes were consumed
         * @return Optional Message if parsing was successful, std::nullopt otherwise
         */
        [[nodiscard]] std::optional<Message> parseMessage(
            const uint8_t* buffer, 
            size_t buffer_size, 
            size_t& bytes_consumed) const noexcept;
    };

} // namespace mav

#endif // MAV_BUFFERPARSER_H