// The MIT License (MIT)
// 
// VectorNav SDK (v0.22.0)
// Copyright (c) 2024 VectorNav Technologies, LLC
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#ifndef VN_PACKETSYNCHRONIZER_HPP_
#define VN_PACKETSYNCHRONIZER_HPP_

#include <cstdint>
#include <functional>

#include "vectornav/Config.hpp"
#include "vectornav/Implementation/PacketDispatcher.hpp"
#include "vectornav/Interface/Errors.hpp"
#include "vectornav/TemplateLibrary/ByteBuffer.hpp"
#include "vectornav/TemplateLibrary/Vector.hpp"

namespace VN
{

constexpr uint8_t PACKET_PARSER_CAPACITY = Config::PacketFinders::maxNumPacketFinders;

class PacketSynchronizer
{
public:
    using AsyncErrorQueuePush = std::function<void(AsyncError&&)>;

    PacketSynchronizer(ByteBuffer& byteBuffer, AsyncErrorQueuePush asyncErrorQueuePush = nullptr,
                       size_t nominalSerialPush = Config::Serial::numBytesToReadPerGetData)
        : _primaryByteBuffer(byteBuffer), _asyncErrorQueuePush{asyncErrorQueuePush}, _nominalSerialPush(nominalSerialPush)
    {
    }
    bool addDispatcher(PacketDispatcher* packetParser) noexcept;

    bool dispatchNextPacket() noexcept;

    void registerSkippedByteBuffer(ByteBuffer* const skippedByteBuffer) noexcept { _pSkippedByteBuffer = skippedByteBuffer; };
    void deregisterSkippedByteBuffer() noexcept { _pSkippedByteBuffer = nullptr; };

    void registerReceivedByteBuffer(ByteBuffer* const receivedByteBuffer) noexcept { _pReceivedByteBuffer = receivedByteBuffer; }
    void deregisterReceivedByteBuffer() noexcept { _pReceivedByteBuffer = nullptr; }

    using SyncBytes = Vector<uint8_t, SYNC_BYTE_CAPACITY>;

    size_t getValidPacketCount(const SyncBytes& syncByte) const noexcept;
    size_t getInvalidPacketCount(const SyncBytes& syncByte) const noexcept;
    size_t getSkippedByteCount() const noexcept { return _skippedByteCount; }
    size_t getReceivedByteCount() const noexcept { return _receivedByteCount; }

private:
    struct InternalItem
    {
        PacketDispatcher* packetDispatcher = nullptr;
        SyncBytes syncBytes{};
        PacketDispatcher::FindPacketRetVal latestRetVal{};
        mutable size_t numValidPackets = 0;
        mutable size_t numInvalidPackets = 0;
    };

    void _copyToSkippedByteBufferIfEnabled(const size_t numBytesToCopy) const noexcept;
    void _copyToReceivedByteBufferIfEnabled(const size_t numBytesToCopy) const noexcept;

    Vector<InternalItem, PACKET_PARSER_CAPACITY> _dispatchers{};

    mutable uint64_t _skippedByteCount = 0;
    ByteBuffer* _pSkippedByteBuffer = nullptr;
    mutable uint64_t _receivedByteCount = 0;
    ByteBuffer* _pReceivedByteBuffer = nullptr;
    mutable std::array<uint8_t, Config::PacketFinders::skippedReceivedByteBufferMaxPutLength> _copySkippedReceivedLinearBuffer{};

    ByteBuffer& _primaryByteBuffer;
    uint64_t _prevByteBufferSize = 0;
    size_t _prevBytesRequested = 0;
    PacketDispatcher::FindPacketRetVal::Validity _prevValidity = PacketDispatcher::FindPacketRetVal::Validity::Invalid;
    AsyncErrorQueuePush _asyncErrorQueuePush = nullptr;
    const size_t _nominalSerialPush;
};
}  // namespace VN

#endif  // VN_PACKETSYNCHRONIZER_HPP_
