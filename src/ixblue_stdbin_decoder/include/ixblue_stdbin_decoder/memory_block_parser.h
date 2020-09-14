#pragma once

#include "ixblue_stdbin_decoder/data_models/stdbin.h"
#include <bitset>
#include <boost/asio/buffer.hpp>
#include <boost/core/noncopyable.hpp>
#include <boost/optional.hpp>
#include <memory>

namespace ixblue_stdbin_decoder
{

template<size_t N>
boost::asio::const_buffer& operator>>(boost::asio::const_buffer& buf,
                                      std::array<uint8_t, N>& res)
{
    for(auto& b : res)
    {
        const auto byte = boost::asio::buffer(buf, 1);
        b = *boost::asio::buffer_cast<const uint8_t*>(byte);
        buf = buf + sizeof(uint8_t);
    }
    return buf;
}
boost::asio::const_buffer& operator>>(boost::asio::const_buffer& buf, double& res);
boost::asio::const_buffer& operator>>(boost::asio::const_buffer& buf, float& res);
boost::asio::const_buffer& operator>>(boost::asio::const_buffer& buf, int32_t& res);
boost::asio::const_buffer& operator>>(boost::asio::const_buffer& buf, uint64_t& res);
boost::asio::const_buffer& operator>>(boost::asio::const_buffer& buf, uint32_t& res);
boost::asio::const_buffer& operator>>(boost::asio::const_buffer& buf, uint16_t& res);
boost::asio::const_buffer& operator>>(boost::asio::const_buffer& buf, uint8_t& res);

class MemoryBlockParser : private boost::noncopyable
{
    MemoryBlockParser() = delete; // ensure that offsetInMask will always be set.
public:
    typedef std::bitset<32> tBitMask;
    MemoryBlockParser(size_t offsetInMask, size_t bytesCount)
        : offsetInMask(offsetInMask), bytesCount(bytesCount)
    {}

    void parse(boost::asio::const_buffer& buffer, const tBitMask& mask,
               Data::BinaryNav& outBinaryNav);

    virtual void parse(boost::asio::const_buffer& buffer,
                       Data::BinaryNav& outBinaryNav) = 0;
    size_t getOffsetInMask(void) const { return offsetInMask; }
    size_t getBytesCount(void) const { return bytesCount; }

protected:
    const size_t offsetInMask;
    const size_t bytesCount;
};

typedef std::shared_ptr<MemoryBlockParser> MemoryBlockParserPtr;
} // namespace ixblue_stdbin_decoder
