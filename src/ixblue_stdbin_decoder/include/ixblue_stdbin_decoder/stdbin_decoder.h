#pragma once

#include "data_models/nav_header.h"
#include "data_models/stdbin.h"
#include "memory_block_parser.h"

#include <boost/noncopyable.hpp>
#include <functional>
#include <set>

namespace ixblue_stdbin_decoder
{

/*!
 * \brief Parser of a STDBIN IXblue message.
 * This is the entry point of the library. Usage of this class is as follow :
 * \code
 * StdBinDecoder parser;
 * ///...
 * try{
 *   if(parser.parse(buffer)) {
 *     auto navDatas = parser.getLastMessage();
 *   }
 * }catch(std::runtime_error& e){
 *   // Parsing error are reported by throwing std::runtime_exception.
 * }
 * \endcode
 * parse method can be called with a partial buffer. Data will be agglomerated until the
 * full frame have been received.
 */
class StdBinDecoder : private boost::noncopyable
{
    // we sort the parsers list by offsetMask because this is a design constraint. We need
    // to process parser in the same order than data are filled in memory.
    typedef std::set<
        MemoryBlockParserPtr,
        std::function<bool(const MemoryBlockParserPtr&, const MemoryBlockParserPtr&)>>
        tParsersSet;

    static constexpr size_t HEADER_SIZE_V2 = 21;
    static constexpr size_t HEADER_SIZE_V3 = 25;
    static constexpr size_t HEADER_SIZE_V4 = 27;
    static constexpr size_t HEADER_SIZE_V5 = 27;
    static constexpr size_t ANSWER_HEADER_SIZE = 5;
    static constexpr size_t CHECKSUM_SIZE = 4;

public:
    StdBinDecoder();

    /*!
     * \arg frameData : A part or the full StdBIN frame. The first buffer passed to this
     * function must be the begining of a frame. This will be true if buffer is received
     * via UDP or TCP.
     * \warning This method doesn't work if the STDBIN frame is received via Serial
     * (RS232) link.
     * \return true if the frame has been completly parsed, false otherwise.
     * If frame has been parsed, result is accessible via \c getLastNavData();
     * \exception runtime_error if a parse error occurs.
     */
    bool parse(const std::vector<uint8_t>& frameData);

    Data::BinaryNav getLastNavData(void) const { return lastParsed; }
    Data::NavHeader getLastHeaderData(void) const { return lastHeader; }
    const std::vector<uint8_t>& getLastAnswerData(void) const { return lastAnswer; }

protected:
    /*!
     * \exception runtime_error if a parse error occurs.
     */
    Data::NavHeader parseHeader(boost::asio::const_buffer& buffer) const;
    Data::NavHeader::MessageType getHeaderType(boost::asio::const_buffer& buffer) const;
    bool haveEnoughByteToParseHeader(const std::vector<uint8_t>& frame) const;
    // We set the parsers set "constant" to be sure that the content of this set will be
    // the same during all the lifetime of this object. We can only add memory bloc parser
    // at construction.
    const tParsersSet navigationParsers;
    const tParsersSet extendedNavigationParsers;
    const tParsersSet externalDataParsers;

    Data::BinaryNav lastParsed;
    Data::NavHeader lastHeader;
    std::vector<uint8_t> lastAnswer;

    // We store in this buffer the current frame's data. This memory chunk is managed by
    // the parsing state machine. See function \c parse.
    std::vector<uint8_t> currentFrame;
};
} // namespace ixblue_stdbin_decoder
