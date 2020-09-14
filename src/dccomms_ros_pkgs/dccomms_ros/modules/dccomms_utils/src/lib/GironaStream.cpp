/*
 * GironaStream.cpp
 *
 *  Created on: 16 nov. 2016
 *      Author: centelld
 */

#include <cstdint>
#include <dccomms/DataLinkFrame.h>
#include <dccomms_utils/GironaStream.h>
#include <string>

namespace dccomms_utils {

GironaStream::GironaStream(std::string serialportname,
                           SerialPortStream::BaudRate baudrate)
    : SerialPortStream(serialportname.c_str(), baudrate),
      pbmregex("^RECVPBM,(\\d+),(\\d+),(\\d+),(\\d+(?:\\.\\d+)?),(-(?:\\d+(?:"
               "\\.\\d+)?)),(\\d+(?:\\.\\d+)?),(\\d+(?:\\.\\d+)?)(,)") {
  // TODO Auto-generated constructor stub
  init();
  dlf = DataLinkFrame::BuildDataLinkFrame(DataLinkFrame::fcsType::crc16);
}

GironaStream::~GironaStream() {
  // TODO Auto-generated destructor stub
}

void GironaStream::init() {
  recPbmHeader = "RECVPBM,";
  recPbmHeaderLength = recPbmHeader.length();
}

int GironaStream::_Recv(void *dbuf, int n, bool block) {
  return Read((unsigned char *)dbuf, n, (unsigned int)block);
}

void GironaStream::ReadPacket(const PacketPtr &pkt) {
  bool receivedPacket = false;

  while (!receivedPacket) {
    // Example of pbm notification:
    //+++AT:47:RECVPBM,10,1,2,311411,-17,536,0.0307,1234567890
    int notificationPayloadLength;
    uint16_t payloadLength;
    uint8_t sourceAddr, dstAddr;
    float duration, rssi, integrity, velocity;
    char nextByte;
    std::cmatch cm;

    WaitFor((uint8_t *)bes, BES_LENGTH);

    // READ THE ENTIRE NOTIFICATION
    int notNameLength = ReadUntil((uint8_t *)notification, (uint8_t *)":", 1,
                                  MAX_CMDNAME_LENGTH) -
                        1;
    char *notLengthPtr = notification + notNameLength + 1;

    int notLengthLength = ReadUntil((uint8_t *)notLengthPtr, (uint8_t *)":", 1,
                                    MAX_NOTIFICATION_LENGHTFIELD_LENGTH) -
                          1;

    try {
      notLengthPtr[notLengthLength] = 0;
      notificationPayloadLength = std::stoi(notLengthPtr);
      notLengthPtr[notLengthLength] = ':';
    } catch (std::exception e) {
      throw CommsException(std::string("Expected integer") +
                               std::string(e.what()),
                           COMMS_EXCEPTION_UNKNOWN_ERROR);
    }

    notificationPayload = notLengthPtr + notLengthLength + 1;
    Read(notificationPayload, notificationPayloadLength); // blocking call
    // NOTIFICATION IN MEMORY

    // NOW, WE CHECK IF IT IS A SIMPLE NOTIFICATION OR A RESPONSE
    if (notNameLength == 2 && *notification == 'A' &&
        *(notification + 1) == 'T') {
      // SIMPLE NOTIFICATION

      // IS IT A PBM MESSAGE?
      bool matched = std::regex_search(notificationPayload, cm, pbmregex);
      if (matched) {
        // pbm format:
        // RECVPBM,<length>,<source address>,<destination address>,
        //<duration>,<rssi>,<integrity>,<velocity>,<data>
        Log->debug("Received pbm message: {}", cm[0].str());
        payloadLength = atoi(notificationPayload + cm.position(1));
        sourceAddr = atoi(notificationPayload + cm.position(2));
        dstAddr = atoi(notificationPayload + cm.position(3));
        duration = atof(notificationPayload + cm.position(4));
        rssi = atof(notificationPayload + cm.position(5));
        integrity = atof(notificationPayload + cm.position(6));
        velocity = atof(notificationPayload + cm.position(7));
        pbmData = (uint8_t *)notificationPayload + cm.position(8) + 1;
        Log->debug("Received PBM:\n"
                   "\tpayload length:\t{}\n"
                   "\tsource address:\t{}\n"
                   "\tdestination address:\t{}\n"
                   "\tduration:\t{}\n"
                   "\tRSSI:\t{}\n"
                   "\tintegrity:\t{}\n"
                   "\tvelocity:\t{}\n",
                   payloadLength, sourceAddr, dstAddr, duration, rssi,
                   integrity, velocity);

        dlf->UpdateFrame(dstAddr, sourceAddr, payloadLength, pbmData);
        pkt->CopyFromRawBuffer(dlf->GetBuffer());
        receivedPacket = true;
      }
    }
    notificationPayload[notificationPayloadLength] = 0;
    std::string notificationStr = std::string(notification);
    Log->info("Received notification from modem: {}", notificationStr);
    notificationReceivedCallback(notificationStr);
  }
}

} /* namespace merbots */
