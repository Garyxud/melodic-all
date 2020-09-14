/*
 * EvologicsBridge.cpp
 *
 *  Created on: 18 nov. 2016
 *      Author: centelld
 */

#include <dccomms/DataLinkFrame.h>
#include <dccomms_utils/EvologicsBridge.h>
#include <string>

namespace dccomms_utils {

using namespace dccomms;

static PacketBuilderPtr pb =
    std::make_shared<DataLinkFramePacketBuilder>(DataLinkFrame::fcsType::crc16);

EvologicsBridge::EvologicsBridge(StreamCommsDevice *_device, int _baudrate)
    : CommsBridge(_device, pb, pb, _baudrate) {
  SetEndOfCmd("\n");
  _InitCommands();
  clusterSize = 30;
  remoteAddr = 0;
  localAddr = 1;
  _streamCommsDevice = _device;
}

EvologicsBridge::~EvologicsBridge() {
  // TODO Auto-generated destructor stub
}

void EvologicsBridge::SetLocalAddr(int _localAddr) { localAddr = _localAddr; }

void EvologicsBridge::SetRemoteAddr(int _remoteAddr) {
  remoteAddr = _remoteAddr;
}

void EvologicsBridge::SetClusterSize(int _clusterSize) {
  clusterSize = _clusterSize;
}

void EvologicsBridge::SetEndOfCmd(std::string _endOfCmd) {
  endOfCmd = _endOfCmd;
}

void EvologicsBridge::_SendInitCommands() {
  lockTransmission.lock();

  Utils::Sleep(2000);
  ClearTransmissionBuffer();
  Utils::Sleep(2000);

  std::string strClusterSize = ATZC + std::to_string(clusterSize) + endOfCmd;
  std::string strLocalAddr = ATALX + std::to_string(localAddr) + endOfCmd;
  std::string strRemoteAddr = std::to_string(remoteAddr) + endOfCmd;
  *_streamCommsDevice
      << AT0 << endOfCmd   // Ensure data mode
      << ATRP0 << endOfCmd // Disable promiscous mode
      << ATZU0
      << endOfCmd // Disable USBLLONG (will take effect only in USBL modem
      << ATZX0 << endOfCmd // Disable extended notifications
      << ATZC + std::to_string(clusterSize)
      << endOfCmd // Set cluster size (packets per packet train)
      << ATALX + std::to_string(localAddr) << endOfCmd; // Set local address

  Utils::Sleep(2000);
  ClearTransmissionBuffer();
  Utils::Sleep(2000);
  *_streamCommsDevice << ATARX + std::to_string(remoteAddr)
                      << endOfCmd // Set remote address
                      << ATKO0
                      << endOfCmd        // Keep online the acoustic connection
                      << ATD << endOfCmd // Establish a connection
                      << "+++AT!RI0" << endOfCmd << "+++AT!DW0" << endOfCmd
                      << "+++AT@ZL8096" << endOfCmd;

  Utils::Sleep(3000);

  *_streamCommsDevice << "+++AT?RP" << endOfCmd << "+++AT?ZU" << endOfCmd
                      << "+++AT?ZX" << endOfCmd << "+++AT?ZC" << endOfCmd
                      << "+++AT?AL" << endOfCmd << "+++AT?AR" << endOfCmd
                      << "+++AT?KO" << endOfCmd;

  Utils::Sleep(2000);

  lockTransmission.unlock();
}

void EvologicsBridge::_InitCommands() {
  AT0 = "ATO";         // Ensure data mode
  ATZ4 = "+++ATZ4";    // Clear transmission buffer
  ATRP0 = "+++AT!RP0"; // Disable promiscous mode
  ATZU0 = "+++AT@ZU0"; // Disable USBLLONG (will take effect only in USBL modem
  ATZX0 = "+++AT@ZX0"; // Disable extended notifications
  ATZC = "+++AT!ZC";   // Set cluster size
  ATALX = "+++AT!AL";  // Set local address
  ATARX = "+++AT!AR";  // Set remote address
  ATKO0 = "+++AT!KO0"; // Keep online the acoustic connection
  ATD = "+++ATD";      // Establish a connection
}

void EvologicsBridge::ClearTransmissionBuffer() {

  Log->warn("TX: clearing transmission buffer...");
  *_streamCommsDevice << ATZ4 << endOfCmd;
}

void EvologicsBridge::TxWork() {
  lockTransmission.lock();
  try {
    phyService.WaitForFramesFromRxFifo();
    phyService.SetPhyLayerState(CommsDeviceService::BUSY);
    do {
      phyService >> txpkt;
      Log->debug("TX: FIFO size: {}", phyService.GetRxFifoSize());

      if (txpkt->IsOk()) {
        // PACKET OK
        Log->debug("TX: frame is OK, ready to send");
        _TransmitPacket();
        unsigned int frameSize = txpkt->GetPacketSize();
        _frameTransmissionTime = ceil(frameSize * _byteTransmissionTime);
        Log->debug("frame transmission time: {}", _frameTransmissionTime);
        timer.Reset();
        unsigned int elapsed = 0;
        std::this_thread::sleep_for(
            std::chrono::milliseconds(_frameTransmissionTime));
        elapsed = timer.Elapsed();
        Log->debug("Tiempo transcurrido: " + std::to_string(elapsed));

        // ClearTransmissionBuffer();
      } else {
        // PACKET WITH ERRORS
        Log->critical("TX: INTERNAL ERROR: frame received with errors from the "
                      "upper layer!");
      }
    } while (phyService.GetRxFifoSize() > 0);
    phyService.SetPhyLayerState(CommsDeviceService::READY);

  } catch (CommsException &e) {
    std::string msg = e.what();
    switch (e.code) {
    case COMMS_EXCEPTION_LINEDOWN:
      Log->error("CONNECTION LOST WITH DEVICE WHEN WRITTING: " + msg);
      // TryToReconnect();
      break;
    }
  }
  lockTransmission.unlock();
}
bool EvologicsBridge::TryToConnect() {
  phyService.SetPhyLayerState(CommsDeviceService::BUSY);
  while (!connected) {
    try {
      device->Open();
      _SendInitCommands();
      connected = true;
    } catch (CommsException &e) {
      std::string msg = e.what();
      Log->error(
          "Problem happened when trying to connect with the comms device (" +
          msg + ")... Trying again...");
      Utils::Sleep(1000);
    }
  }
  phyService.SetPhyLayerState(CommsDeviceService::READY);
  return connected;
}

} /* namespace merbots */
