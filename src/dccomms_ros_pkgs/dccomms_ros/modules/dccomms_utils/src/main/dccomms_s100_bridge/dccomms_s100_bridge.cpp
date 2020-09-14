#include <cpputils/SignalManager.h>
#include <cstdio>
#include <dccomms/CommsBridge.h>
#include <dccomms/Utils.h>
#include <dccomms_packets/SimplePacket.h>
#include <dccomms_utils/S100Stream.h>
#include <iostream>

#include <cstdio>
#include <cxxopts.hpp>
#include <sys/types.h>

using namespace std;
using namespace dccomms;
using namespace dccomms_packets;
using namespace dccomms_utils;
using namespace cpputils;

LoggerPtr Log = cpplogging::CreateLogger("s100bridge");
CommsBridge *bridge;
S100Stream *stream;

int main(int argc, char **argv) {
  std::string modemPort;
  uint32_t modemBitrate = 9600, txPacketSize = 20, rxPacketSize = 20, portBaudrate = 9600;
  std::string dccommsId;
  std::string logLevelStr, logFile;
  bool flush = false, syncLog = false, hwFlowControlEnabled = false;
  enum PktType { DLF = 0, SP = 1};
  uint32_t txPktTypeInt = 1, rxPktTypeInt = 1;
  Log->Info("s100bridge Bridge");
  try {
    cxxopts::Options options("dccomms_utils/s100_bridge",
                             " - command line options");
    options.add_options()
        ("C,flow-control-enabled", "the flow control by hw is enabled in the modem", cxxopts::value<bool>(hwFlowControlEnabled))
        ("F,flush-log", "flush log", cxxopts::value<bool>(flush))
        ("s,sync-log", "sync-log (default: false -> async.)", cxxopts::value<bool>(syncLog))
        ("f,log-file", "File to save the log", cxxopts::value<std::string>(logFile)->default_value("")->implicit_value("bridge_log"))
        ("p,modem-port", "Modem's serial port", cxxopts::value<std::string>(modemPort)->default_value("/dev/ttyUSB0"))
        ("b, baud-rate", "Serial port baudrate (default: 9600)", cxxopts::value<uint32_t>(portBaudrate))
        ("l,log-level", "log level: critical,debug,err,info,off,trace,warn", cxxopts::value<std::string>(logLevelStr)->default_value("info"))
        ("B, modem-bitrate", "maximum bitrate (used when hw flow control is disabled)", cxxopts::value<uint32_t>(modemBitrate))
        ("help", "Print help")
        ("dccomms-id", "dccomms id for bridge", cxxopts::value<std::string>(dccommsId)->default_value("s100"))
        ("tx-packet-size", "transmitted SimplePacket size in bytes (=overhead+payload)", cxxopts::value<uint32_t>(txPacketSize))
        ("rx-packet-size", "received SimplePacket size in bytes (=overhead+payload)", cxxopts::value<uint32_t>(rxPacketSize))
        ("tx-packet-type", "0: DataLinkFrame, 1: SimplePacket (default).", cxxopts::value<uint32_t>(txPktTypeInt))
        ("rx-packet-type", "0: DataLinkFrame, 1: SimplePacket (default).", cxxopts::value<uint32_t>(rxPktTypeInt));

    auto result = options.parse(argc, argv);
    if (result.count("help")) {
      std::cout << options.help({""}) << std::endl;
      exit(0);
    }

  } catch (const cxxopts::OptionException &e) {
    std::cout << "error parsing options: " << e.what() << std::endl;
    exit(1);
  }
  PktType txPktType = static_cast<PktType>(txPktTypeInt);
  PktType rxPktType = static_cast<PktType>(rxPktTypeInt);

  PacketBuilderPtr rxpb, txpb;

  switch(txPktType){
    case DLF:{
        auto checksumType = DataLinkFrame::fcsType::crc16;
        txpb = CreateObject<DataLinkFramePacketBuilder>(checksumType);
        break;
    }
    case SP:{
        txpb = CreateObject<SimplePacketBuilder>(0, FCS::CRC16);
        auto pkt = txpb->Create();
        auto emptyPacketSize = pkt->GetPacketSize();
        auto payloadSize = txPacketSize - emptyPacketSize;
        txpb = CreateObject<SimplePacketBuilder>(payloadSize, FCS::CRC16);
        break;
    }
    default:
      std::cerr << "wrong tx packet type: "<< txPktType << std::endl;
      return 1;
  }
  switch(rxPktType){
    case DLF:{
        auto checksumType = DataLinkFrame::fcsType::crc16;
        rxpb = CreateObject<DataLinkFramePacketBuilder>(checksumType);
        break;
    }
    case SP:{
        rxpb = CreateObject<SimplePacketBuilder>(0, FCS::CRC16);
        auto pk = rxpb->Create();
        auto emptyPacketSize = pk->GetPacketSize();
        auto payloadSize = rxPacketSize - emptyPacketSize;
        rxpb = CreateObject<SimplePacketBuilder>(payloadSize, FCS::CRC16);
        break;
    }
    default:
      std::cerr << "wrong rx packet type: "<< rxPktType << std::endl;
      return 1;
  }
  Log->Info("dccommsId: {} ; port: {} ; baudrate: {} ; hw.FlowC: {}", dccommsId, modemPort, portBaudrate,
            hwFlowControlEnabled);

  LogLevel logLevel = cpplogging::GetLevelFromString(logLevelStr);
  auto baudrate = SerialPortStream::BaudRateFromUInt(portBaudrate);
  stream = new S100Stream(modemPort, baudrate, modemBitrate);
  stream->SetHwFlowControl(hwFlowControlEnabled);

  bridge = new CommsBridge(stream, txpb, rxpb, 0);

  bridge->SetLogLevel(logLevel);
  bridge->SetCommsDeviceId(dccommsId);
  bridge->SetLogName("S100Bridge");
  stream->SetLogName(bridge->GetLogName() + ":S100Stream");
  stream->SetLogLevel(logLevel);

  auto logFormatter = std::make_shared<spdlog::pattern_formatter>("%T.%F %v");
  stream->SetLogFormatter(logFormatter);
  bridge->SetLogFormatter(logFormatter);
  Log->SetLogFormatter(logFormatter);

  if (logFile != "") {
    Log->LogToFile(logFile);
    stream->LogToFile(logFile + "_stream");
    bridge->LogToFile(logFile + "_bridge");
  }
  if (flush) {
    Log->FlushLogOn(info);
    Log->Info("Flush log on info");
  }
  if (!syncLog) {
    Log->SetAsyncMode();
    Log->Info("Async. log");
  }

  bridge->SetReceivedPacketWithoutErrorsCb(
      [](PacketPtr pkt) { Log->Info("RX {}", pkt->GetPacketSize()); });
  bridge->SetReceivedPacketWithErrorsCb(
      [](PacketPtr pkt) { Log->Warn("ERR {}", pkt->GetPacketSize()); });
  bridge->SetTransmitingPacketCb(
      [](PacketPtr pkt) { Log->Info("TX {}", pkt->GetPacketSize()); });

  SignalManager::SetLastCallback(SIGINT, [&](int sig) {
    printf("Received %d signal\nClosing device socket...\n", sig);
    //bridge->Stop();
    printf("Device closed.\n");
    fflush(stdout);
    bridge->FlushLog();
    stream->FlushLog();
    Log->FlushLog();
    Utils::Sleep(2000);
    printf("Log messages flushed.\n");

    exit(0);
  });

  bridge->Start();
  while (1) {
    Utils::Sleep(1000);
  }
}
