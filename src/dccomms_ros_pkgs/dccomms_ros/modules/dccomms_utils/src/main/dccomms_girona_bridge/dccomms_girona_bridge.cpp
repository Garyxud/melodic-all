#include <cstdio>
#include <dccomms/Utils.h>
#include <dccomms_utils/EvologicsBridge.h>
#include <dccomms_utils/GironaStream.h>
#include <iostream>
#include <cpputils/SignalManager.h>

#include <cstdio>
#include <sys/types.h>

using namespace std;
using namespace dccomms;

using namespace dccomms_utils;
using namespace cpputils;

EvologicsBridge *bridge;
GironaStream *stream;

int main(int argc, char **argv) {
  int maxDataRate = std::stoi(argv[1]);
  const char *serialportname = argv[2];
  int localAddr = std::stoi(argv[3]);
  int remoteAddr = std::stoi(argv[4]);

  stream = new GironaStream(serialportname, SerialPortStream::BAUD_19200);
  bridge = new EvologicsBridge(stream, maxDataRate);

  SignalManager::SetLastCallback(SIGINT, [&](int signal)
  {
      printf("Received signal %d.\nClosing device...\n", signal);
      bridge->FlushLog();
      printf("Device closed.\n");
      fflush(stdout);
      bridge->FlushLog();
      stream->FlushLog();
      Utils::Sleep(2000);
      printf("Log messages flushed.\n");
      exit(0);
    });

  bridge->SetLogLevel(cpplogging::LogLevel::debug);
  bridge->SetCommsDeviceId("camera");
  bridge->SetLogName("ROVBridge");
  stream->SetLogName(bridge->GetLogName() + ":ROVStream");
  bridge->SetEndOfCmd("\r");

  bridge->SetLocalAddr(localAddr);   // 2
  bridge->SetRemoteAddr(remoteAddr); // 1

  std::cout << "Local add: " << localAddr << endl;
  std::cout << "Remote add: " << remoteAddr << endl;
  bridge->SetClusterSize(atoi(argv[3]));

  bridge->LogToFile("rov_comms_bridge_log");
  stream->LogToFile("rov_comms_bridge_device_log");

  bridge->Start();
  while (1) {
    Utils::Sleep(1000);
  }
}
