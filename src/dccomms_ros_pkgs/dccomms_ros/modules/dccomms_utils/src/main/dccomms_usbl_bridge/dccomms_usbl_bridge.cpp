#include <cstdio>
#include <dccomms/Utils.h>
#include <dccomms_utils/EvologicsBridge.h>
#include <dccomms_utils/USBLStream.h>
#include <iostream>
#include <cpputils/SignalManager.h>

#include <cstdio>
#include <sys/types.h>

using namespace std;
using namespace dccomms;
using namespace dccomms_utils;
using namespace cpputils;

EvologicsBridge *bridge;
USBLStream *stream;

int main(int argc, char **argv) {
  int baudrate = atoi(argv[1]);
  const char *devicedir = argv[2];

  stream = new USBLStream(devicedir);
  bridge = new EvologicsBridge(stream, baudrate);

  bridge->SetLogLevel(cpplogging::LogLevel::debug);
  bridge->SetCommsDeviceId("operator");
  bridge->SetLogName("OperatorBridge");
  stream->SetLogName(bridge->GetLogName() + ":USBLStream");
  bridge->SetEndOfCmd("\n");

  bridge->SetLocalAddr(1);
  bridge->SetRemoteAddr(3);
  bridge->SetClusterSize(atoi(argv[3]));

  bridge->LogToFile("usbl_comms_bridge_log");
  stream->LogToFile("usbl_comms_bridge_device_log");

  SignalManager::SetLastCallback(SIGINT, [&](int sig)
  {
      printf("Received %d signal\nClosing device socket...\n", sig);
      bridge->Stop();
      printf("Device closed.\n");
      fflush(stdout);
      bridge->FlushLog();
      stream->FlushLog();
      Utils::Sleep(2000);
      printf("Log messages flushed.\n");

      exit(0);
  });

  bridge->Start();
  while (1) {
    Utils::Sleep(1000);
  }
}
