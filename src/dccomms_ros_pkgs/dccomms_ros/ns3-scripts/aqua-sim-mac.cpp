/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2017 University of Connecticut
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Robert Martin <robert.martin@engr.uconn.edu>
 */

#include "ns3/applications-module.h"
#include "ns3/aqua-sim-ng-module.h"
#include "ns3/callback.h"
#include "ns3/core-module.h"
#include "ns3/energy-module.h" //may not be needed here...
#include "ns3/log.h"
#include "ns3/mobility-module.h"
#include "ns3/netanim-module.h"
#include "ns3/network-module.h"
#include <cpplogging/cpplogging.h>
#include <fstream>
#include <thread>
/*
 * BroadCastMAC
 *
 * N ---->  N  -----> N -----> N* -----> S
 *
 */

#include <chrono>
#include <ctime>
#include <iomanip>
#include <iostream>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("MAComp");

class Test : virtual public cpplogging::Logger {
public:
  Test();
  void RunTest(int argc, char **argv);
  void AppPacketTx(std::string context, Ptr<const Packet>);
  void AppPacketRx(std::string context, Ptr<const Packet>);
  void MacPacketTx(std::string context, Ptr<const Packet>);
  void MacPacketRx(std::string context, Ptr<const Packet>);
  void RoutingPacketRx(std::string context, Ptr<const Packet>);
  void CourseChange(std::string context, Ptr<const MobilityModel> model);
  void GetSimTime(const char *format, std::string &datetime,
                  double &secsFromStart);
  void SetSimulationStartDateTime();
  void SendPacket(ns3::Ptr<NetDevice> &dev, Address &addr, int pktSize) {
    AquaSimAddress srcAdd, dstAdd;
    srcAdd = AquaSimAddress::ConvertFrom(dev->GetAddress());
    dstAdd = AquaSimAddress::ConvertFrom(addr);
    Info("######################## SEND DATA FROM {} TO {}: {} bytes", srcAdd.GetAsInt(),
         dstAdd.GetAsInt(), pktSize);
    auto pkt = ns3::Create<ns3::Packet>(pktSize);
    dev->Send(pkt, addr, 0);
  }

private:
  cpplogging::Logger routingLog, macLog, phyLog, mobilityLog;
  const char timeFormat[100] = "%Y-%m-%d %H:%M:%S";
  std::chrono::high_resolution_clock::time_point start;
};

Test::Test() {
  SetLogName("Aqua-Sim test 0");
  routingLog.SetLogName("Routing");
  macLog.SetLogName("Mac");
  phyLog.SetLogName("Phy");
  mobilityLog.SetLogName("Mobility");

  cpplogging::LogLevel level = cpplogging::LogLevel::debug;

  SetLogLevel(level);
  routingLog.SetLogLevel(level);
  macLog.SetLogLevel(level);
  phyLog.SetLogLevel(level);
  mobilityLog.SetLogLevel(level);
  LogToConsole(true);
}

void Test::SetSimulationStartDateTime() {
  start = std::chrono::high_resolution_clock::now();
}
void Test::GetSimTime(const char *format, std::string &datetime,
                      double &secsFromStart) {
  auto simTime = Simulator::Now();
  secsFromStart = simTime.GetSeconds();
  auto tstamp = simTime.GetTimeStep(); // nanoseconds

  auto simNanos = std::chrono::nanoseconds(tstamp);
  auto curpoint = start + simNanos;
  auto t = std::chrono::high_resolution_clock::to_time_t(curpoint);
  auto localEventTime = std::localtime(&t);
  char mbstr[100];
  auto count = std::strftime(mbstr, sizeof(mbstr), format, localEventTime);
  char *mp = mbstr + count;

  auto durationFromEpoch = curpoint.time_since_epoch();
  auto millis =
      std::chrono::duration_cast<std::chrono::milliseconds>(durationFromEpoch) -
      std::chrono::duration_cast<std::chrono::seconds>(durationFromEpoch);
  sprintf(mp, ".%ld", millis.count());
  datetime = mbstr;
}

void Test::AppPacketTx(std::string context, Ptr<const Packet> pkt) {
  AquaSimHeader ash;
  pkt->PeekHeader(ash);
  auto saddr = ash.GetSAddr().GetAsInt();
  auto daddr = ash.GetDAddr().GetAsInt();
  auto nhaddr = ash.GetNextHop().GetAsInt();
  auto size = ash.GetSize();

  std::string datetime;
  double secs;
  GetSimTime(timeFormat, datetime, secs);

  routingLog.Info(
      "({} secs; {}) {}: (Addr: {}) Transmitting packet to routing layer {}. "
      "Next hop: {} ; {} bytes",
      secs, datetime, context, saddr, daddr, nhaddr, size);
}

void Test::AppPacketRx(std::string context, Ptr<const Packet> pkt) {
  AquaSimHeader ash;
  pkt->PeekHeader(ash);
  auto saddr = ash.GetSAddr().GetAsInt();
  auto daddr = ash.GetDAddr().GetAsInt();
  auto size = ash.GetSize();

  std::string datetime;
  double secs;
  GetSimTime(timeFormat, datetime, secs);

  routingLog.Info("({} secs; {}) {}: (Addr: {}) Received packet from routing "
                  "layer{} ; {} bytes",
                  secs, datetime, context, daddr, saddr, size);
}

void Test::RoutingPacketRx(std::string context, Ptr<const Packet> pkt) {
  std::string datetime;
  double secs;
  GetSimTime(timeFormat, datetime, secs);
  AquaSimHeader ash;
  Ptr<Packet> cpkt = pkt->Copy();
  cpkt->RemoveHeader(ash);

  routingLog.Info("({} secs; {}) {}: Received packet from mac layer; {} buffer "
                  "bytes ; {} real bytes",
                  secs, datetime, context, cpkt->GetSize(), ash.GetSize());
}

void Test::MacPacketTx(std::string context, Ptr<const Packet> pkt) {
  std::string datetime;
  double secs;
  GetSimTime(timeFormat, datetime, secs);

  routingLog.Info("({} secs; {}) {}: Transmitting packet to phy {} bytes", secs,
                  datetime, context, pkt->GetSize());
}

void Test::MacPacketRx(std::string context, Ptr<const Packet> pkt) {
  std::string datetime;
  double secs;
  GetSimTime(timeFormat, datetime, secs);

  routingLog.Info("({} secs; {}) {}: Received packet from phy layer; {} bytes",
                  secs, datetime, context, pkt->GetSize());
}

void Test::CourseChange(std::string context, Ptr<const MobilityModel> model) {
  Vector position = model->GetPosition();

  std::string datetime;
  double secs;
  GetSimTime(timeFormat, datetime, secs);
  mobilityLog.Info("({} secs; {}) {}: [x,y,z] = [{},{},{}]", secs, datetime,
                   context, position.x, position.y, position.z);
}

void Test::RunTest(int argc, char **argv) {
  int nodes = 3;
  double range = 500;
  uint devBitRate = 900;
  uint dataPktSize = 50;
  double distance = 20;
  bool debugMac = false;
  bool debugChannel = false;
  bool debugPhy = false;
  int numPkts = 1;
  std::string macProtName = "ns3::AquaSimBroadcastMac";

  std::string asciiTraceFile = "mac-test.asc";

  CommandLine cmd;
  cmd.AddValue("nodes", "Number of nodes", nodes);
  cmd.AddValue("range", "Range of each node", range);
  cmd.AddValue("distance", "Distance between nodes", distance);
  cmd.AddValue("dev-bitrate", "Acoustic modems's bitrate", devBitRate);
  cmd.AddValue("num-pkts", "Number of packets transmitted in every burst", numPkts);
  cmd.AddValue("debug-mac", "Show debg messages in mac layer", debugMac);
  cmd.AddValue("debug-phy", "Show debg messages in mac layer", debugPhy);
  cmd.AddValue("debug-channel", "Show debg messages in mac layer",
               debugChannel);
  cmd.AddValue("pkt-size", "Data packet's size (bytes)", dataPktSize);
  cmd.AddValue("mac",
               "MAC protocol to be used: ns3::AquaSimFama, ns3::AquaSimSFama,"
               "ns3::AquaSimBroadcastMac, ns3::AquaSimAloha",
               macProtName);

  cmd.Parse(argc, argv);

  std::cout << "-----------Initializing simulation-----------\n";

  GlobalValue::Bind("SimulatorImplementationType",
                    StringValue("ns3::RealtimeSimulatorImpl"));

  NodeContainer nodesCon;
  nodesCon.Create(nodes);

  // establish layers using helper's pre-build settings
  AquaSimChannelHelper channel = AquaSimChannelHelper::Default();
  channel.SetPropagation("ns3::AquaSimRangePropagation");
  AquaSimHelper asHelper = AquaSimHelper::Default();

  asHelper.SetChannel(channel.Create());
  asHelper.SetMac(macProtName);
  asHelper.SetMacAttribute("BitRate", DoubleValue(devBitRate));
  asHelper.SetMacAttribute("EncodingEfficiency", DoubleValue(1));

  std::string debugComponent;
  if (macProtName == "ns3::AquaSimSFama") {
    debugComponent = "AquaSimSFama";
  } else if (macProtName == "ns3::AquaSimFama") {
    asHelper.SetMacAttribute("RTSToNextHop", BooleanValue(true));
    asHelper.SetMacAttribute("DataPacketSize", IntegerValue(0));
    asHelper.SetMacAttribute("MaxTransmitDistance", DoubleValue(range));
    debugComponent = "AquaSimFama";
  } else if (macProtName == "ns3::AquaSimBroadcastMac") {
    debugComponent = "AquaSimBroadcastMac";
  } else if (macProtName == "ns3::AquaSimAloha") {
    asHelper.SetMacAttribute("MaxTransmitDistance", DoubleValue(range));
    debugComponent = "AquaSimAloha";
  }

  if (debugMac) {
    LogComponentEnable("AquaSimMac", LogLevel(LOG_ALL | LOG_PREFIX_ALL));
    LogComponentEnable(debugComponent.c_str(),
                       LogLevel(LOG_ALL | LOG_PREFIX_ALL));
  }
  if (debugPhy) {
    LogComponentEnable("AquaSimPhyCmn", LogLevel(LOG_ALL | LOG_PREFIX_ALL));
  }
  if (debugChannel) {
    LogComponentEnable("AquaSimChannel", LogLevel(LOG_ALL | LOG_PREFIX_ALL));
  }

  MobilityHelper mobility;
  NetDeviceContainer devices;
  Ptr<ListPositionAllocator> position = CreateObject<ListPositionAllocator>();

  // Static Y and Z dimension for now
  Vector boundry = Vector(0, 0, 0);

  std::cout << "Creating Nodes\n";

  for (NodeContainer::Iterator i = nodesCon.Begin(); i != nodesCon.End(); i++) {
    Ptr<AquaSimNetDevice> newDevice = CreateObject<AquaSimNetDevice>();
    position->Add(boundry);
    devices.Add(asHelper.CreateWithoutRouting(*i, newDevice));
    boundry.x += 20;

    auto phy = newDevice->GetPhy();
    phy->SetTransRange(range);
    phy->SetAttribute("PT", ns3::DoubleValue(0.2818));
    phy->SetAttribute("Frequency", ns3::DoubleValue(25));
    phy->SetAttribute("L", ns3::DoubleValue(0));
    phy->SetAttribute("K", ns3::DoubleValue(2.0));
    phy->SetAttribute("TurnOnEnergy", ns3::DoubleValue(0));
    phy->SetAttribute("TurnOffEnergy", ns3::DoubleValue(0));
    phy->SetAttribute("Preamble", ns3::DoubleValue(0));
    auto em = phy->EM();
    em->SetAttribute("InitialEnergy", ns3::DoubleValue(900000));
    em->SetAttribute("RxPower", ns3::DoubleValue(0.660));
    em->SetAttribute("TxPower", ns3::DoubleValue(0.395));
    em->SetAttribute("IdlePower", ns3::DoubleValue(0.0));
    ns3::Ptr<AquaSimModulation> modulation = phy->Modulation(NULL);
    modulation->SetAttribute("CodingEff", ns3::DoubleValue(1));
    modulation->SetAttribute("SPS", ns3::UintegerValue(devBitRate));
    modulation->SetAttribute("BER", ns3::DoubleValue(0.0));
  }

  mobility.SetPositionAllocator(position);
  mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  mobility.Install(nodesCon);

  ns3::Ptr<NetDevice> dev0 = devices.Get(0);
  ns3::Address dev2Addr = devices.Get(nodes-1)->GetAddress();

  ns3::Ptr<NetDevice> dev1 = devices.Get(1);

  Packet::EnablePrinting();
  std::ofstream ascii(asciiTraceFile.c_str());
  if (!ascii.is_open()) {
    NS_FATAL_ERROR("Could not open trace file.");
  }
  asHelper.EnableAsciiAll(ascii);

  std::cout << "-----------Running Simulation-----------\n";

  ns3::Config::Connect("/NodeList/*/DeviceList/0/Mac/RoutingRx",
                       MakeCallback(&Test::RoutingPacketRx, this));

  Config::Connect("/NodeList/*/$ns3::MobilityModel/CourseChange",
                  MakeCallback(&Test::CourseChange, this));

  std::thread sch([&]() {
    Simulator::Schedule(Seconds(0),
                        MakeEvent(&Test::SetSimulationStartDateTime, this));
    Simulator::Run();
  });
  sch.detach();

  auto pktSize = dataPktSize;
  std::this_thread::sleep_for(
      std::chrono::seconds(2)); // increase this delay if RTSToNextHop == false
  while (1) {
    for (auto i = 0; i < numPkts; i++) {
      Simulator::ScheduleWithContext(0, Seconds(0), &Test::SendPacket, this,
                                     dev0, dev2Addr, pktSize);
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    std::this_thread::sleep_for(std::chrono::seconds(50));
  };

  Simulator::Destroy();
}

int main(int argc, char *argv[]) {
  Test test;
  test.RunTest(argc, argv);
  return 0;
}
