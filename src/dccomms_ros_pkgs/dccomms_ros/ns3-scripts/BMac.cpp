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

NS_LOG_COMPONENT_DEFINE("BMac");

class Test : virtual public cpplogging::Logger {
public:
  Test();
  void RunTest();
  void RoutingPacketTx(std::string context, Ptr<const Packet>);
  void RoutingPacketRx(std::string context, Ptr<const Packet>);
  void CourseChange(std::string context, Ptr<const MobilityModel> model);
  void GetSimTime(const char *format, std::string &datetime,
                  double &secsFromStart);
  void SetSimulationStartDateTime();

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

void Test::RoutingPacketRx(std::string context, Ptr<const Packet> pkt) {
  AquaSimHeader ash;
  pkt->PeekHeader(ash);
  auto saddr = ash.GetSAddr().GetAsInt();
  auto daddr = ash.GetDAddr().GetAsInt();

  std::string datetime;
  double secs;
  GetSimTime(timeFormat, datetime, secs);

  routingLog.Info(
      "({} secs; {}) {}: (Addr: {}) Received packet from {} ; {} bytes", secs,
      datetime, context, daddr, saddr, pkt->GetSize());
}

void Test::CourseChange(std::string context, Ptr<const MobilityModel> model) {
  Vector position = model->GetPosition();

  std::string datetime;
  double secs;
  GetSimTime(timeFormat, datetime, secs);
  mobilityLog.Info("({} secs; {}) {}: [x,y,z] = [{},{},{}]", secs, datetime,
                   context, position.x, position.y, position.z);
}
void Test::RoutingPacketTx(std::string context, Ptr<const Packet> pkt) {
  AquaSimHeader ash;
  pkt->PeekHeader(ash);
  auto saddr = ash.GetSAddr().GetAsInt();
  auto daddr = ash.GetDAddr().GetAsInt();
  auto nhaddr = ash.GetNextHop().GetAsInt();

  std::string datetime;
  double secs;
  GetSimTime(timeFormat, datetime, secs);

  routingLog.Info("({} secs; {}) {}: (Addr: {}) Transmitting packet to {}. "
                  "Next hop: {} ; {} bytes",
                  secs, datetime, context, saddr, daddr, nhaddr,
                  pkt->GetSize());
}

void Test::RunTest() {
  double simStop = 60; // seconds
  int nodes = 4;
  int sinks = 1;
  uint32_t m_dataRate = 180;   // 120;
  uint32_t m_packetSize = 300; // 32;
  double range = 22;

  std::string asciiTraceFile = "bMAC-trace.asc";

  /*
   * **********
   * Node -> NetDevice -> AquaSimNetDeive -> etc.
   * Note: Nodelist keeps track of all nodes created.
   * ---Also need to look into id of nodes and assignment of this
   * ---need to look at assignment of address and making it unique per node.
   *
   *
   *  Ensure to use NS_LOG when testing in terminal. ie. ./waf --run
   * broadcastMAC_example NS_LOG=Node=level_all or export
   * 'NS_LOG=*=level_all|prefix_func'
   *  *********
   */

  LogComponentEnable("BMac", LOG_LEVEL_INFO);

  std::cout << "-----------Initializing simulation-----------\n";

  GlobalValue::Bind("SimulatorImplementationType",
                    StringValue("ns3::RealtimeSimulatorImpl"));

  NodeContainer nodesCon;
  NodeContainer sinksCon;
  nodesCon.Create(nodes);
  sinksCon.Create(sinks);

  PacketSocketHelper socketHelper;
  socketHelper.Install(nodesCon);
  socketHelper.Install(sinksCon);

  // establish layers using helper's pre-build settings
  AquaSimChannelHelper channel = AquaSimChannelHelper::Default();
  channel.SetPropagation("ns3::AquaSimRangePropagation");
  AquaSimHelper asHelper = AquaSimHelper::Default();
  // AquaSimEnergyHelper energy;	//******this could instead be handled by
  // node
  // helper. ****/
  asHelper.SetChannel(channel.Create());
  asHelper.SetMac("ns3::AquaSimBroadcastMac");
  asHelper.SetRouting("ns3::AquaSimRoutingDummy"); // XXX

  /*
   * Preset up mobility model for nodes and sinks here
   */
  MobilityHelper mobility;
  NetDeviceContainer devices;
  Ptr<ListPositionAllocator> position = CreateObject<ListPositionAllocator>();

  // Static Y and Z dimension for now
  Vector boundry = Vector(0, 0, 0);

  std::cout << "Creating Nodes\n";

  for (NodeContainer::Iterator i = nodesCon.Begin(); i != nodesCon.End(); i++) {
    Ptr<AquaSimNetDevice> newDevice = CreateObject<AquaSimNetDevice>();
    position->Add(boundry);

    devices.Add(asHelper.Create(*i, newDevice));

    /*  NS_LOG_DEBUG("Node: " << *i << " newDevice: " << newDevice << " Position: " <<
                     boundry.x << "," << boundry.y << "," << boundry.z <<
                     " freq:" << newDevice->GetPhy()->GetFrequency() << " addr:" <<
         AquaSimAddress::ConvertFrom(newDevice->GetAddress()).GetAsInt() );
                  */ //<<
    //" NDtypeid:" << newDevice->GetTypeId() <<
    //" Ptypeid:" << newDevice->GetPhy()->GetTypeId());

    boundry.x += 20;
    newDevice->GetPhy()->SetTransRange(range);
  }

  for (NodeContainer::Iterator i = sinksCon.Begin(); i != sinksCon.End(); i++) {
    Ptr<AquaSimNetDevice> newDevice = CreateObject<AquaSimNetDevice>();
    position->Add(boundry);

    devices.Add(asHelper.Create(*i, newDevice));

    /*  NS_LOG_DEBUG("Sink: " << *i << " newDevice: " << newDevice << "
       Position: " <<
                     boundry.x << "," << boundry.y << "," << boundry.z<< "
       addr:" <<
         AquaSimAddress::ConvertFrom(newDevice->GetAddress()).GetAsInt() );
         */
    boundry.x += 20;
    newDevice->GetPhy()->SetTransRange(range);
  }

  mobility.SetPositionAllocator(position);
  mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  mobility.Install(nodesCon);
  mobility.Install(sinksCon);

  PacketSocketAddress socket;
  socket.SetAllDevices();
  // socket.SetSingleDevice (devices.Get(0)->GetIfIndex());
  socket.SetPhysicalAddress(
      devices.Get(nodes)->GetAddress()); // for &dest on Recv()
  socket.SetProtocol(0);

  OnOffHelper app("ns3::PacketSocketFactory", Address(socket));
  app.SetAttribute("OnTime",
                   StringValue("ns3::ConstantRandomVariable[Constant=1]"));
  app.SetAttribute("OffTime",
                   StringValue("ns3::ConstantRandomVariable[Constant=0]"));
  app.SetAttribute("DataRate", DataRateValue(m_dataRate));
  app.SetAttribute("PacketSize", UintegerValue(m_packetSize));

  ApplicationContainer apps = app.Install(nodesCon.Get(0));
  apps.Start(Seconds(0.5));
  apps.Stop(Seconds(simStop + 1));
  //  ApplicationContainer apps2 = app.Install(nodesCon.Get(1));
  //  apps2.Start(Seconds(0.6));
  //  apps2.Stop(Seconds(simStop + 1.1));

  Packet::EnablePrinting(); // for debugging purposes
  Simulator::Stop(Seconds(simStop));
  //  // AnimationInterface anim ("bmac-anim.xml"); /* Animiation is very buggy
  //  with
  //  // Aqua-Sim NG */
  std::ofstream ascii(asciiTraceFile.c_str());
  if (!ascii.is_open()) {
    NS_FATAL_ERROR("Could not open trace file.");
  }
  asHelper.EnableAsciiAll(ascii);

  std::cout << "-----------Running Simulation-----------\n";

  auto node0 = nodesCon.Get(0);
  //  bool cont = true;
  //  std::thread mobilityWorker([node0, &cont]() {
  //    auto mobility = node0->GetObject<MobilityModel>();
  //    while (cont) {
  //      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  ////      auto x = mobility->GetPosition().x;
  ////      auto y = mobility->GetPosition().y;
  ////      mobility->SetPosition(Vector3D(x + 0.2, y + 0.2, 0));
  //    }
  //  });

  Config::Connect("/NodeList/*/DeviceList/*/Routing/PacketReceived",
                  MakeCallback(&Test::RoutingPacketRx, this));
  Config::Connect("/NodeList/*/DeviceList/*/Routing/PacketTransmitting",
                  MakeCallback(&Test::RoutingPacketTx, this));

  Config::Connect("/NodeList/*/$ns3::MobilityModel/CourseChange",
                  MakeCallback(&Test::CourseChange, this));

  Simulator::Schedule(Seconds(0),
                      MakeEvent(&Test::SetSimulationStartDateTime, this));
  Simulator::Run();
  // cont = false;
  // mobilityWorker.join();
  Simulator::Destroy();
}

int main(int argc, char *argv[]) {
  Test test;
  test.RunTest();
  return 0;
}
