/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2016 University of Connecticut
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
#include "ns3/network-module.h"
#include <cpplogging/cpplogging.h>

/*
 * VBF Test
 */

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("VBF");

class Test : virtual public cpplogging::Logger {
public:
  Test();
  void RunTest(int argc, char *argv[]);
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

  std::stringstream stream;

  stream << "r " << Simulator::Now().GetSeconds() << " " << context << " "
         << *pkt << std::endl;
  routingLog.Info("({} secs; {}) {}: (Addr: {}) Received packet from {} ; {} "
                  "bytes.\n\tPacket: {}",
                  secs, datetime, context, daddr, saddr, pkt->GetSize(),
                  stream.str());
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

void Test::RunTest(int argc, char *argv[]) {
  {
    double simStop = 200; // seconds
    int nodes = 200;
    int sinks = 1;
    uint32_t m_dataRate = 10000;
    uint32_t m_packetSize = 40;
    double range = 100;
    // int m_maxBurst =10;
    std::string asciiTraceFile = "bMAC-trace.asc";

    LogComponentEnable("VBF", LOG_LEVEL_INFO);

    CommandLine cmd;
    cmd.AddValue("simStop", "Length of simulation", simStop);
    cmd.AddValue("nodes", "Amount of regular underwater nodes", nodes);
    cmd.AddValue("sinks", "Amount of underwater sinks", sinks);
    cmd.Parse(argc, argv);
    std::cout << "-----------Initializing simulation-----------\n";

    GlobalValue::Bind("SimulatorImplementationType",
                      StringValue("ns3::RealtimeSimulatorImpl"));

    NodeContainer nodesCon;
    NodeContainer sinksCon;
    NodeContainer senderCon;
    nodesCon.Create(nodes);
    sinksCon.Create(sinks);
    senderCon.Create(1);

    PacketSocketHelper socketHelper;
    socketHelper.Install(nodesCon);
    socketHelper.Install(sinksCon);
    socketHelper.Install(senderCon);

    // establish layers using helper's pre-build settings
    AquaSimChannelHelper channel = AquaSimChannelHelper::Default();
    channel.SetPropagation("ns3::AquaSimRangePropagation");
    AquaSimHelper asHelper = AquaSimHelper::Default();
    // AquaSimEnergyHelper energy;	//******this could instead be handled by
    // node
    // helper. ****/
    asHelper.SetChannel(channel.Create());
    asHelper.SetMac("ns3::AquaSimBroadcastMac");
    asHelper.SetRouting("ns3::AquaSimVBF", "Width", DoubleValue(20),
                        "TargetPos", Vector3DValue(Vector(190, 190, 0)));

    /*
     * Preset up mobility model for nodes and sinks here
     */
    MobilityHelper mobility;
    MobilityHelper nodeMobility;
    NetDeviceContainer devices;
    Ptr<ListPositionAllocator> position = CreateObject<ListPositionAllocator>();

    std::cout << "Creating Nodes\n";

    for (NodeContainer::Iterator i = nodesCon.Begin(); i != nodesCon.End();
         i++) {
      Ptr<AquaSimNetDevice> newDevice = CreateObject<AquaSimNetDevice>();
      devices.Add(asHelper.Create(*i, newDevice));
      newDevice->GetPhy()->SetTransRange(range);
    }

    for (NodeContainer::Iterator i = sinksCon.Begin(); i != sinksCon.End();
         i++) {
      Ptr<AquaSimNetDevice> newDevice = CreateObject<AquaSimNetDevice>();
      position->Add(Vector(190, 190, 0));
      devices.Add(asHelper.Create(*i, newDevice));
      newDevice->GetPhy()->SetTransRange(range);
    }

    Ptr<AquaSimNetDevice> newDevice = CreateObject<AquaSimNetDevice>();
    position->Add(Vector(10, 10, 0));
    devices.Add(asHelper.Create(senderCon.Get(0), newDevice));
    newDevice->GetPhy()->SetTransRange(range);

    // Set sink at origin and surround with uniform distribution of regular
    // nodes.
    mobility.SetPositionAllocator(position);
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    nodeMobility.SetPositionAllocator(
        "ns3::UniformDiscPositionAllocator", "X", DoubleValue(100.0), "Y",
        DoubleValue(100.0), "rho", DoubleValue(100));
    nodeMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    nodeMobility.Install(nodesCon);
    mobility.Install(sinksCon);
    mobility.Install(senderCon);

    PacketSocketAddress socket;
    socket.SetAllDevices();
    // socket.SetSingleDevice (devices.Get(0)->GetIfIndex());
    socket.SetPhysicalAddress(devices.Get(nodes)->GetAddress());
    socket.SetProtocol(0);

    // std::cout << devices.Get(nodes)->GetAddress() << " &&& " <<
    // devices.Get(0)->GetIfIndex() << "\n";
    // std::cout << devices.Get(0)->GetAddress() << " &&& " <<
    // devices.Get(1)->GetIfIndex() << "\n";

    OnOffHelper app("ns3::PacketSocketFactory", Address(socket));
    app.SetAttribute(
        "OnTime", StringValue("ns3::ConstantRandomVariable[Constant=0.0066]"));
    app.SetAttribute(
        "OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0.9934]"));
    //  app.SetAttribute ("OnTime", StringValue
    //  ("ns3::ConstantRandomVariable[Constant=0.026]"));
    //  app.SetAttribute ("OffTime", StringValue
    //  ("ns3::ConstantRandomVariable[Constant=0.974]"));
    app.SetAttribute("DataRate", DataRateValue(m_dataRate));
    app.SetAttribute("PacketSize", UintegerValue(m_packetSize));

    ApplicationContainer apps = app.Install(senderCon);
    apps.Start(Seconds(0.5));
    apps.Stop(Seconds(simStop));

    Ptr<Node> sinkNode = sinksCon.Get(0);
    TypeId psfid = TypeId::LookupByName("ns3::PacketSocketFactory");

    Ptr<Socket> sinkSocket = Socket::CreateSocket(sinkNode, psfid);
    sinkSocket->Bind(socket);

    std::ofstream ascii(asciiTraceFile.c_str());
    if (!ascii.is_open()) {
      NS_FATAL_ERROR("Could not open trace file.");
    }
    asHelper.EnableAsciiAll(ascii);

    Packet::EnablePrinting(); // for debugging purposes
    Config::Connect("/NodeList/*/DeviceList/*/Routing/PacketReceived",
                    MakeCallback(&Test::RoutingPacketRx, this));
    Config::Connect("/NodeList/*/DeviceList/*/Routing/PacketTransmitting",
                    MakeCallback(&Test::RoutingPacketTx, this));

    Config::Connect("/NodeList/*/$ns3::MobilityModel/CourseChange",
                    MakeCallback(&Test::CourseChange, this));

    Simulator::Schedule(Seconds(0),
                        MakeEvent(&Test::SetSimulationStartDateTime, this));
    std::cout << "-----------Running Simulation-----------\n";
    Simulator::Stop(Seconds(simStop));
    Simulator::Run();
    asHelper.GetChannel()->PrintCounters();
    Simulator::Destroy();

    std::cout << "fin.\n";
  }
}
int main(int argc, char *argv[]) {

  // to change on the fly
  Test test;
  test.RunTest(argc, argv);
  return 0;
}
