/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
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
 */

// Network topology
//
//       n0    n1   n2   n3
//       |     |    |    |
//       =================
//              LAN
//
// - CBR/UDP flows from n0 to n1 and from n3 to n0
// - DropTail queues
// - Tracing of queues and packet receptions to file "csma-one-subnet.tr"

#include <fstream>
#include <iostream>

#include <ns3/applications-module.h>
#include <ns3/core-module.h>
#include <ns3/csma-module.h>
#include <ns3/internet-module.h>
#include <ns3/network-module.h>

#include <dccomms_packets/VariableLengthPacket.h>
#include <dccomms_ros/simulator/NetsimTime.h>

using namespace ns3;
using namespace dccomms_ros;

NS_LOG_COMPONENT_DEFINE("CsmaOneSubnetExample");

// this class represents addresses which are 2 bytes long.
class MyAddress {
  typedef uint16_t AddSize;

public:
  MyAddress(AddSize addr = 0) { m_addr = addr; }
  Address ConvertTo(void) const;
  static MyAddress ConvertFrom(const Address &address);

private:
  static uint8_t GetType(void);
  AddSize m_addr;
};
Address MyAddress::ConvertTo(void) const {
  return Address(GetType(), reinterpret_cast<const uint8_t *>(&m_addr),
                 sizeof(m_addr));
}
MyAddress MyAddress::ConvertFrom(const Address &address) {
  MyAddress ad;
  NS_ASSERT(address.CheckCompatible(GetType(), sizeof(m_addr)));
  address.CopyTo(reinterpret_cast<uint8_t *>(&ad.m_addr));
  return ad;
}
uint8_t MyAddress::GetType(void) {
  static uint8_t type = Address::Register();
  return type;
}

class Test {
public:
  void SendPacket(ns3::Ptr<NetDevice> &dev, Address &addr) {
    auto pkt = ns3::Create<ns3::Packet>(400);
    dev->Send(pkt, addr, 0);
  }

private:
};

int main(int argc, char *argv[]) {
//
// Users may find it convenient to turn on explicit debugging
// for selected modules; the below lines suggest how to do this
//
#if 0
  LogComponentEnable ("CsmaOneSubnetExample", LOG_LEVEL_INFO);
#endif
  //
  // Allow the user to override any of the defaults and the above Bind() at
  // run-time, via command-line arguments
  //
  CommandLine cmd;
  cmd.Parse(argc, argv);

  GlobalValue::Bind("SimulatorImplementationType",
                    StringValue("ns3::RealtimeSimulatorImpl"));

  //
  // Explicitly create the nodes required by the topology (shown above).
  //
  NS_LOG_INFO("Create nodes.");
  NodeContainer nodes;
  nodes.Create(4);

  NS_LOG_INFO("Build Topology");
  CsmaHelper csma;
  csma.SetChannelAttribute("DataRate", DataRateValue(400));
  csma.SetChannelAttribute("Delay", TimeValue(MilliSeconds(2)));
  csma.SetDeviceAttribute ("EncapsulationMode", StringValue ("Llc"));

  //
  // Now fill out the topology by creating the net devices required to connect
  // the nodes to the channels and hooking them up.
  //
  NetDeviceContainer devices = csma.Install(nodes);

  ns3::Ptr<Node> n0 = nodes.Get(0);
  ns3::Ptr<Node> n3 = nodes.Get(3);
  ns3::Ptr<NetDevice> dev0 = n0->GetDevice(0);
  ns3::Ptr<NetDevice> dev3 = n3->GetDevice(0);
  ns3::Address addr3 = dev3->GetAddress();

  //
  // Configure ascii tracing of all enqueue, dequeue, and NetDevice receive
  // events on all devices.  Trace output will be sent to the file
  // "csma-one-subnet.tr"
  //
  AsciiTraceHelper ascii;
  csma.EnableAsciiAll(ascii.CreateFileStream("csma-one-subnet.tr"));

  //
  // Also configure some tcpdump traces; each interface will be traced.
  // The output files will be named:
  //
  //     csma-one-subnet-<node ID>-<device's interface index>.pcap
  //
  // and can be read by the "tcpdump -r" command (use "-tt" option to
  // display timestamps correctly)
  //
  csma.EnablePcapAll("csma-one-subnet", false);
  //
  // Now, do the actual simulation.
  //
  NS_LOG_INFO("Run Simulation.");

  NetsimTime::Reset();

  std::thread scheduler([&]() { Simulator::Run(); });
  scheduler.detach();

  this_thread::sleep_for(milliseconds(100));

  Test test;
  Simulator::Schedule(Seconds(2),
                      MakeEvent(&Test::SendPacket, &test, dev0, addr3));

  while (1) {
    this_thread::sleep_for(seconds(2));
  }

  Simulator::Destroy();
  NS_LOG_INFO("Done.");
}
