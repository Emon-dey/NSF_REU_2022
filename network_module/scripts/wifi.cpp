#include <iostream>
#include <fstream>
#include “ns3/core-module.h”
#include “ns3/network-module.h”
#include “ns3/mobility-module.h”
#include “ns3/wifi-module.h”
#include “ns3/tap-bridge-module.h”
// network devices, do not exceed COUNT in nns_setup.py
static const int COUNT=5;
using namespace ns3;
NS_LOG_COMPONENT_DEFINE (“LxcNs3Wifi”);
int
main (int argc, char *argv [])
{
double distance = 1.0;
int zltime = 60;
CommandLine cmd;
cmd.AddValue (“distance,” “Distance apart to place nodes (in meters).”,
 distance);
cmd.AddValue (“time,” “time to run experiment (in seconds).”,
 zltime);
cmd.Parse (argc, argv);
distance=distance/10;
std::cout << “Providing ns-3 Wifi network emulation for “ << COUNT << “ devices\
n”;
std::cout << “Distance: “ << distance << “ \n”;
std::cout << “Time: “ << zltime << “ \n”;
GlobalValue::Bind (“SimulatorImplementationType,” StringValue
(“ns3::RealtimeSimulatorImpl”));
GlobalValue::Bind (“ChecksumEnabled,” BooleanValue (true));
NodeContainer nodes;
nodes.Create (COUNT);
WifiHelper wifi;
wifi.SetStandard (WIFI_PHY_STANDARD_80211a);
wifi.SetRemoteStationManager (“ns3::ConstantRateWifiManager,” “DataMode,”
StringValue (“OfdmRate54Mbps”));
WifiMacHelper wifiMac;
wifiMac.SetType (“ns3::AdhocWifiMac”);
YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default ();
YansWifiPhyHelper wifiPhy = YansWifiPhyHelper::Default ();
wifiPhy.SetChannel (wifiChannel.Create ());
NetDeviceContainer devices = wifi.Install (wifiPhy, wifiMac, nodes);
MobilityHelper mobility;
Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
for (int i=0; i<COUNT; i++) {
 positionAlloc->Add (Vector (0.0, 0.0, 0.0));
positionAlloc->Add (Vector (distance, 0.0, 0.0));
positionAlloc->Add (Vector (-distance, 0.0, 0.0));
positionAlloc->Add (Vector (0.0, distance, 0.0));
positionAlloc->Add (Vector (0.0, -distance, 0.0));
}
mobility.SetPositionAllocator (positionAlloc);
mobility.SetMobilityModel (“ns3::ConstantPositionMobilityModel”);
mobility.Install (nodes);
TapBridgeHelper tapBridge;
tapBridge.SetAttribute (“Mode,” StringValue (“UseLocal”));
char buffer [10];
for (int i=0; i<COUNT; i++) {
 sprintf(buffer, “wifi_tap%d,” i+1);
 tapBridge.SetAttribute (“DeviceName,” StringValue(buffer));
 tapBridge.Install (nodes.Get(i), devices.Get(i));
}
Simulator::Stop (Seconds (zltime));
Simulator::Run ();
Simulator::Destroy ();
}