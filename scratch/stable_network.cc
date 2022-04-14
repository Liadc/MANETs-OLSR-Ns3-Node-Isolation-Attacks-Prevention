#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/olsr-module.h"
#include "ns3/mobility-module.h"
#include "ns3/wifi-module.h"
#include "ns3/dsdv-module.h"
#include "ns3/output-stream-wrapper.h"
#include<string>
using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("StableNetworkRoute");

int main (int argc, char *argv[]){
	// Time
	Time::SetResolution (Time::NS);

	// Variables
	uint32_t nNodes = 100;
	bool bMobility = false;
	uint32_t nProtocol = 0;
	double dSimulationSeconds = 301.0;
	uint32_t nSimulationSeconds = 301;
	std::string mProtocolName = "Invalid";
	bool bSuperTransmission = false;
	bool bPrintAll = false;

	// Parameters from command line
	CommandLine cmd;
	cmd.AddValue("nNodes", "Number of nodes in the simulation", nNodes);
	cmd.AddValue("bMobility", "Delcares whenever there is movement in the network", bMobility);
	cmd.AddValue("nProtocol", "OLSR=0, DSDV=1", nProtocol);
	cmd.AddValue("nSimulationSeconds", "Amount of seconds to run the simulation", nSimulationSeconds);
	cmd.AddValue("bSuperTransmission", "Transmission boost to node X?", bSuperTransmission);
	cmd.AddValue("bPrintAll", "Print routing table for all hops", bPrintAll);
	cmd.Parse (argc, argv);

	if (nSimulationSeconds > 10.0) dSimulationSeconds = nSimulationSeconds; // Force minimum time

	// Build network
	NodeContainer nodes;
	nodes.Create (nNodes);
		
	// Add wifi
	WifiHelper wifi;
	wifi.SetStandard(WIFI_PHY_STANDARD_80211g);
	YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default ();
	YansWifiPhyHelper wifiPhy = YansWifiPhyHelper::Default ();
	NqosWifiMacHelper wifiMac = NqosWifiMacHelper::Default ();
	wifiPhy.SetChannel(wifiChannel.Create());
	wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager");
	wifiMac.SetType ("ns3::AdhocWifiMac");
	NetDeviceContainer adhocDevices = wifi.Install (wifiPhy, wifiMac, nodes);

	// Fix range
	wifiChannel.AddPropagationLoss ("ns3::RangePropagationLossModel", "MaxRange", DoubleValue (105));

	// Rig Node for huge wifi boost
	if (bSuperTransmission){
		NodeContainer superNodes;
		superNodes.Create (1);
		wifiPhy.Set("RxGain", DoubleValue(1000.0));
		wifiPhy.Set("TxGain", DoubleValue(1000.0));
		NetDeviceContainer superDevices = wifi.Install (wifiPhy, wifiMac, superNodes);
		adhocDevices.Add(superDevices);
		nodes.Add(superNodes);
	}

	// Install OLSR / DSDV
	OlsrHelper olsr;
	DsdvHelper dsdv;
	Ipv4ListRoutingHelper routeList;
	InternetStackHelper internet;
	std::string fName = "";
	Ptr<OutputStreamWrapper> stream = Create<OutputStreamWrapper>("Stable_Network_Stream.txt",std::ios::out);	

	switch (nProtocol){
		case 0:
			routeList.Add (olsr, 100);
			if (!bPrintAll)
				olsr.PrintRoutingTableEvery(Seconds(10.0), nodes.Get(1), stream);
			else {
				olsr.PrintRoutingTableAllEvery(Seconds(10.0), stream);
			}
			break;
		case 1:
			routeList.Add (dsdv, 100);
			if (!bPrintAll)
				dsdv.PrintRoutingTableEvery(Seconds(10.0), nodes.Get(1), stream);
			else {
				dsdv.PrintRoutingTableAllEvery(Seconds(10.0), stream);
			}
			break;
		default:
			NS_FATAL_ERROR ("Invalid routing protocol chosen " << nProtocol);
			break;
	}
	internet.SetRoutingHelper(routeList);
	internet.Install (nodes);

	// Install IP
	Ipv4AddressHelper addresses;
	addresses.SetBase ("10.0.0.0", "255.255.255.0");
	Ipv4InterfaceContainer interfaces;
	interfaces = addresses.Assign (adhocDevices);

	// Install mobility
	ObjectFactory pos;
	pos.SetTypeId ("ns3::RandomRectanglePositionAllocator");
	pos.Set ("X", StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=300.0]"));
	//pos.Set ("Y", StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=300.0]"));
	pos.Set ("Y", StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=1500.0]"));
	Ptr<PositionAllocator> taPositionAlloc = pos.Create ()->GetObject<PositionAllocator> ();
	MobilityHelper mobility;
	mobility.SetPositionAllocator (taPositionAlloc);
	if (bMobility) {
		mobility.SetMobilityModel ("ns3::RandomWalk2dMobilityModel",
				"Bounds", RectangleValue (Rectangle (-50, 50, -50, 50)));
	} else {
		mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
	}
	mobility.Install (nodes);

	// Routing Protocl dump
	Ptr<Ipv4> stack = nodes.Get (1)->GetObject<Ipv4> ();
	Ptr<Ipv4RoutingProtocol> rp_Gw = (stack->GetRoutingProtocol ());
	Ptr<Ipv4ListRouting> lrp_Gw = DynamicCast<Ipv4ListRouting> (rp_Gw);
	Ptr<olsr::RoutingProtocol> olsrrp_Gw;	

/*	for (uint32_t i = 0; i < lrp_Gw->GetNRoutingProtocols (); i++){
		int16_t priority;
		Ptr<Ipv4RoutingProtocol> temp = lrp_Gw->GetRoutingProtocol (i, priority);
		if (DynamicCast<olsr::RoutingProtocol> (temp)){
			olsrrp_Gw = DynamicCast<olsr::RoutingProtocol> (temp);
			if (i==0) olsrrp_Gw->Dump();
		}
	}
//	olsrrp_Gw->Dump();
*/
	wifiPhy.EnablePcapAll ("Stable_Network_");

	// Run simulation
	NS_LOG_INFO ("Run simulation.");
	Simulator::Stop (Seconds (dSimulationSeconds));

	Simulator::Run ();
	Simulator::Destroy ();

	return 0;
}
