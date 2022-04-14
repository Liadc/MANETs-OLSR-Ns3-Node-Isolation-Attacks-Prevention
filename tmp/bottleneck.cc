#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/solsr-module.h"
#include "ns3/mobility-module.h"
#include "ns3/wifi-module.h"
#include "ns3/dsdv-module.h"
#include "ns3/output-stream-wrapper.h"
#include "ns3/netanim-module.h"
#include "ns3/udp-client-server-helper.h"
#include<string>
using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("BottleNeck");

// Sink for having package pass through
//void PacketTrace (const Packet p){
//	std::count << p;
//}

//static void AbortOnNeighbor (Ptr<Node> node, Ipv4Address address){
//	if (node->GetObject<RoutingProtocol>()->isItUpToTwoHop(address)){
//		Simulator::Stop();
//	}
//}
void MakeBet (NodeContainer* cont, Ipv4Address s, Ipv4Address t){
	std::vector<std::pair<double, Ipv4Address> > bets = cont->Get(2)->GetObject<RoutingProtocol>()->FindPotentionalBottleNeck(s, t);
	std::cout << Simulator::Now().GetSeconds() << "s:\n";
	if (bets.empty()) {
		std::cout << "No prediction\n\n";
		return;
	}
	/*
	double chance = bets[0].first;
	Ipv4Address location = bets[0].second;
	std::cout << Simulator::Now() << "\tPrediction: " << location << "\tChance: " << chance << std::endl;
	*/
	for (size_t i = 0; i < bets.size(); ++i){
		if (bets[i].first > 0)
			std::cout << bets[i].second << "  -  " << bets[i].first << std::endl;
	}
	std::cout << "\n";
}


int main (int argc, char *argv[]){
	// Time
	Time::SetResolution (Time::NS);

	// Variables
	uint32_t nNodes = 100;
	double dMaxGridX = 500.0;
	uint32_t nMaxGridX = 500;
	double dMaxGridY = 500.0;
	uint32_t nMaxGridY = 500;
	bool bMobility = false;
	uint32_t nProtocol = 0;
	double dSimulationSeconds = 301.0;
	uint32_t nSimulationSeconds = 301;
	std::string mProtocolName = "Invalid";
	bool bSuperTransmission = false;
	bool bHighRange = false;
	bool bUdpServer = false;
	bool bMakeBets = false;
	bool bFixPos = false;

	// Parameters from command line
	CommandLine cmd;
	cmd.AddValue("nNodes", "Number of nodes in the simulation", nNodes);
	cmd.AddValue("nMaxGridX", "X of the simulation rectangle", nMaxGridX);
	cmd.AddValue("nMaxGridY", "Y of the simulation rectangle", nMaxGridY);
	cmd.AddValue("bMobility", "Delcares whenever there is movement in the network", bMobility);
	cmd.AddValue("nProtocol", "SOLSR=0, DSDV=1", nProtocol);
	cmd.AddValue("nSimulationSeconds", "Amount of seconds to run the simulation", nSimulationSeconds);
	cmd.AddValue("bSuperTransmission", "Transmission boost to node X?", bSuperTransmission);
	cmd.AddValue("bHighRange", "Higher wifi range", bHighRange);
	cmd.AddValue("bUdpServer", "Try to send Udp packets from random nodes to the attacked node", bUdpServer);
	cmd.AddValue("bMakeBets", "Try to guess where the packet is going to pass", bMakeBets);
	cmd.AddValue("bFixPos", "Fix position for sending and receiving nodes", bFixPos);
	cmd.Parse (argc, argv);

	if (nSimulationSeconds > 10.0) dSimulationSeconds = nSimulationSeconds; // Force minimum time
	if (nMaxGridX > 10.0) dMaxGridX = nMaxGridX; // Force minimum size. Revert to default.
	if (nMaxGridY > 10.0) dMaxGridY = nMaxGridY; // Force minimum size. Revert to default.

	// Build network
	NodeContainer nodes;
	nodes.Create (nNodes);
		
	// Add wifi
	WifiHelper wifi;
	//wifi.SetStandard(WIFI_PHY_STANDARD_80211g);
	YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default ();
	YansWifiPhyHelper wifiPhy = YansWifiPhyHelper::Default ();
	NqosWifiMacHelper wifiMac = NqosWifiMacHelper::Default ();
	if (bHighRange){
		wifiPhy.Set("TxGain", DoubleValue(12.4));
		wifiChannel.AddPropagationLoss ("ns3::RangePropagationLossModel", "MaxRange", DoubleValue (250));
	} else {
		//wifiChannel.AddPropagationLoss ("ns3::RangePropagationLossModel", "MaxRange", DoubleValue (100));
	}
	wifiPhy.SetChannel(wifiChannel.Create());
	wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager");
	wifiMac.SetType ("ns3::AdhocWifiMac");
	NetDeviceContainer adhocDevices = wifi.Install (wifiPhy, wifiMac, nodes);

	// Rig Node for huge wifi boost
	if (bSuperTransmission){
		NodeContainer superNodes;
		superNodes.Create (1);
		wifiPhy.Set("RxGain", DoubleValue(500.0));
		wifiPhy.Set("TxGain", DoubleValue(500.0));
		NetDeviceContainer superDevices = wifi.Install (wifiPhy, wifiMac, superNodes);
		adhocDevices.Add(superDevices);
		nodes.Add(superNodes);
	}

	// Install SOLSR / DSDV
	SOlsrHelper solsr;
	DsdvHelper dsdv;
	Ipv4ListRoutingHelper routeList;
	InternetStackHelper internet;

	switch (nProtocol){
		case 0:
			routeList.Add (solsr, 100);
			if (true){
				//Ptr<OutputStreamWrapper> stream = Create<OutputStreamWrapper>("routing_solsr.txt",std::ios::out);	
				//solsr.PrintRoutingTableAllEvery(Seconds(10.0), stream);
			}
			break;
		case 1:
			routeList.Add (dsdv, 100);
			break;
		default:
			NS_FATAL_ERROR ("Invalid routing protocol chosen " << nProtocol);
			break;
	}
	internet.SetRoutingHelper(routeList);
	internet.Install (nodes);

	// Install IP
	Ipv4AddressHelper addresses;
	addresses.SetBase ("10.0.0.0", "255.0.0.0");
	Ipv4InterfaceContainer interfaces;
	interfaces = addresses.Assign (adhocDevices);

	// Install mobility
	MobilityHelper mobility;
	Ptr<UniformRandomVariable> randomGridX = CreateObject<UniformRandomVariable> ();
	Ptr<UniformRandomVariable> randomGridY = CreateObject<UniformRandomVariable> ();
	randomGridX->SetAttribute ("Min", DoubleValue (0));
	randomGridX->SetAttribute ("Max", DoubleValue (dMaxGridX));
	randomGridY->SetAttribute ("Min", DoubleValue (0));
	randomGridY->SetAttribute ("Max", DoubleValue (dMaxGridY));

	Ptr<RandomRectanglePositionAllocator> taPositionAlloc = CreateObject<RandomRectanglePositionAllocator> ();
	taPositionAlloc->SetX(randomGridX);
	taPositionAlloc->SetY(randomGridY);
	mobility.SetPositionAllocator (taPositionAlloc);
	if (bMobility) {
		mobility.SetMobilityModel ("ns3::RandomWalk2dMobilityModel",
				"Bounds", RectangleValue (Rectangle (0, dMaxGridX, 0, dMaxGridY)),
				//"Speed", StringValue("ns3::UniformRandomVariable[Min=0|Max=2.0]"),
				"Speed", StringValue("ns3::UniformRandomVariable[Min=1.5|Max=2.0]"),
				//"Speed", StringValue("ns3::UniformRandomVariable[Min=9.5|Max=10.0]"),
				"Time", TimeValue(Seconds(3.0)),
				"Mode", EnumValue(RandomWalk2dMobilityModel::MODE_TIME));
	} else {
		mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
	}
	mobility.Install (nodes);
	
	if (bFixPos){
		mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
		mobility.Install (nodes.Get(0));
		mobility.Install (nodes.Get(1));
		Vector vec;
		vec.x = 50;
		vec.y = 50;
		nodes.Get(0)->GetObject<MobilityModel>()->SetPosition(vec);		
		vec.x = dMaxGridX - 250;
		vec.y = dMaxGridY - 250;
		nodes.Get(1)->GetObject<MobilityModel>()->SetPosition(vec);	
	}


	if (bUdpServer){
		UdpServerHelper udpServerHelper(80);
		ApplicationContainer apps = udpServerHelper.Install(nodes.Get(0));
		Ptr<UdpServer> udpServer = udpServerHelper.GetServer();
		UdpClientHelper udpClientHelper(Ipv4Address("10.0.0.1"), 80);
		udpClientHelper.SetAttribute("Interval", TimeValue(Seconds(7)));
		udpClientHelper.SetAttribute("MaxPackets", UintegerValue(100000000));

		
		// Choose random node to become sender
		//Ptr<UniformRandomVariable> rnd = CreateObject<UniformRandomVariable> ();
		//rnd->SetAttribute ("Min", DoubleValue (3));
		//rnd->SetAttribute ("Max", DoubleValue (nodes.GetN()-1));
		

		// Install UdpClient on said node
		//uint32_t sendingNode = rnd->GetInteger();
		uint32_t sendingNode = 1;
		apps.Add(udpClientHelper.Install(nodes.Get(sendingNode)));
		//apps.Add(udpClientHelper.Install(nodes.Get(1)));
		apps.Start(Seconds(60));
		apps.Stop(Seconds(nSimulationSeconds));
		//for (size_t i=30; i<nSimulationSeconds-5; i=i+1){
		//	Simulator::Schedule(Seconds (i), &AbortOnNeighbor, nodes.Get(sendingNode), Ipv4Address("10.0.0.1"));
		//}
		//Simulator::Schedule(Seconds (250), &PrintReceivedPackets, udpServer);
	}

	if (bMakeBets){
		for (size_t i=59; i<nSimulationSeconds; i+=7){
			Simulator::Schedule(Seconds (i), &MakeBet, &nodes, Ipv4Address("10.0.0.2"), Ipv4Address("10.0.0.1"));
		}
	}
	//if (bIsolationAttackBug){
		//nodes.Get(2)->GetObject<RoutingProtocol>()->ExecuteIsolationAttack(Ipv4Address("10.0.0.1"));
	//}
	// Animation
	//AnimationInterface anim ("BottleNeck_animation.xml");
	//anim.SetMobilityPollInterval (Seconds (1));

	// Pcap
	//wifiPhy.EnablePcapAll ("BottleNeck_");
	std::ostringstream sspcap;
	sspcap << "BottleNeck_" << "FixPos_" << bFixPos << "_";
	wifiPhy.EnablePcapAll (sspcap.str());
	
	// Run simulation
	NS_LOG_INFO ("Run simulation.");
	Simulator::Stop (Seconds (dSimulationSeconds));

	Simulator::Run ();
	Simulator::Destroy ();

	return 0;
}
