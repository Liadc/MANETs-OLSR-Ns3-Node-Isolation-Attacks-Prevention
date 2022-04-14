#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/iolsr-module.h"
#include "ns3/mobility-module.h"
#include "ns3/wifi-module.h"
#include "ns3/dsdv-module.h"
#include "ns3/output-stream-wrapper.h"
#include "ns3/netanim-module.h"
#include "ns3/udp-client-server-helper.h"
#include<string>
using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("StableNetworkRouteMod");

// Count the nodes that will require fake nodes
static void PrintCountFakeNodes(NodeContainer* cont){
	//ns3::iolsr::RoutingProtocol rp;
	unsigned int count = 0;
	for (unsigned int i=0;i<cont->GetN();++i){
		Ptr<RoutingProtocol> pt = cont->Get(i)->GetObject<RoutingProtocol>();
		//NS_LOG_INFO("Node id: " << i << "\tTime: " << Simulator::Now().GetSeconds() << "\tRequireFake? " << pt->RequireFake());
		if (pt->RequireFake()) ++count;
	}
	NS_LOG_INFO("Total fakes required: " << count << " out of " << cont->GetN());
	std::cout << "Fictive: " << count << "/" << cont->GetN() << std::endl;
}

static void PrintMprFraction(NodeContainer* cont){
	double sum = 0;
	for (unsigned int i=0;i<cont->GetN();++i){
		Ptr<RoutingProtocol> pt = cont->Get(i)->GetObject<RoutingProtocol>();
		//NS_LOG_INFO("Node id: " << i << "\tTime: " << Simulator::Now().GetSeconds() << "\tFraction: " << pt->FractionOfMpr());
		sum += pt->FractionOfMpr();
	}
	NS_LOG_INFO("Total fraction of MPR: " << sum / (double) cont->GetN());
	NS_LOG_INFO(sum / (double) cont->GetN());
	//... NS_LOG_INFO broke. Using cout...
	//std::cout << "Total fraction of MPR: " << sum / (double) cont->GetN() << std::endl;
	std::cout << "MPR: " << (sum / (double) cont->GetN()) << std::endl;
}

static void PrintTcPowerLevel(NodeContainer* cont){
	double sumLsr = 0;
	double sumOlsr = 0;
	for (unsigned int i=0;i<cont->GetN();++i){
		Ptr<RoutingProtocol> pt = cont->Get(i)->GetObject<RoutingProtocol>();
		sumLsr += pt->tcPowerLevel(true);
		sumOlsr += pt->tcPowerLevel(false);
	}
	//NS_LOG_INFO("TC Level: " << sumOlsr / (double) cont->GetN() << " (lsr: " << sumLsr / (double) cont->GetN() << ")");
	std::cout << "TC Level: " << sumOlsr / (double) cont->GetN() << " \nlsr: " << sumLsr / (double) cont->GetN() << "\n";
}

static void ExecuteIsolationAttack(NodeContainer* cont){
	for (size_t i=0;i<cont->GetN(); ++i){
		cont->Get(i)->GetObject<RoutingProtocol>()->ExecuteIsolationAttack(Ipv4Address("10.0.0.1"));
	}
}

static void PercentageWithFullConnectivity(NodeContainer* cont){
	uint32_t count = 0;
	for (uint32_t i=0;i<cont->GetN();++i){
		uint32_t routingTableSize = cont->Get(i)->GetObject<RoutingProtocol>()->getRoutingTableSize();
		if (routingTableSize == cont->GetN() - 1) {
			++count;
		}
	}
	double result = count / (double) cont->GetN();
	std::cout << "Routing Precentage: " << result << "\n";
	Simulator::Schedule(Seconds (10), &PercentageWithFullConnectivity, cont);
}

static void ActivateFictiveDefence(NodeContainer* cont){
	for (unsigned int i=0; i < cont->GetN(); ++i){
		cont->Get(i)->GetObject<RoutingProtocol>()->activateFictiveDefence();
	}
}

static void PrintReceivedPackets(Ptr<UdpServer> udpServer){
	uint32_t count = udpServer->GetReceived();
	std::cout << "Received Udp Packets: " << count << std::endl;
}

static void AbortOnNeighbor (Ptr<Node> node, Ipv4Address address){
	if (node->GetObject<RoutingProtocol>()->isItUpToTwoHop(address)){
		Simulator::Stop();
	}
}

int main (int argc, char *argv[]){
	// Time
	Time::SetResolution (Time::NS);

	// Variables
	uint32_t nNodes = 38;
	double dMaxGridX = 1000.0;
	uint32_t nMaxGridX = 1000;
	double dMaxGridY = 700.0;
	uint32_t nMaxGridY = 700;
	bool bMobility = true;
	uint32_t nProtocol = 0;
	double dSimulationSeconds = 301.0;
	uint32_t nSimulationSeconds = 301;
	std::string mProtocolName = "Invalid";
	bool bPrintAll = false;
	bool bPrintFakeCount = false;
	bool bPrintMprFraction = false;
	bool bIsolationAttack = false;
	bool bEnableFictive = false;
	bool bPrintTcPowerLevel = false;
	bool bConnectivityPrecentage = false;
	bool bUdpServer = false;
	//double dMaxSpeed = 2;
	bool bHighMovement = false;

	// Parameters from command line
	CommandLine cmd;
	cmd.AddValue("nNodes", "Number of nodes in the simulation", nNodes);
	cmd.AddValue("nMaxGridX", "X of the simulation rectangle", nMaxGridX);
	cmd.AddValue("nMaxGridY", "Y of the simulation rectangle", nMaxGridY);
	cmd.AddValue("bMobility", "Delcares whenever there is movement in the network", bMobility);
	cmd.AddValue("nProtocol", "IOLSR=0, DSDV=1", nProtocol);
	cmd.AddValue("nSimulationSeconds", "Amount of seconds to run the simulation", nSimulationSeconds);
	cmd.AddValue("bPrintAll", "Print routing table for all hops", bPrintAll);
	cmd.AddValue("bPrintFakeCount", "Print amount of fake nodes required", bPrintFakeCount);
	cmd.AddValue("bPrintMprFraction", "Print fraction of MPR", bPrintMprFraction);
	cmd.AddValue("bPrintTcPowerLevel", "Print average TC size", bPrintTcPowerLevel);
	cmd.AddValue("bIsolationAttack", "Execute isolation attack by a node", bIsolationAttack);
	cmd.AddValue("bEnableFictive", "Activate fictive defence mode", bEnableFictive);
	cmd.AddValue("bConnectivityPrecentage", "Print connectivity precetage every X seconds", bConnectivityPrecentage);
	cmd.AddValue("bUdpServer", "Try to send Udp packets from a node to the attacked node", bUdpServer);
	//cmd.AddValue("dMaxSpeed", "Maximum mobility speed", dMaxSpeed);
	cmd.AddValue("bHighMovement", "Increase maximum mobility speed", bHighMovement);
	cmd.Parse (argc, argv);

	if (nSimulationSeconds >= 10.0) dSimulationSeconds = nSimulationSeconds; // Force minimum time
	if (nMaxGridX >= 10.0) dMaxGridX = nMaxGridX; // Force minimum size. Revert to default.
	if (nMaxGridY >= 10.0) dMaxGridY = nMaxGridY; // Force minimum size. Revert to default.

	// Build network
	NodeContainer nodes;
	nodes.Create (nNodes - 8);
	NodeContainer evilNodes;
	evilNodes.Create (8);
		
	// Add wifi
	WifiHelper wifi;
	//wifi.SetStandard(WIFI_PHY_STANDARD_80211g);
	YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default ();
	YansWifiPhyHelper wifiPhy = YansWifiPhyHelper::Default ();
	NqosWifiMacHelper wifiMac = NqosWifiMacHelper::Default ();
	wifiPhy.Set("RxGain", DoubleValue(12.4));
	wifiChannel.AddPropagationLoss ("ns3::RangePropagationLossModel", "MaxRange", DoubleValue (250));
	wifiPhy.SetChannel(wifiChannel.Create());
	wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager");
	wifiMac.SetType ("ns3::AdhocWifiMac");
	NetDeviceContainer adhocDevices = wifi.Install (wifiPhy, wifiMac, nodes);
	adhocDevices.Add (wifi.Install (wifiPhy, wifiMac, evilNodes));

	// Install IOLSR / DSDV
	IOlsrHelper iolsr;
	DsdvHelper dsdv;
	Ipv4ListRoutingHelper routeList;
	InternetStackHelper internet;

	// ... Stream file name
	std::stringstream tmpStringStream;
	std::string fName = "Stable_Network_Stream";
	fName += "_n";
	tmpStringStream << nNodes;
	fName += tmpStringStream.str();
	tmpStringStream.str("");
	fName += "_x";
	tmpStringStream << nMaxGridX;
	fName += tmpStringStream.str();
	tmpStringStream.str("");
	fName += "_y";
	tmpStringStream << nMaxGridY;
	fName += tmpStringStream.str();
	tmpStringStream.str("");
	fName += "_r";
	tmpStringStream << RngSeedManager::GetRun();
	fName += tmpStringStream.str();
	tmpStringStream.str("");
	fName += ".txt";
	//Ptr<OutputStreamWrapper> stream = Create<OutputStreamWrapper>("Stable_Network_Stream_Run",std::ios::out);	

	switch (nProtocol){
		case 0:
			routeList.Add (iolsr, 100);
			if (!bPrintAll){
				//iolsr.PrintRoutingTableEvery(Seconds(10.0), nodes.Get(1), stream);
			} else {
				Ptr<OutputStreamWrapper> stream = Create<OutputStreamWrapper>(fName,std::ios::out);	
				iolsr.PrintRoutingTableAllEvery(Seconds(10.0), stream);
			}
			break;
		case 1:
			routeList.Add (dsdv, 100);
			if (!bPrintAll) {
				//dsdv.PrintRoutingTableEvery(Seconds(10.0), nodes.Get(1), stream);
			} else {
				Ptr<OutputStreamWrapper> stream = Create<OutputStreamWrapper>(fName,std::ios::out);	
				dsdv.PrintRoutingTableAllEvery(Seconds(10.0), stream);
			}
			break;
		default:
			NS_FATAL_ERROR ("Invalid routing protocol chosen " << nProtocol);
			break;
	}
	internet.SetRoutingHelper(routeList);
	internet.Install (nodes);
	internet.Install (evilNodes);

	// Install IP
	Ipv4AddressHelper addresses;
	addresses.SetBase ("10.0.0.0", "255.0.0.0");
	Ipv4InterfaceContainer interfaces;
	interfaces = addresses.Assign (adhocDevices);

	// Install mobility
	//  -- Normal nodes
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
		/*mobility.SetMobilityModel ("ns3::RandomWalk2dMobilityModel",
				"Bounds", RectangleValue (Rectangle (0, dMaxGridX, 0, dMaxGridY)),
				"Speed", StringValue("ns3::UniformRandomVariable[Min=0.0|Max=2.0]"),
				"Time", TimeValue(Seconds(3.0)),
				"Mode", EnumValue(RandomWalk2dMobilityModel::MODE_TIME));
		*/
		/*
		Ptr<RandomWalk2dMobilityModel> movement = CreateObject<RandomWalk2dMobilityModel> ();
		Ptr<UniformRandomVariable> randomSpeed = CreateObject<UniformRandomVariable> ();
		randomSpeed->SetAttribute ("Min", DoubleValue (0));
		randomSpeed->SetAttribute ("Max", DoubleValue (dMaxSpeed));

		movement->SetAttribute ("Time", TimeValue(Seconds(3.0)));
		movement->SetAttribute ("Mode", EnumValue(RandomWalk2dMobilityModel::MODE_TIME));
		movement->SetAttribute ("Speed", randomSpeed);

		mobility.SetMobilityModel (movement);
		*/
		if (bHighMovement){
			mobility.SetMobilityModel ("ns3::RandomWalk2dMobilityModel",
					"Bounds", RectangleValue (Rectangle (0, dMaxGridX, 0, dMaxGridY)),
					"Speed", StringValue("ns3::UniformRandomVariable[Min=0.5|Max=10.0]"),
					"Time", TimeValue(Seconds(3.0)),
					"Mode", EnumValue(RandomWalk2dMobilityModel::MODE_TIME));
		} else {
			mobility.SetMobilityModel ("ns3::RandomWalk2dMobilityModel",
					"Bounds", RectangleValue (Rectangle (0, dMaxGridX, 0, dMaxGridY)),
					"Speed", StringValue("ns3::UniformRandomVariable[Min=1.5|Max=2.0]"),
					"Time", TimeValue(Seconds(3.0)),
					"Mode", EnumValue(RandomWalk2dMobilityModel::MODE_TIME));
		}
	} else {
		mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
	}
	mobility.Install (nodes);

	// -- Evil nodes
	
	mobility.SetPositionAllocator ("ns3::GridPositionAllocator",
								   /*"MinX", DoubleValue (0.0),
								   "MinY", DoubleValue (200.0),
								   "DeltaX", DoubleValue (200.0),
								   "DeltaY", DoubleValue (300.0),
								   "GridWidth", UintegerValue (dMaxGridX / 200),*/
								   "MinX", DoubleValue (100.0),
								   "MinY", DoubleValue (200.0),
								   "DeltaX", DoubleValue (250.0),
								   "DeltaY", DoubleValue (250.0),
								   //"GridWidth", UintegerValue (dMaxGridX / 200),
								   "GridWidth", UintegerValue (4),
								   "LayoutType", StringValue ("RowFirst"));
	mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
	
	mobility.Install (evilNodes);

	// Fix position for Sending node and receiving node:
	/*
	Vector vec;
	vec.x = 1;
	vec.y = 1;
	nodes.Get(0)->GetObject<MobilityModel>()->SetPosition(vec);
	vec.x = nMaxGridX - 1;
	vec.y = nMaxGridY - 1;
	nodes.Get(1)->GetObject<MobilityModel>()->SetPosition(vec);
	*/
	
	


	if (bUdpServer){
		UdpServerHelper udpServerHelper(80);
		ApplicationContainer apps = udpServerHelper.Install(nodes.Get(0));
		Ptr<UdpServer> udpServer = udpServerHelper.GetServer();
		UdpClientHelper udpClientHelper(Ipv4Address("10.0.0.1"), 80);
		udpClientHelper.SetAttribute("Interval", TimeValue(Seconds(7)));
		udpClientHelper.SetAttribute("MaxPackets", UintegerValue(18));

		
		/*
		// Choose random node to become sender (From the good guys)
		Ptr<UniformRandomVariable> rnd = CreateObject<UniformRandomVariable> ();
		rnd->SetAttribute ("Min", DoubleValue (1));
		rnd->SetAttribute ("Max", DoubleValue (nodes.GetN()-1));
		

		// Install UdpClient on said node
		uint32_t sendingNode = rnd->GetInteger();
		*/
		uint32_t sendingNode = 1;
		apps.Add(udpClientHelper.Install(nodes.Get(sendingNode)));
		//apps.Add(udpClientHelper.Install(nodes.Get(1)));
		apps.Start(Seconds(60));
		apps.Stop(Seconds(1024));
		if (true){
		for (size_t i=30; i<nSimulationSeconds-5; i=i+10){
			Simulator::Schedule(Seconds (i), &AbortOnNeighbor, nodes.Get(sendingNode), Ipv4Address("10.0.0.1"));
		}
		}
		Simulator::Schedule(Seconds (250), &PrintReceivedPackets, udpServer);
	}

	if (bPrintFakeCount) {
		//Ptr<OutputStreamWrapper> wrap = Create<OutputStreamWrapper>("StableNetwork-Mod-FakeNodes.txt", ios::out);
		Simulator::Schedule(Seconds (120), &PrintCountFakeNodes, &nodes);
		Simulator::Schedule(Seconds (240), &PrintCountFakeNodes, &nodes);
	}
	if (bPrintMprFraction) {
		Simulator::Schedule(Seconds (120), &PrintMprFraction, &nodes);
		Simulator::Schedule(Seconds (240), &PrintMprFraction, &nodes);
	}
	if (bPrintTcPowerLevel) {
		Simulator::Schedule(Seconds (120), &PrintTcPowerLevel, &nodes);
		Simulator::Schedule(Seconds (240), &PrintTcPowerLevel, &nodes);
	}
	if (bIsolationAttack){
		Simulator::Schedule(Seconds (0), &ExecuteIsolationAttack, &evilNodes);
	}
	if (bEnableFictive){
		Simulator::Schedule(Seconds (0), &ActivateFictiveDefence, &nodes);
		Simulator::Schedule(Seconds (0), &ActivateFictiveDefence, &evilNodes);
		//Simulator::Schedule(Seconds (121), &ActivateFictiveDefence, &nodes);
	}
	if (bConnectivityPrecentage){
		Simulator::Schedule(Seconds (10), &PercentageWithFullConnectivity, &nodes);
	}

	// Print Topology Set
	//Simulator::Schedule(Seconds (35) , &PrintTopologySet, &nodes);
	
	// Animation
	//AnimationInterface anim ("Stable_Network_animation.xml");
	//anim.SetMobilityPollInterval (Seconds (1));

	// Pcap
	//wifiPhy.EnablePcapAll ("Stable_Network_");
	
	// Run simulation
	NS_LOG_INFO ("Run simulation.");
	Simulator::Stop (Seconds (dSimulationSeconds));

	Simulator::Run ();
	Simulator::Destroy ();

	return 0;
}
