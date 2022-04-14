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

void PrintNeighborsToStream(NodeContainer* nodes, Ptr<OutputStreamWrapper> streamWrapper){
	// Unpack stream
	std::ostream *stream = streamWrapper->GetStream();

	// Print Time!
	*stream << Simulator::Now() << std::endl;

	// Print neighborset for everyone
	for (size_t i=0; i< nodes->GetN(); ++i){
		*stream << "Node: " << i << std::endl;
		nodes->Get(i)->GetObject<RoutingProtocol>()->PrintNeighbors(streamWrapper);
		*stream << "\n\n";
	}

	
}

void PrintTcToStream(NodeContainer* nodes, Ptr<OutputStreamWrapper> streamWrapper){
	// Unpack stream
	std::ostream *stream = streamWrapper->GetStream();

	// Print Time!
	*stream << Simulator::Now() << std::endl;

	// Print TC for everyone
	for (size_t i=0; i< nodes->GetN(); ++i){
		*stream << "Node: " << i << std::endl;
		nodes->Get(i)->GetObject<RoutingProtocol>()->PrintTC(streamWrapper);
		*stream << "\n\n";
	}
}

void PrintSelectedTcToStream(NodeContainer* nodes, Ptr<OutputStreamWrapper> streamWrapper, std::list<size_t> numbers){
	// Unpack stream
	std::ostream *stream = streamWrapper->GetStream();

	// Print Time!
	*stream << Simulator::Now() << std::endl;

	// Print TC for everyone
	//for (size_t i=0; i< nodes->GetN(); ++i){
	for (std::list<size_t>::const_iterator it = numbers.begin(); it != numbers.end(); ++it){
		size_t i = *it;
		*stream << "Node: " << i << std::endl;
		nodes->Get(i)->GetObject<RoutingProtocol>()->PrintTC(streamWrapper);
		*stream << "\n\n";
	}
}

void PrintTwoHopsToStream(NodeContainer* nodes, Ptr<OutputStreamWrapper> streamWrapper){
	// Unpack stream
	std::ostream *stream = streamWrapper->GetStream();

	// Print Time!
	*stream << Simulator::Now() << std::endl;

	// Print TwoHops for everyone
	for (size_t i=0; i< nodes->GetN(); ++i){
		*stream << "Node: " << i << std::endl;
		nodes->Get(i)->GetObject<RoutingProtocol>()->PrintTwoHops(streamWrapper);
		*stream << "\n\n";
	}
}

// For x who x appointed as MPR
void PrintMprToStream(NodeContainer* nodes, Ptr<OutputStreamWrapper> streamWrapper){
	// Unpack stream
	std::ostream *stream = streamWrapper->GetStream();

	// Print Time!
	*stream << Simulator::Now() << std::endl;

	// Print TwoHops for everyone
	for (size_t i=0; i< nodes->GetN(); ++i){
		*stream << "Node: " << i << std::endl;
		nodes->Get(i)->GetObject<RoutingProtocol>()->PrintMPR(streamWrapper);
		*stream << "\n\n";
	}
}

// Sink for having package pass through
//void PacketTrace (const Packet p){
//	std::count << p;
//}

//static void AbortOnNeighbor (Ptr<Node> node, Ipv4Address address){
//	if (node->GetObject<RoutingProtocol>()->isItUpToTwoHop(address)){
//		Simulator::Stop();
//	}
//}
std::vector<std::pair<double, Ipv4Address> >
GetRealPathes (NodeContainer* cont, Ipv4Address s, Ipv4Address t){
	// Algorithm 1
	// Build directed graph from Hello
	std::map<Ipv4Address, std::list<Ipv4Address> > adjg;
	for (size_t i=0;i<cont->GetN();++i){
		const std::pair<Ipv4Address, std::vector<Ipv4Address> > r = cont->Get(i)->GetObject<RoutingProtocol>()->GetSymNeighbors();
		for (std::vector<Ipv4Address>::const_iterator it = r.second.begin(); it != r.second.end(); ++it){
			adjg[r.first].push_back(*it);
		}
	}

	// Shouldn't be needed as there shouldn't be duplicates, but just to make sure
	for (std::map<Ipv4Address, std::list<Ipv4Address> >::iterator it = adjg.begin(); it != adjg.end(); ++it){
		it->second.sort();
		it->second.unique();
	}

	// Algorithm 2
	// Create DAG
	std::map<Ipv4Address, std::list<Ipv4Address> > adjc;
	//int k = 0;
	std::list<Ipv4Address> topsort;
	{
		std::map<Ipv4Address, bool> c; // Color 0 == not visited, 1 == visited
		std::map<Ipv4Address, int> d; // Distance. -1 = infinity
		for (std::map<Ipv4Address, std::list<Ipv4Address> >::const_iterator it = adjg.begin(); it != adjg.end(); ++it){
			c[it->first] = false;
			//d[it->first] = -1;
			d[it->first] = std::numeric_limits<int>::max() - 1; // Highest value. Close enough to infinity
		}
		d[t] = std::numeric_limits<int>::max() - 1; // Because who said t got selected as MPR
		c[t] = false;
		c[s] = true;
		d[s] = 0;
		std::queue<Ipv4Address> q;
		q.push(s);
		while (!q.empty()){
			Ipv4Address u = q.front();
			topsort.push_back(u);
			q.pop();
			//std::cout << "u: " << u << " d[u]: " << d[u] << " d[t]: " << d[t] << "\n";
			if ((u != t) && (d[u]+1 <= d[t])) {
				//std::cout << "If is ok\n";
				for (std::list<Ipv4Address>::const_iterator it = adjg[u].begin(); it != adjg[u].end(); ++it){
					const Ipv4Address &v = *it;
					if (!c[v]) {
						c[v] = true;
						d[v] = d[u] + 1;
						adjc[u].push_back(v);
						q.push(v);
					} else if (d[u] < d[v]) {
						adjc[u].push_back(v);
					}
				}
			}
		}
	//	k = d[t];
	}
	//std::cout << "Real K = " << k << "\n";
	// Algorithm 3
	// Count paths
	std::map<Ipv4Address, int> paths; 
	{
		std::map<Ipv4Address, int> pre; 
		std::map<Ipv4Address, int> post; 

		// Init
		for (std::map<Ipv4Address, std::list<Ipv4Address> >::const_iterator it = adjc.begin(); it != adjc.end(); ++it){
			pre[it->first] = 0;
			post[it->first] = 0;
		}
		pre[s] = 1;
		post[t] = 1;

		// Count
		for (std::list<Ipv4Address>::const_iterator it = topsort.begin(); it != topsort.end(); ++it){
			const Ipv4Address &x = *it;
			for (std::list<Ipv4Address>::const_iterator it2 = adjc[x].begin(); it2 != adjc[x].end(); ++it2){
				const Ipv4Address &y = *it2;
				pre[y] += pre[x];
			}
		}
		for (std::list<Ipv4Address>::const_reverse_iterator it = topsort.rbegin(); it != topsort.rend(); ++it){
			const Ipv4Address &x = *it;
			for (std::list<Ipv4Address>::const_iterator it2 = adjc[x].begin(); it2 != adjc[x].end(); ++it2){
				const Ipv4Address &y = *it2;
				post[x] += post[y];
			}
		}

		for (std::map<Ipv4Address, std::list<Ipv4Address> >::const_iterator it = adjc.begin(); it != adjc.end(); ++it){
			const Ipv4Address &x = it->first;
			paths[x] = pre[x] * post[x];
		}
	}
	
	// Algorithm 4
	// Return sorted vector from Ip that has most paths to Ip that has least (not including s t)
	// first is precentege, second is ip
	std::vector<std::pair<double, Ipv4Address> > result;
	result.reserve(paths.size());
	if (paths[s] != paths[t]) {
		std::cout << "paths[s] != paths[t]\n";
		return std::vector<std::pair<double, Ipv4Address> >(); // Abort!
	}
	//double m(paths[s]);
	for (std::map<Ipv4Address, int>::const_iterator it = paths.begin(); it != paths.end(); ++it){
		//if ((it->first == s) || (it->first == t)) continue;
		//
		// Remove addresses that are lastAddr (They aren't used for packet travel)
		// if (m_state.FindTopologyTuple(s,it->first)) continue;

		// Rig lastAddr...
		if (it->second == 0) continue;
		if ((it->first == s) || (it->first == t)) {
			result.push_back(std::make_pair(it->second, it->first.Get()+65536*2));
		} else {
			result.push_back(std::make_pair(it->second, it->first));
		}
	}
	std::sort(result.rbegin(), result.rend());
	return result;
}

void MakeBet (NodeContainer* cont, Ipv4Address s, Ipv4Address t){
	std::cout << Simulator::Now().GetSeconds() << "s:\n";
	std::vector<std::pair<double, Ipv4Address> > bets = cont->Get(2)->GetObject<RoutingProtocol>()->FindPotentionalBottleNeck(s, t);
	//std::vector<std::pair<double, Ipv4Address> > bets = cont->Get(1)->GetObject<RoutingProtocol>()->FindPotentionalBottleNeck(s, t);
	if (bets.empty() || bets.size() == 0) {
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

	// ** Check for true/false positive negetive
	std::vector<std::pair<double, Ipv4Address> > real = GetRealPathes(cont, s, t);
	if (real.empty() || real.size() == 0){
		std::cout << "Error - no paths\n\n";
		return;
	}
	std::vector<Ipv4Address> maxBets;
	std::vector<Ipv4Address> maxReal;
	// Get max values that aren't s/t
	{
		double m = bets[0].first;
		size_t i = 0;
		do{
			if (bets[i].second != Ipv4Address(s.Get()+65536*2) && bets[i].second != Ipv4Address(t.Get()+65536*2)){
				maxBets.push_back(bets[i].second);
			}
			++i;
		} while (bets[i].first == m);
	}
	{
		double m = real[0].first;
		size_t i = 0;
		do{
			if (real[i].second != Ipv4Address(s.Get()+65536*2) && real[i].second != Ipv4Address(t.Get()+65536*2)){
				maxReal.push_back(real[i].second);
			}
			++i;
		} while (real[i].first == m);
	}
	std::cout << "Result - ";
	if (maxReal.size() == 0 && maxBets.size() == 0){
		std::cout << "True Negative (No bottle neck)";
	} else if (maxReal.size() == 0 && maxBets.size() > 0){
		std::cout << "False Positive (No bottle neck)";
	} else if (maxReal.size() > 0 && maxBets.size() == 0){
		std::cout << "False Negative (Bottle neck not found)";
	} else if (maxReal.size() > 0 && maxBets.size() > 0){
		bool found = false;
		for (size_t i=0; i<maxBets.size(); ++i){
			if ((std::find(maxReal.begin(), maxReal.end(), maxBets[i]) != maxReal.end()) ||
				(std::find(maxReal.begin(), maxReal.end(), Ipv4Address(maxBets[i].Get()-65536)) != maxReal.end())){
				found = true;
				break;
			}
		}
		if (found){
			std::cout << "True Positive (Found bottle neck)\n";
		} else {
			std::cout << "False Positive (Detected wrong bottle neck)\n";
			/*std::cout << "Failure (Detected wrong bottle neck)\n";
			for (size_t i = 0; i < real.size(); ++i){
				if (real[i].first > 0)
					std::cout << real[i].second << "  -  " << real[i].first << std::endl;
			}*/
		}
		for (std::vector<Ipv4Address>::const_iterator it=maxReal.begin();it!=maxReal.end();++it){
			std::cout << "Real bottleneck - " << *it << std::endl;
		}

	}
	std::cout << "\n\n";
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
	bool bPrintNeighbors = false;
	bool bPrintTC = false;
	bool bPrintSelectedTC = false;
	bool bPrintMPR = false;
	bool bPrintTwoHops = false;
	bool bRigPath = false;

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
	cmd.AddValue("bPrintNeighbors", "Print Neighbor tables", bPrintNeighbors);
	cmd.AddValue("bPrintTC", "Print TC tables", bPrintTC);
	cmd.AddValue("bPrintSelectedTC", "Print TC tables for specified nodes", bPrintSelectedTC);
	cmd.AddValue("bPrintMPR", "Print MPR selections", bPrintMPR);
	cmd.AddValue("bPrintTwoHops", "Print TwoHop tables", bPrintTwoHops);
	cmd.AddValue("bRigPath", "Rig forwarding of packets", bRigPath);
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
		wifiPhy.Set("TxGain", DoubleValue(13));
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
				Ptr<OutputStreamWrapper> stream = Create<OutputStreamWrapper>("routing_solsr.txt",std::ios::out);	
				solsr.PrintRoutingTableAllEvery(Seconds(60.0), stream);
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
	randomGridX->SetAttribute ("Min", DoubleValue (11));
	randomGridX->SetAttribute ("Max", DoubleValue (dMaxGridX-11));
	randomGridY->SetAttribute ("Min", DoubleValue (11));
	randomGridY->SetAttribute ("Max", DoubleValue (dMaxGridY-11));

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

		// Install on all but nodes
		NodeContainer nodestmp;
		for (size_t i=2;i<nodes.GetN();++i){
			nodestmp.Add(nodes.Get(i));
		}
		mobility.Install (nodestmp);

		mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
		mobility.Install (nodes.Get(0));
		mobility.Install (nodes.Get(1));
		Vector vec;
		vec.x = 50;
		vec.y = 50;
		nodes.Get(0)->GetObject<MobilityModel>()->SetPosition(vec);		
		vec.x = dMaxGridX - 50;
		vec.y = dMaxGridY - 50;
		nodes.Get(1)->GetObject<MobilityModel>()->SetPosition(vec);	
	} else {
		mobility.Install (nodes);
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

		// Extra send
	//	ApplicationContainer apps2 = udpClientHelper.Install(nodes.Get(sendingNode));
	//	apps2.Start(Seconds(61));
	//	apps2.Stop(Seconds(nSimulationSeconds));
	}

	if (bMakeBets){
		for (size_t i=60; i<nSimulationSeconds; i+=7){
			Simulator::Schedule(Seconds (i), &MakeBet, &nodes, Ipv4Address("10.0.0.2"), Ipv4Address("10.0.0.1"));
		}
	}

	if (bPrintNeighbors){
		Ptr<OutputStreamWrapper> stream = Create<OutputStreamWrapper>("neighbors_solsr.txt",std::ios::out);	
		Simulator::Schedule(Seconds (60), &PrintNeighborsToStream, &nodes, stream);
	}

	if (bPrintTC){
		Ptr<OutputStreamWrapper> stream = Create<OutputStreamWrapper>("tc_solsr.txt",std::ios::out);	
		Simulator::Schedule(Seconds (60), &PrintTcToStream, &nodes, stream);
	}
	if (bPrintSelectedTC){
		Ptr<OutputStreamWrapper> stream = Create<OutputStreamWrapper>("tcs_solsr.txt",std::ios::out);	
		std::list<size_t> numbers;
		numbers.push_back(1);
		numbers.push_back(2);
		Simulator::Schedule(Seconds (60), &PrintSelectedTcToStream, &nodes, stream, numbers);
	}
	if (bPrintMPR){
		Ptr<OutputStreamWrapper> stream = Create<OutputStreamWrapper>("mpr_solsr.txt",std::ios::out);	
		Simulator::Schedule(Seconds (60), &PrintMprToStream, &nodes, stream);
	}

	if (bPrintTwoHops){
		Ptr<OutputStreamWrapper> stream = Create<OutputStreamWrapper>("twohops_solsr.txt",std::ios::out);	
		Simulator::Schedule(Seconds (60), &PrintTwoHopsToStream, &nodes, stream);
	}

	if (bRigPath){
		for (size_t i=0;i<nodes.GetN();++i){
			nodes.Get(i)->GetObject<RoutingProtocol>()->rigSending = true;
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
