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

 
static void PrintCountFakeNodes(NodeContainer* cont){
/*
Count the number of nodes that require fake/fictive neighbor
A fake node is required when there is a risk that some node will execute an isolation attack
*/
	//ns3::iolsr::RoutingProtocol rp;
	unsigned int count = 0;
	std::cout << "Nodes who are required to advertise fictive: " ;
	for (unsigned int i=0;i<cont->GetN();++i){
		Ptr<RoutingProtocol> pt = cont->Get(i)->GetObject<RoutingProtocol>();
		//NS_LOG_INFO("Node id: " << i << "\tTime: " << Simulator::Now().GetSeconds() << "\tRequireFake? " << pt->RequireFake());
		if (pt->RequireFake()){
			++count;
			std::cout << i << ", ";
		} ;
	}
	std::cout << std::endl;
	NS_LOG_INFO("Total fakes required: " << count << " out of " << cont->GetN());
	std::cout << "Fictives req.: " << count << std::endl;
}

static void PrintNodesDeclaredFictive(NodeContainer* cont){

	//ns3::iolsr::RoutingProtocol rp;
	unsigned int count = 0;
	std::cout << "Nodes who declared fictive: ";
	for (unsigned int i=0;i<cont->GetN();++i){
		Ptr<RoutingProtocol> pt = cont->Get(i)->GetObject<RoutingProtocol>();
		//NS_LOG_INFO("Node id: " << i << "\tTime: " << Simulator::Now().GetSeconds() << "\tRequireFake? " << pt->RequireFake());
		
		if (pt->returnDeclaredFictive()){
			++count;
			std::cout << i << ", ";
		} 
	}
	std::cout << std::endl;
	NS_LOG_INFO("Total nodes declaring fictives: " << count << " out of " << cont->GetN());
	std::cout << "Total nodes declaring fictives: " << count << std::endl;
}

static void PrintMprFraction(NodeContainer* cont){
/*
For each node get the fraction of it's MPR (using FractionOfMpr: number of MPRs / number of neighbors) and sum all the fractions
Output a message of the fraction of MPR over all nodes
*/
	// double sum = 0;
	// for (unsigned int i=0;i<cont->GetN();++i){
	// 	Ptr<RoutingProtocol> pt = cont->Get(i)->GetObject<RoutingProtocol>();
	// 	//NS_LOG_INFO("Node id: " << i << "\tTime: " << Simulator::Now().GetSeconds() << "\tFraction: " << pt->FractionOfMpr());
	// 	sum += pt->FractionOfMpr();
	// }
	// NS_LOG_INFO("Total fraction of MPR: " << sum / (double) cont->GetN());
	// NS_LOG_INFO(sum / (double) cont->GetN());
	// //... NS_LOG_INFO broke. Using cout...
	// //std::cout << "Total fraction of MPR: " << sum / (double) cont->GetN() << std::endl;
	// std::cout << "MPR: " << (sum / (double) cont->GetN()) << std::endl; 

	double sum = 0;
	uint32_t countTotalNeighbors = 0;
	for (unsigned int i=0;i<cont->GetN();++i){
		Ptr<RoutingProtocol> pt = cont->Get(i)->GetObject<RoutingProtocol>();
		//NS_LOG_INFO("Node id: " << i << "\tTime: " << Simulator::Now().GetSeconds() << "\tFraction: " << pt->FractionOfMpr());
		sum += pt->getMprSize();
		countTotalNeighbors  += pt->getNeighborsSize();
	}
	// NS_LOG_INFO("Total fraction of MPR: " << sum / (double) cont->GetN());
	// NS_LOG_INFO(sum / (double) cont->GetN());
	NS_LOG_INFO("Total fraction of MPR: " << sum / (double) countTotalNeighbors);
	NS_LOG_INFO(sum / (double) countTotalNeighbors);
	//... NS_LOG_INFO broke. Using cout...
	//std::cout << "Total fraction of MPR: " << sum / (double) cont->GetN() << std::endl;
	std::cout << "Total MPRs chosen: " << sum << std::endl;
	std::cout << "Total Neighbors: " << countTotalNeighbors << std::endl;

	std::cout << "Avg MPR fraction: " << (sum / countTotalNeighbors) << std::endl;
}

static void PrintMprs(NodeContainer* cont){
	uint32_t totalMprs = 0;
	MprSet allMPRs = cont->Get(0)->GetObject<RoutingProtocol>()->getMprSet();
	for (unsigned int i=1;i<cont->GetN();++i){
		MprSet currentMprSet = cont->Get(i)->GetObject<RoutingProtocol>()->getMprSet();
		allMPRs.insert(currentMprSet.begin(), currentMprSet.end());
		
	// 	sumLsr += pt->tcPowerLevel(true);
	// 	sumOlsr += pt->tcPowerLevel(false);
	// }
	// //NS_LOG_INFO("TC Level: " << sumOlsr / (double) cont->GetN() << " (lsr: " << sumLsr / (double) cont->GetN() << ")");
	// std::cout << "TC Level: " << sumOlsr / (double) cont->GetN() << " \nlsr: " << sumLsr / (double) cont->GetN() << "\n";
	}
	totalMprs = allMPRs.size();
	std::cout << "Total MPRs: " << totalMprs << std::endl;
	std::cout << "MPR sub-network nodes: " << std::endl;
	for (MprSet::const_iterator it = allMPRs.begin(); it != allMPRs.end(); ++it){
			std::cout << *it << ", ";
		}
	std::cout << std::endl;
}

static void Print2hopNeighborsOfVictim(NodeContainer* cont){
	Ptr<RoutingProtocol> victimNode = cont->Get(0)->GetObject<RoutingProtocol>();

	const NeighborSet& neighbors = victimNode->getNeighborSet();
	const TwoHopNeighborSet& twoHops = victimNode->getTwoHopNeighborSet();
	std::list<Ipv4Address> oneHopsInList;
	std::list<Ipv4Address> twoHopsInList;
	for (NeighborSet::const_iterator it = neighbors.begin(); it != neighbors.end(); ++it){
		//if (it->twoHopNeighborAddr == m_mainAddress) continue;
      //std::cout << it->twoHopNeighborAddr << ", " ;
	  oneHopsInList.push_back(it->neighborMainAddr);
    }
	for (TwoHopNeighborSet::const_iterator it = twoHops.begin(); it != twoHops.end(); ++it){
		//if (it->twoHopNeighborAddr == m_mainAddress) continue;
      //std::cout << it->twoHopNeighborAddr << ", " ;
	  twoHopsInList.push_back(it->twoHopNeighborAddr);
    }
std::cout << "1Hop from victim: " << std::endl;
	oneHopsInList.sort();
	oneHopsInList.unique();
	for (std::list<Ipv4Address>::iterator it = oneHopsInList.begin(); it != oneHopsInList.end(); ++it){
		std::cout << *it << ", " ;
	}
	std::cout << std::endl;

	std::cout << "2Hops from victim: " << std::endl;
	twoHopsInList.sort();
	twoHopsInList.unique();
	for (std::list<Ipv4Address>::iterator it = twoHopsInList.begin(); it != twoHopsInList.end(); ++it){
		std::cout << *it << ", " ;
	}
	std::cout << std::endl;
}

static void PrintC6Detection(NodeContainer* cont){
/*
Count the number of nodes that detected themselves as part of a c6 cycle topology.
*/
	//ns3::iolsr::RoutingProtocol rp;
	unsigned int count = 0;
	std::cout << "Nodes detected as part of c6: ";
	for (unsigned int i=0;i<cont->GetN();++i){
		Ptr<RoutingProtocol> pt = cont->Get(i)->GetObject<RoutingProtocol>();
		//NS_LOG_INFO("Node id: " << i << "\tTime: " << Simulator::Now().GetSeconds() << "\tRequireFake? " << pt->RequireFake());
		if (pt->returnDetectedInC6()){
			std::cout << i << ", ";
			++count;
		}
	}
	NS_LOG_INFO("Total nodes in c6: " << count << " out of " << cont->GetN());
	std::cout << std::endl;
	std::cout << "Num. of nodes detected as part of c6: " << count << std::endl;
}

static void PrintNodeOutputLog(NodeContainer* cont, uint32_t nodeID){ //!!
	Ptr<RoutingProtocol> pt = cont->Get(nodeID)->GetObject<RoutingProtocol>();
	std::string log = pt->getOutputLog();
	std::cout << "Log for node " << nodeID << ": " << std::endl;
	std::cout << log << std::endl << std::endl;
}

static void PrintRiskyFraction(NodeContainer* cont, Ipv4Address ignore = Ipv4Address("0.0.0.0")){
	/*
	*/
	double sum = 0;
	for (unsigned int i=0;i<cont->GetN();++i){
		Ptr<RoutingProtocol> pt = cont->Get(i)->GetObject<RoutingProtocol>();
		sum += pt->FractionOfNodesMarkedAsRisky(ignore);
	}
	std::cout << "Risky nodes: " << sum << std::endl;
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
	// Make it pick a node at random later...
	Ipv4Address target = cont->Get(2)->GetObject<RoutingProtocol>()->ExecuteIsolationAttack();
	target.IsBroadcast(); // Kill the unused warning
	//NS_LOG_INFO("Executing node isolation attack on: " << target);
	//NS_LOG_INFO("Attacker address: " << cont->Get(2)->GetObject<Ipv4>()->GetAddress(1,0));
	//NS_LOG_INFO("Victim  test: " << cont->Get(25)->GetObject<Ipv4>()->GetAddress(1,0));
	std::cout << "Executing node isolation attack on: " << target << std::endl;
	std::cout << "Attacker address: " << cont->Get(2)->GetObject<Ipv4>()->GetAddress(1,0) << std::endl;
}

static void ExecuteIsolationAttackMassive(NodeContainer* cont){
	// Let the first 30% nodes attack and see what happens
	for (uint32_t i=5;i<cont->GetN() * 0.3 + 5; ++i){
		// cont->Get(i)->GetObject<RoutingProtocol>()->ExecuteIsolationAttack();
		cont->Get(i)->GetObject<RoutingProtocol>()->ExecuteIsolationAttack(Ipv4Address("10.0.0.1"));
	}
}

static void ExecuteIsolationAttackByNeighbor(NodeContainer* nodes, Ipv4Address target){
	bool found = false;
	for (uint32_t i=3; i< nodes->GetN(); ++i){
		if (nodes->Get(i)->GetObject<RoutingProtocol>()->isItNeighbor(target)){
			found = true;
			nodes->Get(i)->GetObject<RoutingProtocol>()->ExecuteIsolationAttack(target);
			std::cout << "Attacker found. Attacking from node id: " << i << std::endl;
			break;
		}
	}
	if(!found){
		std::cout << "Attacker not available. *** Terminated *** " << std::endl;
		Simulator::Stop();
	}
}

static void PercentageWithFullConnectivity(NodeContainer* cont){
	uint32_t count = 0;
	for (uint32_t i=0;i<cont->GetN();++i){
		uint32_t routingTableSize = cont->Get(i)->GetObject<RoutingProtocol>()->getRoutingTableSize();
		MprSet nodeMprSet = cont->Get(i)->GetObject<RoutingProtocol>()->getMprSet();
		
		std::cout << "Node id: "<< i << ", nodes in routing table: " << routingTableSize << ".   MPRs(" << nodeMprSet.size() <<"): ";
		for (MprSet::const_iterator it = nodeMprSet.begin(); it != nodeMprSet.end(); ++it){
			std::cout << *it << ", ";
		}
		std::cout << std::endl;
		if (routingTableSize == cont->GetN() - 1) {
			++count;
		}
	}
	double result = count / (double) cont->GetN();
	std::cout << "Routing Percentage: " << result << "\n";
	//Simulator::Schedule(Seconds (10), &PercentageWithFullConnectivity, cont);
}

static void ActivateFictiveDefence(NodeContainer* cont){
	for (unsigned int i=0; i < cont->GetN(); ++i){
		cont->Get(i)->GetObject<RoutingProtocol>()->activateFictiveDefence();
	}
}

static void ActivateFictiveMitigation(NodeContainer* cont){
	for (unsigned int i=0; i < cont->GetN(); ++i){
		cont->Get(i)->GetObject<RoutingProtocol>()->activateFictiveMitigation();
	}
	//std::cout << "Enabled new mitigation defence on all nodes." << std::endl;
}

static void ReportNumReceivedPackets(Ptr<UdpServer> udpServer){
	std::cout << "Packets: " << udpServer->GetReceived() << std::endl;
}

static void AbortIfNotReceivedPackets(Ptr<UdpServer> udpServer){
	if (udpServer->GetReceived() == 0){
		std::cout << "* Received 0 packets, terminating." << std::endl;
		Simulator::Stop();
	}
}

static void AssertConnectivity(NodeContainer* cont){
	for (unsigned int i=0; i < cont->GetN(); ++i){
		if (cont->Get(i)->GetObject<RoutingProtocol>()->getRoutingTableSize() != cont->GetN() - 1) {
			std::cout << "*** Assert connectivity failed, terminated." << std::endl;
			Simulator::Stop();
		}
	}
	//if (cont->Get(0)->GetObject<RoutingProtocol>()->getRoutingTableSize() != cont->GetN() - 1) {
	//	Simulator::Stop();
	//}
}
static void AbortOnNeighbor (Ptr<Node> node, Ipv4Address address){
	if (node->GetObject<RoutingProtocol>()->isItNeighbor(address)){
		std::cout << "*** Sending node of udp packets is a neighbor to victim. Terminated." << std::endl;
		Simulator::Stop();
	}
}

static void TrackTarget (Ptr<Node> target, Ptr<Node> tracker){
	Vector vec = target->GetObject<MobilityModel>()->GetPosition();
	vec.x += 8;
	vec.y += 0;
	tracker->GetObject<MobilityModel>()->SetPosition(vec);
}

static void PrintTables(Ptr<Node> n, std::string fname){
	std::ofstream o;
	o.open((std::string("_TwoHop") + fname + std::string(".txt")).c_str());
	const TwoHopNeighborSet &two = n->GetObject<RoutingProtocol>()->getTwoHopNeighborSet();
	for (TwoHopNeighborSet::const_iterator it = two.begin(); it!=two.end(); ++it){
		o << *it << "\n";
	}
	o.close();
	o.open((std::string("_Topology") + fname + std::string(".txt")).c_str());
	const TopologySet &tp = n->GetObject<RoutingProtocol>()->getTopologySet();
	for (TopologySet::const_iterator it = tp.begin(); it!=tp.end(); ++it){
		o << *it << "\n";
	}
	o.close();
	o.open((std::string("_Neighbor") + fname + std::string(".txt")).c_str());
	const NeighborSet &nei = n->GetObject<RoutingProtocol>()->getNeighborSet();
	for (NeighborSet::const_iterator it = nei.begin(); it!=nei.end(); ++it){
		o << *it << "\n";
	}
	o.close();
}

static void IneffectiveNeighorWrite(NodeContainer *cont){
	std::ofstream o;
	o.open ("_mpr.txt");
	//For each node write a list of it's MPRs
	for (unsigned int i=0; i< cont->GetN(); ++i){
		const MprSet mpr = cont->Get(i)->GetObject<RoutingProtocol>()->getMprSet();
		o << cont->Get(i)->GetObject<Ipv4>()->GetAddress(1,0).GetLocal() << ":";
		for (MprSet::const_iterator it = mpr.begin(); it != mpr.end(); ++it){
			o << *it << ",";
		}
		o << "\n";
	}
	o.close();
	o.open ("_neighbors.txt");
	//For each node write a list of it's 1-hop neighbors and their address
	for (unsigned int i=0; i< cont->GetN(); ++i){
		const NeighborSet neighbor = cont->Get(i)->GetObject<RoutingProtocol>()->getNeighborSet();
		o << cont->Get(i)->GetObject<Ipv4>()->GetAddress(1,0).GetLocal() << ":";
		for (NeighborSet::const_iterator it = neighbor.begin(); it != neighbor.end(); ++it){
			o << it->neighborMainAddr << ",";
		}
		o << "\n";
	}
	o.close();

}

bool bIsolationAttackBug;
bool bEnableFictive;
bool bIsolationAttackNeighbor;
bool bEnableFictiveMitigation;

static void PrintSimStats(NodeContainer* cont){

	std::cout << "@@   New Simulation   @@" << std::endl;
	std::cout << "Seed RngRun: " << RngSeedManager::GetRun() << std::endl;
	if(bIsolationAttackBug || bIsolationAttackNeighbor){
	std::cout << "Attack: ON" << std::endl;
	} else{
		std::cout << "Attack: OFF" << std::endl;
	}
	if(bEnableFictive || bEnableFictiveMitigation){
		std::cout << "Defence: ON" << std::endl;
	} else{
		std::cout << "Defence: OFF" << std::endl;
	}

	
}

/*
static void PrintTopologySet(NodeContainer* cont)
{
	ns3::iolsr::RoutingProtocol rp;
	for (unsigned int i=0;i<cont->GetN();i++)
	{
		Ptr<RoutingProtocol> pt = cont->Get(i)->GetObject<RoutingProtocol>();
		NS_LOG_INFO("Node id: "<<i<<"   "<<"Time: "<<Simulator::Now().GetSeconds());
		pt->PrintTopologySet();
	} 
}
*/
//alternative for cpp simulations:
int main2 (int argc, char *argv[]){

	//CPP version:
	if (__cplusplus == 201703L) std::cout << "C++17\n";
	    else if (__cplusplus == 201402L) std::cout << "C++14\n";
	    else if (__cplusplus == 201103L) std::cout << "C++11\n";
	    else if (__cplusplus == 199711L) std::cout << "C++98\n";
	    else std::cout << "pre-standard C++\n";


	return 0;
}

int main (int argc, char *argv[]){

	//CPP version:
	//if (__cplusplus == 201703L) std::cout << "C++17\n";
	//    else if (__cplusplus == 201402L) std::cout << "C++14\n";
	//    else if (__cplusplus == 201103L) std::cout << "C++11\n";
	//    else if (__cplusplus == 199711L) std::cout << "C++98\n";
	//    else std::cout << "pre-standard C++\n";

	// Time
	Time::SetResolution (Time::NS);
	
	// Variables
	uint32_t nNodes = 50; //Number of nodes in the simulation, default 100

	//X,Y simulation rectangle - the range of movement of the nodes in the simulation
	double dMaxGridX = 750.0; //default 500x500
	uint32_t nMaxGridX = 750;
	double dMaxGridY = 1000.0;
	uint32_t nMaxGridY = 1000;

	bool bMobility = false; //Delcares whenever there is movement in the network
	uint32_t nProtocol = 0; //IOLSR=0, DSDV=1

	// Set running time
	double dSimulationSeconds = 301.0;
	uint32_t nSimulationSeconds = 301;
	//double dSimulationSeconds = 301.0;
	//uint32_t nSimulationSeconds = 301;

	std::string mProtocolName = "Invalid";
	bool bPrintSimStats = true; //simulation stats, such as random seed used, time.
	bool bSuperTransmission = false; //Transmission boost to node X?
	bool bPrintAll = false; //Print routing table for all hops
	bool bPrintFakeCount = true; //Print amount of fake nodes required
	bool bPrintMprFraction = true; //Print fraction of MPR
	bool bPrintRiskyFraction = true; //Print fraction of risky
	bool bIsolationAttack = false; //Execute isolation attack by a node
	bIsolationAttackBug = false; //Have an attacker stick to it's target  *****
	bIsolationAttackNeighbor = true; //Execute isolation attack by a random neighbor
	bEnableFictive = true; //Activate fictive defence mode   *****
	bEnableFictiveMitigation = false; //Activate new fictive defence mode (new algorithm, mitigation)
	bool bHighRange = false; //Higher wifi range. 250m should suffice. txGain at 12.4
	bool bPrintTcPowerLevel = true; //Print average TC size
	bool bNeighborDump = false; //Neighbor dump
	bool bIsolationAttackMassive = false; //Execute isolation attack by many nodes
	bool bConnectivityPrecentage = true; //Print connectivity precetage every X seconds
	bool bUdpServer = true; //Try to send UDP packets from node1 to node0
	bool bAssertConnectivity = false; //Stop simulation if network is not fully connected, at certain time.
	bool printTotalMprs = true; //prints the MPR sub-network total MPRs, and addresses of MPRs.
	bool printDetectionInC6 = true;
	bool print2hop = true;
	bool printNodesDeclaringFictive = true;
	uint32_t printNodeOutputLog = 999; // 999 to disable this function, otherwise supply node ID.
	uint32_t assertConnectivityAtTime = 160; //The time for simulation termination if network is not fully connected.
	uint32_t startAttackTime = 150; //The time for an attacker to start attacking.
	uint32_t startDefenceTime = 60; //The time for defense to start.
	uint32_t startUdpSend = 250; //The time to start sending messages to victim. total 18 msgs. 4sec intervals. 72seconds to complete.
	uint32_t reportStatsAtTime = 295; //The time for simulation to report statistics and data to cout << .
	

	// Parameters from command line
	CommandLine cmd;
	cmd.AddValue("nNodes", "Number of nodes in the simulation", nNodes);
	cmd.AddValue("nMaxGridX", "X of the simulation rectangle", nMaxGridX);
	cmd.AddValue("nMaxGridY", "Y of the simulation rectangle", nMaxGridY);
	cmd.AddValue("bMobility", "Delcares whenever there is movement in the network", bMobility);
	cmd.AddValue("nProtocol", "IOLSR=0, DSDV=1", nProtocol);
	cmd.AddValue("nSimulationSeconds", "Amount of seconds to run the simulation", nSimulationSeconds);
	cmd.AddValue("bSuperTransmission", "Transmission boost to node X?", bSuperTransmission);
	cmd.AddValue("bPrintAll", "Print routing table for all hops", bPrintAll);
	cmd.AddValue("bPrintFakeCount", "Print amount of fake nodes required", bPrintFakeCount);
	cmd.AddValue("bPrintMprFraction", "Print fraction of MPR", bPrintMprFraction);
	cmd.AddValue("bPrintRiskyFraction", "Print fraction of risky", bPrintRiskyFraction);
	cmd.AddValue("bPrintTcPowerLevel", "Print average TC size", bPrintTcPowerLevel);
	cmd.AddValue("bIsolationAttack", "Execute isolation attack by a node", bIsolationAttack);
	cmd.AddValue("bIsolationAttackNeighbor", "Execute isolation attack by a random neighbor", bIsolationAttackNeighbor);
	cmd.AddValue("bIsolationAttackMassive", "Execute isolation attack by many nodes", bIsolationAttackMassive);
	cmd.AddValue("bEnableFictive", "Activate fictive defence mode", bEnableFictive);
	cmd.AddValue("bEnableFictiveMitigation", "Activate new fictive defence mode (new algorithm, mitigation)", bEnableFictive);
	cmd.AddValue("bHighRange", "Higher wifi range", bHighRange);
	cmd.AddValue("bNeighborDump", "Neighbor dump", bNeighborDump);
	cmd.AddValue("bConnectivityPrecentage", "Print connectivity precetage every X seconds", bConnectivityPrecentage);
	cmd.AddValue("bIsolationAttackBug", "Have an attacker stick to it's target", bIsolationAttackBug);
	cmd.AddValue("bUdpServer", "Try to send UDP packets from node1 to node0", bUdpServer);
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
		wifiPhy.Set("TxGain", DoubleValue(12.4));
		wifiChannel.AddPropagationLoss ("ns3::RangePropagationLossModel", "MaxRange", DoubleValue (190));
	}
	wifiPhy.SetChannel(wifiChannel.Create());
	wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager");
	wifiMac.SetType ("ns3::AdhocWifiMac");
	//wifiChannel.AddPropagationLoss ("ns3::RangePropagationLossModel", "MaxRange", DoubleValue (105));
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

	// Install IOLSR / DSDV
	IOlsrHelper iolsr;
	DsdvHelper dsdv;
	Ipv4ListRoutingHelper routeList;
	InternetStackHelper internet;
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
	//Ptr<OutputStreamWrapper> stream = Create<OutputStreamWrapper>(fName,std::ios::out);	

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


	if (bUdpServer){
		UdpServerHelper udpServerHelper(80);
		ApplicationContainer apps = udpServerHelper.Install(nodes.Get(0));
		Ptr<UdpServer> udpServer = udpServerHelper.GetServer();
		UdpClientHelper udpClientHelper(Ipv4Address("10.0.0.1"), 80);
		udpClientHelper.SetAttribute("Interval", TimeValue(Seconds(2))); //The time to wait between packets
		udpClientHelper.SetAttribute("MaxPackets", UintegerValue(18)); //The maximum number of packets the application will send
		udpClientHelper.SetAttribute("PacketSize", UintegerValue(512));

		
		// Choose random node to become sender
		//Ptr<UniformRandomVariable> rnd = CreateObject<UniformRandomVariable> ();
		//rnd->SetAttribute ("Min", DoubleValue (3));
		//rnd->SetAttribute ("Max", DoubleValue (nodes.GetN()-1));
		

		// Install UdpClient on said node
		//uint32_t sendingNode = rnd->GetInteger();
		uint32_t sendingNode = 1;
		apps.Add(udpClientHelper.Install(nodes.Get(sendingNode)));
		//apps.Add(udpClientHelper.Install(nodes.Get(1)));
		apps.Start(Seconds(startUdpSend)); 
		apps.Stop(Seconds(nSimulationSeconds)); //at the end of simulation
//		if (false){
//			for (size_t i=30; i<nSimulationSeconds-5; i=i+1){
		Simulator::Schedule(Seconds (startUdpSend-2), &AbortOnNeighbor, nodes.Get(sendingNode), Ipv4Address("10.0.0.1"));
//			}
//		}
		Simulator::Schedule(Seconds (nSimulationSeconds+2), &AbortIfNotReceivedPackets, udpServer); //(currently, never)
		Simulator::Schedule(Seconds (reportStatsAtTime+0.01), &ReportNumReceivedPackets, udpServer);
	}


	if(bPrintSimStats){
		Simulator::Schedule(Seconds (5), &PrintSimStats, &nodes);
	}
	
	if(printDetectionInC6){
		Simulator::Schedule(Seconds (reportStatsAtTime+0.02), &PrintC6Detection, &nodes);
	}

	if (bPrintFakeCount) {
	// Execute PrintCountFakeNodes after the delay time
		//Ptr<OutputStreamWrapper> wrap = Create<OutputStreamWrapper>("StableNetwork-Mod-FakeNodes.txt", ios::out);
		Simulator::Schedule(Seconds (reportStatsAtTime), &PrintCountFakeNodes, &nodes);
		//Simulator::Schedule(Seconds (240), &PrintCountFakeNodes, &nodes);
	}

	if(printNodeOutputLog != 999){
		Simulator::Schedule(Seconds (reportStatsAtTime+0.3), &PrintNodeOutputLog, &nodes, printNodeOutputLog);
	}

	if (printNodesDeclaringFictive) {
	// Execute PrintCountFakeNodes after the delay time
		//Ptr<OutputStreamWrapper> wrap = Create<OutputStreamWrapper>("StableNetwork-Mod-FakeNodes.txt", ios::out);
		Simulator::Schedule(Seconds (reportStatsAtTime), &PrintNodesDeclaredFictive, &nodes);
		//Simulator::Schedule(Seconds (240), &PrintCountFakeNodes, &nodes);
	}

	if (bPrintMprFraction) {
	// Execute PrintMprFraction after the delay time 
		Simulator::Schedule(Seconds (reportStatsAtTime), &PrintMprFraction, &nodes);
		//Simulator::Schedule(Seconds (240), &PrintMprFraction, &nodes);
	}

	if(printTotalMprs){
		Simulator::Schedule(Seconds (reportStatsAtTime), &PrintMprs, &nodes);

	}

	if(print2hop){
		Simulator::Schedule(Seconds (reportStatsAtTime), &Print2hopNeighborsOfVictim, &nodes);
	}

	if (bPrintRiskyFraction) {
	// Execute PrintMprFraction after the delay time 
		Ipv4Address ignore = Ipv4Address("0.0.0.0");
		if (bIsolationAttackBug) ignore = Ipv4Address("10.0.0.3"); //If an attacker stick to it's target then we ignore 10.0.0.3
		Simulator::Schedule(Seconds (reportStatsAtTime), &PrintRiskyFraction, &nodes, ignore);
		//Simulator::Schedule(Seconds (240), &PrintRiskyFraction, &nodes, ignore);
	}
	if (bPrintTcPowerLevel) {
	// If bPrintTcPowerLevel -> Print average TC size
		Simulator::Schedule(Seconds (reportStatsAtTime), &PrintTcPowerLevel, &nodes);
		//Simulator::Schedule(Seconds (240), &PrintTcPowerLevel, &nodes);
	}
	if (bIsolationAttack){
	// If bIsolationAttack -> Execute isolation attack by a node
		Simulator::Schedule(Seconds (startAttackTime), &ExecuteIsolationAttack, &nodes);
	}
	if (bIsolationAttackMassive){
	// If bIsolationAttackMassive -> Execute isolation attack by many nodes
		Simulator::Schedule(Seconds (startAttackTime), &ExecuteIsolationAttackMassive, &nodes);
	}
	if (bIsolationAttackBug){
		// If bIsolationAttackBug -> Have an attacker stick to it's target

		
		Vector vec = nodes.Get(0)->GetObject<MobilityModel>()->GetPosition();
		vec.x += 8;
		vec.y += 0;
		nodes.Get(2)->GetObject<MobilityModel>()->SetPosition(vec);
		//DynamicCast<YansWifiPhy>(DynamicCast<WifiNetDevice>(nodes.Get(2)->GetDevice(0))->GetPhy())->SetTxGain(1.2);

		for (size_t i=1; i<nSimulationSeconds; ++i){
			Simulator::Schedule(Seconds (i), &TrackTarget, nodes.Get(0), nodes.Get(2));
			if(i == startAttackTime){
				nodes.Get(2)->GetObject<RoutingProtocol>()->ExecuteIsolationAttack(Ipv4Address("10.0.0.1"));
			}
		}
	}
	if (bIsolationAttackNeighbor){
		// If bIsolationAttackNeighbor -> Execute isolation attack by a random neighbor
			Simulator::Schedule(Seconds (startAttackTime), &ExecuteIsolationAttackByNeighbor, &nodes, Ipv4Address("10.0.0.1"));
	}
	if (bEnableFictive){
		// If bEnableFictive -> Activate fictive defence mode
		//Simulator::Schedule(Seconds (0), &ActivateFictiveDefence, &nodes);
		Simulator::Schedule(Seconds (startDefenceTime), &ActivateFictiveDefence, &nodes);
	}
	if (bEnableFictiveMitigation){ //new algo.... not used.
		// If bEnableFictiveMitigation -> Activate new fictive defence mode (new algorithm, mitigation)
		//Simulator::Schedule(Seconds (0), &ActivateFictiveMitigation, &nodes);
		Simulator::Schedule(Seconds (startDefenceTime), &ActivateFictiveMitigation, &nodes);
	}
	if (bNeighborDump){
		/* IneffectiveNeighorWrite create two txt files
		   _mpr.txt - for each node a list of it's MPRs  
		   _neighbor.txt - for each node node a list of it's 1-hop neighbors and their address 
		   */
		Simulator::Schedule(Seconds (reportStatsAtTime), &IneffectiveNeighorWrite, &nodes);
	}
	
	if (bAssertConnectivity){
		// If bAssertConnectivity -> 

		// Assert connectivity
		Simulator::Schedule(Seconds (assertConnectivityAtTime), &AssertConnectivity, &nodes); 
	}
	if (bConnectivityPrecentage){
	// If bConnectivityPrecentage -> Print connectivity precetage every X seconds
		Simulator::Schedule(Seconds (reportStatsAtTime+0.02), &PercentageWithFullConnectivity, &nodes);
	}
	if (false){
		Simulator::Schedule(Seconds (reportStatsAtTime), &PrintTables, nodes.Get(0), std::string("V")); //default 251, for all 3.
		Simulator::Schedule(Seconds (reportStatsAtTime), &PrintTables, nodes.Get(2), std::string("A"));
		Simulator::Schedule(Seconds (reportStatsAtTime), &IneffectiveNeighorWrite, &nodes);
	}

	// Print Topology Set
	//Simulator::Schedule(Seconds (35) , &PrintTopologySet, &nodes);
	
	// Animation
	AnimationInterface anim ("Stable_Network_animation.xml");
	anim.SetMobilityPollInterval (Seconds (1));

	// Pcap
	wifiPhy.EnablePcap("DelayTest_", NodeContainer(nodes.Get(0)));
	wifiPhy.EnablePcap("DelayTest_", NodeContainer(nodes.Get(1)));
	//wifiPhy.EnablePcap("DelayTest_", nodes.Get(1));
	//wifiPhy.EnablePcapAll ("DelayTest_");
	//std::ostringstream sspcap;
	//sspcap << "BottleNeck_" << "FixPos_" << bFixPos << "_";
	//wifiPhy.EnablePcapAll (sspcap.str());
	
	// Run simulation
	NS_LOG_INFO ("Run simulation.");
	Simulator::Stop (Seconds (dSimulationSeconds));

	Simulator::Run ();
	Simulator::Destroy ();

	return 0;
}
