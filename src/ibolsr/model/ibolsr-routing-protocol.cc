/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2004 Francisco J. Ros 
 * Copyright (c) 2007 INESC Porto
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
 * Authors: Francisco J. Ros  <fjrm@dif.um.es>
 *          Gustavo J. A. M. Carneiro <gjc@inescporto.pt>
 */


///
/// \brief Implementation of IBOLSR agent and related classes.
///
/// This is the main file of this software because %IBOLSR's behaviour is
/// implemented here.
///

#define NS_LOG_APPEND_CONTEXT                                   \
  if (GetObject<Node> ()) { std::clog << "[node " << GetObject<Node> ()->GetId () << "] "; }


#include "ibolsr-routing-protocol.h"
#include "ns3/socket-factory.h"
#include "ns3/udp-socket-factory.h"
#include "ns3/simulator.h"
#include "ns3/log.h"
#include "ns3/names.h"
#include "ns3/inet-socket-address.h"
#include "ns3/ipv4-routing-protocol.h"
#include "ns3/ipv4-routing-table-entry.h"
#include "ns3/ipv4-route.h"
#include "ns3/boolean.h"
#include "ns3/uinteger.h"
#include "ns3/enum.h"
#include "ns3/trace-source-accessor.h"
#include "ns3/ipv4-header.h"

#include<algorithm>
#include<stack>
#include<cassert>

/********** Useful macros **********/

///
/// \brief Gets the delay between a given time and the current time.
///
/// If given time is previous to the current one, then this macro returns
/// a number close to 0. This is used for scheduling events at a certain moment.
///
#define DELAY(time) (((time) < (Simulator::Now ())) ? Seconds (0.000001) : \
                     (time - Simulator::Now () + Seconds (0.000001)))



///
/// \brief Period at which a node must cite every link and every neighbor.
///
/// We only use this value in order to define IBOLSR_NEIGHB_HOLD_TIME.
///
#define IBOLSR_REFRESH_INTERVAL   m_helloInterval


/********** Holding times **********/

/// Neighbor holding time.
#define IBOLSR_NEIGHB_HOLD_TIME   Time (3 * IBOLSR_REFRESH_INTERVAL)
/// Top holding time.
#define IBOLSR_TOP_HOLD_TIME      Time (3 * m_tcInterval)
/// Dup holding time.
#define IBOLSR_DUP_HOLD_TIME      Seconds (30)
/// MID holding time.
#define IBOLSR_MID_HOLD_TIME      Time (3 * m_midInterval)
/// HNA holding time.
#define IBOLSR_HNA_HOLD_TIME      Time (3 * m_hnaInterval)

/********** Link types **********/

/// Unspecified link type.
#define IBOLSR_UNSPEC_LINK        0
/// Asymmetric link type.
#define IBOLSR_ASYM_LINK          1
/// Symmetric link type.
#define IBOLSR_SYM_LINK           2
/// Lost link type.
#define IBOLSR_LOST_LINK          3

/********** Neighbor types **********/

/// Not neighbor type.
#define IBOLSR_NOT_NEIGH          0
/// Symmetric neighbor type.
#define IBOLSR_SYM_NEIGH          1
/// Asymmetric neighbor type.
#define IBOLSR_MPR_NEIGH          2


/********** Willingness **********/

/// Willingness for forwarding packets from other nodes: never.
#define IBOLSR_WILL_NEVER         0
/// Willingness for forwarding packets from other nodes: low.
#define IBOLSR_WILL_LOW           1
/// Willingness for forwarding packets from other nodes: medium.
#define IBOLSR_WILL_DEFAULT       3
/// Willingness for forwarding packets from other nodes: high.
#define IBOLSR_WILL_HIGH          6
/// Willingness for forwarding packets from other nodes: always.
#define IBOLSR_WILL_ALWAYS        7


/********** Miscellaneous constants **********/

/// Maximum allowed jitter.
#define IBOLSR_MAXJITTER          (m_helloInterval.GetSeconds () / 4)
/// Maximum allowed sequence number.
#define IBOLSR_MAX_SEQ_NUM        65535
/// Random number between [0-IBOLSR_MAXJITTER] used to jitter IBOLSR packet transmission.
#define JITTER (Seconds (m_uniformRandomVariable->GetValue (0, IBOLSR_MAXJITTER)))


#define IBOLSR_PORT_NUMBER 698
/// Maximum number of messages per packet.
#define IBOLSR_MAX_MSGS           64

/// Maximum number of hellos per message (4 possible link types * 3 possible nb types).
#define IBOLSR_MAX_HELLOS         12

/// Maximum number of addresses advertised on a message.
#define IBOLSR_MAX_ADDRS          64


namespace ns3 {
namespace ibolsr {

NS_LOG_COMPONENT_DEFINE ("IBOlsrRoutingProtocol");


/********** IBOLSR class **********/

NS_OBJECT_ENSURE_REGISTERED (RoutingProtocol);

Ipv4Address RoutingProtocol::getFakeAddress(){
	Ipv4Address imaginaryAddress(m_mainAddress.Get() + 65536);
	return imaginaryAddress;
}
void RoutingProtocol::PrintTopologySet()
{
    m_state.PrintTopologySet();
}
const TwoHopNeighborSet& RoutingProtocol::getTwoHopNeighborSet(){
	return m_state.GetTwoHopNeighbors();
}
const TopologySet& RoutingProtocol::getTopologySet(){
	return m_state.GetTopologySet();
}

size_t RoutingProtocol::getRoutingTableSize(){ 
	std::map<Ipv4Address, RoutingTableEntry> tab = m_table;
	tab.erase(m_mainAddress);
	// Quick 'n dirty hack to remove fake nodes
	// That 167837697 is 10.1.0.1
	for (int i=0;i<255;++i){
		tab.erase(Ipv4Address(167837697 + i));
	}
	/*
	if (tab.size() == 100){
		for (std::map<Ipv4Address, RoutingTableEntry>::iterator it = tab.begin(); it != tab.end(); ++it){
			std::cout << it->first << " ";
		}
		std::cout << "\n\n";
	}
	*/
	return tab.size(); 
}
bool RoutingProtocol::RequireFake(){
/*
	// v2
	NeighborSet m_neighborSet = m_state.GetNeighbors();
	TwoHopNeighborSet m_twoHopNeighborSet = m_state.GetTwoHopNeighbors();
	TopologySet m_topologySet = m_state.GetTopologySet();
	// Check if there are 2 different 2hops with different 1 hop
	// A B !! C D
	for (TwoHopNeighborSet::iterator it = m_twoHopNeighborSet.begin(); it != m_twoHopNeighborSet.end(); ++it){
		for (TwoHopNeighborSet::iterator it2 = m_twoHopNeighborSet.begin(); it2 != m_twoHopNeighborSet.end(); ++it2){
			if (it->neighborMainAddr != it2->neighborMainAddr) {
				if (it->twoHopNeighborAddr != it2->twoHopNeighborAddr){
					if ((it->twoHopNeighborAddr != it2->neighborMainAddr) && (it->neighborMainAddr != it2->twoHopNeighborAddr)){
						return false;
					}
				}
			}
		}
	}

	// A B C !! D
	for (TopologySet::iterator it = m_topologySet.begin(); it != m_topologySet.end(); ++it){
		if (it->destAddr == m_mainAddress) continue;
		for (TwoHopNeighborSet::iterator it2 = m_twoHopNeighborSet.begin(); it2 != m_twoHopNeighborSet.end(); ++it2){
			if (it->lastAddr == it2->twoHopNeighborAddr) {
				for (NeighborSet::iterator it3 = m_neighborSet.begin(); it3 != m_neighborSet.end(); ++it3){
					if ((it2->neighborMainAddr != it3->neighborMainAddr) && (it->destAddr != it3->neighborMainAddr) && 
						(it2->twoHopNeighborAddr != it3->neighborMainAddr)){
						return false;
					}
				}
			}
		}
	}
	
	// Couldn't find a "fake" node (or path...)
	return true;

	*/

	// v4
	const NeighborSet &m_neighborSet = m_state.GetNeighbors();
	const TwoHopNeighborSet &m_twoHopNeighborSet = m_state.GetTwoHopNeighbors();
	// Mark all 2 hops as risky
	std::list<Ipv4Address> riskyNodes;
	/*for (TopologySet::iterator it = m_topologySet.begin(); it != m_topologySet.end(); ++it){
		if (it->destAddr == m_mainAddress) continue;
		for (TwoHopNeighborSet::iterator it2 = m_twoHopNeighborSet.begin(); it2 != m_twoHopNeighborSet.end(); ++it2){
			if (it->lastAddr == it2->twoHopNeighborAddr) {
				riskyNodes.push_back(it->destAddr);
			}
		}
	}*/
	for (TwoHopNeighborSet::const_iterator it2 = m_twoHopNeighborSet.begin(); it2 != m_twoHopNeighborSet.end(); ++it2){
		//if (it2->twoHopNeighborAddr == m_mainAddress) continue;
		riskyNodes.push_back(it2->twoHopNeighborAddr);
	}

	// Remove duplicates
	riskyNodes.sort();
	riskyNodes.unique();
	riskyNodes.remove(m_mainAddress);

	// Remove those who are 1 hop away
	for (NeighborSet::const_iterator it = m_neighborSet.begin(); it != m_neighborSet.end(); ++it){
		riskyNodes.remove(it->neighborMainAddr);
	}

	/*
	// Scan if a node is reachable via a 1 or 2 hop from the neighbors
	for (std::list<Ipv4Address>::iterator it = riskyNodes.begin(); it != riskyNodes.end(); ++it){
		if (!reachableWithDistanceLimitFromEveryNeighbor(*it)){
			it = riskyNodes.erase(it);
			--it;
		}
	}
	if (riskyNodes.empty()) return false; // No risk detected
	return true; // Risk detected, fake required
	*/

	// Scan if a node is reachable via a 1 or 2 hop from the neighbors
	for (std::list<Ipv4Address>::iterator it = riskyNodes.begin(); it != riskyNodes.end(); ++it){
		if (reachableWithDistanceLimitFromEveryNeighbor(*it)){
			return true;
		}
	}
	return false;
}

bool RoutingProtocol::reachableWithDistanceLimitFromEveryNeighbor(const Ipv4Address &dest){
	const NeighborSet &m_neighborSet = m_state.GetNeighbors();

	for (NeighborSet::const_iterator it = m_neighborSet.begin(); it != m_neighborSet.end(); ++it){
		if (!reachableWithDistanceLimitFromNeighbor(it->neighborMainAddr, dest)) return false;
	}
	return true;
}
bool RoutingProtocol::reachableWithDistanceLimitFromNeighbor(const Ipv4Address &src, const Ipv4Address &dest){
	//const NeighborSet &m_neighborSet = m_state.GetNeighbors();
	//const TwoHopNeighborSet &m_twoHopNeighborSet = m_state.GetTwoHopNeighbors();
	const TopologySet &m_topologySet = m_state.GetTopologySet();

	// Single hop?
	/*
	for (TwoHopNeighborSet::const_iterator it = m_twoHopNeighborSet.begin(); it != m_twoHopNeighborSet.end(); ++it){
		if ((it->neighborMainAddr == src) && (it->twoHopNeighborAddr == dest)) return true;
	}
	*/
	if (m_state.FindTwoHopNeighborTuple(src, dest) != NULL) return true;

	// Two hop?
	for (TopologySet::const_iterator it = m_topologySet.begin(); it != m_topologySet.end(); ++it){
		if (it->destAddr != dest) continue;
		//if (it->lastAddr == m_mainAddress) continue;
		/*for (TwoHopNeighborSet::const_iterator it2 = m_twoHopNeighborSet.begin(); it2 != m_twoHopNeighborSet.end(); ++it2){
			if ((it2->neighborMainAddr == src) && (it2->twoHopNeighborAddr == it->lastAddr)) return true;
		}*/
		if (m_state.FindTwoHopNeighborTuple(src, it->lastAddr) != NULL) return true;
	}

	//for (TwoHopNeighborSet::const_iterator it = m_twoHopNeighborSet.begin(); it != m_twoHopNeighborSet.end(); ++it){
	//	if ((it->neighborMainAddr == src) && (m_state.FindTopologyTuple(dest, it->twoHopNeighborAddr) != NULL)) return true;
	//}

	return false;
}
double RoutingProtocol::FractionOfMpr(){
	//return m_state.FractionOfMpr();
	return m_state.GetMprSet().size() / (double) m_state.GetNeighbors().size();
}
double RoutingProtocol::FractionOfNodesMarkedAsRisky(Ipv4Address ignore){
	const NeighborSet &nei = m_state.GetNeighbors();
	size_t risk=0;
	if (nei.size() == 0) return 0;
	for (NeighborSet::const_iterator n = nei.begin(); n != nei.end(); ++n){
		if ((n->risky) && (n->neighborMainAddr != ignore)){
			++risk;
		}
	}
	return risk / double(nei.size());
}

size_t RoutingProtocol::tcPowerLevel(bool lsr){
	bool fake;
	if (activeDefence){
		fake = RequireFake();
	} else {
		fake = false;
	}
    if ((lsr) && (activeDefence) && (fake)){
		const NeighborSet &m_neighborSet = m_state.GetNeighbors();
		return m_neighborSet.size() + 1;
	} else if (lsr){
		const NeighborSet &m_neighborSet = m_state.GetNeighbors();
		return m_neighborSet.size();
	} else {
		const MprSelectorSet &mpr = m_state.GetMprSelectors();
		if ((activeDefence) && (fake)){
			return mpr.size() + 1;
		} else {
			return mpr.size();
		}
	}
}

bool RoutingProtocol::isPartOf6Cycle(){
	typedef std::map<Ipv4Address, std::set<Ipv4Address> > Graph;
	// Create Graph
	Graph graph;
	for (NeighborSet::const_iterator it = m_state.GetNeighbors().begin(); it != m_state.GetNeighbors().end();++it){
		if (it->status == NeighborTuple::STATUS_SYM){
			const Ipv4Address &u = it->neighborMainAddr;
			graph[m_mainAddress].insert(u);
			graph[u].insert(m_mainAddress);
		}
	}
	if (graph[m_mainAddress].size() != 2) {
		return false; // Must have exectly 2 neighbors for cycle
	}
	for (TwoHopNeighborSet::const_iterator it = m_state.GetTwoHopNeighbors().begin(); it != m_state.GetTwoHopNeighbors().end(); ++it){
		const Ipv4Address &u = it->neighborMainAddr;
		const Ipv4Address &v = it->twoHopNeighborAddr;
		graph[u].insert(v);
		graph[v].insert(u);
	}
	for (TopologySet::const_iterator it = m_state.GetTopologySet().begin(); it != m_state.GetTopologySet().end(); ++it){
		// Only accept new TC messages TIME
		//if (it->expirationTime > Time(Simulator::Now() + 2 * m_tcInterval)){
			graph[it->lastAddr].insert(it->destAddr);
			graph[it->destAddr].insert(it->lastAddr);
		//}
	}

	{
		// This code doesn't account for fictives...
		// Both cases can be united but for sake of simplicity they are seperate.
		// Case middle:
		bool ok = true;
		std::set<Ipv4Address> parts;
		std::set<Ipv4Address> level2;
		parts.insert(m_mainAddress);
		for (std::set<Ipv4Address>::const_iterator it = graph[m_mainAddress].begin(); it != graph[m_mainAddress].end(); ++it){
			const Ipv4Address& u = *it;
			if (graph[u].size() != 2) {
				ok = false;
				break;
			}
			parts.insert(u);
			for (std::set<Ipv4Address>::const_iterator it = graph[u].begin(); it != graph[u].end(); ++it){
				if (*it != m_mainAddress) level2.insert(*it);
			}
		}
		parts.insert(level2.begin(), level2.end());
		if (ok && (level2.size() == 2) && (parts.size() == 5)) {
			// Possible middle case
			std::set<Ipv4Address>::const_iterator it = level2.begin();
			std::set<Ipv4Address> &a = graph[*it];
			++it;
			std::set<Ipv4Address> &b = graph[*it];
			std::set<Ipv4Address> intersect;
			std::set_intersection(a.begin(), a.end(), b.begin(), b.end(), std::inserter(intersect, intersect.begin()));

			std::vector<Ipv4Address> intersectionWithoutPrev;
			std::set_difference(intersect.begin(), intersect.end(), parts.begin(), parts.end(), std::inserter(intersectionWithoutPrev, intersectionWithoutPrev.begin()));
			if (intersectionWithoutPrev.size() > 0) return true;
		}

		parts.clear();
		level2.clear();

		// Case side:
		ok = true;
		Ipv4Address neiWithMany = "0.0.0.0", neiWithTwo = "0.0.0.0", neiOfNei = "0.0.0.0";
		//Ipv4Address neiWithMany = uint32_t(0), neiWithTwo = uint32_t(0), neiOfNei = uint32_t(0);
		for (std::set<Ipv4Address>::const_iterator it = graph[m_mainAddress].begin(); it != graph[m_mainAddress].end(); ++it){
			const Ipv4Address& u = *it;
			if (graph[u].size() != 2) {
				if (!ok) break; // If the second one doesn't have rank 2
				ok = false;
				neiWithMany = u;
				continue;
			}
			neiWithTwo = u;
			std::set<Ipv4Address>::const_iterator itTmp = it;
			if (neiWithMany == "0.0.0.0") neiWithMany = *(++itTmp);
			parts.insert(m_mainAddress);
			parts.insert(neiWithMany);
			parts.insert(neiWithTwo);
			for (std::set<Ipv4Address>::const_iterator it2 = graph[u].begin(); it2 != graph[u].end(); ++it2){
				const Ipv4Address& v = *it2;
				if (v == m_mainAddress) continue;
				if (graph[v].size() != 2) return false;
				neiOfNei = v;
				parts.insert(v);
			}
			if (graph[neiWithTwo].count(neiWithMany) > 0) return false;
			if (graph[neiWithMany].count(neiOfNei) > 0) return false;

			std::set<Ipv4Address> ringCandidates;
			// Gather neighbors of neiWithMany
			for (std::set<Ipv4Address>::const_iterator it = graph[neiWithMany].begin(); it != graph[neiWithMany].end(); ++it){
				std::set<Ipv4Address> tmp;
				std::set_union(ringCandidates.begin(), ringCandidates.end(), graph[*it].begin(), graph[*it].end(), std::inserter(tmp, tmp.begin()));
				ringCandidates = tmp;
			}

			std::set<Ipv4Address> intersect;
			//std::set_intersection(graph[neiWithMany].begin(), graph[neiWithMany].end(), graph[neiOfNei].begin(), graph[neiOfNei].end(), std::inserter(intersect, intersect.begin()));
			std::set_intersection(ringCandidates.begin(), ringCandidates.end(), graph[neiOfNei].begin(), graph[neiOfNei].end(), std::inserter(intersect, intersect.begin()));
			if (intersect.size() > 0) return true;
			return false;
		}

	}
	return false;
	// Original version. Code is bad.
	// DFS
	/*
	std::map<Ipv4Address, bool> c; // Color - false = not visited
	std::map<Ipv4Address, Ipv4Address> p; // Pi - Parent of node
	std::map<Ipv4Address, size_t> d; // Depth to node
	std::set<Ipv4Address> cycle; // Nodes that lead to m_mainAddress - i.e. back edges to self
	for (Graph::const_iterator it = graph.begin(); it != graph.end(); ++it){
		const Ipv4Address &x = it->first;
		c[x] = false;
		p[x] = Ipv4Address("0.0.0.0");
		d[x] = std::numeric_limits<size_t>::max();
	}
	std::stack<Ipv4Address> s;
	s.push(m_mainAddress);
	d[m_mainAddress] = 0;
	std::cout << m_mainAddress << ": \n";
	while (!s.empty()){
		const Ipv4Address u = s.top();
		s.pop();
		std::cout << u << " - " << d[u] << std::endl;
		if (!c[u]){
			if (d[u] < 5){
				c[u] = true;
				if (d[u] != 0) d[u] = d[p[u]]+1;
				for (std::set<Ipv4Address>::const_iterator it = graph[u].begin(); it != graph[u].end(); ++it){
					const Ipv4Address &v = *it;
					if (c[v] == false){
						p[v] = u;
						d[v] = d[u]+1;
						s.push(v);
					}
				}
			} else if (d[u] == 5) { 
				// D = 5 --> Last step is to m_mainAddress
				c[u] = true;
				if (graph[u].count(m_mainAddress)){
				std::cout << m_mainAddress << ": " << u << std::endl;
					cycle.insert(u);
				}
			}
		}
	}

	std::cout << cycle.size() << std::endl;
	// Scan cycle
	for (std::set<Ipv4Address>::const_iterator it = cycle.begin(); it != cycle.end(); ++it){
		size_t moreThanTwoPaths = 0;
		std::set<Ipv4Address> inCycle;
		Ipv4Address u = *it;
		for (size_t i=0; i<5; ++i){
			inCycle.insert(u);
			u = p[u];
		}
		assert(u == m_mainAddress);
		assert(inCycle.size() == 5);
		for (std::set<Ipv4Address>::const_iterator it2 = cycle.begin(); it2 != cycle.end(); ++it2){
			if (graph[*it2].size() > 2) ++moreThanTwoPaths;
		}
		if (moreThanTwoPaths <= 2) return true;
	}
	return false;
	*/
}

void RoutingProtocol::activateFictiveDefence(){
	activeDefence = true;
}
void RoutingProtocol::activateImpDefence(){
	activateFictiveDefence();
	activeImpDefence = true;
}
void RoutingProtocol::activateListenDefence(){
	listenDefence = true;
}
bool RoutingProtocol::MightBeEvil(Ipv4Address x){
	// Is it an old MPR?
	if (listenChoose.find(x) == listenChoose.end()){
		return false;
	}
	// Did enough time pass?
	if (listenChoose[x] > Simulator::Now() + Seconds(m_tcInterval)){
		return false;
	}
	// Check TC records
	TopologyTuple* t = m_state.FindTopologyTuple(m_mainAddress, x);
	if (t == NULL){
		return true;
	}
	if (Simulator::Now() - (t->expirationTime - (IBOLSR_TOP_HOLD_TIME)) > m_tcInterval){
		return true;
	}
	return false;
}
Ipv4Address RoutingProtocol::ExecuteBlackholeAttack(){
	blackholeAttack = true;
	for (NeighborSet::iterator it = m_state.GetNeighbors().begin();
			it != m_state.GetNeighbors().end(); ++it){
		if (it->status == NeighborTuple::STATUS_SYM){
			blackholeAttackTarget = it->neighborMainAddr;
			break;
		}
	}
	NS_LOG_INFO("Executing isolation attack on " << blackholeAttackTarget);
	m_willingness = IBOLSR_WILL_ALWAYS;
	return blackholeAttackTarget;
}
Ipv4Address RoutingProtocol::ExecuteBlackholeAttack(Ipv4Address addr){
	blackholeAttack = true;
	blackholeAttackTarget = addr;
	NS_LOG_INFO("Executing isolation attack on " << blackholeAttackTarget);
	m_willingness = IBOLSR_WILL_ALWAYS;
	return blackholeAttackTarget;
}

void RoutingProtocol::ExecuteBlackholeAttackColl1(Ipv4Address t, Ipv4Address f){
	blackholeAttackColl1 = true;
	blackholeAttackTarget = t;
	blackholeAttackCollFriend = f;
	m_willingness = IBOLSR_WILL_ALWAYS;
}

void RoutingProtocol::ExecuteBlackholeAttackColl2(Ipv4Address t, Ipv4Address f){
	blackholeAttackColl2 = true;
	blackholeAttackTarget = t;
	blackholeAttackCollFriend = f;
}

const MprSet RoutingProtocol::getMprSet(){
	return m_state.GetMprSet();
}
const NeighborSet& RoutingProtocol::getNeighborSet(){
	return m_state.GetNeighbors();
}
bool RoutingProtocol::isItNeighbor(Ipv4Address addr){
	/*
	const NeighborSet &nei = m_state.GetNeighbors();
	for (NeighborSet::const_iterator it = nei.begin(); it != nei.end(); ++it){
		if (it->neighborMainAddr == addr)
			return true;
	}
	return false;
	*/
	return (m_state.FindNeighborTuple(addr) != NULL);
}
bool RoutingProtocol::isItUpToTwoHop(Ipv4Address addr){
	if (isItNeighbor(addr)) return true;
	const TwoHopNeighborSet &two = m_state.GetTwoHopNeighbors();
	for (TwoHopNeighborSet::const_iterator it = two.begin(); it != two.end(); ++it){
		if (it->twoHopNeighborAddr == addr)
			return true;
	}
	return false;
}

std::pair<Ipv4Address, std::vector<Ipv4Address> > RoutingProtocol::GetSymNeighbors(){
	const NeighborSet& nei = m_state.GetNeighbors();
	std::vector<Ipv4Address> result;
	for (NeighborSet::const_iterator it = nei.begin(); it != nei.end(); ++it){
		if (it->status == NeighborTuple::STATUS_SYM){
			result.push_back(it->neighborMainAddr);
		}
	}
	return std::make_pair(m_mainAddress, result);
}

TypeId 
RoutingProtocol::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::ibolsr::RoutingProtocol")
    .SetParent<Ipv4RoutingProtocol> ()
    .AddConstructor<RoutingProtocol> ()
    .AddAttribute ("HelloInterval", "HELLO messages emission interval.",
                   TimeValue (Seconds (2)),
                   MakeTimeAccessor (&RoutingProtocol::m_helloInterval),
                   MakeTimeChecker ())
    .AddAttribute ("TcInterval", "TC messages emission interval.",
                   TimeValue (Seconds (5)),
                   MakeTimeAccessor (&RoutingProtocol::m_tcInterval),
                   MakeTimeChecker ())
    .AddAttribute ("MidInterval", "MID messages emission interval.  Normally it is equal to TcInterval.",
                   TimeValue (Seconds (5)),
                   MakeTimeAccessor (&RoutingProtocol::m_midInterval),
                   MakeTimeChecker ())
    .AddAttribute ("HnaInterval", "HNA messages emission interval.  Normally it is equal to TcInterval.",
                   TimeValue (Seconds (5)),
                   MakeTimeAccessor (&RoutingProtocol::m_hnaInterval),
                   MakeTimeChecker ())
    .AddAttribute ("Willingness", "Willingness of a node to carry and forward traffic for other nodes.",
                   EnumValue (IBOLSR_WILL_DEFAULT),
                   MakeEnumAccessor (&RoutingProtocol::m_willingness),
                   MakeEnumChecker (IBOLSR_WILL_NEVER, "never",
                                    IBOLSR_WILL_LOW, "low",
                                    IBOLSR_WILL_DEFAULT, "default",
                                    IBOLSR_WILL_HIGH, "high",
                                    IBOLSR_WILL_ALWAYS, "always"))
    .AddTraceSource ("Rx", "Receive IBOLSR packet.",
                     MakeTraceSourceAccessor (&RoutingProtocol::m_rxPacketTrace))
    .AddTraceSource ("Tx", "Send IBOLSR packet.",
                     MakeTraceSourceAccessor (&RoutingProtocol::m_txPacketTrace))
    .AddTraceSource ("RoutingTableChanged", "The IBOLSR routing table has changed.",
                     MakeTraceSourceAccessor (&RoutingProtocol::m_routingTableChanged))
  ;
  return tid;
}


RoutingProtocol::RoutingProtocol ()
  : m_routingTableAssociation (0),
    m_ipv4 (0),
    m_helloTimer (Timer::CANCEL_ON_DESTROY),
    m_tcTimer (Timer::CANCEL_ON_DESTROY),
    m_midTimer (Timer::CANCEL_ON_DESTROY),
    m_hnaTimer (Timer::CANCEL_ON_DESTROY),
    m_queuedMessagesTimer (Timer::CANCEL_ON_DESTROY)
{
  m_uniformRandomVariable = CreateObject<UniformRandomVariable> ();

  m_hnaRoutingTable = Create<Ipv4StaticRouting> ();

  blackholeAttack = false;
  activeDefence = false;
  blackholeAttackColl1 = false;
  blackholeAttackColl2 = false;

  listenDefence = false;
}

RoutingProtocol::~RoutingProtocol ()
{
}

void
RoutingProtocol::SetIpv4 (Ptr<Ipv4> ipv4)
{
  NS_ASSERT (ipv4 != 0);
  NS_ASSERT (m_ipv4 == 0);
  NS_LOG_DEBUG ("Created ibolsr::RoutingProtocol");
  m_helloTimer.SetFunction (&RoutingProtocol::HelloTimerExpire, this);
  m_tcTimer.SetFunction (&RoutingProtocol::TcTimerExpire, this);
  m_midTimer.SetFunction (&RoutingProtocol::MidTimerExpire, this);
  m_hnaTimer.SetFunction (&RoutingProtocol::HnaTimerExpire, this);
  m_queuedMessagesTimer.SetFunction (&RoutingProtocol::SendQueuedMessages, this);

  m_packetSequenceNumber = IBOLSR_MAX_SEQ_NUM;
  m_messageSequenceNumber = IBOLSR_MAX_SEQ_NUM;
  m_ansn = IBOLSR_MAX_SEQ_NUM;

  m_linkTupleTimerFirstTime = true;

  m_ipv4 = ipv4;

  m_hnaRoutingTable->SetIpv4 (ipv4);
}

void RoutingProtocol::DoDispose ()
{
  m_ipv4 = 0;
  m_hnaRoutingTable = 0;
  m_routingTableAssociation = 0;

  for (std::map< Ptr<Socket>, Ipv4InterfaceAddress >::iterator iter = m_socketAddresses.begin ();
       iter != m_socketAddresses.end (); iter++)
    {
      iter->first->Close ();
    }
  m_socketAddresses.clear ();

  Ipv4RoutingProtocol::DoDispose ();
}

void
RoutingProtocol::PrintRoutingTable (Ptr<OutputStreamWrapper> stream) const
{
  std::ostream* os = stream->GetStream ();
  *os << "Destination\t\tNextHop\t\tInterface\tDistance\n";

  for (std::map<Ipv4Address, RoutingTableEntry>::const_iterator iter = m_table.begin ();
       iter != m_table.end (); iter++)
    {
      *os << iter->first << "\t\t";
      *os << iter->second.nextAddr << "\t\t";
      if (Names::FindName (m_ipv4->GetNetDevice (iter->second.interface)) != "")
        {
          *os << Names::FindName (m_ipv4->GetNetDevice (iter->second.interface)) << "\t\t";
        }
      else
        {
          *os << iter->second.interface << "\t\t";
        }
      *os << iter->second.distance << "\t";
      *os << "\n";
    }
  // Also print the HNA routing table
  *os << " HNA Routing Table:\n";
  m_hnaRoutingTable->PrintRoutingTable (stream);
}

void RoutingProtocol::DoInitialize ()
{
  if (m_mainAddress == Ipv4Address ())
    {
      Ipv4Address loopback ("127.0.0.1");
      for (uint32_t i = 0; i < m_ipv4->GetNInterfaces (); i++)
        {
          // Use primary address, if multiple
          Ipv4Address addr = m_ipv4->GetAddress (i, 0).GetLocal ();
          if (addr != loopback)
            {
              m_mainAddress = addr;
              break;
            }
        }

      NS_ASSERT (m_mainAddress != Ipv4Address ());
    }

  NS_LOG_DEBUG ("Starting IBOLSR on node " << m_mainAddress);

  Ipv4Address loopback ("127.0.0.1");

  bool canRunIBOlsr = false;
  for (uint32_t i = 0; i < m_ipv4->GetNInterfaces (); i++)
    {
      Ipv4Address addr = m_ipv4->GetAddress (i, 0).GetLocal ();
      if (addr == loopback)
        continue;

      if (addr != m_mainAddress)
        {
          // Create never expiring interface association tuple entries for our
          // own network interfaces, so that GetMainAddress () works to
          // translate the node's own interface addresses into the main address.
          IfaceAssocTuple tuple;
          tuple.ifaceAddr = addr;
          tuple.mainAddr = m_mainAddress;
          AddIfaceAssocTuple (tuple);
          NS_ASSERT (GetMainAddress (addr) == m_mainAddress);
        }

      if(m_interfaceExclusions.find (i) != m_interfaceExclusions.end ())
        continue;

      // Create a socket to listen only on this interface
      Ptr<Socket> socket = Socket::CreateSocket (GetObject<Node> (), 
                                                 UdpSocketFactory::GetTypeId ());
      socket->SetAllowBroadcast (true);
      InetSocketAddress inetAddr (m_ipv4->GetAddress (i, 0).GetLocal (), IBOLSR_PORT_NUMBER);
      socket->SetRecvCallback (MakeCallback (&RoutingProtocol::RecvIBOlsr,  this));
      if (socket->Bind (inetAddr))
        {
          NS_FATAL_ERROR ("Failed to bind() IBOLSR socket");
        }
      socket->BindToNetDevice (m_ipv4->GetNetDevice (i));
      m_socketAddresses[socket] = m_ipv4->GetAddress (i, 0);

      canRunIBOlsr = true;
    }

  if(canRunIBOlsr)
    {
      HelloTimerExpire ();
      TcTimerExpire ();
      MidTimerExpire ();
      HnaTimerExpire ();

      NS_LOG_DEBUG ("IBOLSR on node " << m_mainAddress << " started");
    }
}

void RoutingProtocol::SetMainInterface (uint32_t interface)
{
  m_mainAddress = m_ipv4->GetAddress (interface, 0).GetLocal ();
}

void RoutingProtocol::SetInterfaceExclusions (std::set<uint32_t> exceptions)
{
  m_interfaceExclusions = exceptions;
}

//
// \brief Processes an incoming %IBOLSR packet following \RFC{3626} specification.
void
RoutingProtocol::RecvIBOlsr (Ptr<Socket> socket)
{
  Ptr<Packet> receivedPacket;
  Address sourceAddress;
  receivedPacket = socket->RecvFrom (sourceAddress);

  InetSocketAddress inetSourceAddr = InetSocketAddress::ConvertFrom (sourceAddress);
  Ipv4Address senderIfaceAddr = inetSourceAddr.GetIpv4 ();
  Ipv4Address receiverIfaceAddr = m_socketAddresses[socket].GetLocal ();
  NS_ASSERT (receiverIfaceAddr != Ipv4Address ());
  NS_LOG_DEBUG ("IBOLSR node " << m_mainAddress << " received a IBOLSR packet from "
                             << senderIfaceAddr << " to " << receiverIfaceAddr);

  // All routing messages are sent from and to port RT_PORT,
  // so we check it.
  NS_ASSERT (inetSourceAddr.GetPort () == IBOLSR_PORT_NUMBER);

  Ptr<Packet> packet = receivedPacket;

  ibolsr::PacketHeader ibolsrPacketHeader;
  packet->RemoveHeader (ibolsrPacketHeader);
  NS_ASSERT (ibolsrPacketHeader.GetPacketLength () >= ibolsrPacketHeader.GetSerializedSize ());
  uint32_t sizeLeft = ibolsrPacketHeader.GetPacketLength () - ibolsrPacketHeader.GetSerializedSize ();

  MessageList messages;

  while (sizeLeft)
    {
      MessageHeader messageHeader;
      if (packet->RemoveHeader (messageHeader) == 0)
        NS_ASSERT (false);

      sizeLeft -= messageHeader.GetSerializedSize ();

      NS_LOG_DEBUG ("IBOlsr Msg received with type "
                    << std::dec << int (messageHeader.GetMessageType ())
                    << " TTL=" << int (messageHeader.GetTimeToLive ())
                    << " origAddr=" << messageHeader.GetOriginatorAddress ());
      messages.push_back (messageHeader);
    }

  m_rxPacketTrace (ibolsrPacketHeader, messages);

  //for (MessageList::const_iterator messageIter = messages.begin ();
  for (MessageList::iterator messageIter = messages.begin ();
       messageIter != messages.end (); messageIter++)
    {
      const MessageHeader &messageHeader = *messageIter;
      // If ttl is less than or equal to zero, or
      // the receiver is the same as the originator,
      // the message must be silently dropped
      if (messageHeader.GetTimeToLive () == 0
          || messageHeader.GetOriginatorAddress () == m_mainAddress)
        {
          packet->RemoveAtStart (messageHeader.GetSerializedSize ()
                                 - messageHeader.GetSerializedSize ());
          continue;
        }

      // If the message has been processed it must not be processed again
      bool do_forwarding = true;
      DuplicateTuple *duplicated = m_state.FindDuplicateTuple
          (messageHeader.GetOriginatorAddress (),
          messageHeader.GetMessageSequenceNumber ());

      // Get main address of the peer, which may be different from the packet source address
//       const IfaceAssocTuple *ifaceAssoc = m_state.FindIfaceAssocTuple (inetSourceAddr.GetIpv4 ());
//       Ipv4Address peerMainAddress;
//       if (ifaceAssoc != NULL)
//         {
//           peerMainAddress = ifaceAssoc->mainAddr;
//         }
//       else
//         {
//           peerMainAddress = inetSourceAddr.GetIpv4 () ;
//         }

      if (duplicated == NULL)
        {
          switch (messageHeader.GetMessageType ())
            {
            case ibolsr::MessageHeader::HELLO_MESSAGE:
              NS_LOG_DEBUG (Simulator::Now ().GetSeconds ()
                            << "s IBOLSR node " << m_mainAddress
                            << " received HELLO message of size " << messageHeader.GetSerializedSize ());
              ProcessHello (messageHeader, receiverIfaceAddr, senderIfaceAddr);
              break;

            case ibolsr::MessageHeader::TC_MESSAGE:
              NS_LOG_DEBUG (Simulator::Now ().GetSeconds ()
                            << "s IBOLSR node " << m_mainAddress
                            << " received TC message of size " << messageHeader.GetSerializedSize ());
              ProcessTc (messageHeader, senderIfaceAddr);
              break;

            case ibolsr::MessageHeader::MID_MESSAGE:
              NS_LOG_DEBUG (Simulator::Now ().GetSeconds ()
                            << "s IBOLSR node " << m_mainAddress
                            <<  " received MID message of size " << messageHeader.GetSerializedSize ());
              ProcessMid (messageHeader, senderIfaceAddr);
              break;
            case ibolsr::MessageHeader::HNA_MESSAGE:
              NS_LOG_DEBUG (Simulator::Now ().GetSeconds ()
                            << "s IBOLSR node " << m_mainAddress
                            <<  " received HNA message of size " << messageHeader.GetSerializedSize ());
              ProcessHna (messageHeader, senderIfaceAddr);
              break; 

            default:
              NS_LOG_DEBUG ("IBOLSR message type " <<
                            int (messageHeader.GetMessageType ()) <<
                            " not implemented");
            }
        }
      else
        {
          NS_LOG_DEBUG ("IBOLSR message is duplicated, not reading it.");

          // If the message has been considered for forwarding, it should
          // not be retransmitted again
          for (std::vector<Ipv4Address>::const_iterator it = duplicated->ifaceList.begin ();
               it != duplicated->ifaceList.end (); it++)
            {
              if (*it == receiverIfaceAddr)
                {
                  do_forwarding = false;
                  break;
                }
            }
        }

      if (do_forwarding)
        {
          // HELLO messages are never forwarded.
          // TC and MID messages are forwarded using the default algorithm.
          // Remaining messages are also forwarded using the default algorithm.
          if (messageHeader.GetMessageType ()  != ibolsr::MessageHeader::HELLO_MESSAGE)
            {
				// Colluding isolation attack
				// Strip some data from message before sending it
				if (blackholeAttackColl2){
					if ((messageHeader.GetMessageType() == ibolsr::MessageHeader::TC_MESSAGE) && (sourceAddress == blackholeAttackCollFriend)){
						 ibolsr::MessageHeader::Tc &tc = messageIter->GetTc ();
						 std::remove(tc.neighborAddresses.begin(), tc.neighborAddresses.end(), blackholeAttackTarget);
					}
				}
              ForwardDefault (messageHeader, duplicated,
                              receiverIfaceAddr, inetSourceAddr.GetIpv4 ());
            }
        }
    }

  // After processing all IBOLSR messages, we must recompute the routing table
  RoutingTableComputation ();
}

///
/// \brief This auxiliary function (defined in \RFC{3626}) is used for calculating the MPR Set.
///
/// \param tuple the neighbor tuple which has the main address of the node we are going to calculate its degree to.
/// \return the degree of the node.
///
int
RoutingProtocol::Degree (NeighborTuple const &tuple)
{
  int degree = 0;
  for (TwoHopNeighborSet::const_iterator it = m_state.GetTwoHopNeighbors ().begin ();
       it != m_state.GetTwoHopNeighbors ().end (); it++)
    {
      TwoHopNeighborTuple const &nb2hop_tuple = *it;
      if (nb2hop_tuple.neighborMainAddr == tuple.neighborMainAddr)
        {
          const NeighborTuple *nb_tuple =
            m_state.FindNeighborTuple (nb2hop_tuple.neighborMainAddr);
          if (nb_tuple == NULL)
            degree++;
        }
    }
  return degree;
}

namespace {
///
/// \brief Remove all covered 2-hop neighbors from N2 set. This is a helper function used by MprComputation algorithm.
///
void 
CoverTwoHopNeighbors (Ipv4Address neighborMainAddr, TwoHopNeighborSet & N2)
{
  // first gather all 2-hop neighbors to be removed
  std::set<Ipv4Address> toRemove;
  for (TwoHopNeighborSet::iterator twoHopNeigh = N2.begin (); twoHopNeigh != N2.end (); twoHopNeigh++)
    {
      if (twoHopNeigh->neighborMainAddr == neighborMainAddr)
        {
          toRemove.insert (twoHopNeigh->twoHopNeighborAddr);
        }
    }
  // Now remove all matching records from N2
  for (TwoHopNeighborSet::iterator twoHopNeigh = N2.begin (); twoHopNeigh != N2.end (); )
    {
      if (toRemove.find (twoHopNeigh->twoHopNeighborAddr) != toRemove.end ())
        {
          twoHopNeigh = N2.erase (twoHopNeigh);
        }
      else
        {
          twoHopNeigh++;
        }
    }
}
} // anonymous namespace

///
/// \brief Computates MPR set of a node following \RFC{3626} hints.
///
void
RoutingProtocol::MprComputation ()
{
  NS_LOG_FUNCTION (this);

  // MPR computation should be done for each interface. See section 8.3.1
  // (RFC 3626) for details.
  MprSet mprSet;

  // N is the subset of neighbors of the node, which are
  // neighbor "of the interface I"
  NeighborSet N;
  for (NeighborSet::const_iterator neighbor = m_state.GetNeighbors ().begin ();
       neighbor != m_state.GetNeighbors ().end (); neighbor++)
    {
      if (neighbor->status == NeighborTuple::STATUS_SYM) // I think that we need this check
        {
          N.push_back (*neighbor);
        }
    }

  // N2 is the set of 2-hop neighbors reachable from "the interface
  // I", excluding:
  // (i)   the nodes only reachable by members of N with willingness WILL_NEVER
  // (ii)  the node performing the computation
  // (iii) all the symmetric neighbors: the nodes for which there exists a symmetric
  //       link to this node on some interface.
  TwoHopNeighborSet N2;
  for (TwoHopNeighborSet::const_iterator twoHopNeigh = m_state.GetTwoHopNeighbors ().begin ();
       twoHopNeigh != m_state.GetTwoHopNeighbors ().end (); twoHopNeigh++)
    {
      // excluding:
      // (ii)  the node performing the computation
      if (twoHopNeigh->twoHopNeighborAddr == m_mainAddress)
        {
          continue;
        }

      //  excluding:
      // (i)   the nodes only reachable by members of N with willingness WILL_NEVER
      bool ok = false;
      for (NeighborSet::const_iterator neigh = N.begin ();
           neigh != N.end (); neigh++)
        {
          if (neigh->neighborMainAddr == twoHopNeigh->neighborMainAddr)
            {
              if (neigh->willingness == IBOLSR_WILL_NEVER)
                {
                  ok = false;
                  break;
                }
              else
                {
                  ok = true;
                  break;
                }
            }
        }
      if (!ok)
        {
          continue;
        }

      // excluding:
      // (iii) all the symmetric neighbors: the nodes for which there exists a symmetric
      //       link to this node on some interface.
      for (NeighborSet::const_iterator neigh = N.begin ();
           neigh != N.end (); neigh++)
        {
          if (neigh->neighborMainAddr == twoHopNeigh->twoHopNeighborAddr)
            {
              ok = false;
              break;
            }
        }

      if (ok)
        {
          N2.push_back (*twoHopNeigh);
        }
    }

#ifdef NS3_LOG_ENABLE
  {
    std::ostringstream os;
    os << "[";
    for (TwoHopNeighborSet::const_iterator iter = N2.begin ();
         iter != N2.end (); iter++)
      {
        TwoHopNeighborSet::const_iterator next = iter;
        next++;
        os << iter->neighborMainAddr << "->" << iter->twoHopNeighborAddr;
        if (next != N2.end ())
          os << ", ";
      }
    os << "]";
    NS_LOG_DEBUG ("N2: " << os.str ());
  }
#endif  //NS3_LOG_ENABLE

  // 1. Start with an MPR set made of all members of N with
  // N_willingness equal to WILL_ALWAYS
  for (NeighborSet::const_iterator neighbor = N.begin (); neighbor != N.end (); neighbor++)
    {
      //if (neighbor->willingness == IBOLSR_WILL_ALWAYS)
      if ((neighbor->willingness == IBOLSR_WILL_ALWAYS) && !(neighbor->risky))
        {
          mprSet.insert (neighbor->neighborMainAddr);
          // (not in RFC but I think is needed: remove the 2-hop
          // neighbors reachable by the MPR from N2)
          CoverTwoHopNeighbors (neighbor->neighborMainAddr, N2);
        }
    }

  // 2. Calculate D(y), where y is a member of N, for all nodes in N.
  // (we do this later)

  // 3. Add to the MPR set those nodes in N, which are the *only*
  // nodes to provide reachability to a node in N2.
  std::set<Ipv4Address> coveredTwoHopNeighbors;
  for (TwoHopNeighborSet::const_iterator twoHopNeigh = N2.begin (); twoHopNeigh != N2.end (); twoHopNeigh++)
    {
      bool onlyOne = true;
      // try to find another neighbor that can reach twoHopNeigh->twoHopNeighborAddr
      for (TwoHopNeighborSet::const_iterator otherTwoHopNeigh = N2.begin (); otherTwoHopNeigh != N2.end (); otherTwoHopNeigh++)
        {
          if (otherTwoHopNeigh->twoHopNeighborAddr == twoHopNeigh->twoHopNeighborAddr
              && otherTwoHopNeigh->neighborMainAddr != twoHopNeigh->neighborMainAddr)
            {
              onlyOne = false;
              break;
            }
        }
      if (onlyOne)
        {
          NS_LOG_LOGIC ("Neighbor " << twoHopNeigh->neighborMainAddr
                                    << " is the only that can reach 2-hop neigh. "
                                    << twoHopNeigh->twoHopNeighborAddr
                                    << " => select as MPR.");

		  NeighborTuple *nei =  m_state.FindNeighborTuple(twoHopNeigh->neighborMainAddr);
		  if (nei != NULL){
			  if (nei->risky) break;
		  }
          mprSet.insert (twoHopNeigh->neighborMainAddr);

          // take note of all the 2-hop neighbors reachable by the newly elected MPR
          for (TwoHopNeighborSet::const_iterator otherTwoHopNeigh = N2.begin ();
               otherTwoHopNeigh != N2.end (); otherTwoHopNeigh++)
            {
              if (otherTwoHopNeigh->neighborMainAddr == twoHopNeigh->neighborMainAddr)
                {
                  coveredTwoHopNeighbors.insert (otherTwoHopNeigh->twoHopNeighborAddr);
                }
            }
        }
    }
  // Remove the nodes from N2 which are now covered by a node in the MPR set.
  for (TwoHopNeighborSet::iterator twoHopNeigh = N2.begin ();
       twoHopNeigh != N2.end (); )
    {
      if (coveredTwoHopNeighbors.find (twoHopNeigh->twoHopNeighborAddr) != coveredTwoHopNeighbors.end ())
        {
          // This works correctly only because it is known that twoHopNeigh is reachable by exactly one neighbor, 
          // so only one record in N2 exists for each of them. This record is erased here.
          NS_LOG_LOGIC ("2-hop neigh. " << twoHopNeigh->twoHopNeighborAddr << " is already covered by an MPR.");
          twoHopNeigh = N2.erase (twoHopNeigh);
        }
      else
        {
          twoHopNeigh++;
        }
    }

  // Temporarily move risky nodes away
  NeighborSet NRisky;
  for (NeighborSet::iterator it = N.begin(); it != N.end(); ++it){
	  //if ((m_mainAddress == Ipv4Address("10.0.0.1")) &&(it->willingness > 3)){
		//  std::cout << *it << "\n";
	  //}
	  if (it->risky){
		  NRisky.push_back(*it);
		  it = N.erase(it);
		  --it;
	  }
  }

  // 4. While there exist nodes in N2 which are not covered by at
  // least one node in the MPR set:
  while (N2.begin () != N2.end ())
    {

#ifdef NS3_LOG_ENABLE
      {
        std::ostringstream os;
        os << "[";
        for (TwoHopNeighborSet::const_iterator iter = N2.begin ();
             iter != N2.end (); iter++)
          {
            TwoHopNeighborSet::const_iterator next = iter;
            next++;
            os << iter->neighborMainAddr << "->" << iter->twoHopNeighborAddr;
            if (next != N2.end ())
              os << ", ";
          }
        os << "]";
        NS_LOG_DEBUG ("Step 4 iteration: N2=" << os.str ());
      }
#endif  //NS3_LOG_ENABLE


      // 4.1. For each node in N, calculate the reachability, i.e., the
      // number of nodes in N2 which are not yet covered by at
      // least one node in the MPR set, and which are reachable
      // through this 1-hop neighbor
      std::map<int, std::vector<const NeighborTuple *> > reachability;
      std::set<int> rs;
      for (NeighborSet::iterator it = N.begin (); it != N.end (); it++)
        {
          NeighborTuple const &nb_tuple = *it;
          int r = 0;
		  //if ((activeDefence) && (nb_tuple.risky)) continue; // Don't check risky ones
          for (TwoHopNeighborSet::iterator it2 = N2.begin (); it2 != N2.end (); it2++)
            {
              TwoHopNeighborTuple const &nb2hop_tuple = *it2;
              if (nb_tuple.neighborMainAddr == nb2hop_tuple.neighborMainAddr)
                r++;
            }
          rs.insert (r);
          reachability[r].push_back (&nb_tuple);
        }

      // 4.2. Select as a MPR the node with highest N_willingness among
      // the nodes in N with non-zero reachability. In case of
      // multiple choice select the node which provides
      // reachability to the maximum number of nodes in N2. In
      // case of multiple nodes providing the same amount of
      // reachability, select the node as MPR whose D(y) is
      // greater. Remove the nodes from N2 which are now covered
      // by a node in the MPR set.
      NeighborTuple const *max = NULL;
      int max_r = 0;
      for (std::set<int>::iterator it = rs.begin (); it != rs.end (); it++)
        {
          int r = *it;
          if (r == 0)
            {
              continue;
            }
          for (std::vector<const NeighborTuple *>::iterator it2 = reachability[r].begin ();
               it2 != reachability[r].end (); it2++)
            {
              const NeighborTuple *nb_tuple = *it2;
              if (max == NULL || nb_tuple->willingness > max->willingness)
                {
                  max = nb_tuple;
                  max_r = r;
                }
              else if (nb_tuple->willingness == max->willingness)
                {
                  if (r > max_r)
                    {
                      max = nb_tuple;
                      max_r = r;
                    }
                  else if (r == max_r)
                    {
                      if (Degree (*nb_tuple) > Degree (*max))
                        {
                          max = nb_tuple;
                          max_r = r;
                        }
                    }
                }
            }
        }

	  // Restore risky nodes if needed
	  if (max_r == 0){
		  NeighborSet::iterator it = NRisky.begin();
		  while (it != NRisky.end()){
			  N.push_back(*it);
			  it = NRisky.erase(it);
		  }
	  }

      if (max != NULL)
        {
          mprSet.insert (max->neighborMainAddr);
          CoverTwoHopNeighbors (max->neighborMainAddr, N2);
          NS_LOG_LOGIC (N2.size () << " 2-hop neighbors left to cover!");
        }
    }

  /*
  if (activeDefence){ 
	  // Check we aren't using only unverified nodes
	  bool unsafeMprSelection = true;
	  for (MprSet::iterator it = mprSet.begin(); it != mprSet.end(); ++it){
		if (std::find(unverifiedNodes.begin(), unverifiedNodes.end(), *it) 
		  == unverifiedNodes.end()){
			unsafeMprSelection = false;
		}
	  }
	  if (unsafeMprSelection){
		for (NeighborSet::iterator it = m_state.GetNeighbors().begin();
				it != m_state.GetNeighbors().end(); ++it){
		  if (std::find(unverifiedNodes.begin(), unverifiedNodes.end(), it->neighborMainAddr) 
			== unverifiedNodes.end()){
			  mprSet.insert (it->neighborMainAddr);
			  break;
		  }
		}
	  }
  }
  */
  

#ifdef NS3_LOG_ENABLE
  {
    std::ostringstream os;
    os << "[";
    for (MprSet::const_iterator iter = mprSet.begin ();
         iter != mprSet.end (); iter++)
      {
        MprSet::const_iterator next = iter;
        next++;
        os << *iter;
        if (next != mprSet.end ())
          os << ", ";
      }
    os << "]";
    NS_LOG_DEBUG ("Computed MPR set for node " << m_mainAddress << ": " << os.str ());
  }
#endif  //NS3_LOG_ENABLE

  // Listen Test. Super Naiive 2006
  bool addmore = true;
  for (MprSet::iterator it = mprSet.begin(); it != mprSet.end(); ++it){
	  if (!MightBeEvil(*it)){
		  addmore = false;
		  break;
	  }
  }
  if (addmore){
	  const NeighborSet &nei = m_state.GetNeighbors();
	  for (NeighborSet::const_iterator it = nei.begin(); it != nei.end(); ++it){
		  if ((mprSet.find(it->neighborMainAddr) == mprSet.end()) && !MightBeEvil(it->neighborMainAddr)){
			  if (it->status == NeighborTuple::STATUS_NOT_SYM) continue;
			  mprSet.insert(it->neighborMainAddr);
			  break;
		  }
	  }
  }

  // Rig MPR set for colluding attacker
  if (blackholeAttackColl1){
	  mprSet.clear();
	  mprSet.insert(blackholeAttackCollFriend);
  }
  m_state.SetMprSet (mprSet);
}

///
/// \brief Gets the main address associated with a given interface address.
///
/// \param iface_addr the interface address.
/// \return the corresponding main address.
///
Ipv4Address
RoutingProtocol::GetMainAddress (Ipv4Address iface_addr) const
{
  const IfaceAssocTuple *tuple =
    m_state.FindIfaceAssocTuple (iface_addr);

  if (tuple != NULL)
    return tuple->mainAddr;
  else
    return iface_addr;
}

///
/// \brief Creates the routing table of the node following \RFC{3626} hints.
///
void
RoutingProtocol::RoutingTableComputation ()
{
  NS_LOG_DEBUG (Simulator::Now ().GetSeconds () << " s: Node " << m_mainAddress
                                                << ": RoutingTableComputation begin...");

  // 1. All the entries from the routing table are removed.
  Clear ();

  // 2. The new routing entries are added starting with the
  // symmetric neighbors (h=1) as the destination nodes.
  const NeighborSet &neighborSet = m_state.GetNeighbors ();
  for (NeighborSet::const_iterator it = neighborSet.begin ();
       it != neighborSet.end (); it++)
    {
      NeighborTuple const &nb_tuple = *it;
      NS_LOG_DEBUG ("Looking at neighbor tuple: " << nb_tuple);
      if (nb_tuple.status == NeighborTuple::STATUS_SYM)
        {
          bool nb_main_addr = false;
          const LinkTuple *lt = NULL;
          const LinkSet &linkSet = m_state.GetLinks ();
          for (LinkSet::const_iterator it2 = linkSet.begin ();
               it2 != linkSet.end (); it2++)
            {
              LinkTuple const &link_tuple = *it2;
              NS_LOG_DEBUG ("Looking at link tuple: " << link_tuple
                                                      << (link_tuple.time >= Simulator::Now () ? "" : " (expired)"));
              if ((GetMainAddress (link_tuple.neighborIfaceAddr) == nb_tuple.neighborMainAddr)
                  && link_tuple.time >= Simulator::Now ())
                {
                  NS_LOG_LOGIC ("Link tuple matches neighbor " << nb_tuple.neighborMainAddr
                                                               << " => adding routing table entry to neighbor");
                  lt = &link_tuple;
                  AddEntry (link_tuple.neighborIfaceAddr,
                            link_tuple.neighborIfaceAddr,
                            link_tuple.localIfaceAddr,
                            1);
                  if (link_tuple.neighborIfaceAddr == nb_tuple.neighborMainAddr)
                    {
                      nb_main_addr = true;
                    }
                }
              else
                {
                  NS_LOG_LOGIC ("Link tuple: linkMainAddress= " << GetMainAddress (link_tuple.neighborIfaceAddr)
                                                                << "; neighborMainAddr =  " << nb_tuple.neighborMainAddr
                                                                << "; expired=" << int (link_tuple.time < Simulator::Now ())
                                                                << " => IGNORE");
                }
            }

          // If, in the above, no R_dest_addr is equal to the main
          // address of the neighbor, then another new routing entry
          // with MUST be added, with:
          //      R_dest_addr  = main address of the neighbor;
          //      R_next_addr  = L_neighbor_iface_addr of one of the
          //                     associated link tuple with L_time >= current time;
          //      R_dist       = 1;
          //      R_iface_addr = L_local_iface_addr of the
          //                     associated link tuple.
          if (!nb_main_addr && lt != NULL)
            {
              NS_LOG_LOGIC ("no R_dest_addr is equal to the main address of the neighbor "
                            "=> adding additional routing entry");
              AddEntry (nb_tuple.neighborMainAddr,
                        lt->neighborIfaceAddr,
                        lt->localIfaceAddr,
                        1);
            }
        }
    }

  //  3. for each node in N2, i.e., a 2-hop neighbor which is not a
  //  neighbor node or the node itself, and such that there exist at
  //  least one entry in the 2-hop neighbor set where
  //  N_neighbor_main_addr correspond to a neighbor node with
  //  willingness different of WILL_NEVER,
  const TwoHopNeighborSet &twoHopNeighbors = m_state.GetTwoHopNeighbors ();
  for (TwoHopNeighborSet::const_iterator it = twoHopNeighbors.begin ();
       it != twoHopNeighbors.end (); it++)
    {
      TwoHopNeighborTuple const &nb2hop_tuple = *it;

	  // BLACKHOLE DEFENCE
	  if (activeDefence && activeImpDefence){
		  const NeighborTuple* nb_tuple = m_state.FindSymNeighborTuple (nb2hop_tuple.neighborMainAddr);
         if(nb_tuple)
         {
            if(nb_tuple->risky){

                RoutingTableEntry entry;
                bool foundEntry = Lookup (nb2hop_tuple.twoHopNeighborAddr, entry);
                if (foundEntry)
                {
                    continue;
                }
                bool onlyWay=true;
                //if there is another way - continue
				{
					TwoHopNeighborSet::const_iterator it2 = it;
					++it2;
					for (;it2 != twoHopNeighbors.end (); ++it2){
					  if(it2->twoHopNeighborAddr==nb2hop_tuple.twoHopNeighborAddr
							&& it2->neighborMainAddr!=nb2hop_tuple.neighborMainAddr){
						onlyWay=false;
						break;
					  }
					}
				}
                if(!onlyWay)
                {
                    continue;
                }
            }
         }

	  }
	  // END BLACKHOLE DEFENCE

      NS_LOG_LOGIC ("Looking at two-hop neighbor tuple: " << nb2hop_tuple);

      // a 2-hop neighbor which is not a neighbor node or the node itself
      if (m_state.FindSymNeighborTuple (nb2hop_tuple.twoHopNeighborAddr))
        {
          NS_LOG_LOGIC ("Two-hop neighbor tuple is also neighbor; skipped.");
          continue;
        }

      if (nb2hop_tuple.twoHopNeighborAddr == m_mainAddress)
        {
          NS_LOG_LOGIC ("Two-hop neighbor is self; skipped.");
          continue;
        }

      // ...and such that there exist at least one entry in the 2-hop
      // neighbor set where N_neighbor_main_addr correspond to a
      // neighbor node with willingness different of WILL_NEVER...
      bool nb2hopOk = false;
      for (NeighborSet::const_iterator neighbor = neighborSet.begin ();
           neighbor != neighborSet.end (); neighbor++)
        {
          if (neighbor->neighborMainAddr == nb2hop_tuple.neighborMainAddr
              && neighbor->willingness != IBOLSR_WILL_NEVER)
            {
              nb2hopOk = true;
              break;
            }
        }
      if (!nb2hopOk)
        {
          NS_LOG_LOGIC ("Two-hop neighbor tuple skipped: 2-hop neighbor "
                        << nb2hop_tuple.twoHopNeighborAddr
                        << " is attached to neighbor " << nb2hop_tuple.neighborMainAddr
                        << ", which was not found in the Neighbor Set.");
          continue;
        }

      // one selects one 2-hop tuple and creates one entry in the routing table with:
      //                R_dest_addr  =  the main address of the 2-hop neighbor;
      //                R_next_addr  = the R_next_addr of the entry in the
      //                               routing table with:
      //                                   R_dest_addr == N_neighbor_main_addr
      //                                                  of the 2-hop tuple;
      //                R_dist       = 2;
      //                R_iface_addr = the R_iface_addr of the entry in the
      //                               routing table with:
      //                                   R_dest_addr == N_neighbor_main_addr
      //                                                  of the 2-hop tuple;
      RoutingTableEntry entry;
      bool foundEntry = Lookup (nb2hop_tuple.neighborMainAddr, entry);
      if (foundEntry)
        {
          NS_LOG_LOGIC ("Adding routing entry for two-hop neighbor.");
          AddEntry (nb2hop_tuple.twoHopNeighborAddr,
                    entry.nextAddr,
                    entry.interface,
                    2);
        }
      else
        {
          NS_LOG_LOGIC ("NOT adding routing entry for two-hop neighbor ("
                        << nb2hop_tuple.twoHopNeighborAddr
                        << " not found in the routing table)");
        }
    }

  for (uint32_t h = 2;; h++)
    {
      bool added = false;

      // 3.1. For each topology entry in the topology table, if its
      // T_dest_addr does not correspond to R_dest_addr of any
      // route entry in the routing table AND its T_last_addr
      // corresponds to R_dest_addr of a route entry whose R_dist
      // is equal to h, then a new route entry MUST be recorded in
      // the routing table (if it does not already exist)
      const TopologySet &topology = m_state.GetTopologySet ();
      for (TopologySet::const_iterator it = topology.begin ();
           it != topology.end (); it++)
        {
          const TopologyTuple &topology_tuple = *it;
          NS_LOG_LOGIC ("Looking at topology tuple: " << topology_tuple);

          RoutingTableEntry destAddrEntry, lastAddrEntry;
          bool have_destAddrEntry = Lookup (topology_tuple.destAddr, destAddrEntry);
          bool have_lastAddrEntry = Lookup (topology_tuple.lastAddr, lastAddrEntry);
          if (!have_destAddrEntry && have_lastAddrEntry && lastAddrEntry.distance == h)
            {
              NS_LOG_LOGIC ("Adding routing table entry based on the topology tuple.");
              // then a new route entry MUST be recorded in
              //                the routing table (if it does not already exist) where:
              //                     R_dest_addr  = T_dest_addr;
              //                     R_next_addr  = R_next_addr of the recorded
              //                                    route entry where:
              //                                    R_dest_addr == T_last_addr
              //                     R_dist       = h+1; and
              //                     R_iface_addr = R_iface_addr of the recorded
              //                                    route entry where:
              //                                       R_dest_addr == T_last_addr.
              AddEntry (topology_tuple.destAddr,
                        lastAddrEntry.nextAddr,
                        lastAddrEntry.interface,
                        h + 1);
              added = true;
            }
          else
            {
              NS_LOG_LOGIC ("NOT adding routing table entry based on the topology tuple: "
                            "have_destAddrEntry=" << have_destAddrEntry
                                                  << " have_lastAddrEntry=" << have_lastAddrEntry
                                                  << " lastAddrEntry.distance=" << (int) lastAddrEntry.distance
                                                  << " (h=" << h << ")");
            }
        }

      if (!added)
        break;
    }

  // 4. For each entry in the multiple interface association base
  // where there exists a routing entry such that:
  // R_dest_addr == I_main_addr (of the multiple interface association entry)
  // AND there is no routing entry such that:
  // R_dest_addr == I_iface_addr
  const IfaceAssocSet &ifaceAssocSet = m_state.GetIfaceAssocSet ();
  for (IfaceAssocSet::const_iterator it = ifaceAssocSet.begin ();
       it != ifaceAssocSet.end (); it++)
    {
      IfaceAssocTuple const &tuple = *it;
      RoutingTableEntry entry1, entry2;
      bool have_entry1 = Lookup (tuple.mainAddr, entry1);
      bool have_entry2 = Lookup (tuple.ifaceAddr, entry2);
      if (have_entry1 && !have_entry2)
        {
          // then a route entry is created in the routing table with:
          //       R_dest_addr  =  I_iface_addr (of the multiple interface
          //                                     association entry)
          //       R_next_addr  =  R_next_addr  (of the recorded route entry)
          //       R_dist       =  R_dist       (of the recorded route entry)
          //       R_iface_addr =  R_iface_addr (of the recorded route entry).
          AddEntry (tuple.ifaceAddr,
                    entry1.nextAddr,
                    entry1.interface,
                    entry1.distance);
        }
    }

  // 5. For each tuple in the association set,
  //    If there is no entry in the routing table with:
  //        R_dest_addr     == A_network_addr/A_netmask
  //   and if the announced network is not announced by the node itself,
  //   then a new routing entry is created.
  const AssociationSet &associationSet = m_state.GetAssociationSet ();

  // Clear HNA routing table
  for (uint32_t i = 0; i < m_hnaRoutingTable->GetNRoutes (); i++)
    {
      m_hnaRoutingTable->RemoveRoute (0);
    }

  for (AssociationSet::const_iterator it = associationSet.begin ();
       it != associationSet.end (); it++)
    {
      AssociationTuple const &tuple = *it;

      // Test if HNA associations received from other gateways
      // are also announced by this node. In such a case, no route
      // is created for this association tuple (go to the next one).
      bool goToNextAssociationTuple = false;
      const Associations &localHnaAssociations = m_state.GetAssociations ();
      NS_LOG_DEBUG ("Nb local associations: " << localHnaAssociations.size ());
      for (Associations::const_iterator assocIterator = localHnaAssociations.begin ();
           assocIterator != localHnaAssociations.end (); assocIterator++)
        {
          Association const &localHnaAssoc = *assocIterator;
          if (localHnaAssoc.networkAddr == tuple.networkAddr && localHnaAssoc.netmask == tuple.netmask)
            {
              NS_LOG_DEBUG ("HNA association received from another GW is part of local HNA associations: no route added for network "
                            << tuple.networkAddr << "/" << tuple.netmask);
              goToNextAssociationTuple = true;
            }
        }
      if (goToNextAssociationTuple)
        {
          continue;
        }

      RoutingTableEntry gatewayEntry;

      bool gatewayEntryExists = Lookup (tuple.gatewayAddr, gatewayEntry);
      bool addRoute = false;

      uint32_t routeIndex = 0;

      for (routeIndex = 0; routeIndex < m_hnaRoutingTable->GetNRoutes (); routeIndex++)
        {
          Ipv4RoutingTableEntry route = m_hnaRoutingTable->GetRoute (routeIndex);
          if (route.GetDestNetwork () == tuple.networkAddr &&
              route.GetDestNetworkMask () == tuple.netmask)
            {
              break;
            }
        }

      if (routeIndex == m_hnaRoutingTable->GetNRoutes ())
        {
          addRoute = true;
        }
      else if(gatewayEntryExists && m_hnaRoutingTable->GetMetric (routeIndex) > gatewayEntry.distance)
        {
          m_hnaRoutingTable->RemoveRoute (routeIndex);
          addRoute = true;
        }

      if(addRoute && gatewayEntryExists)
        {
          m_hnaRoutingTable->AddNetworkRouteTo (tuple.networkAddr,
                                                tuple.netmask,
                                                gatewayEntry.nextAddr,
                                                gatewayEntry.interface,
                                                gatewayEntry.distance);

        }
    }

  NS_LOG_DEBUG ("Node " << m_mainAddress << ": RoutingTableComputation end.");
  m_routingTableChanged (GetSize ());
}


///
/// \brief Processes a HELLO message following \RFC{3626} specification.
///
/// Link sensing and population of the Neighbor Set, 2-hop Neighbor Set and MPR
/// Selector Set are performed.
///
/// \param msg the %IBOLSR message which contains the HELLO message.
/// \param receiver_iface the address of the interface where the message was received from.
/// \param sender_iface the address of the interface where the message was sent from.
///
void
RoutingProtocol::ProcessHello (const ibolsr::MessageHeader &msg,
                               const Ipv4Address &receiverIface,
                               const Ipv4Address &senderIface)
{
  NS_LOG_FUNCTION (msg << receiverIface << senderIface);

  const ibolsr::MessageHeader::Hello &hello = msg.GetHello ();

  LinkSensing (msg, hello, receiverIface, senderIface);

#ifdef NS3_LOG_ENABLE
  {
    const LinkSet &links = m_state.GetLinks ();
    NS_LOG_DEBUG (Simulator::Now ().GetSeconds ()
                  << "s ** BEGIN dump Link Set for IBOLSR Node " << m_mainAddress);
    for (LinkSet::const_iterator link = links.begin (); link != links.end (); link++)
      {
        NS_LOG_DEBUG (*link);
      }
    NS_LOG_DEBUG ("** END dump Link Set for IBOLSR Node " << m_mainAddress);

    const NeighborSet &neighbors = m_state.GetNeighbors ();
    NS_LOG_DEBUG (Simulator::Now ().GetSeconds ()
                  << "s ** BEGIN dump Neighbor Set for IBOLSR Node " << m_mainAddress);
    for (NeighborSet::const_iterator neighbor = neighbors.begin (); neighbor != neighbors.end (); neighbor++)
      {
        NS_LOG_DEBUG (*neighbor);
      }
    NS_LOG_DEBUG ("** END dump Neighbor Set for IBOLSR Node " << m_mainAddress);
  }
#endif // NS3_LOG_ENABLE

  PopulateNeighborSet (msg, hello);
  PopulateTwoHopNeighborSet (msg, hello);

#ifdef NS3_LOG_ENABLE
  {
    const TwoHopNeighborSet &twoHopNeighbors = m_state.GetTwoHopNeighbors ();
    NS_LOG_DEBUG (Simulator::Now ().GetSeconds ()
                  << "s ** BEGIN dump TwoHopNeighbor Set for IBOLSR Node " << m_mainAddress);
    for (TwoHopNeighborSet::const_iterator tuple = twoHopNeighbors.begin ();
         tuple != twoHopNeighbors.end (); tuple++)
      {
        NS_LOG_DEBUG (*tuple);
      }
    NS_LOG_DEBUG ("** END dump TwoHopNeighbor Set for IBOLSR Node " << m_mainAddress);
  }
#endif // NS3_LOG_ENABLE

  MprComputation ();
  PopulateMprSelectorSet (msg, hello);
}

///
/// \brief Processes a TC message following \RFC{3626} specification.
///
/// The Topology Set is updated (if needed) with the information of
/// the received TC message.
///
/// \param msg the %IBOLSR message which contains the TC message.
/// \param sender_iface the address of the interface where the message was sent from.
///
void
RoutingProtocol::ProcessTc (const ibolsr::MessageHeader &msg,
                            const Ipv4Address &senderIface)
{
  const ibolsr::MessageHeader::Tc &tc = msg.GetTc ();
  Time now = Simulator::Now ();

  // 1. If the sender interface of this message is not in the symmetric
  // 1-hop neighborhood of this node, the message MUST be discarded.
  const LinkTuple *link_tuple = m_state.FindSymLinkTuple (senderIface, now);
  if (link_tuple == NULL)
    return;

  // 2. If there exist some tuple in the topology set where:
  //    T_last_addr == originator address AND
  //    T_seq       >  ANSN,
  // then further processing of this TC message MUST NOT be
  // performed.
  const TopologyTuple *topologyTuple =
    m_state.FindNewerTopologyTuple (msg.GetOriginatorAddress (), tc.ansn);
  if (topologyTuple != NULL)
    return;

  // 3. All tuples in the topology set where:
  //    T_last_addr == originator address AND
  //    T_seq       <  ANSN
  // MUST be removed from the topology set.
  m_state.EraseOlderTopologyTuples (msg.GetOriginatorAddress (), tc.ansn);

  // 4. For each of the advertised neighbor main address received in
  // the TC message:
  for (std::vector<Ipv4Address>::const_iterator i = tc.neighborAddresses.begin ();
       i != tc.neighborAddresses.end (); i++)
    {
      const Ipv4Address &addr = *i;
      // 4.1. If there exist some tuple in the topology set where:
      //      T_dest_addr == advertised neighbor main address, AND
      //      T_last_addr == originator address,
      // then the holding time of that tuple MUST be set to:
      //      T_time      =  current time + validity time.
      TopologyTuple *topologyTuple =
        m_state.FindTopologyTuple (addr, msg.GetOriginatorAddress ());

      if (topologyTuple != NULL)
        {
          topologyTuple->expirationTime = now + msg.GetVTime ();
        }
      else
        {
          // 4.2. Otherwise, a new tuple MUST be recorded in the topology
          // set where:
          //      T_dest_addr = advertised neighbor main address,
          //      T_last_addr = originator address,
          //      T_seq       = ANSN,
          //      T_time      = current time + validity time.
          TopologyTuple topologyTuple;;
          topologyTuple.destAddr = addr;
          topologyTuple.lastAddr = msg.GetOriginatorAddress ();
          topologyTuple.sequenceNumber = tc.ansn;
          topologyTuple.expirationTime = now + msg.GetVTime ();
          AddTopologyTuple (topologyTuple);

          // Schedules topology tuple deletion
          m_events.Track (Simulator::Schedule (DELAY (topologyTuple.expirationTime),
                                               &RoutingProtocol::TopologyTupleTimerExpire,
                                               this,
                                               topologyTuple.destAddr,
                                               topologyTuple.lastAddr));
        }
    }

#ifdef NS3_LOG_ENABLE
  {
    const TopologySet &topology = m_state.GetTopologySet ();
    NS_LOG_DEBUG (Simulator::Now ().GetSeconds ()
                  << "s ** BEGIN dump TopologySet for IBOLSR Node " << m_mainAddress);
    for (TopologySet::const_iterator tuple = topology.begin ();
         tuple != topology.end (); tuple++)
      {
        NS_LOG_DEBUG (*tuple);
      }
    NS_LOG_DEBUG ("** END dump TopologySet Set for IBOLSR Node " << m_mainAddress);
  }
#endif // NS3_LOG_ENABLE
}

///
/// \brief Processes a MID message following \RFC{3626} specification.
///
/// The Interface Association Set is updated (if needed) with the information
/// of the received MID message.
///
/// \param msg the %IBOLSR message which contains the MID message.
/// \param sender_iface the address of the interface where the message was sent from.
///
void
RoutingProtocol::ProcessMid (const ibolsr::MessageHeader &msg,
                             const Ipv4Address &senderIface)
{
  const ibolsr::MessageHeader::Mid &mid = msg.GetMid ();
  Time now = Simulator::Now ();

  NS_LOG_DEBUG ("Node " << m_mainAddress << " ProcessMid from " << senderIface);
  // 1. If the sender interface of this message is not in the symmetric
  // 1-hop neighborhood of this node, the message MUST be discarded.
  const LinkTuple *linkTuple = m_state.FindSymLinkTuple (senderIface, now);
  if (linkTuple == NULL)
    {
      NS_LOG_LOGIC ("Node " << m_mainAddress <<
                    ": the sender interface of this message is not in the "
                    "symmetric 1-hop neighborhood of this node,"
                    " the message MUST be discarded.");
      return;
    }

  // 2. For each interface address listed in the MID message
  for (std::vector<Ipv4Address>::const_iterator i = mid.interfaceAddresses.begin ();
       i != mid.interfaceAddresses.end (); i++)
    {
      bool updated = false;
      IfaceAssocSet &ifaceAssoc = m_state.GetIfaceAssocSetMutable ();
      for (IfaceAssocSet::iterator tuple = ifaceAssoc.begin ();
           tuple != ifaceAssoc.end (); tuple++)
        {
          if (tuple->ifaceAddr == *i
              && tuple->mainAddr == msg.GetOriginatorAddress ())
            {
              NS_LOG_LOGIC ("IfaceAssoc updated: " << *tuple);
              tuple->time = now + msg.GetVTime ();
              updated = true;
            }
        }
      if (!updated)
        {
          IfaceAssocTuple tuple;
          tuple.ifaceAddr = *i;
          tuple.mainAddr = msg.GetOriginatorAddress ();
          tuple.time = now + msg.GetVTime ();
          AddIfaceAssocTuple (tuple);
          NS_LOG_LOGIC ("New IfaceAssoc added: " << tuple);
          // Schedules iface association tuple deletion
          Simulator::Schedule (DELAY (tuple.time),
                               &RoutingProtocol::IfaceAssocTupleTimerExpire, this, tuple.ifaceAddr);
        }
    }

  // 3. (not part of the RFC) iterate over all NeighborTuple's and
  // TwoHopNeighborTuples, update the neighbor addresses taking into account
  // the new MID information.
  NeighborSet &neighbors = m_state.GetNeighbors ();
  for (NeighborSet::iterator neighbor = neighbors.begin (); neighbor != neighbors.end (); neighbor++)
    {
      neighbor->neighborMainAddr = GetMainAddress (neighbor->neighborMainAddr);
    }

  TwoHopNeighborSet &twoHopNeighbors = m_state.GetTwoHopNeighbors ();
  for (TwoHopNeighborSet::iterator twoHopNeighbor = twoHopNeighbors.begin ();
       twoHopNeighbor != twoHopNeighbors.end (); twoHopNeighbor++)
    {
      twoHopNeighbor->neighborMainAddr = GetMainAddress (twoHopNeighbor->neighborMainAddr);
      twoHopNeighbor->twoHopNeighborAddr = GetMainAddress (twoHopNeighbor->twoHopNeighborAddr);
    }
  NS_LOG_DEBUG ("Node " << m_mainAddress << " ProcessMid from " << senderIface << " -> END.");
}

///
/// \brief Processes a HNA message following \RFC{3626} specification.
///
/// The Host Network Association Set is updated (if needed) with the information
/// of the received HNA message.
///
/// \param msg the %IBOLSR message which contains the HNA message.
/// \param sender_iface the address of the interface where the message was sent from.
///
void
RoutingProtocol::ProcessHna (const ibolsr::MessageHeader &msg,
                             const Ipv4Address &senderIface)
{

  const ibolsr::MessageHeader::Hna &hna = msg.GetHna ();
  Time now = Simulator::Now ();

  // 1. If the sender interface of this message is not in the symmetric
  // 1-hop neighborhood of this node, the message MUST be discarded.
  const LinkTuple *link_tuple = m_state.FindSymLinkTuple (senderIface, now);
  if (link_tuple == NULL)
    return;

  // 2. Otherwise, for each (network address, netmask) pair in the
  // message:

  for (std::vector<ibolsr::MessageHeader::Hna::Association>::const_iterator it = hna.associations.begin ();
       it != hna.associations.end (); it++)
    {
      AssociationTuple *tuple = m_state.FindAssociationTuple (msg.GetOriginatorAddress (),it->address,it->mask);

      // 2.1  if an entry in the association set already exists, where:
      //          A_gateway_addr == originator address
      //          A_network_addr == network address
      //          A_netmask      == netmask
      //      then the holding time for that tuple MUST be set to:
      //          A_time         =  current time + validity time
      if(tuple != NULL)
        {
          tuple->expirationTime = now + msg.GetVTime ();
        }

      // 2.2 otherwise, a new tuple MUST be recorded with:
      //          A_gateway_addr =  originator address
      //          A_network_addr =  network address
      //          A_netmask      =  netmask
      //          A_time         =  current time + validity time
      else
        {
          AssociationTuple assocTuple = {
            msg.GetOriginatorAddress (),
            it->address,
            it->mask,
            now + msg.GetVTime ()
          };
          AddAssociationTuple (assocTuple);

          //Schedule Association Tuple deletion
          Simulator::Schedule (DELAY (assocTuple.expirationTime),
                               &RoutingProtocol::AssociationTupleTimerExpire, this,
                               assocTuple.gatewayAddr,assocTuple.networkAddr,assocTuple.netmask);
        }

    }
}

///
/// \brief IBOLSR's default forwarding algorithm.
///
/// See \RFC{3626} for details.
///
/// \param p the %IBOLSR packet which has been received.
/// \param msg the %IBOLSR message which must be forwarded.
/// \param dup_tuple NULL if the message has never been considered for forwarding,
/// or a duplicate tuple in other case.
/// \param local_iface the address of the interface where the message was received from.
///
void
RoutingProtocol::ForwardDefault (ibolsr::MessageHeader ibolsrMessage,
                                 DuplicateTuple *duplicated,
                                 const Ipv4Address &localIface,
                                 const Ipv4Address &senderAddress)
{
  Time now = Simulator::Now ();

  // If the sender interface address is not in the symmetric
  // 1-hop neighborhood the message must not be forwarded
  const LinkTuple *linkTuple = m_state.FindSymLinkTuple (senderAddress, now);
  if (linkTuple == NULL)
    return;

  // If the message has already been considered for forwarding,
  // it must not be retransmitted again
  if (duplicated != NULL && duplicated->retransmitted)
    {
      NS_LOG_LOGIC (Simulator::Now () << "Node " << m_mainAddress << " does not forward a message received"
                    " from " << ibolsrMessage.GetOriginatorAddress () << " because it is duplicated");
      return;
    }

  // If the sender interface address is an interface address
  // of a MPR selector of this node and ttl is greater than 1,
  // the message must be retransmitted
  bool retransmitted = false;
  if (ibolsrMessage.GetTimeToLive () > 1)
    {
      const MprSelectorTuple *mprselTuple =
        m_state.FindMprSelectorTuple (GetMainAddress (senderAddress));
      if (mprselTuple != NULL)
        {
          ibolsrMessage.SetTimeToLive (ibolsrMessage.GetTimeToLive () - 1);
          ibolsrMessage.SetHopCount (ibolsrMessage.GetHopCount () + 1);
          // We have to introduce a random delay to avoid
          // synchronization with neighbors.
          QueueMessage (ibolsrMessage, JITTER);
          retransmitted = true;
        }
    }

  // Update duplicate tuple...
  if (duplicated != NULL)
    {
      duplicated->expirationTime = now + IBOLSR_DUP_HOLD_TIME;
      duplicated->retransmitted = retransmitted;
      duplicated->ifaceList.push_back (localIface);
    }
  // ...or create a new one
  else
    {
      DuplicateTuple newDup;
      newDup.address = ibolsrMessage.GetOriginatorAddress ();
      newDup.sequenceNumber = ibolsrMessage.GetMessageSequenceNumber ();
      newDup.expirationTime = now + IBOLSR_DUP_HOLD_TIME;
      newDup.retransmitted = retransmitted;
      newDup.ifaceList.push_back (localIface);
      AddDuplicateTuple (newDup);
      // Schedule dup tuple deletion
      Simulator::Schedule (IBOLSR_DUP_HOLD_TIME,
                           &RoutingProtocol::DupTupleTimerExpire, this,
                           newDup.address, newDup.sequenceNumber);
    }
}

///
/// \brief Enques an %IBOLSR message which will be sent with a delay of (0, delay].
///
/// This buffering system is used in order to piggyback several %IBOLSR messages in
/// a same %IBOLSR packet.
///
/// \param msg the %IBOLSR message which must be sent.
/// \param delay maximum delay the %IBOLSR message is going to be buffered.
///
void
RoutingProtocol::QueueMessage (const ibolsr::MessageHeader &message, Time delay)
{
  m_queuedMessages.push_back (message);
  if (not m_queuedMessagesTimer.IsRunning ())
    {
      m_queuedMessagesTimer.SetDelay (delay);
      m_queuedMessagesTimer.Schedule ();
    }
}

void
RoutingProtocol::SendPacket (Ptr<Packet> packet, 
                             const MessageList &containedMessages)
{
  NS_LOG_DEBUG ("IBOLSR node " << m_mainAddress << " sending a IBOLSR packet");

  // Add a header
  ibolsr::PacketHeader header;
  header.SetPacketLength (header.GetSerializedSize () + packet->GetSize ());
  header.SetPacketSequenceNumber (GetPacketSequenceNumber ());
  packet->AddHeader (header);

  // Trace it
  m_txPacketTrace (header, containedMessages);

  // Send it
  for (std::map<Ptr<Socket>, Ipv4InterfaceAddress>::const_iterator i =
         m_socketAddresses.begin (); i != m_socketAddresses.end (); i++)
    {
      Ipv4Address bcast = i->second.GetLocal ().GetSubnetDirectedBroadcast (i->second.GetMask ());
      i->first->SendTo (packet, 0, InetSocketAddress (bcast, IBOLSR_PORT_NUMBER));
    }
}

///
/// \brief Creates as many %IBOLSR packets as needed in order to send all buffered
/// %IBOLSR messages.
///
/// Maximum number of messages which can be contained in an %IBOLSR packet is
/// dictated by IBOLSR_MAX_MSGS constant.
///
void
RoutingProtocol::SendQueuedMessages ()
{
  Ptr<Packet> packet = Create<Packet> ();
  int numMessages = 0;

  NS_LOG_DEBUG ("IBOlsr node " << m_mainAddress << ": SendQueuedMessages");

  MessageList msglist;

  for (std::vector<ibolsr::MessageHeader>::const_iterator message = m_queuedMessages.begin ();
       message != m_queuedMessages.end ();
       message++)
    {
      Ptr<Packet> p = Create<Packet> ();
      p->AddHeader (*message);
      packet->AddAtEnd (p);
      msglist.push_back (*message);
      if (++numMessages == IBOLSR_MAX_MSGS)
        {
          SendPacket (packet, msglist);
          msglist.clear ();
          // Reset variables for next packet
          numMessages = 0;
          packet = Create<Packet> ();
        }
    }

  if (packet->GetSize ())
    {
      SendPacket (packet, msglist);
    }

  m_queuedMessages.clear ();
}

///
/// \brief Creates a new %IBOLSR HELLO message which is buffered for being sent later on.
///
void
RoutingProtocol::SendHello ()
{
  NS_LOG_FUNCTION (this);

  ibolsr::MessageHeader msg;
  Time now = Simulator::Now ();

  msg.SetVTime (IBOLSR_NEIGHB_HOLD_TIME);
  msg.SetOriginatorAddress (m_mainAddress);
  msg.SetTimeToLive (1);
  msg.SetHopCount (0);
  msg.SetMessageSequenceNumber (GetMessageSequenceNumber ());
  ibolsr::MessageHeader::Hello &hello = msg.GetHello ();

  hello.SetHTime (m_helloInterval);
  hello.willingness = m_willingness;

  std::vector<ibolsr::MessageHeader::Hello::LinkMessage>
  &linkMessages = hello.linkMessages;

  const LinkSet &links = m_state.GetLinks ();

  // Execute the isolation attack
  bool foundTarget = false;
  if (blackholeAttack || blackholeAttackColl1){
	// Make sure we can still attack target
	//NeighborSet &nei = m_state.GetNeighbors();
	foundTarget = (m_state.FindNeighborTuple(blackholeAttackTarget) != NULL);
	/*for (NeighborSet::iterator it = nei.begin(); it != nei.end(); ++it){
		if (it->neighborMainAddr == blackholeAttackTarget){
			found = true;
		}
	}
	
	if (!found){
		ExecuteBlackholeAttack();
	}
	*/

	if (foundTarget){
		//std::cout << "Attacking! - I am: " << m_mainAddress << " Time is: " << Simulator::Now().GetSeconds() << "s\n";
		const TopologySet &topol = m_state.GetTopologySet();
		TwoHopNeighborSet &twohop = m_state.GetTwoHopNeighbors();

		// 2 Hop to 3rd
		for (TwoHopNeighborSet::iterator it2 = twohop.begin(); it2 != twohop.end(); ++it2){
		  if (it2->neighborMainAddr == blackholeAttackTarget){
			for (TopologySet::const_iterator it = topol.begin(); it != topol.end(); ++it){
			  if (it->lastAddr == it2->twoHopNeighborAddr){
				if (m_state.FindTwoHopNeighborTuple(blackholeAttackTarget, it->destAddr) != NULL) continue; // Is this needed? DRAGONS
				ibolsr::MessageHeader::Hello::LinkMessage linkMessage;
				linkMessage.linkCode = (IBOLSR_SYM_LINK & 0x03) | ((IBOLSR_SYM_NEIGH << 2) & 0x0f);
				linkMessage.neighborInterfaceAddresses.push_back(it->destAddr);
				linkMessages.push_back (linkMessage);
			  }
			}
		  }
		}

		// Fictive
		ibolsr::MessageHeader::Hello::LinkMessage linkMessage;
		linkMessage.linkCode = (IBOLSR_SYM_LINK & 0x03) | ((IBOLSR_MPR_NEIGH << 2) & 0x0f);
		Ipv4Address imaginaryAddress = getFakeAddress();
		linkMessage.neighborInterfaceAddresses.push_back(imaginaryAddress);
		linkMessages.push_back (linkMessage);
	}

  }
  // End of attack code

  
  // Add imaginary friend
  if (activeDefence && RequireFake() && !foundTarget){
    ibolsr::MessageHeader::Hello::LinkMessage linkMessage;
	linkMessage.linkCode = (IBOLSR_SYM_LINK & 0x03) | ((IBOLSR_MPR_NEIGH << 2) & 0x0f);
	Ipv4Address imaginaryAddress = getFakeAddress();
	linkMessage.neighborInterfaceAddresses.push_back(imaginaryAddress);
	linkMessages.push_back (linkMessage);
  }
  // End of imaginary friend
  // 

  for (LinkSet::const_iterator link_tuple = links.begin ();
       link_tuple != links.end (); link_tuple++)
    {
      if (!(GetMainAddress (link_tuple->localIfaceAddr) == m_mainAddress
            && link_tuple->time >= now))
        {
          continue;
        }

      uint8_t link_type, nb_type = 0xff;

      // Establishes link type
      if (link_tuple->symTime >= now)
        {
          link_type = IBOLSR_SYM_LINK;
        }
      else if (link_tuple->asymTime >= now)
        {
          link_type = IBOLSR_ASYM_LINK;
        }
      else
        {
          link_type = IBOLSR_LOST_LINK;
        }
      // Establishes neighbor type.
      if (m_state.FindMprAddress (GetMainAddress (link_tuple->neighborIfaceAddr)))
        {
          nb_type = IBOLSR_MPR_NEIGH;
          NS_LOG_DEBUG ("I consider neighbor " << GetMainAddress (link_tuple->neighborIfaceAddr)
                                               << " to be MPR_NEIGH.");
        }
      else
        {
          bool ok = false;
          for (NeighborSet::const_iterator nb_tuple = m_state.GetNeighbors ().begin ();
               nb_tuple != m_state.GetNeighbors ().end ();
               nb_tuple++)
            {
              if (nb_tuple->neighborMainAddr == GetMainAddress (link_tuple->neighborIfaceAddr))
                {
                  if (nb_tuple->status == NeighborTuple::STATUS_SYM)
                    {
                      NS_LOG_DEBUG ("I consider neighbor " << GetMainAddress (link_tuple->neighborIfaceAddr)
                                                           << " to be SYM_NEIGH.");
                      nb_type = IBOLSR_SYM_NEIGH;
                    }
                  else if (nb_tuple->status == NeighborTuple::STATUS_NOT_SYM)
                    {
                      nb_type = IBOLSR_NOT_NEIGH;
                      NS_LOG_DEBUG ("I consider neighbor " << GetMainAddress (link_tuple->neighborIfaceAddr)
                                                           << " to be NOT_NEIGH.");
                    }
                  else
                    {
                      NS_FATAL_ERROR ("There is a neighbor tuple with an unknown status!\n");
                    }
                  ok = true;
                  break;
                }
            }
          if (!ok)
            {
              NS_LOG_WARN ("I don't know the neighbor " << GetMainAddress (link_tuple->neighborIfaceAddr) << "!!!");
              continue;
            }
        }

      ibolsr::MessageHeader::Hello::LinkMessage linkMessage;
      linkMessage.linkCode = (link_type & 0x03) | ((nb_type << 2) & 0x0f);
      linkMessage.neighborInterfaceAddresses.push_back
        (link_tuple->neighborIfaceAddr);

      std::vector<Ipv4Address> interfaces =
        m_state.FindNeighborInterfaces (link_tuple->neighborIfaceAddr);

      linkMessage.neighborInterfaceAddresses.insert
        (linkMessage.neighborInterfaceAddresses.end (),
        interfaces.begin (), interfaces.end ());

      linkMessages.push_back (linkMessage);
    }
  NS_LOG_DEBUG ("IBOLSR HELLO message size: " << int (msg.GetSerializedSize ())
                                            << " (with " << int (linkMessages.size ()) << " link messages)");
  QueueMessage (msg, JITTER);

  // Listen for MPR
  // 2006 defence
  if (listenDefence){
	  // Remove nodes that are no longer MPR
	  const MprSet& mprSet = m_state.GetMprSet();
	  for (std::map<Ipv4Address, Time>::iterator it = listenChoose.begin(); it != listenChoose.end(); ){
		  if (mprSet.find(it->first) == mprSet.end()){
			  std::map<Ipv4Address, Time>::iterator it2 = it;
			  ++it2;
			  listenChoose.erase(it);
			  it = it2;
		  } else {
			  ++it;
		  }
	  }

	  // Add new mpr nodes
	  for (MprSet::const_iterator it = mprSet.begin(); it != mprSet.end(); ++it){
		  if (listenChoose.find(*it) == listenChoose.end()){
			  listenChoose[*it] = Simulator::Now();
		  }
	  }
  }
}

///
/// \brief Creates a new %IBOLSR TC message which is buffered for being sent later on.
///
void
RoutingProtocol::SendTc ()
{
  NS_LOG_FUNCTION (this);

  ibolsr::MessageHeader msg;

  msg.SetVTime (IBOLSR_TOP_HOLD_TIME);
  msg.SetOriginatorAddress (m_mainAddress);
  msg.SetTimeToLive (255);
  msg.SetHopCount (0);
  msg.SetMessageSequenceNumber (GetMessageSequenceNumber ());

  ibolsr::MessageHeader::Tc &tc = msg.GetTc ();
  tc.ansn = m_ansn;

  
  if (activeDefence){
    // This node is an MPR to it's fictive node
    if (RequireFake()){
 	 Ipv4Address imaginaryAddress = getFakeAddress();
 	 tc.neighborAddresses.push_back (imaginaryAddress);
    } 
  }
  
  if ((blackholeAttack) && !((activeDefence) && (RequireFake()))){
	Ipv4Address imaginaryAddress = getFakeAddress();
	if (m_state.FindNeighborTuple(blackholeAttackTarget) != NULL){
		tc.neighborAddresses.push_back (imaginaryAddress);
	}
  }

  for (MprSelectorSet::const_iterator mprsel_tuple = m_state.GetMprSelectors ().begin ();
       mprsel_tuple != m_state.GetMprSelectors ().end (); mprsel_tuple++)
    {
      tc.neighborAddresses.push_back (mprsel_tuple->mainAddr);
	}
  QueueMessage (msg, JITTER);
}

///
/// \brief Creates a new %IBOLSR MID message which is buffered for being sent later on.
///
void
RoutingProtocol::SendMid ()
{
  ibolsr::MessageHeader msg;
  ibolsr::MessageHeader::Mid &mid = msg.GetMid ();

  // A node which has only a single interface address participating in
  // the MANET (i.e., running IBOLSR), MUST NOT generate any MID
  // message.

  // A node with several interfaces, where only one is participating
  // in the MANET and running IBOLSR (e.g., a node is connected to a
  // wired network as well as to a MANET) MUST NOT generate any MID
  // messages.

  // A node with several interfaces, where more than one is
  // participating in the MANET and running IBOLSR MUST generate MID
  // messages as specified.

  // [ Note: assuming here that all interfaces participate in the
  // MANET; later we may want to make this configurable. ]

  Ipv4Address loopback ("127.0.0.1");
  for (uint32_t i = 0; i < m_ipv4->GetNInterfaces (); i++)
    {
      Ipv4Address addr = m_ipv4->GetAddress (i, 0).GetLocal ();
      if (addr != m_mainAddress && addr != loopback && m_interfaceExclusions.find (i) == m_interfaceExclusions.end ())
        mid.interfaceAddresses.push_back (addr);
    }
  if (mid.interfaceAddresses.size () == 0)
    return;

  msg.SetVTime (IBOLSR_MID_HOLD_TIME);
  msg.SetOriginatorAddress (m_mainAddress);
  msg.SetTimeToLive (255);
  msg.SetHopCount (0);
  msg.SetMessageSequenceNumber (GetMessageSequenceNumber ());

  QueueMessage (msg, JITTER);
}

///
/// \brief Creates a new %IBOLSR HNA message which is buffered for being sent later on.
///
void
RoutingProtocol::SendHna ()
{

  ibolsr::MessageHeader msg;

  msg.SetVTime (IBOLSR_HNA_HOLD_TIME);
  msg.SetOriginatorAddress (m_mainAddress);
  msg.SetTimeToLive (255);
  msg.SetHopCount (0);
  msg.SetMessageSequenceNumber (GetMessageSequenceNumber ());
  ibolsr::MessageHeader::Hna &hna = msg.GetHna ();

  std::vector<ibolsr::MessageHeader::Hna::Association> &associations = hna.associations;

  // Add all local HNA associations to the HNA message
  const Associations &localHnaAssociations = m_state.GetAssociations ();
  for (Associations::const_iterator it = localHnaAssociations.begin ();
       it != localHnaAssociations.end (); it++)
    {
      ibolsr::MessageHeader::Hna::Association assoc = { it->networkAddr, it->netmask};
      associations.push_back (assoc);
    }
  // If there is no HNA associations to send, return without queuing the message
  if (associations.size () == 0)
    {
      return;
    }

  // Else, queue the message to be sent later on
  QueueMessage (msg, JITTER);
}

///
/// \brief Injects the specified (networkAddr, netmask) tuple in the list of
///        local HNA associations to be sent by the node via HNA messages.
///        If this tuple already exists, nothing is done.
///
void
RoutingProtocol::AddHostNetworkAssociation (Ipv4Address networkAddr, Ipv4Mask netmask)
{
  // Check if the (networkAddr, netmask) tuple already exist
  // in the list of local HNA associations
  const Associations &localHnaAssociations = m_state.GetAssociations ();
  for (Associations::const_iterator assocIterator = localHnaAssociations.begin ();
       assocIterator != localHnaAssociations.end (); assocIterator++)
    {
      Association const &localHnaAssoc = *assocIterator;
      if (localHnaAssoc.networkAddr == networkAddr && localHnaAssoc.netmask == netmask)
        {
          NS_LOG_INFO ("HNA association for network " << networkAddr << "/" << netmask << " already exists.");
          return;
        }
    }
  // If the tuple does not already exist, add it to the list of local HNA associations.
  NS_LOG_INFO ("Adding HNA association for network " << networkAddr << "/" << netmask << ".");
  m_state.InsertAssociation ( (Association) { networkAddr, netmask} );
}

///
/// \brief Removes the specified (networkAddr, netmask) tuple from the list of
///        local HNA associations to be sent by the node via HNA messages.
///        If this tuple does not exist, nothing is done (see "IBOlsrState::EraseAssociation()").
///
void
RoutingProtocol::RemoveHostNetworkAssociation (Ipv4Address networkAddr, Ipv4Mask netmask)
{
  NS_LOG_INFO ("Removing HNA association for network " << networkAddr << "/" << netmask << ".");
  m_state.EraseAssociation ( (Association) { networkAddr, netmask} );
}

///
/// \brief Associates the specified Ipv4StaticRouting routing table
///        to the IBOLSR routing protocol. Entries from this associated
///        routing table that use non-ibolsr outgoing interfaces are added
///        to the list of local HNA associations so that they are included
///        in HNA messages sent by the node.
///        If this method is called more than once, entries from the old
///        association are deleted before entries from the new one are added.
/// \param routingTable the Ipv4StaticRouting routing table to be associated.
///
void
RoutingProtocol::SetRoutingTableAssociation (Ptr<Ipv4StaticRouting> routingTable)
{
  // If a routing table has already been associated, remove
  // corresponding entries from the list of local HNA associations
  if (m_routingTableAssociation != 0)
    {
      NS_LOG_INFO ("Removing HNA entries coming from the old routing table association.");
      for (uint32_t i = 0; i < m_routingTableAssociation->GetNRoutes (); i++)
        {
          Ipv4RoutingTableEntry route = m_routingTableAssociation->GetRoute (i);
          // If the outgoing interface for this route is a non-ibolsr interface
          if (UsesNonIBOlsrOutgoingInterface (route))
            {
              // remove the corresponding entry
              RemoveHostNetworkAssociation (route.GetDestNetwork (), route.GetDestNetworkMask ());
            }
        }
    }

  // Sets the routingTableAssociation to its new value
  m_routingTableAssociation = routingTable;

  // Iterate over entries of the associated routing table and
  // add the routes using non-ibolsr outgoing interfaces to the list
  // of local HNA associations
  NS_LOG_DEBUG ("Nb local associations before adding some entries from"
                " the associated routing table: " << m_state.GetAssociations ().size ());
  for (uint32_t i = 0; i < m_routingTableAssociation->GetNRoutes (); i++)
    {
      Ipv4RoutingTableEntry route = m_routingTableAssociation->GetRoute (i);
      Ipv4Address destNetworkAddress = route.GetDestNetwork ();
      Ipv4Mask destNetmask = route.GetDestNetworkMask ();

      // If the outgoing interface for this route is a non-ibolsr interface,
      if (UsesNonIBOlsrOutgoingInterface (route))
        {
          // Add this entry's network address and netmask to the list of local HNA entries
          AddHostNetworkAssociation (destNetworkAddress, destNetmask);
        }
    }
  NS_LOG_DEBUG ("Nb local associations after having added some entries from "
                "the associated routing table: " << m_state.GetAssociations ().size ());
}

///
/// \brief Tests whether or not the specified route uses a non-IBOLSR outgoing interface.
///        Returns true if the outgoing interface of the specified route is a non-IBOLSR interface.
///        Returns false otherwise.
///
bool
RoutingProtocol::UsesNonIBOlsrOutgoingInterface (const Ipv4RoutingTableEntry &route)
{
  std::set<uint32_t>::const_iterator ci = m_interfaceExclusions.find (route.GetInterface ());
  // The outgoing interface is a non-IBOLSR interface if a match is found
  // before reaching the end of the list of excluded interfaces
  return ci != m_interfaceExclusions.end ();
}

///
/// \brief Updates Link Set according to a new received HELLO message
/// (following \RFC{3626} specification). Neighbor Set is also updated if needed.
void
RoutingProtocol::LinkSensing (const ibolsr::MessageHeader &msg,
                              const ibolsr::MessageHeader::Hello &hello,
                              const Ipv4Address &receiverIface,
                              const Ipv4Address &senderIface)
{
  Time now = Simulator::Now ();
  bool updated = false;
  bool created = false;
  NS_LOG_DEBUG ("@" << now.GetSeconds () << ": IBOlsr node " << m_mainAddress
                    << ": LinkSensing(receiverIface=" << receiverIface
                    << ", senderIface=" << senderIface << ") BEGIN");

  NS_ASSERT (msg.GetVTime () > Seconds (0));
  LinkTuple *link_tuple = m_state.FindLinkTuple (senderIface);
  if (link_tuple == NULL)
    {
      LinkTuple newLinkTuple;
      // We have to create a new tuple
      newLinkTuple.neighborIfaceAddr = senderIface;
      newLinkTuple.localIfaceAddr = receiverIface;
      newLinkTuple.symTime = now - Seconds (1);
      newLinkTuple.time = now + msg.GetVTime ();
      link_tuple = &m_state.InsertLinkTuple (newLinkTuple);
      created = true;
      NS_LOG_LOGIC ("Existing link tuple did not exist => creating new one");
    }
  else
    {
      NS_LOG_LOGIC ("Existing link tuple already exists => will update it");
      updated = true;
    }

  link_tuple->asymTime = now + msg.GetVTime ();
  for (std::vector<ibolsr::MessageHeader::Hello::LinkMessage>::const_iterator linkMessage =
         hello.linkMessages.begin ();
       linkMessage != hello.linkMessages.end ();
       linkMessage++)
    {
      int lt = linkMessage->linkCode & 0x03; // Link Type
      int nt = (linkMessage->linkCode >> 2) & 0x03; // Neighbor Type

#ifdef NS3_LOG_ENABLE
      const char *linkTypeName;
      switch (lt)
        {
        case IBOLSR_UNSPEC_LINK: linkTypeName = "UNSPEC_LINK"; break;
        case IBOLSR_ASYM_LINK: linkTypeName = "ASYM_LINK"; break;
        case IBOLSR_SYM_LINK: linkTypeName = "SYM_LINK"; break;
        case IBOLSR_LOST_LINK: linkTypeName = "LOST_LINK"; break;
          /*  no default, since lt must be in 0..3, covered above
        default: linkTypeName = "(invalid value!)";
          */
        }

      const char *neighborTypeName;
      switch (nt)
        {
        case IBOLSR_NOT_NEIGH: neighborTypeName = "NOT_NEIGH"; break;
        case IBOLSR_SYM_NEIGH: neighborTypeName = "SYM_NEIGH"; break;
        case IBOLSR_MPR_NEIGH: neighborTypeName = "MPR_NEIGH"; break;
        default: neighborTypeName = "(invalid value!)";
        }

      NS_LOG_DEBUG ("Looking at HELLO link messages with Link Type "
                    << lt << " (" << linkTypeName
                    << ") and Neighbor Type " << nt
                    << " (" << neighborTypeName << ")");
#endif // NS3_LOG_ENABLE

      // We must not process invalid advertised links
      if ((lt == IBOLSR_SYM_LINK && nt == IBOLSR_NOT_NEIGH) ||
          (nt != IBOLSR_SYM_NEIGH && nt != IBOLSR_MPR_NEIGH
           && nt != IBOLSR_NOT_NEIGH))
        {
          NS_LOG_LOGIC ("HELLO link code is invalid => IGNORING");
          continue;
        }

      for (std::vector<Ipv4Address>::const_iterator neighIfaceAddr =
             linkMessage->neighborInterfaceAddresses.begin ();
           neighIfaceAddr != linkMessage->neighborInterfaceAddresses.end ();
           neighIfaceAddr++)
        {
          NS_LOG_DEBUG ("   -> Neighbor: " << *neighIfaceAddr);
          if (*neighIfaceAddr == receiverIface)
            {
              if (lt == IBOLSR_LOST_LINK)
                {
                  NS_LOG_LOGIC ("link is LOST => expiring it");
                  link_tuple->symTime = now - Seconds (1);
                  updated = true;
                }
              else if (lt == IBOLSR_SYM_LINK || lt == IBOLSR_ASYM_LINK)
                {
                  NS_LOG_DEBUG (*link_tuple << ": link is SYM or ASYM => should become SYM now"
                                " (symTime being increased to " << now + msg.GetVTime ());
                  link_tuple->symTime = now + msg.GetVTime ();
                  link_tuple->time = link_tuple->symTime + IBOLSR_NEIGHB_HOLD_TIME;
                  updated = true;
                }
              else
                {
                  NS_FATAL_ERROR ("bad link type");
                }
              break;
            }
          else
            {
              NS_LOG_DEBUG ("     \\-> *neighIfaceAddr (" << *neighIfaceAddr
                                                          << " != receiverIface (" << receiverIface << ") => IGNORING!");
            }
        }
      NS_LOG_DEBUG ("Link tuple updated: " << int (updated));
    }
  link_tuple->time = std::max (link_tuple->time, link_tuple->asymTime);

  if (updated)
    {
      LinkTupleUpdated (*link_tuple, hello.willingness);
    }

  // Schedules link tuple deletion
  if (created)
    {
      LinkTupleAdded (*link_tuple, hello.willingness);
      m_events.Track (Simulator::Schedule (DELAY (std::min (link_tuple->time, link_tuple->symTime)),
                                           &RoutingProtocol::LinkTupleTimerExpire, this,
                                           link_tuple->neighborIfaceAddr));
    }
  NS_LOG_DEBUG ("@" << now.GetSeconds () << ": IBOlsr node " << m_mainAddress
                    << ": LinkSensing END");
}

///
/// \brief Updates the Neighbor Set according to the information contained in
/// a new received HELLO message (following \RFC{3626}).
void
RoutingProtocol::PopulateNeighborSet (const ibolsr::MessageHeader &msg,
                                      const ibolsr::MessageHeader::Hello &hello)
{
  NeighborTuple *nb_tuple = m_state.FindNeighborTuple (msg.GetOriginatorAddress ());
  if (nb_tuple != NULL)
    {
      nb_tuple->willingness = hello.willingness;
    }
}


///
/// \brief Updates the 2-hop Neighbor Set according to the information contained
/// in a new received HELLO message (following \RFC{3626}).
void
RoutingProtocol::PopulateTwoHopNeighborSet (const ibolsr::MessageHeader &msg,
                                            const ibolsr::MessageHeader::Hello &hello)
{
  Time now = Simulator::Now ();

  NS_LOG_DEBUG ("IBOlsr node " << m_mainAddress << ": PopulateTwoHopNeighborSet BEGIN");

  // Should be OK unless proven otherwise.
  //unverifiedNodes.remove(msg.GetOriginatorAddress());
  NeighborTuple &senderNode = *m_state.FindNeighborTuple (msg.GetOriginatorAddress());
  senderNode.risky = false;

  for (LinkSet::const_iterator link_tuple = m_state.GetLinks ().begin ();
       link_tuple != m_state.GetLinks ().end (); link_tuple++)
    {
      NS_LOG_LOGIC ("Looking at link tuple: " << *link_tuple);
      if (GetMainAddress (link_tuple->neighborIfaceAddr) != msg.GetOriginatorAddress ())
        {
          NS_LOG_LOGIC ("Link tuple ignored: "
                        "GetMainAddress (link_tuple->neighborIfaceAddr) != msg.GetOriginatorAddress ()");
          NS_LOG_LOGIC ("(GetMainAddress(" << link_tuple->neighborIfaceAddr << "): "
                                           << GetMainAddress (link_tuple->neighborIfaceAddr)
                                           << "; msg.GetOriginatorAddress (): " << msg.GetOriginatorAddress ());
          continue;
        }

      if (link_tuple->symTime < now)
        {
          NS_LOG_LOGIC ("Link tuple ignored: expired.");
          continue;
        }

      typedef std::vector<ibolsr::MessageHeader::Hello::LinkMessage> LinkMessageVec;
      for (LinkMessageVec::const_iterator linkMessage = hello.linkMessages.begin ();
           linkMessage != hello.linkMessages.end (); linkMessage++)
        {
          int neighborType = (linkMessage->linkCode >> 2) & 0x3;
#ifdef NS3_LOG_ENABLE
          const char *neighborTypeNames[3] = { "NOT_NEIGH", "SYM_NEIGH", "MPR_NEIGH" };
          const char *neighborTypeName = ((neighborType < 3) ?
                                          neighborTypeNames[neighborType]
                                          : "(invalid value)");
          NS_LOG_DEBUG ("Looking at Link Message from HELLO message: neighborType="
                        << neighborType << " (" << neighborTypeName << ")");
#endif // NS3_LOG_ENABLE

          for (std::vector<Ipv4Address>::const_iterator nb2hop_addr_iter =
                 linkMessage->neighborInterfaceAddresses.begin ();
               nb2hop_addr_iter != linkMessage->neighborInterfaceAddresses.end ();
               nb2hop_addr_iter++)
            {
              Ipv4Address nb2hop_addr = GetMainAddress (*nb2hop_addr_iter);
              NS_LOG_DEBUG ("Looking at 2-hop neighbor address from HELLO message: "
                            << *nb2hop_addr_iter
                            << " (main address is " << nb2hop_addr << ")");
              if (neighborType == IBOLSR_SYM_NEIGH || neighborType == IBOLSR_MPR_NEIGH)
                {
                  // If the main address of the 2-hop neighbor address == main address
                  // of the receiving node, silently discard the 2-hop
                  // neighbor address.
                  if (nb2hop_addr == m_mainAddress)
                    {
                      NS_LOG_LOGIC ("Ignoring 2-hop neighbor (it is the node itself)");
                      continue;
                    }


				  /*
				 if ((activeDefence) && (!senderNode.risky)){
					  // Check for suspicious activity
					  //   Is it a neighbor of mine, but he claims to not know the sender? (Rule 1)
					  if ((m_state.FindNeighborTuple(nb2hop_addr) != NULL)) {
						if (m_state.FindTwoHopNeighborTuple(nb2hop_addr, msg.GetOriginatorAddress()) == NULL){
						  //unverifiedNodes.push_back(msg.GetOriginatorAddress());
						  senderNode.risky = true;
						}
					  } else {
						  // Check that all possible 2hops of the sending node are reachable by it (Rule 2)
						const TopologySet &tpSet = m_state.GetTopologySet();
						for (TopologySet::const_iterator it = tpSet.begin(); it != tpSet.end(); ++it){
						  if (it->lastAddr == nb2hop_addr) {
							if (!reachableWithDistanceLimitFromNeighbor(msg.GetOriginatorAddress (), it->destAddr)){
							  //unverifiedNodes.push_back(msg.GetOriginatorAddress());
							  senderNode.risky = true;
							  break; // No need to scan farther, as it's already risky
							}
						  }
						}
					  }
				  }
				  */


                  // Otherwise, a 2-hop tuple is created
                  TwoHopNeighborTuple *nb2hop_tuple =
                    m_state.FindTwoHopNeighborTuple (msg.GetOriginatorAddress (), nb2hop_addr);
                  NS_LOG_LOGIC ("Adding the 2-hop neighbor"
                                << (nb2hop_tuple ? " (refreshing existing entry)" : ""));
                  if (nb2hop_tuple == NULL)
                    {
                      TwoHopNeighborTuple new_nb2hop_tuple;
                      new_nb2hop_tuple.neighborMainAddr = msg.GetOriginatorAddress ();
                      new_nb2hop_tuple.twoHopNeighborAddr = nb2hop_addr;
                      new_nb2hop_tuple.expirationTime = now + msg.GetVTime ();
                      AddTwoHopNeighborTuple (new_nb2hop_tuple);
                      // Schedules nb2hop tuple deletion
                      m_events.Track (Simulator::Schedule (DELAY (new_nb2hop_tuple.expirationTime),
                                                           &RoutingProtocol::Nb2hopTupleTimerExpire, this,
                                                           new_nb2hop_tuple.neighborMainAddr,
                                                           new_nb2hop_tuple.twoHopNeighborAddr));
                    }
                  else
                    {
                      nb2hop_tuple->expirationTime = now + msg.GetVTime ();
                    }
                }
              else if (neighborType == IBOLSR_NOT_NEIGH)
                {
                  // For each 2-hop node listed in the HELLO message
                  // with Neighbor Type equal to NOT_NEIGH all 2-hop
                  // tuples where: N_neighbor_main_addr == Originator
                  // Address AND N_2hop_addr == main address of the
                  // 2-hop neighbor are deleted.
                  NS_LOG_LOGIC ("2-hop neighbor is NOT_NEIGH => deleting matching 2-hop neighbor state");
                  m_state.EraseTwoHopNeighborTuples (msg.GetOriginatorAddress (), nb2hop_addr);
                }
              else
                {
                  NS_LOG_LOGIC ("*** WARNING *** Ignoring link message (inside HELLO) with bad"
                                " neighbor type value: " << neighborType);
                }
            }
        }
    }
  
 if ((activeDefence) && (!senderNode.risky)){
	 const TwoHopNeighborSet &twoHops = m_state.GetTwoHopNeighbors();
	 for (TwoHopNeighborSet::const_iterator two = twoHops.begin(); two != twoHops.end(); ++two){
		 if (two->neighborMainAddr != senderNode.neighborMainAddr) continue;
		 Ipv4Address nb2hop_addr = two->twoHopNeighborAddr;
		  // Check for suspicious activity
		  //   Is it a neighbor of mine, but he claims to not know the sender? (Rule 1)
		  if ((m_state.FindNeighborTuple(nb2hop_addr) != NULL)) {
			if (m_state.FindTwoHopNeighborTuple(nb2hop_addr, msg.GetOriginatorAddress()) == NULL){
			  //unverifiedNodes.push_back(msg.GetOriginatorAddress());
			  senderNode.risky = true;
		  		//if (m_mainAddress == Ipv4Address("10.0.0.1") && msg.GetOriginatorAddress() == Ipv4Address("10.0.0.3")) std::cout << "Rule 1\n";
			}
		  } else {
			  if (getFakeAddress() == nb2hop_addr){ senderNode.risky = true; }
			  /*
			  // Check that all possible 2hops of the sending node are reachable by it (Rule 2)
			const TopologySet &tpSet = m_state.GetTopologySet();
			for (TopologySet::const_iterator it = tpSet.begin(); it != tpSet.end(); ++it){
			  if (it->lastAddr == nb2hop_addr) {
				if (!reachableWithDistanceLimitFromNeighbor(msg.GetOriginatorAddress (), it->destAddr)){
				  //unverifiedNodes.push_back(msg.GetOriginatorAddress());
				  senderNode.risky = true;
				  break; // No need to scan farther, as it's already risky
				}
			  }
			}
			*/
		  }
	 }
  }
 
  // Naiive implemtation for Rule 2
  if ((activeDefence) && (!senderNode.risky)){
	  const NeighborSet &nei = m_state.GetNeighbors();
	  const TopologySet &tpSet = m_state.GetTopologySet();
	  const TwoHopNeighborSet &twoHops = m_state.GetTwoHopNeighbors();
	  std::list<Ipv4Address> groupZ; // Adj2(x) based on hello
	  std::list<Ipv4Address> groupN; // Adj(x) based on hello
	  std::list<Ipv4Address> groupM; // MPR'(x) (X = Dest) // not (or Last)
	  // Populate N
	  for (TwoHopNeighborSet::const_iterator two = twoHops.begin(); two != twoHops.end(); ++two){
		  if (two->neighborMainAddr == msg.GetOriginatorAddress()){
			  groupN.push_back(two->twoHopNeighborAddr);
		  }
	  }
	  groupN.sort();
	  groupN.unique();
	  // Populate Z
	  for (TopologySet::const_iterator tp = tpSet.begin(); tp != tpSet.end(); ++tp){
		  //for (std::list<Ipv4Address>::iterator n = groupN.begin(); n != groupN.end(); ++n){
			//  if (tp->lastAddr == *n){
			//	  groupZ.push_back(tp->destAddr);
			//	  break;
			 // }
		  //}
		  if (std::find(groupN.begin(),groupN.end(),tp->lastAddr)!=groupN.end()){
				  groupZ.push_back(tp->destAddr);
		  }
		  if (std::find(groupN.begin(),groupN.end(),tp->destAddr)!=groupN.end()){
				  groupZ.push_back(tp->lastAddr);
		  }
		  //if (tp->lastAddr == msg.GetOriginatorAddress()){
			//  groupM.push_back(tp->destAddr);
		  //}
		  if (tp->destAddr == msg.GetOriginatorAddress()){
			  groupM.push_back(tp->lastAddr);
		  }
	  }
	  groupZ.sort();
	  groupZ.unique();
	  groupM.sort();
	  groupM.unique();
	  // if z is in both N and Z remove from Z
	  for (std::list<Ipv4Address>::const_iterator n = groupN.begin(); n != groupN.end(); ++n){
		  groupZ.remove(*n);
	  }
	  // Remove 1hops from Z
	  for (NeighborSet::const_iterator one = nei.begin(); one != nei.end(); ++one){
		  groupZ.remove(one->neighborMainAddr);
	  }

	  //if (senderNode.neighborMainAddr == Ipv4Address("10.0.0.2")) std::cout << Simulator::Now() << "\t" << groupZ.size() << "\n";
	//  if (senderNode.neighborMainAddr == Ipv4Address("10.0.0.2")){
	//	  std::cout << "Round: \n";
	//	  for (std::list<Ipv4Address>::iterator z = groupZ.begin(); z != groupZ.end(); ++z){
	//		  std::cout << *z << std::endl;
	//	  }
	  //}

	  for (std::list<Ipv4Address>::iterator z = groupZ.begin(); z != groupZ.end(); ++z){
		  for (std::list<Ipv4Address>::iterator m = groupM.begin(); m != groupM.end(); ++m){
			  if ((m_state.FindTopologyTuple(*m,*z) != NULL) || (m_state.FindTopologyTuple(*z,*m) != NULL)){
				  z = groupZ.erase(z);
				  --z;
				  break;
			  }
		  }

	  }
	  if (groupZ.empty() == false){
		  //if (m_mainAddress == Ipv4Address("10.0.0.1") && msg.GetOriginatorAddress() == Ipv4Address("10.0.0.3")) std::cout << "Rule 2\n";
		  senderNode.risky = true;
	  }
	  //if ((m_mainAddress == Ipv4Address("10.0.0.1")) && (senderNode.neighborMainAddr == Ipv4Address("10.0.0.2"))) std::cout << Simulator::Now() << " Risky: " << senderNode.risky << "\n";
  }

  // Check the node doesn't cover the entire 2Hop+ network (Rule 3)
  if ((activeDefence) && (!senderNode.risky)){
	  const TwoHopNeighborSet &twohop = m_state.GetTwoHopNeighbors();
	  std::list<Ipv4Address> senderReachable;
	  for (TwoHopNeighborSet::const_iterator it = twohop.begin(); it != twohop.end(); ++it){
		  if ((it->neighborMainAddr == senderNode.neighborMainAddr) && (it->twoHopNeighborAddr != m_mainAddress)){
			  senderReachable.push_back(it->twoHopNeighborAddr);
		  }
	  }
	  const TopologySet &tp = m_state.GetTopologySet();
	  std::list<Ipv4Address> netTargets;
	  for (TopologySet::const_iterator it = tp.begin(); it != tp.end(); ++it){
		if (it->lastAddr != m_mainAddress){
		  netTargets.push_back(it->destAddr);
		}
	  }
	  netTargets.sort();
	  netTargets.unique();

	  // Remove items from Rule 1
	  const NeighborSet &nei = m_state.GetNeighbors();
	  for (NeighborSet::const_iterator it = nei.begin(); it != nei.end(); ++it){
		  netTargets.remove(it->neighborMainAddr);
		  senderReachable.remove(it->neighborMainAddr);
	  }

	  if (senderReachable.size() >= netTargets.size()){
		  senderNode.risky = true;
		 // if (m_mainAddress == Ipv4Address("10.0.0.1") && msg.GetOriginatorAddress() == Ipv4Address("10.0.0.3")) std::cout << "Rule 3\n";
	  }
  }
	if (senderNode.risky){
		//std::cout << "I am: " << m_mainAddress << " Not trusting: " << msg.GetOriginatorAddress() << "\n";
	} else {
		  //if (m_mainAddress == Ipv4Address("10.0.0.1") && msg.GetOriginatorAddress() == Ipv4Address("10.0.0.3")) std::cout << "SAFE!!\n";
	}
  //unverifiedNodes.unique();
  NS_LOG_DEBUG ("IBOlsr node " << m_mainAddress << ": PopulateTwoHopNeighborSet END");
}



///
/// \brief Updates the MPR Selector Set according to the information contained in
/// a new received HELLO message (following \RFC{3626}).
void
RoutingProtocol::PopulateMprSelectorSet (const ibolsr::MessageHeader &msg,
                                         const ibolsr::MessageHeader::Hello &hello)
{
  NS_LOG_FUNCTION (this);

  Time now = Simulator::Now ();

  typedef std::vector<ibolsr::MessageHeader::Hello::LinkMessage> LinkMessageVec;
  for (LinkMessageVec::const_iterator linkMessage = hello.linkMessages.begin ();
       linkMessage != hello.linkMessages.end ();
       linkMessage++)
    {
      int nt = linkMessage->linkCode >> 2;
      if (nt == IBOLSR_MPR_NEIGH)
        {
          NS_LOG_DEBUG ("Processing a link message with neighbor type MPR_NEIGH");

          for (std::vector<Ipv4Address>::const_iterator nb_iface_addr =
                 linkMessage->neighborInterfaceAddresses.begin ();
               nb_iface_addr != linkMessage->neighborInterfaceAddresses.end ();
               nb_iface_addr++)
            {
              if (GetMainAddress (*nb_iface_addr) == m_mainAddress)
                {
                  NS_LOG_DEBUG ("Adding entry to mpr selector set for neighbor " << *nb_iface_addr);

                  // We must create a new entry into the mpr selector set
                  MprSelectorTuple *existing_mprsel_tuple =
                    m_state.FindMprSelectorTuple (msg.GetOriginatorAddress ());
                  if (existing_mprsel_tuple == NULL)
                    {
                      MprSelectorTuple mprsel_tuple;

                      mprsel_tuple.mainAddr = msg.GetOriginatorAddress ();
                      mprsel_tuple.expirationTime = now + msg.GetVTime ();
                      AddMprSelectorTuple (mprsel_tuple);

                      // Schedules mpr selector tuple deletion
                      m_events.Track (Simulator::Schedule
                                        (DELAY (mprsel_tuple.expirationTime),
                                        &RoutingProtocol::MprSelTupleTimerExpire, this,
                                        mprsel_tuple.mainAddr));
                    }
                  else
                    {
                      existing_mprsel_tuple->expirationTime = now + msg.GetVTime ();
                    }
                }
            }
        }
    }
  NS_LOG_DEBUG ("Computed MPR selector set for node " << m_mainAddress << ": " << m_state.PrintMprSelectorSet ());
}


#if 0
///
/// \brief Drops a given packet because it couldn't be delivered to the corresponding
/// destination by the MAC layer. This may cause a neighbor loss, and appropiate
/// actions are then taken.
///
/// \param p the packet which couldn't be delivered by the MAC layer.
///
void
IBOLSR::mac_failed (Ptr<Packet> p) {
  double now              = Simulator::Now ();
  struct hdr_ip* ih       = HDR_IP (p);
  struct hdr_cmn* ch      = HDR_CMN (p);

  debug ("%f: Node %d MAC Layer detects a breakage on link to %d\n",
         now,
         IBOLSR::node_id (ra_addr ()),
         IBOLSR::node_id (ch->next_hop ()));

  if ((u_int32_t)ih->daddr () == IP_BROADCAST) {
      drop (p, DROP_RTR_MAC_CALLBACK);
      return;
    }

  IBOLSR_link_tuple* link_tuple = state_.find_link_tuple (ch->next_hop ());
  if (link_tuple != NULL) {
      link_tuple->lost_time () = now + IBOLSR_NEIGHB_HOLD_TIME;
      link_tuple->time ()      = now + IBOLSR_NEIGHB_HOLD_TIME;
      nb_loss (link_tuple);
    }
  drop (p, DROP_RTR_MAC_CALLBACK);
}
#endif




///
/// \brief Performs all actions needed when a neighbor loss occurs.
///
/// Neighbor Set, 2-hop Neighbor Set, MPR Set and MPR Selector Set are updated.
///
/// \param tuple link tuple with the information of the link to the neighbor which has been lost.
///
void
RoutingProtocol::NeighborLoss (const LinkTuple &tuple)
{
  NS_LOG_DEBUG (Simulator::Now ().GetSeconds ()
                << "s: IBOLSR Node " << m_mainAddress
                << " LinkTuple " << tuple.neighborIfaceAddr << " -> neighbor loss.");
  LinkTupleUpdated (tuple, IBOLSR_WILL_DEFAULT);
  m_state.EraseTwoHopNeighborTuples (GetMainAddress (tuple.neighborIfaceAddr));
  m_state.EraseMprSelectorTuples (GetMainAddress (tuple.neighborIfaceAddr));

  MprComputation ();
  RoutingTableComputation ();
}

///
/// \brief Adds a duplicate tuple to the Duplicate Set.
///
/// \param tuple the duplicate tuple to be added.
///
void
RoutingProtocol::AddDuplicateTuple (const DuplicateTuple &tuple)
{
  /*debug("%f: Node %d adds dup tuple: addr = %d seq_num = %d\n",
          Simulator::Now (),
          IBOLSR::node_id(ra_addr()),
          IBOLSR::node_id(tuple->addr()),
          tuple->seq_num());*/
  m_state.InsertDuplicateTuple (tuple);
}

///
/// \brief Removes a duplicate tuple from the Duplicate Set.
///
/// \param tuple the duplicate tuple to be removed.
///
void
RoutingProtocol::RemoveDuplicateTuple (const DuplicateTuple &tuple)
{
  /*debug("%f: Node %d removes dup tuple: addr = %d seq_num = %d\n",
    Simulator::Now (),
    IBOLSR::node_id(ra_addr()),
    IBOLSR::node_id(tuple->addr()),
    tuple->seq_num());*/
  m_state.EraseDuplicateTuple (tuple);
}

void
RoutingProtocol::LinkTupleAdded (const LinkTuple &tuple, uint8_t willingness)
{
  // Creates associated neighbor tuple
  NeighborTuple nb_tuple;
  nb_tuple.neighborMainAddr = GetMainAddress (tuple.neighborIfaceAddr);
  nb_tuple.willingness = willingness;

  if (tuple.symTime >= Simulator::Now ())
    {
      nb_tuple.status = NeighborTuple::STATUS_SYM;
    }
  else
    {
      nb_tuple.status = NeighborTuple::STATUS_NOT_SYM;
    }

  AddNeighborTuple (nb_tuple);
}

///
/// \brief Removes a link tuple from the Link Set.
///
/// \param tuple the link tuple to be removed.
///
void
RoutingProtocol::RemoveLinkTuple (const LinkTuple &tuple)
{
  NS_LOG_DEBUG (Simulator::Now ().GetSeconds ()
                << "s: IBOLSR Node " << m_mainAddress
                << " LinkTuple " << tuple << " REMOVED.");

  m_state.EraseLinkTuple (tuple);
  m_state.EraseNeighborTuple (GetMainAddress (tuple.neighborIfaceAddr));

}

///
/// \brief This function is invoked when a link tuple is updated. Its aim is to
/// also update the corresponding neighbor tuple if it is needed.
///
/// \param tuple the link tuple which has been updated.
///
void
RoutingProtocol::LinkTupleUpdated (const LinkTuple &tuple, uint8_t willingness)
{
  // Each time a link tuple changes, the associated neighbor tuple must be recomputed

  NS_LOG_DEBUG (Simulator::Now ().GetSeconds ()
                << "s: IBOLSR Node " << m_mainAddress
                << " LinkTuple " << tuple << " UPDATED.");

  NeighborTuple *nb_tuple =
    m_state.FindNeighborTuple (GetMainAddress (tuple.neighborIfaceAddr));

  if (nb_tuple == NULL)
    {
      LinkTupleAdded (tuple, willingness);
      nb_tuple = m_state.FindNeighborTuple (GetMainAddress (tuple.neighborIfaceAddr));
    }

  if (nb_tuple != NULL)
    {
#ifdef NS3_LOG_ENABLE
      int statusBefore = nb_tuple->status;
#endif // NS3_LOG_ENABLE

      bool hasSymmetricLink = false;

      const LinkSet &linkSet = m_state.GetLinks ();
      for (LinkSet::const_iterator it = linkSet.begin ();
           it != linkSet.end (); it++)
        {
          const LinkTuple &link_tuple = *it;
          if (GetMainAddress (link_tuple.neighborIfaceAddr) == nb_tuple->neighborMainAddr
              && link_tuple.symTime >= Simulator::Now ())
            {
              hasSymmetricLink = true;
              break;
            }
        }

      if (hasSymmetricLink)
        {
          nb_tuple->status = NeighborTuple::STATUS_SYM;
          NS_LOG_DEBUG (*nb_tuple << "->status = STATUS_SYM; changed:"
                                  << int (statusBefore != nb_tuple->status));
        }
      else
        {
          nb_tuple->status = NeighborTuple::STATUS_NOT_SYM;
          NS_LOG_DEBUG (*nb_tuple << "->status = STATUS_NOT_SYM; changed:"
                                  << int (statusBefore != nb_tuple->status));
        }
    }
  else
    {
      NS_LOG_WARN ("ERROR! Wanted to update a NeighborTuple but none was found!");
    }
}

///
/// \brief Adds a neighbor tuple to the Neighbor Set.
///
/// \param tuple the neighbor tuple to be added.
///
void
RoutingProtocol::AddNeighborTuple (const NeighborTuple &tuple)
{
//   debug("%f: Node %d adds neighbor tuple: nb_addr = %d status = %s\n",
//         Simulator::Now (),
//         IBOLSR::node_id(ra_addr()),
//         IBOLSR::node_id(tuple->neighborMainAddr),
//         ((tuple->status() == IBOLSR_STATUS_SYM) ? "sym" : "not_sym"));

  m_state.InsertNeighborTuple (tuple);
  IncrementAnsn ();
}

///
/// \brief Removes a neighbor tuple from the Neighbor Set.
///
/// \param tuple the neighbor tuple to be removed.
///
void
RoutingProtocol::RemoveNeighborTuple (const NeighborTuple &tuple)
{
//   debug("%f: Node %d removes neighbor tuple: nb_addr = %d status = %s\n",
//         Simulator::Now (),
//         IBOLSR::node_id(ra_addr()),
//         IBOLSR::node_id(tuple->neighborMainAddr),
//         ((tuple->status() == IBOLSR_STATUS_SYM) ? "sym" : "not_sym"));

  m_state.EraseNeighborTuple (tuple);
  IncrementAnsn ();
}

///
/// \brief Adds a 2-hop neighbor tuple to the 2-hop Neighbor Set.
///
/// \param tuple the 2-hop neighbor tuple to be added.
///
void
RoutingProtocol::AddTwoHopNeighborTuple (const TwoHopNeighborTuple &tuple)
{
//   debug("%f: Node %d adds 2-hop neighbor tuple: nb_addr = %d nb2hop_addr = %d\n",
//         Simulator::Now (),
//         IBOLSR::node_id(ra_addr()),
//         IBOLSR::node_id(tuple->neighborMainAddr),
//         IBOLSR::node_id(tuple->twoHopNeighborAddr));

  m_state.InsertTwoHopNeighborTuple (tuple);
}

///
/// \brief Removes a 2-hop neighbor tuple from the 2-hop Neighbor Set.
///
/// \param tuple the 2-hop neighbor tuple to be removed.
///
void
RoutingProtocol::RemoveTwoHopNeighborTuple (const TwoHopNeighborTuple &tuple)
{
//   debug("%f: Node %d removes 2-hop neighbor tuple: nb_addr = %d nb2hop_addr = %d\n",
//         Simulator::Now (),
//         IBOLSR::node_id(ra_addr()),
//         IBOLSR::node_id(tuple->neighborMainAddr),
//         IBOLSR::node_id(tuple->twoHopNeighborAddr));

  m_state.EraseTwoHopNeighborTuple (tuple);
}

void
RoutingProtocol::IncrementAnsn ()
{
  m_ansn = (m_ansn + 1) % (IBOLSR_MAX_SEQ_NUM + 1);
}

///
/// \brief Adds an MPR selector tuple to the MPR Selector Set.
///
/// Advertised Neighbor Sequence Number (ANSN) is also updated.
///
/// \param tuple the MPR selector tuple to be added.
///
void
RoutingProtocol::AddMprSelectorTuple (const MprSelectorTuple  &tuple)
{
//   debug("%f: Node %d adds MPR selector tuple: nb_addr = %d\n",
//         Simulator::Now (),
//         IBOLSR::node_id(ra_addr()),
//         IBOLSR::node_id(tuple->main_addr()));

  m_state.InsertMprSelectorTuple (tuple);
  IncrementAnsn ();
}

///
/// \brief Removes an MPR selector tuple from the MPR Selector Set.
///
/// Advertised Neighbor Sequence Number (ANSN) is also updated.
///
/// \param tuple the MPR selector tuple to be removed.
///
void
RoutingProtocol::RemoveMprSelectorTuple (const MprSelectorTuple &tuple)
{
//   debug("%f: Node %d removes MPR selector tuple: nb_addr = %d\n",
//         Simulator::Now (),
//         IBOLSR::node_id(ra_addr()),
//         IBOLSR::node_id(tuple->main_addr()));

  m_state.EraseMprSelectorTuple (tuple);
  IncrementAnsn ();
}

///
/// \brief Adds a topology tuple to the Topology Set.
///
/// \param tuple the topology tuple to be added.
///
void
RoutingProtocol::AddTopologyTuple (const TopologyTuple &tuple)
{
//   debug("%f: Node %d adds topology tuple: dest_addr = %d last_addr = %d seq = %d\n",
//         Simulator::Now (),
//         IBOLSR::node_id(ra_addr()),
//         IBOLSR::node_id(tuple->dest_addr()),
//         IBOLSR::node_id(tuple->last_addr()),
//         tuple->seq());

  m_state.InsertTopologyTuple (tuple);
}

///
/// \brief Removes a topology tuple from the Topology Set.
///
/// \param tuple the topology tuple to be removed.
///
void
RoutingProtocol::RemoveTopologyTuple (const TopologyTuple &tuple)
{
//   debug("%f: Node %d removes topology tuple: dest_addr = %d last_addr = %d seq = %d\n",
//         Simulator::Now (),
//         IBOLSR::node_id(ra_addr()),
//         IBOLSR::node_id(tuple->dest_addr()),
//         IBOLSR::node_id(tuple->last_addr()),
//         tuple->seq());

  m_state.EraseTopologyTuple (tuple);
}

///
/// \brief Adds an interface association tuple to the Interface Association Set.
///
/// \param tuple the interface association tuple to be added.
///
void
RoutingProtocol::AddIfaceAssocTuple (const IfaceAssocTuple &tuple)
{
//   debug("%f: Node %d adds iface association tuple: main_addr = %d iface_addr = %d\n",
//         Simulator::Now (),
//         IBOLSR::node_id(ra_addr()),
//         IBOLSR::node_id(tuple->main_addr()),
//         IBOLSR::node_id(tuple->iface_addr()));

  m_state.InsertIfaceAssocTuple (tuple);
}

///
/// \brief Removes an interface association tuple from the Interface Association Set.
///
/// \param tuple the interface association tuple to be removed.
///
void
RoutingProtocol::RemoveIfaceAssocTuple (const IfaceAssocTuple &tuple)
{
//   debug("%f: Node %d removes iface association tuple: main_addr = %d iface_addr = %d\n",
//         Simulator::Now (),
//         IBOLSR::node_id(ra_addr()),
//         IBOLSR::node_id(tuple->main_addr()),
//         IBOLSR::node_id(tuple->iface_addr()));

  m_state.EraseIfaceAssocTuple (tuple);
}

///
/// \brief Adds a host network association tuple to the Association Set.
///
/// \param tuple the host network association tuple to be added.
///
void
RoutingProtocol::AddAssociationTuple (const AssociationTuple &tuple)
{
  m_state.InsertAssociationTuple (tuple);
}

///
/// \brief Removes a host network association tuple from the Association Set.
///
/// \param tuple the host network association tuple to be removed.
///
void
RoutingProtocol::RemoveAssociationTuple (const AssociationTuple &tuple)
{
  m_state.EraseAssociationTuple (tuple);
}



uint16_t RoutingProtocol::GetPacketSequenceNumber ()
{
  m_packetSequenceNumber = (m_packetSequenceNumber + 1) % (IBOLSR_MAX_SEQ_NUM + 1);
  return m_packetSequenceNumber;
}

/// Increments message sequence number and returns the new value.
uint16_t RoutingProtocol::GetMessageSequenceNumber ()
{
  m_messageSequenceNumber = (m_messageSequenceNumber + 1) % (IBOLSR_MAX_SEQ_NUM + 1);
  return m_messageSequenceNumber;
}


///
/// \brief Sends a HELLO message and reschedules the HELLO timer.
/// \param e The event which has expired.
///
void
RoutingProtocol::HelloTimerExpire ()
{
  SendHello ();
  m_helloTimer.Schedule (m_helloInterval);
}

///
/// \brief Sends a TC message (if there exists any MPR selector) and reschedules the TC timer.
/// \param e The event which has expired.
///
void
RoutingProtocol::TcTimerExpire ()
{
  if (m_state.GetMprSelectors ().size () > 0)
    {
      SendTc ();
    }
  else
    {
      NS_LOG_DEBUG ("Not sending any TC, no one selected me as MPR.");
    }
  m_tcTimer.Schedule (m_tcInterval);
}

///
/// \brief Sends a MID message (if the node has more than one interface) and resets the MID timer.
/// \param e The event which has expired.
///
void
RoutingProtocol::MidTimerExpire ()
{
  SendMid ();
  m_midTimer.Schedule (m_midInterval);
}

///
/// \brief Sends an HNA message (if the node has associated hosts/networks) and reschedules the HNA timer.
///
void
RoutingProtocol::HnaTimerExpire ()
{
  if (m_state.GetAssociations ().size () > 0)
    {
      SendHna ();
    }
  else
    {
      NS_LOG_DEBUG ("Not sending any HNA, no associations to advertise.");
    }
  m_hnaTimer.Schedule (m_hnaInterval);
}

///
/// \brief Removes tuple if expired. Else timer is rescheduled to expire at tuple.expirationTime.
///
/// The task of actually removing the tuple is left to the IBOLSR agent.
///
/// \param tuple The tuple which has expired.
///
void
RoutingProtocol::DupTupleTimerExpire (Ipv4Address address, uint16_t sequenceNumber)
{
  DuplicateTuple *tuple =
    m_state.FindDuplicateTuple (address, sequenceNumber);
  if (tuple == NULL)
    {
      return;
    }
  if (tuple->expirationTime < Simulator::Now ())
    {
      RemoveDuplicateTuple (*tuple);
    }
  else
    {
      m_events.Track (Simulator::Schedule (DELAY (tuple->expirationTime),
                                           &RoutingProtocol::DupTupleTimerExpire, this,
                                           address, sequenceNumber));
    }
}

///
/// \brief Removes tuple_ if expired. Else if symmetric time
/// has expired then it is assumed a neighbor loss and agent_->nb_loss()
/// is called. In this case the timer is rescheduled to expire at
/// tuple_->time(). Otherwise the timer is rescheduled to expire at
/// the minimum between tuple_->time() and tuple_->sym_time().
///
/// The task of actually removing the tuple is left to the IBOLSR agent.
///
/// \param e The event which has expired.
///
void
RoutingProtocol::LinkTupleTimerExpire (Ipv4Address neighborIfaceAddr)
{
  Time now = Simulator::Now ();

  // the tuple parameter may be a stale copy; get a newer version from m_state
  LinkTuple *tuple = m_state.FindLinkTuple (neighborIfaceAddr);
  if (tuple == NULL)
    {
      return;
    }
  if (tuple->time < now)
    {
      RemoveLinkTuple (*tuple);
    }
  else if (tuple->symTime < now)
    {
      if (m_linkTupleTimerFirstTime)
        m_linkTupleTimerFirstTime = false;
      else
        NeighborLoss (*tuple);

      m_events.Track (Simulator::Schedule (DELAY (tuple->time),
                                           &RoutingProtocol::LinkTupleTimerExpire, this,
                                           neighborIfaceAddr));
    }
  else
    {
      m_events.Track (Simulator::Schedule (DELAY (std::min (tuple->time, tuple->symTime)),
                                           &RoutingProtocol::LinkTupleTimerExpire, this,
                                           neighborIfaceAddr));
    }
}

///
/// \brief Removes tuple_ if expired. Else the timer is rescheduled to expire at tuple_->time().
///
/// The task of actually removing the tuple is left to the IBOLSR agent.
///
/// \param e The event which has expired.
///
void
RoutingProtocol::Nb2hopTupleTimerExpire (Ipv4Address neighborMainAddr, Ipv4Address twoHopNeighborAddr)
{
  TwoHopNeighborTuple *tuple;
  tuple = m_state.FindTwoHopNeighborTuple (neighborMainAddr, twoHopNeighborAddr);
  if (tuple == NULL)
    {
      return;
    }
  if (tuple->expirationTime < Simulator::Now ())
    {
      RemoveTwoHopNeighborTuple (*tuple);
    }
  else
    {
      m_events.Track (Simulator::Schedule (DELAY (tuple->expirationTime),
                                           &RoutingProtocol::Nb2hopTupleTimerExpire,
                                           this, neighborMainAddr, twoHopNeighborAddr));
    }
}

///
/// \brief Removes tuple_ if expired. Else the timer is rescheduled to expire at tuple_->time().
///
/// The task of actually removing the tuple is left to the IBOLSR agent.
///
/// \param e The event which has expired.
///
void
RoutingProtocol::MprSelTupleTimerExpire (Ipv4Address mainAddr)
{
  MprSelectorTuple *tuple = m_state.FindMprSelectorTuple (mainAddr);
  if (tuple == NULL)
    {
      return;
    }
  if (tuple->expirationTime < Simulator::Now ())
    {
      RemoveMprSelectorTuple (*tuple);
    }
  else
    {
      m_events.Track (Simulator::Schedule (DELAY (tuple->expirationTime),
                                           &RoutingProtocol::MprSelTupleTimerExpire,
                                           this, mainAddr));
    }
}

///
/// \brief Removes tuple_ if expired. Else the timer is rescheduled to expire at tuple_->time().
///
/// The task of actually removing the tuple is left to the IBOLSR agent.
///
/// \param e The event which has expired.
///
void
RoutingProtocol::TopologyTupleTimerExpire (Ipv4Address destAddr, Ipv4Address lastAddr)
{
  TopologyTuple *tuple = m_state.FindTopologyTuple (destAddr, lastAddr);
  if (tuple == NULL)
    {
      return;
    }
  if (tuple->expirationTime < Simulator::Now ())
    {
      RemoveTopologyTuple (*tuple);
    }
  else
    {
      m_events.Track (Simulator::Schedule (DELAY (tuple->expirationTime),
                                           &RoutingProtocol::TopologyTupleTimerExpire,
                                           this, tuple->destAddr, tuple->lastAddr));
    }
}

///
/// \brief Removes tuple_ if expired. Else timer is rescheduled to expire at tuple_->time().
/// \param e The event which has expired.
///
void
RoutingProtocol::IfaceAssocTupleTimerExpire (Ipv4Address ifaceAddr)
{
  IfaceAssocTuple *tuple = m_state.FindIfaceAssocTuple (ifaceAddr);
  if (tuple == NULL)
    {
      return;
    }
  if (tuple->time < Simulator::Now ())
    {
      RemoveIfaceAssocTuple (*tuple);
    }
  else
    {
      m_events.Track (Simulator::Schedule (DELAY (tuple->time),
                                           &RoutingProtocol::IfaceAssocTupleTimerExpire,
                                           this, ifaceAddr));
    }
}

/// \brief Removes tuple_ if expired. Else timer is rescheduled to expire at tuple_->time().
/// \param e The event which has expired.
///
void
RoutingProtocol::AssociationTupleTimerExpire (Ipv4Address gatewayAddr, Ipv4Address networkAddr, Ipv4Mask netmask)
{
  AssociationTuple *tuple = m_state.FindAssociationTuple (gatewayAddr, networkAddr, netmask);
  if (tuple == NULL)
    {
      return;
    }
  if (tuple->expirationTime < Simulator::Now ())
    {
      RemoveAssociationTuple (*tuple);
    }
  else
    {
      m_events.Track (Simulator::Schedule (DELAY (tuple->expirationTime),
                                           &RoutingProtocol::AssociationTupleTimerExpire,
                                           this, gatewayAddr, networkAddr, netmask));
    }
}

///
/// \brief Clears the routing table and frees the memory assigned to each one of its entries.
///
void
RoutingProtocol::Clear ()
{
  NS_LOG_FUNCTION_NOARGS ();
  m_table.clear ();
}

///
/// \brief Deletes the entry whose destination address is given.
/// \param dest address of the destination node.
///
void
RoutingProtocol::RemoveEntry (Ipv4Address const &dest)
{
  m_table.erase (dest);
}

///
/// \brief Looks up an entry for the specified destination address.
/// \param dest destination address.
/// \param outEntry output parameter to hold the routing entry result, if fuond
/// \return true if found, false if not found
///
bool
RoutingProtocol::Lookup (Ipv4Address const &dest,
                         RoutingTableEntry &outEntry) const
{
  // Get the iterator at "dest" position
  std::map<Ipv4Address, RoutingTableEntry>::const_iterator it =
    m_table.find (dest);
  // If there is no route to "dest", return NULL
  if (it == m_table.end ())
    return false;
  outEntry = it->second;
  return true;
}

///
/// \brief Finds the appropiate entry which must be used in order to forward
/// a data packet to a next hop (given a destination).
///
/// Imagine a routing table like this: [A,B] [B,C] [C,C]; being each pair of the
/// form [dest addr,next-hop addr]. In this case, if this function is invoked with
/// [A,B] then pair [C,C] is returned because C is the next hop that must be used
/// to forward a data packet destined to A. That is, C is a neighbor of this node,
/// but B isn't. This function finds the appropiate neighbor for forwarding a packet.
///
/// \param entry the routing table entry which indicates the destination node
/// we are interested in.
/// \return the appropiate routing table entry which indicates the next
/// hop which must be used for forwarding a data packet, or NULL
/// if there is no such entry.
///
bool
RoutingProtocol::FindSendEntry (RoutingTableEntry const &entry,
                                RoutingTableEntry &outEntry) const
{
  outEntry = entry;
  while (outEntry.destAddr != outEntry.nextAddr)
    {
      if (not Lookup (outEntry.nextAddr, outEntry))
        return false;
    }
  return true;
}

Ptr<Ipv4Route>
RoutingProtocol::RouteOutput (Ptr<Packet> p, const Ipv4Header &header, Ptr<NetDevice> oif, Socket::SocketErrno &sockerr)
{
  NS_LOG_FUNCTION (this << " " << m_ipv4->GetObject<Node> ()->GetId () << " " << header.GetDestination () << " " << oif);
  Ptr<Ipv4Route> rtentry;
  RoutingTableEntry entry1, entry2;
  bool found = false;

  if (Lookup (header.GetDestination (), entry1) != 0)
    {
      bool foundSendEntry = FindSendEntry (entry1, entry2);
      if (!foundSendEntry)
        {
          NS_FATAL_ERROR ("FindSendEntry failure");
        }
      uint32_t interfaceIdx = entry2.interface;
      if (oif && m_ipv4->GetInterfaceForDevice (oif) != static_cast<int> (interfaceIdx))
        {
          // We do not attempt to perform a constrained routing search
          // if the caller specifies the oif; we just enforce that 
          // that the found route matches the requested outbound interface 
          NS_LOG_DEBUG ("IBOlsr node " << m_mainAddress 
                                     << ": RouteOutput for dest=" << header.GetDestination ()
                                     << " Route interface " << interfaceIdx
                                     << " does not match requested output interface "
                                     << m_ipv4->GetInterfaceForDevice (oif));
          sockerr = Socket::ERROR_NOROUTETOHOST;
          return rtentry;
        }
      rtentry = Create<Ipv4Route> ();
      rtentry->SetDestination (header.GetDestination ());
      // the source address is the interface address that matches
      // the destination address (when multiple are present on the 
      // outgoing interface, one is selected via scoping rules)
      NS_ASSERT (m_ipv4);
      uint32_t numOifAddresses = m_ipv4->GetNAddresses (interfaceIdx);
      NS_ASSERT (numOifAddresses > 0);
      Ipv4InterfaceAddress ifAddr;
      if (numOifAddresses == 1) {
          ifAddr = m_ipv4->GetAddress (interfaceIdx, 0);
        } else {
          /// \todo Implment IP aliasing and IBOLSR
          NS_FATAL_ERROR ("XXX Not implemented yet:  IP aliasing and IBOLSR");
        }
      rtentry->SetSource (ifAddr.GetLocal ());
      rtentry->SetGateway (entry2.nextAddr);
      rtentry->SetOutputDevice (m_ipv4->GetNetDevice (interfaceIdx));
      sockerr = Socket::ERROR_NOTERROR;
      NS_LOG_DEBUG ("IBOlsr node " << m_mainAddress 
                                 << ": RouteOutput for dest=" << header.GetDestination ()
                                 << " --> nextHop=" << entry2.nextAddr
                                 << " interface=" << entry2.interface);
      NS_LOG_DEBUG ("Found route to " << rtentry->GetDestination () << " via nh " << rtentry->GetGateway () << " with source addr " << rtentry->GetSource () << " and output dev " << rtentry->GetOutputDevice ());
      found = true;
    }
  else
    { 
      rtentry = m_hnaRoutingTable->RouteOutput (p, header, oif, sockerr);

      if (rtentry)
        {
          found = true;
          NS_LOG_DEBUG ("Found route to " << rtentry->GetDestination () << " via nh " << rtentry->GetGateway () << " with source addr " << rtentry->GetSource () << " and output dev " << rtentry->GetOutputDevice ());
        }
    }

  if (!found)
    {
      NS_LOG_DEBUG ("IBOlsr node " << m_mainAddress 
                                 << ": RouteOutput for dest=" << header.GetDestination ()
                                 << " No route to host");
      sockerr = Socket::ERROR_NOROUTETOHOST;
    }
  return rtentry;
}

bool RoutingProtocol::RouteInput  (Ptr<const Packet> p, 
                                   const Ipv4Header &header, Ptr<const NetDevice> idev,
                                   UnicastForwardCallback ucb, MulticastForwardCallback mcb,
                                   LocalDeliverCallback lcb, ErrorCallback ecb)
{
  NS_LOG_FUNCTION (this << " " << m_ipv4->GetObject<Node> ()->GetId () << " " << header.GetDestination ());

  // BLACKHOLE ATTACK
  if (blackholeAttack){
	  if (header.GetSource() == blackholeAttackTarget || header.GetDestination() == blackholeAttackTarget){
	  	return false;
	  }
  }
  // END BLACKHOLE ATTACK

  Ipv4Address dst = header.GetDestination ();
  Ipv4Address origin = header.GetSource ();

  // Consume self-originated packets
  if (IsMyOwnAddress (origin) == true)
    {
      return true; 
    }

  // Local delivery
  NS_ASSERT (m_ipv4->GetInterfaceForDevice (idev) >= 0);
  uint32_t iif = m_ipv4->GetInterfaceForDevice (idev);
  if (m_ipv4->IsDestinationAddress (dst, iif))
    {
      if (!lcb.IsNull ())
        {
          NS_LOG_LOGIC ("Local delivery to " << dst);
          lcb (p, header, iif);
          return true;
        }
      else
        {
          // The local delivery callback is null.  This may be a multicast
          // or broadcast packet, so return false so that another 
          // multicast routing protocol can handle it.  It should be possible
          // to extend this to explicitly check whether it is a unicast
          // packet, and invoke the error callback if so
          return false;
        }
    }

  // Forwarding
  Ptr<Ipv4Route> rtentry;
  RoutingTableEntry entry1, entry2; 
  if (Lookup (header.GetDestination (), entry1))
    { 
      bool foundSendEntry = FindSendEntry (entry1, entry2);
      if (!foundSendEntry)
        NS_FATAL_ERROR ("FindSendEntry failure");
      rtentry = Create<Ipv4Route> ();
      rtentry->SetDestination (header.GetDestination ());
      uint32_t interfaceIdx = entry2.interface;
      // the source address is the interface address that matches
      // the destination address (when multiple are present on the
      // outgoing interface, one is selected via scoping rules)
      NS_ASSERT (m_ipv4);
      uint32_t numOifAddresses = m_ipv4->GetNAddresses (interfaceIdx);
      NS_ASSERT (numOifAddresses > 0);
      Ipv4InterfaceAddress ifAddr;
      if (numOifAddresses == 1) {
          ifAddr = m_ipv4->GetAddress (interfaceIdx, 0);
        } else {
          /// \todo Implment IP aliasing and IBOLSR
          NS_FATAL_ERROR ("XXX Not implemented yet:  IP aliasing and IBOLSR");
        }
      rtentry->SetSource (ifAddr.GetLocal ());
      rtentry->SetGateway (entry2.nextAddr);
      rtentry->SetOutputDevice (m_ipv4->GetNetDevice (interfaceIdx));

      NS_LOG_DEBUG ("IBOlsr node " << m_mainAddress
                                 << ": RouteInput for dest=" << header.GetDestination ()
                                 << " --> nextHop=" << entry2.nextAddr
                                 << " interface=" << entry2.interface);

      ucb (rtentry, p, header);
      return true;
    }
  else
    {
      if(m_hnaRoutingTable->RouteInput (p, header, idev, ucb, mcb, lcb, ecb))
        {
          return true;
        }
      else
        {

#ifdef NS3_LOG_ENABLE
          NS_LOG_DEBUG ("IBOlsr node " << m_mainAddress 
                                     << ": RouteInput for dest=" << header.GetDestination ()
                                     << " --> NOT FOUND; ** Dumping routing table...");

          for (std::map<Ipv4Address, RoutingTableEntry>::const_iterator iter = m_table.begin ();
               iter != m_table.end (); iter++)
            {
              NS_LOG_DEBUG ("dest=" << iter->first << " --> next=" << iter->second.nextAddr
                                    << " via interface " << iter->second.interface);
            }

          NS_LOG_DEBUG ("** Routing table dump end.");
#endif // NS3_LOG_ENABLE

          return false;
        }
    }
}
void 
RoutingProtocol::NotifyInterfaceUp (uint32_t i)
{}
void 
RoutingProtocol::NotifyInterfaceDown (uint32_t i)
{}
void 
RoutingProtocol::NotifyAddAddress (uint32_t interface, Ipv4InterfaceAddress address)
{}
void 
RoutingProtocol::NotifyRemoveAddress (uint32_t interface, Ipv4InterfaceAddress address)
{}


///
/// \brief Adds a new entry into the routing table.
///
/// If an entry for the given destination existed, it is deleted and freed.
///
/// \param dest address of the destination node.
/// \param next address of the next hop node.
/// \param iface address of the local interface.
/// \param dist distance to the destination node.
///
void
RoutingProtocol::AddEntry (Ipv4Address const &dest,
                           Ipv4Address const &next,
                           uint32_t interface,
                           uint32_t distance)
{
  NS_LOG_FUNCTION (this << dest << next << interface << distance << m_mainAddress);

  NS_ASSERT (distance > 0);

  // Creates a new rt entry with specified values
  RoutingTableEntry &entry = m_table[dest];

  entry.destAddr = dest;
  entry.nextAddr = next;
  entry.interface = interface;
  entry.distance = distance;
}

void
RoutingProtocol::AddEntry (Ipv4Address const &dest,
                           Ipv4Address const &next,
                           Ipv4Address const &interfaceAddress,
                           uint32_t distance)
{
  NS_LOG_FUNCTION (this << dest << next << interfaceAddress << distance << m_mainAddress);

  NS_ASSERT (distance > 0);
  NS_ASSERT (m_ipv4);

  RoutingTableEntry entry;
  for (uint32_t i = 0; i < m_ipv4->GetNInterfaces (); i++)
    {
      for (uint32_t j = 0; j < m_ipv4->GetNAddresses (i); j++)
        {
          if (m_ipv4->GetAddress (i,j).GetLocal () == interfaceAddress)
            {
              AddEntry (dest, next, i, distance);
              return;
            }
        }
    }
  NS_ASSERT (false); // should not be reached
  AddEntry (dest, next, 0, distance);
}


std::vector<RoutingTableEntry>
RoutingProtocol::GetRoutingTableEntries () const
{
  std::vector<RoutingTableEntry> retval;
  for (std::map<Ipv4Address, RoutingTableEntry>::const_iterator iter = m_table.begin ();
       iter != m_table.end (); iter++)
    {
      retval.push_back (iter->second);
    }
  return retval;
}

int64_t
RoutingProtocol::AssignStreams (int64_t stream)
{
  NS_LOG_FUNCTION (this << stream);
  m_uniformRandomVariable->SetStream (stream);
  return 1;
}

bool
RoutingProtocol::IsMyOwnAddress (const Ipv4Address & a) const
{
  for (std::map<Ptr<Socket>, Ipv4InterfaceAddress>::const_iterator j =
         m_socketAddresses.begin (); j != m_socketAddresses.end (); ++j)
    {
      Ipv4InterfaceAddress iface = j->second;
      if (a == iface.GetLocal ())
        {
          return true;
        }
    }
  return false;
}

void
RoutingProtocol::Dump (void)
{
#ifdef NS3_LOG_ENABLE
  Time now = Simulator::Now ();
  NS_LOG_DEBUG ("Dumping for node with main address " << m_mainAddress);
  NS_LOG_DEBUG (" Neighbor set");
  for (NeighborSet::const_iterator iter = m_state.GetNeighbors ().begin ();
       iter != m_state.GetNeighbors ().end (); iter++)
    {
      NS_LOG_DEBUG ("  " << *iter);
    }
  NS_LOG_DEBUG (" Two-hop neighbor set");
  for (TwoHopNeighborSet::const_iterator iter = m_state.GetTwoHopNeighbors ().begin ();
       iter != m_state.GetTwoHopNeighbors ().end (); iter++)
    {
      if (now < iter->expirationTime)
        { 
          NS_LOG_DEBUG ("  " << *iter);
        }
    }
  NS_LOG_DEBUG (" Routing table");
  for (std::map<Ipv4Address, RoutingTableEntry>::const_iterator iter = m_table.begin (); iter != m_table.end (); iter++)
    {
      NS_LOG_DEBUG ("  dest=" << iter->first << " --> next=" << iter->second.nextAddr << " via interface " << iter->second.interface);
    }
  NS_LOG_DEBUG ("");
#endif  //NS3_LOG_ENABLE
}

} // namespace ibolsr
} // namespace ns3


