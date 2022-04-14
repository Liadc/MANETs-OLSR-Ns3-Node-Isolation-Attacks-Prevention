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
/// \file	ibolsr-state.cc
/// \brief	Implementation of all functions needed for manipulating the internal
///		state of an IBOLSR node.
///

#include "ibolsr-state.h"


namespace ns3 {


/********** MPR Selector Set Manipulation **********/

MprSelectorTuple*
IBOlsrState::FindMprSelectorTuple (Ipv4Address const &mainAddr)
{
  for (MprSelectorSet::iterator it = m_mprSelectorSet.begin ();
       it != m_mprSelectorSet.end (); it++)
    {
      if (it->mainAddr == mainAddr)
        return &(*it);
    }
  return NULL;
}

void
IBOlsrState::EraseMprSelectorTuple (const MprSelectorTuple &tuple)
{
  for (MprSelectorSet::iterator it = m_mprSelectorSet.begin ();
       it != m_mprSelectorSet.end (); it++)
    {
      if (*it == tuple)
        {
          m_mprSelectorSet.erase (it);
          break;
        }
    }
}

void
IBOlsrState::EraseMprSelectorTuples (const Ipv4Address &mainAddr)
{
  for (MprSelectorSet::iterator it = m_mprSelectorSet.begin ();
       it != m_mprSelectorSet.end ();)
    {
      if (it->mainAddr == mainAddr)
        {
          it = m_mprSelectorSet.erase (it);
        }
      else
        {
          it++;
        }
    }
}

void
IBOlsrState::InsertMprSelectorTuple (MprSelectorTuple const &tuple)
{
  m_mprSelectorSet.push_back (tuple);
}

std::string
IBOlsrState::PrintMprSelectorSet () const
{
  std::ostringstream os;
  os << "[";
  for (MprSelectorSet::const_iterator iter = m_mprSelectorSet.begin ();
       iter != m_mprSelectorSet.end (); iter++)
    {
      MprSelectorSet::const_iterator next = iter;
      next++;
      os << iter->mainAddr;
      if (next != m_mprSelectorSet.end ())
        os << ", ";
    }
  os << "]";
  return os.str ();
}


/********** Neighbor Set Manipulation **********/

NeighborTuple*
IBOlsrState::FindNeighborTuple (Ipv4Address const &mainAddr)
{
  for (NeighborSet::iterator it = m_neighborSet.begin ();
       it != m_neighborSet.end (); it++)
    {
      if (it->neighborMainAddr == mainAddr)
        return &(*it);
    }
  return NULL;
}

const NeighborTuple*
IBOlsrState::FindSymNeighborTuple (Ipv4Address const &mainAddr) const
{
  for (NeighborSet::const_iterator it = m_neighborSet.begin ();
       it != m_neighborSet.end (); it++)
    {
      if (it->neighborMainAddr == mainAddr && it->status == NeighborTuple::STATUS_SYM)
        return &(*it);
    }
  return NULL;
}

NeighborTuple*
IBOlsrState::FindNeighborTuple (Ipv4Address const &mainAddr, uint8_t willingness)
{
  for (NeighborSet::iterator it = m_neighborSet.begin ();
       it != m_neighborSet.end (); it++)
    {
      if (it->neighborMainAddr == mainAddr && it->willingness == willingness)
        return &(*it);
    }
  return NULL;
}

void
IBOlsrState::EraseNeighborTuple (const NeighborTuple &tuple)
{
  for (NeighborSet::iterator it = m_neighborSet.begin ();
       it != m_neighborSet.end (); it++)
    {
      if (*it == tuple)
        {
          m_neighborSet.erase (it);
          break;
        }
    }
}

void
IBOlsrState::EraseNeighborTuple (const Ipv4Address &mainAddr)
{
  for (NeighborSet::iterator it = m_neighborSet.begin ();
       it != m_neighborSet.end (); it++)
    {
      if (it->neighborMainAddr == mainAddr)
        {
          it = m_neighborSet.erase (it);
          break;
        }
    }
}

void
IBOlsrState::InsertNeighborTuple (NeighborTuple const &tuple)
{
  for (NeighborSet::iterator it = m_neighborSet.begin ();
       it != m_neighborSet.end (); it++)
    {
      if (it->neighborMainAddr == tuple.neighborMainAddr)
        {
          // Update it
          *it = tuple;
          return;
        }
    }
  m_neighborSet.push_back (tuple);
}

/********** Neighbor 2 Hop Set Manipulation **********/

TwoHopNeighborTuple*
IBOlsrState::FindTwoHopNeighborTuple (Ipv4Address const &neighborMainAddr,
                                    Ipv4Address const &twoHopNeighborAddr)
{
  for (TwoHopNeighborSet::iterator it = m_twoHopNeighborSet.begin ();
       it != m_twoHopNeighborSet.end (); it++)
    {
      if (it->neighborMainAddr == neighborMainAddr
          && it->twoHopNeighborAddr == twoHopNeighborAddr)
        {
          return &(*it);
        }
    }
  return NULL;
}

void
IBOlsrState::EraseTwoHopNeighborTuple (const TwoHopNeighborTuple &tuple)
{
  for (TwoHopNeighborSet::iterator it = m_twoHopNeighborSet.begin ();
       it != m_twoHopNeighborSet.end (); it++)
    {
      if (*it == tuple)
        {
          m_twoHopNeighborSet.erase (it);
          break;
        }
    }
}

void
IBOlsrState::EraseTwoHopNeighborTuples (const Ipv4Address &neighborMainAddr,
                                      const Ipv4Address &twoHopNeighborAddr)
{
  for (TwoHopNeighborSet::iterator it = m_twoHopNeighborSet.begin ();
       it != m_twoHopNeighborSet.end ();)
    {
      if (it->neighborMainAddr == neighborMainAddr
          && it->twoHopNeighborAddr == twoHopNeighborAddr)
        {
          it = m_twoHopNeighborSet.erase (it);
        }
      else
        {
          it++;
        }
    }
}

void
IBOlsrState::EraseTwoHopNeighborTuples (const Ipv4Address &neighborMainAddr)
{
  for (TwoHopNeighborSet::iterator it = m_twoHopNeighborSet.begin ();
       it != m_twoHopNeighborSet.end ();)
    {
      if (it->neighborMainAddr == neighborMainAddr)
        {
          it = m_twoHopNeighborSet.erase (it);
        }
      else
        {
          it++;
        }
    }
}

void
IBOlsrState::InsertTwoHopNeighborTuple (TwoHopNeighborTuple const &tuple)
{
  m_twoHopNeighborSet.push_back (tuple);
}

/********** MPR Set Manipulation **********/

bool
IBOlsrState::FindMprAddress (Ipv4Address const &addr)
{
  MprSet::iterator it = m_mprSet.find (addr);
  return (it != m_mprSet.end ());
}

void
IBOlsrState::SetMprSet (MprSet mprSet)
{
  m_mprSet = mprSet;
}
MprSet
IBOlsrState::GetMprSet () const
{
  return m_mprSet;
}

/********** Duplicate Set Manipulation **********/

DuplicateTuple*
IBOlsrState::FindDuplicateTuple (Ipv4Address const &addr, uint16_t sequenceNumber)
{
  for (DuplicateSet::iterator it = m_duplicateSet.begin ();
       it != m_duplicateSet.end (); it++)
    {
      if (it->address == addr && it->sequenceNumber == sequenceNumber)
        return &(*it);
    }
  return NULL;
}

void
IBOlsrState::EraseDuplicateTuple (const DuplicateTuple &tuple)
{
  for (DuplicateSet::iterator it = m_duplicateSet.begin ();
       it != m_duplicateSet.end (); it++)
    {
      if (*it == tuple)
        {
          m_duplicateSet.erase (it);
          break;
        }
    }
}

void
IBOlsrState::InsertDuplicateTuple (DuplicateTuple const &tuple)
{
  m_duplicateSet.push_back (tuple);
}

/********** Link Set Manipulation **********/

LinkTuple*
IBOlsrState::FindLinkTuple (Ipv4Address const & ifaceAddr)
{
  for (LinkSet::iterator it = m_linkSet.begin ();
       it != m_linkSet.end (); it++)
    {
      if (it->neighborIfaceAddr == ifaceAddr)
        return &(*it);
    }
  return NULL;
}

LinkTuple*
IBOlsrState::FindSymLinkTuple (Ipv4Address const &ifaceAddr, Time now)
{
  for (LinkSet::iterator it = m_linkSet.begin ();
       it != m_linkSet.end (); it++)
    {
      if (it->neighborIfaceAddr == ifaceAddr)
        {
          if (it->symTime > now)
            return &(*it);
          else
            break;
        }
    }
  return NULL;
}

void
IBOlsrState::EraseLinkTuple (const LinkTuple &tuple)
{
  for (LinkSet::iterator it = m_linkSet.begin ();
       it != m_linkSet.end (); it++)
    {
      if (*it == tuple)
        {
          m_linkSet.erase (it);
          break;
        }
    }
}

LinkTuple&
IBOlsrState::InsertLinkTuple (LinkTuple const &tuple)
{
  m_linkSet.push_back (tuple);
  return m_linkSet.back ();
}

/********** Topology Set Manipulation **********/

TopologyTuple*
IBOlsrState::FindTopologyTuple (Ipv4Address const &destAddr,
                              Ipv4Address const &lastAddr)
{
  for (TopologySet::iterator it = m_topologySet.begin ();
       it != m_topologySet.end (); it++)
    {
      if (it->destAddr == destAddr && it->lastAddr == lastAddr)
        return &(*it);
    }
  return NULL;
}

TopologyTuple*
IBOlsrState::FindNewerTopologyTuple (Ipv4Address const & lastAddr, uint16_t ansn)
{
  for (TopologySet::iterator it = m_topologySet.begin ();
       it != m_topologySet.end (); it++)
    {
      if (it->lastAddr == lastAddr && it->sequenceNumber > ansn)
        return &(*it);
    }
  return NULL;
}

void
IBOlsrState::EraseTopologyTuple (const TopologyTuple &tuple)
{
  for (TopologySet::iterator it = m_topologySet.begin ();
       it != m_topologySet.end (); it++)
    {
      if (*it == tuple)
        {
          m_topologySet.erase (it);
          break;
        }
    }
}

void
IBOlsrState::EraseOlderTopologyTuples (const Ipv4Address &lastAddr, uint16_t ansn)
{
  for (TopologySet::iterator it = m_topologySet.begin ();
       it != m_topologySet.end ();)
    {
      if (it->lastAddr == lastAddr && it->sequenceNumber < ansn)
        {
          it = m_topologySet.erase (it);
        }
      else
        {
          it++;
        }
    }
}

void
IBOlsrState::InsertTopologyTuple (TopologyTuple const &tuple)
{
  m_topologySet.push_back (tuple);
}

/********** Interface Association Set Manipulation **********/

IfaceAssocTuple*
IBOlsrState::FindIfaceAssocTuple (Ipv4Address const &ifaceAddr)
{
  for (IfaceAssocSet::iterator it = m_ifaceAssocSet.begin ();
       it != m_ifaceAssocSet.end (); it++)
    {
      if (it->ifaceAddr == ifaceAddr)
        return &(*it);
    }
  return NULL;
}

const IfaceAssocTuple*
IBOlsrState::FindIfaceAssocTuple (Ipv4Address const &ifaceAddr) const
{
  for (IfaceAssocSet::const_iterator it = m_ifaceAssocSet.begin ();
       it != m_ifaceAssocSet.end (); it++)
    {
      if (it->ifaceAddr == ifaceAddr)
        return &(*it);
    }
  return NULL;
}

void
IBOlsrState::EraseIfaceAssocTuple (const IfaceAssocTuple &tuple)
{
  for (IfaceAssocSet::iterator it = m_ifaceAssocSet.begin ();
       it != m_ifaceAssocSet.end (); it++)
    {
      if (*it == tuple)
        {
          m_ifaceAssocSet.erase (it);
          break;
        }
    }
}

void
IBOlsrState::InsertIfaceAssocTuple (const IfaceAssocTuple &tuple)
{
  m_ifaceAssocSet.push_back (tuple);
}

std::vector<Ipv4Address>
IBOlsrState::FindNeighborInterfaces (const Ipv4Address &neighborMainAddr) const
{
  std::vector<Ipv4Address> retval;
  for (IfaceAssocSet::const_iterator it = m_ifaceAssocSet.begin ();
       it != m_ifaceAssocSet.end (); it++)
    {
      if (it->mainAddr == neighborMainAddr)
        retval.push_back (it->ifaceAddr);
    }
  return retval;
}

/********** Host-Network Association Set Manipulation **********/

AssociationTuple*
IBOlsrState::FindAssociationTuple (const Ipv4Address &gatewayAddr, const Ipv4Address &networkAddr, const Ipv4Mask &netmask)
{
  for (AssociationSet::iterator it = m_associationSet.begin ();
       it != m_associationSet.end (); it++)
    {
      if (it->gatewayAddr == gatewayAddr and it->networkAddr == networkAddr and it->netmask == netmask)
        {
          return &(*it);
        }
    }
  return NULL;
}

void
IBOlsrState::EraseAssociationTuple (const AssociationTuple &tuple)
{
  for (AssociationSet::iterator it = m_associationSet.begin ();
       it != m_associationSet.end (); it++)
    {
      if (*it == tuple)
        {
          m_associationSet.erase (it);
          break;
        }
    }
}

void
IBOlsrState::InsertAssociationTuple (const AssociationTuple &tuple)
{
  m_associationSet.push_back (tuple);
}

void
IBOlsrState::EraseAssociation (const Association &tuple)
{
  for (Associations::iterator it = m_associations.begin ();
       it != m_associations.end (); it++)
    {
      if (*it == tuple)
        {
          m_associations.erase (it);
          break;
        }
    }
}

void
IBOlsrState::InsertAssociation (const Association &tuple)
{
  m_associations.push_back (tuple);
}

// Added functions (some of 'em)

/* This is OBOLUTE. The function was moved to protocol.
// Returns whenever the node would require a fake node
bool IBOlsrState::RequireFake(){
	// v1
	for (NeighborSet::iterator it = m_neighborSet.begin(); it != m_neighborSet.end(); ++it){
		bool failed = false;
		std::list<Ipv4Address> twohops;
		// Check if that neighbor is a 2hop of a neighbor (and generate his 2hops list)
		for (TwoHopNeighborSet::iterator it2 = m_twoHopNeighborSet.begin(); it2 != m_twoHopNeighborSet.end(); ++it2){
			if (it->neighborMainAddr == it2->twoHopNeighborAddr){
				failed = true;
				break;
			}
			if (it->neighborMainAddr == it2->neighborMainAddr){
				twohops.push_back(it2->neighborMainAddr);
			}
		}
		if (failed) continue;

		// Check if there is a 2hop without any route other than this neighbor
		for (TwoHopNeighborSet::iterator it2 = m_twoHopNeighborSet.begin(); it2 != m_twoHopNeighborSet.end(); ++it2){
			if (it->neighborMainAddr == it2->neighborMainAddr) {
				continue;
			} else {
				twohops.remove(it2->twoHopNeighborAddr);
			}
		}

		// Check if there's anything left
		if (!twohops.empty()){
			return false;
		}

	}
	// Couldn't find a "fake" node
	return true;


	// v2
	
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

	// v3
	
	for (TopologySet::iterator ita3 = m_topologySet.begin(); ita3 != m_topologySet.end(); ++ita3){
		for (NeighborSet::iterator ita1 = m_neighborSet.begin(); ita1 != m_neighborSet.end(); ++ita1){
			if (ita3->destAddr == ita1->neighborMainAddr){
				goto FindNextAPath;
			}
		}
		for (TwoHopNeighborSet::iterator ita2 = m_twoHopNeighborSet.begin(); ita2 != m_twoHopNeighborSet.end(); ++ita2){
			if (ita3->lastAddr == ita2->twoHopNeighborAddr){
				for (TopologySet::iterator itb3 = m_topologySet.begin(); itb3 != m_topologySet.end(); ++itb3){
					if ((ita3->lastAddr != itb3->lastAddr) && (ita3->destAddr != itb3->destAddr)) {
						for (NeighborSet::iterator itb1 = m_neighborSet.begin(); itb1 != m_neighborSet.end(); ++itb1){
							if (itb3->destAddr == itb1->neighborMainAddr){
								goto FindNextBPath;
							}
						}
						for (TwoHopNeighborSet::iterator itb2 = m_twoHopNeighborSet.begin(); itb2 != m_twoHopNeighborSet.end(); ++itb2){
							if ((itb3->lastAddr == itb2->twoHopNeighborAddr) && (ita2->neighborMainAddr != itb2->neighborMainAddr)){
								for (TopologySet::iterator itc3 = m_topologySet.begin(); itc3 != m_topologySet.end(); ++itc3){
									if ((ita3->lastAddr != itc3->lastAddr) && (ita3->destAddr != itc3->destAddr) && (itb3->lastAddr != itc3->lastAddr) && (itb3->destAddr != itc3->destAddr)) {
										for (NeighborSet::iterator itc1 = m_neighborSet.begin(); itc1 != m_neighborSet.end(); ++itc1){
											if (itc3->destAddr == itc1->neighborMainAddr){
												goto FindNextCPath;
											}
										}
										for (TwoHopNeighborSet::iterator itc2 = m_twoHopNeighborSet.begin(); itc2 != m_twoHopNeighborSet.end(); ++itc2){
											if ((itc3->lastAddr == itc2->twoHopNeighborAddr) && (ita2->neighborMainAddr != itc2->neighborMainAddr) && (itb2->neighborMainAddr != itc2->neighborMainAddr)){
												return false;
											}
										}
									}
									FindNextCPath:;
								}
							}
						}
					}
					FindNextBPath:;
				}
			}
		}
		FindNextAPath:;
	}
				


	// Couldn't find a "fake" node (or path...)
	return true;
}
*/

/*
double IBOlsrState::FractionOfMpr(){
		return m_mprSet.size() / (double) m_neighborSet.size();
}
*/

} // namespace ns3
