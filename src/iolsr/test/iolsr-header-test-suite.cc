/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
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
 * Author: Gustavo J. A. M. Carneiro  <gjc@inescporto.pt>
 */

#include "ns3/test.h"
#include "ns3/iolsr-header.h"
#include "ns3/packet.h"

namespace ns3 {

class IOlsrEmfTestCase : public TestCase {
public:
  IOlsrEmfTestCase ();
  virtual void DoRun (void);
};

IOlsrEmfTestCase::IOlsrEmfTestCase ()
  : TestCase ("Check Emf iolsr time conversion")
{
}
void
IOlsrEmfTestCase::DoRun (void)
{
  for (int time = 1; time <= 30; time++)
    {
      uint8_t emf = iolsr::SecondsToEmf (time);
      double seconds = iolsr::EmfToSeconds (emf);
      NS_TEST_ASSERT_MSG_EQ ((seconds < 0 || std::fabs (seconds - time) > 0.1), false,
                             "100");
    }
}


class IOlsrMidTestCase : public TestCase {
public:
  IOlsrMidTestCase ();
  virtual void DoRun (void);
};

IOlsrMidTestCase::IOlsrMidTestCase ()
  : TestCase ("Check Mid iolsr messages")
{
}
void
IOlsrMidTestCase::DoRun (void)
{
  Packet packet;

  {
    iolsr::PacketHeader hdr;
    iolsr::MessageHeader msg1;
    iolsr::MessageHeader::Mid &mid1 = msg1.GetMid ();
    iolsr::MessageHeader msg2;
    iolsr::MessageHeader::Mid &mid2 = msg2.GetMid ();

    // MID message #1
    {
      std::vector<Ipv4Address> &addresses = mid1.interfaceAddresses;
      addresses.clear ();
      addresses.push_back (Ipv4Address ("1.2.3.4"));
      addresses.push_back (Ipv4Address ("1.2.3.5"));
    }

    msg1.SetTimeToLive (255);
    msg1.SetOriginatorAddress (Ipv4Address ("11.22.33.44"));
    msg1.SetVTime (Seconds (9));
    msg1.SetMessageSequenceNumber (7);

    // MID message #2
    {
      std::vector<Ipv4Address> &addresses = mid2.interfaceAddresses;
      addresses.clear ();
      addresses.push_back (Ipv4Address ("2.2.3.4"));
      addresses.push_back (Ipv4Address ("2.2.3.5"));
    }

    msg2.SetTimeToLive (254);
    msg2.SetOriginatorAddress (Ipv4Address ("12.22.33.44"));
    msg2.SetVTime (Seconds (10));
    msg2.SetMessageType (iolsr::MessageHeader::MID_MESSAGE);
    msg2.SetMessageSequenceNumber (7);

    // Build an IOLSR packet header
    hdr.SetPacketLength (hdr.GetSerializedSize () + msg1.GetSerializedSize () + msg2.GetSerializedSize ());
    hdr.SetPacketSequenceNumber (123);


    // Now add all the headers in the correct order
    packet.AddHeader (msg2);
    packet.AddHeader (msg1);
    packet.AddHeader (hdr);
  }

  {
    iolsr::PacketHeader hdr;
    packet.RemoveHeader (hdr);
    NS_TEST_ASSERT_MSG_EQ (hdr.GetPacketSequenceNumber (), 123, "200");
    uint32_t sizeLeft = hdr.GetPacketLength () - hdr.GetSerializedSize ();
    {
      iolsr::MessageHeader msg1;

      packet.RemoveHeader (msg1);

      NS_TEST_ASSERT_MSG_EQ (msg1.GetTimeToLive (),  255, "201");
      NS_TEST_ASSERT_MSG_EQ (msg1.GetOriginatorAddress (), Ipv4Address ("11.22.33.44"), "202");
      NS_TEST_ASSERT_MSG_EQ (msg1.GetVTime (), Seconds (9), "203");
      NS_TEST_ASSERT_MSG_EQ (msg1.GetMessageType (), iolsr::MessageHeader::MID_MESSAGE, "204");
      NS_TEST_ASSERT_MSG_EQ (msg1.GetMessageSequenceNumber (), 7, "205");

      iolsr::MessageHeader::Mid &mid1 = msg1.GetMid ();
      NS_TEST_ASSERT_MSG_EQ (mid1.interfaceAddresses.size (), 2, "206");
      NS_TEST_ASSERT_MSG_EQ (*mid1.interfaceAddresses.begin (), Ipv4Address ("1.2.3.4"), "207");

      sizeLeft -= msg1.GetSerializedSize ();
      NS_TEST_ASSERT_MSG_EQ ((sizeLeft > 0), true, "208");
    }
    {
      // now read the second message
      iolsr::MessageHeader msg2;

      packet.RemoveHeader (msg2);

      NS_TEST_ASSERT_MSG_EQ (msg2.GetTimeToLive (),  254, "209");
      NS_TEST_ASSERT_MSG_EQ (msg2.GetOriginatorAddress (), Ipv4Address ("12.22.33.44"), "210");
      NS_TEST_ASSERT_MSG_EQ (msg2.GetVTime (), Seconds (10), "211");
      NS_TEST_ASSERT_MSG_EQ (msg2.GetMessageType (), iolsr::MessageHeader::MID_MESSAGE, "212");
      NS_TEST_ASSERT_MSG_EQ (msg2.GetMessageSequenceNumber (), 7, "213");

      iolsr::MessageHeader::Mid mid2 = msg2.GetMid ();
      NS_TEST_ASSERT_MSG_EQ (mid2.interfaceAddresses.size (), 2, "214");
      NS_TEST_ASSERT_MSG_EQ (*mid2.interfaceAddresses.begin (), Ipv4Address ("2.2.3.4"), "215");

      sizeLeft -= msg2.GetSerializedSize ();
      NS_TEST_ASSERT_MSG_EQ (sizeLeft, 0, "216");
    }
  }
}


class IOlsrHelloTestCase : public TestCase {
public:
  IOlsrHelloTestCase ();
  virtual void DoRun (void);
};

IOlsrHelloTestCase::IOlsrHelloTestCase ()
  : TestCase ("Check Hello iolsr messages")
{
}
void
IOlsrHelloTestCase::DoRun (void)
{
  Packet packet;
  iolsr::MessageHeader msgIn;
  iolsr::MessageHeader::Hello &helloIn = msgIn.GetHello ();

  helloIn.SetHTime (Seconds (7));
  helloIn.willingness = 66;

  {
    iolsr::MessageHeader::Hello::LinkMessage lm1;
    lm1.linkCode = 2;
    lm1.neighborInterfaceAddresses.push_back (Ipv4Address ("1.2.3.4"));
    lm1.neighborInterfaceAddresses.push_back (Ipv4Address ("1.2.3.5"));
    helloIn.linkMessages.push_back (lm1);

    iolsr::MessageHeader::Hello::LinkMessage lm2;
    lm2.linkCode = 3;
    lm2.neighborInterfaceAddresses.push_back (Ipv4Address ("2.2.3.4"));
    lm2.neighborInterfaceAddresses.push_back (Ipv4Address ("2.2.3.5"));
    helloIn.linkMessages.push_back (lm2);
  }

  packet.AddHeader (msgIn);

  iolsr::MessageHeader msgOut;
  packet.RemoveHeader (msgOut);
  iolsr::MessageHeader::Hello &helloOut = msgOut.GetHello ();

  NS_TEST_ASSERT_MSG_EQ (helloOut.GetHTime (), Seconds (7), "300");
  NS_TEST_ASSERT_MSG_EQ (helloOut.willingness, 66, "301");
  NS_TEST_ASSERT_MSG_EQ (helloOut.linkMessages.size (), 2, "302");

  NS_TEST_ASSERT_MSG_EQ (helloOut.linkMessages[0].linkCode, 2, "303");
  NS_TEST_ASSERT_MSG_EQ (helloOut.linkMessages[0].neighborInterfaceAddresses[0],
                         Ipv4Address ("1.2.3.4"), "304");
  NS_TEST_ASSERT_MSG_EQ (helloOut.linkMessages[0].neighborInterfaceAddresses[1],
                         Ipv4Address ("1.2.3.5"), "305");

  NS_TEST_ASSERT_MSG_EQ (helloOut.linkMessages[1].linkCode, 3, "306");
  NS_TEST_ASSERT_MSG_EQ (helloOut.linkMessages[1].neighborInterfaceAddresses[0],
                         Ipv4Address ("2.2.3.4"), "307");
  NS_TEST_ASSERT_MSG_EQ (helloOut.linkMessages[1].neighborInterfaceAddresses[1],
                         Ipv4Address ("2.2.3.5"), "308");

  NS_TEST_ASSERT_MSG_EQ (packet.GetSize (), 0, "All bytes in packet were not read");

}

class IOlsrTcTestCase : public TestCase {
public:
  IOlsrTcTestCase ();
  virtual void DoRun (void);
};

IOlsrTcTestCase::IOlsrTcTestCase ()
  : TestCase ("Check Tc iolsr messages")
{
}
void
IOlsrTcTestCase::DoRun (void)
{
  Packet packet;
  iolsr::MessageHeader msgIn;
  iolsr::MessageHeader::Tc &tcIn = msgIn.GetTc ();

  tcIn.ansn = 0x1234;
  tcIn.neighborAddresses.push_back (Ipv4Address ("1.2.3.4"));
  tcIn.neighborAddresses.push_back (Ipv4Address ("1.2.3.5"));
  packet.AddHeader (msgIn);

  iolsr::MessageHeader msgOut;
  packet.RemoveHeader (msgOut);
  iolsr::MessageHeader::Tc &tcOut = msgOut.GetTc ();

  NS_TEST_ASSERT_MSG_EQ (tcOut.ansn, 0x1234, "400");
  NS_TEST_ASSERT_MSG_EQ (tcOut.neighborAddresses.size (), 2, "401");

  NS_TEST_ASSERT_MSG_EQ (tcOut.neighborAddresses[0],
                         Ipv4Address ("1.2.3.4"), "402");
  NS_TEST_ASSERT_MSG_EQ (tcOut.neighborAddresses[1],
                         Ipv4Address ("1.2.3.5"), "403");

  NS_TEST_ASSERT_MSG_EQ (packet.GetSize (), 0, "404");

}

class IOlsrHnaTestCase : public TestCase {
public:
  IOlsrHnaTestCase ();
  virtual void DoRun (void);
};

IOlsrHnaTestCase::IOlsrHnaTestCase ()
  : TestCase ("Check Hna iolsr messages")
{
}

void
IOlsrHnaTestCase::DoRun (void)
{
  Packet packet;
  iolsr::MessageHeader msgIn;
  iolsr::MessageHeader::Hna &hnaIn = msgIn.GetHna ();

  hnaIn.associations.push_back ((iolsr::MessageHeader::Hna::Association)
                                { Ipv4Address ("1.2.3.4"), Ipv4Mask ("255.255.255.0")});
  hnaIn.associations.push_back ((iolsr::MessageHeader::Hna::Association)
                                { Ipv4Address ("1.2.3.5"), Ipv4Mask ("255.255.0.0")});
  packet.AddHeader (msgIn);

  iolsr::MessageHeader msgOut;
  packet.RemoveHeader (msgOut);
  iolsr::MessageHeader::Hna &hnaOut = msgOut.GetHna ();

  NS_TEST_ASSERT_MSG_EQ (hnaOut.associations.size (), 2, "500");

  NS_TEST_ASSERT_MSG_EQ (hnaOut.associations[0].address,
                         Ipv4Address ("1.2.3.4"), "501");
  NS_TEST_ASSERT_MSG_EQ (hnaOut.associations[0].mask,
                         Ipv4Mask ("255.255.255.0"), "502");

  NS_TEST_ASSERT_MSG_EQ (hnaOut.associations[1].address,
                         Ipv4Address ("1.2.3.5"), "503");
  NS_TEST_ASSERT_MSG_EQ (hnaOut.associations[1].mask,
                         Ipv4Mask ("255.255.0.0"), "504");

  NS_TEST_ASSERT_MSG_EQ (packet.GetSize (), 0, "All bytes in packet were not read");

}


static class IOlsrTestSuite : public TestSuite
{
public:
  IOlsrTestSuite ();
} g_iolsrTestSuite;

IOlsrTestSuite::IOlsrTestSuite()
  : TestSuite ("routing-iolsr-header", UNIT)
{
  AddTestCase (new IOlsrHnaTestCase (), TestCase::QUICK);
  AddTestCase (new IOlsrTcTestCase (), TestCase::QUICK);
  AddTestCase (new IOlsrHelloTestCase (), TestCase::QUICK);
  AddTestCase (new IOlsrMidTestCase (), TestCase::QUICK);
  AddTestCase (new IOlsrEmfTestCase (), TestCase::QUICK);
}

} // namespace ns3

