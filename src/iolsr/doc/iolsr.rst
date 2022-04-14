.. include:: replace.txt

Optimized Link State Routing (IOLSR)
-----------------------------------

This model implements the base specification of the Optimized
Link State Routing (IOLSR) protocol, which is a dynamic mobile ad hoc
unicast routing protocol.  It has been developed at the
University of Murcia (Spain) by Francisco J. Ros for NS-2, and was
ported to NS-3 by Gustavo Carneiro at INESC Porto (Portugal).

Model Description
*****************

The source code for the IOLSR model lives in the directory `src/iolsr`.

Design
++++++

Scope and Limitations
+++++++++++++++++++++

The model is for IPv4 only.  

* Mostly compliant with IOLSR as documented in [rfc3626]_, 
* The use of multiple interfaces was not supported by the NS-2 version, but is supported in NS-3;
* IOLSR does not respond to the routing event notifications corresponding to dynamic interface up and down (``ns3::RoutingProtocol::NotifyInterfaceUp`` and ``ns3::RoutingProtocol::NotifyInterfaceDown``) or address insertion/removal ``ns3::RoutingProtocol::NotifyAddAddress`` and ``ns3::RoutingProtocol::NotifyRemoveAddress``).
* Unlike the NS-2 version, does not yet support MAC layer feedback as described in RFC 3626;

Host Network Association (HNA) is supported in this implementation
of IOLSR. Refer to ``examples/iolsr-hna.cc`` to see how the API
is used.

References
++++++++++

.. [rfc3626] RFC 3626 *Optimized Link State Routing*

Usage
*****

Examples
++++++++

Helpers
+++++++

A helper class for IOLSR has been written.  After an IPv4 topology
has been created and unique IP addresses assigned to each node, the
simulation script writer can call one of three overloaded functions
with different scope to enable IOLSR: ``ns3::IOlsrHelper::Install
(NodeContainer container)``; ``ns3::IOlsrHelper::Install (Ptr<Node>
node)``; or ``ns3::IOlsrHelper::InstallAll (void)``

Attributes
++++++++++

In addition, the behavior of IOLSR can be modified by changing certain
attributes.  The method ``ns3::IOlsrHelper::Set ()`` can be used
to set IOLSR attributes.  These include HelloInterval, TcInterval,
MidInterval, Willingness.  Other parameters are defined as macros
in ``iolsr-routing-protocol.cc``.

Tracing
+++++++

Logging
+++++++

Caveats
+++++++

Validation
**********

Unit tests
++++++++++

Larger-scale performance tests
++++++++++++++++++++++++++++++

