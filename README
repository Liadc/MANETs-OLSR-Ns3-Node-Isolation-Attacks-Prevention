# Are Fictitious Nodes Needed At All?

Table of Contents:
------------------

1) An overview
2) Building ns-3
3) Running ns-3
4) MANETs-OLSR Simulations: How to run, edit simulations, and change simulations' configurations
5) Getting access to the ns-3 documentation
6) Working with the development version of ns-3


Note:  Much more substantial information about ns-3 can be found at
http://www.nsnam.org

1) An Open Source project
-------------------------

ns-3 is a free open source project aiming to build a discrete-event
network simulator targeted for simulation research and education.   
This is a collaborative project; we hope that
the missing pieces of the models we have not yet implemented
will be contributed by the community in an open collaboration
process.

The process of contributing to the ns-3 project varies with
the people involved, the amount of time they can invest
and the type of model they want to work on, but the current
process that the project tries to follow is described here:
http://www.nsnam.org/developers/contributing-code/

This README excerpts some details from a more extensive
tutorial that is maintained at:
http://www.nsnam.org/documentation/latest/

2) Building ns-3
----------------

The code for the framework and the default models provided
by ns-3 is built as a set of libraries. User simulations
are expected to be written as simple programs that make
use of these ns-3 libraries.

To build the set of default libraries and the example
programs included in this package, you need to use the
tool 'waf'. Detailed information on how use waf is 
included in the file doc/build.txt

However, the real quick and dirty way to get started is to
type the command
  ./waf configure --enable-examples
followed by
  ./waf 
in the the directory which contains
this README file. The files built will be copied in the
build/ directory.

The current codebase is expected to build and run on the
set of platforms listed in the RELEASE_NOTES file.

Other platforms may or may not work: we welcome patches to 
improve the portability of the code to these other platforms. 

3) Running ns-3
---------------

On recent Linux systems, once you have built ns-3 (with examples
enabled), it should be easy to run the sample programs with the
following command, such as:

  ./waf --run simple-global-routing

That program should generate a simple-global-routing.tr text 
trace file and a set of simple-global-routing-xx-xx.pcap binary
pcap trace files, which can be read by tcpdump -tt -r filename.pcap
The program source can be found in the examples/routing directory.

4) MANETs-OLSR Simulations: How to run, edit simulations, choose simulation configurations
-------------------------------------------
Once you have verified everything is set up and you can run the above example, you are ready to run the OLSR simulations.

With this project, we included implementation for OLSR and algorithms proposed in our research. 

We implemented the node isolation attack, and several defense mechanisms including DCFM to run on nodes during the simulation in order to mitigate the attack.

Additional implementation for telemtry for nodes during simulation time, and additional logging mechanisms to create and capture enough data to assist, evaluate and compare different approaches to node isolation attacks.

We ran multiple sets of simulations, each set with different configurations: one set with 50 nodes, another with 60 nodes, ... and so on. One with the attack enabled, another with the attack disabled. One with nodes random movement enabled, another without movement at all. And other configurations which are elaborated and presented on the research paper.

Additionaly, each set of simulations will run thousands of times - each time the nodes will be spread differently, the starting point of the nodes in each simulation will be different. If movement is enabled - the randomness of movement will be different in each simulation. 

To allow for consistency between sets of simulations regarding the randomness of movement and starting points - the simulation will be ran using a **SEED**. This way, the simulations with DCFM enabled, DCFM disabled, movement enabled, movement disabled, defence algorithms enabled, disabled and so on.... **WILL ALL SHARE THE SAME** positioning of the nodes, the same movement of nodes - so we can compare apples to apples. 

Additionally, thousands of SEEDs are generated and used to allow for proper calculation of the average benchmarking of each algorithm, including those proposed in the research.

Now that we understand how simulations are properly designed, you are welcome to edit the following files to run those with any configuration you are interested in:

In the following file: ```scratch/iolsr-tests-100.cc```
All configurations will be set in the above file under ```Main``` function for easy edits before runs.


Choose number of nodes: ```nNodes```

Choose map size: ```dMaxGridX``` and ```dMaxGridY```

Choose simulation running time: ```dSimulationSeconds``` and ```nSimulationSeconds```

Enable/disable movement for nodes: **```bMobility```**


And many more for easy manipulation of simulations:

* Enable/disable node isolation attacks,

* Enable/disable DCFM with fictitious nodes,

* Enable/disable proposed defense algorithms 

* and much more, with new telemetry and logs!


A ready example of a set of simulations is included, and can also be used with the following code for easy reference:
```cpp
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


	std::string mProtocolName = "Invalid";
	bool bPrintSimStats = true; //simulation stats, such as random seed used, time.
	bool bSuperTransmission = false; //Transmission boost to node X?
	bool bPrintAll = false; //Print routing table for all hops
	bool bPrintFakeCount = true; //Print amount of fake nodes required
	bool bPrintMprFraction = true; //Print fraction of MPR
	bool bPrintRiskyFraction = true; //Print fraction of risky
	bool bIsolationAttack = false; //Execute isolation attack by a node
	bIsolationAttackBug = false; //Have an attacker stick to it's target  *****
	bIsolationAttackNeighbor = false; //Execute isolation attack
	bEnableFictive = true; //Activate fictive defence mode   *****
	bool bEnableFictiveMitigation = false; //Activate new fictive defence mode (new algorithm, mitigation)
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
      ...


```

To make life easier, as we introduced the SEED to pre-determine the randomness on which the simulation will be using, just run the simulation with ```--RngRun=<YOUR_SEED>```
where ``<YOUR_SEED>`` is replaced with an int between 0 to 2147483647 (max int). 

This way, you can configure different configurations as mentioned, but the randomness of movement or the positioning of nodes between each set of configuration will be determined and consistent if supplied with the same SEED to the ```--RngRun``` argument.
</n></n>



5) Getting access to the ns-3 documentation
-------------------------------------------

Once you have verified that your build of ns-3 works by running
the simple-point-to-point example as outlined in 3) above, and the OLSR simulations and the research paper's simulations,
it is quite likely that you will want to get started on reading
some ns-3 documentation. 

All of that documentation should always be available from
the ns-3 website: http:://www.nsnam.org/documentation/.

This documentation includes:

  - a tutorial
 
  - a reference manual

  - models in the ns-3 model library

  - a wiki for user-contributed tips: http://www.nsnam.org/wiki/

  - API documentation generated using doxygen: this is
    a reference manual, most likely not very well suited 
    as introductory text:
    http://www.nsnam.org/doxygen/index.html

6) Working with the development version of ns-3
-----------------------------------------------

If you want to download and use the development version 
of ns-3, you need to use the tool 'mercurial'. A quick and
dirty cheat sheet is included in doc/mercurial.txt but
reading through the mercurial tutorials included on the
mercurial website is usually a good idea if you are not
familiar with it.

If you have successfully installed mercurial, you can get
a copy of the development version with the following command:
"hg clone http://code.nsnam.org/ns-3-dev"
