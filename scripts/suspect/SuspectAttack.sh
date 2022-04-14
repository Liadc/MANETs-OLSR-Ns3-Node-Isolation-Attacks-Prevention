#!/bin/bash
X=1000
Y=750
N=100
TIME=61
FICTIVE="true"
ATTACK="true"
MOBILITY="false"
HIGHSPEED="false"
FILE="Suspect_Rate_n${N}_${X}x${Y}_Fictive-${FICTIVE}-Attack-${ATTACK}-Mobility-${MOBILITY}.txt"
for ((i=0;i<1200;i+=1)); do  
    echo "This is run $i" >> $FILE
    NS_GLOBAL_VALUE="RngRun=$i" NS_LOG="StableNetworkRouteMod=level_info" ./waf --run "scratch/stable_network_mod --nNodes=$N --nMaxGridX=$X --nMaxGridY=$Y --bPrintMprFraction=false --nSimulationSeconds=$TIME --bPrintAll=false --bPrintFakeCount=false --bEnableFictive=$FICTIVE --bIsolationAttackBug=$ATTACK --bUdpServer=false --bHighRange=true --bPrintTcPowerLevel=false --bPrintRiskyFraction=true --bMobility=$MOBILITY" >> $FILE
date
    echo This run is $i
done

