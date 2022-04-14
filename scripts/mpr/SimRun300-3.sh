#!/bin/bash
X=1000
Y=750
N=300
TIME=245
R=3
FILE="Test_n${N}_${X}x${Y}-r${R}.txt"
for ((i=1000+${R}*50;i<1000+(${R}+1)*50;i+=1)); do
    echo "This is run $i" >> $FILE
    NS_GLOBAL_VALUE="RngRun=$i" ./waf --run "scratch/mprRate --nNodes=$N --nMaxGridX=$X --nMaxGridY=$Y --bPrintMprFraction=true --nSimulationSeconds=$TIME --bPrintAll=false --bPrintFakeCount=true --bEnableFictive=true --bHighRange=false --bPrintTcPowerLevel=true" >> $FILE
    date
    echo This run is $i
done
