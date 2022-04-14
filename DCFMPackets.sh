#!/bin/bash

function runScript () {
R=$1
X=500
Y=500
N=50
TIME=425
FICTIVE="true"
ATTACK="false"
MOBILITY="false"
HIGHRANGE="false"
FILE="../dcfmDelay_n${N}_${X}x${Y}_Mobility-${MOBILITY}-"
OUTPUTFILE="${FILE}output-r${R}.txt"
PACKETSFILE="${FILE}packets-r${R}.txt"
OUTPUTFILETOTAL="${FILE}output.txt"
PACKETSFILETOTAL="${FILE}packets.txt"

# Create temp folder to run from
MYTEMPDIR=$(mktemp -dp .)
cd $MYTEMPDIR

for ((i=$2;i<$3;i+=1)); do
  echo "This is run $i" >> $OUTPUTFILE
  echo "This is run $i" >> $PACKETSFILE

  # Run script
  #NS_GLOBAL_VALUE="RngRun=$i" ./waf --run "scratch/bottleneck --nNodes=$N --nMaxGridX=$X --nMaxGridY=$Y --bMobility=$MOBILITY --nSimulationSeconds=$TIME --bHighRange=$HIGHRANGE --bUdpServer=true --bMakeBets=true --bFixPos=$FIXPOS" >> $OUTPUTFILE
  #NS_GLOBAL_VALUE="RngRun=$i" ../build/scratch/bottleneck --nNodes=$N --nMaxGridX=$X --nMaxGridY=$Y --bMobility=$MOBILITY --nSimulationSeconds=$TIME --bHighRange=$HIGHRANGE --bUdpServer=true --bMakeBets=true --bFixPos=$FIXPOS --bRigPath=$RIGPATH >> $OUTPUTFILE
  NS_GLOBAL_VALUE="RngRun=$i" ../build/scratch/iolsr-tests --nNodes=$N --nMaxGridX=$X --nMaxGridY=$Y --bMobility=$MOBILITY --nSimulationSeconds=$TIME --bHighRange=$HIGHRANGE --bUdpServer=true --bEnableFictive=true --bIsolationAttackBug=false --bPrintMprFraction=true >> $OUTPUTFILE

  # Extract packets
  mergecap -w tmp.pcap DelayTest_*.pcap
  tcpdump -n -tt -e -r tmp.pcap | grep -i UDP | grep -P '(\:01|\:02)' | uniq -u | awk '{print $1, $2, $3, $4}' >> $PACKETSFILE

  # Clean
  rm DelayTest_*.pcap
  rm tmp.pcap

  echo This run is $i
  date
done

# Pack result
flock -e $PACKETSFILETOTAL cat $PACKETSFILE >> $PACKETSFILETOTAL
flock -e $OUTPUTFILETOTAL cat $OUTPUTFILE >> $OUTPUTFILETOTAL

mkdir -p ../tmpdel
mv $PACKETSFILE ../tmpdel
mv $OUTPUTFILE ../tmpdel
# delete temp folder
cd ..
rm -R $MYTEMPDIR

}

set -m # Don't lose job control!
runScript 1 0 63 &
runScript 2 63 125 &
runScript 3 125 188 &
runScript 4 188 250 &
runScript 5 250 313 &
runScript 6 313 375 &
runScript 7 375 438 &
runScript 8 438 500 &
#runScript 1 0 125 &
#runScript 2 125 250 &
#runScript 3 250 375 &
#runScript 4 375 500 &
#runScript 5 500 625 &
#runScript 6 625 750 &
#runScript 7 750 875 &
#runScript 8 875 1000 &
#runScript 1 0 2500 &
#runScript 2 2500 5000 &
#runScript 3 5000 7500 &
#runScript 4 7500 10000 &
#runScript 1 0 10 &
#runScript 2 10 20 &
#runScript 3 20 30 &
#runScript 4 30 40 &
#runScript 5 40 50 &
