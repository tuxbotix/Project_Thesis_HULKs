#!/bin/bash

naoConfPath="../../nao/home/"
# supFeet=("l" "r" "d")

if [ -z "${1}" ]
  then
    echo "outFile not supplied"
    exit
fi
if [ -z "${2}" ]
  then
    echo "Support foot not supplied"
    exit    
fi
outFile=$1
supFoot=$2

outFilePath=$(dirname ${outFile})
outFileBaseName=$(basename ${outFile})

find $(dirname "${PWD}/cppLinkNao/*.cpp") -maxdepth 1 -wholename "*.cpp"

tempPoseName="out_poses_${supFoot}_temp.txt"
tempSensName="out_senses_${supFoot}_temp.txt"

# for var in "${supFeet[@]}"
# do
    echo "Starting ${supFoot} foot; poseGen";

    ./poseGen "${outFile}_${supFoot}" ${supFoot} ${naoConfPath}

    echo "Starting ${supFoot} foot; SensExtractor";

    ./sensitivityExtractor "${outFile}_${supFoot}" ${naoConfPath}

    echo "Starting poseFilter - generic";
    ./poseFilter "${outFile}_${supFoot}" -1 ${naoConfPath}
    for j in {8..19}
    do
        echo "Starting poseFilter Joint: ${j}";
        ./poseFilter "${outFile}_${supFoot}" ${j} ${naoConfPath}
    done
# done
