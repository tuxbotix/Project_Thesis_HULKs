#!/bin/bash

naoConfPath="../../../nao/home/"

# supFeet=("l" "r" "d")

chaining="n"
mirror="noMirror"

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

if [ -n "${3}" ]
  then
    case "${3}" in
      "sum")
        chaining="s"
        ;;
      "mul")
        chaining="m"
        ;;
      *)
        echo " Skip options invalid ${3}"
        exit 1
    esac
fi

if [ -n "${4}" ]
  then
    case "${4}" in
      "extract")
        skipUpto="e"
        ;;
      "filter")
        skipUpto="f"
        ;;
      *)
        echo " Skip ptions invalid ${4}"
        exit 1
    esac
fi

#if [ -n "${5}" ]
#  then
#    case "${5}" in
#      "m")
#        mirror="m"
#        ;;
#      *)
#        ;;
#    esac
#fi

outFile=$1
supFoot=$2

outFilePath=$(dirname ${outFile})
outFileBaseName=$(basename ${outFile})

#find $(dirname "${PWD}/cppLinkNao/*.cpp") -maxdepth 1 -wholename "*.cpp"

tempPoseName="out_poses_${supFoot}_temp.txt"
tempSensName="out_senses_${supFoot}_temp.txt"

# for var in "${supFeet[@]}"
# do
    echo "Starting ${supFoot} foot; poseGen";
if [ -z "${skipUpto}" ]
  then
    ./poseGen "${outFile}_${supFoot}" ${supFoot} ${naoConfPath}
fi
if [ -z "${skipUpto}" ] || [ "${skipUpto}" == "e" ]
  then
    echo "Starting ${supFoot} foot; SensExtractor";
    ./sensitivityExtractor "${outFile}_${supFoot}" ${naoConfPath}
fi
    echo "Starting poseFilter - generic";
    ./poseFilter "${outFile}_${supFoot}" -1 all ${chaining} ${naoConfPath}
    for j in {8..19}
    do
        echo "Starting poseFilter Joint: ${j}";
        ./poseFilter "${outFile}_${supFoot}" ${j} all ${chaining} ${naoConfPath}
    done
# done
