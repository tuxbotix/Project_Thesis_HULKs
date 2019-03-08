#!/bin/bash

naoConfPath="../../../nao_master/home/"

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
    ./poseGen -f "${outFile}_${supFoot}" -s ${supFoot} -c ${naoConfPath}
fi
if [ -z "${skipUpto}" ] || [ "${skipUpto}" == "e" ]
  then
    echo "Starting ${supFoot} foot; SensExtractor";
    ./sensitivityExtractor -f "${outFile}_${supFoot}" -c ${naoConfPath}
fi
    echo "Starting poseFilter - generic";
    ./poseFilter -f "${outFile}_${supFoot}" -l ${chaining} -c ${naoConfPath}
    for j in {8..19}
    do
        echo "Starting poseFilter Joint: ${j}";
        ./poseFilter -f "${outFile}_${supFoot}" -l ${chaining} -j ${j} -c ${naoConfPath}
    done
# done
