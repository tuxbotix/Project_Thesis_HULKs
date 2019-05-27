#!/bin/bash 

baseDir=$(pwd)

# if [ -z "${1}" ]
#   then
#     echo "input folder not supplied"
#     exit
# fi
# if [ -z "${2}" ]
#   then
#     echo "output file not supplied"
#     exit    
# fi

repoBase="${baseDir}/../../"
binDir="${repoBase}/build/Release/"
dataDir="${repoBase}/Data"

nao_config_dir="${repoBase}/../nao_master/home/"

sim="${binDir}/simulatedEvaluation"

errors1k6_5deg="${dataDir}/l_1000_jointErrors.txt"
errors10k6_5deg="${dataDir}/l_10k_jointErrors.txt"
errors10k4_5deg="${dataDir}/l_10k_jointErrors.txt"

poses="${dataDir}/2019-03-19_data2019_dbl_n_FilteredPoses_generic.txt"
tests="${dataDir}/2019-03-19_data2019_dbl_n_TestPoses_generic"

errors=${errors1k6_5deg}

simStd="${sim} --confRoot ${binDir} --naoConfRoot ${nao_config_dir} -f ${poses} -t ${tests} -e ${errors} -h"

######### Test configurations.

## Error types.
# erroSource

# calib feature
calibGround="${dataDir}/calibFeatureGround.txt"
calibCharuco="${dataDir}/calibFeatureCharucoMulti.txt"

$simStd --calib "${calibCharuco}"
# simWithPixelErr="${simStd} -p"

# boundingBoxDir="$2/3d_boundingBoxes/"
# # logLoc="$2/3d_tracking_logs"

# bboxExtractor="${baseDir}/../BoundingBoxExtractor/build/Release/boundingBox2middle"

# # trackedFilePrefix="$tracked_idc_dir/" 

# mkdir -p $boundingBoxDir
# # mkdir -p $logLoc

# trap "exit" INT
# for f in "$tracked_idc_dir"/*.idc; do
#     if [ -f $f ]; then
#         b=$(basename $f)
#         echo "processing .. $b"
#         prefix="${b:0:-4}"
#         if [ -f "${tracked_idc_dir}${prefix}.mhead" ]; then
          
#           out_prefix="${boundingBoxDir}${prefix}"

#           "$bboxExtractor" "${f}" "${out_prefix}"
#         fi
#     # Control will enter here if $DIRECTORY exists.
#     fi
# done
