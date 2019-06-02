import os
import sys
import subprocess

import json
import enum

import testConfigUtils as testCUtils


def runCalibSimProcess(stdParams, extraParams, outPrefix):
    os.makedirs(os.path.realpath(outPrefix), exist_ok=True)

    p = subprocess.Popen(
        (stdParams + extraParams), stdout=subprocess.PIPE, stderr=subprocess.PIPE)

    data = p.communicate()

    with open(os.path.join(os.path.realpath(outPrefix), "out_log.txt"), "wb") as f:
        f.write(data[0])
    with open(os.path.join(os.path.realpath(outPrefix), "out_errors.txt"), "wb") as f:
        f.write(data[1])
    return p.returncode


def main():
    baseDir = os.path.dirname(os.path.realpath(sys.argv[0]))

    repoBase = os.path.realpath(os.path.join(baseDir, "../../"))
    binDir = os.path.join(repoBase, "build/Release/")
    dataDir = os.path.realpath(os.path.join(repoBase, "Data/"))
    logDir = os.path.realpath(os.path.join(repoBase, "doc/logs/"))
    nao_config_dir = os.path.join(repoBase, "../nao_master/home/")

    sim = os.path.join(binDir, "simulatedEvaluation")

    # Errors and other data
    errors1k6_5deg = os.path.join(dataDir, "l_1000_jointErrors.txt")
    errors10k6_5deg = os.path.join(dataDir, "l_10k_jointErrors.txt")
    errors10k3_5deg = os.path.join(dataDir, "l_10k_3_5degjointErrors.txt")
    calibGround = os.path.join(dataDir, "calibFeatureGround.txt")
    calibCharucoMulti = os.path.join(dataDir, "calibFeatureCharucoMulti.txt")

    poses = os.path.join(
        dataDir, "2019-03-19_data2019_dbl_n_FilteredPoses_generic.txt")
    tests = os.path.join(
        dataDir, "2019-03-19_data2019_dbl_n_TestPoses_generic")

    # standard params for sim
    simStd = [sim, "--confRoot", binDir, "--naoConfRoot",
              nao_config_dir, "-f", poses, "-t", tests, "-h"]

    # Test configurations.
    configurations = {
        # Error types.
        "errors": [("3.5_2k", errors10k3_5deg, 3.5, 500)],
        # errors = [("3.5_10k", errors10k3_5deg, 3.5, 10000),
        #           ("6.5_10k", errors10k6_5deg, 6.5, 10000)]
        # errors = [("6.5_1k", errors1k6_5deg, 6.5, 1000)]
        "features": [("charucoMulti", calibCharucoMulti), ("gnd", calibGround)],
        # pose Number
        # poseNum = [3, 5, 8]
        "poseNum": [5],
        "noiseTypes": [testCUtils.noiseConfigs.noNoise, testCUtils.noiseConfigs.pixelOnly,
                       testCUtils.noiseConfigs.jointOnly, testCUtils.noiseConfigs.both]
    }

    outputDir = os.path.join(logDir, "2019-05-26")

    if not os.path.isdir(os.path.abspath(os.path.join(outputDir, os.pardir))):
        print("Output dir not existing..")
        exit(1)

    configs = testCUtils.makeConfigs(configurations, outputDir)

    configCount = len(configs)

    # print(simStd)
    with open(os.path.join(outputDir, "configPrefixInfos.json"), "w") as f:
        completed = 0
        infoArr = []
        for config in configs:

            # args = " ".join(simStd) + " ".join(config[0])
            print("Start config: ", config)
            returnCode = runCalibSimProcess(simStd, config[0], config[2])
            completed += 1
            print("Finished, ", int(completed * 100 /
                                    configCount), "% completed.\n")

            if returnCode == 0:
                infoArr.append(config[1])
            else:
                print("failed running...")

        configsWithInfo = {
            "configurations": configurations, "runInfo": infoArr}
        json.dump(configsWithInfo, f, indent=2)
        print("All done, check: ", os.path.realpath(outputDir))


if __name__ == "__main__":
    main()