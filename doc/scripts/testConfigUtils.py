import json
import enum
import os


class noiseConfigs(str, enum.Enum):
    noNoise = "noNoise"
    jointOnly = "joint"
    pixelOnly = "pixel"
    both = "both"


class SolverTypes(str, enum.Enum):
    stdLM = "stdLm"
    stochasticLM = "stochLM"


def solverTypeToParam(solverType: SolverTypes):
    if solverType == SolverTypes.stdLM:
        return []
    elif solverType == SolverTypes.stochasticLM:
        return ["-s"]
    else:
        raise ValueError("Not a correct solver type")


def noiseTypeToParam(noiseType: noiseConfigs):
    if noiseType == noiseConfigs.noNoise:
        return []
    elif noiseType == noiseConfigs.pixelOnly:
        return ["-p"]
    elif noiseType == noiseConfigs.jointOnly:
        return ["-j"]
    elif noiseType == noiseConfigs.both:
        return ["-p", "-j"]
    else:
        raise ValueError("Not a correct noise type")


def makeOutFileFromConfigs(errorType, featureType, solver, poseNumber, noiseType, jointSampleCount):
    val = str(errorType) + "_n"+str(poseNumber) + \
        "_"+str(featureType) + "_" + str(solver.value) + "_" + \
        noiseType.value + "_jSample_" + str(jointSampleCount)

    return val


def makeConfigs(cfgParams, outputDir):

    errRanges = list(cfgParams["errors"])
    featureTypes = list(cfgParams["features"])
    poseNumbers = list(cfgParams["poseNum"])
    noiseTypes = list(cfgParams["noiseTypes"])
    jointSampleCounts = list(cfgParams.get("jointSampleCount", [1]))
    solvers = list(cfgParams.get("solvers", [SolverTypes.stdLM.value]))

    configs = []

    for feature in featureTypes:
        for solver in solvers:
            for err in errRanges:
                for poseNum in poseNumbers:
                    for noise in noiseTypes:
                        for jointSampleCount in jointSampleCounts:

                            if jointSampleCount != 1 and (noise == noiseConfigs.noNoise or noise == noiseConfigs.pixelOnly):
                                continue
                            if feature[0] == "gnd" and solver != SolverTypes.stdLM:
                                continue

                            errPopulation = err[3]

                            config = ["-e", err[1], "--testErrorCount", str(errPopulation), "--calib",
                                      feature[1], "-n", str(poseNum), "--jointSampleCount", str(jointSampleCount)]

                            if len(solverTypeToParam(solver)):
                                config += solverTypeToParam(solver)

                            if len(noiseTypeToParam(noise)):
                                config += noiseTypeToParam(noise)
                            outPrefix = makeOutFileFromConfigs(
                                err[0], feature[0], solver, poseNum, noise, jointSampleCount)

                            info = {
                                "errName": err[1],
                                "errAmount":  err[2],
                                "errPopulation": errPopulation,
                                "poseNumber": poseNum,
                                "feature": feature[1],
                                "featureName": feature[0],
                                "noiseSource": noise.value,
                                "prefix": os.path.join(outPrefix, "out"),
                                "jointSampleCount": jointSampleCount,
                                "solver": solver.value
                            }
                            config += ["-o",
                                       os.path.join(outputDir, outPrefix, "out")]
                            configs.append(
                                (config, info, os.path.join(outputDir,  outPrefix)))
                        # print(config, "\n")
    return configs


def makeErrorConfig(prefix, filePath, errorAbsMax, popSize):
    return (prefix, filePath, errorAbsMax, popSize)


def makeFeatureConfig(prefix, filePath):
    return (prefix, filePath)


def getTestConfigStructure():
    return {
        # Error types.
        "errors": [],
        # errors = [("3.5_10k", errors10k3_5deg, 3.5, 10000),
        #           ("6.5_10k", errors10k6_5deg, 6.5, 10000)]
        # errors = [("6.5_1k", errors1k6_5deg, 6.5, 1000)]
        "features": [],
        # pose Number
        # poseNum = [3, 5, 8]
        "poseNum": [],
        "noiseTypes": [
            noiseConfigs.noNoise, noiseConfigs.pixelOnly,
            noiseConfigs.jointOnly, noiseConfigs.both
        ],
        "jointSampleCount": [],
        "solvers": []
    }
