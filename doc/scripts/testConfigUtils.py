import json
import enum
import os


class noiseConfigs(str, enum.Enum):
    noNoise = "noNoise"
    jointOnly = "joint"
    pixelOnly = "pixel"
    both = "both"


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


def makeOutFileFromConfigs(errorType, featureType, poseNumber, noiseType):
    val = str(errorType) + "_n"+str(poseNumber) + "_"+str(featureType)
    val += ("_" + noiseType.value)

    return val


def makeConfigs(cfgParams, outputDir):

    errRanges = list(cfgParams["errors"])
    featureTypes = list(cfgParams["features"])
    poseNumbers = list(cfgParams["poseNum"])
    noiseTypes = list(cfgParams["noiseTypes"])

    configs = []

    for feature in featureTypes:
        for err in errRanges:
            for poseNum in poseNumbers:
                for noise in noiseTypes:
                    errPopulation = err[3]

                    config = ["-e", err[1], "--testErrorCount", str(errPopulation), "--calib",
                              feature[1], "-n", str(poseNum)]
                    if len(noiseTypeToParam(noise)):
                        config += noiseTypeToParam(noise)
                    outPrefix = makeOutFileFromConfigs(
                        err[0], feature[0], poseNum, noise)

                    info = {
                        "errName": err[1],
                        "errAmount":  err[2],
                        "errPopulation": errPopulation,
                        "poseNumber": poseNum,
                        "feature": feature[1],
                        "featureName": feature[0],
                        "noiseSource": noise.value,
                        "prefix": os.path.join(outPrefix, "out")}
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
        "noiseTypes": [noiseConfigs.noNoise, noiseConfigs.pixelOnly,
                       noiseConfigs.jointOnly, noiseConfigs.both]
    }
