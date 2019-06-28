#!/usr/bin/env python
# coding: utf-8


import os
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from scipy.optimize import curve_fit
from scipy import asarray as ar, exp
from matplotlib import cm
import json
import sys
import matplotlib.ticker as ticker
import utils

import matplotlib2tikz

import testConfigUtils

plt.rcParams["figure.figsize"] = (6, 8)


class TestRun:
    jointNames = ["HEAD_YAW",
                  "HEAD_PITCH",
                  "L_HIP_YAW_PITCH",
                  "L_HIP_ROLL",
                  "L_HIP_PITCH",
                  "L_KNEE_PITCH",
                  "L_ANKLE_PITCH",
                  "L_ANKLE_ROLL",
                  "R_HIP_YAW_PITCH",
                  "R_HIP_ROLL",
                  "R_HIP_PITCH",
                  "R_KNEE_PITCH",
                  "R_ANKLE_PITCH",
                  "R_ANKLE_ROLL"]
    jointNamesShort = [
                  "HEAD_Y",
                  "HEAD_P",
                  "L_HIP_YP",
                  "L_HIP_R",
                  "L_HIP_P",
                  "L_KNEE_P",
                  "L_ANKL_P",
                  "L_ANKL_R",
                  "R_HIP_YP",
                  "R_HIP_R",
                  "R_HIP_P",
                  "R_KNEE_P",
                  "R_ANKL_P",
                  "R_ANKL_R"]

    JointCount = len(jointNames)

    def __init__(self, data):
        self.status = data[0]
        self.reprojectionErrorMeanTest = data[1]
        self.rmsTest_x = data[2]
        self.rmsTest_y = data[3]
        self.reprojectionErrorMeanCalib = data[4]
        self.rmsCalib_x = data[5]
        self.rmsCalib_y = data[6]
        self.residual = data[7:7+TestRun.JointCount]
        self.errorConfig = data[7+TestRun.JointCount:
                                7+(TestRun.JointCount * 2)]


def loadLogs(log_root):

    resultList = []
    labels = None

    with open(log_root + "_resultOut", "r") as f:
        labels = f.readline()
        lines = f.readlines()
        # for line in lines:
        resultList = np.array(
            [np.array(line.split(), dtype=float) for line in lines])
    # print(resultList.shape)
    testRuns = [TestRun(val) for val in resultList]

    unique, counts = np.unique(resultList[:, 0], return_counts=True)
    stats = dict(zip(unique, counts))
    resultLen = len(resultList[:, 0])

    GoodResults = resultList[resultList[:, 0] == 5]
    BadResults = resultList[resultList[:, 0] != 5]

    # prePostResiduals = []
    # with open(log_root + "_prePostResiduals") as f:
    #     # for line in lines:
    #     lines = f.readlines()
    #     prePostResiduals = [
    #         np.array(line.split(), dtype=float) for line in lines]

    # origResX = prePostResiduals[0]
    # origResY = prePostResiduals[1]
    # calibResX = prePostResiduals[2]
    # calibResY = prePostResiduals[3]

    goodCalibJointRes = np.array(
        [val.residual for val in testRuns if val.status == 5])
    badCalibJointRes = np.array(
        [val.residual for val in testRuns if val.status != 5])
    unCalibJointRes = np.array([val.errorConfig for val in testRuns])

    # print(goodCalibJointRes.shape, badCalibJointRes.shape, unCalibJointRes.shape)

    return {
        "stats": {
            "FAIL_LOCAL_MINIMA": (stats.get(0, 0) / resultLen),
            "FAIL_NO_CONVERGE": (stats.get(1, 0) / resultLen),
            "FAIL_NUMERICAL": (stats.get(2, 0) / resultLen),
            "FAIL_NO_CAPTURES": (stats.get(3, 0) / resultLen),
            "FAIL_NO_TEST_CAPTURES": (stats.get(4, 0) / resultLen),
            "SUCCESS": (stats.get(5, 0) / resultLen)
        },
        "GoodResults": GoodResults, "BadResults": BadResults,
        # "origResX": origResX, "origResY": origResY,
        # "calibResX": calibResX, "calibResY": calibResY,
        "badCalibJointRes": badCalibJointRes, "goodCalibJointRes": goodCalibJointRes,
        "unCalibJointRes": unCalibJointRes
    }


def plotResults(logs, log_dir):
    GoodResults = logs["GoodResults"]
    BadResults = logs["BadResults"]
    # origResX = logs["origResX"]
    # origResY = logs["origResY"]
    # calibResX = logs["calibResX"]
    # calibResY = logs["calibResY"]
    goodCalibJointRes = logs["goodCalibJointRes"]
    badCalibJointRes = logs["badCalibJointRes"]
    unCalibJointRes = logs["unCalibJointRes"]
    stats = logs["stats"]

    fig, _axs = plt.subplots(nrows=2, ncols=1)
    axs = _axs.flatten()

    ax = axs[0]

    n, bins, patches = ax.hist(
        GoodResults[:, 2], 200, density=False, facecolor='g', alpha=0.5)
    n, bins, patches = ax.hist(
        BadResults[:, 2], bins=bins, density=False,  facecolor='r', alpha=0.5)

    ax = axs[1]

    n, bins, patches = ax.hist(
        GoodResults[:, 3], 200, density=False, facecolor='g', alpha=0.5)
    n, bins, patches = ax.hist(
        BadResults[:, 3], bins=bins, density=False, facecolor='r', alpha=0.5)

    matplotlib2tikz.save(log_dir + "rms_hist.tex",  figurewidth='15cm',
                         figureheight='10cm')
    fig.savefig(log_dir + "rms_hist.pdf", bbox_inches='tight')
    plt.close(fig)

##########################################################

    # plt.figure(num=None, figsize=(6, 3), dpi=200)
    fig, _axs = plt.subplots(nrows=1, ncols=1, num=None,
                             figsize=(6, 4), dpi=200)
    # plt.subplots_adjust(top=0.95, bottom=0.05)
    # plt.subplots_adjust(left=0, bottom=0, right=1, top=1, wspace=0, hspace=0)

    # axs = _axs[0]
    # axs[0].get_shared_x_axes().join(axs[0], axs[1])
    ax = plt.gca()
    flierprops = dict(marker='o', markersize=3,
                      alpha=0.4, markeredgecolor='purple')
    capprops = dict(linestyle='--', linewidth=1.5, color='black')

    # print("box1")
    

    # ax.set_xlabel('Error in Degrees')
    ax.set_xlabel('Error distribution ($\degree$)')
    ax.set_ylabel('Joint')

    ax.set_title('Joint error distribution before calibration')
    # ax.violinplot(unCalibJointRes, vert=False)
    ax.boxplot(unCalibJointRes, vert=False, showfliers=True,
               flierprops=flierprops, capprops=capprops)

    ax.set_yticks(list(range(1, len(TestRun.jointNames) + 1)), minor=False)

    ax.xaxis.set_major_locator(ticker.MultipleLocator(1))
    ax.xaxis.set_minor_locator(ticker.MultipleLocator(0.2))
    ax.set_xlim([-8, 8])

    ax.set_yticklabels(TestRun.jointNamesShort, minor=False)

    # ax.xaxis.set_minor_locator(ticker.MultipleLocator(0.005))
    ax.xaxis.grid()
    fig.tight_layout()
    fig.savefig(log_dir + "before_hist.pdf")
    matplotlib2tikz.save(log_dir + "before_hist.tex",  figurewidth='15cm',
                         figureheight='8cm')
    plt.close(fig)

# after
    fig, _axs = plt.subplots(nrows=1, ncols=1, num=None,
                             figsize=(6, 4), dpi=200)
    # plt.figure(num=None, figsize=(6, 3), dpi=200)
    # violinMargins = ax.margins()
    # ax = axs[0]
    ax = plt.gca()

    ax.set_title('Joint error distribution after calibration')
    ax.set_xlabel('Error distribution ($\degree$)')
    ax.set_ylabel('Joint')

    totalData = np.append(badCalibJointRes, goodCalibJointRes, axis=0)
    # ax.violinplot(totalData, vert=False)
    ax.boxplot(totalData, vert=False, showfliers=True,
               flierprops=flierprops, capprops=capprops)

    ax.set_yticks(list(range(1, len(TestRun.jointNames) + 1)), minor=False)
    ax.xaxis.set_major_locator(ticker.MultipleLocator(1))
    ax.xaxis.set_minor_locator(ticker.MultipleLocator(0.2))
    ax.set_xlim([-8, 8])
    ax.xaxis.grid()

    ax.set_yticklabels(TestRun.jointNamesShort, minor=False)

    # locs, labels = ax.xticks()
    # if len(labels) != TestRun.jointNames:
    #     print("Joint names and numbers mismatc")
    #     raise ValueError("Joint names and numbers mismatc")
    # ax.xticks(locs, TestRun.jointNames)

    # ax.margins(violinMargins[0], violinMargins[1])
    # plt.tight_layout()
    fig.tight_layout()

    fig.savefig(log_dir + "after_hist.pdf")
    matplotlib2tikz.save(log_dir + "after_hist.tex",  figurewidth='15cm',
                         figureheight='8cm')
    # plt.show()
    plt.close(fig)


if __name__ == "__main__":

    baseDir = os.path.dirname(os.path.realpath(sys.argv[0]))

    log_root, manifest = utils.selectLogLocation(baseDir)
    print("LOG_ROOT ", log_root)

    testConfigs = manifest["configurations"]
    runInfo = manifest["runInfo"]

    dataInfo = []
    for testRun in runInfo:
        prefix = testRun["prefix"]
        print(prefix)
        logs = loadLogs(os.path.join(log_root, prefix))

        poseNumber = testRun["poseNumber"]
        errAmount = testRun["errAmount"]
        errPopulation = testRun["errPopulation"]
        noiseSource = testRun["noiseSource"]
        featureName = testRun["featureName"]
        featureName = testRun["featureName"]
        jointSampleCounts = testRun.get("jointSampleCount", 1)
        solver = testRun.get("solver", testConfigUtils.SolverTypes.stdLM.value)

        successRate = logs["stats"]["SUCCESS"] * 100
        localMinimaRate = logs["stats"]["FAIL_LOCAL_MINIMA"] * 100
        noConvergeRate = logs["stats"]["FAIL_NO_CONVERGE"] * 100

        data = {"errPopulation": errPopulation, "errAmount": errAmount,
                "poseNumber": poseNumber, "noiseSource": noiseSource,
                "successRate": successRate, "localMinimaRate": localMinimaRate,
                "noConvergeRate": noConvergeRate, "prefix": prefix,
                "featureName": featureName, "jointSampleCount": jointSampleCounts,
                "solver": solver
                }

        dataInfo.append(data)

        plotResults(logs, os.path.join(log_root, testRun["prefix"]))

    with open(os.path.join(log_root, "summary.json"), 'w') as f:
        json.dump(dataInfo, f, indent=2)

    # for data in dataInfo:
    #     print(data)
    # for config in makeConfigs
    # statsToTable()
    # exit(0)

    # test = runInfo[0]
    # logs = loadLogs(os.path.join(log_root, test["prefix"]))

    # plotResults(logs)

    # statsToTable(testConfig, runInfo)
