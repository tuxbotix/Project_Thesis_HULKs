#!/usr/bin/env python
# coding: utf-8

import collections
import os
import math
import json
import sys

import numpy as np
from scipy.optimize import curve_fit
from scipy import asarray as ar, exp
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from matplotlib import cm
import matplotlib.ticker as ticker
import matplotlib2tikz

import utils
import testConfigUtils


class calibStatTable:
    def __init__(self, poseSet, noiseSources):
        self.poseSet = sorted(poseSet)
        self.noiseSources = noiseSources

        self.rowCols = {}

        for noiseSource in noiseSources:
            self.rowCols[noiseSource] = {
                poseNum: {"s": 0, "l": 0, "o": 0} for poseNum in self.poseSet}

        colConfig = "|c|" + \
            "|".join(
                [("".ljust(3, "c"))] * len(self.poseSet)
            ) + "|"
        self.tableBegin = "\\begin{table}[]\n" +\
            "\\resizebox{\\textwidth}{!}{%\n" +\
            "\\def\\arraystretch{1.5}%  1 is the default, change whatever you need\n" +\
            "\\begin{tabular}{"+colConfig+"}\n" +\
            "\\hline\n"

        self.tableEnd = "\\end{tabular}%\n" +\
                        "}\n" +\
                        "\\end{table}"
        self.tableHeader = ""
        for poseNum in self.poseSet:
            self.tableHeader += "& \\multicolumn{3}{c|}{" + \
                str(poseNum)+" Poses}"
        self.tableHeader += "\\\\ \\hline \n"

    def updateCell(self, noiseSource, poseNum, successRate,
                   localMinimaRate, others):
        if noiseSource in self.rowCols and \
                poseNum in self.rowCols[noiseSource]:
            self.rowCols[noiseSource][poseNum]["s"] = successRate
            self.rowCols[noiseSource][poseNum]["l"] = localMinimaRate
            self.rowCols[noiseSource][poseNum]["o"] = others
        else:
            print("insert fail")

    def renderTable(self):
        output = self.tableBegin
        output += self.tableHeader

        t = "Noise Src. & "
        for poseNum in self.poseSet:
            t += " Succ.\\% & LocMin.\\% & Other\\% &"

        output += t[:-1] + "\\\\ \\hline \n"

        noiseSourceCount = len(self.noiseSources)

        for num, noiseSource in enumerate(self.noiseSources):
            rowData = self.rowCols[noiseSource]

            row = str(noiseSource) + " &\t"
            for poseNum in self.poseSet:
                p = rowData[poseNum]
                row += "{:.2f}".format(p["s"]) + " & " + "{:.2f}".format(p["l"]) +\
                    " & " + "{:.2f}".format(p["o"]) + " &"

            # remove trailing & and attach line breaks
            output += row[:-1] + "\\\\ \n"
        output += "\\hline \n"
        return output + self.tableEnd


def combineNoiseSrcWithJointSample(noiseSrc, jointSampleCount):
    if jointSampleCount != 1:
        return noiseSrc + "_" + str(jointSampleCount)
    else:
        return noiseSrc


def combineFeatureWithSolver(feat, solver):
    return str(feat) + "_" + str(solver)


def drawTable(configData, summaryData):

    curPoseNum = 0

    noiseWithSampleInfo = []
    featureWithSolvers = []

    jointSampleCounts = list(configData.get("jointSampleCount", [1]))
    solvers = list(configData.get(
        "solvers", [testConfigUtils.SolverTypes.stdLM.value]))

    for sampleCount in jointSampleCounts:
        for noise in configData["noiseTypes"]:
            noiseWithSampleInfo.append(
                combineNoiseSrcWithJointSample(noise, sampleCount))

    for feat in configData["features"]:
        for solver in solvers:
            featureWithSolvers.append(combineFeatureWithSolver(feat[0], solver))

    featureTables = {
        val: calibStatTable(configData["poseNum"], noiseWithSampleInfo)
        for val in featureWithSolvers
    }

    for data in summaryData:
        feature = data["featureName"]
        solver = data.get("solver", testConfigUtils.SolverTypes.stdLM.value)

        comboFeat = combineFeatureWithSolver(feature, solver)

        if curPoseNum != data["poseNumber"]:
            curPoseNum = data["poseNumber"]

        success = data["successRate"]
        localMinima = data["localMinimaRate"]
        others = 100.0 - success - localMinima

        augNoiseName = combineNoiseSrcWithJointSample(
            data["noiseSource"], data.get("jointSampleCount", 1))

        featureTables[comboFeat].updateCell(augNoiseName, data["poseNumber"],
                                            success, localMinima, others)
        # print(data["errAmount"], data["poseNumber"], data["featureName"][:3],
        #       data["noiseSource"][:4], "{:.2f}".format(data["successRate"]),
        #       "{:.2f}".format(data["localMinimaRate"]))

    for featName, table in featureTables.items():
        print()
        print(featName, "\n")
        print(table.renderTable())
    print()

if __name__ == "__main__":
    summaryFileName = os.path.join(sys.argv[1], "summary.json")
    configInfoFileName = os.path.join(sys.argv[1], "configPrefixInfos.json")

    summaryData = {}
    configData = {}
    with open(summaryFileName) as f:
        summaryData = json.load(f)
    with open(configInfoFileName) as f:
        configData = json.load(f)

    drawTable(configData["configurations"], summaryData)
