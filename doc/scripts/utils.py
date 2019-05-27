import os
import json
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from scipy.optimize import curve_fit
from scipy import asarray as ar, exp
from matplotlib import cm


def gaus(x, a, x0, sigma):
    return a * exp(-(x - x0)**2 / (2 * sigma**2))


def fitGauss(data):
    x = data[:, 0]
    y = data[:, 1]
    n = len(x)
    mean = sum(x * y) / n
    sigma = sum(y * (x - mean)**2) / n

    return curve_fit(gaus, x, y, p0=[1, mean, sigma])


def selectLogLocation(baseDir):

    # init
    location = os.path.join(baseDir, "../logs/")
    print("Location ", location)
    files = os.listdir(location)
    folders = [folder for folder in files if os.path.isdir(
        os.path.join(location, folder))]

    folders.sort(reverse=True)

    if len(folders) <= 0:
        print("No logs available! exiting..", files)
        exit(1)

    for i, name in enumerate(folders):
        print(i, name)

    inp = input("Select item number from above: ")
    if inp:
        x = int(inp)
    else:
        x = 0

    date = folders[x]
    curDir = os.path.join(location, date)

    manifestFile = os.path.join(curDir, "configPrefixInfos.json")

    if os.path.isfile(manifestFile):
        return curDir, json.load(open(manifestFile))

    # files = os.listdir(os.path.join(location, date))
    # folders = [folder for folder in files if os.path.join(
    #     location, date, folder)]

    # if len(folders) <= 0:
    #     print("No logs available! exiting..")
    #     exit(1)
    # for i, name in enumerate(folders):
    #     print(i, name)

    # x = int(input("Select item number from above: "))
    # test_run_configuration = folders[x]

    # log_root = os.path.join(location, date, test_run_configuration)

    # return log_root
