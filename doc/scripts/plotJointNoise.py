#!/usr/bin/env python
# coding: utf-8

import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from scipy.optimize import curve_fit
import scipy.stats as stats
from scipy import asarray as ar, exp
from matplotlib import cm
import matplotlib.ticker as ticker

import matplotlib2tikz

def gaus(x, a, x0, sigma):
    return a * exp(-(x - x0)**2 / (2 * sigma**2))


def fitGauss(data):
    x = data[:, 0]
    y = data[:, 1]
    n = len(x)
    mean = sum(x * y) / n
    sigma = sum(y * (x - mean)**2) / n

    return curve_fit(gaus, x, y, p0=[1, mean, sigma])


# status, reprojErrMeanTest, rmsTest, reprojErrMeanCalib, rmsCalib
angleDump = np.loadtxt("../logs/2019-03-30_jointAngleDump.txt", dtype = np.float64).transpose()

for i in range(0, angleDump.shape[1]):
    arr = angleDump[:, i]
    minVal = np.min(arr)
    maxVal = np.max(arr)
    average = np.median(arr)
    arr = arr - average
    newMinVal = np.min(arr)
    newMaxVal = np.max(arr)
    print(i, np.rad2deg(abs(maxVal - minVal)), np.rad2deg(minVal), np.rad2deg(maxVal))
    print("\t", i, np.rad2deg(abs(newMinVal - newMaxVal)), np.rad2deg(newMinVal), np.rad2deg(newMaxVal),np.rad2deg(average))

    angleDump[:, i] = np.rad2deg(arr)
    # vals, covariance = fitGauss(angleDump[:, i])
    # print(vals, covariance)

# plt.figure(0)
# plt.figure(num=None, figsize=(6, 6), dpi=300)

fig, _axs = plt.subplots(nrows=1, ncols=1, num=None, figsize=(6, 6), dpi=200)
# fig.figsize(6, 6)
# fig.figsize(6, 6)

# axs = _axs.flatten()
axs = [_axs]


# ax.boxplot(angleDump, positions = list(range(1, angleDump.shape[1] + 1)))
# plt.xticks(rotation=90)
ax = axs[0]
# plt.xticks(rotation=90)
ax.boxplot(angleDump, vert = False, showmeans = True, positions = list(range(1, angleDump.shape[1]+1)))
ax.xaxis.set_major_locator(ticker.MultipleLocator(0.02))
ax.xaxis.set_minor_locator(ticker.MultipleLocator(0.005))
ax.xaxis.grid()

# plt.xticks(rotation=90)

boxMargins = ax.margins()
ax.set_ylim(0, angleDump.shape[1] + 1)
plt.setp(ax.get_yticklabels(), visible=True)
ticks = ax.get_yticklabels()

matplotlib2tikz.save("../tuvisionthesis/figures/joint_noise.tex",  figurewidth='15cm')

plt.show()
# ax = axs[1]

plt.figure(num=1, figsize=(6, 6), dpi=200)

data = angleDump.ravel()
dataMin=np.min(data)
dataMax=np.max(data)

# totalBins=400

n, bins, patches = plt.hist(data, density=True)

mu = 0.0
sigma = 0.007

p = np.arange(-1.0, 1.0, 0.01)
y =  stats.norm.pdf(bins, mu, sigma)
print(np.sum(y), np.max(bins))

plt.plot(bins, y, '--')

matplotlib2tikz.save("../tuvisionthesis/figures/joint_noise_gaussian.tex", figurewidth='15cm')
plt.show()
