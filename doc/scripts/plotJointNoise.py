#!/usr/bin/env python
# coding: utf-8

# In[97]:


import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from scipy.optimize import curve_fit
from scipy import asarray as ar, exp
from matplotlib import cm
import matplotlib.ticker as ticker

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

fig, _axs = plt.subplots(nrows=1, ncols=1)
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

# ax = axs[1]

# ax.violinplot(angleDump, vert = False, showmeans = True, positions = list(range(1, angleDump.shape[1]+1)))
# ax.xaxis.set_major_locator(ticker.MultipleLocator(0.02))
# ax.xaxis.set_minor_locator(ticker.MultipleLocator(0.005))
# ax.xaxis.grid()

# ax.set_ylim(0, angleDump.shape[1] + 1)
# ax.margins(boxMargins[0], boxMargins[1])
# plt.yticks(range(0, angleDump.shape[1] + 1))
# plt.setp(ticks)

plt.show()
exit(0)

fig, _axs = plt.subplots(nrows=2, ncols=3)
axs = _axs.flatten()

maxCalResX = max(calibResX[:, 0].max(), calibResY[:, 0].max()) * 1.1
maxCalResY = max(calibResX[:, 1].max(), calibResY[:, 1].max()) * 1.1
maxOrigResY = max(origResX[:, 1].max(), origResY[:, 1].max()) * 1.1
maxOrigResX = max(origResX[:, 0].max(), origResY[:, 0].max()) * 1.1


# maxCalJointResX = max(calibJointRes[:, 0].max(), calibJointRes[:, 0].max()) * 1.1
# maxCalJointResY = max(calibJointRes[:, 1].max(), calibJointRes[:, 1].max()) * 1.1
# maxUnCalJointResX = max(unCalibJointRes[:, 0].max(), unCalibJointRes[:, 0].max()) * 1.1
# maxUnCalJointResY = max(unCalibJointRes[:, 1].max(), unCalibJointRes[:, 1].max()) * 1.1
maxAllUncalibY = max(maxOrigResY, 0)

ax = axs[0]

ax.set_title('Reproj. error (x ax.) distribution before calibration.')
ax.plot(origResX[:, 0], origResX[:, 1], 1)
ax.axis([-maxOrigResX, maxOrigResX, -0.02, maxAllUncalibY])

ax = axs[3]
#ax.boxplot(origResX[:, 1])
ax.set_title('Reproj. error (x ax.) distribution after calibration.')
ax.plot(calibResX[:, 0], calibResX[:, 1], 1, label='Raw')
#popt,pcov = fitGauss(calibResX)

#ax.text(-maxCalResX * 0.9, maxCalResY * 0.9, r'$\mu='+'{:.5e}'.format(popt[1])+'$')
#ax.text(-maxCalResX * 0.9, maxCalResY * 0.8, r'$\sigma='+'{:.5e}'.format(popt[2])+'$')
#ax.plot(calibResX[:, 0], gaus(calibResX[:, 0], *popt), 'o', markersize = 2, label='Gaussian fit')
ax.axis([-maxCalResX, maxCalResX, -0.02, maxCalResY])
#print("X std dev and mean", popt[1:])

ax = axs[1]
ax.set_title('Reproj. error (y ax.) distribution before calibration.')
ax.plot(origResY[:, 0], origResY[:, 1], 1)
ax.axis([-maxOrigResX, maxOrigResX, -0.02, maxAllUncalibY])

ax = axs[4]
#popt,pcov = fitGauss(calibResY)
ax.set_title('Reproj. error (y ax.) distribution before calibration.')
ax.plot(calibResY[:, 0], calibResY[:, 1], 1, label='Raw')


#ax.text(-maxCalResX * 0.9, maxCalResY * 0.9, r'$\mu='+'{:.5e}'.format(popt[1])+'$')
#ax.text(-maxCalResX * 0.9, maxCalResY * 0.8, r'$\sigma='+'{:.5e}'.format(popt[2])+'$')
#ax.plot(calibResY[:, 0], gaus(calibResY[:, 0], *popt), 'o', markersize = 2, label='Gaussian fit')

ax.axis([-maxCalResX, maxCalResX, -0.02, maxCalResY])
#print("Y std dev and mean", popt[1:])

print("box1")
ax = axs[2]
ax.set_title('Joint error distribution before calibration')
ax.violinplot(unCalibJointRes)
# ax.plot(unCalibJointRes[:, 0], unCalibJointRes[:, 1], 1, label='Raw')
# ax.axis([-maxUnCalJointResX, maxUnCalJointResX, -0.02, maxAllUncalibY])

ax = axs[5]
#popt,pcov = fitGauss(calibJointRes)
ax.set_title('Joint error distribution after calibration')

ax.boxplot(calibJointRes)
# ax.plot(calibJointRes[:, 0], calibJointRes[:, 1], 1, label='Raw')


#ax.text(-maxCalJointResX * 0.9, maxCalJointResY * 0.9, r'$\mu='+'{:.5e}'.format(popt[1])+'$')
#ax.text(-maxCalJointResX * 0.9, maxCalJointResY * 0.8, r'$\sigma='+'{:.5e}'.format(popt[2])+'$')
#ax.plot(calibJointRes[:, 0], gaus(calibJointRes[:, 0], *popt), 'o', markersize = 2, label='Gaussian fit')

# ax.axis([-maxCalJointResX, maxCalJointResX, -0.02, maxCalJointResY])
#print("Y std dev and mean", popt[1:])


# n, bins, patches = plt.hist(val[:,1], 50, density=True, facecolor='r', alpha=0.75)
# val.size()
# plt.plot(val)


# In[ ]:

plt.show()

# Scatter plots


def prepFor2DHist(x, y, valX, valY):
    '''
    x and y are actually x y value pairs
    '''
    __X, __Y = np.meshgrid(x, y)

    __lenX = len(x)
    __lenY = len(y)
    __Z = [[]] * __lenY

#     __Z = valX * valY
#     print(__Z)
    for idxY in range(0, __lenY):
        __Z[idxY] = valX * valY[idxY]
#         __Z[idxY] = [0] * __lenX
    return __X, __Y, np.array(__Z)


def doHist(plotAxis, fig, XHistData, YHistData):
    xAxis = XHistData[:, 0]
    yAxis = YHistData[:, 0]
    xVals = XHistData[:, 1]
    yVals = YHistData[:, 1]

    newX, newY, newZ = prepFor2DHist(
        origResX[:, 0], origResY[:, 0], origResX[:, 1], origResY[:, 1])

    newZTot = newZ.sum()
    newZ = newZ * 10000 / newZTot

    xAxLim = max(abs(xAxis.min()), xAxis.max()) * 1.1
    yAxLim = max(abs(yAxis.min()), yAxis.max()) * 1.1
    zMin = newZ.min()
    zMax = newZ.max()

    print(zMin, zMax)

    im = ax.imshow(newZ, interpolation='bilinear',
                   origin='lower', extent=[-xAxLim, xAxLim, -yAxLim, xAxLim],
                   vmax=zMax, vmin=zMin)
    # Plot the surface.
    #fig = plt.figure()
    #ax = fig.gca(projection='3d')
    #im = ax.plot_surface(newX, newY, newZ, cmap=cm.coolwarm)

    fig.colorbar(im, ax=ax)


fig, _axs = plt.subplots(nrows=2, ncols=1)
axs = _axs.flatten()

#ax = plt.subplot(231)
ax = axs[0]
ax.set_title('Left Leg X error distribution before calib')
# plt.plot(origResX[:, 0], origResX[:, 1], 1)
# cset1 = ax.contourf(newX, newY, newZ, levels, norm=norm,
#                      cmap=cm.get_cmap(cmap, len(levels) - 1))
# for elem in newZ:
#     print(elem)
# c = ax.pcolor(newX, newY, newZ, cmap='RdBu', vmin=0, vmax=abs(newZ).max())
#ax.axis([x.min(), x.max(), y.min(), y.max()])
# fig.colorbar(c, ax=ax)
doHist(ax, fig, origResX, origResY)

#ax = plt.subplot(234)
ax = axs[1]
# #ax.boxplot(origResX[:, 1])
ax.set_title('Left Leg X error distribution after calib')
doHist(ax, fig, calibResX, calibResY)

plt.show()
