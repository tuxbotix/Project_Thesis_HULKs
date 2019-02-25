import numpy as np
from matplotlib.patches import Circle, Wedge, Polygon
from matplotlib.collections import PatchCollection
import matplotlib.pyplot as plt

# Fixing random state for reproducibility
np.random.seed(19680801)

file = open("/tmp/rois.txt")
lines = [line.rstrip('\n').split() for line in file]

fig, ax = plt.subplots()

# resolution = 50  # the number of vertices
N = 3
# x = np.random.rand(N)
# y = np.random.rand(N)
# radii = 0.1*np.random.rand(N)

# for x1, y1, r in zip(x, y, radii):
#     circle = Circle((x1, y1), r)
#     patches.append(circle)

# x = np.random.rand(N)
# y = np.random.rand(N)
# radii = 0.1*np.random.rand(N)
# theta1 = 360.0*np.random.rand(N)
# theta2 = 360.0*np.random.rand(N)
# for x1, y1, r, t1, t2 in zip(x, y, radii, theta1, theta2):
#     wedge = Wedge((x1, y1), r, t1, t2)
#     patches.append(wedge)

# # Some limiting conditions on Wedge
# patches += [
#     Wedge((.3, .7), .1, 0, 360),             # Full circle
#     Wedge((.7, .8), .2, 0, 360, width=0.05),  # Full ring
#     Wedge((.8, .3), .2, 0, 45),              # Full sector
#     Wedge((.8, .3), .2, 45, 90, width=0.10),  # Ring sector
# ]

# for i in range(N):
patches = []
minMaxList = []

# len([p for p in PeopleList if p.Gender == 'F'])
# bottom will be first
lines.sort(key=lambda line: line[1] == 'top')

bottomCount = 0
for line in lines:

    # if line[1] == "top":
        # continue
    # NOTE X-Y are swapped to match with Nao's viewpoint
    xy = np.array([[-float(line[idx+1]), float(line[idx])]
                   for idx in range(2, 10, 2)])

    minMaxList.append(np.amin(xy, axis=0))
    minMaxList.append(np.amax(xy, axis=0))

    polygon = Polygon(xy, True)
    if line[1] != "top":
        bottomCount += 1
    patches.append(polygon)

minMaxList = [np.amin(minMaxList, axis=0), np.amax(minMaxList, axis=0)]

bottomColors = np.random.uniform(0, 45, bottomCount)
topColors = np.random.uniform(55, 100, len(lines) - bottomCount)

colours=np.concatenate((bottomColors, topColors))
print(bottomColors, topColors, colours)

p=PatchCollection(patches, alpha = 0.4)
p.set_array(np.array(colours))

ax.invert_yaxis()

ax.set_xlim([-4, 4])
ax.set_ylim([-4, 4])
ax.grid()
ax.add_collection(p)


fig.colorbar(p, ax = ax)

plt.show()
