import numpy as np
import matplotlib.pyplot as plt

from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm

plt.rcParams['figure.figsize'] = [8,8]      # This line sets the size of the figures (you can change the numbers)

KP_POS = 0.1 
KP_NEG = 1

goal = np.array( [40,100] )                   # This is where we want to go 
pmap = np.zeros( [100,100] )                  # This is where we will store the potential functions

obstacles = [
    [20,40],
    [20,55],
    [30,55],
    [40,55],
    [50,10],
    [60,55],
    [70,55],
    [70,40],
]

def pos_potential(ori, pos) :
    dist2 = np.inner( (ori - pos)/20, (ori - pos)/20 )
    return KP_POS*dist2

def neg_potential(ori, pos) :
    dist2 = np.inner( (ori - pos)/4, (ori - pos)/4 )
    if dist2 < 1 :
        return KP_NEG
    return KP_NEG*(1/dist2)

X = np.arange(0, 100, 1)
Y = np.arange(0, 100, 1)
X, Y = np.meshgrid(X, Y)

for x in range(100) :
    for y in range(100) :
        pmap[x][y] += pos_potential(goal, np.array([x,y]))
        for o in obstacles :
            pmap[x][y] += neg_potential(np.array(o), np.array([x,y]))

            
plt.matshow(pmap)
plt.show()
# Define the figure
fig = plt.figure()
ax = fig.gca(projection='3d')

# Plot the surface.
surf = ax.plot_surface(X, Y, pmap, cmap=cm.coolwarm, linewidth=0, 
antialiased=False)

# Add a color bar which maps values to colors.
fig.colorbar(surf, shrink=0.5, aspect=5)

plt.show()
