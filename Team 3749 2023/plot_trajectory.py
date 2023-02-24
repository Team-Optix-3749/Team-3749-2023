#!/usr/bin/python3

# imports
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection

# set plot sizing configs
plt.rcParams["figure.figsize"] = [7.50, 3.50]
plt.rcParams["figure.autolayout"] = True

# set csv headers
headers = ['x', 'y']

# read data.csv using pandas
df = pd.read_csv('data.csv', names=headers)

# set axis
x = df.set_index('y')
y = df.set_index('x')
cols = np.linspace(0,1,len(x))

# reshape points for gradient
points = np.array([x, y]).T.reshape(-1, 1, 2)
segments = np.concatenate([points[:-1], points[1:]], axis=1)

# add line color gradient
fig, ax = plt.subplots()
lc = LineCollection(segments, cmap='viridis')
lc.set_array(cols)
lc.set_linewidth(2)
line = ax.add_collection(lc)
fig.colorbar(line,ax=ax)

# waypoints for scoring nodes
x_scoring_locations = [0.9398, 1.4478]
y_scoring_locations = [0.8636 - 0.5588, 1.1938 - 0.5588]

# graph scoring nodes as bar graph
ax.bar(x_scoring_locations, y_scoring_locations, width=0.025)

# set plot axis titles
plt.title("Arm Trajectory Visualization")
plt.xlabel("X Position (m)")
plt.ylabel("Y Position (m)")

# increase axis ticks
plt.xticks(np.arange(min(x.to_numpy()), max(x.to_numpy())+1, 0.1))
plt.yticks(np.arange(min(y.to_numpy()), max(y.to_numpy())+1, 0.1))

# highlight start and end points
plt.plot(x.to_numpy()[0], y.to_numpy()[0], 'r.')
plt.plot(x.to_numpy()[-1], y.to_numpy()[-1], 'r.')

# set plot type to scatter
plt.scatter(x, y, marker='')

# show plot :100:
plt.show()