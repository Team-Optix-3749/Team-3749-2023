#!/usr/bin/python3

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection

plt.rcParams["figure.figsize"] = [7.50, 3.50]
plt.rcParams["figure.autolayout"] = True

headers = ['x', 'y']

df = pd.read_csv('data.csv', names=headers)

x = df.set_index('y')
y = df.set_index('x')
cols = np.linspace(0,1,len(x))

points = np.array([x, y]).T.reshape(-1, 1, 2)
segments = np.concatenate([points[:-1], points[1:]], axis=1)

fig, ax = plt.subplots()
lc = LineCollection(segments, cmap='viridis')
lc.set_array(cols)
lc.set_linewidth(2)
line = ax.add_collection(lc)
fig.colorbar(line,ax=ax)

plt.title("Arm Trajectory Visualization")
plt.xlabel("X Position (m)")
plt.ylabel("Y Position (m)")

plt.xticks(np.arange(min(x.to_numpy()), max(x.to_numpy())+1, 0.1))
plt.yticks(np.arange(min(y.to_numpy()), max(y.to_numpy())+1, 0.1))

plt.scatter(x, y, marker='')

plt.show()