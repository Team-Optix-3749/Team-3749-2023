#!/usr/bin/python3

import pandas as pd
import matplotlib.pyplot as plt

plt.rcParams["figure.figsize"] = [7.50, 3.50]
plt.rcParams["figure.autolayout"] = True

headers = ['x', 'y']

df = pd.read_csv('data.csv', names=headers)

df.set_index('x').plot()

plt.show()