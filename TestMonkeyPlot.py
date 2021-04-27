# -*- coding: utf-8 -*-
"""
Created on Mon Apr 26 20:24:35 2021

@author: klund
"""

import pandas as pd
import matplotlib.pyplot as plt
import scipy.stats import linregress

dataframe = pd.read_csv("scottish_hills.csv")
x = dataframe.Height
y = datafram.Latitude

stats = linregress(x, y)

m = stats.slope
b = stats.intercept

# Changing default figure size
plt.figure(figsize=(10, 10))

# Change default marker for scatter from circles to x's
plt.scatter(x, y, market = 'x')

# Set the linewidth on the regression line to 3 px
plt.plot(x, m * x + b, color = "red", linewidth = 3)

# Add x and y lables, and set their font size
plt.xlabel("Time (s)", fontsize = 20)
plt.ylabel("Voltage (s)", fontsize = 20)

# Set the font size of the number lables on the axes
plt.xticks(fontsize = 18)
plt.yticks(fontsize = 18)


plt.scatter(x, y)
plt.plot(x, m * x + b, color="red")

plt.savefig("serial_datastream.png")
