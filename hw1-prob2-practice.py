#Homework 1 Problem 2

## Plotting a sine curve
import matplotlib.pyplot as plt
import numpy as np

x = np.arange(0,4*np.pi,0.1)
y = np.sin(x)

plt.title("Sine Curve")
plt.xlabel("Time")
plt.ylabel("Amplitude")
plt.plot(x,y)
plt.show()