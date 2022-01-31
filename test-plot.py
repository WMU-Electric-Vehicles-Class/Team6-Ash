x = np.arange(1,11)
y = (-2 * x**2 + 250)/250

plt.title("I Can Make Plots in Python")
plt.xlabel("Distance (mi)")
plt.ylabel("Electric Vehicle Battery State of Charge (%)")
plt.plot(x,y)
plt.show()