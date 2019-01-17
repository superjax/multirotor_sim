from plotWindow import plotWindow
import numpy as np
import matplotlib.pyplot as plt

data = np.reshape(np.fromfile('../build/Dynamics.Propagate.log', dtype=np.float64), (-1, 14)).T
t = data[0,:]
x = data[1:14,:]

debug = 1

pw = plotWindow()

f = plt.figure()
legend = ['x', 'y', 'z']
for i in range(3):
    plt.subplot(3, 1, i+1)
    plt.plot(t, x[i, :], label="x")
    plt.title(legend[i])
    plt.legend()
pw.addPlot("position", f)

f = plt.figure()
legend = ['w', 'x', 'y', 'z']
for i in range(4):
    plt.subplot(4, 1, i+1)
    plt.plot(t, x[i+3, :], label="x")
    plt.title(legend[i])
    plt.legend()
pw.addPlot("attitude", f)

f = plt.figure()
legend = ['x', 'y', 'z']
for i in range(3):
    plt.subplot(3, 1, i+1)
    plt.plot(t, x[i+7, :], label="x")
    plt.title(legend[i])
    plt.legend()
pw.addPlot("velocity", f)

f = plt.figure()
legend = ['x', 'y', 'z']
for i in range(3):
    plt.subplot(3, 1, i+1)
    plt.plot(t, x[i+10, :], label="x")
    plt.title(legend[i])
    plt.legend()
pw.addPlot("omega", f)


pw.show()