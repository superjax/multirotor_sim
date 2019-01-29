from plotWindow import plotWindow
import numpy as np
import matplotlib.pyplot as plt

data = np.reshape(np.fromfile('/tmp/ceres_sandbox/Dynamics.Propagate.log', dtype=np.float64), (-1, 27)).T
t = data[0,:]
rk4 = data[1:14,:]
euler = data[14:,:]


pw = plotWindow()

f = plt.figure()
legend = ['x', 'y', 'z']
for i in range(3):
    plt.subplot(3, 1, i+1)
    plt.plot(t, rk4[i, :], label="rk4")
    plt.plot(t, euler[i, :], label="euler")
    plt.title(legend[i])
    plt.legend()
pw.addPlot("position", f)

f = plt.figure()
legend = ['w', 'x', 'y', 'z']
for i in range(4):
    plt.subplot(4, 1, i+1)
    plt.plot(t, rk4[i+3, :], label="rk4")
    plt.plot(t, euler[i+3, :], label="euler")
    plt.title(legend[i])
    plt.legend()
pw.addPlot("attitude", f)

f = plt.figure()
legend = ['x', 'y', 'z']
for i in range(3):
    plt.subplot(3, 1, i+1)
    plt.plot(t, rk4[i+7, :], label="rk4")
    plt.plot(t, euler[i+7, :], label="euler")
    plt.title(legend[i])
    plt.legend()
pw.addPlot("velocity", f)

f = plt.figure()
legend = ['x', 'y', 'z']
for i in range(3):
    plt.subplot(3, 1, i+1)
    plt.plot(t, rk4[i+10, :], label="rk4")
    plt.plot(t, euler[i+10, :], label="euler")
    plt.title(legend[i])
    plt.legend()
pw.addPlot("omega", f)


pw.show()
