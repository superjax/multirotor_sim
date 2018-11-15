#!/usr/bin/env python

import os
import numpy as np
import matplotlib.pyplot as plt

def main():
    ODOM_VAL_PER_ROW = 14
    COMMAND_VAL_PER_ROW = 5

    logDir = os.path.dirname(os.path.realpath(__file__)) + '/../logs/'
    print("looking in {} for logs".format(logDir))

    # get data and shape it
    rawOdom = np.fromfile(logDir + 'odom.bin', float, -1, "")
    rawOdom = np.reshape(rawOdom, (-1, ODOM_VAL_PER_ROW))

    rawCommand = np.fromfile(logDir + 'command.bin', float, -1, "")
    rawCommand = np.reshape(rawCommand, (-1, COMMAND_VAL_PER_ROW))

    # Command column format: time, thrust, x, y, z
    commandYLabels = ['Command: Thrust', 'Command: X', 'Command: Y', 'Command: Z']
    colors  = ['b-', 'r-', 'y-', 'g-']

    plt.figure('Command Out')
    for i in range(1,COMMAND_VAL_PER_ROW):
        plt.subplot(2,2,i)
        plt.plot(rawCommand[:,0], rawCommand[:,i], colors[i-1])
        plt.ylabel(commandYLabels[i-1])


    odomPoseLabels = ['Estimator: X', 'Estimator: Y', 'Estimator: Z']
    plt.figure("Estimator Pose In")
    for i in range(1, 4):
        plt.subplot(3,1,i)
        plt.plot(rawOdom[:,0], rawOdom[:,i], colors[i-1])
        plt.ylabel(odomPoseLabels[i-1])

    plt.show()


if __name__ == '__main__':
    main()