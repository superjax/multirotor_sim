#!/usr/bin/env python

import os
import numpy as np
# from tqdm import tqdm # progress bar
# import matplotlib.pyplot as plt

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

    # Formatted Time, Thrust, X, Y, Z for clmns
    print(rawCommand[0:4,:])

if __name__ == '__main__':
    main()