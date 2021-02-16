import csv
import numpy as np
import matplotlib.pyplot as plt
import EncodingSchemes as ES
import DecodingSchemes as DS
import FunctionGenerator as FG


if __name__ == '__main__':
    dt = 0.01
    time = np.arange(0, 1, dt)

    myInput = FG.sum_of_sine_waves([2, -0.5, 0.75], [1.0, 3.0, 5.0], [0.0, 0.0, 0.0], 0.0, time)
    mySpikes, myThreshold = ES.temporalcontrast(myInput, 1.005)
    mySignal = 2*DS.temporal_contrast(mySpikes, myThreshold)

    plt.subplot(2, 1, 1)
    plt.plot(time, myInput)
    plt.plot(time, mySignal)
    plt.subplot(2, 1, 2)
    plt.stem(time, mySpikes)
    plt.show()

