import matplotlib.pyplot as plt
import numpy as np


fig, ax = plt.subplots(2, 2)

with open("numbers.txt") as f:
    line = f.readline()
    numbers = list(map(int, line.split(',')))
    y_axis = range(0, len(numbers))

    ax[0, 0].plot(np.array(y_axis), np.array(numbers))
    ax[0, 0].set_title("Number of detected tomatoes")


with open("val.txt") as f:
    line = f.readline()
    numbers = list(map(float, line.split(',')))
    y_axis = range(0, len(numbers))

    ax[0, 1].plot(np.array(y_axis), np.array(numbers))
    ax[0, 1].set_title("Distance to center")

with open("index.txt") as f:
    line = f.readline()
    numbers = list(map(float, line.split(',')))
    y_axis = range(0, len(numbers))

    ax[1, 0].plot(np.array(y_axis), np.array(numbers))
    ax[1, 0].set_title("Index")

with open("pos.txt") as f:
    line = f.readline()
    numbers = list(map(float, line.split(',')))
    y_axis = range(0, len(numbers))

    ax[1, 1].plot(np.array(y_axis), np.array(numbers))
    ax[1, 1].set_title("Position")
plt.show()
