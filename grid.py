# grid 
import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt


def create_grid(rows,cols):
# Create a grid with specified rows and columns and xtixks, yticks and labels
    fig, ax = plt.subplots()
    ax.set_xticks(np.arange(0, cols+1, 1))
    ax.set_yticks(np.arange(0, rows+1, 1))
    ax.set_xticklabels(np.arange(0, cols+1, 1))
    ax.set_yticklabels(np.arange(0, rows+1, 1)) 
    ax.grid(True)

    #labels
    plt.xlabel('X-axis')
    plt.ylabel('Y-axis')
    plt.title("Robot Grid Navigation")
    



create_grid(5,5)
plt.show()

