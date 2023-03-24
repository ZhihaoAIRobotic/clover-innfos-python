import numpy as np
from math import atan2, pow, sqrt

def euler(T):
    rpy = np.zeros(3)
    rpy[0] = atan2(T[2, 1], T[2, 2])
    rpy[1] = atan2(-T[2, 0], sqrt(pow(T[2, 1], 2) + pow(T[2, 2], 2)))
    rpy[2] = atan2(T[1, 0], T[0, 0])

    return rpy

def p(T):

    pose = np.zeros(3)
    pose[:] = T[0:3, 3]

    return pose

if __name__ == "__main__":
    T = np.array([[-1, 2.095e-16, 0, -0.236],
                  [0, 2.11e-16, 1, 0.09705],
                  [2.088e-16, 1, 0, 0.2656],
                  [0, 0, 0, 1]])

    print(euler(T))
    print(p(T))


