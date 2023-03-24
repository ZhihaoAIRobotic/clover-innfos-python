import numpy as np

def jrange(ve):
    r = []

    if abs(ve[0]) > 0:
        if ve[0] < 0:
            sgn = -1
        if ve[0] > 0:
            sgn = 1
        x = np.array([sgn, 0, 0, 0, 0, 0]).reshape(6, 1)
        r.append(np.hstack(x))

    if abs(ve[1]) > 0:
        if ve[1] < 0:
            sgn = -1
        if ve[1] > 0:
            sgn = 1
        y = np.array([0, sgn, 0, 0, 0, 0]).reshape(6, 1)
        r.append(np.hstack(y))

    if abs(ve[2]) > 0:
        if ve[2] < 0:
            sgn = -1
        if ve[2] > 0:
            sgn = 1
        z = np.array([0, 0, sgn, 0, 0, 0]).reshape(6, 1)
        r.append(np.hstack(z))


    if abs(ve[3]) > 0:
        if ve[3] < 0:
            sgn = -1
        if ve[3] > 0:
            sgn = 1
        roll = np.array([0, 0, 0, sgn, 0, 0]).reshape(6, 1)
        r.append(np.hstack(roll))

    if abs(ve[4]) > 0:
        if ve[4] < 0:
            sgn = -1
        if ve[4] > 0:
            sgn = 1
        pitch = np.array([0, 0, 0, 0, sgn, 0]).reshape(6, 1)
        r.append(np.hstack(pitch))

    if abs(ve[5]) > 0:
        if ve[5] < 0:
            sgn = -1
        if ve[5] > 0:
            sgn = 1
        yaw = np.array([0, 0, 0, 0, 0, sgn]).reshape(6, 1)
        r.append(np.hstack(yaw))

    i = 0
    for row in r:
        i = i + 1


    return np.array(r).T, i