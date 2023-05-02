import pinocchio as pin
import numpy as np

def gravity(q):
    """
    This function takes in a joint configuration and returns the gravity vector

    Input
    :param q: The joint configuration

    Output
    :return: The gravity vector
    """

    # Create a model
    model = pin.buildModelFromUrdf("/home/ubuntu/Github/clover-innfos-python/Urdf/gluon.urdf")

    # Create a data object
    data = model.createData()

    # Compute the gravity vector
    g = pin.computeGeneralizedGravity(model, data, q)

    return g

def inertia(q):
    """
    This function takes in a joint configuration and returns the inertia matrix

    Input
    :param q: The joint configuration

    Output
    :return: The inertia matrix
    """

    # Create a model
    model = pin.buildModelFromUrdf("/home/ubuntu/Github/clover-innfos-python/Urdf/gluon.urdf")

    # Create a data object
    data = model.createData()

    # Compute the inertia matrix
    H = pin.computeMinverse(model, data, q)

    return H

def centrifugalterms(q, qd):

    # Create a model
    model = pin.buildModelFromUrdf("/home/ubuntu/Github/clover-innfos-python/Urdf/gluon.urdf")

    # Create a data object
    data = model.createData()

    # Compute the centrifugal terms
    C = pin.computeCoriolisMatrix(model, data, q, qd)

    return C

def array_clean_print(sep=' ', vert='|', pad=10, precision=4):
    def prettyprint(a):
        s = "\n"
        if len(a.shape) == 1:
            a = [a]  # Make array appear 2D if it's 1D
        for row in a:
            s += vert + sep + sep.join([f"{x: ^ {pad}.{precision}g}" for x in row]) + vert + "\n"
        return s

    return prettyprint


np.set_string_function(array_clean_print(), repr=False)
np.set_string_function(array_clean_print(), repr=True)

if __name__ == '__main__':
    q = np.array([0, np.pi/2, -np.pi/2, 0, 0, 0])
    g = gravity(q)
    print(g)

    print("..." * 60)

    H = inertia(q)
    print(H)

    print("..." * 60)

    dq = np.array([0, 0, 1, 0, 0, 0])

    I = centrifugalterms(q, dq)
    print(I)

    print("..." * 60)



