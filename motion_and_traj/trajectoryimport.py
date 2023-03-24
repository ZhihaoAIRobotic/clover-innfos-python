import numpy as np
kin = scps.ikpykinematics.Kinematics()

def traj_import(traj_trans):

    """
    Returns a trajectory list in the joint space

    input: List of trajectory transformation matrices
    Returns: List of trajectory joint values
    """

    joint_list = []

    for i in len(traj_trans):

        """Returns the trajectory in the joint space for each time step and stores them in a list"""

        traj_joint = kin.ik(traj_trans[i], initial_position=None, orientation="all")
        joint_list.append(traj_joint)

        i = i + 1

    return joint_list