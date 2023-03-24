import numpy as np
from scps.ikpykinematics import Kinematics
from scps.quattorot import quaternion_rotation_matrix
from scipy.spatial.transform import Rotation as R
import rofunc as rf
import os
kin = Kinematics()

def traj_convert(quat):

    """
    Returns a trajectory list in the joint space

    input: List of Quaternions
    Returns: List of trajectory joint values
    """

    joint_list = []

    for i in range(len(quat)):

        """Returns the trajectory in the joint space for each time step and stores them in a list"""
        if i == 0:
            pos = quat[i,:3]
            rot = quat[i, 3:]
            rot[0] = quat[i, 6]
            rot[1:] = quat[i, 3:6]
            rotation = quaternion_rotation_matrix(rot)
            traj_trans = np.array([[*rotation[0,:], pos[0]],
                                   [*rotation[1,:], pos[1]],
                                   [*rotation[2,:], pos[2]],
                                   [0, 0, 0, 1]])

            traj_joint = kin.ik(traj_trans, initial_position=None, orientation=None)
            joint_list.append(traj_joint)

        else:
            pos = quat[i, :3]
            rot = quat[i, 3:]

            rot[0] = quat[i, 6]
            rot[1:] = quat[i, 3:6]

            rotationp = R.from_quat(rot)
            rotation = rotationp.as_matrix()

            traj_trans = np.array([[*rotation[0, :], pos[0]],
                                   [*rotation[1, :], pos[1]],
                                   [*rotation[2, :], pos[2]],
                                   [0, 0, 0, 1]])

            traj_joint = kin.ik(traj_trans, initial_position=joint_list[i-1], orientation=None)
            joint_list.append(traj_joint)

        i = i + 1

    return joint_list

if __name__ == "__main__":

    m = np.load('/home/clover/Github/Rofunc/rofunc/data/taichi_1l.npy')
    p = traj_convert(m)
    print(p[1][1:])