# This file contains the code for concatenating the bimanual trajectories for mujoco simulations

import numpy as np


def get_cof(milk_pos, traj2):
    """
    Input Parameters:
    milk_pos = static position of the milk pitcher
    traj1 = trajectory of the coffee end-effector
    """

    traj = np.hstack([*traj2, *milk_pos])

    return traj


def get_milk(traj1, cof_pos):
    """
    Input Parameters:
    traj2 = trajectory of the milk pitcher end-effector
    cof_pos = static position of the coffee arm
    """

    traj = np.hstack([*traj1, *cof_pos])

    return traj


def get_bimanual(traj1, traj2):
    """
    Input Parameters:
    traj1 = trajectory of the coffee end-effector
    traj2 = trajectory of the milk pitcher end-effector
    """
    if len(traj1) == len(traj2):
        traj = np.hstack([*traj1, *traj2])

    if len(traj1) > len(traj2):
        traj = np.zeros([len(traj1), 12])
        traj[:len(traj2), :] = np.hstack([traj1[:len(traj2), :], traj2])
        traj[len(traj2):, :] = np.hstack([traj1[len(traj2):, :], traj2[-1]])

    if len(traj2) > len(traj1):
        traj = np.zeros([len(traj2), 12])
        traj[:len(traj1), :] = np.hstack([traj1, traj2[:len(traj1), :]])
        traj[len(traj1):, :] = np.hstack([traj1[-1], traj2[len(traj1):, :]])

    return traj
