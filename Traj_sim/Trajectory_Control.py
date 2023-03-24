import numpy as np
from scps.ikpykinematics import Kinematics
import matplotlib.pyplot as plt
from math import atan2, sqrt


class Trajectory_control(object):

    def __init__(self, desired_trajectory):
        '''
        :param desired_trajectory: The chosen trajectory such as : Circle, Line, Curve
        '''
        # This script will simulate and plot the trajectory
        self.dsr_traj = desired_trajectory
    def control(self):

        # initialize the kinematics
        kin = Kinematics()

        # Inverse Kinematics in trajectories uses the joint values of the previous time step as the initial guess
        # initialize the estimation dictionary for the initial guesses that will be put into the inverse kinematics
        estimation = []

        # initialize the joint trajectory dictionary
        self.joint_trajectory = []

        # initialize the joint trajectory
        Transform_initial = ([1, 0, 0, 0.0792],
                             [0, 1, 0, -2.2696000e-05],
                             [0, 0, 1, 5.3397900e-01],
                             [0, 0, 0, 1])

        # Calculate the initial joint values
        joint_value_initial = kin.ik(Transform_initial, initial_position=None, orientation=self.dsr_traj.orientation)

        # Record the initial estimation into the dictionary
        estimation = joint_value_initial
        self.rotation_traj = []

        # Record the initial joint values into the dictionary
        self.joint_trajectory[0] = joint_value_initial

        i = 0

        for i in np.arange(1, (self.dsr_traj.timespan)).reshape(-1):

            # Initialize the positions derived from the desired trajectory
            self.point_in_trajectory = self.dsr_traj.coordinate[i]

            # Initialize the orientation derived from the desired trajectory

            # intialize the rotation matrix from the desired trajectory
            self.rot = np.array(self.dsr_traj.rotation_traj[i])

            # Build the transformation matrix from the desired trajectory
            Transform = ([[self.rot[0, 0], self.rot[0, 1], self.rot[0, 2], self.point_in_trajectory[0]],
                              [self.rot[1, 0], self.rot[1, 1], self.rot[1, 2], self.point_in_trajectory[1]],
                              [self.rot[2, 0], self.rot[2, 1], self.rot[2, 2], self.point_in_trajectory[2]],
                              [0, 0, 0, 1]])

            # Calculate the joint values, note that the estimation dictionary is used here
            joint_value = kin.ik(Transform, estimation[i - 1], self.dsr_traj.orientation)

            # Record the current joint value to be used as estimation for the next timestep
            estimation.append(joint_value)

            joint_actual = joint_value_initial

            # Record the current joint value to the joint trajectory dictionary
            self.joint_trajectory.append(joint_actual)



            i = i+1


        return self.joint_trajectory




if __name__ == '__main__':

    from TrajectoryCircle import TrajectoryCircle
    import os

    traj = TrajectoryCircle(150, x_axis=False, y_axis=True, z_axis=False, orientation="all")
    real_tran = traj.get_instant_circle(0.15)
    real_rot = traj.get_instant_rotation()
    traj_control = Trajectory_control(traj)
    print(traj_control)