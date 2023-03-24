import numpy as np
import mujoco_viewer
from mujoco_py import MjSim, load_model_from_path
import mujoco
import ikpyKinematics
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
        kin = ikpyKinematics.Kinematics()

        # Inverse Kinematics in trajectories uses the joint values of the previous time step as the initial guess
        # initialize the estimation dictionary for the initial guesses that will be put into the inverse kinematics
        estimation = {}

        # initialize the joint trajectory dictionary
        self.joint_trajectory = {}

        # initialize the joint trajectory
        Transform_initial = ([1, 0, 0, 0.0792],
                             [0, 1, 0, -2.2696000e-05],
                             [0, 0, 1, 5.3397900e-01],
                             [0, 0, 0, 1])

        # Calculate the initial joint values
        joint_value_initial = kin.ik(Transform_initial, initial_position=None, orientation=self.dsr_traj.orientation)

        # Record the initial estimation into the dictionary
        estimation[0] = joint_value_initial
        self.rotation_traj = {}

        # Record the initial joint values into the dictionary
        self.joint_trajectory[0] = joint_value_initial[1:7]

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
            estimation[i] = joint_value

            joint_actual = [joint_value[1:7]]

            # Record the current joint value to the joint trajectory dictionary
            self.joint_trajectory[i] = joint_actual

            i = i+1

        # print(self.joint_trajectory)

        return self.joint_trajectory

    def simulation(self, model, data):

        # initialize the simulation
        viewer = mujoco_viewer.MujocoViewer(model, data)

        # the joint trajectory is used to control the robot trajectory
        while True:
            for i in range(1, len(self.joint_trajectory)):
                data.qpos[:] = np.array(self.joint_trajectory[i])
                # print(data.qpos)
                mujoco.mj_step(model, data)
                viewer.render()

    def control_points(self, no_of_points):

        points = {}

        points[0] = self.joint_trajectory[0]
        points[1] = self.joint_trajectory[25]

        for i in range(2, no_of_points):
            points[i] = self.joint_trajectory[self.dsr_traj.timespan/5 + int((i/(no_of_points-2))*3*self.dsr_traj.timespan/5)]

        points[no_of_points] = self.joint_trajectory[self.dsr_traj.timespan-1]

        print(points)

        return points

    def plot(self, all, x_plot, y_plot, z_plot, roll_plot, pitch_plot, yaw_plot):

        self.all = all
        self.x_plot = x_plot
        self.y_plot = y_plot
        self.z_plot = z_plot
        self.roll_plot = roll_plot
        self.pitch_plot = pitch_plot
        self.yaw_plot = yaw_plot

        x = np.zeros(self.dsr_traj.timespan)
        y = np.zeros(self.dsr_traj.timespan)
        z = np.zeros(self.dsr_traj.timespan)
        roll = np.zeros(self.dsr_traj.timespan)
        pitch = np.zeros(self.dsr_traj.timespan)
        yaw = np.zeros(self.dsr_traj.timespan)

        for i in range(0, len(self.joint_trajectory)):
            # print(self.joint_trajectory[i])
            position = self.dsr_traj.coordinate[i]
            x[i] = position[0]
            y[i] = position[1]
            z[i] = position[2]

            rotation = self.dsr_traj.rotation_traj[i]

            roll[i] = atan2(rotation[2, 1], rotation[2, 2])
            pitch[i] = atan2(-rotation[2, 0], sqrt(rotation[2, 1] ** 2 + rotation[2, 2] ** 2))
            yaw[i] = atan2(rotation[1, 0], rotation[0, 0])


        if self.all is True:

            self.x_plot = True
            self.y_plot = True
            self.z_plot = True
            self.roll_plot = True
            self.pitch_plot = True
            self.yaw_plot = True

        if self.x_plot is True:
            plt.title('x coordinate trajectory')
            plt.ylabel('x coordinate')
            plt.xlabel('Time')
            plt.plot(x)
        plt.show()

        if self.y_plot is True:
            plt.plot(y)
            plt.title('y coordinate trajectory')
            plt.ylabel('y coordinate')
            plt.xlabel('Time')
        plt.show()

        if self.z_plot is True:
            plt.plot(z)
            plt.title('z coordinate trajectory')
            plt.ylabel('z coordinate')
            plt.xlabel('Time')
        plt.show()

        if self.roll_plot is True:
            plt.plot(roll)
            plt.title('roll coordinate trajectory')
            plt.ylabel('roll coordinate')
            plt.xlabel('Time')
        plt.show()

        if self.pitch_plot is True:
            plt.plot(pitch)
            plt.title('pitch coordinate trajectory')
            plt.ylabel('pitch coordinate')
            plt.xlabel('Time')
        plt.show()

        if self.yaw_plot is True:
            plt.plot(yaw)
            plt.title('yaw coordinate trajectory')
            plt.ylabel('yaw coordinate')
            plt.xlabel('Time')
        plt.show()




if __name__ == '__main__':

    from TrajectoryStraightLine import TrajectoryStraightLine
    from TrajectoryCircle import TrajectoryCircle
    from TrajectoryCurve import TrajectoryCurve
    import os

    ASSETS = {}
    root_path = '/home/ubuntu/Github/Robase/environment models/meshes'
    files = os.listdir(root_path)
    for file in files:
        f = open(os.path.join(root_path, file), 'rb')
        num = file.split('_')[0]
        ASSETS['{}_Link.stl'.format(num)] = f.read()

    model = mujoco.MjModel.from_xml_path("/home/ubuntu/Github/Robase/environment models/gluon_robot.xml", ASSETS)

    data = mujoco.MjData(model)

    Target = np.array([-0.19048943824796497, 1.1273486972839692, -0.9188716579218693, -0.47542402841030373, -1.5707963267948837, -1.7612857650428848])
    rotation = ([0, 1, 0],
                [1, 0, 0],
                [0, 0, 1])
    traj = TrajectoryCircle(150, x_axis=False, y_axis=True, z_axis=False, orientation="all")
    real_tran = traj.get_instant_circle(0.15)
    real_rot = traj.get_instant_rotation()
    traj_control = Trajectory_control(traj)
    traj_control.control()
    traj_control.plot(all=True, x_plot=False, y_plot=False, z_plot=False, roll_plot=False, pitch_plot=False, yaw_plot=False)
    traj_control.control_points(10)
    traj_control.simulation(model, data)