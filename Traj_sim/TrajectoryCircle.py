import numpy as np
import ikpyKinematics

class TrajectoryCircle(object):
    def __init__(self, timespan, x_axis, y_axis, z_axis, orientation):

        self.xt, self.yt, self.zt, self.thetat, self.phit, self.psit = 7.5685673e-02, -2.2696000e-05, 5.3397900e-01, 0, 0, 0
        self.timespan = timespan
        self.initial_orientation = np.array(([1, 0, 0],
                                             [0, 1, 0],
                                             [0, 0, 1]))

        # Allow for rotation about multiple axis
        self.x_axis = x_axis
        self.y_axis = y_axis
        self.z_axis = z_axis

        # Allow for consideration regarding the rotation of the end effector
        self.orientation = orientation

    def get_instant_circle(self, radius):

        # initialize the kinematics library
        kin = ikpyKinematics.Kinematics()

        # Calculate time step, -2 is to consider the time it takes to reach from initial to the start point of the circle
        self.angle = (2*np.pi)/((3*self.timespan)/5)

        # Calculate the coordinates of the end-effector at every time step within the circle of radius
        self.radius = radius

        # Initialize the coordinate dictionary
        self.coordinate = {}

        if self.x_axis is True:

            # move to starting position of circle
            # starting position is x = 0.2, y = 0, z = 0.3 + self.radius

            self.coordinate[0] = np.array([7.5685673e-02, -2.2696000e-05, 5.3397900e-01])

            # self.xt = 0.2
            # self.yt = self.radius*np.sin(0)
            # self.zt = self.radius*np.cos(0) + 0.2

            i = 0

            for i in np.arange(1, self.timespan/5):

                # Trajectory to the start point of the circle

                self.xt = self.xt - ((7.5685673e-02 - 0.2)/(self.timespan/5))
                self.yt = self.yt + ((2.2696000e-05 - 0)/(self.timespan/5))
                self.zt = self.zt - ((5.3397900e-01 - (0.3 + self.radius))/(self.timespan/5))

                self.coordinate[i] = np.array([self.xt, self.yt, self.zt])

                i = i + 1

            while self.timespan/5 <= i <= (4*self.timespan)/5:

                # Trajectory of the circle

                self.xt = 0.2
                self.yt = self.radius*np.sin(self.angle*(i-self.timespan/5))
                self.zt = self.radius*np.cos(self.angle*(i-self.timespan/5)) + 0.3

                traj_t = np.array([self.xt, self.yt, self.zt])

                self.coordinate[i] = traj_t

                i = i + 1

            for i in np.arange((4*self.timespan)/5, self.timespan):

                # Trajectory to the origin from the end point of the circle

                self.xt = self.xt + ((7.5685673e-02 - 0.2)/(self.timespan/5))
                self.yt = self.yt - ((2.2696000e-05 - 0)/(self.timespan/5))
                self.zt = self.zt + ((5.3397900e-01 - (0.3 + self.radius))/(self.timespan/5))

                i = i + 1

                self.coordinate[i] = np.array([self.xt, self.yt, self.zt])

                print(self.coordinate)

            return self.coordinate

        if self.y_axis is True:

            # Initial pose of the robot
            self.coordinate[0] = np.array([7.5685673e-02, -2.2696000e-05, 5.3397900e-01])

            i = 0

            for i in np.arange(1, self.timespan/5):

                # Trajectory to the start point of the circle

                self.xt = self.xt - ((7.5685673e-02 - 0) / (self.timespan / 5))
                self.yt = self.yt - ((-2.2696000e-05 - 0.2) / (self.timespan / 5))
                self.zt = self.zt - ((5.3397900e-01 - (0.3 + self.radius)) / (self.timespan / 5))

                self.coordinate[i] = np.array([self.xt, self.yt, self.zt])

                i = i + 1

            while self.timespan/5 <= i <= (4/5)*self.timespan:

                # Trajectory of the circle

                self.xt = self.radius*np.sin(self.angle*(i-self.timespan/5))
                self.yt = 0.2
                self.zt = self.radius*np.cos(self.angle*(i-self.timespan/5)) + 0.3

                traj_t = np.array([self.xt, self.yt, self.zt])

                self.coordinate[i] = traj_t
                # print(self.coordinate)

                i = i + 1


            while (4/5)*self.timespan <= i <= self.timespan:

                # Trajectory to the origin from the end point of the circle

                self.xt = self.xt + ((7.5685673e-02 - 0) / (self.timespan / 5))
                self.yt = self.yt + ((-2.2696000e-05 - 0.2) / (self.timespan / 5))
                self.zt = self.zt + ((5.3397900e-01 - (0.3 + self.radius)) / (self.timespan / 5))

                self.coordinate[i] = np.array([self.xt, self.yt, self.zt])

                # print(self.coordinate)

                i = i + 1

            return self.coordinate

        if self.z_axis is True:

            # Initial pose of the robot

            self.coordinate[0] = np.array([7.5685673e-02, -2.2696000e-05, 5.3397900e-01, ])

            i = 0

            for i in np.arange(1, self.timespan/5):

                # Trajectory to the start point of the circle

                self.xt = self.xt - ((7.5685673e-02 - 0) / (self.timespan / 5))
                self.yt = self.yt - ((2.2696000e-05 - self.radius) / (self.timespan / 5))
                self.zt = self.zt - ((5.3397900e-01 - 0.4) / (self.timespan / 5))

                self.coordinate[i] = np.array([self.xt, self.yt, self.zt])

                i = i + 1

            while self.timespan/5 <= i <= (self.timespan * 4)/5:

                # Trajectory of the circle

                self.xt = self.radius*np.sin(self.angle*(i-self.timespan/5))
                self.yt = self.radius*np.cos(self.angle*(i-self.timespan/5))
                self.zt = 0.4

                traj_t = np.array([self.xt, self.yt, self.zt])

                self.coordinate[i] = traj_t

                i = i + 1

            for i in np.arange(((4*self.timespan)/5)+1, self.timespan+1):

                # Trajectory to the origin from the end point of the circle

                self.xt = self.xt + ((7.5685673e-02 - 0) / (self.timespan / 5))
                self.yt = self.yt + ((2.2696000e-05 - self.radius) / (self.timespan / 5))
                self.zt = self.zt + ((5.3397900e-01 - 0.4) / (self.timespan / 5))

                self.coordinate[i] = np.array([self.xt, self.yt, self.zt])

                print(self.coordinate)

                i = i + 1

            return self.coordinate


    def get_instant_rotation(self):

        # Initialize the rotation trajectory dictionary
        self.rotation_traj = {}

        initial_orientation = np.array(([1, 0, 0],
                                        [0, 1, 0],
                                        [0, 0, 1]))

        self.rotation_traj[0] = initial_orientation

        i = 0

        while i <= self.timespan:

            # Maintain the orientation of the robot

            self.rotation_traj[i] = initial_orientation

            i = i + 1

        return self.rotation_traj



        #     self.rotation_traj[0] = initial_orientation
        #
        #     self.r00 = 1
        #     self.r01 = 0
        #     self.r02 = 0
        #     self.r10 = 0
        #     self.r11 = 1
        #     self.r12 = 0
        #     self.r20 = 0
        #     self.r21 = 0
        #     self.r22 = 1
        #
        #     if self.x_axis is True:
        #
        #         self.rotation_traj[0] = np.array(([1, 0, 0],
        #                                           [0, 1, 0],
        #                                           [0, 0, 1]))
        #
        #         i = 0
        #
        #         for i in np.arange(1, self.timespan/5):
        #
        #             self.r00 = self.r00 - (1/(self.timespan/5))
        #             self.r01 = 0
        #             self.r02 = self.r02 + (1/(self.timespan/5))
        #             self.r10 = 0
        #             self.r11 = 1
        #             self.r12 = 0
        #             self.r20 = self.r20 + (1/(self.timespan/5))
        #             self.r21 = 0
        #             self.r22 = self.r22 - (1/(self.timespan/5))
        #
        #             self.rotation_traj[i] = ([self.r00, self.r01, self.r02],
        #                                      [self.r10, self.r11, self.r12],
        #                                      [self.r20, self.r21, self.r22])
        #
        #             i = i + 1
        #
        #         while self.timespan/5 <= i <= (self.timespan/5+(3*self.timespan)/20):
        #
        #             self.r00 = 0
        #             self.r01 = 0
        #             self.r02 = 1
        #             self.r10 = self.r10 + (1/(3*(self.timespan/20)))
        #             self.r11 = self.r11 - (1/(3*(self.timespan/20)))
        #             self.r12 = 0
        #             self.r20 = self.r20 - (1/(3*(self.timespan/20)))
        #             self.r21 = self.r21 - (1/(3*(self.timespan/20)))
        #             self.r22 = 0
        #
        #
        #             self.rotation_traj[i] = ([self.r00, self.r01, self.r02],
        #                                      [self.r10, self.r11, self.r12],
        #                                      [self.r20, self.r21, self.r22])
        #
        #             i = i+1
        #
        #         while (self.timespan/5+(3*self.timespan)/20) <= i <= (self.timespan/5+(6*self.timespan)/20):
        #
        #             self.r00 = 0
        #             self.r01 = 0
        #             self.r02 = 1
        #             self.r10 = self.r10 -+ (1/(3*(self.timespan/20)))
        #             self.r11 = self.r11 - (1/(3*(self.timespan/20)))
        #             self.r12 = 0
        #             self.r20 = self.r20 + (1/(3*(self.timespan/20)))
        #             self.r21 = self.r21 + (1/(3*(self.timespan/20)))
        #             self.r22 = 0
        #
        #             self.rotation_traj[i] = ([self.r00, self.r01, self.r02],
        #                                      [self.r10, self.r11, self.r12],
        #                                      [self.r20, self.r21, self.r22])
        #
        #             i = i + 1
        #
        #         while (self.timespan/5+(6*self.timespan)/20) <= i <= (self.timespan/5+(9*self.timespan)/20):
        #
        #             self.r00 = 0
        #             self.r01 = 0
        #             self.r02 = 1
        #             self.r10 = self.r10 + (1/(3*(self.timespan/20)))
        #             self.r11 = self.r11 + (1/(3*(self.timespan/20)))
        #             self.r12 = 0
        #             self.r20 = self.r20 - (1/(3*(self.timespan/20)))
        #             self.r21 = self.r21 + (1/(3*(self.timespan/20)))
        #             self.r22 = 0
        #
        #             self.rotation_traj[i] = ([self.r00, self.r01, self.r02],
        #                                      [self.r10, self.r11, self.r12],
        #                                      [self.r20, self.r21, self.r22])
        #
        #             i = i + 1
        #
        #         while (self.timespan/5+(9*self.timespan)/20) <= i <= (self.timespan/5+(12*self.timespan)/20):
        #
        #             self.r00 = 0
        #             self.r01 = 0
        #             self.r02 = 1
        #             self.r10 = self.r10 - (1/(3*(self.timespan/20)))
        #             self.r11 = self.r11 + (1/(3*(self.timespan/20)))
        #             self.r12 = 0
        #             self.r20 = self.r20 - (1/(3*(self.timespan/20)))
        #             self.r21 = self.r21 - (1/(3*(self.timespan/20)))
        #             self.r22 = 0
        #
        #             self.rotation_traj[i] = ([self.r00, self.r01, self.r02],
        #                                      [self.r10, self.r11, self.r12],
        #                                      [self.r20, self.r21, self.r22])
        #
        #             i = i + 1
        #
        #         while (self.timespan/5+(12*self.timespan)/20) <= i <= self.timespan:
        #
        #             self.r00 = self.r00 + (1/(self.timespan/5))
        #             self.r01 = 0
        #             self.r02 = self.r02 - (1/(self.timespan/5))
        #             self.r10 = 0
        #             self.r11 = 1
        #             self.r12 = 0
        #             self.r20 = self.r20 + (1/(self.timespan/5))
        #             self.r21 = 0
        #             self.r22 = self.r22 + (1/(self.timespan/5))
        #
        #             self.rotation_traj[i] = ([self.r00, self.r01, self.r02],
        #                                      [self.r10, self.r11, self.r12],
        #                                      [self.r20, self.r21, self.r22])
        #             i = i + 1
        #
        #             print(self.rotation_traj)
        #
        #         return self.rotation_traj
        #
        #     if self.y_axis is True:
        #
        #         self.rotation_traj[0] = np.array(([1, 0, 0],
        #                                           [0, 1, 0],
        #                                           [0, 0, 1]))
        #
        #         i = 0
        #
        #         while i <= 25:
        #             self.rotation_traj[i] = np.array(([1, 0, 0],
        #                                               [0, 1, 0],
        #                                               [0, 0, 1]))
        #
        #             i = i + 1
        #
        #
        #         while 25 <= i <= 25 + self.timespan/4:
        #
        #             self.r00 = self.r00 - (1/(self.timespan/4))
        #             self.r01 = 0
        #             self.r02 = self.r02 - (1/(self.timespan/4))
        #             self.r10 = 0
        #             self.r11 = 1
        #             self.r12 = 0
        #             self.r20 = self.r20 + (1/(self.timespan/4))
        #             self.r21 = 0
        #             self.r22 = self.r22 - (1/(self.timespan/4))
        #
        #             self.rotation_traj[i] = ([self.r00, self.r01, self.r02],
        #                                      [self.r10, self.r11, self.r12],
        #                                      [self.r20, self.r21, self.r22])
        #
        #             i = i+1
        #
        #         while (25 + self.timespan/4) <= i <= (25 + self.timespan/2):
        #
        #             self.r00 = self.r00 - (1/(self.timespan/4))
        #             self.r01 = 0
        #             self.r02 = self.r02 + (1/(self.timespan/4))
        #             self.r10 = 0
        #             self.r11 = 1
        #             self.r12 = 0
        #             self.r20 = self.r20 - (1/(self.timespan/4))
        #             self.r21 = 0
        #             self.r22 = self.r22 - (1/(self.timespan/4))
        #
        #             self.rotation_traj[i] = ([self.r00, self.r01, self.r02],
        #                                      [self.r10, self.r11, self.r12],
        #                                      [self.r20, self.r21, self.r22])
        #
        #             i = i+1
        #
        #         while (25 + self.timespan/2) <= i <= (25 + 3*self.timespan/4):
        #
        #             self.r00 = self.r00 + (1/(self.timespan/4))
        #             self.r01 = 0
        #             self.r02 = self.r02 + (1/(self.timespan/4))
        #             self.r10 = 0
        #             self.r11 = 1
        #             self.r12 = 0
        #             self.r20 = self.r20 - (1/(self.timespan/4))
        #             self.r21 = 0
        #             self.r22 = self.r22 + (1/(self.timespan/4))
        #
        #             self.rotation_traj[i] = ([self.r00, self.r01, self.r02],
        #                                      [self.r10, self.r11, self.r12],
        #                                      [self.r20, self.r21, self.r22])
        #
        #             i = i+1
        #
        #         while (25 + 3*self.timespan/4) <= i <= (25 + self.timespan):
        #
        #             self.r00 = self.r00 + (1/(self.timespan/4))
        #             self.r01 = 0
        #             self.r02 = self.r02 - (1/(self.timespan/4))
        #             self.r10 = 0
        #             self.r11 = 1
        #             self.r12 = 0
        #             self.r20 = self.r20 + (1/(self.timespan/4))
        #             self.r21 = 0
        #             self.r22 = self.r22 + (1/(self.timespan/4))
        #
        #             self.rotation_traj[i] = ([self.r00, self.r01, self.r02],
        #                                      [self.r10, self.r11, self.r12],
        #                                      [self.r20, self.r21, self.r22])
        #
        #             i = i + 1
        #
        #             print(self.rotation_traj)
        #
        #         return self.rotation_traj
        #
        #
        #     if self.z_axis is True:
        #
        #         self.rotation_traj[0] = np.array(([1, 0, 0],
        #                                           [0, 1, 0],
        #                                           [0, 0, 1]))
        #
        #         i = 0
        #
        #         while i <= 25:
        #
        #             self.rotation_traj[i] = np.array(([1, 0, 0],
        #                                               [0, 1, 0],
        #                                               [0, 0, 1]))
        #
        #             i = i + 1
        #
        #         while 25 < i <= ((self.timespan / 4)+25):
        #
        #             self.r00 = self.r00 - (1/(self.timespan/4))
        #             self.r01 = self.r01 + (1/(self.timespan/4))
        #             self.r02 = 0
        #             self.r10 = self.r10 - (1/(self.timespan/4))
        #             self.r11 = self.r11 - (1/(self.timespan/4))
        #             self.r12 = 0
        #             self.r20 = 0
        #             self.r21 = 0
        #             self.r22 = 1
        #
        #             self.rotation_traj[i] = ([self.r00, self.r01, self.r02],
        #                                      [self.r10, self.r11, self.r12],
        #                                      [self.r20, self.r21, self.r22])
        #
        #
        #             i = i + 1
        #
        #         while (25 + self.timespan/4) <= i <= (25 + self.timespan/2):
        #
        #             self.r00 = self.r00 - (1/(self.timespan/4))
        #             self.r01 = self.r01 - (1/(self.timespan/4))
        #             self.r02 = 0
        #             self.r10 = self.r10 + (1/(self.timespan/4))
        #             self.r11 = self.r11 - (1/(self.timespan/4))
        #             self.r12 = 0
        #             self.r20 = 0
        #             self.r21 = 0
        #             self.r22 = 1
        #
        #             self.rotation_traj[i] = ([self.r00, self.r01, self.r02],
        #                                      [self.r10, self.r11, self.r12],
        #                                      [self.r20, self.r21, self.r22])
        #
        #             i = i + 1
        #
        #         while (25 + self.timespan/2) <= i <= (25 + 3*self.timespan/4):
        #
        #             self.r00 = self.r00 + (1/(self.timespan/4))
        #             self.r01 = self.r01 - (1/(self.timespan/4))
        #             self.r02 = 0
        #             self.r10 = self.r10 + (1/(self.timespan/4))
        #             self.r11 = self.r11 + (1/(self.timespan/4))
        #             self.r12 = 0
        #             self.r20 = 0
        #             self.r21 = 0
        #             self.r22 = 1
        #
        #             self.rotation_traj[i] = ([self.r00, self.r01, self.r02],
        #                                      [self.r10, self.r11, self.r12],
        #                                      [self.r20, self.r21, self.r22])
        #
        #             i = i + 1
        #
        #         while (25 + 3*self.timespan/4) <= i <= (25 + self.timespan):
        #
        #             self.r00 = self.r00 + (1/(self.timespan/4))
        #             self.r01 = self.r01 + (1/(self.timespan/4))
        #             self.r02 = self.r02
        #             self.r10 = self.r10 - (1/(self.timespan/4))
        #             self.r11 = self.r11 + (1/(self.timespan/4))
        #             self.r12 = 0
        #             self.r20 = 0
        #             self.r21 = 0
        #             self.r22 = 1
        #
        #             self.rotation_traj[i] = ([self.r00, self.r01, self.r02],
        #                                      [self.r10, self.r11, self.r12],
        #                                      [self.r20, self.r21, self.r22])
        #
        #             print(self.rotation_traj)
        #
        #             i = i + 1
        #
        #         return self.rotation_traj
        #
        #
        # if self.rotation is False:
        #
        #     self.rotation_traj = {}
        #
        #     initial_orientation = np.array(([1, 0, 0],
        #                                     [0, 1, 0],
        #                                     [0, 0, 1]))
        #
        #     self.rotation_traj[0] = initial_orientation
        #
        #     i = 0
        #
        #     while i <= (self.timespan+1):
        #
        #         self.rotation_traj[i] = initial_orientation
        #
        #         i = i + 1
        #
        #         print(self.rotation_traj)
        #
        #     return self.rotation_traj

if __name__ == '__main__':

    traj = TrajectoryCircle(150, x_axis=False, y_axis=False, z_axis=True, orientation="all")
    traj.get_instant_circle(0.1)

