import numpy as np
from scps.ikpykinematics import Kinematics

class TrajectoryCurve(object):
    def __init__(self, Target, timespan, Rotation, orientation):
        self.x, self.y, self.z, self.theta, self.phi, self.psi = np.zeros(timespan), np.zeros(timespan), np.zeros(
            timespan), np.zeros(timespan), np.zeros(timespan), np.zeros(timespan)

        self.xt, self.yt, self.zt, self.thetat, self.phit, self.psit = 7.5685673e-02, -2.2696000e-05, 5.3397900e-01, 0, 0, 0
        self.timespan = timespan
        self.target = [0, Target[0], Target[1], Target[2], Target[3], Target[4], Target[5]]
        self.initial_orientation = np.array(([1, 0, 0],
                                             [0, 1, 0],
                                             [0, 0, 1]))
        self.orientation = orientation
        self.dsr_rotationmatrix = np.array(Rotation)

    def get_instant_curve(self):

        # initialize the kinematics library
        kin = Kinematics()

        q = self.target[1:7]

        Transform_Target = kin.fk(q)

        # Then extract the position from the 4x4 transformation matrix
        target_x = Transform_Target[0, 3]
        target_y = Transform_Target[1, 3]
        target_z = Transform_Target[2, 3]

        self.coordinate = []

        dx = (0.075685673 - target_x)/self.timespan
        dy = (-0.000022696 - target_y)/self.timespan
        dz = (0.533979 - target_z)/self.timespan

        self.coordinate.append(np.array([7.5685673e-02, -2.2696000e-05, 5.3397900e-01]))

        i = 0

        for i in np.arange(1, self.timespan+1):

            self.xt = self.xt - dx
            self.yt = self.yt - dy
            self.zt = 0.08*np.sin((2*((2*np.pi)/(target_x-7.5685673e-02)))*(self.xt-7.5685673e-02))+5.3397900e-01 - dz*i

            self.coordinate.append(np.array([self.xt, self.yt, self.zt]))

            i = i + 1

            # print(self.coordinate)

        return self.coordinate

    def get_instant_rotation(self):

        self.initial_orientation = np.array(([1, 0, 0],
                                             [0, 1, 0],
                                             [0, 0, 1]))

        # Calculate the change in rotation for each time step
        targetrotation1 = self.dsr_rotationmatrix[0, 0]
        targetrotation2 = self.dsr_rotationmatrix[0, 1]
        targetrotation3 = self.dsr_rotationmatrix[0, 2]
        targetrotation4 = self.dsr_rotationmatrix[1, 0]
        targetrotation5 = self.dsr_rotationmatrix[1, 1]
        targetrotation6 = self.dsr_rotationmatrix[1, 2]
        targetrotation7 = self.dsr_rotationmatrix[2, 0]
        targetrotation8 = self.dsr_rotationmatrix[2, 1]
        targetrotation9 = self.dsr_rotationmatrix[2, 2]

        # Calculate the change in rotation for each time step
        dr00 = (targetrotation1 - self.initial_orientation[0, 0]) / self.timespan
        dr01 = (targetrotation2 - self.initial_orientation[0, 1]) / self.timespan
        dr02 = (targetrotation3 - self.initial_orientation[0, 2]) / self.timespan
        dr10 = (targetrotation4 - self.initial_orientation[1, 0]) / self.timespan
        dr11 = (targetrotation5 - self.initial_orientation[1, 1]) / self.timespan
        dr12 = (targetrotation6 - self.initial_orientation[1, 2]) / self.timespan
        dr20 = (targetrotation7 - self.initial_orientation[2, 0]) / self.timespan
        dr21 = (targetrotation8 - self.initial_orientation[2, 1]) / self.timespan
        dr22 = (targetrotation9 - self.initial_orientation[2, 2]) / self.timespan

        # Initialize the rotation trajectory through an initial step
        self.initial_orientation[0, 0] += dr00
        self.initial_orientation[0, 1] += dr01
        self.initial_orientation[0, 2] += dr02
        self.initial_orientation[1, 0] += dr10
        self.initial_orientation[1, 1] += dr11
        self.initial_orientation[1, 2] += dr12
        self.initial_orientation[2, 0] += dr20
        self.initial_orientation[2, 1] += dr21
        self.initial_orientation[2, 2] += dr22

        # Note: xyz variables are used here as using the += on the self.variables did not work
        # This is because the += operator did not work and gave integers only and not floats
        # initialize the initial rotation trajectory
        r00 = 1
        r01 = 0
        r02 = 0
        r10 = 0
        r11 = 1
        r12 = 0
        r20 = 0
        r21 = 0
        r22 = 1

        # initialize the rotation trajectory dictionary
        self.rotation_traj = []

        i = 0

        # Loop through the time span and calculate the position for each time step
        while i <= (self.timespan):

            r00 = r00 + dr00
            r01 = r01 + dr01
            r02 = r02 + dr02
            r10 = r10 + dr10
            r11 = r11 + dr11
            r12 = r12 + dr12
            r20 = r20 + dr20
            r21 = r21 + dr21
            r22 = r22 + dr22

            # Record all the rotation matrix at each timestep into the dictionary
            self.rotation_traj.append(np.array([[r00, r01, r02],
                                       [r10, r11, r12],
                                       [r20, r21, r22]]))

            i = i+1

        return self.rotation_traj


if __name__ == '__main__':
    Target = np.array([-0.19190641732275654, -2.661618746277845, -1.7225457138037745, 0.9390738728429078, 0.19190412669933546,
                       -8.201720369527266e-07])
    Rotation = ([1, 0, 0],
                [0, 1, 0],
                [0, 0, 1])
    a = TrajectoryCurve(Target, 100, Rotation, orientation="all")
    a.get_instant_curve()
    a.get_instant_rotation()
    print(a.get_instant_curve())

# 100: array([ 0.11732505, -0.20689729,  0.12532135])}
# 100: array([ 0.11650859, -0.20284092,  0.13334344])}
