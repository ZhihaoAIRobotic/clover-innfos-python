import scps.dynamics as dyn
import scps.ikpykinematics as kin
import numpy as np

kin = kin.Kinematics()

class PD_controller():

    """
    This function takes in a joint configuration and returns the gravity vector

    Input
    :param q: The joint configuration

    Output
    :return: The gravity vector
    """

    def __int__(self, kp, kd):
        self.kp = kp
        self.kd = kd

    def pos_control(self, q, qd, dq):

        self.qe = qd - q
        grav = dyn.gravity(q)

        self.u = grav + self.kp*self.qe - self.kd*dq

        return self.u

    def force_control(self, q, qd, dq):

        self.qe = qd - q

        grav = dyn.gravity(q)

        self.u = grav + self.kp*self.qe - self.kd*dq

        return self.u


