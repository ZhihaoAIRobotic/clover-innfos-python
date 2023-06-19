import numpy as np
from ikpy import chain
from ikpy import inverse_kinematics as ik
from scipy.spatial.transform import Rotation as R

class Kinematics:

    def __init__(self):

        # initialize the chain class [for the ikpy use - explained in github documentation] for Edubot arm
        self.urdf = chain.Chain.from_urdf_file(
            "/home/hengyi/GitHub/clover-innfos-python/Urdf/gluon.urdf",
            base_elements=["base_link", "axis_joint_1", "1_Link", "axis_joint_2", "2_Link",
                           "axis_joint_3", "3_Link", "axis_joint_4", "4_Link", "axis_joint_5", "5_Link", "axis_joint_6", "6_Link"],
            last_link_vector=None,
            active_links_mask=[False, True, True, True, True, True, True],
            name="Gluon"
        )

    def ik(self, Trans_Target, initial_position, orientation, **kwargs):

        self.orientation = orientation
        # initial_position = np.array([0, *initial_position])

        # initial position will be ([0, 0, 0, 0, 0, 0]) when it is set to None
        if initial_position is None:
            initial_position = [0] * len(self.urdf.links)

        # Set to np.array to change it to an array of integers instead of a tuple that will not work
        Trans_Target = np.array(Trans_Target)

        # Parameters
        # ----------
        # regularization_parameter: float - The coefficient of the regularization (idk as well)
        # max_iter: int - The maximum number of iterations
        # no_position: bool - If True, the position of the end effector will not be taken into account
        # orientation_mode: str - The orientation mode, can be "X", "Y", "Z", "None", "all"
        return ik.inverse_kinematic_optimization(self.urdf, Trans_Target, starting_nodes_angles=initial_position, regularization_parameter=None, max_iter=2000, orientation_mode=self.orientation, no_position=False)
    def fk(self, Joint_Target):

        # Parameters
        # full_kinematics: bool - If True, the full kinematics will be computed, otherwise only the position of the end effector will be computed
        Joint_Target = np.array([0, *Joint_Target])
        return self.urdf.forward_kinematics(Joint_Target, full_kinematics=False)

    def fk_to_pose(self, Joint_Target):

        T = self.fk(Joint_Target)
        rot = R.from_matrix(T[0:3, 0:3])
        quat = rot.as_quat()

        pose = np.array([*T[:3, 3], *quat[:3]])
        return pose

if __name__ == '__main__':

    kin = Kinematics()

    theta = np.array([0, 0, 1.570796, -1.570796, 1.570796, -3.141593])
    forward = kin.fk(theta)
    print(forward)

    theta = [0, 0, 1.570796, -1.570796, 1.570796, -3.141593]
    # theta = [0, 1, 0, 0.2, 0.3, 0.6]
    theta = np.array([0, *theta])
    pose = np.array([[0, 1, 0, 0.075731],
                     [0, 0, -1, -0.176746],
                     [-1, 0, 0, 0.2],
                     [0, 0, 0, 1]])

    theta_new = kin.ik(forward, initial_position=theta, orientation="all")
    print(theta_new)

    # forward = kin.fk_to_pose(theta)
    # print(forward)

    # forward_new = kin.fk(theta_new[1:7])
    # print(forward_new)

