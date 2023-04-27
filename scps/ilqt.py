import rofunc as rf
import numpy as np
from scps.ikpykinematics import Kinematics
from scipy.spatial.transform import Rotation as R

kin = Kinematics()

def ilqt(j_traj):

    """
    This function takes in a list of joint via points and returns a smooth trajectory

    Input
    :param j_traj: A list of joint via points. Each via point is one joint configuration.

    Output
    :return: A smooth trajectory of joint configurations
    """
    fk_traj = []

    i = 0

    for i in range(len(j_traj)):
        j_tra = np.array(j_traj[i])
        fk_traj.append(kin.fk(j_tra))

    fk_traj = np.array(fk_traj)

    q_traj = []

    for i in range(len(fk_traj)):
        r_matrix = np.array([fk_traj[i][0, 0:3],
                             fk_traj[i][1, 0:3],
                             fk_traj[i][2, 0:3]])

        r = R.from_matrix(r_matrix)
        q_r = r.as_quat()

        q = [[fk_traj[i][0, 3]],
             [fk_traj[i][1, 3]],
             [fk_traj[i][2, 3]],
             [q_r[0]],
             [q_r[1]],
             [q_r[2]],
             [q_r[3]]]

        q_traj.append(q)


    q_traj = np.reshape(q_traj, (len(q_traj), 7))

    controller = rf.planning_control.lqt.LQT(q_traj)
    u_hat, x_hat, mu, idx_slices = controller.solve()
    x_hat = np.array(x_hat)

    x_hat = np.array(x_hat[:, 0:7])
    x_hat_dot = np.array(x_hat[:, 7:])

    quat = x_hat[:, 3:7]
    q_quat = R.from_quat(quat)
    r_hat = q_quat.as_matrix()

    final_traj = []

    i = 0

    for i in range(len(r_hat)):
        trans_hat = np.array([[*r_hat[i][0], x_hat[i][0]],
                              [*r_hat[i][1], x_hat[i][1]],
                              [*r_hat[i][2], x_hat[i][2]],
                              [0, 0, 0, 1]])

        final_jv = kin.ik(r_hat[i], initial_position=None, orientation="all")

        final_traj.append(final_jv[1:])

        i = i + 1

    return final_traj

if __name__ == '__main__':

    Traj = np.array([[0, 0, 0, 0, 0, 0],
                     [-0.10808229, 5.93054473, 83.56811523, 3.47045898, 0.09765625, 86.52282715],
                     [-4.94830012, 15.38734794, 134.1973877, 62.75939941, -0.73364258, 86.5222168],
                     [-3.97674561, 50.89657676, 120.01220703, 87.05993652, -7.04406738, 86.52160645],
                     [-4.89110529, 60.83504009, 95.49743652, 77.04040527, -2.04101562, 86.52282715],
                     [3.00322831, 73.76044106, 66.89868164, 49.23095703, 2.04040527, 86.52099609]])

    final_traj = ilqt(Traj)
    print(final_traj)
