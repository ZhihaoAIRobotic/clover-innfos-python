import numpy as np
from scipy.spatial.transform import Rotation as R
from scps.ikpykinematics import Kinematics

kin = Kinematics()
def pose_ik(p, initial_guess):

    quat = R.from_quat(p[3:])
    rot = quat.as_matrix()

    T = np.array([[*rot[0, :], p[0]],
                  [*rot[1, :], p[1]],
                  [*rot[2, :], p[2]],
                  [0, 0, 0, 1]])


    q = kin.ik(T, initial_guess, orientation="all")
    q = q[1:]

    return q

def pose_traj(x, initial):

    q_list = np.zeros([len(x), 6])

    q_list[0] = np.array(initial)

    i = 1
    for i in range(len(x)):
        guess = np.array([0, *q_list[-1]])
        q_list[i] = pose_ik(x[i], guess)
        i = i+1

    return q_list

if __name__ == "__main__":

    from matplotlib import pyplot as plt

    traj = np.load('base.npy')
    # traj_init = np.load('readjust1joint.npy')

    base_init = [0, 0, -2.356194, -2.356194, 1.570796, 0]

    x = np.zeros([len(traj), 7])

    for i in range(len(traj)):
        x[i] = traj[i, 0:7]

    # x1 = np.zeros([len(traj_init), 6])
    #
    # for i in range(len(traj_init)):
    #     x1[i] = traj_init[i, 0:6]


    q = pose_traj(x, base_init)
    # print(q[2])
    # print(x1[-1])

    np.save('basejoint.npy', q)
    q1 = q[:, :]

    # print(np.array(q[0: 100]))

    graphlist = np.array(q1).reshape(6, np.array(q1).shape[0])

    for i in range(len(graphlist)):
        # print(row)
        plt.plot(graphlist[i])
        plt.ylabel("y (meters)")
        plt.xlabel("x (meters)")
        plt.title("Trajectory of the cup")
    plt.show()
