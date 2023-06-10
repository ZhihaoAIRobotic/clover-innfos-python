import numpy as np
from scipy.spatial.transform import Rotation as R

def dq_to_w(pose, dpose):
    w = np.zeros([len(dpose), 4])
    q = pose[:, 3:]
    dq = dpose[:, 3:]
    dx = np.zeros([len(dpose), 6])
    for i in range(len(dq)):
        quat = np.array([*q[i][1:], q[i][0]]).reshape(1, 4)
        dquat = np.array([*dq[i][1:], dq[i][0]]).reshape(1, 4)
        w[i] = 2 * dquat * (np.linalg.pinv(quat).reshape(1, 4))
        dx[i] = np.array([*dpose[i, :3], *w[i, 1:]])
    # print(w)
    return dx

def dq_w(q, dq):
    q = np.array([*q[1:3], q[0]])
    dq = np.array([*dq[1:3], dq[0]])
    w = 2 * dq * (np.linalg.pinv(q).reshape(1, 4))

    return w
def ddq_to_dw(pose, dpose, ddpose):
    dw = np.zeros([len(ddpose), 4])
    q = pose[:, 3:]
    dq = dpose[:, 3:]
    ddq = ddpose[:, 3:]
    ddx = np.zeros([len(dpose), 6])

    for i in range(len(ddq)):
        quat = np.array([*q[i][1:], q[i][0]]).reshape(1, 4)
        dquat = np.array([*dq[i][1:], dq[i][0]]).reshape(1, 4)
        ddquat = np.array([*ddq[i][1:], ddq[i][0]]).reshape(1, 4)
        w = dq_w(quat, dquat)
        # print(quat)
        dw = (2 * ddquat - w * dquat) * (np.linalg.pinv(quat).reshape(1, 4))
        dw = dw[0]
        ddx[i] = np.array([*ddpose[i, :3], *dw[1:]])

    return ddx

if __name__ == "__main__":
    from matplotlib import pyplot as plt
    traj = np.load('coffee_traj.npy')

    x = np.zeros([len(traj), 7])
    dx = np.zeros([len(traj), 7])
    ddx = np.zeros([len(traj), 7])

    for i in range(len(traj)):
        x[i] = traj[i, 0:7]
        dx[i] = traj[i, 7:14]
        ddx[i] = traj[i, 14:]

    p = np.hstack([x[:, :3], x[:, 4:]])
    v = dq_to_w(x, dx)
    a = ddq_to_dw(x, dx, ddx)

    trajectory = np.hstack([p, v, a])
    # print(trajectory.shape)
    # np.save('coffee_traj_dt04_6by1', trajectory)


    # plotx = p[:, 0]
    # ploty = p[:, 1]
    #
    # graphlist = np.array(plotx).reshape(1, np.array(plotx).shape[0])
    # graphlist2 = np.array(ploty).reshape(1, np.array(ploty).shape[0])
    # errorlist = graphlist - graphlist2
    #
    # for i in range(len(graphlist)):
    #     # print(row)
    #     plt.plot(graphlist[i], graphlist2[i])
    #     plt.ylabel("y (meters)")
    #     plt.xlabel("x (meters)")
    #     plt.title("Trajectory of the cup")
    # plt.show()
    #
    # plotx = v[:, 0]
    # ploty = v[:, 1]
    #
    # graphlist = np.array(plotx).reshape(1, np.array(plotx).shape[0])
    # graphlist2 = np.array(ploty).reshape(1, np.array(ploty).shape[0])
    # errorlist = graphlist - graphlist2
    #
    # for i in range(len(graphlist)):
    #     # print(row)
    #     plt.plot(graphlist[i], graphlist2[i])
    #     plt.ylabel("y (meters)")
    #     plt.xlabel("x (meters)")
    #     plt.title("Trajectory of the cup")
    # plt.show()
