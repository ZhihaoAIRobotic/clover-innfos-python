import numpy as np

def concat(qpos, qpos1, qpos2, qpos3, qpos4):

    qpos = np.vstack([qpos, qpos1, qpos2, qpos3, qpos4])

    return qpos

if __name__ == '__main__':
    traj = np.load('basejoint_lqt.npy')
    traj1 = np.load('readjust1joint_lqt.npy')
    traj2 = np.load('designjoint_lqt.npy')
    traj3 = np.load('readjust2joint_lqt.npy')
    traj4 = np.load('finishjoint_lqt.npy')

    q1 = np.zeros([len(traj), 18])

    for i in range(len(traj)):
        q1[i] = traj[i, :]

    q2 = np.zeros([len(traj1), 18])

    for i in range(len(traj1)):
        q2[i] = traj1[i, :]

    q3 = np.zeros([len(traj2), 18])

    for i in range(len(traj2)):
        q3[i] = traj2[i, :]

    q4 = np.zeros([len(traj3), 18])

    for i in range(len(traj3)):
        q4[i] = traj3[i, :]

    q5 = np.zeros([len(traj4), 18])

    for i in range(len(traj4)):
        q5[i] = traj4[i, :]

    q = concat(q1, q2, q3, q4, q5)
    print(q.shape)

    np.save('art_lqt', q)