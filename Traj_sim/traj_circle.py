import numpy as np
from TrajectoryCircle import TrajectoryCircle
from scps.ikpykinematics import Kinematics

kin = Kinematics()
def change(linear, rotation):

    joint_list = []

    for i in range(len(linear)):

        if i == 0:
            l = linear[i]
            r = rotation[i]
            print(r[0])

            transform = np.array([[*r[0, :], l[0]],
                                  [*r[1, :],l[1]],
                                  [*r[2, :],l[2]],
                                  [0,0,0,1]])

            traj_joint = kin.ik(transform, initial_position=None, orientation=None)
            joint_list.append(traj_joint)

            i = i+1

        else:
            l = linear[i]
            r = rotation[i]

            transform = np.array([[*r[0, :], l[0]],
                                  [*r[1, :], l[1]],
                                  [*r[2, :], l[2]],
                                  [0, 0, 0, 1]])

            traj_joint = kin.ik(transform, initial_position=joint_list[i-1], orientation=None)
            joint_list.append(traj_joint)

            i = i+1

    return joint_list

# if __name__ == "__main__":
#     traj = TrajectoryCircle(150, x_axis=True, y_axis=False, z_axis=False, orientation="all")
#     a = traj.get_instant_circle(0.1)
#     b = traj.get_instant_rotation()
#
#     q = change(a,b)
#     print(q[1])
