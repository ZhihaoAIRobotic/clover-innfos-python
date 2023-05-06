from urdfpy import URDF
import numpy as np


def fk(urdf_path, joint_name, joint_value, export_link='panda_left_link7'):
    robot = URDF.load(urdf_path)
    link_name = []
    for link in robot.links:
        str = link.name
        link_name.append(str)

    cfg = {}

    for key, value in zip(joint_name, joint_value):
        if key != 'reference':
            cfg[key] = value

    forward_kinematics = robot.link_fk(cfg=cfg)

    export_pose = forward_kinematics[robot.links[link_name.index(export_link)]]

    return robot, export_pose, cfg

if __name__ == '__main__':
    import numpy as np

    urdf_path = "/home/ubuntu/Rofunc/rofunc/simulator/assets/urdf/gluon/gluon.urdf"
    joint_name = ["axis_joint_1", "axis_joint_2", "axis_joint_3", "axis_joint_4", "axis_joint_5", "axis_joint_6"]
    link_name = ["1_Link", "2_Link", "3_Link", "4_Link", "5_Link", "6_Link"]
    joint_value = np.array([0, np.pi/4, -np.pi/4, 0, 1.1, 0.2])
    robot, export_pose, cfg = fk("/home/ubuntu/Rofunc/rofunc/simulator/assets/urdf/gluon/gluon.urdf", joint_name, joint_value, export_link='6_Link')

    print(export_pose)

    from scipy.spatial.transform import Rotation as R

    rot = R.from_matrix(np.array([export_pose[:3, :3]]))
    q = rot.as_quat()

    print(q)

    eul = rot.as_euler('xyz')
    print(eul)


    # f_all = []
    #
    # for i in range(len(joint_name)):
    #     robot, f, cfg = fk(urdf_path, joint_name, joint_value, export_link=link_name[i])
    #
    #     f_all.append(f)
    #
    # f_all = np.asarray(f_all)
    #
    # print(f_all)
