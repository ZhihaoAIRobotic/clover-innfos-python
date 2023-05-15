from urdfpy import URDF
import numpy as np
urdf_path = "/home/ubuntu/Github/clover-innfos-python/Urdf/gluon.urdf"
joint_name = ["axis_joint_1", "axis_joint_2", "axis_joint_3", "axis_joint_4", "axis_joint_5", "axis_joint_6"]
Link_name = ["1_Link", "2_Link", "3_Link", "4_Link", "5_Link", "6_Link"]


def fk(joint_value, export_link='6_Link'):
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

    return export_pose

def fkall(joint_value, urdf_path=urdf_path):
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

    fk_all = np.zeros([len(joint_name), 4, 4])

    i = 0
    for i in range(len(joint_value)):
        export_pose = forward_kinematics[robot.links[link_name.index(Link_name[i])]]
        fk_all[i] = export_pose
        i = i + 1

    return fk_all

def fk_pose(joint_value):

    T = fk(joint_value)
    rot = R.from_matrix(np.array([T[:3, :3]]))
    eul = rot.as_euler('xyz')

    return np.array([*T[:3, 3], *eul[0]])



if __name__ == '__main__':
    import numpy as np
<<<<<<< HEAD:kinematics/fkin.py
    joint_value = np.array([1.0, 1.40, -1.10, 1.30, 0.2, 0.8])
    export_pose = fk(joint_value)
=======

    urdf_path = "/home/ubuntu/Rofunc/rofunc/simulator/assets/urdf/gluon/gluon.urdf"
    joint_name = ["axis_joint_1", "axis_joint_2", "axis_joint_3", "axis_joint_4", "axis_joint_5", "axis_joint_6"]
    link_name = ["1_Link", "2_Link", "3_Link", "4_Link", "5_Link", "6_Link"]
    joint_value = np.array([0, np.pi/2, -np.pi/2, 0, 0, 0])
    robot, export_pose, cfg = fk("/home/ubuntu/Rofunc/rofunc/simulator/assets/urdf/gluon/gluon.urdf", joint_name, joint_value, export_link='6_Link')
>>>>>>> master:kinematics/fk.py

    print(export_pose)

    from scipy.spatial.transform import Rotation as R

    rot = R.from_matrix(np.array([export_pose[:3, :3]]))
    q = rot.as_quat()

    print(q)

    eul = rot.as_euler('xyz')
    print(eul)

    print(fk_pose(joint_value))