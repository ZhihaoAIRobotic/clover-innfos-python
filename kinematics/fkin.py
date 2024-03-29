import numpy as np
from urdfpy.urdf import URDF
from scipy.spatial.transform import Rotation as R

urdf_path = "/home/hengyi/GitHub/clover-innfos-python/Urdf/gluon.urdf"
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
    quat = rot.as_quat()

    return np.array([*T[:3, 3], *quat[0]])



if __name__ == '__main__':
    import numpy as np
    joint_value = np.array([0, 0, 0, 0, 0, 0])
    export_pose = fk(joint_value)

    joint_name = ["axis_joint_1", "axis_joint_2", "axis_joint_3", "axis_joint_4", "axis_joint_5", "axis_joint_6"]
    link_name = ["1_Link", "2_Link", "3_Link", "4_Link", "5_Link", "6_Link"]
    # joint_value = np.array([0, np.pi/2, -np.pi/2, 0, 0, 0])
    # export_pose = fk(joint_value)
    print(export_pose)

    from scipy.spatial.transform import Rotation as R

    rot = R.from_matrix(np.array([export_pose[:3, :3]]))
    q = rot.as_quat()
    q = q[0]
    print(q)

    pose = [export_pose[0, 3], export_pose[1, 3], export_pose[2, 3], q[0], q[1], q[2], q[3]]
    print(pose)

    # eul = rot.as_euler('xyz')
    # print(eul)
    #
    # print(fk_pose(joint_value))