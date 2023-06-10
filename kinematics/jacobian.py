import numpy
import numpy as np
import pinocchio as pin

path = "/home/hengyi/GitHub/clover-innfos-python/Urdf/gluon.urdf"
joint_name = ["axis_joint_1", "axis_joint_2", "axis_joint_3", "axis_joint_4", "axis_joint_5", "axis_joint_6"]
Link_name = ["1_Link", "2_Link", "3_Link", "4_Link", "5_Link", "6_Link"]
ee_link_name = "6_Link"


def get_J(joint_values):
    """
    Get the Jacobian matrix J of the robot from the 3D model, \dot x= J \dot q
    :param file_path:
    :param joint_values:
    :return:
    """
    import kinpy as kp
    from lxml import etree
    file_path = path
    if file_path.endswith('.urdf'):
        data = open(file_path)
        xslt_content = data.read().encode()
        chain = kp.build_serial_chain_from_urdf(xslt_content, end_link_name=ee_link_name)
    elif file_path.endswith('.xml'):
        chain = kp.build_serial_chain_from_mjcf(open(file_path).read(), end_link_name=ee_link_name)

    J = chain.jacobian(joint_values)
    J = np.asarray(J)

    return J

def get_Jas(joint_values):
    """
        Get the Jacobian matrix J of the robot from the 3D model
        :param file_path:
        :param joint_values:
        :return:
    """
    model = pin.buildModelFromUrdf("/home/hengyi/GitHub/clover-innfos-python/Urdf/gluon.urdf")
    data = model.createData()
    # print(data)
    jid = model.getJointId(ee_link_name)

    # jid = ee_link_name

    # Ja = pin.computeJointJacobians(model, data, joint_values)
    Ja = pin.computeJointJacobian(model, data, joint_values, jid)



    return Ja


def get_dJ(q, dq):

    """
        Get the dJ/dt of the robot from the 3D model
        :param file_path:
        :param joint_values:
        :return:
    """
    import kinematics.fkin as fk

    T = fk.fkall(q)

    Q = np.zeros([len(T), 3, 3])
    z = np.zeros([len(T), 3, 1])
    a = np.zeros([len(T), 3, 1])

    for i in range(len(T)):
        Q[i] = T[i][:3, :3]
        z[i] = T[i][:3, 2].reshape(3, 1)
        a[i] = T[i][:3, 3].reshape(3, 1)

    # Calculate the omega vectors
    w = np.zeros([len(dq), 3, 1])

    i = 0
    w[0] = dq[0] * z[0]
    for i in range(1, len(dq)):
        # w[i] = dq[i]*z[i]
        w[i] = dq[i]*z[i] + np.matmul(Q[i].T, w[i - 1])
        i = i + 1

    # Calculate dz/dt
    z_dot = np.zeros([len(dq), 3, 1])
    z_dot[0] = 0
    for j in range(1, len(dq)):
        z_dot[j] = np.cross(w[j].reshape(3), z[j].reshape(3)).reshape(3, 1)
        j = j + 1

    # Calculate dr/dt
    r_dot = np.zeros([len(dq), 3, 1])
    for p in range(len(dq)):
        r_dot[p] = np.cross(w[p].reshape(3), a[p].reshape(3)).reshape(3, 1)

    # Calculate du/dt
    u_dot = np.zeros([len(dq), 3, 1])
    u_dot[0] = np.cross(z[0].reshape(3), r_dot[0].reshape(3)).reshape(3, 1)
    for k in range(1, len(dq)):
        r = a[-1] - a[k]
        u_dot[k] = np.cross(z_dot[k].reshape(3), r.reshape(3)).reshape(3, 1) + np.cross(z[k].reshape(3), r_dot[k - 1].reshape(3)).reshape(3, 1)

    j_dot = np.zeros([6, len(dq)])
    for l in range(len(dq)):
        j_dot[:, l] = np.vstack((z_dot[l], u_dot[l])).reshape(6)

    return j_dot

def get_eng_dJ(q):
    """
        Get the dJ/dt with and engineering method for lower computational cost
        :param file_path:
        :param joint_values:
        :return:
    """
    delta_q = q*0.001
    dJ = (get_J(q+delta_q) - get_J(q-delta_q))/(2*delta_q)

    return dJ

if __name__ == '__main__':
    q = np.array([0.4, 0.2, 0.5, 0.7, 0.2, 0.1])
    joint_vel = np.array([1, 1, 0.4, 0.4, 0.2, 1.1])

    # J = get_dJ(q, joint_vel)
    # print(J.shape)
    #
    J = get_J(q)
    print(J)



    # print(dJ)
    # print(dt)

