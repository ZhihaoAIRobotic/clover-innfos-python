import numpy as np

<<<<<<< HEAD
=======


file_path = '/home/ubuntu/Github/clover-innfos-python/Urdf/gluon2.urdf'
ee_link_name = '6_Link'
>>>>>>> master

def get_jacobian_from_model(file_path, ee_link_name, joint_values):
    """
    Get the Jacobian matrix J of the robot from the 3D model, \dot x= J \dot q
    :param file_path:
    :param joint_values:
    :return:
    """
    import kinpy as kp
    from lxml import etree
    if file_path.endswith('.urdf'):
        data = open(file_path)
        xslt_content = data.read().encode()
        chain = kp.build_serial_chain_from_urdf(xslt_content, end_link_name=ee_link_name)
    elif file_path.endswith('.xml'):
        chain = kp.build_serial_chain_from_mjcf(open(file_path).read(), end_link_name=ee_link_name)

    J = chain.jacobian(joint_values)
    J = np.asarray(J)

    return J

<<<<<<< HEAD
def get_dJ(q, dq):
=======
<<<<<<< HEAD

if __name__ == '__main__':
    import kinpy as kp
    urdf_path = '/home/ubuntu/Rofunc/rofunc/simulator/assets/urdf/gluon/gluon.urdf'
    joint_values = [0, np.pi / 2.0, -np.pi / 2.0, np.pi/2, np.pi/2, 0]
    J = get_jacobian_from_model(urdf_path, ee_link_name='6_Link', joint_values=joint_values)
=======
def get_jacobian_deriv(q, v):
>>>>>>> master
    """
        Get the dJ/dt of the robot from the 3D model
        :param file_path:
        :param joint_values:
        :return:
    """
    import fkin

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
    dJ = (get_jacobian_from_model(q+delta_q) - get_jacobian_from_model(q-delta_q))/(2*delta_q)

    return dJ

if __name__ == '__main__':

<<<<<<< HEAD
=======
    # dJ, dt = get_jacobian_deriv(q, joint_vel)
>>>>>>> master


    ################# np.array pretty print #################################
>>>>>>> master
    def array_clean_print(sep=' ', vert='|', pad=10, precision=4):
        def prettyprint(a):
            s = "\n"
            if len(a.shape) == 1:
                a = [a]  # Make array appear 2D if it's 1D
            for row in a:
                s += vert + sep + sep.join([f"{x: ^ {pad}.{precision}g}" for x in row]) + vert + "\n"
            return s

        return prettyprint

    np.set_string_function(array_clean_print(), repr=False)
    np.set_string_function(array_clean_print(), repr=True)

<<<<<<< HEAD
    import kinpy as kp
    q = np.array([0.4, 0.2, 0.5, 0.7, 0.2, 0.1])
    joint_vel = np.array([1, 1, 0.4, 0.4, 0.2, 1.1])

    J = get_jacobian_from_model(q)

    dJ = get_dJ(q, joint_vel)
    #
    print(dJ)

    dJ = get_eng_dJ(q)
    print(dJ)
=======
    print(J)
<<<<<<< HEAD
=======
    print(J_also)
    # print(dJ)
    # print(dt)
>>>>>>> master
>>>>>>> master

