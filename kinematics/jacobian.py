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

    # ret = chain.forward_kinematics(joint_values, end_only=False)

    # viz = kp.Visualizer()
    # viz.add_robot(ret, chain.visuals_map(), mesh_file_path="/home/ubuntu/Github/Manipulation/kinpy/examples/kuka_iiwa/", axes=True)
    # viz.spin()

    return J

<<<<<<< HEAD

if __name__ == '__main__':
    import kinpy as kp
    urdf_path = '/home/ubuntu/Rofunc/rofunc/simulator/assets/urdf/gluon/gluon.urdf'
    joint_values = [0, np.pi / 2.0, -np.pi / 2.0, np.pi/2, np.pi/2, 0]
    J = get_jacobian_from_model(urdf_path, ee_link_name='6_Link', joint_values=joint_values)
=======
def get_jacobian_deriv(q, v):
    """
        Get the dJ/dt of the robot from the 3D model
        :param file_path:
        :param joint_values:
        :return:
    """

    # Create a model
    model = pin.buildModelFromUrdf("/home/ubuntu/Github/clover-innfos-python/Urdf/gluon2.urdf")

    # Create a data object
    data = model.createData()

    J = pin.computeJointJacobiansTimeVariation(model, data, q, v)

    joint_id = model.getJointId('axis_joint_1')
    frame = model.getFrameId('6_Link')

    dJ = pin.getJointJacobianTimeVariation(model, data, joint_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
    # dJ = pin.getFrameJacobianTimeVariation(model, data, frame, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)

    return dJ, data.J

def get_jacobian(q):
    """
            Get the dJ/dt of the robot from the 3D model
            :param file_path:
            :param joint_values:
            :return:
        """

    # Create a model
    model = pin.buildModelFromUrdf("/home/ubuntu/Github/clover-innfos-python/Urdf/gluon2.urdf")

    # Create a data object
    data = model.createData()
    joint_id = model.getFrameId("6_Link")


    J = pin.computeJointJacobian(model, data, q, model.getJointId('axis_joint_6'))
    # J = pin.computeJointJacobians(model, data, q)

    # J = pin.getJointJacobian(model, data, joint_id, pin.ReferenceFrame.LOCAL)

    return J


if __name__ == '__main__':
    import kinpy as kp
    q = np.array([0, 0, 0, 0, 0, 0])
    joint_vel = np.array([0, 1, 0.4, 0.4, 0.2, 0])

    J = get_jacobian_from_model(q)
    J_also = get_jacobian(q)


    # dJ, dt = get_jacobian_deriv(q, joint_vel)
>>>>>>> master


    ################# np.array pretty print #################################
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

    print(J)
<<<<<<< HEAD
=======
    print(J_also)
    # print(dJ)
    # print(dt)
>>>>>>> master

