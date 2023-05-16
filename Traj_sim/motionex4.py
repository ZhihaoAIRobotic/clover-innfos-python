import numpy as np
import mujoco_viewer
import mujoco
import os
from kinematics.jacobian import get_J
from coffee_project.polynomial_trajectory_cartesian import PolynomialGeneratorCart
from coffee_project.polynomial_trajectory import PolynomialGenerator

pg = PolynomialGenerator()
pgc = PolynomialGeneratorCart()

ASSETS = {}
root_path = '/home/hengyi/GitHub/clover-innfos-python/environment models/meshes'
files = os.listdir(root_path)
for file in files:
    f = open(os.path.join(root_path, file), 'rb')
    num = file.split('_')[0]
    ASSETS['{}_Link.stl'.format(num)] = f.read()

model = mujoco.MjModel.from_xml_path("/home/hengyi/GitHub/clover-innfos-python/environment models/gluon_robot.xml", ASSETS)
data = mujoco.MjData(model)

x0 = [0.0756856730, -0.0000226960000, 0.533979, 0, 1.57079633, 0]
dx0 = [0, 0, 0, 0, 0, 0]
ddx0 = [0, 0, 0, 0, 0, 0]
x_start = np.array([x0, dx0, ddx0])

via_points = np.array([[0.0757261258, -0.12336681, 0.482877920, 1.57079633, 1.37079633, 2.67079633],
                       [0.07568567, -0.1744927, 0.359509, 1.57079633, 1.07079633, 1.77079633]])

t_list = np.array([10.0, 20.0])
n_list = np.array([5000, 5000])
x, dx, ddx = pgc.generate_trajectory(x_start, via_points, 0, t_list, n_list)

q0 = [0, 0, 0, 0, 0, 0]
dq0 = [0, 0, 0, 0, 0, 0]
ddq0 = [0, 0, 0, 0, 0, 0]
q_start = np.array([q0, dq0, ddq0])

via_points = np.array([[0, np.pi / 4, -np.pi / 4, 0, 1.1, 0.2],
                        [0, np.pi / 2, -np.pi / 2, 0, 0.2, 0.5]])

q, dq, ddq = pg.generate_trajectory(q_start, via_points, 0, t_list, n_list)

viewer = mujoco_viewer.MujocoViewer(model, data)

i = 0
data.qpos[:] = q[1]
print(q[i])
print(x[i])
for i in range(len(x)):

    theta = data.qpos
    vel_control = np.linalg.inv(get_J(theta)) @ dx[i]
    data.qvel[:] = vel_control

    mujoco.mj_step(model, data)
    viewer.render()

    i = i + 1

