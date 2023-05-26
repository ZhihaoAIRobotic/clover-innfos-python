import matplotlib as plt
import numpy as np
import numpy as np
import mujoco_viewer
import mujoco
import matplotlib.pyplot as plt
import os
from kinematics import fk, jacobian
from scps.dynamics import gravity, inertia, centrifugalterms
from scipy.spatial.transform import Rotation as R
from scps.ikpykinematics import Kinematics

kin = Kinematics()
ASSETS = {}
root_path = '/home/hengyi/GitHub/clover-innfos-python/Urdf/meshes'
files = os.listdir(root_path)
for file in files:
    f = open(os.path.join(root_path, file), 'rb')
    num = file.split('_')[0]
    ASSETS['{}_Link.stl'.format(num)] = f.read()

model = mujoco.MjModel.from_xml_path("/home/hengyi/GitHub/clover-innfos-python/Urdf/gluon_robot.xml", ASSETS)
data = mujoco.MjData(model)

viewer = mujoco_viewer.MujocoViewer(model, data)

traj = np.load("/home/hengyi/GitHub/clover-innfos-python/coffee_project/Test Code/test_traj.npy")
x = traj[:, 0:7]
dx = traj[:, 7:14]
ddx = traj[:, 14:21]

des_pos = []
con = []

i = 0

for i in range(len(x)):

    r = R.from_quat(x[i, 3:7])
    rot = r.as_matrix()
    pos = x[i, 0:3]
    T = np.array([[*rot[0, :], pos[0]],
                  [*rot[1, :], pos[1]],
                  [*rot[2, :], pos[2]],
                  [0, 0, 0, 1]])

    q = kin.ik(T, initial_position=None, orientation="all")
    q = q[1:]

    J = jacobian.get_J(q)
    dq = np.linalg.pinv(J) @ dx[i, :6].reshape(6, 1)

    des_pos.append(q[1])
    con.append(data.qpos.reshape(6)[1])


    data.qvel[:] = dq.reshape(6)


    mujoco.mj_step(model, data)
    viewer.render()
    # time.sleep(5)

    i = i + 1
