"""
Infoss Arm Controller Testing
----------------
Use PD control to control the arm to a desired position
"""

from isaacgym import gymapi
from isaacgym import gymutil
from isaacgym import gymtorch
from isaacgym.torch_utils import *

import math
import numpy as np
import torch
import random
import time
from scps import dynamics
from scps.ikpykinematics import Kinematics
from scipy.spatial.transform import Rotation as R


def joint_quat(q):
    kin = Kinematics()
    forward = kin.forward(q)
    rot = np.array([[*forward[0, 0:3]],
                    [*forward[1, 0:3]],
                    [*forward[2, 0:3]]])
    
    rotation = R.from_matrix(rot)
    
    quat = np.zeros(7).T
    quat[0:3] = forward[0:3, 3]
    quat[3:] = rotation.as_quat()
    
    return quat

def pos_control(self, q, qd, dq):
    self.qe = qd - q

    grav = dynamics.gravity(q)

    self.u = grav + self.kp * self.qe - self.kd * dq

    return self.u


# set random seed
np.random.seed(42)

torch.set_printoptions(precision=4, sci_mode=False)

# acquire gym interface
gym = gymapi.acquire_gym()


# parse arguments
custom_parameters = [
    {"name": "--controller", "type": str, "default": "ik",
     "help": "Controller to use for Franka. Options are {ik, osc}"},
    {"name": "--num_envs", "type": int, "default": 256, "help": "Number of environments to create"},
]
args = gymutil.parse_arguments(
    description="Franka Jacobian Inverse Kinematics (IK) + Operational Space Control (OSC) Example",
    custom_parameters=custom_parameters,
)

# set torch device
device = args.sim_device if args.use_gpu_pipeline else 'cpu'

# configure sim
sim_params = gymapi.SimParams()
sim_params.up_axis = gymapi.UP_AXIS_Z
sim_params.gravity = gymapi.Vec3(0.0, 0.0, -9.8)
sim_params.dt = 1.0 / 60.0
sim_params.substeps = 2
sim_params.use_gpu_pipeline = args.use_gpu_pipeline
if args.physics_engine == gymapi.SIM_PHYSX:
    sim_params.physx.solver_type = 1
    sim_params.physx.num_position_iterations = 8
    sim_params.physx.num_velocity_iterations = 1
    sim_params.physx.rest_offset = 0.0
    sim_params.physx.contact_offset = 0.001
    sim_params.physx.friction_offset_threshold = 0.001
    sim_params.physx.friction_correlation_distance = 0.0005
    sim_params.physx.num_threads = args.num_threads
    sim_params.physx.use_gpu = args.use_gpu
else:
    raise Exception("This example can only be used with PhysX")

# Set controller parameters
# IK params
damping = 0.05

# OSC params
kp = 150.
kd = 2.0 * np.sqrt(kp)
kp_null = 10.
kd_null = 2.0 * np.sqrt(kp_null)

# create sim
sim = gym.create_sim(args.compute_device_id, args.graphics_device_id, args.physics_engine, sim_params)
if sim is None:
    raise Exception("Failed to create sim")

# create viewer
viewer = gym.create_viewer(sim, gymapi.CameraProperties())
if viewer is None:
    raise Exception("Failed to create viewer")

asset_root = "/home/ubuntu/Github/clover-innfos-python"

# load franka asset
gluon_asset_file = "/Urdf/gluon.urdf"
asset_options = gymapi.AssetOptions()
asset_options.armature = 0.01
asset_options.fix_base_link = True
asset_options.disable_gravity = True
asset_options.flip_visual_attachments = True
gluon_asset = gym.load_asset(sim, asset_root, gluon_asset_file, asset_options)

# configure franka dofs
gluon_dof_props = gym.get_asset_dof_properties(gluon_asset)
gluon_lower_limits = gluon_dof_props["lower"]
gluon_upper_limits = gluon_dof_props["upper"]
gluon_ranges = gluon_upper_limits - gluon_lower_limits
gluon_mids = 0.3 * (gluon_upper_limits + gluon_lower_limits)

# use position drive for all dofs
gluon_dof_props["driveMode"][:6].fill(gymapi.DOF_MODE_POS)
gluon_dof_props["stiffness"][:6].fill(400.0)
gluon_dof_props["damping"][:6].fill(40.0)

# grippers
# franka_dof_props["driveMode"][7:].fill(gymapi.DOF_MODE_POS)
# franka_dof_props["stiffness"][7:].fill(800.0)
# franka_dof_props["damping"][7:].fill(40.0)

# default dof states and position targets
gluon_num_dofs = gym.get_asset_dof_count(gluon_asset)
default_dof_pos = np.zeros(gluon_num_dofs, dtype=np.float32)
default_dof_pos[:6] = gluon_mids[:6]
# grippers open
default_dof_pos[6:] = gluon_upper_limits[6:]

default_dof_state = np.zeros(gluon_num_dofs, gymapi.DofState.dtype)
default_dof_state["pos"] = default_dof_pos

# send to torch
default_dof_pos_tensor = to_torch(default_dof_pos, device=device)

# get link index of panda hand, which we will use as end effector
gluon_link_dict = gym.get_asset_rigid_body_dict(gluon_asset)
gluon_hand_index = gluon_link_dict["6_Link"]

# configure env grid
num_envs = args.num_envs
num_per_row = int(math.sqrt(num_envs))
spacing = 1.0
env_lower = gymapi.Vec3(-spacing, -spacing, 0.0)
env_upper = gymapi.Vec3(spacing, spacing, spacing)
print("Creating %d environments" % num_envs)

gluon_pose = gymapi.Transform()
gluon_pose.p = gymapi.Vec3(0, 0, 0)

box_pose = gymapi.Transform()

envs = []
box_idxs = []
hand_idxs = []
init_pos_list = []
init_rot_list = []

# add ground plane
plane_params = gymapi.PlaneParams()
plane_params.normal = gymapi.Vec3(0, 0, 1)
gym.add_ground(sim, plane_params)

for i in range(num_envs):
    # create env
    env = gym.create_env(sim, env_lower, env_upper, num_per_row)
    envs.append(env)

    # add franka
    gluon_handle = gym.create_actor(env, gluon_asset, gluon_pose, "gluon", i, 2)

    # set dof properties
    gym.set_actor_dof_properties(env, gluon_handle, gluon_dof_props)

    # set initial dof states
    gym.set_actor_dof_states(env, gluon_handle, default_dof_state, gymapi.STATE_ALL)

    # set initial position targets
    gym.set_actor_dof_position_targets(env, gluon_handle, default_dof_pos)

    # get inital hand pose
    hand_handle = gym.find_actor_rigid_body_handle(env, gluon_handle, "gluon_hand")
    hand_pose = gym.get_rigid_transform(env, hand_handle)
    init_pos_list.append([hand_pose.p.x, hand_pose.p.y, hand_pose.p.z])
    init_rot_list.append([hand_pose.r.x, hand_pose.r.y, hand_pose.r.z, hand_pose.r.w])

    # get global index of hand in rigid body state tensor
    hand_idx = gym.find_actor_rigid_body_index(env, gluon_handle, "gluon_hand", gymapi.DOMAIN_SIM)
    hand_idxs.append(hand_idx)

# point camera at middle env
cam_pos = gymapi.Vec3(4, 3, 2)
cam_target = gymapi.Vec3(-4, -3, 0)
middle_env = envs[num_envs // 2 + num_per_row // 2]
gym.viewer_camera_look_at(viewer, middle_env, cam_pos, cam_target)

# ==== prepare tensors =====
# from now on, we will use the tensor API that can run on CPU or GPU
gym.prepare_sim(sim)

# initial hand position and orientation tensors
init_pos = torch.Tensor(init_pos_list).view(num_envs, 3).to(device)
init_rot = torch.Tensor(init_rot_list).view(num_envs, 4).to(device)

# downard axis
down_dir = torch.Tensor([0, 0, -1]).to(device).view(1, 3)

# # get jacobian tensor
# # for fixed-base franka, tensor has shape (num envs, 10, 6, 9)
# _jacobian = gym.acquire_jacobian_tensor(sim, "franka")
# jacobian = gymtorch.wrap_tensor(_jacobian)
#
# # jacobian entries corresponding to franka hand
# j_eef = jacobian[:, franka_hand_index - 1, :, :7]

# get mass matrix tensor
_massmatrix = gym.acquire_mass_matrix_tensor(sim, "gluon")
mm = gymtorch.wrap_tensor(_massmatrix)
mm = mm[:, :6, :6]          # only need elements corresponding to the franka arm

# get rigid body state tensor
_rb_states = gym.acquire_rigid_body_state_tensor(sim)
rb_states = gymtorch.wrap_tensor(_rb_states)

# get dof state tensor
_dof_states = gym.acquire_dof_state_tensor(sim)
dof_states = gymtorch.wrap_tensor(_dof_states)
dof_pos = dof_states[:, 0].view(num_envs, 6, 1)
dof_vel = dof_states[:, 1].view(num_envs, 6, 1)

# Create a tensor noting whether the hand should return to the initial position
hand_restart = torch.full([num_envs], False, dtype=torch.bool).to(device)

# Set action tensors
pos_action = torch.zeros_like(dof_pos).squeeze(-1)
effort_action = torch.zeros_like(pos_action)

# simulation loop
while not gym.query_viewer_has_closed(viewer):

    # step the physics
    gym.simulate(sim)
    gym.fetch_results(sim, True)

    # refresh tensors
    gym.refresh_rigid_body_state_tensor(sim)
    gym.refresh_dof_state_tensor(sim)
    gym.refresh_jacobian_tensors(sim)
    gym.refresh_mass_matrix_tensors(sim)



    # Deploy actions
    gym.set_dof_position_target_tensor(sim, gymtorch.unwrap_tensor(pos_action))
    gym.set_dof_actuation_force_tensor(sim, gymtorch.unwrap_tensor(effort_action))

    # update viewer
    gym.step_graphics(sim)
    gym.draw_viewer(viewer, sim, False)
    gym.sync_frame_time(sim)

# cleanup
gym.destroy_viewer(viewer)
gym.destroy_sim(sim)