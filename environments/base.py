from mujoco_py import MjSim
import mujoco_py

class Base_env(object):
    def __init__(self,sim):
        self.sim = sim
        self.view = mujoco_py.MjViewer(self.sim)

    def render(self, *args):
        self.view.render()

    def reset(self, *args):
        self.sim.reset()

    def step(self, *args):
        self.sim.step()


