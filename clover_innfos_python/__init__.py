from clover_ActuatorController import Actuator, ActuatorController
__all__ = ["Actuator", "Clover_GLUON"]

class ActuatorControllerBase(object):

    def __new__(cls):
        obj = super().__new__(cls)
        obj._singleton_ = ActuatorController.initController()
        return obj
        
    def __getattr__(self, key):
        return getattr(self._singleton_, key )

class Clover_GLUON(ActuatorControllerBase):

    def __init__(self):
        err = Actuator.ErrorsDefine(0) # Create object to store errors during actuator lookup
        jointlist = self.lookupActuators(err)
        if err != Actuator.ErrorsDefine(0):
            raise Exception("Failed to lookup actuators! Error: ",err)
        self.jointlist = jointlist
        self.jointids = [j.actuatorID for j in self.jointlist]

    def set_safety_values(self, max_acc, max_dec, max_vel, min_pos, max_pos):
        self.setProfilePositionAcceleration(max_acc)
        self.setProfilePositionDeceleration(-abs(max_dec))
        self.setProfilePositionMaxVelocity(max_vel)
        self.setMinimumPosition(min_pos)
        self.setMaximumPosition(max_pos)
