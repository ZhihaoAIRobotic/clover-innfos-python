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

        for i in self.jointids:
            self.setPositionOffset(i,0.0)

        self.home_position = [0,0,0,0,0,0]

    def set_safety_values(self, max_acc, max_dec, max_vel, min_pos, max_pos):
        self.setProfilePositionAcceleration(max_acc)
        self.setProfilePositionDeceleration(-abs(max_dec))
        self.setProfilePositionMaxVelocity(max_vel)
        self.setMinimumPosition(min_pos)
        self.setMaximumPosition(max_pos)

    def home(self):
        self.home_position = self.getPositions()

    def getPositions(self):
        return [self.getPosition(i,bRefresh=True) for i in self.jointids]

    def setPositions(self, pos):
        return [self.setPosition(i,self.home_position[self.jointids.index(i)]+q) for i,q in zip(self.jointids, pos)]

    def report(self):
        def quickstring(func):
            return f"{func.__name__}: {[round(func(i,True),4) for i in self.jointids]}"
        nl = "\n"
        s = ""
        s += quickstring(self.getPosition) + nl
        s += quickstring(self.getPositionUmax) + nl
        s += quickstring(self.getPositionUmin) + nl
        s += quickstring(self.getPositionOffset) + nl
        s += quickstring(self.getMaximumPosition) + nl
        s += quickstring(self.getMinimumPosition) + nl
        s += quickstring(self.getPositionKp) + nl
        s += quickstring(self.getPositionKd) + nl
        s += quickstring(self.getPositionKi) + nl
        s += quickstring(self.getPositionCutoffFrequency) + nl
        s += quickstring(self.getProfilePositionAcceleration) + nl
        s += quickstring(self.getProfilePositionDeceleration) + nl
        s += quickstring(self.getProfilePositionMaxVelocity) + nl
        s += quickstring(self.getVelocityKp) + nl
        s += quickstring(self.getVelocityKi) + nl
        s += quickstring(self.getVelocityUmax) + nl
        s += quickstring(self.getVelocityUmin) + nl
        s += quickstring(self.getVelocityRange) + nl
        s += quickstring(self.getVelocityLimit) + nl
        s += quickstring(self.getProfileVelocityAcceleration) + nl
        s += quickstring(self.getProfileVelocityDeceleration) + nl
        return s
