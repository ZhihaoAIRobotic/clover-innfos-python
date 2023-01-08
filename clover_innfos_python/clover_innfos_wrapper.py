
class ActuatorControllerBase(object):

    def __new__(cls):
        obj = super().__new__(cls)
        obj._singleton_ = ActuatorController.initController()
        return obj
        
    # Make sure we have __dict__ keys for functions that end in `s`, see __getattr__ below
    def lookupActuators(self,*args): return [UID()]#self._singleton_.lookupActuators(*args)
    def processEvents(self): self._singleton_.processEvents()
    def enableAllActuators(self): return self._singleton_.enableAllActuators()
    def disableAllActuators(self): return self._singleton_.disableAllActuators()

    def __getattr__(self, key):
        """!
        @brief Redirect attribute access to the _sigleton_. Autoexpand member function calls that end in 's' into multiple calls
        to base functions.

        @param key (str): attribute name

        @return _type_: attribute (or proxy function if key ends in 's')
        """
        if key[-1] == 's':
            func_to_call = getattr(self,key[:-1])
            if callable(func_to_call):
                # Create a function that calls the function repeatedly
                def repeated(ids, *args, **kwargs):
                    return [ func_to_call(i, *args, **kwargs)  for i in ids ]
                
                return repeated
            else:
                pass # Drop to returning the full key

        return getattr(self._singleton_, key)

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
