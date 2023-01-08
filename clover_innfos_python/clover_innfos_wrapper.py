from clover_ActuatorController import Actuator, ActuatorController
import numpy as np

A = np.array

class ActuatorControllerBase(object):

    def __new__(cls):
        obj = super().__new__(cls)
        obj._singleton_ = ActuatorController.initController()
        return obj
        
    # Make sure we have __dict__ keys for functions that end in `s`, see __getattr__ below
    def lookupActuators(self,*args): return self._singleton_.lookupActuators(*args)
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
                def repeated(*args, **kwargs):
                    if len(args) > 0 and isinstance(args[0], list):
                        return A([ func_to_call(i, j, *args[1:], **kwargs)  for i,j in zip(self.actuator_ids, args[0]) ])
                    else:
                        return A([ func_to_call(i, *args, **kwargs)  for i in self.actuator_ids ])
                
                return repeated
            else:
                pass # Drop to returning the full key

        return getattr(self._singleton_, key)

class Clover_GLUON(ActuatorControllerBase):

    def __init__(self, actuator_id_list=None):
        err = Actuator.ErrorsDefine(0) # Create object to store errors during actuator lookup
        jointlist = self.lookupActuators(err)
        if err != Actuator.ErrorsDefine(0):
            raise Exception("Failed to lookup actuators! Error: ",err)
        self.jointlist = jointlist
        
        if actuator_id_list:
            self.actuator_ids = actuator_id_list
        else:
            self.actuator_ids = [j.actuatorID for j in self.jointlist] # [2, 3, 5] => Aid[J0] = 2

        self.dof = len(self.actuator_ids)

        self.joint_idxs = [None for i in range(max(self.actuator_ids)+1)] # joint_idxs[actuatorid] = actuator_ids.index(joint_index)
        for i, myid in enumerate(self.actuator_ids):
            self.joint_idxs[myid] = self.actuator_ids.index(myid)

        # for i in self.actuator_ids:
        #     self.setPositionOffset(i,0.0)

        self.home_position = A([0,0,0,0,0,0])[:self.dof]  # Keep track of what our home position is
        self.set_safety_values(max_acc=400, max_dec=-200, max_vel=500, min_pos=9, max_pos=9)

    def set_safety_values(self, max_acc, max_dec, max_vel, min_pos, max_pos):
        for i in self.actuator_ids:
            self.setProfilePositionAcceleration(i, max_acc)
            self.setProfilePositionDeceleration(i, -abs(max_dec))
            self.setProfilePositionMaxVelocity(i, max_vel)
            self.setMinimumPosition(i, min_pos)
            self.setMaximumPosition(i, max_pos)

    def home(self):
        self.home_position = A([self.getPosition(i,bRefresh=True) for i in self.actuator_ids])

    def getPositions(self,bRefresh=True):
        pos = A([self.getPosition(i,bRefresh=bRefresh) for i in self.actuator_ids])
        # print('getPosition', -self.home_position+pos)
        return pos-self.home_position

    def setPositions(self, pos):
        pos = A(pos)[:self.dof]
        # print('setPosition', self.home_position+pos,[(i,q) for i,q in zip(self.actuator_ids,self.home_position + pos)])
        return [self.setPosition(i,q) for i,q in zip(self.actuator_ids,self.home_position + pos)]

    def report(self):
        def quickstring(func):
            return f"{func.__name__}: {[round(func(i,True),4) for i in self.actuator_ids]}"

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
