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
        if not key in self.__dict__ and key[-1] == 's':
            func_to_call = getattr(self,key[:-1])
            if callable(func_to_call):
                # Create a function that calls the function repeatedly
                def repeated(*args, **kwargs):
                    if len(args) > 0 and isinstance(args[0], (list,np.ndarray)):
                        return A([ func_to_call(i, j, *args[1:], **kwargs)  for i,j in zip(getattr(self,'actuator_id_list'), args[0]) ])
                    else:
                        return A([ func_to_call(i, *args, **kwargs)  for i in getattr(self,'actuator_id_list') ])
                
                return repeated
            else:
                pass # Drop to returning the full key

        return getattr(self._singleton_, key)


pi = np.pi
Degrees = 0.1 # Degrees_value*Degrees => Control_value, Control_value/Degrees => Degrees_value
Radians = 18.0/pi
innfos_position_units = 1.0
second = 1.0/60.0
minute = 1.0
innfos_velocity_units = 1.0

class Clover_GLUON(ActuatorControllerBase):


    scale_pos= {"Degrees": Degrees, "Radians": Radians, "Control": innfos_position_units}
    scale_vel= {"Minute": minute, "Second": second, "Control": innfos_position_units}

    def __init__(self, actuator_id_list=None):
        super().__init__()
        err = Actuator.ErrorsDefine(0) # Create object to store errors during actuator lookup
        jointlist = self.lookupActuators(err)
        if err != Actuator.ErrorsDefine(0):
            self.clearErrors()

        self.jointlist = jointlist
        
        if actuator_id_list:
            self.actuator_id_list = actuator_id_list
        else:
            self.actuator_id_list = [j.actuatorID for j in self.jointlist] # [2, 3, 5] => Aid[J0] = 2

        self.dof = len(self.actuator_id_list)

        self.joint_idxs = [None for i in range(max(self.actuator_id_list)+1)] # joint_idxs[actuatorid] = actuator_id_list.index(joint_index)
        for i, myid in enumerate(self.actuator_id_list):
            self.joint_idxs[myid] = self.actuator_id_list.index(myid)

        self.set_safety_values(max_acc=400, max_dec=-200, max_vel=500, min_pos=-9, max_pos=9)

    def set_safety_values(self, max_acc, max_dec, max_vel, min_pos, max_pos):
        for i in self.actuator_id_list:
            self.setProfilePositionAcceleration(i, max_acc)
            self.setProfilePositionDeceleration(i, -abs(max_dec))
            self.setProfilePositionMaxVelocity(i, max_vel)
            self.setMinimumPosition(i, min_pos)
            self.setMaximumPosition(i, max_pos)

    def home(self, position = [0,0,0,0,0,0]):
        """ Set the home (zero) position.
        """
        self.activateActuatorModeInBantch(self.jointlist, Actuator.ActuatorMode.Mode_Homing)
        self.setHomingPositions(position)
        time.sleep(0.1) # Without sleep, there is a race condition and homing may or may not fail
        self.activateActuatorModeInBantch(self.jointlist, Actuator.ActuatorMode.Mode_Profile_Pos)

    def getPositions(self,bRefresh=True, units = innfos_position_units):
        pos = A([self.getPosition(i,bRefresh=bRefresh) for i in self.actuator_id_list])
        return (pos)/units

    def setPositions(self, pos, units = innfos_position_units):
        pos = A(pos)[:self.dof]*units# Coerce pos to be np.array with self.dof elements
        return [self.setPosition(i,q) for i,q in zip(self.actuator_id_list,pos)]

    def report(self, pos_unit = innfos_position_units, vel_unit = innfos_velocity_units):
        def quickstring(func, unit=1.0):
            return f"{func.__name__}: {A([round(func(i,True),4) for i in self.actuator_id_list])/unit}"

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

import time
class ArmInterface(Clover_GLUON):

    def enableActuators(self):
        """
        enable the robot arm's actuators
        :param: None
        :return: None
        """
        self.enableAllActuators()

    def RadiansToControl(self, radians):
        """
        :param: radians: list/numpy array of 6 floats
        :return: control value: list/numpy array of 6 floats
        """
        return np.array(radians)*18/np.pi

    def DegreeToControl(self, degree):
        """
        :param: degree: list/numpy array of 6 floats
        :return: control value: list/numpy array of 6 floats
        """
        return np.array(degree)/10

    def ControlToDegree(self, control_value):
        """
        :param: degree: list/numpy array of 6 floats
        :return: control value: list/numpy array of 6 floats
        """
        return np.array(control_value) * 10

    def disableActuators(self):
        """
        enable the robot arm's actuators
        :param: None
        :return: None
        """
        self.disableAllActuators()

    def lookupActuators(self,*args):
        """
        lookup actuators
        :param: None
        :return: None
        """
        err = Actuator.ErrorsDefine(0)
        return super().lookupActuators(err)

    def setPositionMode(self):
        """
        set robot arm into position control mode
        :param: None
        :return: None
        """
        self.activateActuatorModeInBantch(self.jointlist, Actuator.ActuatorMode.Mode_Profile_Pos)

    def safePositionMode(self,max_acc=None, max_dec=None, max_vel=None, min_pos=None, max_pos=None):
        """
        set robot arm into safe position control mode
        :max_acc: Max acceleration in Degrees/minute**2 (scalar or array)
        :max_dec: Max deceleration in Degrees/minute**2 (scalar or array)
        :max_vel: Max velocity in Degrees/minute (scalar or array)
        :min_pos: Minimum position in Degrees (scalar or array)
        :max_pos: Maximum position in Degrees (scalar or array)
        :return: None
        """
        def coerce_to_array(x):
            if not isinstance(x, (tuple, list, np.ndarray)):
                x = [x for i in range(self.dof)]
            return np.array(x)

        if max_pos is not None:
            self.setMaximumPositions(coerce_to_array(max_pos)*Degrees)
        if min_pos is not None:
            self.setMinimumPositions(coerce_to_array(min_pos)*Degrees)
        if max_acc is not None:
            self.setProfilePositionAccelerations(coerce_to_array(max_acc*Degrees/minute**2))
        if max_dec is not None:
            self.setProfilePositionDecelerations(coerce_to_array( -abs(max_dec)*Degrees/minute**2))
        if max_vel is not None:
            self.setProfilePositionMaxVelocitys(coerce_to_array(max_vel*Degrees/minute))


    def setVelocityMode(self):
        """
        set robot arm into safe position control mode
        :param: None
        :return: None
        """
        self.activateActuatorModeInBantch(self.jointlist, Actuator.ActuatorMode.Mode_Profile_Vel)

    def getArmPosition(self):
        """
        get robot arm joint position
        :param: None
        :return: joint position
        """
        ret = self.getPositions(units=Degrees)
        return ret

    def getArmVelocity(self):
        """
        get robot arm joint velocity 
        :param: None
        :return: joint velocity
        """
        return self.getVelocitys(bRefresh=True) / (Degrees/minute)

    def setArmPosition(self, positions):
        """
        set robot arm joint position
        :param positions: list/numpy array of 6 floats
        :return: None
        """
        self.setPositions(np.array(positions),units=Degrees)

    def setArmVelocity(self, velocity):
        """
        set robot arm joint position
        :param positions: list/numpy array of 6 floats
        :return: None
        """
        velocity = np.array(velocity) * Degrees / minute
        self.setVelocitys(velocity)

    def joint_trajectory_tracking(self, trajectory_generator):
        """
        :param: trajectory_generator:
        :return:
        """
        init_time = time.time()
        time_list = []
        position_list = []
        velocity_list = []
        ideal_pos_list = []
        ideal_vel_list = []
        ideal_acc_list = []
        while self.__get_time(init_time) < trajectory_generator.t_via[-1]:
            t_now = self.__get_time(init_time)
            # pos = trajectory_generator.getPosition(t_now)
            tra = trajectory_generator.getTrajectory(t_now)
            self.setArmPosition(tra[:,0])

            time_list.append(t_now)
            ArmPosition = self.getArmPosition()
            arm_velocity = self.getArmVelocity()
            position_list.append(ArmPosition[1])
            velocity_list.append(arm_velocity[1])
            ideal_pos_list.append(tra[1, 0])
            ideal_vel_list.append(tra[1, 1])
            ideal_acc_list.append(tra[1, 2])
            # time.sleep(0.0001)
        return time_list, position_list, velocity_list, ideal_pos_list, ideal_vel_list, ideal_acc_list

    def joint_trajectory_tracking_vel(self, trajectory_generator):
        """
        :param: trajectory_generator:
        :return:
        """
        init_time = time.time()
        time_list = []
        position_list = []
        velocity_list = []
        ideal_pos_list = []
        ideal_vel_list = []
        ideal_acc_list = []
        while self.__get_time(init_time) < trajectory_generator.t_via[-1]:
            t_now = self.__get_time(init_time)
            # pos = trajectory_generator.getPosition(t_now)
            tra = trajectory_generator.getTrajectory(t_now)
            self.setArmVelocity(tra[:,1])
            time_list.append(t_now)
            ArmPosition = self.getArmPosition()
            arm_velocity = self.getArmVelocity()
            position_list.append(ArmPosition[4])
            velocity_list.append(arm_velocity[4])
            # print(tra)
            ideal_pos_list.append(tra[4, 0])
            ideal_vel_list.append(tra[4, 1])
            ideal_acc_list.append(tra[4, 2])
            # time.sleep(0.0001)
        return time_list, position_list, velocity_list, ideal_pos_list, ideal_vel_list, ideal_acc_list


    def __get_time(self, init_time):
        now_time = time.time()
        t = (now_time - init_time)
        return t

    def goHome(self):
        """
        :param: None
        :return: None
        """
        tolerance = 0.1
        
        self.setArmPosition([0, 0, 0, 0, 0, 0])

        while True:
            time.sleep(0.01)
            if (np.absolute(np.array(self.getArmPosition()))< tolerance).all():
                break
