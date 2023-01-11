

from clover_ActuatorController import Actuator, ActuatorController
import numpy as np
import sys

arm = ActuatorController.initController()

err = Actuator.ErrorsDefine(0) # Create object to store errors during actuator lookup
jointlist = arm.lookupActuators(err)

arm.enableAllActuators()

arm.activateActuatorModeInBantch(jointlist, Actuator.ActuatorMode.Mode_Cur)
arm.setVelocityKi(1,0)
arm.setVelocityKp(1,0)

arm.mydata = 20
print(arm.mydata)

while(1):
    pass