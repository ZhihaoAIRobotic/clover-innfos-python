import time
import clover_innfos_python
from clover_innfos_python import Actuator

actuator_ids =  [1,2,4,7,6] 
arm = clover_innfos_python.ArmInterface( actuator_ids )

arm.enableAllActuators()

arm.activateActuatorModeInBantch(arm.jointlist, Actuator.ActuatorMode.Mode_Cur)
arm.setVelocityKis([0,0,0,0,0,0])
arm.setVelocityKps([1,1,1,1,1,1])
print("Ready")
while(1):
    pass

