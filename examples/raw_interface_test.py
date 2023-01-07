from clover_ActuatorController import Actuator, ActuatorController
import time

arm: ActuatorController = ActuatorController.initController()
err = Actuator.ErrorsDefine(0) # Create object to store errors
jointlist = arm.lookupActuators(err)

print(err)
jointids = [j.actuatorID for j in jointlist]

# Enable all
print("Enabling arm...")
start = time.time()

arm.enableAllActuators()
arm.activateActuatorModeInBantch(jointlist, Actuator.ActuatorMode.Mode_Vel)
endtime = time.time()

print("It took ",endtime-start," seconds to enable the arm")
print([arm.getActuatorMode(i) for i in [1,2,3,4,6,7]])
time.sleep(2)

getAllPositions = lambda self: [self.getPosition(i,bRefresh=True) for i in jointids]

for x in range(10):
    pos = getAllPositions(arm)
    print([arm.getPosition(i,bRefresh=True) for i in jointids] )
    #time.sleep(0.1)

start = time.time()
arm.disableAllActuators()
endtime = time.time()
print("It took ",endtime-start," seconds to disable the arm")