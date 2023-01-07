from clover_innfos_python import *
import time
arm = Clover_GLUON()

# Enable all
print("Enabling arm...")
start = time.time()

arm.enableAllActuators()
err = Actuator.ErrorsDefine(0)
#jointlist = arm.lookupActuators(err)
#jointids = [j.actuatorID for j in jointlist]
arm.activateActuatorModeInBantch(arm.jointlist, Actuator.ActuatorMode.Mode_Cur)
if err != Actuator.ErrorsDefine(0):
    print("Error: ",err)
    arm.disableAllActuators()
    exit()
    
endtime = time.time()

print("It took ",endtime-start," seconds to enable the arm")
print([arm.getActuatorMode(i) for i in [1,2,3,4,6,7]])
time.sleep(2)

getAllPositions = lambda self: [self.getPosition(i,bRefresh=True) for i in arm.jointids]

for x in range(1000):
    pos = getAllPositions(arm)
    print(pos )

start = time.time()
arm.disableAllActuators()
endtime = time.time()
print("It took ",endtime-start," seconds to disable the arm")