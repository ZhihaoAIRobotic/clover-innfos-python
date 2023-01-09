from pyPS4Controller.controller import Controller
import time
from clover_innfos_python import *

#arm = Clover_GLUON()
arm = ArmInterface()

class MyController(Controller):

    def __init__(self, **kwargs):
        Controller.__init__(self, **kwargs)

    def on_x_press(self):
       print("Hello world")

    def on_x_release(self):
       print("Goodbye world")

    def on_timer(self):
        print("timer func2")

# controller = MyController(interface="/dev/input/js0", connecting_using_ds4drv=False)
# controller.wait_for_interface()
# controller.open()

# while(1):
#     # Process events that are 0.001 seconds apart all together
#     while(event := controller.poll_events(0.001)):
#         controller.process_event(event)

#     print('doing something')
#     time.sleep(0.05)



# Enable all
print("Enabling arm...")

start = time.time()

arm.enableAllActuators()
arm.activateActuatorModeInBantch(arm.jointlist, Actuator.ActuatorMode.Mode_Cur)


endtime = time.time()

print("It took ",endtime-start," seconds to enable the arm")
print([arm.getActuatorMode(i) for i in [1,2,3,4,6,7]])
# [arm.setPositionKi(i,0.01) for i in arm.actuator_ids]

# arm.clearHomingInfo(arm.actuator_ids[0])
# arm.setPositionOffset(arm.actuator_ids[0], 0)
# arm.setMaximumPosition(arm.actuator_ids[0], 6)
# arm.setMinimumPosition(arm.actuator_ids[0], -6)
input("Move arm to zero")
arm.home()
print("\n",arm.report())
print('Home position is: ',arm.home_position)
print("\n\nPosition is ",arm.getPositions(bRefresh=True),'\n')

print(arm.actuator_ids)


input("Move arm to start")
arm.activateActuatorModeInBantch(arm.jointlist, Actuator.ActuatorMode.Mode_Profile_Pos)
startpos = arm.getPositions(bRefresh=True)
print("\n\nInitialPosition is ",startpos,'\n')

print("Going to zero")
res = arm.setPositions([0,0,0,0,0,0])

time.sleep(5)

print("\n\nPosition at zero is ",arm.getPositions(bRefresh=True),'\n')

print("Going home")
arm.setPositions(startpos)
time.sleep(0.2)
print("\n\Velocity at going home is ",arm.getVelocitys(bRefresh=True),'\n')

time.sleep(5)
print("\n\nPosition at start is",arm.getPositions(bRefresh=True),'\n')
start = time.time()
arm.disableAllActuators()
endtime = time.time()
print("It took ",endtime-start," seconds to disable the arm")