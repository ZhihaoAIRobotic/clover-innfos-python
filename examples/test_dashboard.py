import time
import sys

import numpy as np
from scipy import signal
from clover_innfos_python import *


if __name__ == '__main__':

    np.set_printoptions(precision=4,floatmode='fixed',suppress=True)
    actuator_ids =  [1,2,4,7,6] # J1=id(1) ... J5=id(6)
    arm = ArmInterface(actuator_ids)

    def arm_sequence(*args):
        """ Sequence of operations on the arm to run """
        arm = args[0]
        print("Enabling arm...")
        start = time.time()
        arm.enableAllActuators()
        endtime = time.time()
        print("It took ",endtime-start," seconds to enable the arm")
        arm.activateActuatorModeInBantch(arm.jointlist, Actuator.ActuatorMode.Mode_Cur)

        input("Move to zero")
        arm.home() # Will set position to profile mode

        arm.safePositionMode(max_vel=30*60,min_pos=-360,max_pos=+360)

        arm.setPositionMode()
        jidx = 0
        trigger_current = 0.5
        search_limit = 360
        target = np.zeros(arm.dof)

        def find_limits(arm, jidx, search_limit, trigger_current):
            target[jidx] = search_limit
            print("Moving to ",target)
            arm.setArmPosition(target)
            while(current:=abs(arm.getCurrents(True)[jidx]) < trigger_current):
                lastpos = arm.getArmPosition()
            # Reset position mode (remove profile ramp and integrator windup) and go the other direction
            arm.setPositionMode()
            
            print('positive stop position:',lastpos)
            target[jidx] = -search_limit
            arm.setArmPosition(target)

            time.sleep(2) # Sleep a little to wait for direction change

            pos_stop = lastpos[jidx]
            while(current:=abs(arm.getCurrents(True)[jidx]) < trigger_current):
                lastpos = arm.getArmPosition()
            arm.setPositionMode()
            print(arm.getArmPosition())

            print('negative stop position:',lastpos)
            neg_stop = lastpos[jidx]
            
            
            target[jidx] = (neg_stop+pos_stop)/2
            arm.setArmPosition(target)

            # Wait until we get to our target
            while((pos:=np.abs(arm.getArmPosition()-A(target)) > 0.1).any()):
                pass
                

            arm.home() # Will set position to profile mode
            return (neg_stop, pos_stop)

        j0_limit = find_limits(arm, 0, 360, 0.5)
        print("j0 limit was",j0_limit)


    run_with_dashboard(arm_sequence, arm)




    