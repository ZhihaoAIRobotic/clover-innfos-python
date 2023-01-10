#!/bin/env python3
"""
    Basic example of displaying Mintasca GLUON arm position.
"""

import time
import clover_innfos_python
import numpy as np

np.set_printoptions(linewidth=120) # Print positions all on one line

actuator_ids =  [1,2,4,7,6] # J1=id(1) ... J5=id(6)
arm = clover_innfos_python.ArmInterface(actuator_ids)

arm.enableAllActuators()


while(1):
    print("( CTRL+C to stop) Position:", arm.getArmPosition(), " (Degrees) ")
    time.sleep(0.1)



