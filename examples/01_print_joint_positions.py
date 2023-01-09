#!/bin/env python3
"""
    Basic example of displaying Mintasca GLUON arm position.
"""

import time
import clover_innfos_python


arm = clover_innfos_python.ArmInterface()

arm.enableAllActuators()

# Infinite loop should be inside try/finally so we can
# correctly disable the actuators at the end
try:
    while(1):
        print("( CTRL+C to stop) Position:", arm.getArmPosition(), " (Degrees) ")

# After Keyboardinterrupt exception, disable the arm
finally:
    arm.disableAllActuators()
