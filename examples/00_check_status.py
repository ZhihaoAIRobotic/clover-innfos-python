#!/bin/env python3
"""
    Basic example of connecting to Mintasca GLUON arm.

    GLUON is on network 192.168.1.0 by default. Please make sure that the 
    arm is connected to a network device on your computer and that the
    network device has a route to 192.168.1.0/24.

    For example: Set you ip address to 192.168.1.100 and your netmask to 255.255.255.0.

    This example connects to the arm, displays how much time it took to enable the arm,
    and prints a report of some of the arm parameters
"""

import time
import clover_innfos_python


arm = clover_innfos_python.ArmInterface()


print("Enabling arm...")

starttime = time.time()
arm.enableAllActuators()
endtime = time.time()

print("It took ",endtime-starttime," seconds to enable the arm")
print("\nActator config:")
print("\n",arm.report())