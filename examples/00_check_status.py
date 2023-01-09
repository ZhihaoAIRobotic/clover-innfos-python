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

actuator_ids =  [1,2,4,7,6] # J1=id(1) ... J5=id(6)
arm = clover_innfos_python.ArmInterface( actuator_ids )


print("Enabling arm...")

starttime = time.time()
arm.enableAllActuators()
endtime = time.time()

print("It took ",endtime-starttime," seconds to enable the arm")
print("\nActator config:")
print("\n",arm.report())
print("\n\n")
print("(Joint index, actuator index):",[(i+1,a) for i,a in enumerate(arm.actuator_id_list)])
