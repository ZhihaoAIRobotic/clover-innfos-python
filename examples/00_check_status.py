#!/bin/env python3
"""
    Basic example of connecting to Mintasca GLUON arm.

    GLUON is on network 192.168.1.0 by default. Please make sure that the 
    arm is connected to a network device on your computer and that the
    network device has a route to 192.168.1.0/24.

    For example: Set your ip address to 192.168.1.100 and your netmask to 255.255.255.0.

    This example connects to the arm, displays how much time it took to enable the arm,
    and prints a report of some of the arm parameters

    READ THE CODE CAREFULLY as it illustrates basic python techniques used throughout
    these tutorials.
"""

import time
import clover_innfos_python

actuator_ids =  [1,2,4,7,6] 
arm = clover_innfos_python.ArmInterface( actuator_ids )


print("Enabling arm...")

starttime = time.time()
arm.enableAllActuators()
endtime = time.time()

# We use formatted string literals: 
# https://docs.python.org/3/reference/lexical_analysis.html#formatted-string-literals

print(f"It took {endtime-starttime} seconds to enable the arm!\n")

input("Hit <ENTER> to continue")



# Naturally when programming in python we use zero based counting
# https://en.wikipedia.org/wiki/Zero-based_numbering
print(f"""
The arm has {arm.dof} joints. So the joint 1 has index 0 and 
Joint {arm.dof} has index {arm.dof-1}

""")

input("Hit <ENTER> to continue")



print(f"""
Each actuator has an ID number assigned by the manufacturer.
Actuator IDs do not necessarily match joint numbers and the
clover_innfos_python library has no way to know this. Presently
the `ArmInterface` object has the following mapping:

""")
for i,a in enumerate(arm.actuator_id_list):
    print(f"J{i+1} (index {i}) => actuator_id {a}" )

input("Hit <ENTER> to continue")

print(f"""
One can configure the mapping of actuator ids to joint indexes
by passig in a list of actuator ids to `ArmInterface`. For example,
if there are 5 actuators with ids [1,2,4,7,6] starting from the first
joint, one would establish a connection with the arm using:

```
actuator_ids =  [1,2,4,7,6] 
arm = clover_innfos_python.ArmInterface( actuator_ids )
```
""")

print(f"""
Actuator config:

{arm.report()}

""")

