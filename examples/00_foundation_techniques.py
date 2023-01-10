#!/env python3
"""
    Illustrated examples of the python programming techniques that are used in
    this code base.
"""
import numpy as np
from numpy import pi

## Units ##
"""
    We use the direct expression method of integrating units into our calculations.
    Multiplying by a units constant gives the value in the natural unit.
    Dividing by a units constant gives the value in the units of the constant.
"""
## Units Constants ##
# Natural position units are 0.1 Degrees
Innfos_length_units = 1.0
Degrees = 0.1
Radians = 18.0/pi
Rotation = 36.0
second = 1/60.0
minute = 1.0

desired_position = 90*Degrees # => 9.0 Innfos_length_units
value_in_degrees = desired_position / Degrees # => 90
value_in_radians = desired_position / Radians # => pi/2

desired_velocity = 10*Degrees/second # => 60.0 innfos_length_units / minute
value_in_RPM = desired_velocity / (Rotation/minute)
value_in_RadPS = desired_velocity / (Radians/second)


## numpy shorthand ##
"""
    vector and matrix math are done using numpy. We use some conventions and 
    shorthand to make the code a bit cleaner.
"""

# numpy array building function
def A(*args):
    # A(np.ndarray) => np.ndarray
    if len(args)==1 and isinstance(args[0],(list,np.ndarray)): return np.array(args[0])
    # A(1, 0) => np.array([1,0]), A([1,2],[3,4]) => np.array([[1,2],[3,4]])
    return np.array([arg for arg in args])

print(f"""
A(1,2,3) => {A(1,2,3)}

A([1,2],[3,4]) => {A([1,2],[3,4])}

A(np.array([[1,2],[3,4]])) => {A(np.array([[1,2],[3,4]]))}

A(A(1,2),A(3,4)) => {A(A(1,2),A(3,4))}
""")

# arm functions will accept a list or np.array
"""
`arm.setArmPosition([0,0,0,0,0,0])`
`arm.setArmPosition(np.zeros(6))
"""