## Models

This package contains classes holding all of the information
specific to each kind of robot that StarL can interface with
and simulate. Most of the work required to integrate a new 
robot into StarL is done here, without having to modify any
external classes.

### Class Model

`Model` is the base class for all types of robots, and is
used widely to represent the "current robot." It provides
abstract methods for characteristics that all types of robots
share, including a method to return the type of physical
interface to be used to control each robot.

### Class Model_Ground

`Model_Ground` is the base class for non-aerial robots that 
have zero-point turning, i.e. a forward velocity and an angle.
