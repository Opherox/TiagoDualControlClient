# TiagoDualControlClient
C++ file to create control clients so that the different parts of the robot can be moved.

It has been created downloading the docker image of the official robot from Pal Robotics.

Code action client: Beware of the time to reach a waypoint, if no increment is done, it can give an error about the trajectory. 

Download zip, unpack it, source the docker setup.bash so that you have catkin. Then enter the workspace (tiago_movement_ws) and do catkin_make
