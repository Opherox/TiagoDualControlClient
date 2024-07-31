# TiagoDualControlClient
C++ file to create control clients so that the different parts of the robot can be moved.

It has been created downloading the docker image of the official robot from Pal Robotics.

Code action client: Beware of the time to reach a waypoint, if no increment is done, it can give an error about the trajectory. 

1. Download zip
2. Unpack it
3. Source the docker setup.bash so that you have catkin.
4. Enter the workspace (tiago_movement_ws)
5. catkin_make
6. sudo rosdep init
7. sudo rosdep update
8. rospack depends1 tiago_movement  //check dependencies
9. rosrun tiago_movement move_tiago  //whenever connected to the robot or simulation is running

Connect to the real robot:

export ROS_MASTER_URI=http://<robot_ip>:11311

export ROS_IP=<computer_ip>

rosrun tiago_movement move_tiago

For some reason, I cant do rostopic echo /tf and it showing the information, but if I do ssh to the robot it does. That might have to do with the robot not doing what this code is supposed to make him do.
If you enter the robot, using ssh, it doesnt have rosdep.
