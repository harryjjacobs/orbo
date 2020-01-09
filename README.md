# orbo
ROCO318 (Mobile and Humanoid Robotics) Coursework

Tested with ROS Kinetic and ROS Melodic.

[V-REP](http://www.coppeliarobotics.com/downloads) is required for this project.

# Installation

The following is order in which the packages should be build using `catkin_make`:
`orbo_drivers`
`orbo_ws`
`orbo_simulation`
`vrep_ros_interface`

Then use `source source_all.bash` in order to source all of the workspaces.

After building the packages in the `vrep_ros_interface` workspace copy `vrep_ros_interface/devel/lib/libv_repExtRos.so` to the directory containing V-REP.

# Usage

Use `roslaunch op3_description orbo_rviz.launch` in order to start RVIZ with the orbo model

Then run V-REP using ./vrep.sh in the V-REP directory, and start the simulation to begin publishing the tf messages.