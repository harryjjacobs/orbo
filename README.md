# orbo
ROCO318 (Mobile and Humanoid Robotics) Coursework

Tested with ROS Kinetic and ROS Melodic.

[V-REP](http://www.coppeliarobotics.com/downloads) is required for this project.

# Installation

The following is the order in which the code in the workspaces should be build using `catkin_make`:
`drivers_ws`
`orbo_ws`
`orbo_simulation_ws`
`vrep_ros_interface_ws`

Then use `source source_all.bash` in order to source all of the workspaces.

After building the packages in the `vrep_ros_interface_ws` workspace copy `vrep_ros_interface_ws/devel/lib/libv_repExtRos.so` to the directory containing V-REP.

# Usage

Use `roslaunch op3_description orbo_rviz.launch` in order to start RVIZ with the orbo model

Then run V-REP using ./vrep.sh in the V-REP directory, and start the simulation to begin publishing the tf messages.