# manipulator_controllers

Warning: impedance control doesn't work on the UR5e ros2 driver yet. I think the issue is the controller runs for a while before the robot moves, and the force torque sensor is constantly being read before then, this sensor compounds up error in the motion/state of the controller before the robot is even running. Then when you startup the robot it jumps somewhere randomly. Try to figure out way to make it so that you can start robot/controller at same time.

The main issue is that for ur driver you start robot by clicking start robot control on external control urcap, but before then the robot controller on ROS2 is already activated and running (this didn't happen in the ROS1 version).
