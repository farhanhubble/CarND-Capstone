# System Integration Project
This is the project repo for the final project of the Udacity Self-Driving Car Nanodegree: Programming a Real Self-Driving Car.
The goal of this project is to integrate different modules like trajectory planning, traffic light detection and drive-by-wire controller (DBW). These modules are implemented as [ROS](http://www.ros.org/) nodes. The DBW node generates mesages that can either control a simulator or Udacity's own self-driving car Carla.

## Implementation Status
This is a team project. I have (partially) implemented the waypoint updater node and the DBW node. The implementations has been tested on the simulator and the car is able to drive follwoing the waypoints and the DBW control commands. Since traffic light node is not yet implemented the car does not stop at red lights.
