move-base-ompl
==============

ROS move-base planner plugin using ompl

About
-----

The move-base package for ROS contains several planners for 2D navigation
of moving base. This package provides two plugins to expose the routing
capabilities of the OMPL library.

Usage
-----

To use this planner, check it out in your local catkin workspace:

$ cd ~/catkin_ws/src
$ git clone https://github.com/windelbouwman/move-base-ompl.git
$ cd ..
$ catkin_make

Now the planner should be placed as shared object in the devel/lib folder. Now you can use in 
together with the move_base node.


