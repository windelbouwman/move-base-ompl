move-base-ompl
==============

ROS move-base planner plugin using ompl

[![Build Status](https://travis-ci.org/windelbouwman/move-base-ompl.svg)](https://travis-ci.org/windelbouwman/move-base-ompl)

About
-----

The move-base package for ROS contains several planners for 2D navigation
of moving base. This package provides two plugins to expose the routing
capabilities of the OMPL library.

Attention: This work is very experimental and pull-requests and suggestions are welcome!

Usage
-----

To use this planner, check it out in your local catkin workspace:

    $ cd ~/catkin_ws/src
    $ git clone https://github.com/windelbouwman/move-base-ompl.git
    $ cd ..
    $ catkin_make

Now the planner should be placed as shared object in the devel/lib folder. Now you can use in 
together with the move_base node.

List the plugin with the command:

    $ rospack plugins --attrib=plugin nav_core


Then use the plugin in your move_base_parameters.yaml file:

    base_global_planner: ompl_global_planner/OmplGlobalPlanner

