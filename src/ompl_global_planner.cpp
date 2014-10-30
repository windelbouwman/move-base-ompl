/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 * Modificator: Windel Bouwman
 *********************************************************************/

/*

    Greatly copied from:

    http://ompl.kavrakilab.org/RigidBodyPlanningWithControls_8cpp_source.html

*/

#include <ompl_global_planner.h>
#include <pluginlib/class_list_macros.h>
#include <tf/transform_listener.h>


//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(ompl_global_planner::OmplGlobalPlanner, nav_core::BaseGlobalPlanner)

namespace ompl_global_planner
{


OmplGlobalPlanner::OmplGlobalPlanner() :
        _costmap_ros(NULL), _initialized(false), _allow_unknown(true),
        _space(new ob::SE2StateSpace()),
        _costmap_model(NULL)
{
}

void OmplGlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
    if (!_initialized)
    {
        ros::NodeHandle private_nh("~/" + name);
        _costmap_ros = costmap_ros;
        _frame_id = "map";
        _costmap_model = new base_local_planner::CostmapModel(*_costmap_ros->getCostmap());

        _plan_pub = private_nh.advertise<nav_msgs::Path>("plan", 1);
        private_nh.param("allow_unknown", _allow_unknown, true);

        //get the tf prefix
        ros::NodeHandle prefix_nh;
        tf_prefix_ = tf::getPrefixParam(prefix_nh);

        _initialized = true;
        ROS_INFO("Ompl global planner initialized!");
    }
    else
    {
        ROS_WARN("This planner has already been initialized, you can't call it twice, doing nothing");
    }

}

// Check the current state:
bool OmplGlobalPlanner::isStateValid(const oc::SpaceInformation *si, const ob::State *state)
{
    const ob::SE2StateSpace::StateType *se2state = state->as<ob::SE2StateSpace::StateType>();
    const double* pos = se2state->as<ob::RealVectorStateSpace::StateType>(0)->values;
    const double rot = se2state->as<ob::SO2StateSpace::StateType>(1)->value;

    if (!si->satisfiesBounds(state))
    {
        return false;
    }

    // Get the cost of the footprint at the current location:
    double cost = _costmap_model->footprintCost(pos[0], pos[1], rot, _costmap_ros->getRobotFootprint());

    // std::cout << cost << std::endl;
    // Too high cost:
    if (cost > 240)
    {
        return false;
    }

    // Error? Unknown space?
    if (cost < 0)
    {
    }

    return true;
}

void OmplGlobalPlanner::propagate(const ob::State *start, const oc::Control *control, const double duration, ob::State *result)
{
    // Implement vehicle dynamics:
    const ob::SE2StateSpace::StateType *se2state = start->as<ob::SE2StateSpace::StateType>();
    const double* pos = se2state->as<ob::RealVectorStateSpace::StateType>(0)->values;
    const double rot = se2state->as<ob::SO2StateSpace::StateType>(1)->value;
    const double* ctrl = control->as<oc::RealVectorControlSpace::ControlType>()->values;

    // TODO: elaborate further..
    result->as<ob::SE2StateSpace::StateType>()->setX(pos[0] + ctrl[0] * duration * cos(rot));
    result->as<ob::SE2StateSpace::StateType>()->setY(pos[1] + ctrl[0] * duration * sin(rot));

    double vehicle_length = 0.4;
    double lengthInv = 1 / vehicle_length;
    double omega = ctrl[0] * lengthInv * tan(ctrl[1]);
    result->as<ob::SE2StateSpace::StateType>()->setYaw( rot + omega * duration );

    // Make sure angle is (-pi,pi]:
    const ob::SO2StateSpace* SO2 = _space->as<ob::SE2StateSpace>()->as<ob::SO2StateSpace>(1);
    ob::SO2StateSpace::StateType* so2 = result->as<ob::SE2StateSpace::StateType>()->as<ob::SO2StateSpace::StateType>(1);
    SO2->enforceBounds(so2);
}


bool OmplGlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                           std::vector<geometry_msgs::PoseStamped>& plan)
{
    boost::mutex::scoped_lock lock(_mutex);
    if (!_initialized) {
        ROS_ERROR(
                "This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return false;
    }

    //clear the plan, just in case
    plan.clear();

    ros::NodeHandle n;
    std::string global_frame = _frame_id;

    //until tf can handle transforming things that are way in the past... we'll require the goal to be in our global frame
    if (tf::resolve(tf_prefix_, goal.header.frame_id) != tf::resolve(tf_prefix_, global_frame)) {
        ROS_ERROR(
                "The goal pose passed to this planner must be in the %s frame.  It is instead in the %s frame.", tf::resolve(tf_prefix_, global_frame).c_str(), tf::resolve(tf_prefix_, goal.header.frame_id).c_str());
        return false;
    }

    if (tf::resolve(tf_prefix_, start.header.frame_id) != tf::resolve(tf_prefix_, global_frame)) {
        ROS_ERROR(
                "The start pose passed to this planner must be in the %s frame.  It is instead in the %s frame.", tf::resolve(tf_prefix_, global_frame).c_str(), tf::resolve(tf_prefix_, start.header.frame_id).c_str());
        return false;
    }

    double wx = start.pose.position.x;
    double wy = start.pose.position.y;

    //clear the starting cell within the costmap because we know it can't be an obstacle
    tf::Stamped<tf::Pose> start_pose;
    tf::poseStampedMsgToTF(start, start_pose);

    ROS_INFO("Thinking about OMPL path..");
    // Create OMPL problem:
    ob::RealVectorBounds bounds(2);
    bounds.setLow(-10);
    bounds.setHigh(10);
    _space->as<ob::SE2StateSpace>()->setBounds(bounds);

    oc::ControlSpacePtr cspace(new oc::RealVectorControlSpace(_space, 2));
    ob::RealVectorBounds cbounds(2);
    cbounds.setLow(0, -0.1);
    cbounds.setHigh(0, 0.6);
    cbounds.setLow(1, -0.5);
    cbounds.setHigh(1, 0.5);
    cspace->as<oc::RealVectorControlSpace>()->setBounds(cbounds);

    // Create a simple setup:
    oc::SimpleSetup ss(cspace);
    ss.setStatePropagator(boost::bind(&OmplGlobalPlanner::propagate, this, _1, _2, _3,_4));
    ss.setStateValidityChecker(boost::bind(&OmplGlobalPlanner::isStateValid, this, ss.getSpaceInformation().get(), _1));

    // Define problem:
    ob::ScopedState<ob::SE2StateSpace> ompl_start(_space);
    ompl_start->setX(start.pose.position.x);
    ompl_start->setY(start.pose.position.y);
    ompl_start->setYaw(start.pose.ori);

    ob::ScopedState<ob::SE2StateSpace> ompl_goal(_space);
    ompl_goal->setX(goal.pose.position.x);
    ompl_goal->setY(goal.pose.position.y);
    ompl_goal->setYaw(0);

    ss.setStartAndGoalStates(ompl_start, ompl_goal, 0.05);

    ob::PlannerStatus solved = ss.solve(2.0);


    // Convert path into ROS messages:
    if (solved)
    {
        ROS_INFO("Ompl done!");
        oc::PathControl& result_path = ss.getSolutionPath();
        // result_path.interpolate(25);
        result_path.printAsMatrix(std::cout);

        // Create path:
        plan.push_back(start);

        // Conversion loop from states to messages:
        std::vector<ob::State*>& result_states = result_path.getStates();
        for (std::vector<ob::State*>::iterator it = result_states.begin(); it != result_states.end(); ++it)
        {
            // Get the data from the state:
            ob::SE2StateSpace::StateType *se2state = (*it)->as<ob::SE2StateSpace::StateType>();
            double* pos = se2state->as<ob::RealVectorStateSpace::StateType>(0)->values;
            double rot = se2state->as<ob::SO2StateSpace::StateType>(1)->value;

            // Place data into the pose:
            geometry_msgs::PoseStamped ps = goal;
            ps.header.stamp = ros::Time::now();
            ps.pose.position.x = pos[0];
            ps.pose.position.y = pos[1];
            plan.push_back(ps);
        }

        plan.push_back(goal);
    }
    else
    {
        ROS_ERROR("Failed to determine plan");
    }


    //publish the plan for visualization purposes
    publishPlan(plan);
    return !plan.empty();
}

void OmplGlobalPlanner::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path)
{
    if (!_initialized) {
        ROS_ERROR(
                "This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return;
    }

    //create a message for the plan 
    nav_msgs::Path gui_path;
    gui_path.poses.resize(path.size());

    if (!path.empty()) {
        gui_path.header.frame_id = path[0].header.frame_id;
        gui_path.header.stamp = path[0].header.stamp;
    }

    // Extract the plan in world co-ordinates, we assume the path is all in the same frame
    for (unsigned int i = 0; i < path.size(); i++) {
        gui_path.poses[i] = path[i];
    }

    _plan_pub.publish(gui_path);
}


} //end namespace global_planner

