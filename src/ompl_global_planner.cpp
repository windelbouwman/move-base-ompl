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
        _se2_space(new ob::SE2StateSpace()),
        _velocity_space(new ob::RealVectorStateSpace(1)),
        _space(_se2_space + _velocity_space),
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

void OmplGlobalPlanner::get_xy_theta_v(const ob::State *s, double& x, double& y, double& theta, double& v)
{
    const ob::CompoundStateSpace::StateType* compound_state = s->as<ob::CompoundStateSpace::StateType>();
    const ob::SE2StateSpace::StateType* se2state = compound_state->as<ob::SE2StateSpace::StateType>(0);
    const ob::RealVectorStateSpace::StateType* v_state = compound_state->as<ob::RealVectorStateSpace::StateType>(1);

    // Get the values:
    x = se2state->getX();
    y = se2state->getY();
    theta = se2state->getYaw();
    v = (*v_state)[0];
}

// Store x,y and theta into state:
void OmplGlobalPlanner::set_xy_theta_v(ob::State* rs, double x, double y, double theta, double v)
{
    ob::CompoundStateSpace::StateType* compound_state = rs->as<ob::CompoundStateSpace::StateType>();
    ob::SE2StateSpace::StateType* se2state = compound_state->as<ob::SE2StateSpace::StateType>(0);
    ob::RealVectorStateSpace::StateType* v_state = compound_state->as<ob::RealVectorStateSpace::StateType>(1);

    // Set values:
    se2state->setX(x);
    se2state->setY(y);
    se2state->setYaw(theta);
    (*v_state)[0] = v;

    // Make sure angle is (-pi,pi]:
    const ob::SO2StateSpace* SO2 = _se2_space->as<ob::SE2StateSpace>()->as<ob::SO2StateSpace>(1);
    ob::SO2StateSpace::StateType* so2 = se2state->as<ob::SO2StateSpace::StateType>(1);
    SO2->enforceBounds(so2);
}

double OmplGlobalPlanner::calc_cost(const ob::State *state)
{
    double x, y, theta, velocity;
    get_xy_theta_v(state, x, y, theta, velocity);

    // Get the cost of the footprint at the current location:
    double cost = _costmap_model->footprintCost(x, y, theta, _costmap_ros->getRobotFootprint());

    if (cost < 0)
    {
        // Unknown cell, assume zero cost here!
        cost = 0;
    }

    return cost;
}

// Calculate the cost of a motion:
double motion_cost(const ob::State* s1, const ob::State* s2)
{
    // int nd = validSegmentCount(s1, s2);
    // TODO: interpolate?
    double cst = 0;

    // cst = 

    return cst;
}

// Check the current state:
bool OmplGlobalPlanner::isStateValid(const oc::SpaceInformation *si, const ob::State *state)
{
    if (!si->satisfiesBounds(state))
    {
        return false;
    }

    // Get the cost of the footprint at the current location:
    double cost = calc_cost(state);

    // std::cout << cost << std::endl;
    // Too high cost:
    if (cost > 90)
    {
        return false;
    }

    // Error? Unknown space?
    if (cost < 0)
    {
    }

    return true;
}

// Calculate vehicle dynamics, assume a velocity state, but steering to be instantly possibe.
void OmplGlobalPlanner::propagate(const ob::State *start, const oc::Control *control, const double duration, ob::State* result)
{
    // Implement vehicle dynamics:
    double x, y, theta, velocity;
    get_xy_theta_v(start, x, y, theta, velocity);

    const double* ctrl = control->as<oc::RealVectorControlSpace::ControlType>()->values;
    double acc = ctrl[0];
    double steer_angle = ctrl[1];

    // Calculate vehicle dynamics:
    double x_n, y_n, theta_n, velocity_n;
    // TODO: elaborate further..
    x_n = x + velocity * duration * cos(theta);
    y_n = y + velocity * duration * sin(theta);
    velocity_n = velocity + acc * duration;

    double vehicle_length = 0.4;
    double lengthInv = 1 / vehicle_length;
    double omega = velocity * lengthInv * tan(steer_angle);
    theta_n = theta + omega * duration;

    // Store new state in result:
    set_xy_theta_v(result, x_n, y_n, theta_n, velocity_n);
}

double calcYaw(const geometry_msgs::Pose pose)
{
    double yaw, pitch, roll;
    tf::Pose pose_tf;
    tf::poseMsgToTF(pose, pose_tf);
    pose_tf.getBasis().getEulerYPR(yaw, pitch, roll);
    return yaw;
}

void pose2state(const geometry_msgs::Pose& pose, ob::State* state)
{
    //ompl_start[0] = start.pose.position.x;
    //ompl_start[1] = start.pose.position.y;
    //ompl_start[2] = calcYaw(start.pose);
}

bool OmplGlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                           std::vector<geometry_msgs::PoseStamped>& plan)
{
    boost::mutex::scoped_lock lock(_mutex);
    if (!_initialized)
    {
        ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
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
    _se2_space->as<ob::SE2StateSpace>()->setBounds(bounds);

    ob::RealVectorBounds velocity_bounds(1);
    velocity_bounds.setHigh(0.6);
    velocity_bounds.setLow(-0.1);
    _velocity_space->as<ob::RealVectorStateSpace>()->setBounds(velocity_bounds);

    oc::ControlSpacePtr cspace(new oc::RealVectorControlSpace(_space, 2));
    ob::RealVectorBounds cbounds(2);
    cbounds.setLow(0, -0.04);
    cbounds.setHigh(0, 0.3);
    cbounds.setLow(1, -0.2);
    cbounds.setHigh(1, 0.2);
    cspace->as<oc::RealVectorControlSpace>()->setBounds(cbounds);

    // Create space information:
    oc::SpaceInformationPtr si(new oc::SpaceInformation(_space, cspace));
    si->setStatePropagator(boost::bind(&OmplGlobalPlanner::propagate, this, _1, _2, _3,_4));
    si->setStateValidityChecker(boost::bind(&OmplGlobalPlanner::isStateValid, this, si.get(), _1));

    // Define problem:
    ob::ScopedState<> ompl_start(_space);
    ompl_start[0] = start.pose.position.x;
    ompl_start[1] = start.pose.position.y;
    ompl_start[2] = calcYaw(start.pose);
    ompl_start[3] = 0; // Speed

    ob::ScopedState<> ompl_goal(_space);
    ompl_goal[0] = goal.pose.position.x;
    ompl_goal[1] = goal.pose.position.y;
    ompl_goal[2] = calcYaw(start.pose);
    ompl_goal[3] = 0; // Speed

    // Optimize criteria:
    ob::OptimizationObjectivePtr cost_objective(new CostMapObjective(*this, si));
    ob::OptimizationObjectivePtr length_objective(new ob::PathLengthOptimizationObjective(si));
    //ob::OptimizationObjectivePtr objective(new CostMapWorkObjective(*this, si));

    ob::ProblemDefinitionPtr pdef(new ob::ProblemDefinition(si));
    pdef->setStartAndGoalStates(ompl_start, ompl_goal, 0.1);
    pdef->setOptimizationObjective(cost_objective + length_objective);

    ROS_INFO("Problem defined, running planner");
    // oc::DecompositionPtr decomp(new My
    // ob::PlannerPtr planner(new oc::LBTRRT(si));
    // ob::PlannerPtr planner(new og::RRTConnect(si));
    ob::PlannerPtr planner(new og::RRTstar(si));
    // ob::PlannerPtr planner(new og::PRMstar(si)); // works
    // ob::PlannerPtr planner(new og::PRM(si)); // segfault
    // ob::PlannerPtr planner(new og::TRRT(si));
    planner->setProblemDefinition(pdef);
    planner->setup();
    ob::PlannerStatus solved = planner->solve(3.0);


    // Convert path into ROS messages:
    if (solved)
    {
        ROS_INFO("Ompl done!");
        ob::PathPtr result_path1 = pdef->getSolutionPath();

        // Cast path into geometric path:
        og::PathGeometric& result_path = static_cast<og::PathGeometric&>(*result_path1);

        // result_path.interpolate(100);
        // result_path->print(std::cout);

        // Create path:
        plan.push_back(start);

        // Conversion loop from states to messages:
        std::vector<ob::State*>& result_states = result_path.getStates();
        for (std::vector<ob::State*>::iterator it = result_states.begin(); it != result_states.end(); ++it)
        {
            // Get the data from the state:
            double x, y, theta, velocity;
            get_xy_theta_v(*it, x, y, theta, velocity);

            // Place data into the pose:
            geometry_msgs::PoseStamped ps = goal;
            ps.header.stamp = ros::Time::now();
            ps.pose.position.x = x;
            ps.pose.position.y = y;
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

