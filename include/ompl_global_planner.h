#ifndef _OMPL_GLOBAL_PLANNER_H
#define _OMPL_GLOBAL_PLANNER_H
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
 *********************************************************************/

/*
    Modified by: Windel Bouwman
*/

#include <ros/ros.h>
#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>
#include <tf/transform_datatypes.h>
#include <nav_core/base_global_planner.h>
#include <nav_msgs/GetPlan.h>
#include <base_local_planner/costmap_model.h>

#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/TRRT.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/base/objectives/MechanicalWorkOptimizationObjective.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>

namespace ob = ompl::base;
namespace oc = ompl::control;
namespace og = ompl::geometric;

namespace ompl_global_planner {

class OmplGlobalPlanner : public nav_core::BaseGlobalPlanner
{
    public:
        OmplGlobalPlanner();

        // Implemented functions for plugin:
        virtual void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
        virtual bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                      std::vector<geometry_msgs::PoseStamped>& plan);

        void publishPlan(const std::vector<geometry_msgs::PoseStamped>& path);

        ~OmplGlobalPlanner() {
        }

        // Ompl related functions:
        void propagate(const ob::State *start, const oc::Control *control, const double duration, ob::State *result);
        bool isStateValid(const oc::SpaceInformation *si, const ob::State *state);

        double calc_cost(const ob::State*);
        double motion_cost(const ob::State* s1, const ob::State* s2);
        void get_xy_theta_v(const ob::State* s, double& x, double& y, double& theta, double& velocity);
        void set_xy_theta_v(ob::State*, double x, double y, double theta, double velocity);

    private:
        costmap_2d::Costmap2DROS* _costmap_ros;
        std::string _frame_id;
        ros::Publisher _plan_pub;
        bool _initialized;
        bool _allow_unknown;

        std::string tf_prefix_;
        boost::mutex _mutex;
        base_local_planner::CostmapModel* _costmap_model;

        // State spaces:
        ob::StateSpacePtr _se2_space;
        ob::StateSpacePtr _velocity_space;
        ob::StateSpacePtr _space;
};


class CostMapObjective : public ob::StateCostIntegralObjective
{
    public:
    CostMapObjective(OmplGlobalPlanner& op, const ob::SpaceInformationPtr& si)
        : ob::StateCostIntegralObjective(si, true),
        _ompl_planner(op)
    {
    }

    virtual ob::Cost stateCost(const ob::State* s) const
    {
        return ob::Cost(_ompl_planner.calc_cost(s));
    }


    private:
        OmplGlobalPlanner& _ompl_planner;
};


class CostMapWorkObjective : public ob::MechanicalWorkOptimizationObjective
{
    public:
    CostMapWorkObjective(OmplGlobalPlanner& op, const ob::SpaceInformationPtr& si)
        : ob::MechanicalWorkOptimizationObjective(si),
        _ompl_planner(op)
    {
    }

    ob::Cost stateCost(const ob::State* s) const
    {
        return ob::Cost(_ompl_planner.calc_cost(s));
    }

    /*
    ob::Cost motionCost(const ob::State* s1, const ob::State* s2) const
    {
        return ob::Cost(_ompl_planner.motion_cost(s1, s2));
    }
    */

    private:
        OmplGlobalPlanner& _ompl_planner;
};

}

#endif
