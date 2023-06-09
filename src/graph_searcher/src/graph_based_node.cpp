/*
Copyright (C) 2023 Ziping Yu (krasjet.ziping@gmail.com)
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.
THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO
EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
OF SUCH DAMAGE.
*/
#include "occ_grid/occ_map.h"
#include "Astar_searcher.h"
#include "JPS_searcher.h"

#include "ros/ros.h"
#include "visulization/visulization.hpp"
#include "geometry_msgs/PoseStamped.h"

class GraphBasedPathFinder
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber wp_sub_, glb_sub_;
    ros::Timer execution_timer_;

    env::OccMap::Ptr env_ptr_;
    std::shared_ptr<visualization::Visualization> vis_ptr_;

    Eigen::Vector3d start_, goal_;

    bool run_a_star_, run_jps_;

    AstarPathFinder::Ptr a_star_;
    JPSPathFinder::Ptr jps_;

public:
    GraphBasedPathFinder(const ros::NodeHandle &nh) : nh_(nh)
    {
        env_ptr_ = std::make_shared<env::OccMap>();
        env_ptr_->init(nh_);

        vis_ptr_ = std::make_shared<visualization::Visualization>(nh_);
        vis_ptr_->registe<visualization_msgs::Marker>("start");
        vis_ptr_->registe<visualization_msgs::Marker>("goal");

        wp_sub_ = nh_.subscribe("/goal", 1, &GraphBasedPathFinder::rcvWaypointsCallback, this);
        // glb_sub_ = nh_.subscribe("/occ_map/glb_map", 1, &GraphBasedPathFinder::rcvPointCloudCallBack, this);
        execution_timer_ = nh_.createTimer(ros::Duration(1), &TesterPathFinder::executionCallback, this);

        // Path Finding Algorithms
        a_star_ = std::make_shared<AstarPathFinder>(nh_);
        a_star_->initOccMap(env_ptr_, Eigen::Vector3i(100, 100, 100));

        start_.setZero();

        nh_.param("run_a_star", run_a_star_, true);
        nh_.param("run_jps", run_jps_, true);
    }

    ~GraphBasedPathFinder(){}

    void rcvWaypointsCallback(const geometry_msgs::PoseStamped::ConstPtr &wp)
    {
        goal_[0] = wp->pose.position.x;
        goal_[1] = wp->pose.position.y;
        goal_[2] = wp->pose.position.z;
        ROS_INFO_STREAM("\n-----------------------------\ngoal rcved at " << goal_.transpose());
        vis_ptr_->visualize_a_ball(start_, 0.3, "start", visualization::Color::pink);
        vis_ptr_->visualize_a_ball(goal_, 0.3, "goal", visualization::Color::steelblue);

        if (run_a_star_)
        {

        }

        if (run_jps_)
        {

        }

        start_ = goal_;
    }

    void executionCallback(const ros::TimeEvent &event)
    {
        if (!env_ptr_->mapValid())
        {
            ROS_WARN("no map rcved yet.");
        }
        else
        {
            execution_timer_.stop();
        }
    }

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "graph_based_node");
    ros::NodeHandle nh("~");

    
}