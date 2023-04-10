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
#include "kino_planner/kino_a_star.h"

#include "ros/ros.h"
#include "visualization/visualization.hpp"
#include "geometry_msgs/PoseStamped.h"

class KinoBasedPathFinder
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber wp_sub_, glb_sub_;
    ros::Timer execution_timer_;

    env::OccMap::Ptr env_ptr_;
    std::shared_ptr<visualization::Visualization> vis_ptr_;

    Eigen::Vector3d start_, goal_;
    Eigen::Vector3d start_v_, start_a_, goal_v_;

    bool run_kino_a_star_;

    KinodynamicAstar::Ptr kino_a_star_;

public:
    KinoBasedPathFinder(const ros::NodeHandle &nh) : nh_(nh)
    {
        env_ptr_ = std::make_shared<env::OccMap>();
        env_ptr_->init(nh_);

        vis_ptr_ = std::make_shared<visualization::Visualization>(nh_);
        vis_ptr_->registe<visualization_msgs::Marker>("start");
        vis_ptr_->registe<visualization_msgs::Marker>("goal");

        wp_sub_ = nh_.subscribe("/goal", 1, &KinoBasedPathFinder::rcvWaypointsCallback, this);
        // glb_sub_ = nh_.subscribe("/occ_map/glb_map", 1, &KinoBasedPathFinder::rcvPointCloudCallBack, this);
        execution_timer_ = nh_.createTimer(ros::Duration(1), &KinoBasedPathFinder::executionCallback, this);
        
        nh_.param("search/run_kino_a_star", run_kino_a_star_, false);

        // Path Finding Algorithms
        if (run_kino_a_star_) {
            kino_a_star_.reset(new KinodynamicAstar);
            kino_a_star_->initParam(nh_);
            kino_a_star_->initOccMap(env_ptr_, Eigen::Vector3i(100, 100, 100));
            vis_ptr_->registe<nav_msgs::Path>("kino_a_star_final_path");
            vis_ptr_->registe<sensor_msgs::PointCloud2>("kino_a_star_final_wpts");
        }

        start_.setZero();
        start_v_.setZero();
        start_a_.setZero();
        start_(2) = 1.0;   // Prevent the starting point from getting stuck in an obstacle
    }

    ~KinoBasedPathFinder(){}

    void rcvWaypointsCallback(const geometry_msgs::PoseStamped::ConstPtr &wp)
    {
        goal_[0] = wp->pose.position.x;
        goal_[1] = wp->pose.position.y;
        // goal_[2] = wp->pose.position.z;
        goal_[2] = 2.0;
        goal_v_.setZero();
        ROS_INFO_STREAM("\n-----------------------------\ngoal rcved at " << goal_.transpose());
        vis_ptr_->visualize_a_ball(start_, 0.3, "start", visualization::Color::pink);
        vis_ptr_->visualize_a_ball(goal_, 0.3, "goal", visualization::Color::steelblue);

        if (run_kino_a_star_) {
            ROS_INFO_STREAM("\n-----------------------------\n[RUNNING] Run Kinodynamic AStar !!! ");
            int kino_a_star_res = kino_a_star_->KinoAstarSearch(start_, start_v_, start_a_, goal_, goal_v_,
                                                                false, env_ptr_->getResolution(),
                                                                true, 0.0);
            if (kino_a_star_res == KINO_RET::NO_PATH) {
                ROS_INFO("Acceleated Start!!!");
                kino_a_star_res = kino_a_star_->KinoAstarSearch(start_, start_v_, start_a_, goal_, goal_v_,
                                                                    false, env_ptr_->getResolution(),
                                                                    true, 0.0);
            }
            if (kino_a_star_res == KINO_RET::REACH_END || kino_a_star_res == KINO_RET::REACH_HORIZON
                || kino_a_star_res == KINO_RET::NEAR_END) {
                ROS_INFO_STREAM("\n-----------------------------\n[FINISH] Finish AStar !!! ");
                std::vector<Eigen::Vector3d> final_path = kino_a_star_->getPath();
                vis_ptr_->visualize_path(final_path, "kino_a_star_final_path");
                vis_ptr_->visualize_pointcloud(final_path, "kino_a_star_final_wpts");
            }
            else if (kino_a_star_res == KINO_RET::INIT_ERR /*&& i + 1 < segment_ids.size()*/) { // connect the next segment
                // segment_ids[i].second = segment_ids[i + 1].second;
                // segment_ids.erase(segment_ids.begin() + i + 1);
                // --i;
                ROS_WARN("Init Failed, Please check your start or goal point ! ");
            }
            else {
                ROS_ERROR("kino A-star error, force return!");
                // return CHK_RET::ERR;
                return;
            }
        }

        start_ = goal_;
    }

    void executionCallback(const ros::TimerEvent &event)
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

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "kino_based_node");
    ros::NodeHandle nh("~");

    KinoBasedPathFinder kbpf(nh);

    ros::AsyncSpinner spinner(0);
    spinner.start();
    ros::waitForShutdown();
    return 0;
}