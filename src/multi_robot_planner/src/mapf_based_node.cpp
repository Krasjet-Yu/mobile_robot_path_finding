/*
Copyright (C) 2023 Krasjet Yu (krasjet.ziping@gmail.com)
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
#include "visualization/visualization.hpp"

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <geometry_msgs/PoseStamped.h>

class MultiRobotPathFinder
{
  private:
    ros::NodeHandle nh_;
    ros::Subscriber wp_sub;
    ros::Timer execution_timer_;
    env::OccMap::Ptr env_ptr_;
    std::shared_ptr<visualization::Visualization> vis_ptr_;

    Eigen::Vector3d start_, goal_;

    bool run_single_agent_a_star, run_hca_star, run_m_star;
    bool run_id, run_cbs_, run_ecbs;
  public:
    MultiRobotPathFinder(const ros::NodeHandle &nh) : nh_(nh) {
      env_ptr_ = std::make_shared<env::OccMap>();
      env_ptr_ -> init(nh_);
    }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mapf_based_node");
  ros::NodeHandle nh("~");

  MultiRobotPathFinder mapf(nh);

  ros::AsyncSpinner spinner(0);
  spinner.start();
  ros::waitForShutdown();
  return 0;
}