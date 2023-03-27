#include<ros/ros.h>
#include<pcl/point_cloud.h>
#include<pcl_conversions/pcl_conversions.h>
#include<sensor_msgs/PointCloud2.h>
#include<pcl/io/pcd_io.h>//which contains the required definitions to load and store point clouds to PCD and other file formats.

std::string pcd_file = "/home/krasjet/Documents/ROS/planning_catkin/src/grid_path_searcher/0.pcd";

int main (int argc, char **argv)
{
  ros::init (argc, argv, "pcd_read_node");
  ros::NodeHandle nh("~");
  ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2> ("pcl_output", 1);
  pcl::PointCloud<pcl::PointXYZ> cloud;
  sensor_msgs::PointCloud2 output;
  pcl::io::loadPCDFile (pcd_file, cloud); //修改自己pcd文件所在路径
  //Convert the cloud to ROS message
  pcl::toROSMsg(cloud, output);
  // output.header.frame_id = "odom";//this has been done in order to be able to visualize our PointCloud2 message on the RViz visualizer    
  //！！！这一步需要注意，是后面rviz的 fixed_frame
  ros::Rate loop_rate(1);
  while (ros::ok())
  {
    pcl_pub.publish(output);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
