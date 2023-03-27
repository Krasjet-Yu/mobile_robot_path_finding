#include <iostream>
#include <fstream>
#include <math.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
using namespace std;

// ref: https://blog.csdn.net/weixin_46098577/article/details/112280924
// command: rosrun pcl_ros pointcloud_to_pcd /input:=/{name_topic}

// ros related
ros::Subscriber _map_sub;
// useful global variables
bool _has_map   = false;
std::string file_path = "test.pcd";

void rcvPointCloudCallBack(const sensor_msgs::PointCloud2 & pointcloud_map)
{   
    if(_has_map ) return;
	pcl::PointCloud<pcl::PointXYZ> cloud_in;

    pcl::fromROSMsg(pointcloud_map, cloud_in);

    _has_map = true;
	//------------------------------- 将点云投影至XOY平面 ----------------------------
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>); //存放投影后的点云

	//定义cloud_out的大小等参数，防止下标越界
	cloud_out->width = cloud_in.width;
	cloud_out->height = 1;
	cloud_out->is_dense = true;
	cloud_out->points.resize(cloud_out->width*cloud_out->height);

	for (size_t i = 0; i < cloud_in.points.size(); i++) {
		cloud_out->points[i].x = cloud_in.points[i].x;
		cloud_out->points[i].y = cloud_in.points[i].y;
		cloud_out->points[i].z = 0;
	}
    //-------------------------------- 将点云保存到硬盘 -----------------------------
	pcl::PCDWriter writer;
    ///保存为Binary格式，不可用记事本打开，但更快速。
	if (!cloud_out->empty())
	{
		writer.write("test.pcd", *cloud_out, true);	//true，保存为Binary格式
		cout << "->(test)保存点数为：" << cloud_out->points.size() << endl;
	}
	else
	{
		PCL_ERROR("\a->保存点云为空！\n");
		system("pause");
	}
	//==============================================================================
}

void readPCDFile() {
	//-------------------------------- 从硬盘中读取点云 -----------------------------
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>); //存放读取点云的对象
	pcl::PCDReader reader;	//定义点云读取对象
	if (reader.read("test.pcd", *cloud_in) < 0)
	{
		PCL_ERROR("\a->点云文件不存在！\n");
		system("pause");
		return;
	}
	cout << "->加载了 " << cloud_in->points.size() << " 个数据点" << endl;
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "pcd_save_node");
    ros::NodeHandle nh("~");

    _map_sub  = nh.subscribe( "map",       1, rcvPointCloudCallBack );
    
	ros::Rate rate(100);
    bool status = ros::ok();
    while(status) 
    {
        ros::spinOnce();      
        status = ros::ok();
        rate.sleep();
    }

	return 0;
}