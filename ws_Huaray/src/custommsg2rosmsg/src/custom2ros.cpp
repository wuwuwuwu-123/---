#include "ros/ros.h"
#include "livox_ros_driver2/CustomMsg.h"
#include "sensor_msgs/PointCloud2.h"
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>

// 全局发布者，用于发布转换后的点云
ros::Publisher pub;

// 回调函数：将 CustomMsg 转换为 sensor_msgs::PointCloud2
void cloudCallback(const livox_ros_driver2::CustomMsg::ConstPtr& msg)
{
    pcl::PointCloud<pcl::PointXYZI> cloud;

    for (size_t i = 0; i < msg->points.size(); ++i)
    {
        pcl::PointXYZI pt;
        pt.x = msg->points[i].x;
        pt.y = msg->points[i].y;
        pt.z = msg->points[i].z;
        pt.intensity = msg->points[i].reflectivity;
        cloud.points.push_back(pt);
    }
    cloud.width = cloud.points.size();
    cloud.height = 1;
    cloud.is_dense = false;

    // 将 pcl 点云转换为 ROS 的 sensor_msgs::PointCloud2 消息
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(cloud, output);

    output.header.stamp = msg->header.stamp;
    output.header.frame_id = msg->header.frame_id;

    pub.publish(output);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "custom_to_pointcloud");
    ros::NodeHandle nh;

    pub = nh.advertise<sensor_msgs::PointCloud2>("/custom2ros/rviz_pointcloud", 1);

    ros::Subscriber sub = nh.subscribe("/livox/lidar", 1, cloudCallback);

    ros::spin();
    return 0;
}
