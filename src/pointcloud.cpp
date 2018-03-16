#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/PointCloud2.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "laser_geometry/laser_geometry.h"

class LaserScanToPointCloud2{

public:

  ros::NodeHandle node;
  laser_geometry::LaserProjection projector;
  laser_geometry::channel_option::ChannelOption Distance ;
  tf::TransformListener tfListener;
  message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub;
  tf::MessageFilter<sensor_msgs::LaserScan> laser_transform;
  ros::Publisher point_cloud_publisher;



  LaserScanToPointCloud2(ros::NodeHandle n) : 
    node(n),
    laser_sub(node, "/scan", 10),
    laser_transform(laser_sub, tfListener, "base_link", 10)
  {
    laser_transform.registerCallback(boost::bind(&LaserScanToPointCloud2::scanCallback, this, _1));
    laser_transform.setTolerance(ros::Duration(0.01));
    point_cloud_publisher = node.advertise<sensor_msgs::PointCloud2>("/pointcloud",1);

  }
  void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
  {
    sensor_msgs::PointCloud2 cloud;
    projector.transformLaserScanToPointCloud("base_link", *scan_in, cloud, tfListener, Distance );
    int Distance=5;
    projector.projectLaser(*scan_in, cloud);
    point_cloud_publisher.publish(cloud);

  }


};

int main(int argc, char** argv)
{
  
  ros::init(argc, argv, "my_scan_to_cloud");
  ros::NodeHandle n;
  LaserScanToPointCloud2 lstopc(n);
  
  ros::spin();
  
  return 0;
}