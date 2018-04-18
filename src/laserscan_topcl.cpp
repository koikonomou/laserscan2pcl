#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <tf/message_filter.h>
#include <pcl_ros/point_cloud.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud.h>
#include <pcl/impl/point_types.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <sensor_msgs/ChannelFloat32.h>
#include <pcl/common/projection_matrix.h>
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/point_cloud_conversion.h>


class LaserScanToPointCloud2{
public:

  ros::Publisher pub;
  ros::NodeHandle node;
  ros::Time prev_time, my_time ;

  sensor_msgs::PointCloud2 accumulator;
  std::vector<sensor_msgs::PointCloud> v;
  laser_geometry::LaserProjection projector;
  message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub;

  tf::TransformListener tfListener;
  tf::MessageFilter<sensor_msgs::LaserScan> laser_transform;
  
  int size = 40;
  double factor = 1.0;




  LaserScanToPointCloud2(ros::NodeHandle n) : 
    node(n),
    laser_sub(node, "/scan", 10),
    laser_transform(laser_sub, tfListener, "base_link", 10)
  {
    laser_transform.registerCallback(boost::bind(&LaserScanToPointCloud2::scanCallback, this, _1));
    laser_transform.setTolerance(ros::Duration(0.01));
    pub = node.advertise<sensor_msgs::PointCloud2> ("/my_pointcloud", 1);

  }

  sensor_msgs::PointCloud2 bufferToAccumulator(const std::vector<sensor_msgs::PointCloud> buffer, double factor){
    sensor_msgs::PointCloud2 accumulator;

    sensor_msgs::PointCloud pcl_;
    for(unsigned j=0; j < buffer.size(); j++){

      sensor_msgs::PointCloud pc1 = sensor_msgs::PointCloud(buffer[j]);
      my_time = pc1.header.stamp;
      if (prev_time == my_time){
        prev_time = my_time;
      }
      boost::chrono::system_clock::time_point before = boost::chrono::system_clock::now(); 

      if (j > 0){
        for(size_t i=0; i < pc1.points.size(); i++) {
          pc1.points[i].z = pcl_.points[0].z + (ros::Duration(my_time - prev_time)).toSec() * factor;
        }
      }
      pcl_ = sensor_msgs::PointCloud(pc1);
      // boost::chrono::system_clock::time_point now = boost::chrono::system_clock::now();

      sensor_msgs::PointCloud2 pc2;
      sensor_msgs::convertPointCloudToPointCloud2(pc1, pc2);
      pcl::concatenatePointCloud( accumulator, pc2, accumulator);
      // boost::chrono::nanoseconds t = boost::chrono::duration_cast<boost::chrono::nanoseconds>(now-before);
      prev_time = my_time;
      }

    return accumulator;
  }


 void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
  {
    sensor_msgs::PointCloud2 cloud;
    projector.transformLaserScanToPointCloud("base_link", *scan_in, cloud, tfListener);

    sensor_msgs::PointCloud pc1;
    sensor_msgs::convertPointCloud2ToPointCloud (cloud, pc1);


    v.push_back(pc1);
    if (v.size() > size){
        v.erase(v.begin());
        pub.publish(bufferToAccumulator(v, factor));
    }

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