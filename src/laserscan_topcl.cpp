#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <tf/message_filter.h>
#include <pcl_ros/point_cloud.h>
#include <tf/transform_listener.h>
#include <my_new_msgs/clustering.h>
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
  int size, overlap, cnt, num_scans;
  double factor;
  ros::Publisher pub;
  ros::NodeHandle node;
  std::deque<sensor_msgs::PointCloud> v;

  std::string out_topic;
  std::string input_topic;

  laser_geometry::LaserProjection projector;
  message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub;

  tf::TransformListener tfListener;
  tf::MessageFilter<sensor_msgs::LaserScan> laser_transform;

  LaserScanToPointCloud2(ros::NodeHandle n) :
    node(n),
    laser_sub(node, input_topic, 10),
    laser_transform(laser_sub, tfListener, "base_link", 10)
  {
    laser_transform.registerCallback(boost::bind(&LaserScanToPointCloud2::scanCallback, this, _1));
    laser_transform.setTolerance(ros::Duration(0.01));
    pub = node.advertise<my_new_msgs::clustering> (out_topic, 1);
    cnt = 0;
  }

    my_new_msgs::clustering bufferToAccumulator(std::deque<sensor_msgs::PointCloud> v_){
    sensor_msgs::PointCloud2 accumulator;

    for(unsigned j=0; j < v_.size(); j++){

      // boost::chrono::system_clock::time_point before = boost::chrono::system_clock::now(); 
      if (j > 0){
        for(size_t i=0; i < v_.at(j).points.size(); i++) {
          v_.at(j).points[i].z = v_.at(j-1).points[0].z + ros::Duration(v_.at(j).header.stamp - v_.at(j-1).header.stamp).toSec() * factor;
        }
      }

      // boost::chrono::system_clock::time_point now = boost::chrono::system_clock::now();

      sensor_msgs::PointCloud2 pc2;
      sensor_msgs::convertPointCloudToPointCloud2(v_[j], pc2);
      pcl::concatenatePointCloud( accumulator, pc2, accumulator);
      // boost::chrono::nanoseconds t = boost::chrono::duration_cast<boost::chrono::nanoseconds>(now-before);
      }

      my_new_msgs::clustering c;
      c.header.stamp = ros::Time::now();
      c.clusters.push_back(accumulator);
      c.factor = factor;
      c.overlap = overlap ;
      c.first_stamp = v_.at(0).header.stamp;
      c.num_scans = num_scans ;

    return c;
  }


 void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in){
    sensor_msgs::PointCloud2 cloud;
    projector.transformLaserScanToPointCloud("base_link", *scan_in, cloud, tfListener, 1000.0);

    sensor_msgs::PointCloud pc1;
    sensor_msgs::convertPointCloud2ToPointCloud (cloud, pc1);
    // TODO Investigate future problems (?)
    pc1.header.stamp = ros::Time::now();
    cnt++;
    v.push_back(pc1);
    if (v.size() > size and cnt >= overlap){
      if(cnt != v.size()){
          //v = std::vector<sensor_msgs::PointCloud>(v.begin() + overlap, v.end());
        v.erase(v.begin(), v.begin() + overlap);
      }
      pub.publish(bufferToAccumulator(v));
      num_scans = cnt ;
      cnt = 0;
    }

}

};

int main(int argc, char** argv){
  
  ros::init(argc, argv, "laserscan_topcl");
  ros::NodeHandle n;

  LaserScanToPointCloud2 lstopc(n);

  n.param("laserscan_topcl/size", lstopc.size, 40);
  n.param("laserscan_topcl/factor", lstopc.factor, 5.0);
  n.param("laserscan_topcl/overlap", lstopc.overlap, 35);
  n.param("laserscan_topcl/input_topic", lstopc.input_topic, std::string("/scan"));
  n.param("laserscan_topcl/out_topic", lstopc.out_topic, std::string("/my_pointcloud"));


  ros::spin();
  
  return 0;
}