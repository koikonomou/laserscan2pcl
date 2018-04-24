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


int size;
ros::Publisher pub;
ros::Time prev_time, time_now;
std::vector<sensor_msgs::PointCloud> vec_;

void callback(const my_new_msgs::clustering& msg){

    sensor_msgs::PointCloud cloud;
    my_new_msgs::clustering c_;

    if (msg.clusters.size() > 0){
        sensor_msgs::convertPointCloud2ToPointCloud( msg.clusters[0] , cloud );
        vec_.push_back(cloud);
    }

    sensor_msgs::PointCloud oldFirst;

    if (vec_.size() > 1){
        oldFirst = sensor_msgs::PointCloud(vec_[0]);
    }
    if (vec_.size() > size){
        vec_.erase(vec_.begin());
    }


    for (unsigned j=0; j < vec_.size(); j++){
        float tmp = vec_[j].points[0].z;

        if (j > 0){
            vec_[j].points[0].z = vec_[j-1].points[0].z + ros::Duration( vec_[j].header.stamp - vec_[j-1].header.stamp).toSec() * msg.factor;
            for (size_t k=1; k < vec_[j].points.size(); k++) {
                vec_[j].points[k].z += vec_[j].points[0].z - tmp;

            }
        }
        else if (oldFirst.points.size() > 0){
            for (size_t k=0; k < vec_[j].points.size(); k++) {
                vec_[j].points[k].z -= oldFirst.points[0].z + tmp;
            }
        }
        sensor_msgs::PointCloud2 pc2;
        sensor_msgs::convertPointCloudToPointCloud2( vec_[j] , pc2 );
        c_.clusters.push_back(pc2);
    }
    pub.publish(c_);

}



int main (int argc, char** argv){
    ros::init (argc, argv, "pointcloud_layers");
    ros::NodeHandle n_;
    n_.param("pointcloud_layers/size", size , 4);

    ros::Subscriber sub = n_.subscribe("my_pointcloud", 1 , callback);
    pub = n_.advertise<my_new_msgs::clustering>("pointcloudlayers", 1);
    ros::spin ();
}