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
float offset;
ros::Publisher pub;

// std::vector<sensor_msgs::PointCloud> v;
std::vector<my_new_msgs::clustering> vec_;

void callback(const my_new_msgs::clustering& msg){

    sensor_msgs::PointCloud cloud;
    my_new_msgs::clustering c_;
    vec_.push_back(msg);

    if (vec_.size() > size){
        vec_.erase(vec_.begin());

    }

    for (unsigned i=0; i < vec_.size(); i++){
 
        if ( i > 0 ){

            offset = ros::Duration( vec_[i].first_stamp - vec_[0].first_stamp).toSec() * msg.factor;
            
        }
        else{
            offset = 0 ;
        }


        for (unsigned j=0; j < vec_[i].clusters.size(); j++){
            sensor_msgs::PointCloud cloud;
            sensor_msgs::convertPointCloud2ToPointCloud( vec_[i].clusters[j] , cloud);
 

            // v.push_back(cloud);

            // sensor_msgs::PointCloud oldFirst;
            for (unsigned k=0; k < cloud.points.size(); k++){
                cloud.points[k].z = cloud.points[k].z + offset;
            }

            sensor_msgs::PointCloud2 pc2;
            sensor_msgs::convertPointCloudToPointCloud2( cloud , pc2 );

            c_.clusters.push_back(pc2);

        }

    }
   pub.publish(c_);
}



int main (int argc, char** argv){
    ros::init (argc, argv, "pointcloud_layers");
    ros::NodeHandle n_;
    n_.param("pointcloud_layers/size", size , 4);

    ros::Subscriber sub = n_.subscribe("/new_pcl", 1 , callback);
    pub = n_.advertise<my_new_msgs::clustering>("pointcloudlayers", 1);
    ros::spin ();
}