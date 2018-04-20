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
ros::Time prev_time, my_time;
std::vector<sensor_msgs::PointCloud> vec_;

my_new_msgs::clustering pointcloudlayers(std::vector<sensor_msgs::PointCloud> v){

    my_new_msgs::clustering msg;

    for(unsigned j=0; j < v.size(); j++){

        my_time = msg.header.stamp;


        if (j > 0){
        for(size_t i=0; i < v[j].points.size(); i++) {

            v[j].points[i].z = v[j-1].points[i-1].z + ros::Duration( my_time- prev_time).toSec() * msg.factor + 5 ;
            // ROS_WARN("%lu", my_time);
            // ROS_INFO("%lu", prev_time);
            }
        }
    prev_time = my_time;
    sensor_msgs::PointCloud2 pc2;
    sensor_msgs::convertPointCloudToPointCloud2(v[j],pc2);
    msg.clusters.push_back(pc2);
    }

    return msg;
}


void callback(const my_new_msgs::clustering& msg){

    sensor_msgs::PointCloud cloud;
    for (size_t i=0; i< msg.clusters.size(); i++){

        sensor_msgs::convertPointCloud2ToPointCloud(msg.clusters[i],cloud);

        }
        vec_.push_back(cloud);
        if (vec_.size() > size){
            vec_.erase(vec_.begin());
            pub.publish(pointcloudlayers(vec_));
        //     if(counter == 0){
        //     vec_.push_back(cloud);
        // }
        // else{
        //     if(counter == layers-1){
        //         counter = -1;
        //         vec_.popback();
        //         pub.publish(pointcloudlayers(vec_));

        //     }
        // }
    }

}



int main ( int argc, char** argv){
    ros::init (argc, argv, "pointcloud_buffer");
    ros::NodeHandle n_;
    n_.param("pointcloud_buffer/size", size , 4);

    ros::Subscriber sub = n_.subscribe("my_pointcloud",1 , callback);
    pub = n_.advertise<my_new_msgs::clustering>("pointcloudlayers", 1);
    ros::spin ();


}