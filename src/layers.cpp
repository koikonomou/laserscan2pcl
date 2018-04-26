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

    for (unsigned i=0; vec_.size(); i++){
        offset = ros::Duration( vec_[i].header.stamp - vec_[i-1].header.stamp).toSec() * msg.factor;

        for (unsigned j=0; vec_[i].clusters.size(); j++){
            sensor_msgs::PointCloud cloud;
            sensor_msgs::convertPointCloud2ToPointCloud( vec_[i].clusters[j] , cloud);

            // v.push_back(cloud);

            // sensor_msgs::PointCloud oldFirst;
            for (unsigned k=0; k < cloud.points.size(); k++){
                cloud.points[k].z = cloud.points[k-1].z + offset;
            }
            sensor_msgs::PointCloud2 pc2;
            sensor_msgs::convertPointCloudToPointCloud2( cloud , pc2 );
            c_.clusters.push_back(pc2);
        }
       pub.publish(c_);
    }
}

            // oldFirst = sensor_msgs::PointCloud(v[0]);
            // if (v.size() > 1){
            // }

            // for (unsigned j = 0; j < v.size() ; j++){
            //     float tmp = v[j].points[0].z ;

            //     if (j>0) {
            //         v[j].points[0].z = v[j-1].points[0].z + offset ;
            //         for (size_t k=1; k < v[j].points.size(); k++) {
            //                 v[j].points[k].z += v[j].points[0].z - tmp;
            //         }
            //     }
            //     else if (oldFirst.points.size() > 0){
            //         for (size_t k=0; k < v[j].points.size(); k++) {
            //             v[j].points[k].z -= oldFirst.points[0].z + tmp;
            //         }
            //     }
        //     }



int main (int argc, char** argv){
    ros::init (argc, argv, "pointcloud_layers");
    ros::NodeHandle n_;
    n_.param("pointcloud_layers/size", size , 4);

    ros::Subscriber sub = n_.subscribe("/new_pcl", 1 , callback);
    pub = n_.advertise<my_new_msgs::clustering>("pointcloudlayers", 1);
    ros::spin ();
}