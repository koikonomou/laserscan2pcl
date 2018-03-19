#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/ChannelFloat32.h>
#include <pcl/common/projection_matrix.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/impl/point_types.hpp>

ros::Publisher point_pub;


void pointcloud_cb (const sensor_msgs::PointCloudConstPtr &msg ){

sensor_msgs::PointCloud pointmsg;
//pcl::PointCloud<pcl::PointXYZ>::iterator it;
//pcl::PointXYZRGBA point_n;
//pcl::PointCloud<pcl::PointXYZ> cloud;

pointmsg.header.seq = msg->header.seq;
pointmsg.header.stamp = ros::Time::now();
pointmsg.header.frame_id = msg->header.frame_id;
pointmsg.points = msg->points;
pointmsg.channels = msg->channels;
pointmsg.points[0].x = msg->points[0].x;
pointmsg.points[1].y = msg->points[1].y;
pointmsg.points[2].z = msg->points[2].z;


/*
if (msg->points[2].z == 0){
		pointmsg.points[2].z = 4.0;
	}
	//pointmsg.point[2].z = points[1].z;


/*

float it;
for (it = cloud.points.begin(); it < cloud.points.end(); it++){
	point_n.x = it->x;
	point_n.y = it->y;
	point_n.z = it->z;

 }*/


point_pub.publish (msg);
}

int main (int argc, char** argv){
	ros::init (argc, argv, "new_point_pointmsg");
	ros::NodeHandle n_;

	ros::Subscriber sub = n_.subscribe ("/pointcloud", 1, pointcloud_cb);

	point_pub = n_.advertise<sensor_msgs::PointCloud> ("/new_point_cloud", 1);

	ros::spin ();
}