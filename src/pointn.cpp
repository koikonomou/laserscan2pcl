#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud.h>
#include <pcl/impl/point_types.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/ChannelFloat32.h>
#include <pcl/common/projection_matrix.h>
#include <sensor_msgs/point_cloud_conversion.h>

int counter = 0;
ros::Publisher point_pub;
int no_laserscans = 0;
sensor_msgs::PointCloud2 accumulator;

void cloud_callback (sensor_msgs::PointCloud2Ptr msg){

	sensor_msgs::PointCloud pc1;

	sensor_msgs::convertPointCloud2ToPointCloud (*msg, pc1);

	for(size_t i=0; i<pc1.points.size(); i++) {
		pc1.points[i].z = ((float)counter)/10.f;
	}

	sensor_msgs::convertPointCloudToPointCloud2(pc1, *msg); 

	if( counter == 0 ) {
		accumulator = sensor_msgs::PointCloud2(*msg);
	}
	else {
		pcl::concatenatePointCloud( accumulator, *msg, accumulator );

		if( counter == no_laserscans-1 ) {
			counter = -1;
			point_pub.publish( accumulator );
		}

	}
	counter++;
}

int main (int argc, char** argv){
	ros::init (argc, argv, "new_point_pointmsg");
	ros::NodeHandle n_;
	no_laserscans = 0;
	std::string topic;
	std::string out_topic;
	n_.param("new_point_pointmsg/number_of_laserscans", no_laserscans, 10);
	n_.param("new_point_pointmsg/cloud_topic", topic, std::string("/pointcloud2"));
	n_.param("new_point_pointmsg/output_cloud_topic", out_topic, std::string("/new_point_cloud"));

	ros::Subscriber sub = n_.subscribe (topic, 1, cloud_callback);

	point_pub = n_.advertise<sensor_msgs::PointCloud2> (out_topic, 1);

	ros::spin ();
}
