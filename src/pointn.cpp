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


ros::Time prev_time, my_time ;
double off_set;
int size;


ros::Publisher point_pub;
sensor_msgs::PointCloud2 accumulator;
std::vector<sensor_msgs::PointCloud> v;


sensor_msgs::PointCloud2 bufferToAccumulator(const std::vector<sensor_msgs::PointCloud> buffer, double offset){
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
				pc1.points[i].z = pcl_.points[0].z + ((ros::Duration)(my_time - prev_time)).toSec() * offset;
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

void cloud_callback (sensor_msgs::PointCloud2Ptr msg){

	sensor_msgs::PointCloud pc1;
	sensor_msgs::convertPointCloud2ToPointCloud (*msg, pc1);


	// Replace vector with list for optimization
	v.push_back(pc1);
	if (v.size() > size){
		v.erase(v.begin());
		point_pub.publish(bufferToAccumulator(v, off_set));
	}

}

int main (int argc, char** argv){
	ros::init (argc, argv, "new_point_pointmsg");
	ros::NodeHandle n_;

	std::string topic;
	std::string out_topic;
	n_.param("new_point_pointmsg/offset", off_set, 1.0);
	n_.param("new_point_pointmsg/offset", size, 40);
	n_.param("new_point_pointmsg/cloud_topic", topic, std::string("/pointcloud2"));
	n_.param("new_point_pointmsg/output_cloud_topic", out_topic, std::string("/new_point_cloud"));

	ros::Subscriber sub = n_.subscribe (topic, 1, cloud_callback);

	point_pub = n_.advertise<sensor_msgs::PointCloud2> (out_topic, 1);

	ros::spin ();
}
