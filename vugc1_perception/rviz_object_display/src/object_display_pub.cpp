#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <sstream>

int main(int argc char **argv) {
	ros::init(argc, argv, "basic_shapes");
	ros::NodeHangle n;
	ros::Rate r(1);
	    ros::Publisher object_pub = n.advertise<visualization_msgs::Marker>("vugc1_pedestrian_visualization", 1);
	uint32_t shape = visualization_msgs:Marker::CYLINDER;
	while (ros::ok()) {
		createMarker(0, shape, 0, 0, 0);	
	

		while (object_pub.getNumSubscribers() < 1) {
			if (!ros::ok()) {
				return 0;
			}
			ROS_WARN_ONCE("Please create a subscriber to the marker");
			sleep(1);
		}
		object_pub.publish(marker);
	}
}

visualization_msgs::Marker createMarker(int id, uint32_t shape, int x, int y, int z) {
	visualization_msgs::Marker marker;
	marker.header.frame_id = "/my_frame";
	marker.header.stamp = ros::Time::now();

	marker.ns = "basic_shapes";
	marker.id = id;

	marker.type = shape;

	marker.action = visualization_msgs::Marker::ADD;

	marker.pose.position.x = x;
	marker.pose.position.y = y;
	marker.pose.position.z = z;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;

	marker.color.r = 1.0f;
	marker.color.g = 0.0f;
	marker.color.b = 0.0f;
	marker.color.a = 1.0;

	marker.lifetime = ros::Duration();

	return marker;
			  
}
