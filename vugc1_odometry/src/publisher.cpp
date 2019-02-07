#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <cmath>

double x = 0.0;
double y = 0.0;
double th = 0.0;
double vx = 0.0;
double vy = 0.0;

ros::Time current_time, last_time;
ros::Publisher odometry_publisher;

void callback(const sensor_msgs::Imu::ConstPtr& msg) {  
  current_time = msg->header.stamp;

  double dt = (current_time - last_time).toSec();
  double ax = msg->linear_acceleration.x;
  double ay = msg->linear_acceleration.y;
  double vth = msg->angular_velocity.z;

  x += 0.5 * (ax * dt * dt) + (vx * dt);
  y += 0.5 * (ay * dt * dt) + (vy * dt);
  th += vth * dt;
  vx += ax * dt;
  vy += ay * dt;

  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

  // tf
  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = current_time;
  odom_trans.header.frame_id = "odom";
  odom_trans.child_frame_id = "base_link";

  odom_trans.transform.translation.x = x;
  odom_trans.transform.translation.y = y;
  odom_trans.transform.translation.z = 0.0;
  odom_trans.transform.rotation = odom_quat;

  static tf::TransformBroadcaster odometry_broadcaster;
  odometry_broadcaster.sendTransform(odom_trans);
  ROS_INFO("[vugc1_odometry_publisher]: sendTransform");

  nav_msgs::Odometry odom;
  odom.header.stamp = current_time;
  odom.header.frame_id = "odom";

  // position
  odom.pose.pose.position.x = x;
  odom.pose.pose.position.y = y;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = odom_quat;

  // velocity
  odom.child_frame_id = "base_link";
  odom.twist.twist.linear.x = vx;
  odom.twist.twist.linear.y = vy;
  odom.twist.twist.angular.z = vth;

  odometry_publisher.publish(odom);
  ROS_INFO("[vugc1_odometry_publisher]: published to /odom");

  last_time = ros::Time::now();
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "vugc1_odometry_publisher");
  ros::NodeHandle n;
  ROS_INFO("[vugc1_odometry_publisher]: init");

  current_time = ros::Time::now();
  last_time = ros::Time::now();

  ros::Subscriber imu_subscriber = n.subscribe("imu", 1, &callback);
  odometry_publisher = n.advertise<nav_msgs::Odometry>("odom", 1);

  // ros::Rate r(1);
  // while(ros::ok()) {
  //   ros::spinOnce();
  //   r.sleep();
  // }

  ros::spin();

  return 0;
}
