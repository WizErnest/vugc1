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

double reduce_noise(double value, double threshold) {
  if (std::abs(value) < threshold) {
    return 0.0;
  } else {
    return value;
  }
}

void callback(const sensor_msgs::Imu::ConstPtr& msg) {  
  current_time = ros::Time::now();
  double dt = (current_time - last_time).toSec();
  double ax = reduce_noise(msg->linear_acceleration.x, 0.9);
  double ay = reduce_noise(msg->linear_acceleration.y, 0.9);
  double vth = reduce_noise(msg->angular_velocity.z, 0.02);

  x += 0.5 * (ax * dt * dt) + (vx * dt);
  y += 0.5 * (ay * dt * dt) + (vy * dt);
  th += vth * dt;
  vx += ax * dt;
  vy += ay * dt;

  ROS_INFO("dt = %f", dt);
  ROS_INFO("x = %f", x);
  ROS_INFO("y = %f", y);
  ROS_INFO("th = %f", th);
  ROS_INFO("ax = %f", ax);
  ROS_INFO("ay = %f", ay);
  ROS_INFO("vx = %f", vx);
  ROS_INFO("vy = %f", vy);
  ROS_INFO("vth = %f", vth);

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

  tf::TransformBroadcaster odometry_broadcaster;
  odometry_broadcaster.sendTransform(odom_trans);

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

  last_time = ros::Time::now();
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "vugc1_odometry_publisher");
  ros::NodeHandle n;

  current_time = ros::Time::now();
  last_time = ros::Time::now();

  ros::Subscriber imu_subscriber = n.subscribe("imu", 1000, callback);
  odometry_publisher = n.advertise<nav_msgs::Odometry>("odom", 50);

  ros::spin();

  return 0;
}