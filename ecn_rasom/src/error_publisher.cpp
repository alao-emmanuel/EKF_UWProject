#include <math.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <random>
#include <sstream>
#include <string>

#include <ecn_rasom/beacons_ping.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>

using namespace std;

// global variables for subscriber
bool odom_ok;
bool odom2_ok;
bool odom3_ok;

ros::Publisher setpoint_pub;
geometry_msgs::Pose pose;
geometry_msgs::Pose statepose;
int idx = 0;
void poseCallback(const nav_msgs::OdometryConstPtr& msg)
{
  pose = msg->pose.pose;
  odom_ok = true;
}
void stateCallback(const nav_msgs::OdometryConstPtr& msg2)
{
  statepose = msg2->pose.pose;
  odom2_ok = true;
}

void beaconCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg3)
{
  pose = msg3->pose.pose;
  odom3_ok = true;
}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "error_publisher");
  ros::NodeHandle nh;

  // subscriber
  odom_ok = false;
  odom2_ok = false;
  odom3_ok = false;
  ros::Subscriber state_sub = nh.subscribe<nav_msgs::Odometry>("/auv/ground_truth", 1, poseCallback);

  // ros::Subscriber state_sub2 = nh.subscribe<nav_msgs::Odometry>("/auv/state", 1, stateCallback);

  ros::Subscriber state_sub3 = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/usbl_range", 1, beaconCallback);

  // publisher

  ros::Publisher error_pub = nh.advertise<ecn_rasom::beacons_ping>("/position_error", 1);

  ros::Rate rate(20);

  while (ros::ok())
  {
    if (odom_ok && odom3_ok)
    {
      // empty available beacon array
      ecn_rasom::beacons_ping error_msg;

      // check pose errors
      idx++;
      double x_error = statepose.position.x - pose.position.x;

      double y_error = statepose.position.y - pose.position.y;

      double z_error = statepose.position.z - pose.position.z;

      double angle_error = atan2(statepose.orientation.z, statepose.orientation.w) * 2 -
                           atan2(pose.orientation.z, pose.orientation.w) * 2;

      error_msg.id.push_back(idx);
      error_msg.x.push_back(x_error);
      error_msg.y.push_back(y_error);
      error_msg.z.push_back(z_error);
      error_msg.r.push_back(angle_error);

      error_pub.publish(error_msg);

      odom_ok = false;
      odom2_ok = false;
      odom3_ok = false;
    }

    // publish setpoint
    // setpoint_pub.publish(setpoint);

    ros::spinOnce();
    rate.sleep();
  }
}
