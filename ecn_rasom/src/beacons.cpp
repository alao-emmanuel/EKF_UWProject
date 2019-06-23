#include <math.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <random>
#include <sstream>
#include <string>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <yaml-cpp/yaml.h>

#include <ecn_rasom/beacons_ping.h>

using namespace std;

// global variables for subscriber
bool odom_ok;
double mean = 0.1;
double stddev = 0.3;
ros::Publisher setpoint_pub;
geometry_msgs::Pose pose;
visualization_msgs::Marker marker;

default_random_engine generator;
normal_distribution<double> dist(mean, stddev);

double beacon_range = 0.0;
double angle_error = 0.0;

void poseCallback(const nav_msgs::OdometryConstPtr& msg)
{
  pose = msg->pose.pose;
  odom_ok = true;
}

// global vars for BC / setpoint
YAML::Node bc;
geometry_msgs::PoseStamped setpoint;

// get a particular coordinate of a given BC
double coord(int idx, std::string var)
{
  return bc[idx][var].as<double>();
}

void initializeMarker()
{
  // Fetch node name. Markers will be blue if the word "blue" is in the name, red otherwise.
  std::string nodeName;
  nodeName = ros::this_node::getName();
  // Create a marker for this node. Only timestamp and position will be later updated.
  marker.header.frame_id = "world";
  marker.ns = nodeName;
  marker.id = 0;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.2;
  marker.scale.y = 0.2;
  marker.scale.z = 0.2;
  marker.color.r = 1.0f;
  marker.color.g = 0.7f;
  marker.color.b = 0.7f;
  marker.color.a = 1.0;
}

// Function to publish a marke at a given (x,y,z) position.

void publishMarkerAt(geometry_msgs::PoseStamped markerPos)
{
  marker.header.stamp = ros::Time::now();
  marker.pose.position.x = markerPos.pose.position.x;
  marker.pose.position.y = markerPos.pose.position.y;
  marker.pose.position.z = markerPos.pose.position.z;
  marker.lifetime = ros::Duration();
  setpoint_pub.publish(marker);
}

void writeBC(int idx)
{
  setpoint.pose.position.x = coord(idx, "x");
  setpoint.pose.position.y = coord(idx, "y");
  setpoint.pose.position.z = coord(idx, "z");
  setpoint.pose.orientation.x = 0;
  setpoint.pose.orientation.y = 0;
  setpoint.pose.orientation.z = 0;
  setpoint.pose.orientation.w = 1;

  setpoint.header.stamp = ros::Time::now();
  marker.id = idx;
  publishMarkerAt(setpoint);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "beacons");
  ros::NodeHandle nh("~");

  if (nh.getParam("noise_mean", mean))
  {
    ROS_INFO("Got param: mean");
  }
  else
  {
    ROS_ERROR("Failed to get param 'mean'");
  }

  if (nh.getParam("noise_stddev", stddev))
  {
    ROS_INFO("Got param: noise_stddev");
  }
  else
  {
    ROS_ERROR("Failed to get param 'noise_stddev'");
  }

  // subscriber
  odom_ok = false;
  ros::Subscriber state_sub = nh.subscribe<nav_msgs::Odometry>("/auv/ground_truth", 1, poseCallback);

  // publisher
  setpoint_pub = nh.advertise<visualization_msgs::Marker>("/auv/beacons_position_setpoint", 1);

  ros::Publisher beacons_pub = nh.advertise<ecn_rasom::beacons_ping>("/beacons_detected", 1);

  // load beacons
  std::string bc_path = ros::package::getPath("ecn_rasom") + "/config/beacons.yaml";
  YAML::Node node = YAML::LoadFile(bc_path);

  // get beacons
  bc = node["bc"];
  std::cout << "Found " << bc.size() << " beacons" << std::endl;

  int idx;
  for (idx = 0; idx < bc.size(); ++idx)
  {
    std::cout << "   BC #" << idx << ": ";
    std::cout << "x = " << coord(idx, "x");
    std::cout << ", y = " << coord(idx, "y");
    std::cout << ", z = " << coord(idx, "z");
    std::cout << std::endl;
  }

  // write first BC
  writeBC(0);
  initializeMarker();
  ros::Rate rate(20);

  // beacon index
  idx = 0;

  while (ros::ok())
  {
    if (odom_ok)
    {
      // empty available beacon array
      ecn_rasom::beacons_ping beacons_msg;
      for (idx = 0; idx < bc.size(); ++idx)
      {
        // check Cartesian distance to current bc
        // update setpoint if needed

        writeBC(idx);

        beacon_range = sqrt(pow(setpoint.pose.position.x - pose.position.x, 2.0) +
                            pow(setpoint.pose.position.y - pose.position.y, 2.0) +
                            pow(setpoint.pose.position.z - pose.position.z, 2.0));

        // beacon_range += dist(generator);

        std::cout << "Beacon: " << idx;
        std::cout << " Range is: " << beacon_range << std::endl << std::endl;

        beacons_msg.id.push_back(idx);
        beacons_msg.x.push_back(setpoint.pose.position.x);
        beacons_msg.y.push_back(setpoint.pose.position.y);
        beacons_msg.z.push_back(setpoint.pose.position.z);
        beacons_msg.r.push_back(beacon_range);
      }
      beacons_pub.publish(beacons_msg);
      odom_ok = false;
      //   ++idx;
      //   if (idx >= bc.size())
      // 	idx = 0;
    }

    // publish setpoint
    // setpoint_pub.publish(setpoint);

    ros::spinOnce();
    rate.sleep();
  }
}
