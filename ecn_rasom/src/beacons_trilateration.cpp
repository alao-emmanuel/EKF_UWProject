#include <math.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <visp/vpSubMatrix.h>
#include <sstream>
#include <string>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>

#include <ecn_rasom/beacons_ping.h>

using namespace std;

// global variables for subscriber
bool odom_ok;

vpMatrix BeaconsMat;
vpColVector BeaconsBvec;
vpColVector auvPositionVec(3);
geometry_msgs::PoseWithCovarianceStamped poseStamped;
ecn_rasom::beacons_ping beacons_msg;

geometry_msgs::Pose pose;
geometry_msgs::Pose statepose;
void poseCallback(const nav_msgs::OdometryConstPtr& msg)
{
  pose = msg->pose.pose;
}
void beaconCallback(ecn_rasom::beacons_ping msg)
{
  beacons_msg = msg;
  //   poseStamped.header = msg.header;
  poseStamped.header.stamp = ros::Time::now();
  odom_ok = true;
}

void stateCallback(const nav_msgs::OdometryConstPtr& msg2)
{
  statepose = msg2->pose.pose;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "beacons_trilateration");
  ros::NodeHandle nh;

  // subscriber
  odom_ok = false;
  ros::Subscriber state_sub = nh.subscribe<nav_msgs::Odometry>("/auv/ground_truth", 1, poseCallback);

  ros::Subscriber state_sub2 = nh.subscribe<nav_msgs::Odometry>("/auv/state", 1, stateCallback);

  ros::Subscriber beacon_sub = nh.subscribe<ecn_rasom::beacons_ping>("/beacons_detected", 1, beaconCallback);

  // publisher
  ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/usbl_range", 1);
  poseStamped.header.frame_id = "world";

  ros::Publisher error_pub = nh.advertise<ecn_rasom::beacons_ping>("/pose_error", 1);

  ros::Rate rate(20);

  // waypoint index
  int idx = 0;
  double W, Z, x, y, y2;

  while (ros::ok())
  {
    if (odom_ok)
    {
      int rows = beacons_msg.id.size();
      BeaconsMat.resize(rows, 3);
      BeaconsBvec.resize(rows);

      ecn_rasom::beacons_ping error_msg;

      if (rows >= 4)
      {
        for (int i = 0; i < (rows - 1); i++)
        {
          BeaconsMat[i][0] = beacons_msg.x[i + 1] - beacons_msg.x[0];
          BeaconsMat[i][1] = beacons_msg.y[i + 1] - beacons_msg.y[0];
          BeaconsMat[i][2] = beacons_msg.z[i + 1] - beacons_msg.z[0];

          BeaconsBvec[i] =

              pow(beacons_msg.r[0], 2) - pow(beacons_msg.r[i + 1], 2) - pow(beacons_msg.x[0], 2) +
              pow(beacons_msg.x[i + 1], 2) - pow(beacons_msg.y[0], 2) + pow(beacons_msg.y[i + 1], 2) -
              pow(beacons_msg.z[0], 2) + pow(beacons_msg.z[i + 1], 2);
        }

        auvPositionVec = 0.5 * BeaconsMat.pseudoInverse() * BeaconsBvec;

        poseStamped.pose.pose.position.x = auvPositionVec[0];
        poseStamped.pose.pose.position.y = auvPositionVec[1];
        poseStamped.pose.pose.position.z = auvPositionVec[2];

        double beacon_pose_error = sqrt(pow(auvPositionVec[0] - statepose.position.x, 2.0) +
                                        pow(auvPositionVec[1] - statepose.position.y, 2.0) +
                                        pow(auvPositionVec[2] - statepose.position.z, 2.0));

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

        poseStamped.pose.covariance = { 0.3, 0,   0,   0,     0,     0,        // covariance on x
                                        0,   0.3, 0,   0,     0,     0,        // covariance on y
                                        0,   0,   0.3, 0,     0,     0,        // covariance on z
                                        0,   0,   0,   99999, 0,     0,        // large covariance on rot x
                                        0,   0,   0,   0,     99999, 0,        // large covariance on rot y
                                        0,   0,   0,   0,     0,     99999 };  // large covariance on rot z
        ROS_ERROR_STREAM("auvPoseError: \n" << beacon_pose_error);
        // std::cout << BeaconsMat << std::endl;

        // publish pose
        pose_pub.publish(poseStamped);
        odom_ok = false;
      }

      //   W = pow(beacons_msg.r[idx], 2) - pow(beacons_msg.r[idx + 1], 2) - pow(beacons_msg.x[idx], 2) -
      //       pow(beacons_msg.y[idx], 2) + pow(beacons_msg.x[idx + 1], 2) + pow(beacons_msg.y[idx + 1], 2);
      //   Z = pow(beacons_msg.r[idx + 1], 2) - pow(beacons_msg.r[idx + 2], 2) - pow(beacons_msg.x[idx + 1], 2) -
      //       pow(beacons_msg.y[idx + 1], 2) + pow(beacons_msg.x[idx + 2], 2) + pow(beacons_msg.y[idx + 2], 2);

      //   x = (W * (beacons_msg.y[idx + 2] - beacons_msg.y[idx + 1]) - Z * (beacons_msg.y[idx + 1] -
      //   beacons_msg.y[idx])) /
      //       (2 * ((beacons_msg.x[idx + 1] - beacons_msg.x[idx]) * (beacons_msg.y[idx + 2] - beacons_msg.y[idx + 1]) -
      //             (beacons_msg.x[idx + 2] - beacons_msg.x[idx + 1]) * (beacons_msg.y[idx + 1] -
      //             beacons_msg.y[idx])));
      //   y = (W - 2 * x * (beacons_msg.x[idx + 1] - beacons_msg.x[idx])) /
      //       (2 * (beacons_msg.y[idx + 1] - beacons_msg.y[idx]));

      //   // y2 is a second measure of y to mitigate errors
      //   y2 = (Z - 2 * x * (beacons_msg.x[idx + 2] - beacons_msg.x[idx + 1])) /
      //        (2 * (beacons_msg.y[idx + 2] - beacons_msg.y[idx + 1]));
      //   y = (y + y2) / 2;
    }

    ros::spinOnce();
    rate.sleep();
  }
}
