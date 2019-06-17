#include <ros/ros.h>
#include <ros/package.h>
#include <string>
#include <math.h>
#include <sstream>

#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <yaml-cpp/yaml.h>

#include <ecn_rasom/beacons_ping.h>

using namespace std;

// global variables for subscriber
bool odom_ok;
geometry_msgs::Pose pose;
ecn_rasom::beacons_ping beacons_msg;


double beacon_range = 0.0;
double angle_error = 0.0;

void poseCallback(const nav_msgs::OdometryConstPtr & msg)
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
}

int main (int argc, char** argv)
{
	ros::init(argc, argv, "beacons");
	ros::NodeHandle nh;

	// subscriber
	odom_ok = false;
	ros::Subscriber state_sub = nh.subscribe<nav_msgs::Odometry> ("/auv/state", 1, poseCallback);

	// publisher
	ros::Publisher setpoint_pub = nh.advertise<geometry_msgs::PoseStamped>("/auv/beacons_position_setpoint", 10);
	setpoint.header.frame_id = "world";
	
	ros::Publisher beacons_pub = nh.advertise<ecn_rasom::beacons_ping>("/beacons_detected", 10);
	
	// load waypoints
	std::string bc_path = ros::package::getPath("ecn_rasom") + "/config/beacons.yaml";
	YAML::Node node = YAML::LoadFile(bc_path);

	// get beacons
	bc = node["bc"];
	std::cout << "Found " << bc.size() << " beacons" << std::endl;
	int idx;
	for(idx = 0; idx < bc.size(); ++idx)
	{
		std::cout << "   BC #" << idx << ": ";
		std::cout << "x = " << coord(idx, "x");
		std::cout << ", y = " << coord(idx, "y");
		std::cout << ", z = " << coord(idx, "z");
		std::cout << std::endl;
	}

	// write first BC
	writeBC(0);

	ros::Rate rate(20);

	// beacon index
	idx = 0;

	while (ros::ok())
	{

		if(odom_ok)
		{
			//for(idx = 0; idx < bc.size(); ++idx){
				
				// check Cartesian distance to current bc
				// update setpoint if needed
				
				writeBC(idx);
				
				beacon_range = sqrt(pow(setpoint.pose.position.x-pose.position.x,2.0)
				+pow(setpoint.pose.position.y-pose.position.y,2.0)
				+pow(setpoint.pose.position.z-pose.position.z,2.0));


				std::cout<<"Beacon: "<< idx;
				std::cout<<"  Beacon Range is: "<< beacon_range<<std::endl<<std::endl;
				
                beacons_msg.id[idx] = idx;
                beacons_msg.x[idx] = setpoint.pose.position.x;
                beacons_msg.y[idx] = setpoint.pose.position.y;
                beacons_msg.z[idx] = setpoint.pose.position.z;
                beacons_msg.r[idx] = beacon_range;
				
                beacons_pub.publish(beacons_msg);
				
				
			//}
			++idx;
			if(idx>=bc.size()) idx=0;

		}
		
				

		// publish setpoint
		setpoint_pub.publish(setpoint);
		


		

		ros::spinOnce();
		rate.sleep();
	}

}
