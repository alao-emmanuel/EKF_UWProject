    #include <ros/ros.h>
    #include <ros/package.h>
    #include <string>
    #include <math.h>
    #include <sstream>

    #include <geometry_msgs/PoseWithCovarianceStamped.h>
    #include <geometry_msgs/Point.h>
    #include <nav_msgs/Odometry.h>

	#include <ecn_rasom/beacons_ping.h>
	
    using namespace std;

    // global variables for subscriber
    bool odom_ok;
    geometry_msgs::PoseWithCovarianceStamped poseStamped;
	ecn_rasom::beacons_ping beacons_msg;
	
	void beaconCallback(ecn_rasom::beacons_ping msg)
	{
	    beacons_msg = msg;

		odom_ok = true;
	}
    
    int main (int argc, char** argv)
    {
        ros::init(argc, argv, "waypoint");
        ros::NodeHandle nh;

        // subscriber
        odom_ok = false;
        ros::Subscriber beacon_sub = nh.subscribe<ecn_rasom::beacons_ping> ("/beacons_detected", 1, beaconCallback);

        // publisher
        ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/usbl_range", 1);
		poseStamped.header.frame_id = "world";
		
        
        ros::Rate rate(10);

        // waypoint index
        int idx = 0;
        double W, Z,x,y,y2;

        while (ros::ok())
        {

        if(odom_ok)
            {
              W = pow(beacons_msg.r[idx],2) - pow(beacons_msg.r[idx+1],2) - pow(beacons_msg.x[idx],2) - pow(beacons_msg.y[idx],2) + pow(beacons_msg.x[idx+1],2) + pow(beacons_msg.y[idx+1],2);
              Z = pow(beacons_msg.r[idx+1],2) - pow(beacons_msg.r[idx+2],2) - pow(beacons_msg.x[idx+1],2) - pow(beacons_msg.y[idx+1],2) + pow(beacons_msg.x[idx+2],2) + pow(beacons_msg.y[idx+2],2);

              x = (W*(beacons_msg.y[idx+2]-beacons_msg.y[idx+1]) - Z*(beacons_msg.y[idx+1]-beacons_msg.y[idx])) / (2 * ((beacons_msg.x[idx+1]-beacons_msg.x[idx])*(beacons_msg.y[idx+2]-beacons_msg.y[idx+1]) - (beacons_msg.x[idx+2]-beacons_msg.x[idx+1])*(beacons_msg.y[idx+1]-beacons_msg.y[idx])));
              y = (W - 2*x*(beacons_msg.x[idx+1]-beacons_msg.x[idx])) / (2*(beacons_msg.y[idx+1]-beacons_msg.y[idx]));

              //y2 is a second measure of y to mitigate errors
              y2 = (Z - 2*x*(beacons_msg.x[idx+2]-beacons_msg.x[idx+1])) / (2*(beacons_msg.y[idx+2]-beacons_msg.y[idx+1]));
              y = (y + y2) / 2;

              poseStamped.pose.pose.position.x = x;
              poseStamped.pose.pose.position.y = y;


			}
			poseStamped.header.stamp = ros::Time::now();
			// publish pose
			pose_pub.publish(poseStamped);
            
            ros::spinOnce();
            rate.sleep();
        }

    }
