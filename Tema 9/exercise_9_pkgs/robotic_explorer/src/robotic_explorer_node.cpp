#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>

#include <stdlib.h>

// For rand() and RAND_MAX

void processScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	int n_ranges = msg->ranges.size();
	double nearest = *(std::min_element(msg->ranges.begin(),msg->ranges.end()));
	ROS_INFO("[robot_explorer] I've a total of %i measurements to process! Are you ready? Nearest=%.2f",n_ranges, nearest);
}

int main(int argc, char **argv)
{

	// Initialize the ROS system and become a node.

	ros::init( argc , argv , "exploring") ;

    ROS_INFO("[robot_explorer] Robotic explorer node running and initialized! Let's have some fun!");

	ros::NodeHandle nh;

	// Create a subscriber object
	ros::Subscriber sub = nh.subscribe("/laser_scan", 1000, processScanCallback);

	// Create a publisher object
	ros::Publisher pub = nh.advertise <geometry_msgs::Twist>("/cmd_vel" , 1000) ;

	// Seed the random number generator.
	srand ( time (0) ) ;

	// Loop at 2Hz until the node is shut down.
  ros::Rate rate (2) ;
  ros::Time begin = ros::Time::now();

  while ( begin.toSec() == 0 )
    begin = ros::Time::now();

  double ellapsed_time = 0;

  while ( ( ros::ok () ) && ( ellapsed_time < 60*5 ) )
	{
		// Create and fill in the message. The other four
		// fields , which are ignored by stage, default to 0.

		geometry_msgs::Twist msg;
		msg.linear.x = double( rand() ) / double(RAND_MAX);
		msg.angular.z = ( 2*double(rand()) / double(RAND_MAX) ) - 1;

		// Publish the message.

		pub.publish(msg) ;

		// Send a message to rosout with the details .
		ROS_INFO("[robot_explorer] Sending random velocity command: linear= %.2f angular=%.2f", msg.linear.x, msg.angular.z );

		// Wait until it 's time for another iteration .
		rate.sleep () ;
		ros::spinOnce(); // give time to receive /base_scan messages

        ros::Time current = ros::Time::now();
        ellapsed_time = (current - begin).toSec();
        ROS_INFO("[robot_explorer] Ellpased time: %.2f", ellapsed_time );
	}

}
