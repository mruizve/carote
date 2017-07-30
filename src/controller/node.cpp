#include<ros/ros.h>
#include "carote/Follower.h"

int main(int argc, char **argv)
{
	// initialize ros stuff
	ros::init(argc,argv,ros::this_node::getName());

	// initialize follower object
	carote::Follower follower(ros::this_node::getName());

	// handle events
	ros::spin();
	
	return 0;
}
