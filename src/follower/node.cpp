#include<ros/ros.h>
#include "carote/Controller.h"

int main(int argc, char **argv)
{
	// initialize ros stuff
	ros::init(argc,argv,ros::this_node::getName());

	// create the follower controller node
	carote::Follower controller(ros::this_node::getName());

	// handle events
	ros::spin();

	exit(EXIT_SUCCESS);
}
