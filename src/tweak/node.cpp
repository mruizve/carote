#include<ros/ros.h>
#include "carote/Tweak.h"

int main(int argc, char **argv)
{
	// initialize ros stuff
	ros::init(argc,argv,ros::this_node::getName());

	// initialize target object
	carote::Tweak tweak(ros::this_node::getName());

	// handle events
	ros::spin();
	
	return 0;
}
