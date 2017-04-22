#include<ros/ros.h>
#include "nodelet/loader.h"

#include "carote/Target.h"

int main(int argc, char **argv)
{
	// initialize ros stuff
	ros::init(argc,argv,ros::this_node::getName());

	// initialize target object
	carote::Target target(ros::this_node::getName());

	// handle events
	ros::spin();
	
	return 0;
}
