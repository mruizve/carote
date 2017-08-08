#include "carote/Controller.h"

int main(int argc, char **argv)
{
	// initialize ros stuff
	ros::init(argc,argv,ros::this_node::getName());

	// create the follower controller node
	carote::Controller control(ros::this_node::getName());

	// initialize robot
	control.work();
	
	exit(EXIT_SUCCESS);
}
