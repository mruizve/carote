#include "carote/Operator.h"

int main(int argc, char **argv)
{
	// initialize ros stuff
	ros::init(argc,argv,ros::this_node::getName());

	// initialize target object
	carote::Operator operatore(ros::this_node::getName());

	// handle events
	ros::spin();
	
	return 0;
}
