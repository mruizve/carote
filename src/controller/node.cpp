#include<ros/ros.h>
#include "carote/Controller.h"

int main(int argc, char **argv)
{
	int fd[2];
	pid_t pid;

	// create communication pipe between precesses
	if( 0>pipe(fd) )
	{
		// initialize ros stuff
		ros::init(argc,argv,ros::this_node::getName());

		// show error and exit node
		ROS_ERROR_STREAM("pipe() error during multi-master setup");
		exit(EXIT_FAILURE);
	}

	// create multiple process
	pid=fork();
	if( 0>pid )
	{
		// initialize ros stuff
		ros::init(argc,argv,ros::this_node::getName());

		// show error and exit node
		ROS_ERROR_STREAM("fork() error during multi-master setup");
		exit(EXIT_FAILURE);
	}

	if( 0==pid )
	{
		// child process: controller->actuator
		// -----------------------------------

		// setup pipe for reading only
		close(fd[1]);

		// retrieve the robot URI
		char uri[4096]={0};
		if( 0>=read(fd[0],&uri,sizeof(uri)) )
		{
			// initialize ros stuff
			ros::init(argc,argv,ros::this_node::getName());

			// show error and exit node
			ROS_ERROR_STREAM("read() error during multi-master setup");
			exit(EXIT_FAILURE);
		}

		// replace the ROS_MASTER_URI environment variable
		setenv("ROS_MASTER_URI",uri,1);

		// initialize ros stuff
		ros::init(argc,argv,ros::this_node::getName());

		ros::spin();
	}
	else
	{
		// parent process: controller->observer
		// ------------------------------------

		// setup pipe for writing only
		close(fd[0]);

		// initialize ros stuff
		ros::init(argc,argv,ros::this_node::getName());
		ros::NodeHandle node("~");

		// get the robot URI
		std::string uri;
		node.getParam("/carote/uri/robot",uri);
		if( uri.empty() )
		{
			// show error and exit node
			ROS_ERROR_STREAM("invalid robot URI during multi-master setup");
			exit(EXIT_FAILURE);
		}

		// send URI to the child process
		if( 0>write(fd[1],uri.c_str(),uri.length()) )
		{
			// show error and exit node
			ROS_ERROR_STREAM("write() error during multi-master setup");
			exit(EXIT_FAILURE);
		}

		// initialize actuator object
		carote::Follower follower(ros::this_node::getName());

		// handle events
		ros::spin();
	}
	
	exit(EXIT_SUCCESS);
}
