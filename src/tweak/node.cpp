#include<ros/ros.h>
#include "carote/Tweak.h"

int main(int argc, char **argv)
{
	// multi-master tweak stuff
	int fd[2];
	pid_t pid;

	// create communication pipe between precesses
	if( 0>pipe(fd) )
	{
		// initialize ros stuff
		ros::init(argc,argv,ros::this_node::getName());

		// show error and exit node
		ROS_ERROR_STREAM("[" << ros::this_node::getName() << "] pipe() error during multi-master setup");
		exit(EXIT_SUCCESS);
	}

	// create multiple process
	pid=fork();
	if( 0>pid )
	{
		// initialize ros stuff
		ros::init(argc,argv,ros::this_node::getName());

		// show error and exit node
		ROS_ERROR_STREAM("[" << ros::this_node::getName() << "] fork() error during multi-master setup");
		exit(EXIT_SUCCESS);
	}

	if( 0==pid )
	{
		// child process: tweak publisher
		// ------------------------------

		// setup pipe for reading only
		close(fd[1]);

		// retrieve the robot URI
		char uri[4096]={0};
		if( 0>=read(fd[0],&uri,sizeof(uri)) )
		{
			// initialize ros stuff
			ros::init(argc,argv,ros::this_node::getName());

			// show error and exit node
			ROS_ERROR_STREAM("[" << ros::this_node::getName() << "] read() error during multi-master setup");
			exit(EXIT_SUCCESS);
		}

		// replace the ROS_MASTER_URI environment variable
		setenv("ROS_MASTER_URI",uri,1);

		// initialize ros stuff
		ros::init(argc,argv,ros::this_node::getName());

		// create the tweak publisher node
		carote::TweakPublisher tweak(ros::this_node::getName(),fd[0]);
	}
	else
	{
		// parent process: tweak listener
		// ------------------------------

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
			ROS_ERROR_STREAM("[" << ros::this_node::getName() << "] invalid robot URI during multi-master setup");
			exit(EXIT_SUCCESS);
		}

		// send URI to the child process
		if( 0>write(fd[1],uri.c_str(),uri.length()) )
		{
			// show error and exit node
			ROS_ERROR_STREAM("[" << ros::this_node::getName() << "] write() error during multi-master setup");
			exit(EXIT_SUCCESS);
		}

		// create the tweak listener node
		carote::TweakListener tweak(ros::this_node::getName(),fd[1]);

		// handle events
		ros::spin();
	}
	
	exit(EXIT_SUCCESS);
}
