/*
#include<arpa/inet.h>
#include<sys/socket.h>
#include<boost/date_time/posix_time/posix_time.hpp>
*/
#include "carote/Tweak.h"
#include "carote/Utils.h"

carote::Tweak::Tweak(int &_argc, char **_argv)
{
	// create communication pipe between precesses
	if( 0>pipe(pipe_fd_) )
	{
		// initialize ros stuff
		ros::init(_argc,_argv,ros::this_node::getName());

		// show error and exit node
		CAROTE_NODE_ABORT("pipe() error during multi-master setup");
	}

	// create multiple process
	pid_=fork();
	if( 0>pid_ )
	{
		// initialize ros stuff
		ros::init(_argc,_argv,ros::this_node::getName());

		// show error and exit node
		CAROTE_NODE_ABORT("fork() error during multi-master setup");
	}

	if( 0==pid_ )
	{
		// child process: tweak publisher
		// ------------------------------

		// setup pipe for reading only
		close(pipe_fd_[1]);

		// retrieve the robot URI
		char uri[4096]={0};
		if( 0>=read(pipe_fd_[0],&uri,sizeof(uri)) )
		{
			// initialize ros stuff
			ros::init(_argc,_argv,ros::this_node::getName());

			// show error and exit node
			CAROTE_NODE_ABORT("read() error during multi-master setup");
		}

		// replace the ROS_MASTER_URI environment variable
		setenv("ROS_MASTER_URI",uri,1);

		// initialize ros stuff
		ros::init(_argc,_argv,ros::this_node::getName());
		name_=ros::this_node::getName();
	}
	else
	{
		// parent process: tweak listener
		// ------------------------------

		// setup pipe for writing only
		close(pipe_fd_[0]);

		// initialize ros stuff
		ros::init(_argc,_argv,ros::this_node::getName());
		name_=ros::this_node::getName();

		// get the robot URI
		std::string uri;
		ros::NodeHandle node("~");

		node.getParam("/carote/uri/robot",uri);
		if( uri.empty() )
		{
			// show error and exit node
			CAROTE_NODE_ABORT("invalid robot URI during multi-master setup");
		}

		// send URI to the child process
		if( 0>write(pipe_fd_[1],uri.c_str(),uri.length()) )
		{
			// show error and exit node
			CAROTE_NODE_ABORT("write() error during multi-master setup");
		}
	}
}

carote::Tweak::~Tweak(void)
{
}
