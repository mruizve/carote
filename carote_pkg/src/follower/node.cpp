#include<signal.h>
#include<ros/ros.h>
#include<ros/xmlrpc_manager.h>
#include "carote/Follower.h"

// signal-safe flag for whether shutdown is requested
sig_atomic_t volatile g_shutdown_flag=0;

// replacement SIGINT handler
void shutdownSigInt(int sig)
{
	// set flag
	g_shutdown_flag=1;
}

// replacement "shutdown" XMLRPC callback (pure black magic)
void shutdownXmlRpc(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result)
{
	int nparams=0;

	if( XmlRpc::XmlRpcValue::TypeArray==params.getType() )
	{
		nparams=params.size();
	}

	if( 1<nparams )
	{
		std::string reason=params[1];
		ROS_WARN_STREAM("shutdown request received: " << reason);

		// set flag
		g_shutdown_flag=1;
	}

	result=ros::xmlrpc::responseInt(1,"",0);
}

int main(int argc, char **argv)
{
	// initialize ros stuff
	ros::init(argc,argv,ros::this_node::getName(),ros::init_options::NoSigintHandler);

	// override SIGINT handler and XMLRPC shutdown
	signal(SIGINT,shutdownSigInt);
	ros::XMLRPCManager::instance()->unbind("shutdown");
	ros::XMLRPCManager::instance()->bind("shutdown",shutdownXmlRpc);

	// create the follower controller node
	carote::Follower follower(ros::this_node::getName());

	// initialize robot
	follower.work();

	// handle events with custom ros::spin() until a shutdown request is received
	while( !g_shutdown_flag )
	{
		ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.1));
	}

	// send robot to the home configuration
	follower.home();

	// real shutdown
	ros::shutdown();
	
	exit(EXIT_SUCCESS);
}
