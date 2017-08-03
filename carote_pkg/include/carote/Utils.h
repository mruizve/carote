#ifndef _CAROTE_UTILS_H_
#define _CAROTE_UTILS_H_

#include<ros/ros.h>

#define CAROTE_NODE_ABORT(stream) do \
{ \
	ROS_FATAL_STREAM("[" << ros::this_node::getName() << "] " << stream); \
	exit(EXIT_SUCCESS); \
} while(0)


#endif
