#ifndef _CAROTE_PARAMS_H_
#define _CAROTE_PARAMS_H_

#include<ros/ros.h>

template<typename T>
T getParam(ros::NodeHandle& n, const std::string& name, const T& value)
{
	T v;
	if( n.getParam(name,v) )
	{
		return v;
	}
	else
	{
		ROS_WARN_STREAM("Setting default value for parameter '" << name << "'");
		return value;
	}
}

#endif
