#include<geometry_msgs/Twist.h>
#include "carote/Positioner.h"
#include "carote/Utils.h"

carote::Positioner::Positioner(const std::string& _name)
:
	Controller(_name)
{
	// ros stuff: dynamic reconfiguration of controller parameters
	dynamic_reconfigure::Server<carote::PositionerConfig>::CallbackType f;
	f=boost::bind(&carote::Positioner::cbReconfigure,this,_1,_2);
	server_.setCallback(f);
}

carote::Positioner::~Positioner(void)
{
}

void carote::Positioner::clean(void)
{
	u_.clear();
}

void carote::Positioner::cbControl(const ros::TimerEvent& _event)
{
}

void carote::Positioner::cbReconfigure(carote::PositionerConfig& _config, uint32_t _level)
{
	// update parameters
	control_params_=_config;

	if( control_params_.enabled )
	{
		ros::Duration period(1.0/control_params_.rate);
		this->start(period);
	}
	else
	{
		this->stop();
	}
}
