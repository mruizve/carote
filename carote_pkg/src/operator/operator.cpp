#include<carote_msgs/OperatorStamped.h>
#include "carote/Operator.h"
#include "carote/Utils.h"

carote::Operator::Operator(const std::string& _name)
:
	node_("~"),
	name_(_name)
{
	std::string strpar;

	// prepare for advertise to the control topics of the platform
	if( !node_.getParam("/carote/topics/operator",strpar) )
	{
		CAROTE_NODE_ABORT("missing param '/carote/topics/operator' (must be defined at setup.launch)");
	}
	pub_command_=node_.advertise<carote_msgs::OperatorStamped>(strpar,1);

	// ros stuff: dynamic reconfiguration
	dynamic_reconfigure::Server<carote::OperatorConfig>::CallbackType f;
	f=boost::bind(&carote::Operator::cbReconfigure,this,_1,_2);
	server_.setCallback(f);
}

carote::Operator::~Operator(void)
{
}

void carote::Operator::cbReconfigure(carote::OperatorConfig& _config, uint32_t _level)
{
	// prepare operator message
	carote_msgs::OperatorStamped msg;
	msg.header.stamp=ros::Time::now();
	msg.data.lambda=_config.lambda;
	msg.data.phi=_config.phi;
	msg.data.rho=_config.rho;
	msg.data.z_lower=_config.z_lower;
	msg.data.z_upper=_config.z_upper;

	// send message to the controller
	pub_command_.publish(msg);
}
