#include<carote_msgs/OperatorStamped.h>
#include "carote/Operator.h"
#include "carote/Utils.h"

carote::Operator::Operator(const std::string& _name)
:
	node_("~"),
	name_(_name)
{
	std::string strpar;

	// prepare for advertise
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

void carote::Operator::cbAuto(const ros::TimerEvent& _event)
{
	static double t=0.0;

	double w;
	if( 0.0!=config_.phi )
	{
		w=0.04/(config_.rho*sin(config_.phi));
	}
	else
	{
		w=0.0;
	}

	// prepare operator message
	carote_msgs::OperatorStamped msg;
	msg.header.stamp=ros::Time::now();
	msg.data.lambda=config_.phi*sin(w*t);
	msg.data.phi=config_.phi*cos(w*t);
	msg.data.rho=config_.rho;
	msg.data.z_lower=config_.z_lower;
	msg.data.z_upper=config_.z_upper;

	// send operator parameters to the controller
	pub_command_.publish(msg);

	t+=period_.toSec();
}

void carote::Operator::cbReconfigure(carote::OperatorConfig& _config, uint32_t _level)
{
	config_=_config;
	
	if( !config_.automatic )
	{
		timer_.stop();
		
		// prepare operator message
		carote_msgs::OperatorStamped msg;
		msg.header.stamp=ros::Time::now();
		msg.data.lambda=config_.lambda;
		msg.data.phi=config_.phi;
		msg.data.rho=config_.rho;
		msg.data.z_lower=config_.z_lower;
		msg.data.z_upper=config_.z_upper;

		// send operator parameters to the controller
		pub_command_.publish(msg);
	}
	else
	{
		// start internal timer
		period_=ros::Duration(1.0/config_.rate);
		timer_=node_.createTimer(period_,&Operator::cbAuto,this);
	}
}
