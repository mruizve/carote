#include "carote/Controller.h"

carote::Controller::Controller(const std::string& _name)
:
	node_("~"),
	name_(_name)
{
	std::string strpar;

	// ros stuff: prepare for advertise to the control topics of the platform
	if( !node_.getParam("/carote/topics/control_base",strpar) )
	{
		// show error and close node
		ROS_ERROR_STREAM("missing parameter /carote/topics/control_base, include setup.launch in your launch file");
		exit(EXIT_FAILURE);
	}
	pub_control_base_=node_.advertise<geometry_msgs::Twist>(strpar,1);

	// ros stuff: prepare for advertise to the control topics of the arm
	if( !node_.getParam("/carote/topics/control_arm",strpar) )
	{
		// show error and close node
		ROS_ERROR_STREAM("missing parameter /carote/topics/control_arm, include setup.launch in your launch file");
		exit(EXIT_FAILURE);
	}
	pub_control_arm_=node_.advertise<brics_actuator::JointPositions>(strpar,1);

	// ros stuff: prepare for listening to the operator topic
	if( !node_.getParam("/carote/topics/operator",strpar) )
	{
		// show error and close node
		ROS_ERROR_STREAM("missing parameter /carote/topics/operator, include setup.launch in your launch file");
		exit(EXIT_FAILURE);
	}
	sub_operator_=node_.subscribe(strpar,1,&carote::Controller::cbOperator,this);

	// ros stuff: prepare for listening to the target topic
	if( !node_.getParam("/carote/topics/target",strpar) )
	{
		// show error and close node
		ROS_ERROR_STREAM("missing parameter /carote/topics/target, include setup.launch in your launch file");
		exit(EXIT_FAILURE);
	}
	sub_target_=node_.subscribe(strpar,1,&carote::Controller::cbTarget,this);

	// ros stuff: get the name of platform or base reference frame
	if( !node_.getParam("/carote/frames/base",frame_id_base_) )
	{
		// show error and close node
		ROS_ERROR_STREAM("missing parameter /carote/frames/base, include setup.launch in your launch file");
		exit(EXIT_FAILURE);
	}

	// ros stuff: get the name of the target reference frame
	if( !node_.getParam("/carote/frames/target",frame_id_target_) )
	{
		// show error and close node
		ROS_ERROR_STREAM("missing parameter /carote/frames/target, include setup.launch in your launch file");
		exit(EXIT_FAILURE);
	}

/*
	// ros stuff: dynamic reconfiguration
	dynamic_reconfigure::Server<carote::ControllerConfig>::CallbackType f;
	f=boost::bind(&carote::Follower::reconfigure,this,_1,_2);
	server_.setCallback(f);
*/
}

carote::Controller::~Controller(void)
{
}

void carote::Controller::cbTarget(const geometry_msgs::PoseArray& _msg)
{
}

void carote::Controller::cbOperator(const geometry_msgs::Vector3& _msg)
{
}

void carote::Controller::cbControl(const ros::TimerEvent& event)
{
}

