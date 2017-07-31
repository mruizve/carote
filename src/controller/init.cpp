#include "carote/Controller.h"

void carote::Controller::init(void)
{
	std::string strpar;

	// prepare for advertise to the control topics of the platform
	if( !node_.getParam("/carote/topics/control/base",strpar) )
	{
		// show error and close node
		ROS_ERROR_STREAM("missing parameter /carote/topics/control/base, include setup.launch in your launch file");
		exit(EXIT_FAILURE);
	}
	pub_base_vel_=node_.advertise<geometry_msgs::Twist>(strpar,1);

	// prepare for advertise to the control topics of the arm
	if( !node_.getParam("/carote/topics/control/arm_position",strpar) )
	{
		// show error and close node
		ROS_ERROR_STREAM("missing parameter /carote/topics/control/arm_position, include setup.launch in your launch file");
		exit(EXIT_FAILURE);
	}
	pub_arm_pos_=node_.advertise<brics_actuator::JointPositions>(strpar,1);

	// prepare for advertise to the control topics of the arm
	if( !node_.getParam("/carote/topics/control/arm_velocity",strpar) )
	{
		// show error and close node
		ROS_ERROR_STREAM("missing parameter /carote/topics/control/arm_velocity, include setup.launch in your launch file");
		exit(EXIT_FAILURE);
	}
	pub_arm_vel_=node_.advertise<brics_actuator::JointVelocities>(strpar,1);

	// prepare for listening to the operator topic
	if( !node_.getParam("/carote/topics/operator",strpar) )
	{
		// show error and close node
		ROS_ERROR_STREAM("missing parameter /carote/topics/operator, include setup.launch in your launch file");
		exit(EXIT_FAILURE);
	}
	sub_operator_=node_.subscribe(strpar,1,&carote::Controller::cbOperator,this);

	// prepare for listening to the joint states topic
	if( !node_.getParam("/carote/topics/state",strpar) )
	{
		// show error and close node
		ROS_ERROR_STREAM("missing parameter /carote/topics/state, include setup.launch in your launch file");
		exit(EXIT_FAILURE);
	}
	sub_state_=node_.subscribe(strpar,1,&carote::Controller::cbState,this);

	// prepare for listening to the target topic
	if( !node_.getParam("/carote/topics/target",strpar) )
	{
		// show error and close node
		ROS_ERROR_STREAM("missing parameter /carote/topics/target, include setup.launch in your launch file");
		exit(EXIT_FAILURE);
	}
	sub_target_=node_.subscribe(strpar,1,&carote::Controller::cbTarget,this);

	// get the name of platform or base reference frame
	if( !node_.getParam("/carote/frames/base",frame_id_base_) )
	{
		// show error and close node
		ROS_ERROR_STREAM("missing parameter /carote/frames/base, include setup.launch in your launch file");
		exit(EXIT_FAILURE);
	}

	// get the name of platform or base reference frame
	if( !node_.getParam("/carote/frames/gripper",frame_id_gripper_) )
	{
		// show error and close node
		ROS_ERROR_STREAM("missing parameter /carote/frames/gripper, include setup.launch in your launch file");
		exit(EXIT_FAILURE);
	}

	// get the name of the target reference frame
	if( !node_.getParam("/carote/frames/target",frame_id_target_) )
	{
		// show error and close node
		ROS_ERROR_STREAM("missing parameter /carote/frames/target, include setup.launch in your launch file");
		exit(EXIT_FAILURE);
	}
}
