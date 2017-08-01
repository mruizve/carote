#include "carote/Controller.h"

void carote::Controller::initROS(void)
{
	std::string strpar;

	// prepare for advertise to the control topics of the platform
	if( !node_.getParam("/carote/topics/control/base",strpar) )
	{
		CAROTE_NODE_ABORT("missing param '/carote/topics/control/base' (must be defined at setup.launch");
	}
	pub_base_vel_=node_.advertise<geometry_msgs::Twist>(strpar,1);

	// prepare for advertise to the control topics of the arm
	if( !node_.getParam("/carote/topics/control/arm_position",strpar) )
	{
		CAROTE_NODE_ABORT("missing param '/carote/topics/control/arm_position' (must be defined at setup.launch");
	}
	pub_arm_pos_=node_.advertise<brics_actuator::JointPositions>(strpar,1);

	// prepare for advertise to the control topics of the arm
	if( !node_.getParam("/carote/topics/control/arm_velocity",strpar) )
	{
		CAROTE_NODE_ABORT("missing param '/carote/topics/control/arm_velocity' (must be defined at setup.launch");
	}
	pub_arm_vel_=node_.advertise<brics_actuator::JointVelocities>(strpar,1);

	// prepare for listening to the operator topic
	if( !node_.getParam("/carote/topics/operator",strpar) )
	{
		CAROTE_NODE_ABORT("missing param '/carote/topics/operator' (must be defined at setup.launch");
	}
	sub_operator_=node_.subscribe(strpar,1,&carote::Controller::cbOperator,this);

	// prepare for listening to the joint states topic
	if( !node_.getParam("/carote/topics/state",strpar) )
	{
		CAROTE_NODE_ABORT("missing param '/carote/topics/state' (must be defined at setup.launch");
	}
	sub_state_=node_.subscribe(strpar,1,&carote::Controller::cbState,this);

	// prepare for listening to the target topic
	if( !node_.getParam("/carote/topics/target",strpar) )
	{
		CAROTE_NODE_ABORT("missing param '/carote/topics/target' (must be defined at setup.launch");
	}
	sub_target_=node_.subscribe(strpar,1,&carote::Controller::cbTarget,this);

	// get the name of platform or base reference frame
	if( !node_.getParam("/carote/frames/base",frame_id_base_) )
	{
		CAROTE_NODE_ABORT("missing param '/carote/frames/base' (must be defined at setup.launch");
	}

	// get the name of platform or base reference frame
	if( !node_.getParam("/carote/frames/gripper",frame_id_gripper_) )
	{
		CAROTE_NODE_ABORT("missing param '/carote/frames/gripper' (must be defined at setup.launch");
	}

	// get the name of the target reference frame
	if( !node_.getParam("/carote/frames/target",frame_id_target_) )
	{
		CAROTE_NODE_ABORT("missing param '/carote/frames/target' (must be defined at setup.launch");
	}
}
