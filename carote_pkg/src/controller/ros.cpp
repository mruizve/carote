#include<brics_actuator/JointPositions.h>  // robot control (arm)
#include<brics_actuator/JointVelocities.h> // robot control (arm)
#include<geometry_msgs/Twist.h>            // robot control (platform or base)
#include "carote/Controller.h"
#include "carote/Utils.h"

const ros::NodeHandle& carote::Controller::node(void) const
{
	return node_;
}

void carote::Controller::initROS(void)
{
	std::string strpar;

	// prepare for advertise to the control topics of the platform
	if( !node_.getParam("/carote/topics/control/base",strpar) )
	{
		CAROTE_NODE_ABORT("missing param '/carote/topics/control/base' (must be defined at setup.launch)");
	}
	pub_u_=node_.advertise<geometry_msgs::Twist>(strpar,1);

	// prepare for advertise to the control topics of the arm
	if( !node_.getParam("/carote/topics/control/arm_position",strpar) )
	{
		CAROTE_NODE_ABORT("missing param '/carote/topics/control/arm_position' (must be defined at setup.launch)");
	}
	pub_q_=node_.advertise<brics_actuator::JointPositions>(strpar,1);

	// prepare for advertise to the control topics of the arm
	if( !node_.getParam("/carote/topics/control/arm_velocity",strpar) )
	{
		CAROTE_NODE_ABORT("missing param '/carote/topics/control/arm_velocity' (must be defined at setup.launch)");
	}
	pub_qp_=node_.advertise<brics_actuator::JointVelocities>(strpar,1);

	// prepare for listening to the operator topic
	if( !node_.getParam("/carote/topics/operator",strpar) )
	{
		CAROTE_NODE_ABORT("missing param '/carote/topics/operator' (must be defined at setup.launch)");
	}
	sub_operator_=node_.subscribe(strpar,1,&carote::Controller::cbOperator,this);

	// prepare for listening to the joint states topic
	if( !node_.getParam("/carote/topics/state",strpar) )
	{
		CAROTE_NODE_ABORT("missing param '/carote/topics/state' (must be defined at setup.launch)");
	}
	sub_state_=node_.subscribe(strpar,1,&carote::Controller::cbState,this);

	// prepare for listening to the target topic
	if( !node_.getParam("/carote/topics/target",strpar) )
	{
		CAROTE_NODE_ABORT("missing param '/carote/topics/target' (must be defined at setup.launch)");
	}
	sub_target_=node_.subscribe(strpar,1,&carote::Controller::cbTarget,this);

	// get the name of platform or base reference frame
	if( !node_.getParam("/carote/frames/base",frame_id_base_) )
	{
		CAROTE_NODE_ABORT("missing param '/carote/frames/base' (must be defined at setup.launch)");
	}

	// get the name of operator goal reference frame
	if( !node_.getParam("/carote/frames/goal",frame_id_goal_) )
	{
		CAROTE_NODE_ABORT("missing param '/carote/frames/goal' (must be defined at setup.launch)");
	}

	// get the name of the target reference frame
	if( !node_.getParam("/carote/frames/target",frame_id_target_) )
	{
		CAROTE_NODE_ABORT("missing param '/carote/frames/target' (must be defined at setup.launch)");
	}

	// get the name of tip reference frame
	if( !node_.getParam("/carote/frames/tip",frame_id_tip_) )
	{
		CAROTE_NODE_ABORT("missing param '/carote/frames/tip' (must be defined at setup.launch)");
	}
}
