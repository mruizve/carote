#include "carote/Controller.h"
#include "carote/Utils.h"

carote::Controller::Controller(const std::string& _name)
:
	node_("~"),
	name_(_name),
	model_(NULL),
	operator_flag_(0),
	states_flag_(0),
	target_flag_(0)
{
	// ros initialization
	this->initROS();

	// get the robot urdf model
	std::string xmlpar;
	if( !node_.getParam("/robot_description",xmlpar) )
	{
		CAROTE_NODE_ABORT("missing param '/robot_description' (launch robot drivers before controller)");
	}

	// create model
	model_=new carote::Model(xmlpar,frame_id_base_.substr(1),frame_id_tip_.substr(1));

	// resize joint states arrays
	q_.resize(model_->getNrOfJoints());
	qp_.resize(model_->getNrOfJoints());

	// poses initialization
	this->initPoses();
}

carote::Controller::~Controller(void)
{
	if( NULL!=model_ )
	{
		delete model_;
	}
}

void carote::Controller::home(void)
{
	// stop the robot
	this->stop();

	// move robot to the home position
	this->armPose(q_home_);

	// wait some time
	ros::Duration(0.5).sleep();
}

void carote::Controller::start(ros::Duration _period)
{
	// stop the robot
	this->stop();

	// start internal timer
	timer_=node_.createTimer(_period,&Controller::cbControl,this);
}

void carote::Controller::stop(void)
{
	// stop the timer
	timer_.stop();
	
	// stop the robot motion
	this->zero();
}

void carote::Controller::work(void)
{
	// stop the robot
	this->stop();

	// move robot to the work position
	this->armPose(q_work_);

	// wait some time
	ros::Duration(0.5).sleep();
}

void carote::Controller::zero(void)
{
	// delete any residual control action
	// (this is a pure virtual function depends on the implementation)
	this->clean();

	// stop the robot platform or base
	geometry_msgs::Twist msg_base;
	msg_base.linear.x=0.0;
	msg_base.linear.y=0.0;
	msg_base.linear.z=0.0;
	msg_base.angular.x=0.0;
	msg_base.angular.y=0.0;
	msg_base.angular.z=0.0;
	pub_base_vel_.publish(msg_base);	
	
	// stop the arm
	ros::Time stamp=ros::Time::now();
	brics_actuator::JointVelocities msg_arm;
	msg_arm.poisonStamp.originator=name_;

	// for each joint of the chain
	const std::vector<std::string>& q_names=model_->getJointsNames();
	const std::vector<std::string>& qp_units=model_->getSpeedsUnits();
    for( int i=0; model_->getNrOfJoints()>i; i++ )
	{
		// prepare message data
		brics_actuator::JointValue velocity;
		velocity.timeStamp=stamp;
		velocity.joint_uri=q_names[i];
		velocity.unit=qp_units[i];
		velocity.value=0.0;

		// add data to the message
		msg_arm.velocities.push_back(velocity);
	}

	// publish message
	pub_arm_vel_.publish(msg_arm);

	// wait some time
	ros::Duration(0.5).sleep();
}
