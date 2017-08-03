#include "carote/Controller.h"

carote::Controller::Controller(const std::string& _name)
:
	node_("~"),
	name_(_name),
	operator_flag_(0),
	states_flag_(0),
	target_flag_(0)
{
	// ros initialization
	this->initROS();

	// udrf/kld initializations
	this->initKinematics();

	// poses initialization
	this->initPoses();

	// show status to the user
    for( int i=0; kdl_chain_.getNrOfSegments()>i; i++ )
    {
		if( urdf::Joint::FIXED!=q_types_[i] )
		{
			ROS_WARN_STREAM("[arm chain] "
				<< q_names_[i]
				<< ", range=[" << q_lower_(i) << "," << q_upper_(i)
				<< "], poses={ home=" << q_home_(i)
				<< ", work=" << q_work_(i)
				<< " }");
		}
		else
		{
			ROS_WARN_STREAM("[arm chain] " << q_names_[i] << ", fixed");
		}
	}
}

carote::Controller::~Controller(void)
{
}

void carote::Controller::home(void)
{
	// stop the robot
	this->stop();

	// move robot to the home position
	this->moveTo(q_home_);
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
	this->moveTo(q_work_);
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
    for( int i=0; kdl_chain_.getNrOfSegments()>i; i++ )
	{
		// that is not fixed,
		if( urdf::Joint::FIXED==q_types_[i] )
		{
			continue;
		}

		// prepare message data
		brics_actuator::JointValue velocity;
		velocity.timeStamp=stamp;
		velocity.joint_uri=q_names_[i];
		if( urdf::Joint::PRISMATIC==q_types_[i] )
		{
			velocity.unit="s^-1 meters";
		}
		else
		{
			velocity.unit="s^-1 rad";
		}
		velocity.value=0.0;

		// add data to the message
		msg_arm.velocities.push_back(velocity);
	}

	// publish message
	pub_arm_vel_.publish(msg_arm);
}
