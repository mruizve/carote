#include "carote/Controller.h"

void carote::Controller::armPose(const KDL::JntArray& _q)
{
	ros::Time tstamp=ros::Time::now();

	// initialize message
	brics_actuator::JointPositions msg_q;

	// for each joint of the chain
	brics_actuator::JointValue joint;
	const std::vector<std::string> &q_names=model_->getJointsNames();
	const std::vector<std::string> &q_units=model_->getJointsUnits();
	for( int i=0; model_->getNrOfJoints()>i; i++ )
	{
		// prepare message data
		joint.timeStamp=tstamp;
		joint.joint_uri=q_names[i];
		joint.unit=q_units[i];
		joint.value=_q(i);

		// add data to the message
		msg_q.positions.push_back(joint);
	}

	// publish message
	pub_q_.publish(msg_q);
}

void carote::Controller::armSpeed(const KDL::JntArray& _qp)
{
	ros::Time tstamp=ros::Time::now();

	// initialize message
	brics_actuator::JointVelocities msg_qp;

	// for each joint of the chain
	brics_actuator::JointValue velocity;
	const std::vector<std::string> &q_names=model_->getJointsNames();
	const std::vector<std::string> &qp_units=model_->getSpeedsUnits();
	for( int i=0; model_->getNrOfJoints()>i; i++ )
	{
		// prepare message data
		velocity.timeStamp=tstamp;
		velocity.joint_uri=q_names[i];
		velocity.unit=qp_units[i];
		velocity.value=_qp(i);

		// add data to the message
		msg_qp.velocities.push_back(velocity);
	}

	// publish message
	pub_qp_.publish(msg_qp);
}

void carote::Controller::baseTwist(const KDL::Twist& _u)
{
	// initialize message
	geometry_msgs::Twist msg_u;
	msg_u.linear.x=_u(0);
	msg_u.linear.y=_u(1);
	msg_u.linear.z=_u(2);
	msg_u.angular.x=_u(3);
	msg_u.angular.y=_u(4);
	msg_u.angular.z=_u(5);

	// publish message
	pub_u_.publish(msg_u);
}
