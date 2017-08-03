#include<geometry_msgs/Twist.h>
#include "carote/Follower.h"

template<typename T> int sgn(T val)
{
	return (T(0)<val)-(val<T(0));
}

carote::Follower::Follower(const std::string& _name)
:
	Controller(_name)
{
	// ros stuff: dynamic reconfiguration of controller parameters
	dynamic_reconfigure::Server<carote::FollowerConfig>::CallbackType f;
	f=boost::bind(&carote::Follower::cbReconfigure,this,_1,_2);
	server_.setCallback(f);
}

carote::Follower::~Follower(void)
{
/*
	// if some one is listening, then
	if( 0<pub_base_vel_.getNumSubscribers() )
	{
		// stop the robot
		this->stop();
	}
*/
}

void carote::Follower::clean(void)
{
	ROS_WARN_STREAM("clean() not yet implemented!");
}


void carote::Follower::cbControl(const ros::TimerEvent& _event)
{
	ROS_WARN_STREAM("cbControl() not yet implemented!");
	/*
	// get next control command (platform velocity)
	Eigen::Vector3d u;
	if( !u_.empty() )
	{
		u=u_.front();
		u_.clear();
	}
	else
	{
		u=Eigen::Vector3d::Zero();
	}

	// retrieve target state (using Eigen representation)
	Eigen::Map<Eigen::Vector3d> p(&p_[0]);
	p(0)=_msg.poses[0].position.x;
	p(1)=_msg.poses[0].position.y;
	p(2)=_msg.poses[0].position.z;

	Eigen::Map<Eigen::Vector3d> v(&v_[0]);
	v(0)=0.0; //_msg.twist.twist.linear.x;
	v(1)=0.0; //_msg.twist.twist.linear.y;
	v(2)=0.0; //_msg.twist.twist.linear.z;

	// compute distance target distance to the sensor frame
	double d=p.norm();

	// get state respect params_.output_frame
	p_=tf_.getBasis()*p_+tf_.getOrigin();
	v_=tf_.getBasis()*v_;
	v(0)=v(0)+u(0)-u(2)*p(1);
	v(1)=v(1)+u(1)+u(2)*p(0);

	ROS_WARN_STREAM(p.transpose());

	// some algebraic stuff
	Eigen::Vector3d x=Eigen::Vector3d::UnitX();

	// compute linear velocity command
	ROS_WARN_STREAM(d << " -- " << (params_.distance-d));
	u(0)=-params_.Kp*sgn(params_.distance-d)*pow(params_.distance-d,2)-params_.Kv*v.dot(x);
	u(0)=sgn(u(0))*std::min(params_.speed_max,fabs(u(0)));
	
	// compute angular velocity command
	//double w=-params_.Kp*(

	// ROS_WARN_STREAM("p: " << p.transpose());
	// ROS_WARN_STREAM("v: " << v.transpose());
	ROS_WARN_STREAM("u: " << u(0));

	if( params_.enabled )
	{
		// send velocity command
		geometry_msgs::Twist msg_control;
		msg_control.linear.x=u(0);
		msg_control.linear.y=0.0;
		msg_control.linear.z=0.0;
		msg_control.angular.x=0.0;
		msg_control.angular.y=0.0;
		msg_control.angular.z=0.0;
		pub_base_vel_.publish(msg_control);
	}
	else
	{
		// stop the robot
		this->stop();
	}
*/
}

void carote::Follower::cbReconfigure(carote::FollowerConfig& _config, uint32_t _level)
{
	// update parameters
	params_=_config;

	if( params_.enabled )
	{
		ros::Duration period(1.0/params_.rate);
		this->start(period);
		ros::Duration(1.0).sleep();
	}
	else
	{
		this->stop();
		ros::Duration(1.0).sleep();
	}
/*
	// stop the robot (old topic)
	this->stop();

	// update parameters
	params_=config;

	// enable controller
	timer_=node_.createTimer(ros::Rate(params_.rate),&carote::Follower::output,this);
*/
}
