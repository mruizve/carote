#include<geometry_msgs/Twist.h>
#include "carote/Follower.h"

template<typename T> int sgn(T val)
{
	return (T(0)<val)-(val<T(0));
}

carote::Follower::Follower(const std::string& _name)
:
	node_("~"),
	name_(_name)
{
	// ros stuff: dynamic reconfiguration
	dynamic_reconfigure::Server<carote::FollowerConfig>::CallbackType f;
	f=boost::bind(&carote::Follower::reconfigure,this,_1,_2);
	server_.setCallback(f);
}

carote::Follower::~Follower(void)
{
	// if some one is listening, then
	if( 0<output_control_pub_.getNumSubscribers() )
	{
		// stop the robot
		this->stop();
	}
}

void carote::Follower::stop(void)
{
	// stop the timer
	timer_.stop();
	
	// stop the robot
	geometry_msgs::Twist control_msg;
	control_msg.linear.x=0.0;
	control_msg.linear.y=0.0;
	control_msg.linear.z=0.0;
	control_msg.angular.x=0.0;
	control_msg.angular.y=0.0;
	control_msg.angular.z=0.0;
	if( 0<output_control_pub_.getNumSubscribers() )
	{
		output_control_pub_.publish(control_msg);
	}
}

void carote::Follower::reconfigure(carote::FollowerConfig& config, uint32_t level)
{
	// stop the robot (old topic)
	this->stop();

	// update parameters
	params_=config;

	// input topics names
	params_.input_state=params_.input_target+params_.input_state;

	// prepare for listening to input topics
	input_state_sub_=node_.subscribe(params_.input_state,1,&carote::Follower::input,this);

	// prepare for advertise to the output topics
	output_control_pub_=node_.advertise<geometry_msgs::Twist>(params_.output_control,1);

	// stop the robot (new topic)
	this->stop();

	// enable controller
	timer_=node_.createTimer(ros::Rate(params_.rate),&carote::Follower::output,this);
}

void carote::Follower::input(const nav_msgs::Odometry& _msg)
{
	// get frames transform (from params_.input_frame to params_.output_frame)
	#define iframe_ params_.input_frame
	#define oframe_ params_.output_frame
	#define _tstamp _msg.header.stamp
	if( tf_listener_.waitForTransform(oframe_,iframe_,_tstamp,ros::Duration(0.1)) )
	{
		try
		{
			tf_listener_.lookupTransform(oframe_,iframe_,_tstamp,tf_);
		}
		catch( tf::LookupException& ex )
		{
			ROS_INFO_STREAM("no transform available: " << ex.what());
			return;
		}
		catch( tf::ConnectivityException& ex )
		{
			ROS_INFO_STREAM("connectivity error: " << ex.what());
			return;
		}
		catch( tf::ExtrapolationException& ex )
		{
			ROS_INFO_STREAM("extrapolation error: " << ex.what());
			return;
		}
	}
	else
	{
		ROS_INFO_STREAM("transformation not available");
		return;
	}
	#undef iframe_
	#undef oframe_
	#undef _tstamp

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
	p(0)=_msg.pose.pose.position.x;
	p(1)=_msg.pose.pose.position.y;
	p(2)=_msg.pose.pose.position.z;

	Eigen::Map<Eigen::Vector3d> v(&v_[0]);
	v(0)=_msg.twist.twist.linear.x;
	v(1)=_msg.twist.twist.linear.y;
	v(2)=_msg.twist.twist.linear.z;

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
		geometry_msgs::Twist control_msg;
		control_msg.linear.x=u(0);
		control_msg.linear.y=0.0;
		control_msg.linear.z=0.0;
		control_msg.angular.x=0.0;
		control_msg.angular.y=0.0;
		control_msg.angular.z=0.0;
		output_control_pub_.publish(control_msg);
	}
	else
	{
		// stop the robot
		this->stop();
	}
}

void carote::Follower::output(const ros::TimerEvent& event)
{
}
