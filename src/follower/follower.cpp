#include<Eigen/Core>
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
	// disable controller
	timer_.stop();
	
	// stop the robot
	this->stop();

	// update parameters
	params_=config;

	// input topics names
	params_.input_state=params_.input_target+params_.input_state;

	// prepare for listening to input topics
	input_state_sub_=node_.subscribe(params_.input_state,1,&carote::Follower::input,this);

	// prepare for advertise to the output topics
	output_control_pub_=node_.advertise<geometry_msgs::Twist>(params_.output_control,1);

	// stop the robot
	this->stop();

	// enable controller
	timer_=node_.createTimer(ros::Rate(params_.rate),&carote::Follower::output,this);
}

void carote::Follower::input(const nav_msgs::Odometry& _msg)
{
	#define P _msg.pose.pose.position
	#define V _msg.twist.twist.linear
	#define TS _msg.header.stamp
	
	// retrieve target state
	p_[0]=P.x; p_[1]=P.y; p_[2]=P.z;
	v_[0]=V.x; v_[1]=V.y; v_[2]=V.z;

	// change reference frame (from params_.input_frame to params_.output_frame)
	if( tf_listener_.waitForTransform(params_.output_frame,params_.input_frame,TS,ros::Duration(0.1)) )
	{
		try
		{
			tf_listener_.lookupTransform(params_.output_frame,params_.input_frame,TS,tf_);
		}
		catch( tf::LookupException& ex )
		{
			ROS_INFO_STREAM("no transform available: " << ex.what());
			this->stop();
			return;
		}
		catch( tf::ConnectivityException& ex )
		{
			ROS_INFO_STREAM("connectivity error: " << ex.what());
			this->stop();
			return;
		}
		catch( tf::ExtrapolationException& ex )
		{
			ROS_INFO_STREAM("extrapolation error: " << ex.what());
			this->stop();
			return;
		}
	}
	else
	{
		ROS_INFO_STREAM("transformation not available");
		this->stop();
		return;
	}
	
	p_=tf_.getBasis()*p_+tf_.getOrigin();
	v_=tf_.getBasis()*v_;

	// get absolute target velocity
	v_[0]+=v_[0]+u_[0]-u_[2]*p_[1];
	v_[1]+=v_[1]+u_[1]+u_[2]*p_[0];

	Eigen::Map<Eigen::Vector3d> p(&p_[0]);
	Eigen::Map<Eigen::Vector3d> v(&v_[0]);
//	ROS_WARN_STREAM(p.transpose());
	ROS_WARN_STREAM(v.transpose());


	p << P.x,P.y,P.z;
	v << V.x,V.y,V.z;


	// some algebraic stuff
	Eigen::Vector3d z=Eigen::Vector3d::UnitZ();

	// compute linear velocity command
	double u=-params_.Kp*(params_.distance-p.norm())-params_.Kv*v.dot(z);
	u=sgn(u)*std::min(0.1,fabs(u));
	
	// compute angular velocity command
	//double w=-params_.Kp*(

	// ROS_WARN_STREAM("p: " << p.transpose());
	// ROS_WARN_STREAM("v: " << v.transpose());
	// ROS_WARN_STREAM("u: " << u);

	if( params_.enabled )
	{
		// send velocity command
		geometry_msgs::Twist control_msg;
		control_msg.linear.x=u;
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

	#undef P
	#undef V
	#undef TS
}

void carote::Follower::output(const ros::TimerEvent& event)
{
}
