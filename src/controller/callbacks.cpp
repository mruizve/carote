#include<tf_conversions/tf_eigen.h>
#include "carote/Controller.h"

void carote::Controller::cbOperator(const geometry_msgs::Vector3& _msg)
{
	// get operator command
	phi_=_msg.x;
	lambda_=_msg.y;
	rho_=_msg.z;

	// update operator flag
	flag_operator_=1;
}

void carote::Controller::cbState(const sensor_msgs::JointState& _msg)
{
	// update state

	// new data available?
	if( flag_operator_ || flag_target_ )
	{
		// clear flags
		flag_operator_=0;
		flag_target_=0;

		// update control law
		this->update();
	}

// TODO >>>
	// get control command
	VelocityControlData control;
	if( control_queue_.empty() )
	{
		control.u=Eigen::Vector3d::Zero();
		control.qp=Eigen::Matrix<double,5,1>::Zero();
	}
	else
	{
		control=control_queue_.front();
	}

	// publish control command
	brics_actuator::JointVelocities msg_arm;
	geometry_msgs::Twist msg_base;
// >>> TODO
}

void carote::Controller::cbTarget(const geometry_msgs::PoseArray& _msg)
{
	// if the target is not with respect frame_id_base_, then lookup the correct transformation
	if( _msg.header.frame_id!=frame_id_base_ )
	{
		// get frames transform (from target to base)
		if( tf_listener_.waitForTransform(frame_id_base_,frame_id_target_,_msg.header.stamp,ros::Duration(0.1)) )
		{
			try
			{
				tf_listener_.lookupTransform(frame_id_base_,frame_id_target_,_msg.header.stamp,tf_base_target_);
			}
			catch( tf::LookupException& ex )
			{
				ROS_INFO_STREAM("tf not available: " << ex.what());
				return;
			}
			catch( tf::ConnectivityException& ex )
			{
				ROS_INFO_STREAM("tf connectivity error: " << ex.what());
				return;
			}
			catch( tf::ExtrapolationException& ex )
			{
				ROS_INFO_STREAM("tf extrapolation error: " << ex.what());
				return;
			}
		}
		else
		{
			ROS_INFO_STREAM("tf not available between '" << frame_id_target_ << "' and '" << frame_id_base_ << "'");
			return;
		}
	}

	// get target position
	t_(0)=tf_base_target_.getOrigin()[0];
	t_(1)=tf_base_target_.getOrigin()[1];
	t_(2)=tf_base_target_.getOrigin()[2];

	// get target pose
	tf::Matrix3x3 R=tf_base_target_.getBasis();
	tf::matrixTFToEigen(R,R_);

	// update target data flag
	flag_target_=1;
}

void carote::Controller::cbControl(const ros::TimerEvent& event)
{
}
