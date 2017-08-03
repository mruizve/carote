#include<tf_conversions/tf_eigen.h>
#include "carote/Controller.h"
#include "carote/Utils.h"

void carote::Controller::cbOperator(const carote::OperatorStamped& _msg)
{
	// update operator data
	operator_data_=_msg.data;

	// set flag
	operator_flag_=1;
}

void carote::Controller::cbState(const sensor_msgs::JointState& _msg)
{
	// update joint states
	int updated=0;
    for( int i=0; kdl_chain_.getNrOfSegments()>i; i++ )
	{
		for( size_t j=0; _msg.name.size()>j; j++ )
		{
			if( _msg.name[j]==q_names_[i] )
			{
				q_(i)=_msg.position[j];
				qp_(i)=_msg.velocity[j];
				//tau_(i)=_msg.effort[i]
				updated++;
			}
		}
	}

	// update states flag
	if( kdl_chain_.getNrOfSegments()==updated )
	{
		states_flag_=1;
	}
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
	target_t_(0)=tf_base_target_.getOrigin()[0];
	target_t_(1)=tf_base_target_.getOrigin()[1];
	target_t_(2)=tf_base_target_.getOrigin()[2];

	// get target pose
	tf::Matrix3x3 R=tf_base_target_.getBasis();
	tf::matrixTFToEigen(R,target_R_);

	// update target data flag
	target_flag_=1;
}
