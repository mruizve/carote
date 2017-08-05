#include<tf/tf.h>
#include<tf/transform_listener.h>
#include<tf_conversions/tf_kdl.h>
#include "carote/Controller.h"
#include "carote/Utils.h"

void carote::Controller::cbOperator(const carote_msgs::OperatorStamped& _msg)
{
	static tf::StampedTransform tf_base_goal;
	static tf::TransformListener tf_listener;

	// if the target is not with respect frame_id_base_, then lookup the correct transformation
	if( _msg.header.frame_id!=frame_id_base_ )
	{
		// get frames transform (from goal to base)
		if( tf_listener.waitForTransform(frame_id_base_,frame_id_goal_,_msg.header.stamp,ros::Duration(0.5)) )
		{
			try
			{
				tf_listener.lookupTransform(frame_id_base_,frame_id_goal_,_msg.header.stamp,tf_base_goal);
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
			ROS_INFO_STREAM("tf not available between '" << frame_id_goal_ << "' and '" << frame_id_base_ << "'");
			return;
		}
	}

	// get target frame
	tf::transformTFToKDL(tf_base_goal,goal_);

	// set flag
	goal_flag_=1;
}

void carote::Controller::cbState(const sensor_msgs::JointState& _msg)
{
	// update joint states
	int updated=0;
    for( int i=0; model_->getNrOfJoints()>i; i++ )
	{
		for( size_t j=0; _msg.name.size()>j; j++ )
		{
			if( _msg.name[j]==model_->getJointName(i) )
			{
				q_(i)=_msg.position[j];
				qp_(i)=_msg.velocity[j];
				//tau_(i)=_msg.effort[i]
				updated++;
			}
		}
	}

	// update states flag
	if( model_->getNrOfJoints()==updated )
	{
		states_flag_=1;
	}
}

void carote::Controller::cbTarget(const geometry_msgs::PoseArray& _msg)
{
	static tf::StampedTransform tf_base_target;
	static tf::TransformListener tf_listener;

	// if the target is not with respect frame_id_base_, then lookup the correct transformation
	if( _msg.header.frame_id!=frame_id_base_ )
	{
		// get frames transform (from target to base)
		if( tf_listener.waitForTransform(frame_id_base_,frame_id_target_,_msg.header.stamp,ros::Duration(0.05)) )
		{
			try
			{
				tf_listener.lookupTransform(frame_id_base_,frame_id_target_,_msg.header.stamp,tf_base_target);
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
			//ROS_INFO_STREAM("tf not available between '" << frame_id_target_ << "' and '" << frame_id_base_ << "'");
			return;
		}
	}

	// get target frame
	tf::transformTFToKDL(tf_base_target,target_);

	// update target data flag
	target_flag_=1;
}
