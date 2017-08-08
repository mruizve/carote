#include<tf/tf.h>
#include<tf/transform_listener.h>
#include<tf/transform_broadcaster.h>
#include<tf_conversions/tf_kdl.h>
#include "carote/Controller.h"
#include "carote/Utils.h"

void carote::Controller::cbControl(const ros::TimerEvent& _event)
{
	ROS_WARN_STREAM("carote::Controller::cbControl() must NEVER be called");
}

void carote::Controller::cbOperator(const carote_msgs::OperatorStamped& _msg)
{
	// copy goal parameters
	goal_params_=_msg.data;

	// set goal parameters flag
	operator_flag_=1;
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
				tau_(i)=_msg.effort[i];
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
	tf::StampedTransform tf_base_target;
	static tf::TransformListener tf_listener;
	static tf::TransformBroadcaster tf_broadcaster;

	// get the last target transform with respect base
	try
	{
		tf_listener.lookupTransform(frame_id_base_,frame_id_target_,ros::Time(0),tf_base_target);
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

	// convert get target transform to KDL frame
	tf::transformTFToKDL(tf_base_target,target_);

	// update target data flag
	target_flag_=1;

	// if the operator have provided new parameters for the goal computation, then
	if( operator_flag_ )
	{
		// compute goal and clear goal parameters flags
		this->getGoal();
	}

	if( goal_flag_ )
	{
		// broadcast desired goal pose at the same rate of the target pose
		tf::Transform tf_goal;
		tf::transformKDLToTF(goal_,tf_goal);
		tf_broadcaster.sendTransform(tf::StampedTransform(tf_goal,_msg.header.stamp,frame_id_target_,frame_id_goal_));
	}
}
