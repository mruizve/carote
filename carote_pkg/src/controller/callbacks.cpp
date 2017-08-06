#include<tf/tf.h>
#include<tf/transform_listener.h>
#include<tf/transform_broadcaster.h>
#include<tf_conversions/tf_kdl.h>
#include "carote/Controller.h"
#include "carote/Utils.h"

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
	static tf::TransformBroadcaster tf_broadcaster;

	// if the target is not with respect frame_id_base_, then lookup the correct transformation
	if( _msg.header.frame_id!=frame_id_base_ )
	{
		// get frames transform (from target to base)
		if( tf_listener.waitForTransform(frame_id_base_,frame_id_target_,_msg.header.stamp,ros::Duration(0.01)) )
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
			ROS_INFO_STREAM("tf not available between '" << frame_id_target_ << "' and '" << frame_id_base_ << "'");
			return;
		}
	}

	// get target frame
	tf::transformTFToKDL(tf_base_target,target_);

	// update target data flag
	target_flag_=1;

	// if the operator have provided new parameters for the goal computation, then
	if( target_flag_ && operator_flag_ )
	{
		// compute the desired position in virtual target coordinates
		KDL::Vector t;
		t(0)=goal_params_.rho*cos(goal_params_.phi*M_PI/180.0)*cos(goal_params_.lambda*M_PI/180.0);
		t(1)=goal_params_.rho*cos(goal_params_.phi*M_PI/180.0)*sin(goal_params_.lambda*M_PI/180.0);
		t(2)=goal_params_.rho*sin(goal_params_.phi*M_PI/180.0);

		// define the virtual target frame {z_target,cross(z_world,z_target),z_world}
		// (we assume that z_target is properly normalized)
		KDL::Vector x(target_.M.UnitZ()); x.Normalize();
		KDL::Vector z(0.0,0.0,1.0);
		KDL::Vector y=z*x; y.Normalize();
		KDL::Rotation R(x,y,z);

		// computed the displacement vector in base coordinates
		t=R*t;

		// compute the goal frame
		z=-t; z.Normalize();
		y=R.UnitZ()*z; y.Normalize();
		x=y*z; z.Normalize();
		R.UnitX(x);
		R.UnitY(y);
		R.UnitZ(z);

		// generate the goal frame in target coordinates
		goal_=KDL::Frame(R,t+target_.p); // base coordinates
		goal_=target_.Inverse()*goal_;   // target coordinates

		// clear/set corresponding flags
		goal_flag_=1;
		operator_flag_=0;
	}

	// if there is a goal defined, then
	if( goal_flag_ )
	{
		// broadcast the goal transformation
		tf::Transform tf_goal;
		tf::transformKDLToTF(goal_,tf_goal);

		// broadcast desired goal pose
		tf_broadcaster.sendTransform(tf::StampedTransform(tf_goal,_msg.header.stamp,frame_id_target_,frame_id_goal_));
	}
}
