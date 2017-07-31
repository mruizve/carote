#include "carote/Tweak.h"

carote::TweakListener::TweakListener(const std::string& _name, int _fd)
:
	carote::Tweak(_name)
{
	// ros stuff: prepare for listening to the target topic
	sub_target_=node_.subscribe(topic_name_,1,&carote::TweakListener::input,this);

	// store pipe descriptor
	pipe_wr_fd_=_fd;
}

void carote::TweakListener::input(const geometry_msgs::PoseArray& _msg)
{
	// get frames transform (from target to gripper)
	if( tf_listener_.waitForTransform(frame_id_gripper_,frame_id_target_,_msg.header.stamp,ros::Duration(0.1)) )
	{
		try
		{
			tf_listener_.lookupTransform(frame_id_gripper_,frame_id_target_,_msg.header.stamp,tf_gripper_target_);
		}
		catch( tf::LookupException& ex )
		{
			ROS_INFO_STREAM("transform not available available: " << ex.what());
			return;
		}
		catch( tf::ConnectivityException& ex )
		{
			ROS_INFO_STREAM("transform connectivity error: " << ex.what());
			return;
		}
		catch( tf::ExtrapolationException& ex )
		{
			ROS_INFO_STREAM("transform extrapolation error: " << ex.what());
			return;
		}
	}
	else
	{
		ROS_INFO_STREAM("transform not available between '" << frame_id_target_ << "' and '" << frame_id_gripper_ << "'");
		return;
	}

	// get transform data
	tf::Vector3& origin=tf_gripper_target_.getOrigin();
	tf::Matrix3x3& orientation=tf_gripper_target_.getBasis();

	// send transform data to the tweak publisher
	write(pipe_wr_fd_,&origin[0],4*sizeof(tfScalar));
	write(pipe_wr_fd_,&orientation[0],4*sizeof(tfScalar));
	write(pipe_wr_fd_,&orientation[1],4*sizeof(tfScalar));
	write(pipe_wr_fd_,&orientation[2],4*sizeof(tfScalar));
}
