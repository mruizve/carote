#include "carote/Tweak.h"

carote::TweakPublisher::TweakPublisher(const std::string& _name, int _fd)
:
	carote::Tweak(_name)
{
	// ros stuff: prepare for listening to the target topic
	pub_target_=node_.advertise<geometry_msgs::PoseArray>(topic_name_,1);

	// store pipe descriptor
	pipe_rd_fd_=_fd;

	// retrieve and publish transform data
	tf::Vector3 origin;
	tf::Matrix3x3 orientation;
	tf::Quaternion quaternion;
	tf::Transform transform;
	tf::TransformBroadcaster tf_broadcaster;
	while( ros::ok() )
	{
		// read origin
		if( 0>=read(pipe_rd_fd_,&origin[0],4*sizeof(tfScalar)) )
		{
			break;
		}

		// read orientation data
		if( 0>=read(pipe_rd_fd_,&orientation[0],4*sizeof(tfScalar)) )
		{
			break;
		}

		// read orientation data
		if( 0>=read(pipe_rd_fd_,&orientation[1],4*sizeof(tfScalar)) )
		{
			break;
		}

		// read orientation data
		if( 0>=read(pipe_rd_fd_,&orientation[2],4*sizeof(tfScalar)) )
		{
			break;
		}

		// compute orientation quaternion
		orientation.getRotation(quaternion);

		// publish pose message
		geometry_msgs::Pose pose;
		pose.position.x=origin.getX();
		pose.position.y=origin.getY();
		pose.position.z=origin.getZ();
		pose.orientation.x=quaternion.getX();
		pose.orientation.y=quaternion.getY();
		pose.orientation.z=quaternion.getZ();
		pose.orientation.w=quaternion.getW();

		geometry_msgs::PoseArray msg;
		msg.header.stamp=ros::Time::now();
		msg.header.frame_id=frame_id_gripper_;
		msg.poses.push_back(pose);
		pub_target_.publish(msg);

		// generate transformation object
		transform.setOrigin(origin);
		transform.setBasis(orientation);

		// publish transform
		tf_broadcaster.sendTransform(tf::StampedTransform(transform,msg.header.stamp,frame_id_gripper_,frame_id_target_));
	}

	if( ros::ok() )
	{
		ROS_ERROR_STREAM("[" << name_ << "] read() error while tweaking");
	}
}

carote::TweakPublisher::~TweakPublisher(void)
{
}
