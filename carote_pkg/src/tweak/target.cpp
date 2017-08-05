#include<tf/transform_broadcaster.h>
#include "tweak.hpp"

// instantiate tweak template
template carote::Tweak<geometry_msgs::PoseArray>::Tweak(int&,char**);
template carote::Tweak<geometry_msgs::PoseArray>::~Tweak(void);

carote::TweakTarget::TweakTarget(int &_argc, char **_argv)
:
	Tweak(_argc,_argv),
	node_("~")
{
	if( !node_.getParam("/carote/topics/target",topic_name_) )
	{
		CAROTE_NODE_ABORT("missing param '/carote/topics/target' (must be defined at setup.launch)");
	}

	// get static transform between gripper and sensor
	if( !node_.getParam("/carote/frames/gripper",frame_id_gripper_) )
	{
		CAROTE_NODE_ABORT("missing param '/carote/frames/gripper' (must be defined at setup.launch)");
	}

	if( !node_.getParam("/carote/frames/target",frame_id_target_) )
	{
		CAROTE_NODE_ABORT("missing param '/carote/frames/target' (must be defined at setup.launch");
	}

	if( 0==pid_ )
	{
		// child process: tweak publisher
		// ------------------------------

		// ros stuff: prepare for listening to the target topic
		publisher_=node_.advertise<geometry_msgs::PoseArray>(topic_name_,1);

		// start publishing (should be blocking function)
		this->publish();
	}
	else
	{
		// parent process: tweak listener
		// ------------------------------

		// ros stuff: prepare for listening to the target topic
		subscriber_=node_.subscribe(topic_name_,1,&carote::TweakTarget::listen,this);

		// handle events
		ros::spin();
	}
}

carote::TweakTarget::~TweakTarget(void)
{
}

void carote::TweakTarget::listen(const geometry_msgs::PoseArray& _msg)
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
		ROS_INFO_STREAM("tf not available between '" << frame_id_target_ << "' and '" << frame_id_gripper_ << "'");
		return;
	}

	// get transform data
	tf::Vector3& origin=tf_gripper_target_.getOrigin();
	tf::Matrix3x3& orientation=tf_gripper_target_.getBasis();

	// send transform data to the tweak publisher
	if( 0>=write(pipe_fd_[1],&origin[0],4*sizeof(tfScalar)) )
	{
		CAROTE_NODE_ABORT("write() error while tweaking");
	}

	if( 0>=write(pipe_fd_[1],&orientation[0],4*sizeof(tfScalar)) )
	{
		CAROTE_NODE_ABORT("write() error while tweaking");
	}

	if( 0>=write(pipe_fd_[1],&orientation[1],4*sizeof(tfScalar)) )
	{
		CAROTE_NODE_ABORT("write() error while tweaking");
	}

	if( 0>=write(pipe_fd_[1],&orientation[2],4*sizeof(tfScalar)) )
	{
		CAROTE_NODE_ABORT("write() error while tweaking");
	}
}

void carote::TweakTarget::publish(void)
{
	// retrieve and publish transform data
	tf::Vector3 origin;
	tf::Matrix3x3 orientation;
	tf::Quaternion quaternion;
	tf::Transform transform;
	tf::TransformBroadcaster tf_broadcaster;
	while( ros::ok() )
	{
		// read origin
		if( 0>=read(pipe_fd_[0],&origin[0],4*sizeof(tfScalar)) )
		{
			break;
		}

		// read orientation data
		if( 0>=read(pipe_fd_[0],&orientation[0],4*sizeof(tfScalar)) )
		{
			break;
		}

		// read orientation data
		if( 0>=read(pipe_fd_[0],&orientation[1],4*sizeof(tfScalar)) )
		{
			break;
		}

		// read orientation data
		if( 0>=read(pipe_fd_[0],&orientation[2],4*sizeof(tfScalar)) )
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
		publisher_.publish(msg);

		// generate transformation object
		transform.setOrigin(origin);
		transform.setBasis(orientation);

		// publish transform
		tf_broadcaster.sendTransform(tf::StampedTransform(transform,msg.header.stamp,frame_id_gripper_,frame_id_target_));
	}

	// issues while receiving data through the pipe?
	if( ros::ok() )
	{
		CAROTE_NODE_ABORT("read() error while tweaking");
	}
}

int main(int argc, char **argv)
{
	carote::TweakTarget tweak(argc,argv);
}
