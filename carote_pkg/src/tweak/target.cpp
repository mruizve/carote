#include<tf/tf.h>
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

	// get frame id of the sensor and target
	if( !node_.getParam("/carote/frames/sensor",frame_id_sensor_) )
	{
		CAROTE_NODE_ABORT("missing param '/carote/frames/sensor' (must be defined at setup.launch");
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
	// send transform data to the tweak publisher
	if( _msg.poses.size() )
	{
		if( 0>=write(pipe_fd_[1],&_msg.poses[0],sizeof(geometry_msgs::Pose)) )
		{
			CAROTE_NODE_ABORT("write() error while tweaking");
		}
	}
}

void carote::TweakTarget::publish(void)
{
	geometry_msgs::Pose pose;
	tf::TransformBroadcaster tf_broadcaster;

	// retrieve and publish transform data
	while( ros::ok() )
	{
		// read pose
		if( 0>=read(pipe_fd_[0],&pose,sizeof(geometry_msgs::Pose)) )
		{
			break;
		}

		// prepare pose message
		geometry_msgs::PoseArray msg;
		msg.header.stamp=ros::Time::now();
		msg.header.frame_id=frame_id_sensor_;
		msg.poses.push_back(pose);

		// compute target transform
		tf::Vector3 position(pose.position.x,pose.position.y,pose.position.z);
		tf::Quaternion quaternion(pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w);
		tf::Transform tf_sensor_target(quaternion,position);

		// publish target transform with respect sensor
		tf_broadcaster.sendTransform(tf::StampedTransform(tf_sensor_target,msg.header.stamp,msg.header.frame_id,frame_id_target_));

		// publish pose message
		publisher_.publish(msg);
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
