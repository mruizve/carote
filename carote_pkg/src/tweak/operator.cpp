#include<tf_conversions/tf_eigen.h>
#include<tf/transform_broadcaster.h>
#include "tweak.hpp"

// instantiate tweak template
template carote::Tweak<carote_msgs::OperatorStamped>::Tweak(int&,char**);
template carote::Tweak<carote_msgs::OperatorStamped>::~Tweak(void);

carote::TweakOperator::TweakOperator(int &_argc, char **_argv)
:
	Tweak(_argc,_argv),
	node_("~")
{
	if( !node_.getParam("/carote/topics/operator",topic_name_) )
	{
		CAROTE_NODE_ABORT("missing param '/carote/topics/target' (must be defined at setup.launch)");
	}

	// get frames id of the goal and target
	if( !node_.getParam("/carote/frames/goal",frame_id_goal_) )
	{
		CAROTE_NODE_ABORT("missing param '/carote/frames/goal' (must be defined at setup.launch)");
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
		publisher_=node_.advertise<carote_msgs::OperatorStamped>(topic_name_,1);

		// start publishing (should be blocking function)
		this->publish();
	}
	else
	{
		// parent process: tweak listener
		// ------------------------------

		// ros stuff: prepare for listening to the target topic
		subscriber_=node_.subscribe(topic_name_,1,&carote::TweakOperator::listen,this);

		// handle events
		ros::spin();
	}
}

carote::TweakOperator::~TweakOperator(void)
{
}

void carote::TweakOperator::listen(const carote_msgs::OperatorStamped& _msg)
{
	// get transform (from target to tip)
	while( !tf_listener_.waitForTransform(frame_id_target_,frame_id_goal_,_msg.header.stamp,ros::Duration(0.1)) )
	{
		ROS_WARN_STREAM("waiting for transform between target and operator defined goal ");
	}
	try
	{
		tf_listener_.lookupTransform(frame_id_target_,frame_id_goal_,_msg.header.stamp,tf_target_goal_);
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

	// get transform data
	tf::Vector3& origin=tf_target_goal_.getOrigin();
	tf::Matrix3x3& orientation=tf_target_goal_.getBasis();

	// send operator data to the tweak publisher
	if( 0>=write(pipe_fd_[1],&_msg.data,sizeof(carote_msgs::Operator)) )
	{
		CAROTE_NODE_ABORT("write() error while tweaking");
	}

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

void carote::TweakOperator::publish(void)
{
	// retrieve and publish operator parameters and goal transform data
	tf::Vector3 origin;
	tf::Matrix3x3 orientation;
	tf::Transform transform;
	tf::TransformBroadcaster tf_broadcaster;
	carote_msgs::OperatorStamped msg;

	while( ros::ok() )
	{
		// read origin
		if( 0>=read(pipe_fd_[0],&msg.data,sizeof(carote_msgs::Operator)) )
		{
			break;
		}

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

		// publish operator parameters
		msg.header.stamp=ros::Time::now();
		publisher_.publish(msg);

		// generate transformation object
		transform.setOrigin(origin);
		transform.setBasis(orientation);

		// publish goal transform with respect target
		tf_broadcaster.sendTransform(tf::StampedTransform(transform,msg.header.stamp,frame_id_target_,frame_id_goal_));
	}

	// issues while receiving data through the pipe?
	if( ros::ok() )
	{
		CAROTE_NODE_ABORT("read() error while tweaking");
	}
}

int main(int argc, char **argv)
{
	carote::TweakOperator tweak(argc,argv);
}
