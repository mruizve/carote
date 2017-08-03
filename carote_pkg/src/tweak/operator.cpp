#include "carote/Tweak.h"
#include "carote/Utils.h"

carote::TweakOperator::TweakOperator(int &_argc, char **_argv)
:
	Tweak(_argc,_argv),
	node_("~")
{
	if( !node_.getParam("/carote/topics/operator",topic_name_) )
	{
		CAROTE_NODE_ABORT("missing param '/carote/topics/target' (must be defined at setup.launch)");
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
	// send operator data to the tweak publisher
	if( 0>=write(pipe_fd_[1],&_msg.data,sizeof(carote_msgs::Operator)) )
	{
		CAROTE_NODE_ABORT("write() error while tweaking");
	}
}

void carote::TweakOperator::publish(void)
{
	// retrieve and publish operator data
	carote_msgs::OperatorStamped msg;

	while( ros::ok() )
	{
		// read origin
		if( 0>=read(pipe_fd_[0],&msg.data,sizeof(carote_msgs::Operator)) )
		{
			break;
		}

		// publish operator message
		msg.header.stamp=ros::Time::now();
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
	carote::TweakOperator tweak(argc,argv);
}
