//#include<arpa/inet.h>
//#include<sys/socket.h>
//#include<boost/date_time/posix_time/posix_time.hpp>
#include "carote/Tweak.h"

carote::Tweak::Tweak(const std::string& _name):
	node_("~"),
	name_(_name)
{
	// ros stuff: load parameters
	if( !node_.getParam("/carote/topics/tweak",topic_name_) )
	{
		// show error and close node
		ROS_ERROR_STREAM("missing parameter /carote/topics/tweak, include setup.launch in your launch file");
		exit(EXIT_FAILURE);
	}

	// get static transform between gripper and sensor
	if( !node_.getParam("/carote/frames/gripper",frame_gripper_) )
	{
		// show error and close node
		ROS_ERROR_STREAM("missing parameter /carote/frames/gripper, include setup.launch in your launch file");
		exit(EXIT_FAILURE);
	}

	if( !node_.getParam("/carote/frames/target",frame_target_) )
	{
		// show error and close node
		ROS_ERROR_STREAM("missing parameter /carote/frames/target, include setup.launch in your launch file");
		exit(EXIT_FAILURE);
	}
}

carote::Tweak::~Tweak(void)
{
}

/*
carote::Tweak::Tweak(const std::string& _name):
	fd_(-1),
	sd_(-1),
	node_(_name)
{
	// ros stuff: dynamic reconfiguration
	dynamic_reconfigure::Server<carote::TweakConfig>::CallbackType f;
	f=boost::bind(&carote::Tweak::reconfigure,this,_1,_2);
	server_.setCallback(f);
}

carote::Tweak::~Tweak(void)
{
	this->shutdown();
}

void carote::Tweak::input(const nav_msgs::Odometry& _msg)
{
	// retrieve state
	float state[6];
	state[0]=_msg.pose.pose.position.x;
	state[1]=_msg.pose.pose.position.y;
	state[2]=_msg.pose.pose.position.z;
	state[3]=_msg.twist.twist.linear.x;
	state[4]=_msg.twist.twist.linear.y;
	state[5]=_msg.twist.twist.linear.z;

	if( 0>send(fd_,(void*)&state[0],sizeof(state),MSG_DONTWAIT) )
	{
		if( EWOULDBLOCK!=errno && EAGAIN!=errno )
		{
			ROS_ERROR_STREAM("tweak failure (if persistent, restart the node)");
		}
	}
}

void carote::Tweak::output(const ros::TimerEvent& event)
{
	float state[6];

	int err=recv(sd_,(void*)&state[0],sizeof(state),0);

	if( sizeof(state)==err )
	{
		nav_msgs::Odometry msg;
		msg.header.stamp=ros::Time::now();
		msg.pose.pose.position.x=state[0];
		msg.pose.pose.position.y=state[1];
		msg.pose.pose.position.z=state[2];
		msg.twist.twist.linear.x=state[3];
		msg.twist.twist.linear.y=state[4];
		msg.twist.twist.linear.z=state[5];
		output_state_pub_.publish(msg);
	}
		
	// socket shutdown?
	if( 0>=err )
	{
		ROS_ERROR_STREAM("tweak failure (if persistent, restart the node)");
	}
}

int carote::Tweak::publisher(void)
{
	// create socket
	sd_=socket(AF_INET,SOCK_STREAM,0);
	if( 0>sd_ )
	{
		ROS_ERROR_STREAM("tweak failure: socket() -- " << strerror(errno));
		return -1;
	}

	// connect to server
	struct sockaddr_in server;
	server.sin_family=AF_INET;
	server.sin_addr.s_addr=inet_addr(params_.host.c_str());
	server.sin_port=htons(params_.port);
	if( 0>connect(sd_,(struct sockaddr*)&server,sizeof(server)) )
	{
		ROS_ERROR_STREAM("tweak failure: connect() -- " << strerror(errno));
		return -1;
	}

	// prepare for advertise to the output topics
	output_state_pub_=node_.advertise<nav_msgs::Odometry>(params_.output_state,1);
	timer_=node_.createTimer(ros::Rate(params_.rate),&carote::Tweak::output,this);

	return 0;
}

int carote::Tweak::subscriber(void)
{
	// create socket
	sd_=socket(AF_INET,SOCK_STREAM,0);
	if( 0>sd_ )
	{
		ROS_ERROR_STREAM("tweak failure: socket() -- " << strerror(errno));
		return -1;
	}

	// avoid the "already in use" error while binding
	int option=1;
	if( 0>setsockopt(sd_,SOL_SOCKET,SO_REUSEADDR,&option,sizeof(option)) )
	{
		ROS_ERROR_STREAM("tweak failure: setsockopt() -- " << strerror(errno));
		return -1;
	}

	// bind to socket
	struct sockaddr_in server;
	server.sin_family=AF_INET;
	server.sin_addr.s_addr=INADDR_ANY;
	server.sin_port=htons(params_.port);
	if( 0>bind(sd_,(struct sockaddr*)&server,sizeof(server)) )
	{
		ROS_ERROR_STREAM("tweak failure: bind() -- " << strerror(errno));
		return -1;
	}

    // listen to socket
    if( 0>listen(sd_,1) )
	{
		ROS_ERROR_STREAM("tweak failure: listen() -- " << strerror(errno));
		return -1;
	}
     
    // accept incoming connections
    int csize=sizeof(struct sockaddr_in);
	struct sockaddr_in client;
    fd_=accept(sd_,(struct sockaddr*)&client,(socklen_t*)&csize);
    if( 0>fd_ )
    {
		ROS_ERROR_STREAM("tweak failure: accept() -- " << strerror(errno));
		return -1;
    }

	// prepare for listening to input topics
	input_state_sub_=node_.subscribe(params_.input_state,1,&carote::Tweak::input,this);

	return 0;
}

void carote::Tweak::reconfigure(carote::TweakConfig &config, uint32_t level)
{
	// update parameters
	params_=config;

	// stop timer
	timer_.stop();

	// close current connections
	this->shutdown();

	// prepare for tweaking?
	if( params_.enable )
	{
		int err;
		char *uri=getenv("ROS_MASTER_URI");
		if( strstr(uri,params_.host.c_str()) )
		{
			err=this->subscriber();
		}
		else
		{
			err=this->publisher();
		}

		if( 0>err )
		{
			ROS_ERROR_STREAM("tweak failure (if persistent, restart node)");
		}
		else
		{
			ROS_WARN_STREAM("tweak enable");
		}
	}
	else
	{
		ROS_WARN_STREAM("tweak disable");
	}

}

void carote::Tweak::shutdown(void)
{
	if( 0<fd_ )
	{
		if( 0>close(fd_) )
		{
			ROS_ERROR_STREAM("tweak failure: close() -- " << strerror(errno));
		}
		fd_=-1;
	}

	if( 0<sd_ )
	{
		if( 0>close(sd_) )
		{
			ROS_ERROR_STREAM("tweak failure: close() -- " << strerror(errno));
		}
		sd_=-1;
	}

	input_state_sub_.shutdown();
	output_state_pub_.shutdown();
}
*/
