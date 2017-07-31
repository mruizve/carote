#include<tf_conversions/tf_eigen.h>
#include "carote/Controller.h"

carote::Controller::Controller(const std::string& _name)
:
	node_("~"),
	name_(_name),
	flag_operator_(0),
	flag_target_(0)
{
	std::string strpar;

	// ros stuff: prepare for advertise to the control topics of the platform
	if( !node_.getParam("/carote/topics/control_base",strpar) )
	{
		// show error and close node
		ROS_ERROR_STREAM("missing parameter /carote/topics/control_base, include setup.launch in your launch file");
		exit(EXIT_FAILURE);
	}
	pub_control_base_=node_.advertise<geometry_msgs::Twist>(strpar,1);

	// ros stuff: prepare for advertise to the control topics of the arm
	if( !node_.getParam("/carote/topics/control_arm",strpar) )
	{
		// show error and close node
		ROS_ERROR_STREAM("missing parameter /carote/topics/control_arm, include setup.launch in your launch file");
		exit(EXIT_FAILURE);
	}
	pub_control_arm_=node_.advertise<brics_actuator::JointVelocities>(strpar,1);

	// ros stuff: prepare for listening to the operator topic
	if( !node_.getParam("/carote/topics/operator",strpar) )
	{
		// show error and close node
		ROS_ERROR_STREAM("missing parameter /carote/topics/operator, include setup.launch in your launch file");
		exit(EXIT_FAILURE);
	}
	sub_operator_=node_.subscribe(strpar,1,&carote::Controller::cbOperator,this);

	// ros stuff: prepare for listening to the joint states topic
	if( !node_.getParam("/carote/topics/state",strpar) )
	{
		// show error and close node
		ROS_ERROR_STREAM("missing parameter /carote/topics/state, include setup.launch in your launch file");
		exit(EXIT_FAILURE);
	}
	sub_state_=node_.subscribe(strpar,1,&carote::Controller::cbState,this);

	// ros stuff: prepare for listening to the target topic
	if( !node_.getParam("/carote/topics/target",strpar) )
	{
		// show error and close node
		ROS_ERROR_STREAM("missing parameter /carote/topics/target, include setup.launch in your launch file");
		exit(EXIT_FAILURE);
	}
	sub_target_=node_.subscribe(strpar,1,&carote::Controller::cbTarget,this);

	// ros stuff: get the name of platform or base reference frame
	if( !node_.getParam("/carote/frames/base",frame_id_base_) )
	{
		// show error and close node
		ROS_ERROR_STREAM("missing parameter /carote/frames/base, include setup.launch in your launch file");
		exit(EXIT_FAILURE);
	}

	// ros stuff: get the name of platform or base reference frame
	if( !node_.getParam("/carote/frames/gripper",frame_id_gripper_) )
	{
		// show error and close node
		ROS_ERROR_STREAM("missing parameter /carote/frames/gripper, include setup.launch in your launch file");
		exit(EXIT_FAILURE);
	}

	// ros stuff: get the name of the target reference frame
	if( !node_.getParam("/carote/frames/target",frame_id_target_) )
	{
		// show error and close node
		ROS_ERROR_STREAM("missing parameter /carote/frames/target, include setup.launch in your launch file");
		exit(EXIT_FAILURE);
	}

	// ros stuff: get the robot urdf model
	if( !node_.getParam("/robot_description",strpar) )
	{
		// show error and close node
		ROS_ERROR_STREAM("missing parameter /robot_description, launch drivers (youbot.launch) before the controller");
		exit(EXIT_FAILURE);
	}
	if( !kdl_parser::treeFromString(strpar,kdl_tree_) )
	{
		ROS_ERROR_STREAM("cannot construct the kdl tree of the robot");
		exit(EXIT_FAILURE);
	}
	if( !kdl_tree_.getChain(frame_id_base_.substr(1),frame_id_gripper_.substr(1),kdl_chain_) )
	{
		ROS_ERROR_STREAM("cannot retrive the kdl chain of the arm");
		exit(EXIT_FAILURE);
    }

/*
	KDL::SegmentMap segments=kdl_chain_.getSegments();
	for( KDL::SegmentMap::const_iterator i=segments.begin(); segments.end()!=i; i++ )
	{
		ROS_WARN_STREAM("segment: " << i->second.segment.getName());
		ROS_WARN_STREAM("  joint: " << i->second.segment.getJoint().getName());
	}
*/
/*
	// ros stuff: dynamic reconfiguration
	dynamic_reconfigure::Server<carote::ControllerConfig>::CallbackType f;
	f=boost::bind(&carote::Follower::reconfigure,this,_1,_2);
	server_.setCallback(f);
*/
}

carote::Controller::~Controller(void)
{
}

void carote::Controller::cbOperator(const geometry_msgs::Vector3& _msg)
{
	// get operator command
	phi_=_msg.x;
	lambda_=_msg.y;
	rho_=_msg.z;

	// update operator flag
	flag_operator_=1;
}

void carote::Controller::cbState(const sensor_msgs::JointState& _msg)
{
	// update state

	// new data available?
	if( flag_operator_ || flag_target_ )
	{
		// clear flags
		flag_operator_=0;
		flag_target_=0;

		// update control law
		this->update();
	}

	// get control command
	VelocityControlData control;
	if( control_queue_.empty() )
	{
		control.u=Eigen::Vector3d::Zero();
		control.qp=Eigen::Matrix<double,5,1>::Zero();
	}
	else
	{
		control=control_queue_.front();
	}

	// publish control command
	brics_actuator::JointVelocities msg_arm;
	geometry_msgs::Twist msg_base;
}

void carote::Controller::cbTarget(const geometry_msgs::PoseArray& _msg)
{
	// if the target is not with respect frame_id_base_, then lookup the correct transformation
	if( _msg.header.frame_id!=frame_id_base_ )
	{
		// get frames transform (from target to base)
		if( tf_listener_.waitForTransform(frame_id_base_,frame_id_target_,_msg.header.stamp,ros::Duration(0.1)) )
		{
			try
			{
				tf_listener_.lookupTransform(frame_id_base_,frame_id_target_,_msg.header.stamp,tf_base_target_);
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

	// get target position
	t_(0)=tf_base_target_.getOrigin()[0];
	t_(1)=tf_base_target_.getOrigin()[1];
	t_(2)=tf_base_target_.getOrigin()[2];

	// get target pose
	tf::Matrix3x3 R=tf_base_target_.getBasis();
	tf::matrixTFToEigen(R,R_);

	// update target data flag
	flag_target_=1;
}

void carote::Controller::cbControl(const ros::TimerEvent& event)
{
}
