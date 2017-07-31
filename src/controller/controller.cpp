#include<tf_conversions/tf_eigen.h>
#include "carote/Controller.h"

carote::Controller::Controller(const std::string& _name)
:
	node_("~"),
	name_(_name),
	flag_operator_(0),
	flag_target_(0)
{
	// ros initialization
	this->init();

	std::string strpar;

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
