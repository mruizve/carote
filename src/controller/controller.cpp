#include<tf_conversions/tf_eigen.h>
#include "carote/Controller.h"

carote::Controller::Controller(const std::string& _name)
:
	node_("~"),
	name_(_name),
	flag_operator_(0),
	flag_states_(0),
	flag_target_(0)
{
	// ros initialization
	this->initROS();

	// udrf/kld initializations
	this->initKinematics();

	// poses initialization
	this->initPoses();

	// show results to the user
    for( int i=0; kdl_chain_.getNrOfSegments()>i; i++ )
    {
		if( urdf::Joint::FIXED!=q_types_[i] )
		{
			ROS_WARN_STREAM("[arm chain] "
				<< q_names_[i]
				<< ", range=[" << q_lower_(i) << "," << q_upper_(i)
				<< "], poses={ home=" << q_home_(i)
				<< ", zero=" << q_zero_(i)
				<< " }");
		}
		else
		{
			ROS_WARN_STREAM("[arm chain] " << q_names_[i] << ", fixed");
		}
	}
}

carote::Controller::~Controller(void)
{
}
