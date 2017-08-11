#include "carote/Model.h"
#include "carote/Utils.h"

void carote::Model::countNrOfJoints(urdf::Model& urdf_model)
{
	nJoints_=0;
    for( int i=0; kdl_chain_.getNrOfSegments()>i; i++ )
    {
		KDL::Segment segment=kdl_chain_.getSegment(i);

		// get URDF joint information
		boost::shared_ptr<const urdf::Link> link=urdf_model.getLink(segment.getName());
		if( !link )
		{
			CAROTE_NODE_ABORT("KDL segment '" << segment.getName() << "' without equivalent URDF link!?");
		}
		boost::shared_ptr<const urdf::Joint> joint=urdf_model.getJoint(link->parent_joint->name);
		if( !joint )
		{
			CAROTE_NODE_ABORT("KDL segment '" << segment.getName() << "' without equivalent URDF parent_joint!?");
		}

		if( urdf::Joint::FIXED!=joint->type )
		{
			nJoints_++;
		}
	}
}

int carote::Model::getNrOfJoints(void) const
{
	return nJoints_;
}

const std::string& carote::Model::getJointName(int i) const
{
	if( 0>i || q_names_.size()<=i )
	{
        CAROTE_NODE_ABORT("carote::Model::getJointName(): index out of bounds");
	}

	return q_names_[i];
}

const std::vector<std::string>& carote::Model::getJointsNames(void) const
{
	return q_names_;
}

const std::string& carote::Model::getJointUnit(int i) const
{
	if( 0>i || q_names_.size()<=i )
	{
        CAROTE_NODE_ABORT("carote::Model::getJointName(): index out of bounds");
	}

	return q_units_[i];
}

const std::vector<std::string>& carote::Model::getJointsUnits(void) const
{
	return q_units_;
}

const std::string& carote::Model::getSpeedUnit(int i) const
{
	if( 0>i || q_names_.size()<=i )
	{
        CAROTE_NODE_ABORT("carote::Model::getJointName(): index out of bounds");
	}

	return qp_units_[i];
}

const std::vector<std::string>& carote::Model::getSpeedsUnits(void) const
{
	return qp_units_;
}
