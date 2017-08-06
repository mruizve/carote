#include "carote/Model.h"
#include "carote/Utils.h"

int carote::Model::getNrOfJoints(void)
{
	return nJoints_;
}

const std::string& carote::Model::getJointName(int i)
{
	if( 0>i || q_names_.size()<=i )
	{
        CAROTE_NODE_ABORT("carote::Model::getJointName(): index out of bounds");
	}

	return q_names_[i];
}

const std::vector<std::string>& carote::Model::getJointsNames(void)
{
	return q_names_;
}

const std::string& carote::Model::getJointUnit(int i)
{
	if( 0>i || q_names_.size()<=i )
	{
        CAROTE_NODE_ABORT("carote::Model::getJointName(): index out of bounds");
	}

	return q_units_[i];
}

const std::vector<std::string>& carote::Model::getJointsUnits(void)
{
	return q_units_;
}

const std::string& carote::Model::getSpeedUnit(int i)
{
	if( 0>i || q_names_.size()<=i )
	{
        CAROTE_NODE_ABORT("carote::Model::getJointName(): index out of bounds");
	}

	return qp_units_[i];
}

const std::vector<std::string>& carote::Model::getSpeedsUnits(void)
{
	return qp_units_;
}
