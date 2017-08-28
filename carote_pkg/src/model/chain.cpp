#include "carote/Model.h"
#include "carote/Utils.h"

const KDL::Chain& carote::Model::getChain(void) const
{
	return kdl_chain_;
}

int carote::Model::getSegmentIndex(const std::string& _name) const
{
	for( size_t i=0; kdl_chain_.getNrOfSegments()>i; i++ )
	{
		if( kdl_chain_.getSegment(i).getName()==_name )
		{
			return i;
		}
	}

	CAROTE_NODE_ABORT("Model::getSegmentIndex() segment '" << _name << "' not found");
}
