#include "carote/Controller.h"

void carote::Controller::loadXMLPose(const std::string _param, KDL::JntArray& _q)
{
	XmlRpc::XmlRpcValue xmlpar;

	// the input parameter have been defined before running the node?
	if( !node_.getParam(_param,xmlpar) )
	{
		CAROTE_NODE_ABORT("missing param '" << _param << "' (must be defined at setup.launch");
	}

	// validate the parameter data type
	if( XmlRpc::XmlRpcValue::TypeArray!=xmlpar.getType() )
	{
		CAROTE_NODE_ABORT("invalid param '" << _param << "': XmlRpc::XmlRpcValue::TypeArray expected");
	}

	// for each segment of the arm chain,
	for( int i=0; kdl_chain_.getNrOfSegments()>i; i++ )
	{
		if( urdf::Joint::FIXED==q_types_[i] )
		{
			continue;
		}

		// parse joint value on the XML parameter
		int missing=1;
		for( int j=0; xmlpar.size()>j; j++ )
		{
			XmlRpc::XmlRpcValue& xmlval=xmlpar[j];

			// value found?
			if( xmlval.hasMember(q_names_[i]) )
			{
				// validate value type
				if( XmlRpc::XmlRpcValue::TypeDouble!=xmlval[q_names_[i]].getType() )
				{
					CAROTE_NODE_ABORT("invalid param '" << _param << "': wrong '"+q_names_[i]+"' value type");
				}

				// assign value
				_q(i)=(double)xmlval[q_names_[i]];
				missing=0;
				break;
			}
		}
		if( missing )
		{
			CAROTE_NODE_ABORT("invalid param '" << _param << "': missing '"+q_names_[i]+"' value");
		}
	}
}

void carote::Controller::initPoses(void)
{
	// resize home and zero arrays
	q_home_.resize(kdl_chain_.getNrOfSegments());
	q_zero_.resize(kdl_chain_.getNrOfSegments());

	// load home pose
	this->loadXMLPose("/carote/poses/home",q_home_);

	// load zero pose
	this->loadXMLPose("/carote/poses/zero",q_zero_);
}

void carote::Controller::moveTo(KDL::JntArray& _q)
{
	ros::Time tstamp=ros::Time::now();

	// initialize message
	brics_actuator::JointPositions msg;

	// for each joint of the chain
	brics_actuator::JointValue joint;
	for( int i=0; kdl_chain_.getNrOfSegments()>i; i++ )
	{
		// that is not fixed,
		if( urdf::Joint::FIXED==q_types_[i] )
		{
			continue;
		}

		// record data
		joint.timeStamp=tstamp;
		joint.joint_uri=q_names_[i];
		if( urdf::Joint::PRISMATIC==q_types_[i] )
		{
			joint.unit="meters";
		}
		else
		{
			joint.unit="rad";
		}
		joint.value=_q(i);

		// add data to the messages
		msg.positions.push_back(joint);
	}

	// publish message
	pub_arm_pos_.publish(msg);
}
