#include "carote/Controller.h"
#include "carote/Utils.h"

void carote::Controller::loadXMLPose(const std::string _param, KDL::JntArray& _q)
{
	XmlRpc::XmlRpcValue xmlpar;

	// the input parameter have been defined before running the node?
	if( !node_.getParam(_param,xmlpar) )
	{
		CAROTE_NODE_ABORT("missing param '" << _param << "' (must be defined at setup.launch)");
	}

	// validate the parameter data type
	if( XmlRpc::XmlRpcValue::TypeArray!=xmlpar.getType() )
	{
		CAROTE_NODE_ABORT("invalid param '" << _param << "': XmlRpc::XmlRpcValue::TypeArray expected");
	}

	// for each segment of the arm chain,
	const std::vector<std::string> &q_names=model_->getJointsNames();
	for( int i=0; model_->getNrOfJoints()>i; i++ )
	{
		// parse joint value on the XML parameter
		int missing=1;
		for( int j=0; xmlpar.size()>j; j++ )
		{
			XmlRpc::XmlRpcValue& xmlval=xmlpar[j];

			// value found?
			if( xmlval.hasMember(q_names[i]) )
			{
				// validate value type
				if( XmlRpc::XmlRpcValue::TypeDouble!=xmlval[q_names[i]].getType() )
				{
					CAROTE_NODE_ABORT("invalid param '" << _param << "': wrong '"+q_names[i]+"' value type");
				}

				// assign value
				_q(i)=(double)xmlval[q_names[i]];
				missing=0;
				break;
			}
		}
		if( missing )
		{
			CAROTE_NODE_ABORT("invalid param '" << _param << "': missing '"+q_names[i]+"' value");
		}
	}
}

void carote::Controller::initPoses(void)
{
	// resize home and work poses arrays
	q_home_.resize(model_->getNrOfJoints());
	q_work_.resize(model_->getNrOfJoints());

	// load home pose
	this->loadXMLPose("/carote/poses/home",q_home_);

	// load initial work pose
	this->loadXMLPose("/carote/poses/work",q_work_);
}


void carote::Controller::home(void)
{
	// stop the robot
	this->stop();

	// move robot to the home position
	this->armPose(q_home_);

	// wait some time
	ros::Duration(0.5).sleep();
}

void carote::Controller::work(void)
{
	// stop the robot
	this->stop();

	// move robot to the work position
	this->armPose(q_work_);

	// wait some time
	ros::Duration(0.5).sleep();
}

const KDL::JntArray& carote::Controller::getHomePose(void) const
{
	return this->q_home_;
}

const KDL::JntArray& carote::Controller::getWorkPose(void) const
{
	return this->q_work_;
}
