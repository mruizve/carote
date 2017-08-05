#include "carote/Controller.h"
#include "carote/Utils.h"

carote::Controller::Controller(const std::string& _name)
:
	node_("~"),
	name_(_name),
	model_(NULL),
	states_flag_(0),
	goal_flag_(0),
	target_flag_(0)
{
	// ros initialization
	this->initROS();

	// get the robot urdf model
	std::string xmlpar;
	if( !node_.getParam("/robot_description",xmlpar) )
	{
		CAROTE_NODE_ABORT("missing param '/robot_description' (launch robot drivers before controller)");
	}

	// create model
	model_=new carote::Model(xmlpar,frame_id_base_.substr(1),frame_id_tip_.substr(1));

	// resize joint states arrays
	q_.resize(model_->getNrOfJoints());
	qp_.resize(model_->getNrOfJoints());

	// poses initialization
	this->initPoses();
}

carote::Controller::~Controller(void)
{
	if( NULL!=model_ )
	{
		delete model_;
	}
}

void carote::Controller::start(ros::Duration _period)
{
	// stop the robot
	this->stop();

	// start internal timer
	timer_=node_.createTimer(_period,&Controller::cbControl,this);
}

void carote::Controller::stop(void)
{
	// stop the timer
	timer_.stop();
	
	// stop the robot motion
	this->zero();
}
void carote::Controller::zero(void)
{
	// delete any residual control action
	// (this is a pure virtual function depends on the implementation)
	this->clean();

	// stop the robot platform or base
	this->baseTwist(KDL::Twist::Zero());
	
	// stop the arm
	this->armSpeed(KDL::JntArray(model_->getNrOfJoints()));

	// wait some time
	ros::Duration(0.5).sleep();
}
