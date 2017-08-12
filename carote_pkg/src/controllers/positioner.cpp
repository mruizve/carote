#include<Eigen/SVD>
#include<geometry_msgs/Twist.h>
#include "carote/Positioner.h"
#include "carote/Utils.h"

//compute the matrix pseudo inverse using the Eigen library
template<typename T> T pseudoInverse(const T& _m, const double _eps)
{
	// compute the SVD and get the singular values
	typename JacobiSVD<T>::SingularValuesType sigma;
	JacobiSVD<T> svd(_m,ComputeFullU|ComputeFullV);
	sigma=svd.singularValues();

	//compute the inverse of the singular values avoiding kinematics singularities
	for(unsigned int i=0;sigma.size()>i;i++)
	{
		if( _eps<sigma(i) )
		{
			sigma(i)=1.0/sigma(i);
		}
		else
		{
			sigma(i)=1.0/_eps;
		}
	}

	// generate the diagonal matrix of singular values
	T Sigma(_m.cols(),_m.rows());
	Sigma.setZero();
	Sigma.diagonal()=sigma;

	//compute the pseudo inverse psi = V * Sigma^-1 * U^T
	return svd.matrixV()*Sigma*svd.matrixU().transpose();
}


carote::Positioner::Positioner(const std::string& _name)
:
	Controller(_name),
	kdl_J_solver_(NULL)
{
	// ros stuff: dynamic reconfiguration of controller parameters
	dynamic_reconfigure::Server<carote::PositionerConfig>::CallbackType f;
	f=boost::bind(&carote::Positioner::cbReconfigure,this,_1,_2);
	server_.setCallback(f);

	// resize Jacobians to fit robot (model) geometry
	J_tip_.resize(model_->getNrOfJoints());
	J_rcm_.resize(model_->getNrOfJoints());
}

carote::Positioner::~Positioner(void)
{
	if( NULL!=kdl_J_solver_ )
	{
		delete kdl_J_solver_;
	}
}

/*
void carote::Positioner::cbControl(const ros::TimerEvent& _event)
{
	KDL::JntArray u(model_->getNrOfJoints());

	// if there is a control program defined, then
	if( 0<u_.size() )
	{
		// get the next control command
		u=u_.front();
		u_.pop_front();
	}

	// send control command (if any) to the robot
	this->armVelocities(u);

	// if a new target frame has been obtained, then we have
	// to re-plan the control program based on this new feedback
	#define CP control_params_
	if( target_flag_ && goal_flag_ )
	{
		// clear the current control program
		u_.clear();

		// clear the target flag
		target_flag_=0;
	}
	#undef CP
}
*/

void carote::Positioner::cbReconfigure(carote::PositionerConfig& _config, uint32_t _level)
{
	// update parameters
	control_params_=_config;
/*
	if( control_params_.enabled )
	{
		ros::Duration period(1.0/control_params_.rate);
		this->start(period);
	}
	else
	{
		// delete any residual control program
		u_.clear();

		// stop the robot motion and control action
		this->stop();
	}
*/
}

void carote::Positioner::cbState(const sensor_msgs::JointState& _msg)
{
	// call the base controller method
	Controller::cbState(_msg);

	// if the arm state was not updated or there's no an available target, then
	if( !states_flag_ )
	{
		// do nothing
		return;
	}

	// error vector and control variable (default: zero)
	Eigen::Vector3d error=Eigen::Vector3d::Zero();
	KDL::JntArray u(model_->getNrOfJoints());

	// compute feedback and send new velocity command to the arm
	#define CP control_params_
	if( CP.enabled && target_flag_ && goal_flag_ )
	{
		// get tip frame
		if( 0>model_->JntToCart(q_,tip_) )
		{
			CAROTE_NODE_ABORT("cbControl(): unexpected error from forward kinematics");
		}

		// compute Jacobians of tip and RCM
		if( 0>model_->JntToJac(q_,J_tip_) )
		{
			CAROTE_NODE_ABORT("cbControl(): unexpected error from Jacobian solver");
		}
		if( 0>kdl_J_solver_->JntToJac(q_,J_rcm_) )
		{
			CAROTE_NODE_ABORT("cbControl(): unexpected error from Jacobian solver");
		}

		// get goal and RCM frames in base link coordinates
		KDL::Frame goal=target_*goal_;
		KDL::Frame rcm=tip_*rcm_;

		// build the 'augmented' Jacobian, which comprises:
		//   -- tip constraint along the z-axis,
		//   -- RCM constraints along x and z axis.
		// (we assume that q_(0)->"arm_joint_1" and q_(5)->"arm_joint_5")
		Eigen::Matrix3d J;
		J.block<1,3>(0,0)=J_tip_.data.block(2,1,1,3);
		J.block<1,3>(1,0)=J_rcm_.data.block(0,1,1,3);
		J.block<1,3>(2,0)=J_rcm_.data.block(2,1,1,3);

		// compute the pseudo-inverse of the 'augmented' Jacobian
		Eigen::Matrix3d psiJ=pseudoInverse(J,CP.eps);

		// compute positional errors
		KDL::Vector e_tip=goal.p-tip_.p;
		KDL::Vector e_rcm=target_.p-(tip_*rcm_).p;
		error << e_tip[2],e_rcm[0],e_rcm[2];

		double psi=error.norm();
		double gamma=(1.0-exp(-0.5*psi*psi/(100*CP.angular_error*CP.angular_error)));

		// compute joint velocities
		Eigen::Vector3d qp=psiJ*error;
		double n=( CP.eps>qp.norm() )? CP.eps : qp.norm();
		qp=gamma*CP.qp*qp/n;

		// update control values
		u(0)=0.0;
		u(1)=qp(0);
		u(2)=qp(1);
		u(3)=qp(2);
		u(4)=0.0;

		// clear states flags
		states_flag_=0;
	}
	#undef CP

	// send control command
	this->armVelocities(u);
}


void carote::Positioner::cbTarget(const geometry_msgs::PoseArray& _msg)
{
	// retrieve current state
	int flag=operator_flag_;

	// call the base controller method
	Controller::cbTarget(_msg);

	// generate the kinematic chain with the virtual RCM segment
	if( flag )
	{
		// get the robot the kinematics chain
		KDL::Chain kdl_chain(model_->getChain());

		// compute the RCM frame with respect the tip frame
		rcm_=KDL::Frame(KDL::Vector(0.0,0.0,goal_params_.rho));

		// add RCM segment to the kinematic chain
		KDL::Joint joint(KDL::Joint::None);
		KDL::Segment segment(joint,rcm_);
		kdl_chain.addSegment(segment);

		// (re)create the Jacobian solver
		if( NULL!=kdl_J_solver_)
		{
			delete kdl_J_solver_;
		}
		kdl_J_solver_=new KDL::ChainJntToJacSolver(kdl_chain);
	}
}
