#include<Eigen/Geometry>
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
		if( 5.0*_eps<sigma(i) )
		{
			sigma(i)=1.0/sigma(i);
		}
		else
		{
			sigma(i)=1.0/(_eps*_eps+sigma(i)*sigma(i));
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

void carote::Positioner::cbControl(const ros::TimerEvent& _event)
{
	static KDL::JntArray u(model_->getNrOfJoints());

	// if a new target frame has been obtained or the robot
	// state has been updated, then we have to re-plan the
	// control program based on this new feedback
	#define CP control_params_
	if( (states_flag_ || target_flag_) && goal_flag_ )
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

		// build the task Jacobian, which comprises:
		//   -- tip constraint along the z-axis,
		//   -- RCM constraints along x and z axis.
		// (we assume that q_(0)->"arm_joint_1" and q_(5)->"arm_joint_5")
		Eigen::Matrix3d J;
		J.block<1,3>(0,0)=J_tip_.data.block(2,1,1,3);
		J.block<1,3>(1,0)=J_rcm_.data.block(0,1,1,3);
		J.block<1,3>(2,0)=J_rcm_.data.block(2,1,1,3);

		// compute the pseudo-inverse of the 'augmented' Jacobian
		Eigen::Matrix3d psiJ=pseudoInverse(J,CP.mu);

		// compute positional errors
		KDL::Vector e_tip=goal.p-tip_.p;
		KDL::Vector e_rcm=target_.p-(tip_*rcm_).p;

		// compute joints velocities
		Eigen::Vector3d e;
		e << e_tip[2],e_rcm[0],e_rcm[2];
		Eigen::Vector3d qp=psiJ*e;
		qp=CP.qp*(1-exp(-e.norm()*e.norm()/(100*CP.cartesian_error*CP.cartesian_error)))*qp/qp.norm();

		KDL::SetToZero(u);
		u(1)=qp(0);
		u(2)=qp(1);
		u(3)=qp(2);
		this->armVelocities(u);

		KDL::Twist twist;
		KDL::SetToZero(twist);
		if( 0.0<CP.eta )
		{
			// manipulability Jacobian
			J.block<1,3>(0,0)=J_tip_.data.block(0,1,1,3);
			J.block<1,3>(1,0)=J_tip_.data.block(2,1,1,3);
			J.block<1,3>(2,0)=J_tip_.data.block(4,1,1,3);
			psiJ=pseudoInverse(J,CP.mu);

			// manipulability test direction
			Eigen::Vector3d qx=psiJ*Eigen::Vector3d::UnitX();
			qx.normalize();

			double dt=1.0/CP.rate;
			KDL::JntArray q(model_->getNrOfJoints());

			q(0)=q_(0);
			q(1)=q_(1)-dt*qx(0);
			q(2)=q_(2)-dt*qx(1);
			q(3)=q_(3)-dt*qx(2);
			q(4)=q_(4);
			KDL::Jacobian J_tip_l(model_->getNrOfJoints());
			if( 0>model_->JntToJac(q,J_tip_l) )
			{
				CAROTE_NODE_ABORT("cbControl(): unexpected error from Jacobian solver");
			}
			Eigen::Matrix3d J_l;
			J_l.block<1,3>(0,0)=J_tip_l.data.block(0,1,1,3);
			J_l.block<1,3>(1,0)=J_tip_l.data.block(2,1,1,3);
			J_l.block<1,3>(2,0)=J_tip_l.data.block(4,1,1,3);

			double mJ=sqrt(fabs((J*J.transpose()).determinant()));
			double mJ_l=sqrt(fabs((J_l*J_l.transpose()).determinant()));
			double D_l=(mJ_l-mJ)/dt;
			if( 1e-5<fabs(D_l) )
			{
				twist(0)=sgn(D_l)*CP.eta*(1-exp(-(D_l*D_l)*5e5));
				twist(0)+=J_l.block<1,3>(0,0)*qp;
			}
		}
		this->baseTwist(twist);

		// clear the states and target flag
		states_flag_=0;
		target_flag_=0;
	}
	#undef CP
}

void carote::Positioner::cbReconfigure(carote::PositionerConfig& _config, uint32_t _level)
{
	// update parameters
	control_params_=_config;

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
