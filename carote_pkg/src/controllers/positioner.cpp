#include<geometry_msgs/Twist.h>
#include "carote/Positioner.h"
#include "carote/Utils.h"

carote::Positioner::Positioner(const std::string& _name)
:
	Controller(_name)
{
	// ros stuff: dynamic reconfiguration of controller parameters
	dynamic_reconfigure::Server<carote::PositionerConfig>::CallbackType f;
	f=boost::bind(&carote::Positioner::cbReconfigure,this,_1,_2);
	server_.setCallback(f);

	// initialize Jacobians
	J_rcm_.resize(model_->getNrOfJoints());
	J_tip_.resize(model_->getNrOfJoints());
	J_wrist_.resize(model_->getNrOfJoints());
}

carote::Positioner::~Positioner(void)
{
}

double carote::Positioner::manipulability(void)
{
	// get the current time step
	double dt=1.0/control_params_.rate;

	// manipulability Jacobian =  [ xp_tip; zp_tip; wy_tip ]
	Eigen::Matrix3d J;
	J.block<1,3>(0,0)=J_tip_.data.block(0,1,1,3);
	J.block<1,3>(1,0)=J_tip_.data.block(2,1,1,3);
	J.block<1,3>(2,0)=J_tip_.data.block(4,1,1,3);

	// manipulability will be increased by a displacement along the x direction
	// of the base link, therefore we need to map such direction in joints space
	// to compute the directional derivative of the manipulability measure.
	// (note that due all the constraints on the end effector and wrist, any
	// displacement of the base -shoulder- will force a displacement of the elbow,
	// that is, a movement along the null space of the task's Jacobian).
	Eigen::Vector3d qpx=pinv(J,control_params_.mu)*Eigen::Vector3d::UnitX();
	qpx.normalize();

	// based on the compute direction qx, compute the state qx=q+dt*qpx
	static KDL::JntArray qx(model_->getNrOfJoints());
	qx(0)=q_(0);
	qx(1)=q_(1)-dt*qpx(0);
	qx(2)=q_(2)-dt*qpx(1);
	qx(3)=q_(3)-dt*qpx(2);
	qx(4)=q_(4);

	// compute the tip Jacobian at qx
	static KDL::Jacobian J_tip_x(model_->getNrOfJoints());
	if( 0>model_->JntToJac(qx,J_tip_x) )
	{
		CAROTE_NODE_ABORT("Positioner::manipulability(): unexpected error from Jacobian solver");
	}

	// compute the manipulability Jacobian at qx
	Eigen::Matrix3d Jx;
	Jx.block<1,3>(0,0)=J_tip_x.data.block(0,1,1,3);
	Jx.block<1,3>(1,0)=J_tip_x.data.block(2,1,1,3);
	Jx.block<1,3>(2,0)=J_tip_x.data.block(4,1,1,3);

	double m=sqrt(fabs((J*J.transpose()).determinant()));
	double mx=sqrt(fabs((Jx*Jx.transpose()).determinant()));
	double dm=(mx-m)/dt;

	// manipulability maximizing control (in base velocity units)
	return sgn(dm)*control_params_.v*(1-exp(-pow(10*dm/control_params_.eps,2)));
}

void carote::Positioner::updateStates(const KDL::JntArray& _u)
{
	// if the robot was not updated, then
	if( !states_flag_ )
	{
		// get the current time step
		double dt=1.0/control_params_.rate;

		// update the robot pose using vanilla odometry
		q_.data=q_.data+dt*_u.data;
	}
	else
	{
		// otherwise clear the flag and enjoy the feedback
		states_flag_=0;
	}
}

void carote::Positioner::updateTarget(const KDL::Twist& _u)
{
	// if the target frame was not updated, then
	if( !target_flag_ )
	{
		// get the current time step
		double dt=1.0/control_params_.rate;

		// update the base link frame using vanilla odometry
		KDL::Vector p(dt*_u(0),dt*_u(1),0.0);
		KDL::Rotation M=KDL::Rotation::Identity();
		M.DoRotZ(dt*_u(5));
		KDL::Frame base(M,p);

		// update the target frame based on the new base link pose
		target_=base.Inverse()*target_;
	}
	else
	{
		// otherwise clear the flag and enjoy the feedback
		target_flag_=0;
	}
}

void carote::Positioner::updateKinematics(void)
{
	// number of the wrist segment inside the robot model (chain)
	static int wristNR=model_->getSegmentIndex(frame_id_wrist_.substr(1));

	// get tip and wrist frame using forward kinematics
	if( 0>model_->JntToCart(q_,tip_) || 0>model_->JntToCart(q_,wrist_,wristNR) )
	{
		CAROTE_NODE_ABORT("Positioner::cbControl(): unexpected error from forward kinematics");
	}

	// normalized distance between tip and RCM
	double rho=goal_params_.rho/(tip_.p-wrist_.p).Norm();

	// get RCM frame in base link coordinates
	rcm_=KDL::Frame(tip_.M,tip_.p+(tip_.p-wrist_.p)*rho);

	// get Jacobians of tip and wrist
	if( 0>model_->JntToJac(q_,J_tip_) || 0>model_->JntToJac(q_,J_wrist_,wristNR))
	{
		CAROTE_NODE_ABORT("Positioner::cbControl(): unexpected error from Jacobian solver");
	}

	// compute RCM Jacobian
	J_rcm_.data=J_tip_.data+(J_tip_.data-J_wrist_.data)*rho;
}

void carote::Positioner::cbControl(const ros::TimerEvent& _event)
{
	// velocities commands
	static KDL::Twist u_base;
	static KDL::JntArray u_arm(model_->getNrOfJoints());

	// is there's is no goal defined, then
	if( !goal_flag_ )
	{
		// be sure that the robot is doing nothing
		KDL::SetToZero(u_arm);
		this->armVelocities(u_arm);

		// and standing still
		KDL::SetToZero(u_base);
		this->baseTwist(u_base);

		return;
	}

	// update states with vanilla odometry if feedback is not yet available
	this->updateStates(u_arm);
	this->updateTarget(u_base);

	// compute RCM, tip and wrist frames and Jacobians
	this->updateKinematics();

	// build the task Jacobian, which comprises:
	//   -- tip constraint along the z-axis,
	//   -- RCM constraints along x and z axis.
	// (we assume that q_(0)->"arm_joint_1" and q_(5)->"arm_joint_5")
	Eigen::Matrix3d J;
	J.block<1,3>(0,0)=J_tip_.data.block(2,1,1,3);
	J.block<1,3>(1,0)=J_rcm_.data.block(0,1,1,3);
	J.block<1,3>(2,0)=J_rcm_.data.block(2,1,1,3);

	// compute the pseudo-inverse of the task Jacobian
	Eigen::Matrix3d psiJ=pinv(J,control_params_.mu);

	// get goal frame in base link coordinates and compute error
	KDL::Frame goal=target_*goal_;
	KDL::Vector e_tip=goal.p-tip_.p;
	KDL::Vector e_rcm=target_.p-rcm_.p;
	Eigen::Vector3d e;
	e << e_tip[2],e_rcm[0],e_rcm[2];

	// compute joints velocities
	Eigen::Vector3d qp=psiJ*e;
	qp=control_params_.qp*(1-exp(-pow(0.1*e.norm()/control_params_.eps,2)))*qp/qp.norm();

	// clear previous control commands
	KDL::SetToZero(u_base);
	KDL::SetToZero(u_arm);

	// compute the manipulability maximizing control (base velocity along x)
	// (the input parameter is to compute the relative motion along the x axis
	// between base link and the tip produced by the task control law)
	double um=0.5*this->manipulability();

	// map manipulability value to the joints space (from base velocities space)
	um=control_params_.qp*um/control_params_.v;

	// update joints velocities to compensate base movement
	// (note that the base movement is only half of the desired velocity)
	Eigen::Vector3d vo,vf;
	vo=J*qp;
	qp=qp-um*psiJ*Eigen::Vector3d::UnitY();
	vf=J*qp;

	// apply to the base the predicted velocity variation of the RCM along the
	// x-direction of the base reference frame
	u_base(0)=vo(1)-vf(1);

	// send velocity commands
	u_arm(1)=qp(0);
	u_arm(2)=qp(1);
	u_arm(3)=qp(2);
	this->armVelocities(u_arm);
	this->baseTwist(u_base);
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
		// stop the robot motion and control action
		this->stop();
	}
}
