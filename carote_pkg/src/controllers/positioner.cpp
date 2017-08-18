#include<geometry_msgs/Twist.h>
#include "carote/Positioner.h"
#include "carote/Utils.h"

carote::Positioner::Positioner(const std::string& _name)
:
	Controller(_name),
	kdl_J_solver_(NULL)
{
	// ros stuff: dynamic reconfiguration of controller parameters
	dynamic_reconfigure::Server<carote::PositionerConfig>::CallbackType f;
	f=boost::bind(&carote::Positioner::cbReconfigure,this,_1,_2);
	server_.setCallback(f);

	// ros stuff: get the name of shoulder reference frame
	if( !this->node().getParam("/carote/frames/shoulder",frame_id_shoulder_) )
	{
		CAROTE_NODE_ABORT("missing param '/carote/frames/shoulder' (must be defined at setup.launch)");
	}

	// initialize Jacobians
	J_rcm_.resize(model_->getNrOfJoints());
	J_tip_.resize(model_->getNrOfJoints());
}

carote::Positioner::~Positioner(void)
{
	if( NULL!=kdl_J_solver_)
	{
		delete kdl_J_solver_;
	}
}

double carote::Positioner::manipulability(void)
{
	// get the step size proportional to the time step and maximum joints velocities
	double h=control_params_.qp/control_params_.rate;

	// compute the manipulability Jacobian
	//          [ xp_tip; zp_tip; wy_tip ] = J_manipulability * qp
	// (it is assumed that the tip Jacobian in computed with respect the
	// sagittal frame orientation)
	Eigen::Matrix3d J;
	J.block<1,3>(0,0)=J_tip_.data.block(0,1,1,3);
	J.block<1,3>(1,0)=J_tip_.data.block(2,1,1,3);
	J.block<1,3>(2,0)=J_tip_.data.block(4,1,1,3);

	// compute the pseudo-inverse of the manipulability Jacobian
	Eigen::Matrix3d pJ=pinv(J,control_params_.mu);

	// determine the joints space direction associated to a displacement along
	// the x-axis of the sagittal reference frame
	Eigen::Vector3d qpx=pJ*Eigen::Vector3d::UnitX();

	// approximate numerically the directional derivative of the manipulability
	// along such direction (using central difference)
	KDL::JntArray ql(model_->getNrOfJoints());
	KDL::JntArray qr(model_->getNrOfJoints());
	KDL::Jacobian J_tip_l(model_->getNrOfJoints());
	KDL::Jacobian J_tip_r(model_->getNrOfJoints());

	// compute nearby states
	ql=q_; ql(1)-=h*qpx(0); ql(2)-=h*qpx(1); ql(3)-=h*qpx(2);
	qr=q_; qr(1)+=h*qpx(0); qr(2)+=h*qpx(1); qr(3)+=h*qpx(2);

	// compute the tip Jacobian at nearby states
	if( 0>model_->JntToJac(ql,J_tip_l) || 0>model_->JntToJac(qr,J_tip_r) )
	{
		CAROTE_NODE_ABORT("Positioner::manipulability(): unexpected error from Jacobian solver");
	}

	// change Jacobians base using the orientation of the sagittal frame
	J_tip_l.changeBase(sagittal_.M.Inverse());
	J_tip_r.changeBase(sagittal_.M.Inverse());

	// compute the associated manipulability Jacobians
	Eigen::Matrix3d Jl;
	Jl.block<1,3>(0,0)=J_tip_l.data.block(0,1,1,3);
	Jl.block<1,3>(1,0)=J_tip_l.data.block(2,1,1,3);
	Jl.block<1,3>(2,0)=J_tip_l.data.block(4,1,1,3);

	Eigen::Matrix3d Jr;
	Jr.block<1,3>(0,0)=J_tip_r.data.block(0,1,1,3);
	Jr.block<1,3>(1,0)=J_tip_r.data.block(2,1,1,3);
	Jr.block<1,3>(2,0)=J_tip_r.data.block(4,1,1,3);

	// approximate manipulability directional derivative by center difference
	double ml=sqrt(fabs((Jl*Jl.transpose()).determinant()));
	double mr=sqrt(fabs((Jr*Jr.transpose()).determinant()));
	return (mr-ml)/(2.0*h);
}

double carote::Positioner::selfcollision(void)
{
	double d=((target_*goal_).p-shoulder_.p).Norm();
	if( control_params_.d>=d )
	{
		return (1.0/d-1.0/control_params_.d)/(d*d);
	}

	return 0.0;
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
	// number of the shoulder segment inside the robot model (chain)
	static int shoulderNR=model_->getSegmentIndex(frame_id_shoulder_.substr(1));

	// get tip and shoulder frames using forward kinematics
	if( 0>model_->JntToCart(q_,tip_) || 0>model_->JntToCart(q_,shoulder_,shoulderNR) )
	{
		CAROTE_NODE_ABORT("Positioner::cbControl(): unexpected error from forward kinematics");
	}

	// compute Jacobians of tip and RCM
	if( 0>model_->JntToJac(q_,J_tip_) || 0>kdl_J_solver_->JntToJac(q_,J_rcm_) )
	{
		CAROTE_NODE_ABORT("Positioner::cbControl(): unexpected error from Jacobian solver");
	}

	// compute sagittal frame
	KDL::Vector Z(0.0,0.0,1.0);
	KDL::Vector Y(shoulder_.M.UnitY()); Y.Normalize();
	KDL::Vector X(Y*Z); X.Normalize();
	sagittal_=KDL::Frame(KDL::Rotation(X,Y,Z),KDL::Vector(0.0,0.0,0.0));

	// change Jacobians base using the orientation of the sagittal frame
	J_tip_.changeBase(sagittal_.M.Inverse());
	J_rcm_.changeBase(sagittal_.M.Inverse());
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

	// get frames with respect base link but aligned with the sagittal frame
	KDL::Frame goal=sagittal_.Inverse()*target_*goal_;
	KDL::Frame rcm=sagittal_.Inverse()*tip_*rcm_;
	KDL::Frame target=sagittal_.Inverse()*target_;
	KDL::Frame tip=sagittal_.Inverse()*tip_;

	// compute errors in sagittal coordinates
	KDL::Vector e_tip=goal.p-tip.p;
	KDL::Vector e_rcm=target.p-rcm.p;
	Eigen::Vector3d e_xz;
	e_xz << e_tip[2],e_rcm[0],e_rcm[2];
	Eigen::Vector3d e_xy;
	e_xy << e_tip[1],e_rcm[0],e_rcm[1];

	// build the Jacobian of the XY-task
	//   -- tip constraint along the y-axis
	//   -- RCM constraint along the x-axis
	//   -- RCM constraint along the y-axis
	Eigen::Matrix4d J_xy;
	J_xy <<
		J_tip_(1,0), -tip.p[0], 0.0, 1.0,
		J_rcm_(0,0),  rcm.p[1], 1.0, 0.0,
		J_rcm_(1,0), -rcm.p[0], 0.0, 1.0,
		        1.0,       0.0, 0.0, 0.0;

	// build the Jacobian of the XZ-task
	//   -- tip constraint along the z-axis,
	//   -- RCM constraint along x and z axis.
	// (we assume that q_(0)->"arm_joint_1" and q_(5)->"arm_joint_5")
	Eigen::Matrix3d J_xz;
	J_xz.block<1,3>(0,0)=J_tip_.data.block(2,1,1,3);
	J_xz.block<1,3>(1,0)=J_rcm_.data.block(0,1,1,3);
	J_xz.block<1,3>(2,0)=J_rcm_.data.block(2,1,1,3);

	// compute the pseudo-inverse of the XY-task Jacobian
	Eigen::Matrix4d pJ_xy=pinv(J_xy,control_params_.mu);

	// compute the pseudo-inverse of the XZ-task Jacobian
	Eigen::Matrix3d pJ_xz=pinv(J_xz,control_params_.mu);

	// compute joints velocities
	Eigen::Vector3d qp=pJ_xz*e_xz;
	qp=control_params_.qp*(1-exp(-pow(0.1*e_xz.norm()/control_params_.eps,2)))*qp/qp.norm();

	// get the manipulability measure, which should be proportional to the
	// relative motion between the tip and the base (i.e., shoulder)
	double um=this->manipulability();

	// manipulability maximizing control (in joint velocity units)
	um=sgn(um)*control_params_.qp*(1-exp(-pow(25*um/control_params_.eps,2)));

	// get the self-collision potential field value
	double uc=this->selfcollision();

	// self-collision avoidance control (in joint velocity units)
	uc=control_params_.qp*(1-exp(-pow(uc/25.0,2)));

	// generate the null-space of the task along the z-axis
	Eigen::Matrix<double,2,3> J_z;
	J_z.block<1,3>(0,0)=J_tip_.data.block(2,1,1,3);
	J_z.block<1,3>(1,0)=J_rcm_.data.block(2,1,1,3);
	Eigen::Matrix<double,3,2> pJ_z=pinv(J_z,control_params_.mu);

	// compute the desired velocity profile for task error minimization
	Eigen::Vector3d vo=J_xz*qp;

	// update joints velocities with the manipulability and self-collision terms
	// (projected to the null-space of the task's sub-Jacobian related to the z-axis)
	qp=qp+(um+uc)*(Eigen::Matrix3d::Identity()-pJ_z*J_z)*pJ_xz*Eigen::Vector3d::UnitY();

	// compute the task's velocities after the addition of the control terms
	Eigen::Vector3d vf=J_xz*qp;

	// apply joints velocities saturation
	Eigen::Vector3d ve=Eigen::Vector3d::Zero();
	if( control_params_.qp<qp.lpNorm<Eigen::Infinity>() )
	{
		qp=control_params_.qp*qp/qp.lpNorm<Eigen::Infinity>();

		// recompute the perturbed velocity profile after saturation
		ve=vf-J_xz*qp;
	}

	// compute predicted velocity variation with respect base link frame
	KDL::Vector uv(vo(1)-vf(1)+ve(1),0.0,0.0);
	uv=sagittal_.M*uv;

	// clear previous control commands
	KDL::SetToZero(u_base);
	KDL::SetToZero(u_arm);

	// update arm and base velocity commands, applying to the base the predicted
	// velocity variation of the RCM (or tip) due the redundancy control terms
	// and joints speeds saturation 
	u_arm(1)=qp(0);
	u_arm(2)=qp(1);
	u_arm(3)=qp(2);
	u_base(0)=uv(0);
	u_base(1)=uv(1);

	// send velocity commands
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
