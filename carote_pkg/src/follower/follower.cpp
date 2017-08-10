#include<geometry_msgs/Twist.h>
#include "carote/Follower.h"
#include "carote/Utils.h"

template<typename T> int sgn(T val)
{
	return (T(0)<val)-(val<T(0));
}

carote::Follower::Follower(const std::string& _name)
:
	Controller(_name)
{
	// ros stuff: dynamic reconfiguration of controller parameters
	dynamic_reconfigure::Server<carote::FollowerConfig>::CallbackType f;
	f=boost::bind(&carote::Follower::cbReconfigure,this,_1,_2);
	server_.setCallback(f);
}

carote::Follower::~Follower(void)
{
}

void carote::Follower::clean(void)
{
	u_.clear();
}

inline void nonlinearRule(std::vector<double>& out, double dx, double vo, double v_lim, double dt, double eps)
{
	// this static control law if fully empirical. the rule resembles a passive
	// electrical circuit charge and discharge phases, plus an additional
	// non-linear modulation of the steady state control magnitude in terms of
	// the error:
	//
	//           u(t)=uo+(gamma(e(t))*u_max-uo)*(1-exp(-alpha*t))
	//
	// where uo is the initial condition, gamma a symmetric, positive definite,
	// monotonically increasing, nonlinear function with gamma(0)=0, e(t) is the
	// error, u_max the maximum allowed control magnitude and alpha, the
	// equivalent of the RC time constant, represents the temporal smoothing
	// factor and should be -in general- a function of the control rate.

	double alpha=0.0921/dt;
	double v=0.0;
	double tau=0.0;
	double error=0.0;
	double gamma=0.0;
	double psi=fabs(error-dx);
	for( ; psi>eps && 5.0>tau; )
	{
		gamma=(1.0-exp(-0.5*psi*psi/(100*eps*eps)));
		v=vo+(sgn(dx)*gamma*v_lim-vo)*(1.0-exp(-alpha*tau));
		error+=v*dt;
		tau+=dt;
		psi=fabs(error-dx);
		out.push_back(v);
	}
}

void carote::Follower::cbControl(const ros::TimerEvent& _event)
{

	KDL::Twist u(KDL::Vector(0.0,0.0,0.0),KDL::Vector(0.0,0.0,0.0));

	// if there is a control program defined, then
	if( 0<u_.size() )
	{
		// get the next control command
		u=u_.front();
		u_.pop_front();
	}

	// send control command (if any) to the robot
	this->baseTwist(u);

	// if a new target frame has been obtained, then we have
	// to re-plan the control program based on this new feedback
	#define CP control_params_
	if( target_flag_ && goal_flag_ )
	{
		// clear the current control program
		u_.clear();

		// get tip frame
		KDL::Frame tip;
		if( 0>model_->JntToCart(q_,tip) )
		{
			CAROTE_NODE_ABORT("cbControl(): unexpected error from forward kinematics");
		}

		// coordinate axis
		KDL::Vector x(1.0,0.0,0.0);
		KDL::Vector y(0.0,1.0,0.0);
		KDL::Vector z(0.0,0.0,1.0);


		// get goal in base link coordinates
		KDL::Frame goal=target_*goal_;

		// compute errors
		KDL::Vector r=goal.p-tip.p;
		KDL::Vector Z_tip(tip.M.UnitZ()[0],tip.M.UnitZ()[1],0.0);
		KDL::Vector Z_goal(goal.M.UnitZ()[0],goal.M.UnitZ()[1],0.0);
		double ew=acos(KDL::dot(Z_tip,Z_goal)/(Z_tip.Norm()*Z_goal.Norm()));
		double sw=(0.0<KDL::dot(Z_tip*Z_goal,z))?1.0:-1.0;

		// compute feed-forward compensation
		double dt=1.0/CP.rate;
		std::vector<double> xp;
		std::vector<double> yp;
		std::vector<double> w;
		nonlinearRule(xp,r[0],u(0),CP.v,dt,CP.cartesian_error);
		nonlinearRule(yp,r[1],u(1),CP.v,dt,CP.cartesian_error);
		nonlinearRule(w,ew*sw,u(5),CP.w,dt,CP.angular_error);

		// merge trajectories into a single twist command
		const size_t length=std::max(xp.size(),std::max(yp.size(),w.size()));
		double theta=0;
		for( size_t i=0; length>i; i++ )
		{
			KDL::SetToZero(u);

			// compute speed
			if( xp.size()>i )
			{
				u(0)+= cos(theta)*xp[i];
				u(1)+=-sin(theta)*xp[i];
			}
			if( yp.size()>i )
			{
				u(0)+= sin(theta)*yp[i];
				u(1)+= cos(theta)*yp[i];
			}

			if( w.size()>i )
			{
				u(5)=w[i];
				theta+=dt*w[i];
			}
			u_.push_back(u);
		}

		// clear the target flag
		target_flag_=0;
	}
	#undef CP
}

void carote::Follower::cbReconfigure(carote::FollowerConfig& _config, uint32_t _level)
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
		this->stop();
	}
}
