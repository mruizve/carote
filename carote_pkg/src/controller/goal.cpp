#include "carote/Controller.h"
#include "carote/Utils.h"

void carote::Controller::getGoal(void)
{
	// compute the desired position in virtual target coordinates
	KDL::Vector t;
	t(0)=goal_params_.rho*cos(goal_params_.phi*M_PI/180.0)*cos(goal_params_.lambda*M_PI/180.0);
	t(1)=goal_params_.rho*cos(goal_params_.phi*M_PI/180.0)*sin(goal_params_.lambda*M_PI/180.0);
	t(2)=goal_params_.rho*sin(goal_params_.phi*M_PI/180.0);

	// define the virtual target frame {z_target,cross(z_world,z_target),z_world}
	// (we assume that z_target is properly normalized)
	KDL::Vector x(target_.M.UnitZ()); x.Normalize();
	KDL::Vector z(0.0,0.0,1.0);
	KDL::Vector y=z*x; y.Normalize();
	KDL::Rotation R(x,y,z);

	// computed the displacement vector in base coordinates
	t=R*t;

	// compute the goal frame
	z=-t; z.Normalize();
	y=R.UnitZ()*z; y.Normalize();
	x=y*z; z.Normalize();
	R.UnitX(x);
	R.UnitY(y);
	R.UnitZ(z);

	// generate the goal frame in target coordinates
	goal_=KDL::Frame(R,t+target_.p); // base coordinates
	goal_=target_.Inverse()*goal_;   // target coordinates

	// clear/set goal flags
	goal_flag_=1;
	operator_flag_=0;
}
