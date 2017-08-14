#include "carote/Model.h"
#include "carote/Utils.h"

int carote::Model::JntToCart(KDL::JntArray& _q, KDL::Frame& _f) const
{
	return kdl_fp_solver_->JntToCart(_q,_f);
}

int carote::Model::JntToJac(KDL::JntArray& _q, KDL::Jacobian& _J) const
{
	return kdl_J_solver_->JntToJac(_q,_J);
}
