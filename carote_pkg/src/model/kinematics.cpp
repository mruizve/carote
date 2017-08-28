#include "carote/Model.h"
#include "carote/Utils.h"

int carote::Model::JntToCart(const KDL::JntArray& _q, KDL::Frame& _f, const int _segmentNR) const
{
	return kdl_fp_solver_->JntToCart(_q,_f,_segmentNR);
}

int carote::Model::JntToJac(const KDL::JntArray& _q, KDL::Jacobian& _J, const int _segmentNR) const
{
	return kdl_J_solver_->JntToJac(_q,_J,_segmentNR);
}
