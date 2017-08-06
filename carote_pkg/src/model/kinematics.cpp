#include "carote/Model.h"
#include "carote/Utils.h"

int carote::Model::JntToCart(KDL::JntArray& _q, KDL::Frame& _f)
{
	return kdl_fp_solver_->JntToCart(_q,_f);
}
