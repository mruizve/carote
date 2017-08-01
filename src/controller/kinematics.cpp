#include<tf_conversions/tf_eigen.h>
#include "carote/Controller.h"

void carote::Controller::initKinematics(void)
{
	std::string strpar;

	// get the robot urdf model
	if( !node_.getParam("/robot_description",strpar) )
	{
		CAROTE_NODE_ABORT("missing param '/robot_description' (launch robot drivers before controller)");
	}

	// load URDF model
    if( !model_.initString(strpar) )
    {
        CAROTE_NODE_ABORT("could not initialize an URDF model from '/robot_description' parameter");
    }

	// generate KDL tree from URDF model
	if( !kdl_parser::treeFromString(strpar,kdl_tree_) )
	{
        CAROTE_NODE_ABORT("could not initialize a KDL tree from '/robot_description' parameter");
	}

	// extract KDL chain of the arm
	if( !kdl_tree_.getChain(frame_id_base_.substr(1),frame_id_gripper_.substr(1),kdl_chain_) )
	{
		CAROTE_NODE_ABORT("cannot build the kdl chain of the arm");
    }

	// resize joints values, velocities and limits arrays
	q_.resize(kdl_chain_.getNrOfSegments());
	qp_.resize(kdl_chain_.getNrOfSegments());
	q_lower_.resize(kdl_chain_.getNrOfSegments());
	q_upper_.resize(kdl_chain_.getNrOfSegments());

	// for each segment of the chain,
    for( int i=0; kdl_chain_.getNrOfSegments()>i; i++ )
    {
		KDL::Segment segment=kdl_chain_.getSegment(i);

		// get URDF joint information
		boost::shared_ptr<const urdf::Link> link=model_.getLink(segment.getName());
		if( !link )
		{
			CAROTE_NODE_ABORT("KDL segment '" << segment.getName() << "' without equivalent URDF link!?");
		}
		boost::shared_ptr<const urdf::Joint> joint=model_.getJoint(link->parent_joint->name);
		if( !joint )
		{
			CAROTE_NODE_ABORT("KDL segment '" << segment.getName() << "' without equivalent URDF parent_joint!?");
		}

		// extract limits, types and names
		q_lower_(i)=(joint->limits)?joint->limits->lower:0.0;
		q_upper_(i)=(joint->limits)?joint->limits->upper:0.0;
		q_types_.push_back(joint->type);
		q_names_.push_back(joint->name);
	}

    // forward position and inverse velocity solvers
    // (needed by the geometric -inverse position- solver)
    KDL::ChainFkSolverPos_recursive kdl_fp_solver(kdl_chain_);
    KDL::ChainIkSolverVel_pinv_givens kdl_iv_solver(kdl_chain_);
    
    // geometric solver definition (with joint limits)
	KDL::ChainIkSolverPos_NR_JL kdl_ip_solver(kdl_chain_,q_lower_,q_upper_,kdl_fp_solver,kdl_iv_solver,500,1e-6);
}
