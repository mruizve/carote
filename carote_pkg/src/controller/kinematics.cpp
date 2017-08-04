#include "carote/Controller.h"
#include "carote/Utils.h"

void carote::Controller::initKinematics(void)
{
	std::string strpar;

	// this should be part of the constructor initializations
    kdl_fp_solver_=NULL;
    kdl_iv_solver_=NULL;
	kdl_ip_solver_=NULL;

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

	// count number of non-fixed joints
	njoints_=0;
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

		if( urdf::Joint::FIXED!=joint->type )
		{
			njoints_++;
		}
	}

	// resize joints values, velocities and limits arrays
	q_.resize(njoints_);
	qp_.resize(njoints_);
	q_lower_.resize(njoints_);
	q_upper_.resize(njoints_);

	// for each segment of the chain,
    for( int i=0,j=0; kdl_chain_.getNrOfSegments()>i; i++ )
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
		if( urdf::Joint::FIXED!=joint->type )
		{
			if( !joint->limits )
			{
				CAROTE_NODE_ABORT("don't know how to handle joints without limits");
			}
			q_lower_(j)=joint->limits->lower;
			q_upper_(j)=joint->limits->upper;

			q_types_.push_back(joint->type);
			q_names_.push_back(joint->name);

			j++;
		}
	}

    // forward position and inverse velocity solvers
    // (needed by the geometric -inverse position- solver)
    kdl_fp_solver_=new KDL::ChainFkSolverPos_recursive(kdl_chain_);
    kdl_iv_solver_=new KDL::ChainIkSolverVel_pinv_givens(kdl_chain_);
    
    // geometric solver definition (with joint limits)
	kdl_ip_solver_=new KDL::ChainIkSolverPos_NR_JL(kdl_chain_,q_lower_,q_upper_,*kdl_fp_solver_,*kdl_iv_solver_,500,1e-6);
}

void carote::Controller::cleanupKinematics(void)
{
	if( NULL!=kdl_fp_solver_ )
	{
		delete kdl_fp_solver_;
	}

	if( NULL!=kdl_iv_solver_)
	{
		delete kdl_iv_solver_;
	}

	if( NULL!=kdl_ip_solver_ )
	{
		delete kdl_ip_solver_;
	}
}
