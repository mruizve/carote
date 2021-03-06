#include<kdl_parser/kdl_parser.hpp>
#include "carote/Model.h"
#include "carote/Utils.h"

carote::Model::Model(const std::string& _xml, const std::string& _frame_id_base, const std::string& _frame_id_tip)
:
    kdl_J_solver_(NULL),
    kdl_fp_solver_(NULL)
{
	// load URDF model
	urdf::Model urdf_model;
    if( !urdf_model.initString(_xml) )
    {
        CAROTE_NODE_ABORT("could not initialize an URDF model from '/robot_description' parameter");
    }

	// generate KDL tree from URDF model
	KDL::Tree kdl_tree;
	if( !kdl_parser::treeFromString(_xml,kdl_tree) )
	{
        CAROTE_NODE_ABORT("could not initialize a KDL tree from '/robot_description' parameter");
	}

	// extract KDL chain of the arm
	if( !kdl_tree.getChain(_frame_id_base,_frame_id_tip,kdl_chain_) )
	{
		CAROTE_NODE_ABORT("cannot build the kdl chain of the arm");
    }

	// count number of non-fixed joints
	this->countNrOfJoints(urdf_model);

	// resize joints values, velocities and limits arrays
	q_lower_.resize(nJoints_);
	q_upper_.resize(nJoints_);

	// for each segment of the chain,
	for( size_t i=0,j=0; kdl_chain_.getNrOfSegments()>i; i++ )
	{
		KDL::Segment segment=kdl_chain_.getSegment(i);

		// get URDF joint information
		boost::shared_ptr<const urdf::Link> link=urdf_model.getLink(segment.getName());
		if( !link )
		{
			CAROTE_NODE_ABORT("KDL segment '" << segment.getName() << "' without equivalent URDF link!?");
		}
		boost::shared_ptr<const urdf::Joint> joint=urdf_model.getJoint(link->parent_joint->name);
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

			if( urdf::Joint::PRISMATIC==joint->type )
			{
				q_units_.push_back("meters");
				qp_units_.push_back("s^-1 meters");
			}
			else
			{
				q_units_.push_back("rad");
				qp_units_.push_back("s^-1 rad");
			}

			j++;
		}
	}

	// Jacobian solver
	kdl_J_solver_=new KDL::ChainJntToJacSolver(kdl_chain_);

    // forward position solver
    kdl_fp_solver_=new KDL::ChainFkSolverPos_recursive(kdl_chain_);

	// initialize dynamic model
	dyn_B_=Eigen::MatrixXd(nJoints_,nJoints_);
	dyn_S_=Eigen::MatrixXd(nJoints_,nJoints_);
	dyn_g_=Eigen::VectorXd(nJoints_);

	// dump model
    for( int i=0; nJoints_>i; i++ )
    {
		ROS_WARN_STREAM("[arm chain] " << q_names_[i] << ", range=[" << q_lower_(i) << "," << q_upper_(i) << "]");
	}
}

carote::Model::~Model(void)
{
	if( NULL!=kdl_J_solver_ )
	{
		delete kdl_J_solver_;
	}

	if( NULL!=kdl_fp_solver_ )
	{
		delete kdl_fp_solver_;
	}
}
