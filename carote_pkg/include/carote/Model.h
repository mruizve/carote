#ifndef _CAROTE_MODEL_H_
#define _CAROTE_MODEL_H_

#include<ros/ros.h>
#include<urdf/model.h>
#include<kdl/chainfksolverpos_recursive.hpp>
#include<kdl/chainiksolvervel_pinv_givens.hpp>
#include<kdl/chainiksolverpos_nr_jl.hpp>

namespace carote
{
	class Model
	{
		public:
			Model(const std::string& _xml, const std::string& _frame_id_base, const std::string& _frame_id_tip);
			~Model(void);

			// geometry
			int getNrOfJoints(void) const;
			int JntToJac(KDL::JntArray& _q, KDL::Jacobian& _J) const;
			int JntToCart(KDL::JntArray& _q, KDL::Frame& _f) const;

			// chain
			const KDL::Chain& getChain(void) const;

			// joints data
			const std::string& getJointName(int i) const;
			const std::vector<std::string>& getJointsNames(void) const;

			int getJointType(int i) const;
			const std::vector<int>& getJointsTypes(void) const;

			const std::string& getJointUnit(int i) const;
			const std::vector<std::string>& getJointsUnits(void) const;

			const std::string& getSpeedUnit(int i) const;
			const std::vector<std::string>& getSpeedsUnits(void) const;

		private:
			void countNrOfJoints(urdf::Model& urdf_model);

		protected:
			// joints data
			int nJoints_;
			KDL::JntArray q_lower_;
			KDL::JntArray q_upper_;
			std::vector<int> q_types_;
			std::vector<std::string> q_names_;
			std::vector<std::string> q_units_;
			std::vector<std::string> qp_units_;

			// model representation
			KDL::Chain kdl_chain_;

			// kinematics solvers
			KDL::ChainJntToJacSolver *kdl_J_solver_;           // Jacobian
			KDL::ChainFkSolverPos_recursive *kdl_fp_solver_;   // forward position
			KDL::ChainIkSolverVel_pinv_givens *kdl_iv_solver_; // inverse velocity
			KDL::ChainIkSolverPos_NR_JL *kdl_ip_solver_;       // inverse position
	};
}

#endif
