#ifndef _CAROTE_MODEL_H_
#define _CAROTE_MODEL_H_

#include<ros/ros.h>
#include<urdf/model.h>
#include<kdl/chainfksolverpos_recursive.hpp>
#include<kdl/chainiksolvervel_pinv_givens.hpp>
#include<kdl/chainiksolverpos_nr_jl.hpp>
#include "Dynamics.h"

namespace carote
{
	class Model
	{
		public:
			Model(const std::string& _xml, const std::string& _frame_id_base, const std::string& _frame_id_tip);
			~Model(void);

			// dynamics
			int JntToTau(KDL::JntArray _q, const KDL::JntArray& _qp, const KDL::JntArray& _qpp, KDL::JntArray& _tau);
			const Eigen::MatrixXd& getInertiaMatrix(void) const;
			const Eigen::MatrixXd& getCoriolisCentifugalTerms(void) const;
			const Eigen::VectorXd& getGravityTerms(void) const;

			// geometry
			int getNrOfJoints(void) const;
			int JntToJac(const KDL::JntArray& _q, KDL::Jacobian& _J, const int _segmentNR=-1) const;
			int JntToCart(const KDL::JntArray& _q, KDL::Frame& _f, const int _segmentNR=-1) const;

			// chain
			const KDL::Chain& getChain(void) const;
			int getSegmentIndex(const std::string& _name) const;

			// joints data
			const std::string& getJointName(size_t i) const;
			const std::vector<std::string>& getJointsNames(void) const;

			int getJointType(int i) const;
			const std::vector<int>& getJointsTypes(void) const;

			const std::string& getJointUnit(size_t i) const;
			const std::vector<std::string>& getJointsUnits(void) const;

			const std::string& getSpeedUnit(size_t i) const;
			const std::vector<std::string>& getSpeedsUnits(void) const;

		private:
			void countNrOfJoints(const urdf::Model& urdf_model);

			void updateInertia(const KDL::JntArray& _q);
			void updateCoriolisCentrifugal(const KDL::JntArray& _q, const KDL::JntArray& _qp);
			void updateGravity(const KDL::JntArray& _q);

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
			KDL::ChainJntToJacSolver *kdl_J_solver_;         // Jacobian
			KDL::ChainFkSolverPos_recursive *kdl_fp_solver_; // forward position

			// dynamics model
			Eigen::MatrixXd dyn_B_;
			Eigen::MatrixXd dyn_S_;
			Eigen::VectorXd dyn_g_;
	};
}

#endif
