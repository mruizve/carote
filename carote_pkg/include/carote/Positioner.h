#ifndef _CAROTE_POSITIONER_H_
#define _CAROTE_POSITIONER_H_

#include<Eigen/Core>
#include<ros/ros.h>
#include<dynamic_reconfigure/server.h>
#include "carote/PositionerConfig.h"
#include "carote/Controller.h"

namespace carote
{
	class Positioner: public Controller
	{
		public:
			Positioner(const std::string& _name);
			~Positioner(void);

		protected:
			// callbacks
			void cbControl(const ros::TimerEvent& _event);
			void cbReconfigure(carote::PositionerConfig& _config, uint32_t _level);
			void cbTarget(const geometry_msgs::PoseArray& _msg);

		private:
			// when no feedback is available update robot
			// states and target pose with vanilla odometry
			void updateStates(const KDL::JntArray& _u);
			void updateTarget(const KDL::Twist& _u);

			// update kinematics (robot pose -frames- and Jacobians)
			void updateKinematics(void);

			// compute the maximizing manipulability control law
			double manipulability(void);

			// compute the self collision potential field
			double selfcollision(void);

		protected:
			// ros stuff: parameters handling through dynamic reconfigure 
			carote::PositionerConfig control_params_;
			dynamic_reconfigure::Server<carote::PositionerConfig> server_;

			// ros stuff: frames ids
			std::string frame_id_shoulder_;

			// RCM, tip and wrist frames and Jacobians
			KDL::Frame rcm_;
			KDL::Frame tip_;
			KDL::Frame sagittal_;
			KDL::Frame shoulder_;
			KDL::Jacobian J_rcm_;
			KDL::Jacobian J_tip_;
			KDL::ChainJntToJacSolver *kdl_J_solver_;
	};
}

#endif
