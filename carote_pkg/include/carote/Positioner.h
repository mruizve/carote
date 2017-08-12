#ifndef _CAROTE_POSITIONER_H_
#define _CAROTE_POSITIONER_H_

#include<deque>
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

		protected:
			// ros stuff: parameters handling through dynamic reconfigure 
			carote::PositionerConfig control_params_;
			dynamic_reconfigure::Server<carote::PositionerConfig> server_;

			// tip and RCM tip frames and Jacobians
			KDL::Frame tip_;
			KDL::Frame rcm_;
			KDL::Jacobian J_tip_;
			KDL::Jacobian J_rcm_;

			// RCM tip's chain Jacobian solver
			KDL::ChainJntToJacSolver *kdl_J_solver_;

			// control program
			std::deque<KDL::JntArray> u_;
	};
}

#endif
