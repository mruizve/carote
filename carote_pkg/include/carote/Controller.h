#ifndef _CAROTE_CONTROLLER_H_
#define _CAROTE_CONTROLLER_H_

#include<ros/ros.h>
#include<carote_msgs/OperatorStamped.h>    // operator {phi (latitude), lambda (longitude), rho (distance)}
#include<geometry_msgs/PoseArray.h>        // target {position, orientation}
#include<brics_actuator/JointPositions.h>  // robot control (arm)
#include<brics_actuator/JointVelocities.h> // robot control (arm)
#include<geometry_msgs/Twist.h>            // robot control (platform or base)
#include<sensor_msgs/JointState.h>         // robot state (arm and base)
#include "carote/Model.h"

namespace carote
{
	class Controller
	{
		public:
			// predefined robot poses
			void home(void);
			void work(void);

			// stop robot motion
			void zero(void);

			// control enable/disable
			void start(ros::Duration _period);
			void stop(void);

		protected:
			Controller(const std::string& _name);
			~Controller(void);

			// control callbacks
			virtual void cbControl(const ros::TimerEvent& _event)=0;

			// erase any pending control command
			virtual void clean(void)=0;

			// arm and base control
			void armPose(const KDL::JntArray& _q);
			void armSpeed(const KDL::JntArray& _q);
			void baseTwist(const KDL::Twist& _u);

		private:
			// input data callbacks
			void cbOperator(const carote_msgs::OperatorStamped& _msg);
			void cbState(const sensor_msgs::JointState& _msg);
			void cbTarget(const geometry_msgs::PoseArray& _msg);

			// initializations 
			void initROS(void);        // publishers and advertisers
			void initPoses(void);      // predefined poses (zero and home)

			// predefined poses loader
			void loadXMLPose(const std::string _param, KDL::JntArray& _q);

		private:
			// ros stuff: node handle
			std::string name_;
			ros::NodeHandle node_;

			// ros stuff: topics subscribers/publishers
			ros::Publisher pub_q_;         // joint values
			ros::Publisher pub_qp_;        // joints velocities
			ros::Publisher pub_u_;         // base twist
			ros::Subscriber sub_operator_;
			ros::Subscriber sub_state_;
			ros::Subscriber sub_target_;

			// ros stuff: controller timer
			ros::Timer timer_;

			// predefined poses
			KDL::JntArray q_home_; // shutdown pose
			KDL::JntArray q_work_; // initial work pose

		protected:
			// ros stuff: reference frames
			std::string frame_id_base_;
			std::string frame_id_goal_;
			std::string frame_id_target_;
			std::string frame_id_tip_;

			// robot model
			carote::Model *model_;

			// input/output data
			int states_flag_;   // joints states
			KDL::JntArray q_;
			KDL::JntArray qp_;
			KDL::JntArray tau_;

			int goal_flag_;     // goal pose
			int operator_flag_; // goal pose parameters
			carote_msgs::Operator goal_params_;
			KDL::Frame goal_;

			int target_flag_;   // target pose
			KDL::Frame target_;

			KDL::Twist u_;      // base twist
	};
}

#endif
