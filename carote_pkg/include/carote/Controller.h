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

			// control enable/disable
			void start(ros::Duration _period);
			void stop(void);

		protected:
			Controller(const std::string& _name);
			~Controller(void);

			// operator, target and control callbacks
			virtual void cbControl(const ros::TimerEvent& _event)=0;
			void cbOperator(const carote_msgs::OperatorStamped& _msg);
			void cbState(const sensor_msgs::JointState& _msg);
			void cbTarget(const geometry_msgs::PoseArray& _msg);

			// erase any pending control command
			virtual void clean(void)=0;

			// stop robot motion
			void zero(void);

			// move the arm to a given pose
			void armPose(KDL::JntArray& _q);
			void armVelocity(KDL::JntArray& _q);
			void baseTwist(KDL::Twist& _u);

		private:
			// initializations 
			void initROS(void);        // publishers and advertisers
			void initPoses(void);      // predefined poses (zero and home)

			// predefined poses loader
			void loadXMLPose(const std::string _param, KDL::JntArray& _q);

		protected:
			// ros stuff: node handle
			std::string name_;
			ros::NodeHandle node_;

			// ros stuff: topics subscribers/publishers
			ros::Publisher pub_arm_pos_;
			ros::Publisher pub_arm_vel_;
			ros::Publisher pub_base_vel_;
			ros::Subscriber sub_operator_;
			ros::Subscriber sub_state_;
			ros::Subscriber sub_target_;

			// ros stuff: reference frames
			std::string frame_id_base_;
			std::string frame_id_tip_;
			std::string frame_id_target_;

		private:
			// ros stuff: controller timer
			ros::Timer timer_;

			// robot model
			carote::Model *model_;

			// input data: operator
			int operator_flag_;
			carote_msgs::Operator operator_data_;
				
			// input/output data: joints states, target frame, twist commands
			int states_flag_;
			KDL::JntArray q_;
			KDL::JntArray qp_;

			int target_flag_;
			KDL::Frame target_;

			KDL::Twist u_;

			// predefined poses
			KDL::JntArray q_home_; // shutdown pose
			KDL::JntArray q_work_; // initial work pose
	};
}

#endif
