#ifndef _CAROTE_CONTROLLER_H_
#define _CAROTE_CONTROLLER_H_

#include<ros/ros.h>
#include<carote_msgs/OperatorStamped.h>    // operator {phi (latitude), lambda (longitude), rho (distance)}
#include<geometry_msgs/PoseArray.h>        // target {position, orientation}
#include<sensor_msgs/JointState.h>         // robot state (arm and base)
#include "carote/Model.h"

namespace carote
{
	class Controller
	{
		public:
			Controller(const std::string& _name);
			~Controller(void);

			// predefined robot poses
			void home(void);
			void work(void);

			// stop robot motion
			void zero(void);

		protected:
			// control enable/disable
			void start(ros::Duration _period);
			void stop(void);

			// input data and control callbacks
			virtual void cbControl(const ros::TimerEvent& _event);
			virtual void cbOperator(const carote_msgs::OperatorStamped& _msg);
			virtual void cbState(const sensor_msgs::JointState& _msg);
			virtual void cbTarget(const geometry_msgs::PoseArray& _msg);

			// arm and base control
			void armPose(const KDL::JntArray& _q);
			void armVelocities(const KDL::JntArray& _q);
			void baseTwist(const KDL::Twist& _u);

			// reference to the ros node
			const ros::NodeHandle& node(void) const;

			// reference to the predefined poses
			const KDL::JntArray& getHomePose(void) const;
			const KDL::JntArray& getWorkPose(void) const;

		private:
			// initializations 
			void initROS(void);        // publishers and advertisers
			void initPoses(void);      // predefined poses (zero and home)

			// predefined poses loader
			void loadXMLPose(const std::string _param, KDL::JntArray& _q);

			// goal generator based on operator commands
			void getGoal(void);
		
		private:
			// ros stuff: node handle
			std::string name_;
			ros::NodeHandle node_;

			// ros stuff: topics subscribers/publishers
			ros::Publisher pub_q_;         // joint values
			ros::Publisher pub_qp_;        // joint velocities
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

			// input data
			int states_flag_;   // joints states
			KDL::JntArray q_;
			KDL::JntArray qp_;
			KDL::JntArray qpp_;
			KDL::JntArray tau_n_; // nominal
			KDL::JntArray tau_t_; // measured

			int goal_flag_;     // goal pose and parameters
			int operator_flag_;
			carote_msgs::Operator goal_params_;
			KDL::Frame goal_;

			int target_flag_;   // target pose
			KDL::Frame target_;
	};
}

#endif
