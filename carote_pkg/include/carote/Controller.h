#ifndef _CAROTE_CONTROLLER_H_
#define _CAROTE_CONTROLLER_H_

#include<ros/ros.h>

// messages
#include<carote_msgs/OperatorStamped.h>    // operator {phi (latitude), lambda (longitude), rho (distance)}
#include<geometry_msgs/PoseArray.h>        // target {position, orientation}
#include<brics_actuator/JointPositions.h>  // robot control (arm)
#include<brics_actuator/JointVelocities.h> // robot control (arm)
#include<geometry_msgs/Twist.h>            // robot control (platform or base)
#include<sensor_msgs/JointState.h>         // robot state (arm and base)

// frames transforms
#include<tf/tf.h>
#include<tf/transform_listener.h>

// robot model and kinematics
#include<kdl/frames.hpp>
#include<kdl_parser/kdl_parser.hpp>
#include<kdl/chainfksolverpos_recursive.hpp>
#include<kdl/chainiksolvervel_pinv_givens.hpp>
#include<kdl/chainiksolverpos_nr_jl.hpp>
#include<urdf/model.h>

namespace carote
{
	class Controller
	{
		protected:
			Controller(const std::string& _name);
			~Controller(void);

			// operator, target and control callbacks
			virtual void cbControl(const ros::TimerEvent& _event)=0;
			void cbOperator(const carote_msgs::OperatorStamped& _msg);
			void cbState(const sensor_msgs::JointState& _msg);
			void cbTarget(const geometry_msgs::PoseArray& _msg);

		public:
			// controller actions
			virtual void clean(void)=0;
			void start(ros::Duration _period);
			void stop(void);
			void zero(void);

			// move to a robot pose
			void moveTo(KDL::JntArray& _q);
			void home(void);
			void work(void);

		private:
			// initializations 
			void initROS(void);        // publishers and advertisers
			void initKinematics(void); // urdf and kdl
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
			std::string frame_id_gripper_;
			std::string frame_id_target_;
			tf::StampedTransform tf_base_target_;
			tf::TransformListener tf_listener_;

		private:
			// ros stuff: controller timer
			ros::Timer timer_;

			// input data: operator
			int operator_flag_;
			carote_msgs::Operator operator_data_;
				
			// input data: joints states
			int states_flag_;
			KDL::JntArray q_;
			KDL::JntArray qp_;

			// input data: target
			int target_flag_;
			Eigen::Vector3d target_t_;
			Eigen::Matrix3d target_R_;

			// robot model
			urdf::Model model_;
			KDL::JntArray q_lower_;
			KDL::JntArray q_upper_;
			std::vector<int> q_types_;
			std::vector<std::string> q_names_;

			// kinematics
			KDL::Tree kdl_tree_;
			KDL::Chain kdl_chain_;

			// predefined poses
			KDL::JntArray q_home_; // shutdown pose
			KDL::JntArray q_work_; // initial work pose
	};
}

#endif
