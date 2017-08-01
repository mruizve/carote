#ifndef _CAROTE_CONTROLLER_H_
#define _CAROTE_CONTROLLER_H_

// topics messages
#include<brics_actuator/JointPositions.h>  // arm control
#include<brics_actuator/JointVelocities.h> // arm control
#include<geometry_msgs/PoseArray.h>        // target {position, orientation}
#include<geometry_msgs/Twist.h>            // platform or base control
#include<geometry_msgs/Vector3.h>          // operator {phi (latitude), lambda (longitude), rho (distance)}
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

// dynamic reconfigure parameters
#include<dynamic_reconfigure/server.h>
#include "carote/ControllerConfig.h"

#include<deque>
#include<Eigen/Core>

#include "carote/Utils.h"

namespace carote
{
	typedef struct
	{
		Eigen::Vector3d u;
		Eigen::Matrix<double,5,1> qp;
	} VelocityControlData;
	
	class Controller
	{
		protected:
			Controller(const std::string& _name);
			~Controller(void);

			// operator, target and control callbacks
			void cbOperator(const geometry_msgs::Vector3& _msg);
			void cbState(const sensor_msgs::JointState& _msg);
			void cbTarget(const geometry_msgs::PoseArray& _msg);
			void cbControl(const ros::TimerEvent& event);

			// move to predefined robot poses
			void moveTo(KDL::JntArray& _q);

			// controller actions
			void start(void);
			void stop(void);
			void update(void);

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

			// ros stuff: control timer
			ros::Timer timer_;

			// ros stuff: reference frames
			std::string frame_id_base_;
			std::string frame_id_gripper_;
			std::string frame_id_target_;
			tf::StampedTransform tf_base_target_;
			tf::TransformListener tf_listener_;

			// input data: operator
			int flag_operator_;
			double phi_;
			double lambda_;
			double rho_;

			// input data: joints states
			int flag_states_;
			KDL::JntArray q_;
			KDL::JntArray qp_;

			// input data: target
			int flag_target_;
			Eigen::Vector3d t_;
			Eigen::Matrix3d R_;

			// output data control law
			std::deque<VelocityControlData> control_queue_;

			// robot model and kinematics
			KDL::Tree kdl_tree_;
			KDL::Chain kdl_chain_;
			urdf::Model model_;

			// joints information
			KDL::JntArray q_lower_;
			KDL::JntArray q_upper_;
			std::vector<int> q_types_;
			std::vector<std::string> q_names_;

			// predefined poses
			KDL::JntArray q_home_;
			KDL::JntArray q_zero_;
	};
	
	class Follower: public Controller
	{
		public:
			Follower(const std::string& _name);
			~Follower(void);

			void stop(void);
			void input(const geometry_msgs::PoseArray& _msg);
			void output(const ros::TimerEvent& event);
			void reconfigure(carote::ControllerConfig &config, uint32_t level);

		protected:
			// ros stuff: parameters handling through dynamic reconfigure 
			carote::ControllerConfig params_;
			dynamic_reconfigure::Server<carote::ControllerConfig> server_;

			// ros stuff: transforms
			tf::StampedTransform tf_;
			tf::TransformListener tf_listener_;

			// output processing
			tf::Vector3 p_; // target position
			tf::Vector3 v_; // target velocity
			std::deque<Eigen::Vector3d> u_; // control: [v_x, v_y, w]
	};
}

#endif
