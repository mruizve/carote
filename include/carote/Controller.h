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
#include<kdl_parser/kdl_parser.hpp>
#include<urdf/model.h>

// dynamic reconfigure parameters
#include<dynamic_reconfigure/server.h>
#include "carote/ControllerConfig.h"

#include<deque>
#include<Eigen/Core>

namespace carote
{
	typedef struct
	{
		Eigen::Vector3d u;
		Eigen::Matrix<double,5,1> qp;
	} VelocityControlData;
	
	class Controller
	{
		public:
			Controller(const std::string& _name);
			~Controller(void);

			// operator, target and control callbacks
			void cbOperator(const geometry_msgs::Vector3& _msg);
			void cbState(const sensor_msgs::JointState& _msg);
			void cbTarget(const geometry_msgs::PoseArray& _msg);
			void cbControl(const ros::TimerEvent& event);

			void init(void);

			void start(void);
			void stop(void);
			void update(void){};

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

			// input data: joint states
			Eigen::Matrix<double,5,1> q_;
			Eigen::Matrix<double,5,1> qp_;

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
