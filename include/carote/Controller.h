#ifndef _CAROTE_FOLLOWER_H_
#define _CAROTE_FOLLOWER_H_

#include<deque>
#include<Eigen/Core>
#include<dynamic_reconfigure/server.h>
#include<geometry_msgs/PoseArray.h>
#include<tf/tf.h>
#include<tf/transform_listener.h>
#include "carote/FollowerConfig.h"

#include<kdl_parser/kdl_parser.hpp>

namespace carote
{
	class Controller
	{
		public:
			Controller(const std::string& _name);
			~Controller(void);

		protected:
			// ros stuff: node handle
			std::string name_;
			ros::NodeHandle node_;

	};
	
	class Observer: public Controller
	{
		public:
			Observer(const std::string& _name);
			~Observer(void);

			void input(const geometry_msgs::PoseArray& _msg);
	};

	class Actuator: public Controller
	{
		public:
			Actuator(const std::string& _name);
			~Actuator(void);

			void stop(void);
			void input(const geometry_msgs::PoseArray& _msg);
			void output(const ros::TimerEvent& event);
			void reconfigure(carote::FollowerConfig &config, uint32_t level);
	};

	
	class Follower
	{
		public:
			Follower(const std::string& _name);
			~Follower(void);

			void stop(void);
			void input(const geometry_msgs::PoseArray& _msg);
			void output(const ros::TimerEvent& event);
			void reconfigure(carote::FollowerConfig &config, uint32_t level);

		private:
			// ros stuff: node handle
			std::string name_;
			ros::NodeHandle node_;

			// ros stuff: input data
			ros::Subscriber operator_sub_;
			ros::Subscriber target_sub_;

			// ros stuff: output data
			ros::Timer timer_;
			ros::Publisher control_pub_;

			// ros stuff: frames names
			std::string frame_target_;
			std::string frame_reference_;

			// ros stuff: parameters handling through dynamic reconfigure 
			carote::FollowerConfig params_;
			dynamic_reconfigure::Server<carote::FollowerConfig> server_;

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
