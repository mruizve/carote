#ifndef _CAROTE_FOLLOWER_H_
#define _CAROTE_FOLLOWER_H_

#include<Eigen/Core>
#include<dynamic_reconfigure/server.h>
#include<nav_msgs/Odometry.h>
#include "carote/FollowerConfig.h"

namespace carote
{
	class Follower
	{
		public:
			Follower(const std::string& _name);
			~Follower(void);

			void stop(void);
			void input(const nav_msgs::Odometry& _msg);
			void output(const ros::TimerEvent& event);
			void reconfigure(carote::FollowerConfig &config, uint32_t level);

		private:
			// ros stuff: node handle
			std::string name_;
			ros::NodeHandle node_;

			// ros stuff: input topics and frames
			ros::Subscriber input_state_sub_;

			// ros stuff: output topic and frames
			ros::Timer timer_;
			ros::Publisher output_control_pub_;

			// ros stuff: parameters handling through dynamic reconfigure 
			carote::FollowerConfig params_;
			dynamic_reconfigure::Server<carote::FollowerConfig> server_;

			// output processing
			Eigen::Vector3d p_; // target position
			Eigen::Vector3d v_; // target velocity
			Eigen::Vector3d u_; // control: [v_x, v_y, w]
	};
}

#endif
