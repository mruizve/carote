#ifndef _CAROTE_FOLLOWER_H_
#define _CAROTE_FOLLOWER_H_

#include<ros/ros.h>
#include<tf/tf.h>
#include<dynamic_reconfigure/server.h>
#include "carote/FollowerConfig.h"
#include "carote/Controller.h"
#include "carote/Utils.h"

namespace carote
{
	class Follower: public Controller
	{
		public:
			Follower(const std::string& _name);
			~Follower(void);

		protected:
			// callbacks
			void cbControl(const ros::TimerEvent& _event);
			void cbReconfigure(carote::FollowerConfig& _config, uint32_t _level);

			// controller stuff
			void clean(void);

		protected:
			// ros stuff: parameters handling through dynamic reconfigure 
			carote::FollowerConfig params_;
			dynamic_reconfigure::Server<carote::FollowerConfig> server_;

			// output processing
			tf::Vector3 p_; // target position
			tf::Vector3 v_; // target velocity
			std::deque<Eigen::Vector3d> u_; // control: [v_x, v_y, w]
	};
}

#endif
