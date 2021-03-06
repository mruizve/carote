#ifndef _CAROTE_FOLLOWER_H_
#define _CAROTE_FOLLOWER_H_

#include<deque>
#include<ros/ros.h>
#include<dynamic_reconfigure/server.h>
#include "carote/FollowerConfig.h"
#include "carote/Controller.h"

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
			carote::FollowerConfig control_params_;
			dynamic_reconfigure::Server<carote::FollowerConfig> server_;

			// tip frame
			KDL::Frame tip_;

			// control program
			std::deque<KDL::Twist> u_;
	};
}

#endif
