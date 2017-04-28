#ifndef _CAROTE_TWEAK_H_
#define _CAROTE_TWEAK_H_

#include<ros/ros.h>
#include<dynamic_reconfigure/server.h>
#include<nav_msgs/Odometry.h>
#include "carote/TweakConfig.h"

namespace carote
{
	class Tweak
	{
		public:
			Tweak(const std::string& _name);
			~Tweak(void);

			void input(const nav_msgs::Odometry& _msg);
			void output(const ros::TimerEvent& event);
			void reconfigure(carote::TweakConfig& config, uint32_t level);

		protected:
			int publisher(void);
			int subscriber(void);
			void shutdown(void);

		protected:
			// ros stuff: node handle
			ros::NodeHandle node_;

			// ros stuff: input/output topics
			ros::Publisher output_state_pub_;
			ros::Subscriber input_state_sub_;

			// ros stuff: parameters handling through dynamic reconfigure
			carote::TweakConfig params_;
			dynamic_reconfigure::Server<carote::TweakConfig> server_;

			// tweak stuff
			int fd_;
			int sd_;
			ros::Timer timer_;
	};
}

#undef CAROTE_TWEAK_RD
#undef CAROTE_TWEAK_WR

#endif
