#ifndef _CAROTE_OPERATOR_H_
#define _CAROTE_OPERATOR_H_

#include<ros/ros.h>
#include<dynamic_reconfigure/server.h>
#include "carote/OperatorConfig.h"

namespace carote
{
	class Operator
	{
		public:
			Operator(const std::string& _name);
			~Operator(void);

			void cbAuto(const ros::TimerEvent& _event);
			void cbReconfigure(carote::OperatorConfig& _config, uint32_t _level);

		private:
			// ros stuff: node handle
			std::string name_;
			ros::NodeHandle node_;

			// ros stuff: timer
			ros::Timer timer_;

			// ros stuff: topics
			ros::Publisher pub_command_;

			// ros stuff: parameters handling through dynamic reconfigure 
			dynamic_reconfigure::Server<carote::OperatorConfig> server_;
			carote::OperatorConfig config_;

			// sample time for the automatic target generation
			ros::Duration period_;
	};
}

#endif
