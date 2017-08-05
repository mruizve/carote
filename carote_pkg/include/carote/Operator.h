#ifndef _CAROTE_OPERATOR_H_
#define _CAROTE_OPERATOR_H_

#include<ros/ros.h>
#include<tf/tf.h>
#include<tf/transform_broadcaster.h>
#include<dynamic_reconfigure/server.h>
#include "carote/OperatorConfig.h"

namespace carote
{
	class Operator
	{
		public:
			Operator(const std::string& _name);
			~Operator(void);

			void cbReconfigure(carote::OperatorConfig& _config, uint32_t _level);

		private:
			// ros stuff: node handle
			std::string name_;
			ros::NodeHandle node_;

			// ros stuff: topics
			ros::Publisher pub_command_;

			// ros stuff: parameters handling through dynamic reconfigure 
			dynamic_reconfigure::Server<carote::OperatorConfig> server_;

			// ros stuff: transform between tip and target frame
			std::string frame_id_goal_;
			std::string frame_id_target_;
			tf::Transform tf_target_goal_;
			tf::TransformBroadcaster tf_broadcaster_;
	};
}

#endif
