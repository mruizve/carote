#ifndef _CAROTE_TWEAK_H_
#define _CAROTE_TWEAK_H_

#include<ros/ros.h>
#include<carote_msgs/OperatorStamped.h>
#include<geometry_msgs/PoseArray.h>
#include<tf/tf.h>
#include<tf/transform_listener.h>
//#include "carote/TweakConfig.h"

namespace carote
{
	class Tweak
	{
		public:
			Tweak(int &_argc, char **_argv);
			virtual ~Tweak(void)=0;
		
		protected:
			// ros stuff: node
			std::string name_;

			// ros stuff: topic name, publisher and subscriber
			std::string topic_name_;
			ros::Publisher publisher_;
			ros::Subscriber subscriber_;

			// multi-master stuff
			pid_t pid_;
			int pipe_fd_[2];
	};

	class TweakOperator: public Tweak
	{
		public:
			TweakOperator(int &_argc, char **_argv);
			~TweakOperator(void);

		protected:
			void listen(const carote_msgs::OperatorStamped& _msg);
			void publish(void);

		protected:
			// ros stuff: node handle
			ros::NodeHandle node_;
	};
	
	class TweakTarget: public Tweak
	{
		public:
			TweakTarget(int &_argc, char **_argv);
			~TweakTarget(void);

		protected:
			void listen(const geometry_msgs::PoseArray& _msg);
			void publish(void);

		protected:
			// ros stuff: node handle
			ros::NodeHandle node_;

			// ros stuff: transform between gripper and target frames
			std::string frame_id_gripper_;
			std::string frame_id_target_;
			tf::TransformListener tf_listener_;
			tf::StampedTransform tf_gripper_target_;
	};
}

#endif
