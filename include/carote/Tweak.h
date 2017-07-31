#ifndef _CAROTE_TWEAK_H_
#define _CAROTE_TWEAK_H_

#include<ros/ros.h>
#include<geometry_msgs/PoseArray.h>
#include<tf/tf.h>
#include<tf/transform_listener.h>
#include<tf/transform_broadcaster.h>
//#include "carote/TweakConfig.h"

namespace carote
{
	class Tweak
	{
		public:
			Tweak(const std::string& _name);
			~Tweak(void);
		
		protected:
			// ros stuff: node handle
			std::string name_;
			ros::NodeHandle node_;

			// ros stuff: tweak topic name
			std::string topic_name_;

			// ros stuff: transform between gripper and target frames
			std::string frame_id_gripper_;
			std::string frame_id_target_;
			tf::StampedTransform tf_gripper_target_;
	};
	
	class TweakListener: public Tweak
	{
		public:
			TweakListener(const std::string& _name, int _fd);
			~TweakListener(void);

			void input(const geometry_msgs::PoseArray& _msg);

		protected:
			// ros stuff: topic subscriber
			ros::Subscriber sub_target_;

			// ros stuff: transform between gripper and target frames
			tf::TransformListener tf_listener_;

			// multi-master stuff: pipe descriptor
			int pipe_wr_fd_;
	};

	class TweakPublisher: public Tweak
	{
		public:
			TweakPublisher(const std::string& _name, int _fd);
			~TweakPublisher(void);

			void input(const geometry_msgs::PoseArray& _msg);

		protected:
			// ros stuff: topic publisher
			ros::Publisher pub_target_;

			// multi-master stuff: pipe descriptor
			int pipe_rd_fd_;
	};
/*
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
*/
}

#endif
