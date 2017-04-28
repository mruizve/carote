#ifndef _CAROTE_TARGET_H_
#define _CAROTE_TARGET_H_

#include<ros/ros.h>
#include<dynamic_reconfigure/server.h>
#include<image_transport/image_transport.h>
#include<opencv2/opencv.hpp>
#include<sensor_msgs/PointCloud2.h>
#include "carote/TargetConfig.h"
#include "carote/FilterMean.h"
#include "carote/FilterHoloborodko.h"

namespace carote
{
	class Target
	{
		public:
			Target(const std::string& _name);
			~Target(void);

			void callback(const sensor_msgs::PointCloud2::ConstPtr _msg);
			void reconfigure(carote::TargetConfig& config, uint32_t level);

		private:
			// ros stuff: node handle
			std::string name_;
			ros::NodeHandle node_;

			// ros stuff: input topics and frames
			ros::Subscriber input_cloud_sub_;

			// ros stuff: output topic and frames
			ros::Publisher output_state_pub_;
			image_transport::ImageTransport output_image_it_;
			image_transport::Publisher output_image_pub_;

			// ros stuff: node parameters
			cv::Mat morphology_[2];
			cv::Scalar color_low_;
			cv::Scalar color_high_;

			// ros stuff: parameters handling through dynamic reconfigure 
			carote::TargetConfig params_;
			dynamic_reconfigure::Server<carote::TargetConfig> server_;

			// output processing
			MeanFilter *position_filter_;
			HoloborodkoFilter *velocity_filter_;
	};
}

#endif
