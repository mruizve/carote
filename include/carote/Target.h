#ifndef _CAROTE_TARGET_H_
#define _CAROTE_TARGET_H_

#include<ros/ros.h>
#include<dynamic_reconfigure/server.h>
#include<image_transport/image_transport.h>
#include<opencv2/opencv.hpp>
#include<sensor_msgs/PointCloud2.h>
#include "carote/CalibrationConfig.h"
#include "carote/FilterMean.h"
#include "carote/FilterHoloborodko.h"

namespace carote
{
	class Target
	{
		public:
			Target(std::string _name);
			~Target(void);

			void calibrationDR(carote::CalibrationConfig &config, uint32_t level);
			void calibrationGUI(void);
			void callback(const sensor_msgs::PointCloud2::ConstPtr _msg);

		private:
			// ros stuff: node handle
			std::string name_;
			ros::NodeHandle node_;

			// ros stuff: input topics and frames
			std::string cloud_id_;
			std::string sensor_id_;
			std::string sensor_frame_id_;
			ros::Subscriber cloud_sub_;

			// ros stuff: output topic and frames
			std::string target_position_id_;
			std::string target_velocity_id_;
			std::string target_mask_id_;
			ros::Publisher target_position_pub_;
			ros::Publisher target_velocity_pub_;
			image_transport::ImageTransport target_mask_it_;
			image_transport::Publisher target_mask_pub_;

			// input processing
			size_t contours_threshold_;
			double depth_low_;
			double depth_high_;
			cv::Mat morphology_[2];
			cv::Scalar color_low_;
			cv::Scalar color_high_;

			// input processing + ros stuff
			dynamic_reconfigure::Server<carote::CalibrationConfig> server_;

			// output processing
			MeanFilter *position_filter_;
			HoloborodkoFilter *velocity_filter_;

			// extra stuff: color and depth calibration flag
			int calibration_;
	};
}

#endif
