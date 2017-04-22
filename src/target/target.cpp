#include<Eigen/Dense>
#include<cv_bridge/cv_bridge.h>
#include<geometry_msgs/PointStamped.h>
#include<geometry_msgs/Vector3Stamped.h>
#include<pcl_conversions/pcl_conversions.h>
#include "carote/Params.h"
#include "carote/Target.h"

carote::Target::Target(std::string _name)
:
	node_("~"),
	name_(_name),
	output_image_it_(node_),
	position_filter_(NULL),
	velocity_filter_(NULL)
{
	// ros stuff: dynamic reconfiguration
	dynamic_reconfigure::Server<carote::TargetConfig>::CallbackType f;
	f=boost::bind(&carote::Target::reconfigure,this,_1, _2);
	server_.setCallback(f);
}

carote::Target::~Target(void)
{
	if( position_filter_ )
	{
		delete position_filter_;
		position_filter_=NULL;
	}

	if( velocity_filter_ )
	{
		delete velocity_filter_;
		velocity_filter_=NULL;
	}
}

void carote::Target::reconfigure(carote::TargetConfig &config, uint32_t level)
{
	// update parameters
	params_=config;

	// input processing stuff: ranges for the HSV color space
	color_low_=cv::Scalar(params_.H_low,params_.S_low,params_.V_low);
	color_high_=cv::Scalar(params_.H_high,params_.S_high,params_.V_high);

	// input processing stuff: morphology kernels for mask filtering
	cv::Size ksize;
	cv::Point anchor=cv::Point(-1,-1);
	ksize=cv::Size(params_.morph_open,params_.morph_open);
	morphology_[0]=cv::getStructuringElement(cv::MORPH_ELLIPSE,ksize,anchor);
	ksize=cv::Size(params_.morph_close,params_.morph_close);
	morphology_[1]=cv::getStructuringElement(cv::MORPH_ELLIPSE,ksize,anchor);

	// output processing stuff: position and velocity filter
	if( position_filter_ )
	{
		delete position_filter_;
		position_filter_=NULL;
	}
	position_filter_=new MeanFilter(params_.position_order);

	if( velocity_filter_ )
	{
		delete velocity_filter_;
		velocity_filter_=NULL;
	}
	velocity_filter_=new HoloborodkoFilter(params_.velocity_order);

	// input topics names
	params_.input_cloud=params_.input_sensor+params_.input_cloud;

	// output topics names
	params_.output_position=name_+params_.output_position;
	params_.output_velocity=name_+params_.output_velocity;
	params_.output_image=name_+params_.output_image;

	// prepare for listening to input topics
	input_cloud_sub_=node_.subscribe(params_.input_cloud,1,&carote::Target::callback,this);

	// prepare for advertise to the output topics
	output_position_pub_=node_.advertise<geometry_msgs::PointStamped>(params_.output_position,1);
	output_velocity_pub_=node_.advertise<geometry_msgs::Vector3Stamped>(params_.output_velocity,1);
	output_image_pub_=output_image_it_.advertise(params_.output_image,1);
}

void carote::Target::callback(const sensor_msgs::PointCloud2::ConstPtr _input)
{
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    pcl::fromROSMsg(*_input,cloud);

	// generate color and depth images from the point cloud
	cv::Mat image(cloud.height,cloud.width,CV_8UC3);
	cv::Mat depth(cloud.height,cloud.width,CV_8UC1);

    for( size_t i=0; cloud.width>i; i++)
    {
		for( size_t j=0; cloud.height>j; j++)
		{
			pcl::PointXYZRGB *p=&cloud[cloud.width*j+i];

			// generate color image
			image.at<cv::Vec3b>(j,i)=cv::Vec3b(p->b,p->g,p->r);

			// generate depth mask image
			float d=sqrtf(p->x*p->x+p->y*p->y+p->z*p->z);
			depth.at<char>(j,i)=(isnan(d) || d<params_.depth_low || d>params_.depth_high)?0:255;
		}
	}

	// keep a copy of the original image
	cv::Mat original;
	if( params_.calibrate )
	{
		image.copyTo(original);
	}

	// color image filtering and conversion to HSV color space
	cv::GaussianBlur(image,image,cv::Size(5,5),0.5);
	cvtColor(image,image,CV_BGR2HSV);

	// color mask generation
	cv::inRange(image,color_low_,color_high_,image);

	// target mask generation
	cv::Mat mask;
	cv::bitwise_and(image,depth,mask);

	//reduce mask noise by some morphological transformations
	cv::morphologyEx(mask,mask,cv::MORPH_OPEN,morphology_[0]);
	cv::morphologyEx(mask,mask,cv::MORPH_CLOSE,morphology_[1]);

	//contour finding
	cv::Mat edges;
	std::vector<cv::Vec4i> hierarchy;
	std::vector<std::vector<cv::Point> > contours;
	cv::Canny(mask,edges,2.0,100.0,3);
	cv::findContours(mask,contours,hierarchy,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE);

	// if at least one contour is found, try to generate target outline
	if( contours.size() )
	{
		// merge contours to define the target outline
		std::vector<cv::Point> outline;
		for( size_t i=0; contours.size()>i; i++ )
		{
			// merge only contours with sufficient number of points
			if( contours[i].size()>params_.contour_threshold )
			{
				outline.insert(outline.end(),contours[i].begin(),contours[i].end());
			}
		}

		// if the outline contains enough number of points, then
		if( outline.size()>params_.contour_threshold )
		{
			// compute the target's convex hull
			std::vector< std::vector<cv::Point> > hull(1);
			cv::convexHull(cv::Mat(outline),hull[0]);

			// compute moments of the convex hull
			cv::Moments moments=cv::moments(hull[0]);

			// compute centroid coordinates of the convex hull
			cv::Point center=cv::Point(moments.m10/moments.m00,moments.m01/moments.m00);

			// validate target's position
			pcl::PointXYZRGB *p=&cloud[cloud.width*center.y+center.x];
			if( !isnan(p->x+p->y+p->z) )
			{
				// filter target's position
				Eigen::Vector3d position;
				position << p->x,p->y,p->z;
				position=position_filter_->apply(position);

				if( position_filter_->ok() )
				{
					// compute target's velocity
					Eigen::Vector3d velocity=velocity_filter_->apply(position,_input->header.stamp);

					if( velocity_filter_->ok() )
					{
						// target's position message
						geometry_msgs::PointStamped position_msg;
						position_msg.header.stamp=_input->header.stamp;
						position_msg.header.frame_id=params_.input_frame;
						position_msg.point.x=position(0);
						position_msg.point.y=position(1);
						position_msg.point.z=position(2);

						// target's linear velocity message
						if( 0.01>velocity.norm())
						{
							velocity=Eigen::Vector3d::Zero();
						}
						geometry_msgs::Vector3Stamped velocity_msg;
						velocity_msg.header.stamp=_input->header.stamp;
						velocity_msg.header.frame_id=params_.input_frame;
						velocity_msg.vector.x=velocity(0);
						velocity_msg.vector.y=velocity(1);
						velocity_msg.vector.z=velocity(2);

						// publish target's messages
						output_position_pub_.publish(position_msg);
						output_velocity_pub_.publish(velocity_msg);
					}
					
					if( params_.calibrate )
					{
						cv::Scalar color=cv::Scalar(0,255,0);
						
						// show segmented target
						cv::drawContours(original,hull,0,color,3);

						// draw the target position
						cv::Point p1=cv::Point(center.x-6,center.y-6);
						cv::Point p2=cv::Point(center.x+6,center.y+6);
						cv::Point p3=cv::Point(center.x-6,center.y+6);
						cv::Point p4=cv::Point(center.x+6,center.y-6);
						cv::line(original,p1,p2,color,1);
						cv::line(original,p3,p4,color,1);
						cv::rectangle(mask,p1,p2,color,1);
					}
				}
			}
		}
		else
		{
			position_filter_->reset();
			velocity_filter_->reset(_input->header.stamp);
		}
	}
	else
	{
		position_filter_->reset();
		velocity_filter_->reset(_input->header.stamp);
	}

	// publish target mask
	if( params_.calibrate )
	{
		sensor_msgs::ImagePtr mask_msg=cv_bridge::CvImage(std_msgs::Header(),"bgr8",original).toImageMsg();
		output_image_pub_.publish(mask_msg);
	}
}
