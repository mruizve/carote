#include<Eigen/Dense>
#include<geometry_msgs/PointStamped.h>
#include<geometry_msgs/Vector3Stamped.h>
#include<pcl_conversions/pcl_conversions.h>
#include "carote/Params.h"
#include "carote/Target.h"

// trackbars callback function
static void colorOnChange(int current, void *arg)
{
	double *val=(double*)arg;

	if( NULL!=val )
	{
		*val=(double)current;
	}
}

static void depthOnChange(int current, void *arg)
{
	double *val=(double*)arg;

	if( NULL!=val )
	{
		*val=(double)current/100.0;
	}
}

carote::Target::Target(std::string _name)
:
	node_("~"),
	name_(_name)
{
	// input processing stuff: color/depth ranges and thresholds
	std::vector<int> aux;

	static const int color_low[]={1,0,0};
	aux=std::vector<int>(color_low,color_low+sizeof(color_low)/sizeof(int));
	aux=getParam< std::vector<int> >(node_,"color_range_low",aux);
	color_low_=cv::Scalar(aux[0],aux[1],aux[2]);

	static const int color_high[]={255,255,255};
	aux=std::vector<int>(color_high,color_high+sizeof(color_high)/sizeof(int));
	aux=getParam< std::vector<int> >(node_,"color_range_high",aux);
	color_high_=cv::Scalar(aux[0],aux[1],aux[2]);

	depth_low_=getParam<double>(node_,"depth_range_low",0);
	depth_high_=getParam<double>(node_,"depth_range_high",255);

	contours_threshold_=getParam<int>(node_,"contours_threshold",30);

	// input processing stuff: calibrate color and depth ranges?
	calibration_=getParam<int>(node_,"calibration",1);
	if( calibration_ )
	{
		this->calibrationGUI();
	}

	// input processing stuff: morphology kernels for mask filtering
	morphology_[0]=cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(3,3),cv::Point(-1,-1));
	morphology_[1]=cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(7,7),cv::Point(-1,-1));

	// output processing stuff: position and velocity filter
	int position_filter_N=getParam<int>(node_,"position_filter_N",15);
	position_filter_=new MeanFilter(position_filter_N);

	int velocity_filter_N=getParam<int>(node_,"velocity_filter_N",6);
	velocity_filter_=new HoloborodkoFilter(velocity_filter_N);

	// input topics names
	sensor_id_=getParam<std::string>(node_,"sensor","/camera");

	cloud_id_=getParam<std::string>(node_,"point_cloud","/depth_registered/points");
	cloud_id_=sensor_id_+cloud_id_;

	// sensor's reference frame
	sensor_frame_id_=getParam<std::string>(node_,"sensor_frame","camera_depth_optical_frame");

	// output topics names
	target_position_id_=getParam<std::string>(node_,"target_position","/position");
	target_position_id_=_name+target_position_id_;

	target_velocity_id_=getParam<std::string>(node_,"target_velocity","/velocity");
	target_velocity_id_=_name+target_velocity_id_;

	// prepare for listening to input topic
	cloud_sub_=node_.subscribe(cloud_id_,1,&carote::Target::callback,this);

	// prepare for advertise to the output topic
	target_position_pub_=node_.advertise<geometry_msgs::PointStamped>(target_position_id_,1);
	target_velocity_pub_=node_.advertise<geometry_msgs::Vector3Stamped>(target_velocity_id_,1);
}

carote::Target::~Target(void)
{
	delete position_filter_;
	delete velocity_filter_;
}

void carote::Target::calibrationGUI(void)
{
	//create mask window
	cv::namedWindow(name_,CV_WINDOW_AUTOSIZE);
 
	//create range trackbars for BGRD values (low and high)
	cv::createTrackbar("b-low",name_,NULL,255,&colorOnChange,&color_low_[0]);
	cv::setTrackbarPos("b-low",name_,(int)color_low_[0]);
	cv::createTrackbar("b-high",name_,NULL,255,&colorOnChange,&color_high_[0]);
	cv::setTrackbarPos("b-high",name_,(int)color_high_[0]);
	cv::createTrackbar("g-low",name_,NULL,255,&colorOnChange,&color_low_[1]);
	cv::setTrackbarPos("g-low",name_,(int)color_low_[1]);
	cv::createTrackbar("g-high",name_,NULL,255,&colorOnChange,&color_high_[1]);
	cv::setTrackbarPos("g-high",name_,(int)color_high_[1]);
	cv::createTrackbar("r-low",name_,NULL,255,&colorOnChange,&color_low_[2]);
	cv::setTrackbarPos("r-low",name_,(int)color_low_[2]);
	cv::createTrackbar("r-high",name_,NULL,255,&colorOnChange,&color_high_[2]);
	cv::setTrackbarPos("r-high",name_,(int)color_high_[2]);
	cv::createTrackbar("d-low",name_,NULL,255,&depthOnChange,&depth_low_);
	cv::setTrackbarPos("d-low",name_,(int)(100.0*depth_low_));
	cv::createTrackbar("d-high",name_,NULL,255,&depthOnChange,&depth_high_);
	cv::setTrackbarPos("d-high",name_,(int)(100.0*depth_high_));
}


void carote::Target::callback(const sensor_msgs::PointCloud2::ConstPtr _input)
{
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    pcl::fromROSMsg(*_input,cloud);

	// generate color and depth images from the point cloud
	cv::Mat color(cloud.height,cloud.width,CV_8UC3);
	cv::Mat depth(cloud.height,cloud.width,CV_8UC1);

    for( size_t i=0; cloud.width>i; i++)
    {
		for( size_t j=0; cloud.height>j; j++)
		{
			pcl::PointXYZRGB *p=&cloud[cloud.width*j+i];

			// generate color image
			color.at<cv::Vec3b>(j,i)=cv::Vec3b(p->b,p->g,p->r);

			// generate depth mask image
			float d=sqrtf(p->x*p->x+p->y*p->y+p->z*p->z);
			depth.at<char>(j,i)=(isnan(d) || d<depth_low_ || d>depth_high_)?0:255;
		}
	}

	// color image filtering and conversion to HSV color space
	cv::GaussianBlur(color,color,cv::Size(5,5),0.5);
	cvtColor(color,color,CV_BGR2HSV);

	// color mask generation
	cv::inRange(color,color_low_,color_high_,color);

	// target mask generation
	cv::Mat mask;
	cv::bitwise_and(color,depth,mask);

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
			if( contours[i].size()>contours_threshold_ )
			{
				outline.insert(outline.end(),contours[i].begin(),contours[i].end());
			}
		}

		// if the outline contains enough number of points, then
		if( outline.size()>contours_threshold_ )
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
						position_msg.header.frame_id=sensor_frame_id_;
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
						velocity_msg.header.frame_id=sensor_frame_id_;
						velocity_msg.vector.x=velocity(0);
						velocity_msg.vector.y=velocity(1);
						velocity_msg.vector.z=velocity(2);

						// publish target's messages
						target_position_pub_.publish(position_msg);
						target_velocity_pub_.publish(velocity_msg);
					}
					
					if( calibration_ )
					{
						// regenerate mask using the convex hull
						mask=cv::Scalar(0);
						int length=(int)hull[0].size();
						const cv::Point* points[1]={&hull[0][0]};
						cv::fillPoly(mask,points,&length,1,cv::Scalar(255,0,255),8);

						// draw the target position
						cv::Point p1=cv::Point(center.x-6,center.y-6);
						cv::Point p2=cv::Point(center.x+6,center.y+6);
						cv::Point p3=cv::Point(center.x-6,center.y+6);
						cv::Point p4=cv::Point(center.x+6,center.y-6);
						cv::line(mask,p1,p2,0,1);
						cv::line(mask,p3,p4,0,1);
						cv::rectangle(mask,p1,p2,0,1);
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

	// compute target coordinates
	
	// show target mask
	if( calibration_ )
	{
		cv::imshow(name_,mask);
		cv::waitKey(1);
	}
}
