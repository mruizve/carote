#include "carote/FilterHoloborodko.h"

#define NCOEFF 7
static const double rules[][NCOEFF]=
{
	{ 1.00000000,-0.50000000,-1.00000000, 0.50000000, 0.00000000, 0.00000000, 0.00000000},
	{ 0.70000000, 0.10000000,-1.00000000,-0.10000000, 0.30000000, 0.00000000, 0.00000000},
	{ 0.57142857, 0.03571429,-0.35714286,-0.35714286,-0.21428571, 0.32142857, 0.00000000},
	{ 0.42857143, 0.17857143,-0.28571429,-0.21428571,-0.35714286, 0.03571429, 0.21428571}
};

carote::HoloborodkoFilter::HoloborodkoFilter(size_t _order)
{
	// load filter coefficients
	for( size_t i=0; NCOEFF>i; i++ )
	{
		coeff_.push_back(rules[_order-3][i]);
	}

	filter_order_=_order;
	this->reset(ros::Time::now());
}

bool carote::HoloborodkoFilter::ok(void)
{
	return (filter_order_<number_samples_);
}

void carote::HoloborodkoFilter::reset(ros::Time _stamp)
{
	// initialization
	number_samples_=0;
	std::vector< boost::circular_buffer<double> >().swap(samples_);

	// (one ring buffer for each spatial coordinate)
	for( size_t i=0; 3>i; i++ )
	{
		samples_.push_back(boost::circular_buffer<double>(filter_order_+1));
		// (each ring buffer requires N samples)
		for( size_t j=0; filter_order_>j; j++ )
		{
			samples_[i].push_back(0.0);
		}
	}

	last_time_=_stamp;

}

double carote::HoloborodkoFilter::rule(boost::circular_buffer<double> _s, double dt)
{
	double h=1.0/dt;
	double res=0.0;

	for( size_t i=0; filter_order_>=i; i++ )
	{
		res+=coeff_[i]*_s[i];
	}

	return h*res;
}

Eigen::Vector3d carote::HoloborodkoFilter::apply(Eigen::Vector3d _p, ros::Time _stamp)
{
	Eigen::Vector3d v;

	// load data sample into the ring buffers
	samples_[0].push_back(_p(0));
	samples_[1].push_back(_p(1));
	samples_[2].push_back(_p(2));
	number_samples_++;

	// compute time step (we assume that is roughly constant)
	time_step_=(_stamp-last_time_).toSec();

	// apply the filter to each sample component
	v(0)=this->rule(samples_[0],time_step_);
	v(1)=this->rule(samples_[1],time_step_);
	v(2)=this->rule(samples_[2],time_step_);
	last_time_=_stamp;

	return v;
}
