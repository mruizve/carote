#include "carote/FilterMean.h"

carote::MeanFilter::MeanFilter(size_t _order)
{
	filter_order_=_order;
	this->reset();
}

bool carote::MeanFilter::ok(void)
{
	return (filter_order_<number_samples_);
}

void carote::MeanFilter::reset(void)
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
}

double carote::MeanFilter::rule(boost::circular_buffer<double> _s)
{
	double res=0.0;
	static double b=1.0/filter_order_;

	for( size_t i=0; filter_order_>i; i++ )
	{
		res+=_s[i];
	}

	return b*res;
}

Eigen::Vector3d carote::MeanFilter::apply(Eigen::Vector3d _p)
{
	Eigen::Vector3d q;

	// load data sample into the input ring buffers
	samples_[0].push_back(_p(0));
	samples_[1].push_back(_p(1));
	samples_[2].push_back(_p(2));
	number_samples_++;

	// apply the filter to each sample component
	q(0)=this->rule(samples_[0]);
	q(1)=this->rule(samples_[1]);
	q(2)=this->rule(samples_[2]);

	return q;
}
