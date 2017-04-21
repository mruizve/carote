#ifndef _CAROTE_HOLOROBODKO_FILER_H_
#define _CAROTE_HOLOROBODKO_FILER_H_

// filter design reference:
// http://www.holoborodko.com/pavel/numerical-methods/numerical-derivative/smooth-low-noise-differentiators/
// http://www.holoborodko.com/pavel/wp-content/uploads/OneSidedNoiseRobustDifferentiators.pdf

#include<boost/circular_buffer.hpp>
#include<Eigen/Dense>
#include<ros/ros.h>

namespace carote
{
	class HoloborodkoFilter
	{
		public:
			HoloborodkoFilter(size_t _order);
			bool ok(void);
			void reset(ros::Time _stamp);

			Eigen::Vector3d apply(Eigen::Vector3d _p, ros::Time _stamp);

		private:
			double rule(boost::circular_buffer<double> s, double dt);

		protected:
			size_t filter_order_;
			size_t number_samples_;
			double time_step_;
			ros::Time last_time_;
			std::vector<double> coeff_;
			std::vector< boost::circular_buffer<double> > samples_;
	};
}

#endif
