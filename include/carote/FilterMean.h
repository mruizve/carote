#ifndef _CAROTE_MEAN_FILER_H_
#define _CAROTE_MEAN_FILER_H_

#include<boost/circular_buffer.hpp>
#include<Eigen/Dense>
#include<geometry_msgs/Vector3.h>
#include<ros/ros.h>

namespace carote
{
	class MeanFilter
	{
		public:
			MeanFilter(size_t _order);
			bool ok(void);
			void reset(void);

			Eigen::Vector3d apply(Eigen::Vector3d _p);

		private:
			double rule(boost::circular_buffer<double> _s);

		protected:
			size_t filter_order_;
			size_t number_samples_;
			std::vector< boost::circular_buffer<double> > samples_;
	};
}

#endif
