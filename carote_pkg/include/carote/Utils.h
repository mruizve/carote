#ifndef _CAROTE_UTILS_H_
#define _CAROTE_UTILS_H_

#include<Eigen/SVD>
#include<Eigen/Geometry>
#include<ros/ros.h>

#define CAROTE_NODE_ABORT(stream) do \
{ \
	ROS_FATAL_STREAM("[" << ros::this_node::getName() << "] " << stream); \
	exit(EXIT_SUCCESS); \
} while(0)

// sign function with sgn(0)=0
template<typename T> int sgn(T val)
{
	return (T(0)<val)-(val<T(0));
}

//compute the matrix pseudo inverse using the Eigen library
namespace Eigen
{
	template<typename _T, int _rows, int _cols>
	Matrix<_T,_cols,_rows> pinv(const Matrix<_T,_rows,_cols>& _m, const double _eps)
	{
		// compute the SVD and get the singular values
		typename JacobiSVD< Matrix<_T,_rows,_cols> >::SingularValuesType sigma;
		JacobiSVD< Matrix<_T,_rows,_cols> > svd(_m,ComputeFullU|ComputeFullV);
		sigma=svd.singularValues();

		//compute the inverse of the singular values avoiding kinematics singularities
		for(unsigned int i=0;sigma.size()>i;i++)
		{
			if( 5.0*_eps<sigma(i) )
			{
				sigma(i)=1.0/sigma(i);
			}
			else
			{
				sigma(i)=1.0/(_eps*_eps+sigma(i)*sigma(i));
			}
		}

		// generate the diagonal matrix of singular values
		Matrix<_T,_rows,_cols> Sigma;
		Sigma.setZero();
		Sigma.diagonal()=sigma;

		//compute the pseudo inverse psi = V * Sigma^-1 * U^T
		return svd.matrixV()*Sigma.transpose()*svd.matrixU().transpose();
	}
}

// robust vector normalization
template<typename _T> inline void normalize(_T& _u, double _epsilon)
{
	if( _epsilon>_u.norm() )
	{
		_u/=_epsilon;
	}
	else
	{
		_u.normalize();
	}
}

// non-linear feedback control gain
inline double gamma(double _e, double _alpha, double _beta)
{
	return sgn(_e)*(1.0-exp(-pow(_alpha*_e/_beta,2)));
}

#endif
