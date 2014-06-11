#ifndef _EXTENDED_DEPTH_PIXEL_H
#define _EXTENDED_DEPTH_PIXEL_H

#include <Eigen/Dense>


class ExtendedDepthPixel
{
public:
	ExtendedDepthPixel() : d(0), n(Eigen::Vector3f::Ones())
	{

	}
	ExtendedDepthPixel(float depth, const Eigen::Vector3f& normal) : d(depth), n(normal)
	{

	}

	bool operator< (const ExtendedDepthPixel& rhs) const
	{
		return this->d < rhs.d;
	}
	bool operator<= (const ExtendedDepthPixel& rhs) const
	{
		return this->d <= rhs.d;
	}
	bool operator >(const ExtendedDepthPixel& rhs)  const
	{
		return this->d > rhs.d;
	}
	float d;
	Eigen::Vector3f n;
};

#endif