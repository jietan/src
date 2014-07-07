#ifndef _EXTENDED_DEPTH_PIXEL_H
#define _EXTENDED_DEPTH_PIXEL_H

#include <Eigen/Dense>
#define MASK_UNKNOWN 1
#define MASK_KNOWN 0


class ExtendedDepthPixel
{
public:
	ExtendedDepthPixel() : d(0), n(Eigen::Vector3f::Ones())
	{

	}
	ExtendedDepthPixel(float depth) : d(depth), n(Eigen::Vector3f::Zero())
	{

	}
	ExtendedDepthPixel(float depth, const Eigen::Vector3f& normal) : d(depth), n(normal)
	{

	}
	virtual ~ExtendedDepthPixel()
	{

	}

	virtual bool operator< (const ExtendedDepthPixel& rhs) const
	{
		return this->d < rhs.d;
	}
	virtual bool operator<= (const ExtendedDepthPixel& rhs) const
	{
		return this->d <= rhs.d;
	}
	virtual bool operator >(const ExtendedDepthPixel& rhs)  const
	{
		return this->d > rhs.d;
	}
	float d;
	Eigen::Vector3f n;
};

class ExtendedDepthPixelWithMask : public ExtendedDepthPixel
{
public:
	int m;
	ExtendedDepthPixelWithMask()
	{
		m = MASK_KNOWN;
	}
	ExtendedDepthPixelWithMask(float depth)
	{
		d = depth;
		n = Eigen::Vector3f::Zero();
		m = MASK_KNOWN;
	}
	ExtendedDepthPixelWithMask(const ExtendedDepthPixel& extendDepthPx, int mask)
	{
		d = extendDepthPx.d;
		n = extendDepthPx.n;
		m = mask;
	}
	ExtendedDepthPixelWithMask(float depth, const Eigen::Vector3f& normal, int mask)
	{
		d = depth;
		n = normal;
		m = mask;
	}
	virtual ~ExtendedDepthPixelWithMask()
	{

	}
};
#endif