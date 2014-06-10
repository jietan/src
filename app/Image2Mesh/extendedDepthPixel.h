#ifndef _EXTENDED_DEPTH_PIXEL_H
#define _EXTENDED_DEPTH_PIXEL_H



class ExtendedDepthPixel
{
public:
	ExtendedDepthPixel() : d(0), n(Vector3f::Ones())
	{

	}
	ExtendedDepthPixel(float depth, const Vector3f& normal) : d(depth), n(normal)
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
	Vector3f n;
};

#endif