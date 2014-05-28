#ifndef _CONVEX_POLYGON_H
#define _CONVEX_POLYGON_H

#include "mathlib.h"

#ifdef _WIN32
class DecoRenderInterface;
#endif

class ConvexPolygon
{
public:
	ConvexPolygon();
	virtual ~ConvexPolygon();
	virtual void AddVertices(const vector<vector3>& vertices);
	virtual void SetVertices(const vector<vector3>& vertices);
	virtual double GetArea() const;
	virtual Plane GetPlane() const;
	virtual const vector3& GetCenter() const;
	virtual const vector<vector3>& GetVertices() const;
	virtual int GetNumVertices() const;
#ifdef _WIN32
	virtual void Render(DecoRenderInterface* RI);
#endif
private:
	vector<vector3> mVertices;
	vector3 mCenter;
	double mArea;
	vector3 mVertexAverage;
	void calculateVertexAverage();
	void updateArea();
	void updateCenter();
};

#endif
