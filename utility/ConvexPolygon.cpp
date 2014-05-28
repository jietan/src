#include "stdafx.h"
#include "ConvexPolygon.h"
#include "glog/logging.h"
using namespace google;

ConvexPolygon::ConvexPolygon()
{

}
ConvexPolygon::~ConvexPolygon()
{

}
void ConvexPolygon::AddVertices(const vector<vector3>& vertices)
{
	mVertices.insert(mVertices.end(), vertices.begin(), vertices.end());
	calculateVertexAverage();
	updateCenter();
	updateArea();
}

void ConvexPolygon::SetVertices(const vector<vector3>& vertices)
{
	mVertices.clear();
	AddVertices(vertices);
}

#ifdef _WIN32
void ConvexPolygon::Render(DecoRenderInterface* RI)
{
	vector<vector3> allVertices;
	int numVertices = GetNumVertices();
	if (!numVertices) return;

	for (int i = 0; i < numVertices; ++i)
	{
		vector3 start = mVertices[i];
		vector3 end = mVertices[(i + 1) % numVertices];
		allVertices.push_back(start);
		allVertices.push_back(end);
	}
	DecoColor color(0xffff0000);

	DecoVertexBuffer vb;
	vb.SetVertexInformation(allVertices.size(), &(allVertices[0]));
	RI->SetColor(color);
	RI->SetLineWidth(3);
	RI->DrawPrimitive(PT_LineList, allVertices.size() / 2, DT_SolidPolygon, &vb);

	vector3 center = GetCenter();
	DecoRenderMisc::GetSingleton()->DrawPointList(&center, &color, 1, 5);
}
#endif
double ConvexPolygon::GetArea() const
{
	return mArea;
}

Plane ConvexPolygon::GetPlane() const
{
	if (GetNumVertices() < 3)
	{
		LOG(WARNING) << "At lease three vertices needed to define a plane.";
		return Plane();
	}
	Plane pl(mVertices[0], mVertices[1], mVertices[2]);
	return pl;
}

const vector3& ConvexPolygon::GetCenter() const 
{
	return mCenter;
}

const vector<vector3>& ConvexPolygon::GetVertices() const
{
	return mVertices;
}

int ConvexPolygon::GetNumVertices() const
{
	return static_cast<int>(mVertices.size());
}

void ConvexPolygon::calculateVertexAverage()
{
	int numVertices = GetNumVertices();
	mVertexAverage = vector3(0, 0, 0);
	for (int i = 0; i < numVertices; ++i)
	{
		mVertexAverage += mVertices[i];
	}
	mVertexAverage /= numVertices;
}

void ConvexPolygon::updateArea()
{
	mArea = 0;
	int numVertices = GetNumVertices();

	for (int i = 0; i < numVertices; ++i)
	{
		vector3 a = mVertexAverage;
		vector3 b = mVertices[i];
		vector3 c = mVertices[(i + 1) % numVertices];
		mArea += TriangleArea(a, b, c);
	}
	mArea = abs(mArea);
}

void ConvexPolygon::updateCenter()
{
	int numVertices = GetNumVertices();
	double totalArea = 0;
	vector3 totalCenter(0, 0, 0);

	for (int i = 0; i < numVertices; ++i)
	{
		vector3 a = mVertexAverage;
		vector3 b = mVertices[i];
		vector3 c = mVertices[(i + 1) % numVertices];
		vector3 center = (a + b + c) / 3.0;
		double area = TriangleArea(a, b, c); 
		totalCenter += area * center;
		totalArea += area;
	}
	totalCenter /= totalArea;
	mCenter = totalCenter;
}
