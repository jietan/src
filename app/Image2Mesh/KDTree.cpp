/*
 *  KDTree.cpp
 *  CS248-Final-Project
 *
 *  Created by zaltor on 2/17/11.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */

#include "KDTree.h"
#include <vector>
#include <algorithm>
#include <glog/logging.h>
#include "utility/mathlib.h"
#include <Eigen/Dense>
using namespace google;

// split a line segment a1a2 across an axis-aligned plane
// into two parts -- a1b1 and b2a2
// direction is set to true if a1 <= a2 for the specified dimension
// a1b1Valid is set to true if a1b1 is a valid line segment
static void linePlaneIntersect(aiVector3D &a1,
							   aiVector3D &a2,
							   aiVector3D &b1,
							   aiVector3D &b2,
							   bool &a1b1Valid,
							   bool &b2a2Valid,
							   bool &direction,
							   int dimension,
							   float split) {
	float proj1 = a1[dimension] - split;
	float proj2 = a2[dimension] - split;
	direction = (proj1 <= proj2);
	if (proj1 * proj2 < 0) {
		float alpha = -proj1 / (proj2-proj1);
		b1 = a1 + alpha * (a2-a1);
		b2 = b1;
		a1b1Valid = true;
		b2a2Valid = true;
		return;
	}
	if (proj1 == 0 && proj2 == 0) {
		// line segment rides on plane
		b1 = a2;
		b2 = a1;
		a1b1Valid = true;
		b2a2Valid = true;
		return;
	}
	if (proj1 == 0) {
		// one vertex on plane
		b1 = a1;
		b2 = a1;
		a1b1Valid = direction; // a coord is placed into either the [<split] bucket or the [>=split] bucket
		b2a2Valid = true;
		return;
	}
	if (proj2 == 0) {
		// ditto
		b1 = a2;
		b2 = a2;
		a1b1Valid = true;
		b2a2Valid = !direction;
		return;
	}
	// no intersection
	if (fabs(proj1) < fabs(proj2)) {
		b1 = a1;
		b2 = a1;
		a1b1Valid = false;
		b2a2Valid = true;
	} else {
		b1 = a2;
		b2 = a2;
		a1b1Valid = true;
		b2a2Valid = false;
	}
	return;
}

bool findPlane(aiVector3D x, aiVector3D y, aiVector3D &normal) {
	aiVector3D crossprod;
	crossprod.x = x.y * y.z - x.z * y.y;
	crossprod.y = x.z * y.x - x.x * y.z;
	crossprod.z = x.x * y.y - x.y * y.x;
	if (crossprod.Length() == 0.0) {
		return false;
	}
	normal = crossprod.Normalize();
	return true;
}

int lttests = 0;

// determine whether a line segment intersects a triangle
// and return an alpha between 0 and 1 to indicate the point of intersection
bool lineTriangleIntersect1(aiVector3D &a1Orig, aiVector3D &a2Orig,
	aiVector3D &vertex1, aiVector3D &vertex2, aiVector3D &vertex3,
	float &alpha) {

	lttests++;
	const double eps = 0;
	// create two edges of triangle
	Eigen::Vector3d v1(vertex1.x, vertex1.y, vertex1.z);
	Eigen::Vector3d v2(vertex2.x, vertex2.y, vertex2.z);
	Eigen::Vector3d v3(vertex3.x, vertex3.y, vertex3.z);
	Eigen::Vector3d a1(a1Orig.x, a1Orig.y, a1Orig.z);
	Eigen::Vector3d a2(a2Orig.x, a2Orig.y, a2Orig.z);

	Eigen::Vector3d edge1 = v2 - v1;
	Eigen::Vector3d edge2 = v3 - v2;

	// compute plane of triangle
	Eigen::Vector3d normal = edge1.cross(edge2);
	if (normal.norm() == 0)
		return false;
	normal.normalize();
	double offset = normal.dot(v2);
	// compute intersection of ray with plane
	double proj1 = a1.dot(normal) - offset;
	double proj2 = a2.dot(normal) - offset;
	if (proj1 == proj2) {
		// parallel to triangle, we'll consider it not an intersection even if line is embedded in triangle
		return false;
	}
	// create a basis for the plane
	Eigen::Vector3d b1 = edge1.normalized();
	Eigen::Vector3d b2;
	edge2 = normal.cross(edge1);
	edge2.normalize();

	if (proj1 * proj2 <= 0.0) {
		// definite intersection with plane
		alpha = -proj1 / (proj2 - proj1);
		// get intersect
		Eigen::Vector3d intersect = a1 + alpha * (a2 - a1);
		// convert points to 2D

		Eigen::Matrix3d xform;
		xform.row(0) = edge1;
		xform.row(1) = edge2;
		xform.row(2) = normal;

		Eigen::Vector3d i2D = xform*intersect;
		Eigen::Vector3d v12D = xform*v1;
		Eigen::Vector3d v22D = xform*v2;
		Eigen::Vector3d v32D = xform*v3;

		// find edges
		Eigen::Vector3d e12D = v22D - v12D;
		Eigen::Vector3d e22D = v32D - v22D;
		Eigen::Vector3d e32D = v12D - v32D;
		double e1Len = e12D.norm();
		double e2Len = e22D.norm();
		double e3Len = e32D.norm();
		double a1 = -i2D[0] * e12D[1] + i2D[1] * e12D[0] - (-v12D[0] * e12D[1] + v12D[1] * e12D[0]);
		double a2 = -i2D[0] * e22D[1] + i2D[1] * e22D[0] - (-v22D[0] * e22D[1] + v22D[1] * e22D[0]);
		double a3 = -i2D[0] * e32D[1] + i2D[1] * e32D[0] - (-v32D[0] * e32D[1] + v32D[1] * e32D[0]);
		//double offset1 = -v12D.x*e12D.y + v12D.y*e12D.x;
		//double offset2 = -v22D.x*e22D.y + v22D.y*e22D.x;
		//double offset3 = -v32D.x*e32D.y + v32D.y*e32D.x;
		if (e12D[0] * e22D[1] > e12D[1] * e22D[0]) {
			// ccw winding order
			//return ((-i2D.x*e12D.y + i2D.y*e12D.x) > offset1 - (eps / e1Len) &&
			//	(-i2D.x*e22D.y + i2D.y*e22D.x) > offset2 - (eps / e2Len) &&
			//	(-i2D.x*e32D.y + i2D.y*e32D.x) > offset3 - (eps / e3Len));
			return a1 >= -eps / e1Len && a2 >= -eps / e2Len && a3 >= -eps / e3Len;
		}
		else {
			// cw winding order
			//return ((-i2D.x*e12D.y + i2D.y*e12D.x) < offset1 + (eps / e1Len) &&
			//	(-i2D.x*e22D.y + i2D.y*e22D.x) < offset2 + (eps / e2Len) &&
			//	(-i2D.x*e32D.y + i2D.y*e32D.x) < offset3 + (eps / e3Len));
			return a1 <= eps / e1Len && a2 <= eps / e2Len && a3 <= eps / e3Len;
		}
	}
	return false; // no intersection
}

// determine whether a line segment intersects a triangle
// and return an alpha between 0 and 1 to indicate the point of intersection
bool lineTriangleIntersect(aiVector3D &a1, aiVector3D &a2,
						   aiVector3D &v1, aiVector3D &v2, aiVector3D &v3,
						   float &alpha) {
	//vector3 a(v1.x, v1.y, v1.z);
	//vector3 b(v2.x, v2.y, v2.z);
	//vector3 c(v3.x, v3.y, v3.z);
	//vector3 origin(a1.x, a1.y, a1.z);
	//aiVector3D d = a2 - a1;
	//aiVector3D dNormalized = d;// .Normalize();
	//vector3 dir(dNormalized.x, dNormalized.y, dNormalized.z);
	//vector3 intersection;
	//double time;
	//BOOL ret = TriangleRayIntersection(a, b, c, origin, dir, &intersection, &time);
	//alpha = static_cast<float>(time);
	//return ret;
	
	lttests++;
	const double eps = 1e-12;
	// create two edges of triangle
	aiVector3D edge1 = v2-v1;
	aiVector3D edge2 = v3-v2;

	Eigen::Vector3d e1(edge1.x, edge1.y, edge1.z);
	Eigen::Vector3d e2(edge2.x, edge2.y, edge2.z);

	// compute plane of triangle
	aiVector3D normal;
	if (!findPlane(edge1, edge2, normal)) 
		return false; // no intersection with degenerate triangles
	float offset = normal * v2;
	// compute intersection of ray with plane
	float proj1 = a1 * normal - offset;
	float proj2 = a2 * normal - offset;
	if (proj1 == proj2) {
		// parallel to triangle, we'll consider it not an intersection even if line is embedded in triangle
		return false;
	}
	// create a basis for the plane
	aiVector3D b1 = edge1.Normalize();
	aiVector3D b2;
	findPlane(normal, edge1, edge2); // get cross product (be careful about winding order here!)
	aiMatrix3x3 xform(edge1.x, edge1.y, edge1.z,
					  edge2.x, edge2.y, edge2.z,
					  normal.x, normal.y, normal.z);
	if (proj1 * proj2 <= 0.0) {
		// definite intersection with plane
		alpha = -proj1 / (proj2 - proj1);
		// get intersect
		aiVector3D intersect = a1 + alpha * (a2-a1);
		// convert points to 2D

		Eigen::Vector3d n = e1.cross(e2);
		n.normalize();
		e2 = n.cross(e1);
		Eigen::Matrix3d xformEigen;
		xformEigen.row(0) = e1;
		xformEigen.row(1) = e2;
		xformEigen.row(2) = n;

		Eigen::Vector3d v1Eigen(v1.x, v1.y, v1.z);
		Eigen::Vector3d v2Eigen(v2.x, v2.y, v2.z);
		Eigen::Vector3d v3Eigen(v3.x, v3.y, v3.z);
		Eigen::Vector3d iEigen(intersect.x, intersect.y, intersect.z);
		v1Eigen = xformEigen * v1Eigen;
		v2Eigen = xformEigen * v2Eigen;
		v3Eigen = xformEigen * v3Eigen;
		iEigen = xformEigen * iEigen;

		aiVector3D i2D = xform*intersect;
		aiVector3D v12D = xform*v1;
		aiVector3D v22D = xform*v2;
		aiVector3D v32D = xform*v3;

		i2D.x = iEigen[0]; i2D.y = iEigen[1];
		v12D.x = v1Eigen[0]; v12D.y = v1Eigen[1];
		v22D.x = v2Eigen[0]; v22D.y = v2Eigen[1];
		v32D.x = v3Eigen[0]; v32D.y = v3Eigen[1];
		// find edges
		aiVector3D e12D = v22D - v12D;
		aiVector3D e22D = v32D - v22D;
		aiVector3D e32D = v12D - v32D;
		double e1Len = e12D.Length();
		double e2Len = e22D.Length();
		double e3Len = e32D.Length();
		double offset1 = -v12D.x*e12D.y + v12D.y*e12D.x;
		double offset2 = -v22D.x*e22D.y + v22D.y*e22D.x;
		double offset3 = -v32D.x*e32D.y + v32D.y*e32D.x;
		if (e12D.x*e22D.y > e12D.y*e22D.x) {
			// ccw winding order
			return ((-i2D.x*e12D.y + i2D.y*e12D.x) > offset1 - (eps / e1Len) &&
				(-i2D.x*e22D.y + i2D.y*e22D.x) > offset2 - (eps / e2Len) &&
				(-i2D.x*e32D.y + i2D.y*e32D.x) > offset3 - (eps / e3Len));
		} else {
			// cw winding order
			return ((-i2D.x*e12D.y + i2D.y*e12D.x) < offset1 + (eps / e1Len) &&
				(-i2D.x*e22D.y + i2D.y*e22D.x) < offset2 + (eps / e2Len) &&
				(-i2D.x*e32D.y + i2D.y*e32D.x) < offset3 + (eps / e3Len));
		}
	}
	return false; // no intersection
}

KDTreeTriangle::KDTreeTriangle() {
	this->parent = NULL;
	this->meshNumber = 0;
}

KDTreeTriangle::~KDTreeTriangle () {
}

KDTreeItem::KDTreeItem() {
	this->tree = NULL;
	this->triangle = -1;
}

KDTreeItem::~KDTreeItem() {
}

void KDTreeItem::set(KDTree *tree, size_t triangle) {
	if (!tree || triangle >= tree->triangles.size()) {
		std::cerr << "Invalid triangle" << std::endl;
		return;
	}
	this->tree = tree;
	this->triangle = triangle;
	aiVector3D v1 = tree->triangles[triangle].vertexes[0];
	aiVector3D v2 = tree->triangles[triangle].vertexes[1];
	aiVector3D v3 = tree->triangles[triangle].vertexes[2];
	for (int i=0;i<3;i++) {
		this->mins[i] = std::min(std::min(v1[i],v2[i]),v3[i]);
		this->maxes[i] = std::max(std::max(v1[i],v2[i]),v3[i]);
	}
}

bool KDTreeItem::empty(void) {
	// unimplemented
	return false;
}


void KDTreeItem::tighten() {
	// unimplemented
	return;
}


void KDTreeItem::split(KDTreeItem &left, KDTreeItem &right, KDTreeSplit axis, float point, bool &validLeft, bool &validRight) {
	// first, clone triangle info
	left = *this;
	right = *this;
    int index;
	// update bounding boxes
	switch (axis) {
		case SPLIT_X:
		case SPLIT_Y:
		case SPLIT_Z:
			index = ((int)axis)-1;
			if (left.maxes[index] < point) {
				// move everything to left
				validLeft = true;
				validRight = false;
				return;
			}
			if (right.mins[index] >= point) {
				// move everything to right
				validLeft = false;
				validRight = true;
				return;
			}
			// otherwise, split
			left.maxes[index] = point;
			right.mins[index] = point;
			validLeft = true;
			validRight = true;
			validLeft = !left.empty(); 
			validRight = !right.empty();
			left.tighten();
			right.tighten();
			if (validLeft == validRight && validLeft == false) {
				std::cerr << "error! removed triangle entirely!" << std::endl;
			}
			break;
		default:
			std::cerr << "KDTreeTriangle::split called with wrong split axis" << std::endl;
			return;
	}
}

KDTree::KDTree() {
	this->root = new KDTreeNode(this);
}

KDTree::~KDTree() {
	delete this->root;
}

void KDTree::addScene(const aiScene *scene) {
	// default constructor of aiMatrix4x4 creates identity matrix 
	aiMatrix4x4 identity;
	this->addNode(scene, scene->mRootNode, identity);
}

void KDTree::addNode(const aiScene *scene, const aiNode *node, aiMatrix4x4 &parentTransform) {
	// apply current node transform
	aiMatrix4x4 transform = parentTransform * (node->mTransformation);
	// add the meshes
	for (unsigned int i=0; i<node->mNumMeshes; i++) {
		this->addMesh(scene->mMeshes[node->mMeshes[i]], transform, node, i);
	}
	// add the children
	for (unsigned int i=0; i<node->mNumChildren; i++) {
		this->addNode(scene, node->mChildren[i], transform);
	}
}

void KDTree::addMesh(const aiMesh *mesh, aiMatrix4x4 &transform, const aiNode *parent, unsigned int meshNumber) {
	// check to make sure we only have triangles
	if (mesh->mPrimitiveTypes != aiPrimitiveType_TRIANGLE) {
		return;
	}
	// create an array to hold output vertex data
	std::vector<aiVector3D> vertexes;
    std::vector<aiVector3D> normals;
	vertexes.reserve(mesh->mNumVertices);

    //aiMatrix4x4 normalMatrix = transform.Inverse().Transpose();

	// transform vertexes
	for (unsigned int i=0; i<mesh->mNumVertices; i++) {
		aiVector3D temp = mesh->mVertices[i];
		temp *= transform;
		vertexes.push_back(temp);

        //aiVector3D norm = mesh->mNormals[i];
        //norm *= normalMatrix;
        //normals.push_back(norm);
	}
	// now add all the triangles
	for (unsigned int i=0; i<mesh->mNumFaces; i++) {
		KDTreeTriangle triangle;
        
		triangle.vertexes[0] = vertexes[mesh->mFaces[i].mIndices[0]];
		triangle.vertexes[1] = vertexes[mesh->mFaces[i].mIndices[1]];
		triangle.vertexes[2] = vertexes[mesh->mFaces[i].mIndices[2]];

		aiVector3D v01(triangle.vertexes[1] - triangle.vertexes[0]);
		aiVector3D v02(triangle.vertexes[2] - triangle.vertexes[0]);
		triangle.normal = cross(v01, v02);
        //triangle.normal = aiVector3D();
        //triangle.normal += normals[mesh->mFaces[i].mIndices[0]];
        //triangle.normal += normals[mesh->mFaces[i].mIndices[1]];
        //triangle.normal += normals[mesh->mFaces[i].mIndices[2]];
        triangle.normal.Normalize();

		triangle.parent = parent;
		triangle.meshNumber = meshNumber;
		this->triangles.push_back(triangle);
		// add bounding box to root node
		KDTreeItem item;
		item.set(this, this->triangles.size()-1);
		this->root->items.push_back(item);
	}
}

void KDTree::process(void) {
	this->root->process();
}

bool KDTree::intersect(aiVector3D &x1, aiVector3D &x2, aiVector3D &result, int &triangle) {
	return this->root->intersect(x1, x2, result, triangle);
/*  The following code reduces redundant triangle tests, but that doesn't seem to affect performance by too much
	std::set<int> triangles;
	// get a set of triangles to test
	this->root->gather(x1, x2, triangles);
	// test them
	float bestAlpha = 2.0;
	int bestTriangle = -1;
	for (std::set<int>::iterator i=triangles.begin();
		 i != triangles.end();
		 ++i) {
		float alpha;
		KDTreeTriangle *triPtr = &(this->triangles[*i]);
		if (lineTriangleIntersect(x1, x2, triPtr->vertexes[0], triPtr->vertexes[1], triPtr->vertexes[2], alpha)) {
			if (alpha < bestAlpha) {
				bestAlpha = alpha;
				bestTriangle = *i;
			}
		}
	}
	if (bestAlpha >= 0.0 && bestAlpha <= 1.0) {
		result = x1 + bestAlpha * (x2-x1);
		triangle = bestTriangle;
		return true;
	}
	return false;
*/
}

KDTreeNode::KDTreeNode (KDTree *tree) {
	this->tree = tree;
	this->splitDirection = SPLIT_NONE;
	this->splitPoint = 0.0f;
	this->left = NULL;
	this->right = NULL;
}

KDTreeNode::~KDTreeNode () {
	if (this->splitDirection != SPLIT_NONE) {
		if(this->left) {
			delete this->left;
		} else {
			std::cerr << "Error: expecting left node to not be empty" << std::endl;
		}
		if(this->right) {
			delete this->right;
		} else {
			std::cerr << "Error: expecting right node to not be empty" << std::endl;
		}
	} else {
		if(this->left) {
			std::cerr << "Expecting split with nonzero left node" << std::endl;
		}
		if(this->right) {
			std::cerr << "Expecting split with nonzero right node" << std::endl;
		}
	}
}


void KDTreeNode::updateBoundingBox(void) {
	int numItems = this->items.size();
	if (numItems > 0) {
		std::list<KDTreeItem>::iterator i=this->items.begin();
		if (this->splitDirection == SPLIT_NONE) {
			this->mins = i->mins;
			this->maxes = i->maxes;
			++i;
		}
		for (;
			 i!=this->items.end();
			 ++i) {
			for (int j=0;j<3;j++) {
				this->mins[j] = std::min(this->mins[j], i->mins[j]);
				this->maxes[j] = std::max(this->maxes[j], i->maxes[j]);
			}
		}
	}	
}

float KDTreeNode::volume(void) {
	return (this->maxes[0]-this->mins[0])*(this->maxes[1]-this->mins[1])*(this->maxes[2]-this->mins[2]);
}

void KDTreeNode::process(int level) {
	// update bounding box if necessary
	LOG(INFO) << "KDTreeNode::process() start process " << level << "th level.";
	this->updateBoundingBox();
	
	KDTreeSplit axis;
	float point;
	bool newSplit = (this->splitDirection != SPLIT_X && 
					 this->splitDirection != SPLIT_Y &&
					 this->splitDirection != SPLIT_Z);
	// decide whether to calculate new splitting plane
	if (newSplit) {
		if (this->items.size() < 2) {
			// no need to do splitting
			return;
		}
		if (this->volume() < 0.001) {
			// no need to split something this small
			return;
		}
		// find longest axis
		aiVector3D extent = this->maxes-this->mins;
		if (extent.x > extent.y) {
			if (extent.x > extent.z) {
				axis = SPLIT_X;
			} else {
				axis = SPLIT_Z;
			}
		} else {
			if (extent.y > extent.z) {
				axis = SPLIT_Y;
			} else {
				axis = SPLIT_Z;
			}
		}
		point = 0.5f*(this->maxes[axis-1] + this->mins[axis-1]);
	} else {
		// use existing split
		axis = this->splitDirection;
		point = this->splitPoint;
	}
	// sort items down tree
	int leftReject = 0;
	int rightReject = 0;
	std::list<KDTreeItem> leftItems;
	std::list<KDTreeItem> rightItems;
	for (std::list<KDTreeItem>::iterator i=this->items.begin();
		 i!=this->items.end();
		 ++i) {
		// split a bounding box into two bounding boxes if needed
		KDTreeItem leftItem;
		KDTreeItem rightItem;
		bool keepLeft = false;
		bool keepRight = false;
		i->split(leftItem, rightItem, axis, point, keepLeft, keepRight);
		if (keepLeft) 
			leftItems.push_back(leftItem);
		else
			leftReject++;
		if (keepRight) 
			rightItems.push_back(rightItem);
		else
			rightReject++;
	}
	// if either child received all the items, stop
	if (newSplit && (leftReject == 0 && rightReject == 0)) {
		// do nothing
	} else {
		// otherwise, add the items
		if (!this->left)
			this->left = new KDTreeNode(this->tree);
		if (!this->right)
			this->right = new KDTreeNode(this->tree);
		this->left->items.insert(this->left->items.end(),
								 leftItems.begin(),
								 leftItems.end());
		this->right->items.insert(this->right->items.end(),
					 			 rightItems.begin(),
								 rightItems.end());
		this->splitDirection = axis;
		this->splitPoint = point;
		//std::cerr << "Bounding box is " ;
		//for (int i=0;i<3;i++) {
		//	std::cerr << "[" << this->mins[i] << "," << this->maxes[i] << ") ";
		//}
		//std::cerr << std::endl << "Split at level " << level << " along axis " << axis << " at point " << point << " " << leftItems.size() << "<-" << this->items.size() << "->" << rightItems.size() << std::endl;
		this->items.clear();
		// run recursively on children
		//if (leftReject != 0)
			this->left->process(level+1);
		//if (rightReject != 0)
			this->right->process(level+1);	
	}
}

// gather possible triangle candidates
void KDTreeNode::gather(aiVector3D &x1, aiVector3D &x2, std::set<int> &triangles) {
	for (std::list<KDTreeItem>::iterator i=this->items.begin();
		 i != this->items.end();
		 ++i) {
		triangles.insert(i->triangle);
	}
	if (this->splitDirection == SPLIT_X ||
		this->splitDirection == SPLIT_Y ||
		this->splitDirection == SPLIT_Z) {
		// create two line segments
		aiVector3D b1, b2;
		bool a1b1Valid = false, b2a2Valid = false;
		bool direction;
		aiVector3D leftResult, rightResult;
		bool leftIntersects=false, rightIntersects=false;
		
		// split line segment in two
		linePlaneIntersect(x1, x2, b1, b2, a1b1Valid, b2a2Valid, direction, this->splitDirection-1, this->splitPoint);
		
		aiVector3D x1l, x2l;
		aiVector3D x1r, x2r;
		bool leftValid, rightValid;
		
		if (direction) {
			x1l = x1; x2l = b1; x1r = b2; x2r = x2;
			leftValid = a1b1Valid;
			rightValid = b2a2Valid;
		} else {
			x1l = b2; x2l = x2; x1r = x1; x2r = b1;
			leftValid = b2a2Valid;
			rightValid = a1b1Valid;
		}
		if (leftValid)  {
			this->left->gather(x1l, x2l, triangles);
		}
		if (rightValid) {
			this->right->gather(x1r, x2r, triangles);
		}
	}		
}

bool KDTreeNode::intersectSelf(aiVector3D &x1, aiVector3D &x2, aiVector3D &result, int &triangle) {
	float bestAlpha = 2.0;
	int bestTriangle = -1;
	for (std::list<KDTreeItem>::iterator i=this->items.begin();
		 i != this->items.end();
		 ++i) {
		float alpha;
		KDTreeTriangle *triPtr = &(this->tree->triangles[i->triangle]);
		//if (i->triangle == 5311 || i->triangle == 5590)
		//	printf("hello");
		if (lineTriangleIntersect1(x1, x2, triPtr->vertexes[0], triPtr->vertexes[1], triPtr->vertexes[2], alpha)) {
			if (alpha < bestAlpha) {
				bestAlpha = alpha;
				bestTriangle = i->triangle;
			}
		}
	}
	if (bestAlpha >= 0.0 && bestAlpha <= 1.0) {
		result = x1 + bestAlpha * (x2-x1);
		triangle = bestTriangle;
		return true;
	}
	return false;
}

bool KDTreeNode::intersectChildren(aiVector3D &x1, aiVector3D &x2, aiVector3D &result, int &triangle) {
	// create two line segments
	aiVector3D b1, b2;
	bool a1b1Valid = false, b2a2Valid = false;
	bool direction;
	aiVector3D leftResult, rightResult;
	int leftTriangle, rightTriangle;
	bool leftIntersects=false, rightIntersects=false;

	// split line segment in two
	linePlaneIntersect(x1, x2, b1, b2, a1b1Valid, b2a2Valid, direction, this->splitDirection-1, this->splitPoint);

	aiVector3D x1l, x2l;
	aiVector3D x1r, x2r;
	bool leftValid, rightValid;
	
	if (direction) {
		x1l = x1; x2l = b1; x1r = b2; x2r = x2;
		leftValid = a1b1Valid;
		rightValid = b2a2Valid;
	} else {
		x1l = b2; x2l = x2; x1r = x1; x2r = b1;
		leftValid = b2a2Valid;
		rightValid = a1b1Valid;
	}
	if (leftValid)  {
		leftIntersects = this->left->intersect(x1l, x2l, leftResult, leftTriangle);
	}
	if (rightValid) {
		rightIntersects = this->right->intersect(x1r, x2r, rightResult, rightTriangle);
	}
	if (leftIntersects && rightIntersects) {
		// find closest
		aiVector3D leftDelta = leftResult - x1;
		aiVector3D rightDelta = rightResult - x1;
		if (leftDelta.SquareLength() < rightDelta.SquareLength()) {
			triangle = leftTriangle;
			result = leftResult;
		} else {
			triangle = rightTriangle;
			result = rightResult;
		}
		return true;
	}
	if (leftIntersects) {
		triangle = leftTriangle;
		result = leftResult;
		return true;
	}
	if (rightIntersects) {
		triangle = rightTriangle;
		result = rightResult;
		return true;
	}
	return false;
}

bool KDTreeNode::intersect(aiVector3D &x1, aiVector3D &x2, aiVector3D &result, int &triangle) {
	bool selfIntersects;
	int selfTriangle;
	aiVector3D selfResult;
	bool childIntersects = false;
	int childTriangle;
	aiVector3D childResult;
	// first intersect with triangles at node
	selfIntersects = this->intersectSelf(x1, x2, selfResult, selfTriangle);
	// then go for children
	if (this->splitDirection == SPLIT_X ||
		this->splitDirection == SPLIT_Y ||
		this->splitDirection == SPLIT_Z) {
		childIntersects = this->intersectChildren(x1, x2, childResult, childTriangle);
	}
	if (selfIntersects && childIntersects) {
		aiVector3D selfDelta = selfResult - x1;
		aiVector3D childDelta = childResult - x1;
		if (selfDelta.SquareLength() < childDelta.SquareLength()) {
			result = selfResult;
			triangle = selfTriangle;
		} else {
			result = childResult;
			triangle = childTriangle;
		}
		return true;
	}
	if (selfIntersects) {
		result = selfResult;
		triangle = selfTriangle;
		return true;
	}
	if (childIntersects) {
		result = childResult;
		triangle = childTriangle;
		return true;
	}
	return false;
}

int KDTreeNode::numItems(void) {
	bool hasSplit = (this->splitDirection == SPLIT_X || 
					 this->splitDirection == SPLIT_Y ||
					 this->splitDirection == SPLIT_Z);
	if (hasSplit) {
		return this->left->numItems() + this->right->numItems();
	} else {
		return this->items.size();
	}
}

int KDTreeNode::numNodes(void) {
	bool hasSplit = (this->splitDirection == SPLIT_X || 
					 this->splitDirection == SPLIT_Y ||
					 this->splitDirection == SPLIT_Z);
	if (hasSplit) {
		return this->left->numNodes() + this->right->numNodes() + 1;
	} else {
		return 1;
	}
}

int KDTreeNode::depth(void) {
	bool hasSplit = (this->splitDirection == SPLIT_X || 
					 this->splitDirection == SPLIT_Y ||
					 this->splitDirection == SPLIT_Z);
	if (hasSplit) {
		return std::max(this->left->depth(),this->right->depth()) + 1;
	} else {
		return 0;
	}
}

int KDTreeNode::maxItems(void) {
	// return the maximum number of items at any node
	bool hasSplit = (this->splitDirection == SPLIT_X || 
					 this->splitDirection == SPLIT_Y ||
					 this->splitDirection == SPLIT_Z);
	if (hasSplit) {
		return std::max(this->left->maxItems(),this->right->maxItems());
	} else {
		return this->items.size();
	}
}