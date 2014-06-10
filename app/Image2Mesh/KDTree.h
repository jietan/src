/*
 *  KDTree.h
 *  CS248-Final-Project
 *
 *  Created by zaltor on 2/17/11.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef _KDTREE_H_
#define _KDTREE_H_

#include "Framework.h"
#include <list>
#include <vector>
#include <set>

enum KDTreeSplit {
	SPLIT_NONE = 0,
	SPLIT_X = 1,
	SPLIT_Y = 2,
	SPLIT_Z = 3
};

// forward declarations to be filled later
class KDTreeItem;
class KDTreeNode;
class KDTree;

class KDTreeTriangle {
public:
	// A single triangle in the kd-tree
	// the transformed vertexes
	aiVector3D vertexes[3];
	// the following members are just for debugging
	// parent node
	const aiNode *parent;
	// parent node mesh number
	unsigned int meshNumber;
    
    aiVector3D normal;

	KDTreeTriangle ();
	~KDTreeTriangle ();
};

class KDTreeItem {
	// A single bounding box in the kd-tree, holding one triangle
public:
	KDTree *tree; // pointer to the kd-tree that this item is part of
	int triangle; // index to the triangle
	aiVector3D mins;
	aiVector3D maxes;

	KDTreeItem();
	~KDTreeItem();
	
	// associate this item with a specific tree and triangle
	void set(KDTree *tree, size_t triangle);
	// return true if bounding box doesn't contain triangle
	bool empty(void);
	// tighten bounding boxes
	void tighten(void);

	// split current triangle's bounding box across a plane
	// will set validLeft and validRight to true or false depending on whether left and right items
	// should be generated
	void split(KDTreeItem &left, KDTreeItem &right, 
			   KDTreeSplit axis, float point, 
			   bool &validLeft, bool &validRight);
	
};	

class KDTreeNode {
public:
	// the tree this node belongs to
	KDTree *tree;
	// (unprocessed) items at this kdtree node
	std::list<KDTreeItem> items;
	// bounding box of this node (calculated upon split, and can only grow after split)
	aiVector3D mins;
	aiVector3D maxes;
	// if the node is a branch, which way is it split?
	KDTreeSplit splitDirection;
	float splitPoint;
	// LT (left) and GEQ (right) nodes
	KDTreeNode *left;
	KDTreeNode *right;
	
	KDTreeNode (KDTree *tree);
	~KDTreeNode ();
	
	// update bounding box, expanding bounding box if necessary
	void updateBoundingBox(void);
	
	// return volume of the bounding box
	float volume(void);
	
	// process unprocessed triangles, creating a new split if necessary
	void process(int level=0);
	
	void gather(aiVector3D &x1, aiVector3D &x2, std::set<int> &triangles);
	
	// test to see if a line segment from x1 to x2 intersects, and record intersection point and triangle
	bool intersect(aiVector3D &x1, aiVector3D &x2, aiVector3D &result, int &triangle);
	// test to see if a line segment from x1 to x2 intersects, and record intersection point and triangle
	bool intersectSelf(aiVector3D &x1, aiVector3D &x2, aiVector3D &result, int &triangle);
	// test to see if a line segment from x1 to x2 intersects, and record intersection point and triangle
	bool intersectChildren(aiVector3D &x1, aiVector3D &x2, aiVector3D &result, int &triangle);
	
	// statistics about the tree
	int numItems(void);
	int numNodes(void);
	int depth(void);
	int maxItems(void);
};

class KDTree {
	// the main class for a single kd-tree
public:
	std::vector<KDTreeTriangle> triangles; // all the triangles in the tree
	KDTreeNode *root; // the root node of the tree
	
	KDTree ();
	~KDTree ();
	
	// add scene triangles to the kd-tree
	void addScene(const aiScene *scene);
	// add a single scene node
	void addNode(const aiScene *scene, const aiNode *node, aiMatrix4x4 &parentTransform);
	// add a single mesh to the current node
	void addMesh(const aiMesh *mesh, aiMatrix4x4 &transform, const aiNode *parent=NULL, unsigned int meshNumber=0);
	// create kd-tree branches if necessary after new triangles
	void process(void);

	// test whether a line segment intersects the kd-tree and set the intersection point and triangle id
	bool intersect(aiVector3D &x1, aiVector3D &x2, aiVector3D &result, int &triangle);
};


#endif