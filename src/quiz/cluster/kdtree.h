/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	void insertHelper(Node** node, uint depth, std::vector<float> point, int id)
	{
		// double pointer (Node**), because root was defined as Node* (node pointer) originally, and then we pass the memory address
		if (*node == NULL)  // dereferencing to see value  // terminates when it hits a null node
		{
			*node = new Node(point, id);  // pointing root pointer to new data  // could use a pointer reference instead, then no need to dereference it (*node)
		}
		else  // traverse
		{
			// Calculate current dimensions
			uint cd = depth % 2;  // depth even or odd? if 3D, % 3  // will always be 0 or 1
			
			if (point[cd] < (*node)->point[cd])  // if even // point[0]: x value
				insertHelper(&((*node)->left), depth + 1, point, id);  // passing address of dereferenced node's left child
			else
				insertHelper(&((*node)->right), depth + 1, point, id);
		}
	}

	void insert(std::vector<float> point, int id)
	{
		// Takes a 2D point represented by a vector containing two floats, and a point ID
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place it correctly within the root 

		insertHelper(&root, 0, point, id);  // passing the address of root
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		return ids;
	}
	

};




