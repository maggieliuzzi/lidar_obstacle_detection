/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"
#include <iostream>


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
			// 2D
			// uint cd = depth % 2;  // depth even or odd? if 3D, % 3  // will always be 0 or 1 for 2D
			// 3D
			uint cd = depth % 3;  // will alwys be 0, 1 or 2

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

	void search2DHelper(std::vector<float> target, Node* node, uint depth, float distanceTol, std::vector<int>& ids)
	{
		// TODO: change to [0], [1], [2] for 3D tree

		std::cout << "\ndepth: " << depth << std::endl;

		if (node != NULL)
		{
			/*
				std::cout << "node->point[0]: " << node->point[0] << std::endl;
				std::cout << "target[0] - distanceTol: " << target[0] - distanceTol << std::endl;
				std::cout << "target[0] + distanceTol: " << target[0] + distanceTol << std::endl;
				std::cout << "node->point[1]: " << node->point[1] << std::endl;
				std::cout << "target[1] - distanceTol: " << target[1] - distanceTol << std::endl;
				std::cout << "target[1] + distanceTol: " << target[1] + distanceTol << std::endl;
			*/

			// Check if point in that node is inside the target box
			if ( (node->point[0] >= (target[0] - distanceTol) && node->point[0] <= (target[0] + distanceTol)) && (node->point[1] >= (target[1] - distanceTol) && node->point[1] <= (target[1] + distanceTol) ))
			{
				// Find distance between 2D points (node x, y and target x, y)
				float distance = sqrt( (node->point[0] - target[0]) * (node->point[0] - target[0]) + (node->point[1] - target[1]) * (node->point[1] - target[1]) );
				std::cout << "distance: " << distance << std::endl;
				if (distance <= distanceTol)
				{
					std::cout << "Adding index to cluster" << std::endl;
					ids.push_back(node->id);
				}
			}

			// Check box boundary to see where to flow in the tree (left or right)
			if ( (target[depth % 2] - distanceTol) < node->point[depth % 2] )  // depth%2 to check if we should do an x or y comparison  // if <, that box is in the left region
				std::cout << "Moving down (left)" << std::endl;
				search2DHelper(target, node->left, depth + 1, distanceTol, ids);
			if ( (target[depth % 2] + distanceTol) > node->point[depth % 2] )  // if left edge of the box is greater than the node's x or y value, then that box is in the right region
				std::cout << "Moving down (right)" << std::endl;
				search2DHelper(target, node->right, depth + 1, distanceTol, ids);
		}
	}

	void search3DHelper(std::vector<float> target, Node* node, uint depth, float distanceTol, std::vector<int>& ids)
	{
		// TODO: change to [0], [1], [2] for 3D tree

		std::cout << "\ndepth: " << depth << std::endl;

		if (node != NULL)
		{
			/*
				std::cout << "node->point[0]: " << node->point[0] << std::endl;
				std::cout << "target[0] - distanceTol: " << target[0] - distanceTol << std::endl;
				std::cout << "target[0] + distanceTol: " << target[0] + distanceTol << std::endl;
				std::cout << "node->point[1]: " << node->point[1] << std::endl;
				std::cout << "target[1] - distanceTol: " << target[1] - distanceTol << std::endl;
				std::cout << "target[1] + distanceTol: " << target[1] + distanceTol << std::endl;
			*/

			// Check if point in that node is inside the target box
			if ( (node->point[0] >= (target[0] - distanceTol) && node->point[0] <= (target[0] + distanceTol)) && (node->point[1] >= (target[1] - distanceTol) && node->point[1] <= (target[1] + distanceTol) ) && (node->point[2] >= (target[2] - distanceTol) && node->point[2] <= (target[2] + distanceTol) ) )
			{
				// Find distance between 3D points (node x, y, z and target x, y, z)
				float distance = sqrt( (node->point[0] - target[0]) * (node->point[0] - target[0]) + (node->point[1] - target[1]) * (node->point[1] - target[1]) + (node->point[2] - target[2]) * (node->point[2] - target[2]) );
				std::cout << "distance: " << distance << std::endl;
				if (distance <= distanceTol)
				{
					std::cout << "Adding index to cluster" << std::endl;
					ids.push_back(node->id);
				}
			}

			// Check box boundary to see where to flow in the tree (left or right)
			if ( (target[depth % 3] - distanceTol) < node->point[depth % 3] )  // depth%2 to check if we should do an x or y comparison  // if <, that box is in the left region
				std::cout << "Moving down (left)" << std::endl;
				search3DHelper(target, node->left, depth + 1, distanceTol, ids);
			if ( (target[depth % 3] + distanceTol) > node->point[depth % 3] )  // if left edge of the box is greater than the node's x or y value, then that box is in the right region
				std::cout << "Moving down (right)" << std::endl;
				search3DHelper(target, node->right, depth + 1, distanceTol, ids);
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;

		// 2D
		// search2DHelper(target, root, 0, distanceTol, ids); // passing ids by reference
		// 3D
		search3DHelper(target, root, 0, distanceTol, ids);

		return ids;
	}
};