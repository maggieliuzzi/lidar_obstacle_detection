/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../render/render.h"
#include <iostream>


// Structure to represent node of kd tree
struct NodePointXYZ
{
	pcl::PointXYZ point;
	int id;
	NodePointXYZ* left;
	NodePointXYZ* right;

	NodePointXYZ(pcl::PointXYZ pointXYZ, int setId)
	:	point(pointXYZ), id(setId), left(NULL), right(NULL)
	{}
};


struct KdTree
{
	NodePointXYZ* root;

	KdTree()
	: root(NULL)
	{}


    void insertHelperPointXYZ(NodePointXYZ** node, uint depth, pcl::PointXYZ point, int id)
	{
		// double pointer (NodePointXYZ**), because root was defined as NodePointXYZ* (node pointer) originally, and then we pass the memory address
		if (*node == NULL)  // dereferencing to see value  // terminates when it hits a null node
		{
			*node = new NodePointXYZ(point, id);  // pointing root pointer to new data  // could use a pointer reference instead, then no need to dereference it (*node)
		}
		else  // traverse
		{
			// Calculate current dimensions
			// 2D
			// uint cd = depth % 2;  // depth even or odd? if 3D, % 3  // will always be 0 or 1 for 2D
			// 3D
			uint cd = depth % 3;  // will alwys be 0, 1 or 2

            /*
            float pointVariable;
            if (cd == 0)
                pointVariable = point.x;
            else if (cd == 1)
                pointVariable = point.y;
            else if (cd == 2)
                pointVariable = point.z;
            */
            if (cd == 0)
            {
                if (point.x < (*node)->point.x)  // if even // point[0]: x value
                    insertHelperPointXYZ(&((*node)->left), depth + 1, point, id);  // passing address of dereferenced node's left child
                else
                    insertHelperPointXYZ(&((*node)->right), depth + 1, point, id);
            }
            else if (cd == 1)
            {
                if (point.y < (*node)->point.y)  // if even // point[0]: x value
                    insertHelperPointXYZ(&((*node)->left), depth + 1, point, id);  // passing address of dereferenced node's left child
                else
                    insertHelperPointXYZ(&((*node)->right), depth + 1, point, id);
            }
            else if (cd == 2)
            {
                if (point.z < (*node)->point.z)  // if even // point[0]: x value
                    insertHelperPointXYZ(&((*node)->left), depth + 1, point, id);  // passing address of dereferenced node's left child
                else
                    insertHelperPointXYZ(&((*node)->right), depth + 1, point, id);
            }
		}
	}
    

    void insertPointXYZ(pcl::PointXYZ point, int id)
	{
		insertHelperPointXYZ(&root, 0, point, id);  // passing the address of root
	}


	void search3DHelper(pcl::PointXYZ target, NodePointXYZ* node, uint depth, float distanceTol, pcl::PointCloud<pcl::PointXYZ>& nearbyPoints)
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
			if ( (node->point.x >= (target.x - distanceTol) && node->point.x <= (target.x + distanceTol)) && (node->point.y >= (target.y - distanceTol) && node->point.y <= (target.y + distanceTol) ) && (node->point.z >= (target.z - distanceTol) && node->point.z <= (target.z + distanceTol) ) )
			{
				// Find distance between 3D points (node x, y, z and target x, y, z)
				float distance = sqrt( (node->point.x - target.x) * (node->point.x - target.x) + (node->point.y - target.y) * (node->point.y - target.y) + (node->point.z - target.z) * (node->point.z - target.z) );
				std::cout << "distance: " << distance << std::endl;
				if (distance <= distanceTol)
				{
					std::cout << "Adding index to cluster" << std::endl;
					nearbyPoints.points.push_back(node->point);  // TODO: make nearbyPoints a pointer?
				}
			}

			uint pointVariable = depth % 3;

			if (pointVariable == 0)
			{
				// Check box boundary to see where to flow in the tree (left or right)
				if ( (target.x - distanceTol) < node->point.x )  // depth%2 to check if we should do an x or y comparison  // if <, that box is in the left region
					std::cout << "Moving down (left)" << std::endl;
					search3DHelper(target, node->left, depth + 1, distanceTol, nearbyPoints);
				if ( (target.x + distanceTol) > node->point.x )  // if left edge of the box is greater than the node's x or y value, then that box is in the right region
					std::cout << "Moving down (right)" << std::endl;
					search3DHelper(target, node->right, depth + 1, distanceTol, nearbyPoints);
			}
			else if (pointVariable == 1)
			{
				// Check box boundary to see where to flow in the tree (left or right)
				if ( (target.y - distanceTol) < node->point.y )  // depth%2 to check if we should do an x or y comparison  // if <, that box is in the left region
					std::cout << "Moving down (left)" << std::endl;
					search3DHelper(target, node->left, depth + 1, distanceTol, nearbyPoints);
				if ( (target.y + distanceTol) > node->point.y )  // if left edge of the box is greater than the node's x or y value, then that box is in the right region
					std::cout << "Moving down (right)" << std::endl;
					search3DHelper(target, node->right, depth + 1, distanceTol, nearbyPoints);
			}
			else if (pointVariable == 2)
			{
				// Check box boundary to see where to flow in the tree (left or right)
				if ( (target.z - distanceTol) < node->point.z )  // depth%2 to check if we should do an x or y comparison  // if <, that box is in the left region
					std::cout << "Moving down (left)" << std::endl;
					search3DHelper(target, node->left, depth + 1, distanceTol, nearbyPoints);
				if ( (target.z + distanceTol) > node->point.z )  // if left edge of the box is greater than the node's x or y value, then that box is in the right region
					std::cout << "Moving down (right)" << std::endl;
					search3DHelper(target, node->right, depth + 1, distanceTol, nearbyPoints);
			}
		}
	}

	// return a list of pointXYZs in the tree that are within distance of target
	pcl::PointCloud<pcl::PointXYZ> search(pcl::PointXYZ target, float distanceTol)
	{
		pcl::PointCloud<pcl::PointXYZ> nearbyPoints;

		search3DHelper(target, root, 0, distanceTol, nearbyPoints);

		return nearbyPoints;
	}
};