#include <iostream>
#include "../../render/render.h"


template <typename PointT>
struct NodePoint  // node of k-d tree
{
	PointT point;
	int id;
	NodePoint* left;
	NodePoint* right;

	NodePoint(PointT point, int setId)
	:	point(point), id(setId), left(NULL), right(NULL)
	{}
};


template <typename PointT>
struct PointTKdTree
{
	NodePoint<PointT>* root;

	PointTKdTree()
	: root(NULL)
	{}
	

    void insertHelperPoint(NodePoint<PointT>** node, uint depth, PointT point, int id)  // double pointer (NodePoint**) because root was defined as NodePoint* (node pointer) originally, and then we pass the memory address
	{
		if (*node == NULL)  // dereferencing to get value  // terminates when it hits a null node
		{
			*node = new NodePoint<PointT>(point, id);  // pointing root pointer to new data  // TOTRY: use a pointer reference instead, then no need to dereference it (*node)
		}
		else  // traverse
		{
			uint cd = depth % 3;  // 3D, always 0, 1 or 2  // for 2D: % 2, even or odd

            /*
            float pointVariable;
            if (cd == 0)
                pointVariable = point.x;
            else if (cd == 1)
                pointVariable = point.y;
            else if (cd == 2)
                pointVariable = point.z;
            */

            if (cd == 0)  // point[0]: x value
            {
                if (point.x < (*node)->point.x)
                    insertHelperPoint(&((*node)->left), depth + 1, point, id);  // passing address of dereferenced node's left child
                else
                    insertHelperPoint(&((*node)->right), depth + 1, point, id);
            }
            else if (cd == 1)
            {
                if (point.y < (*node)->point.y)
                    insertHelperPoint(&((*node)->left), depth + 1, point, id);
                else
                    insertHelperPoint(&((*node)->right), depth + 1, point, id);
            }
            else if (cd == 2)
            {
                if (point.z < (*node)->point.z)
                    insertHelperPoint(&((*node)->left), depth + 1, point, id);
                else
                    insertHelperPoint(&((*node)->right), depth + 1, point, id);
            }
		}
	}


    void insertPoint(PointT point, int id)
	{
		insertHelperPoint(&root, 0, point, id);  // passing the address of root
	}


	void search3DHelper(PointT target, NodePoint<PointT>* node, uint depth, float distanceTol, typename pcl::PointCloud<PointT>& nearbyPoints)
	{
		std::cout << "\nDepth: " << depth << std::endl;

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

			// Checking if point in that node is inside the target box
			if ( (node->point.x >= (target.x - distanceTol) && node->point.x <= (target.x + distanceTol)) && (node->point.y >= (target.y - distanceTol) && node->point.y <= (target.y + distanceTol) ) && (node->point.z >= (target.z - distanceTol) && node->point.z <= (target.z + distanceTol) ) )
			{
				// Finding distance between 3D points (node x, y, z and target x, y, z)
				float distance = sqrt( (node->point.x - target.x) * (node->point.x - target.x) + (node->point.y - target.y) * (node->point.y - target.y) + (node->point.z - target.z) * (node->point.z - target.z) );
				std::cout << "Distance: " << distance << std::endl;
				if (distance <= distanceTol)
				{
					std::cout << "Adding index to cluster" << std::endl;
					nearbyPoints.points.push_back(node->point);
				}
			}

			uint pointVariable = depth % 3;  // % 3 to check whether to do x, y or z comparison  // if 2D: % 2, x or y

			// Checking box boundary to see where to flow in the tree (left or right)
			if (pointVariable == 0)
			{
				if ( (target.x - distanceTol) < node->point.x )  // if <, that box is in the left region
					std::cout << "Moving down (left)" << std::endl;
					search3DHelper(target, node->left, depth + 1, distanceTol, nearbyPoints);
				if ( (target.x + distanceTol) > node->point.x )  // if left edge of the box is greater than the node's x or y value, then that box is in the right region
					std::cout << "Moving down (right)" << std::endl;
					search3DHelper(target, node->right, depth + 1, distanceTol, nearbyPoints);
			}
			else if (pointVariable == 1)
			{
				if ( (target.y - distanceTol) < node->point.y )
					std::cout << "Moving down (left)" << std::endl;
					search3DHelper(target, node->left, depth + 1, distanceTol, nearbyPoints);
				if ( (target.y + distanceTol) > node->point.y )
					std::cout << "Moving down (right)" << std::endl;
					search3DHelper(target, node->right, depth + 1, distanceTol, nearbyPoints);
			}
			else if (pointVariable == 2)
			{
				if ( (target.z - distanceTol) < node->point.z )
					std::cout << "Moving down (left)" << std::endl;
					search3DHelper(target, node->left, depth + 1, distanceTol, nearbyPoints);
				if ( (target.z + distanceTol) > node->point.z )
					std::cout << "Moving down (right)" << std::endl;
					search3DHelper(target, node->right, depth + 1, distanceTol, nearbyPoints);
			}
		}
	}


	typename pcl::PointCloud<PointT> search(PointT target, float distanceTol)
	{
		pcl::PointCloud<PointT> nearbyPoints;  // list of points in the tree that are within certain distance of target

		search3DHelper(target, root, 0, distanceTol, nearbyPoints);

		return nearbyPoints;
	}
};
