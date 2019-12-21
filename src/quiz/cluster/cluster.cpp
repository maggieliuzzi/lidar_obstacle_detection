#include "../../render/box.h"
#include <chrono>
#include <string>
#include "cluster/kdtree.h"


// TODO: check that all multi-line if and for loops have curly brackets (else the 2nd+ statement inside will be executed regardless)

pcl::visualization::PCLVisualizer::Ptr initScene(Box window, int zoom)
{
	/* Inputs:
	Window: region to draw box around
	Zoom: correlated with how much of the area can be seen
	*/

	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("2D Viewer"));  // TODO: fix for 3D Viewer
	viewer->setBackgroundColor(0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, zoom, 0, 1, 0);
  	viewer->addCoordinateSystem(1.0);

	// viewer->addCube(window.x_min, window.x_max, window.y_min, window.y_max, 0, 0, 0.9, 0.9, 0.9, "window");  // TODO: for 3D Viewer: window.z_min, window.z_max, else: 0, 0
  	
	return viewer;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData(std::vector<std::vector<float>> points)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	
  	for (int i = 0; i < points.size(); i++)
  	{
  		pcl::PointXYZ point;
  		point.x = points[i][0];
  		point.y = points[i][1];
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;
}

void render2DTree(Node* node, pcl::visualization::PCLVisualizer::Ptr& viewer, Box window, int& iteration, uint depth=0)
{
	if (node != NULL)
	{
		Box upperWindow = window;
		Box lowerWindow = window;
		
		if (depth % 2==0)  // split on x axis
		{
			viewer->addLine(pcl::PointXYZ(node->point[0], window.y_min, 0),pcl::PointXYZ(node->point[0], window.y_max, 0),0,0,1,"line"+std::to_string(iteration));
			lowerWindow.x_max = node->point[0];
			upperWindow.x_min = node->point[0];
		}
		else  // split on y axis
		{
			viewer->addLine(pcl::PointXYZ(window.x_min, node->point[1], 0),pcl::PointXYZ(window.x_max, node->point[1], 0),1,0,0,"line"+std::to_string(iteration));
			lowerWindow.y_max = node->point[1];
			upperWindow.y_min = node->point[1];
		}
		iteration++;

		render2DTree(node->left, viewer, lowerWindow, iteration, depth+1);
		render2DTree(node->right, viewer, upperWindow, iteration, depth+1);
	}
}

// TOCHECK: if still relevant: TODO: fix
void render3DTree(Node* node, pcl::visualization::PCLVisualizer::Ptr& viewer, Box window, int& iteration, uint depth=0)
{
	if (node != NULL)
	{
		Box upperWindow = window;
		Box lowerWindow = window;
		
		if (depth % 3 == 0)  // split on x axis
		{
			viewer->addLine(pcl::PointXYZ(node->point[0], window.y_min, window.z_min),pcl::PointXYZ(node->point[0], window.y_max, window.z_max), 0, 0, 1, "line" + std::to_string(iteration));
			lowerWindow.x_max = node->point[0];
			upperWindow.x_min = node->point[0];
		}
		else if (depth % 2 == 0)  // split on y axis  // TODO: check if correct
		{
			viewer->addLine(pcl::PointXYZ(window.x_min, node->point[1], window.z_min),pcl::PointXYZ(window.x_max, node->point[1], window.z_max), 1, 0, 0, "line" + std::to_string(iteration));
			lowerWindow.y_max = node->point[1];
			upperWindow.y_min = node->point[1];
		}
		else  // split on z axis
		{
			viewer->addLine(pcl::PointXYZ(window.x_min, window.y_min, node->point[2]),pcl::PointXYZ(window.x_max, window.y_max, node->point[2]), 0, 1, 0, "line" + std::to_string(iteration));
			lowerWindow.z_max = node->point[2];
			upperWindow.z_min = node->point[2];
		}
		iteration++;

		render3DTree(node->left, viewer, lowerWindow, iteration, depth+1);
		render3DTree(node->right, viewer, upperWindow, iteration, depth+1);
	}
}

void proximity(int index, const std::vector<std::vector<float>> points, std::vector<int>& cluster, std::vector<bool>& processed, KdTree* tree, float distanceTolerance)
{
	/* euclideanClustering helper */

	if (!processed[index])  // TODO: remove, always true
	{
		processed[index] = true;
		cluster.push_back(index);

		std::vector<int> nearby = tree->search(points[index], distanceTolerance);

		for(int index : nearby)
		{
			if (!processed[index])  // if nearby point hasn't been processed yet
				proximity(index, points, cluster, processed, tree, distanceTolerance);
		}
	}
}

std::vector<std::vector<int>> euclideanClustering(const std::vector<std::vector<float>> points, KdTree* tree, float distanceTol)
{
	std::vector<std::vector<int>> clusters;  // list of indices for each cluster

	std::vector<bool> processed(points.size(), false);  // same size as points, each of which starts as false

	int i = 0;
	while (i < points.size())
	{
		if (processed[i])
		{
			i++;
			continue;
		}
		std::vector<int> cluster;
		proximity(i, points, cluster, processed, tree, distanceTol);  // i: point id, cluster passed by reference
		clusters.push_back(cluster);
		i++;
	}

	return clusters;
}


int main()
{
	// Creating viewer

	Box window;
  	window.x_min = -10;
  	window.x_max =  10;
  	window.y_min = -10;
  	window.y_max =  10;
  	window.z_min = -10;
  	window.z_max =  10;
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene(window, 25);

	// Creating data

	// std::vector<std::vector<float>> points = { {-6.2,7}, {-6.3,8.4}, {-5.2,7.1}, {-5.7,6.3}, {7.2,6.1}, {8.0,5.3}, {7.2,7.1}, {0.2,-7.1}, {1.7,-6.9}, {-1.2,-7.2}, {2.2,-8.9} };  // 2D
	std::vector<std::vector<float>> points = { {-6.2,7,3}, {-6.3,8.4,7}, {-5.2,7.1,6}, {-5.7,6.3,3}, {7.2,6.1,7}, {8.0,5.3,1}, {7.2,7.1,6}, {0.2,-7.1,2}, {1.7,-6.9,8}, {-1.2,-7.2,3}, {2.2,-8.9,2} };  // 3D
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData(points);

	// Creating and populating k-d tree

	KdTree* tree = new KdTree;
  
    for (int i = 0; i < points.size(); i++) 
    	tree->insert(points[i], i); 

	// Visualising k-d tree

	int it = 0;
  	// render2DTree(tree->root, viewer, window, it);  // 2D
	render3DTree(tree->root, viewer, window, it);  // 3D

	// Just for testing that k-d tree search works
	/*
  	std::cout << "Point ids found with tree->search(): " << std::endl;
	// std::vector<std::vector<float>> targetPointVector = {{0, -7}};  // 2D
	std::vector<std::vector<float>> targetPointVector = {{-6, 7, 3}};  // 3D

	pcl::PointCloud<pcl::PointXYZ>::Ptr targetPoint = CreateData(targetPointVector);
	renderPointCloud(viewer, targetPoint, "targetPoint", Color(1, 0, 0));

	float distanceTolerance = 3.0;
  	std::vector<int> nearby = tree->search(targetPointVector[0], distanceTolerance);  // with {-6, 7}, 3.0: 0 1 2 3
  	for(int index : nearby)
      std::cout << index << " ";
  	std::cout << std::endl;
	*/

  	// Segmentating points

  	auto startTime = std::chrono::steady_clock::now();

  	std::vector<std::vector<int>> clusters = euclideanClustering(points, tree, 3.0);
	std::cout << "clusters.size(): " << clusters.size() << std::endl;

  	auto endTime = std::chrono::steady_clock::now();
  	auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  	std::cout << "Clustering found " << clusters.size() << " cluster(s) and took " << elapsedTime.count() << " milliseconds" << std::endl;

  	// Rendering clusters in viewer

  	int clusterId = 0;
	std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};
  	for (std::vector<int> cluster : clusters)
  	{
		std::cout << "cluster.size(): " << cluster.size() << std::endl;
  		pcl::PointCloud<pcl::PointXYZ>::Ptr clusterCloud(new pcl::PointCloud<pcl::PointXYZ>());
  		for(int index : cluster)
			// clusterCloud->points.push_back(pcl::PointXYZ(points[index][0], points[index][1], 0));  // 2D
			clusterCloud->points.push_back(pcl::PointXYZ(points[index][0], points[index][1], points[index][2]));  // 3D
		std::cout << "clusterId % 3: " << clusterId % 3 << std::endl;
		std::cout << "clusterCloud->points.size(): " << clusterCloud->points.size() << std::endl;
  		renderPointCloud(viewer, clusterCloud, "cluster_" + std::to_string(clusterId), colors[clusterId % 3]);
  		++clusterId;
  	}

  	if (clusters.size() == 0)
	{
	  	std::cout << "clusters.size() == 0" << std::endl;
  		renderPointCloud(viewer, cloud, "data", Color(0, 0, 0));
	}

  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce();
  	}
}
