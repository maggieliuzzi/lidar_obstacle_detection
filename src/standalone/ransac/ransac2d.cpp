#include <unordered_set>
#include "../../render/render.h"
#include "../../processPointClouds.h"
#include "../../processPointClouds.cpp"  // using templates for processPointClouds, so including .cpp to help linker


pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData2D()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());

  	// Adding inliers
  	float scatter = 0.6;
  	for (int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}

  	// Adding outliers
  	int numOutliers = 10;
  	while (numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}

  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;
}


pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;

	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);

  	return viewer;
}


int main()
{
	// TODO: compare time of custom RANSAC implementation with PCL's built in RANSAC functions

	// Creating viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Creating data
	// pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData2D();  // 2D
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();  // 3D

	// std::unordered_set<int> inliers = Ransac2D(cloud, 50, 0.5);  // 2D  // TOTRY: max interations: 10, distance tolerance: 1.0
	std::unordered_set<int> inliers = Ransac3D(cloud, 50, 0.3);  // 3D  // TOTRY: 0.5

	std::cout << "Final inliersResult.size(): " << inliers.size() << std::endl;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for (int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];

		if (inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}

	std::cout << "cloud->points.size(): " << cloud->points.size() << std::endl;
	std::cout << "cloudInliers->points.size(): " << cloudInliers->points.size() << std::endl;
	std::cout << "cloudOutliers->points.size(): " << cloudOutliers->points.size() << std::endl;

	// Rendering 2D point cloud with inliers and outliers
	if (inliers.size())
	{
		renderPointCloud(viewer, cloudInliers, "inliers", Color(0,1,0));
  		renderPointCloud(viewer, cloudOutliers, "outliers", Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer, cloud, "data");
  	}
	
  	while (!viewer->wasStopped())
  		viewer->spinOnce();
}
