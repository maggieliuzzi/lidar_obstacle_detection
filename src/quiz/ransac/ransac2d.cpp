#include "../../render/render.h"
#include <unordered_set>
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

std::unordered_set<int> Ransac2D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::cout << "Total cloud->points.size(): " << cloud->points.size() << std::endl;

	std::unordered_set<int> inliersResult;  // starts as 0
	srand(time(NULL));
	
	// For max iterations
	// Randomly sample subset and fit line
	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier
	// Return indicies of inliers from fitted line with most inliers

	while (maxIterations--)  // > 0
	{
		std::unordered_set<int> inliers;  // hash set - order doesn't matter, we're just hashing into the index  // in sets, elements have to be unique, else it won't insert them
		while(inliers.size() <= 2)
			inliers.insert(rand()%(cloud->points.size()));  // using modulo, value between 0 and the size of cloud  // inliers will hold the indices of points

		float x1, y1, x2, y2;

		std::cout << "inliers.size(): " << inliers.size() << std::endl;
		auto itr = inliers.begin();  // pointer to the beginning of inliers
		x1 = cloud->points[*itr].x;  // checking what value it is by dereferencing the pointer
		y1 = cloud->points[*itr].y;
		itr++;  // incrementing the iterator by one
		x2 = cloud->points[*itr].x;
		y2 = cloud->points[*itr].y;

		float a = static_cast<float>(y1 - y2);
		float b = static_cast<float>(x2 - x1);
		float c = static_cast<float>(x1 * y2 - x2 * y1);

		for (int i = 0; i < cloud->points.size(); i++)
		{
			if (inliers.count(i) > 0) // if point is one of the two points that make the line
				continue;

			pcl::PointXYZ point = cloud->points[i];
			std::cout << "point: " << point << std::endl;

			float x3 = point.x; // member x from point
			float y3 = point.y;
			// z in this case is 0 because it's 2D

			float distance = fabs(a * x3 + b * y3 + c) / sqrtf(static_cast<float>(a * a + b * b)); // float absolute
			std::cout << "distance: " << distance << std::endl;

			if (distance <= distanceTol)
				inliers.insert(i);
		}

		std::cout << "Fitted line: inliers.size(): " << inliers.size() << std::endl;

		if (inliers.size() > inliersResult.size())
		{
			inliersResult = inliers;
		}
	}

	return inliersResult;
}

std::unordered_set<int> Ransac3D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::cout << "Total cloud->points.size(): " << cloud->points.size() << std::endl;

	std::unordered_set<int> inliersResult;  // starts as 0
	srand(time(NULL));

	// For max iterations
	// Randomly sample subset and fit line
	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier
	// Return indicies of inliers from fitted line with most inliers

	while (maxIterations--) // > 0
	{
		std::unordered_set<int> inliers;  // hash set - order doesn't matter, we're just hashing into the index  // in sets, elements have to be unique, else it won't insert them
		while(inliers.size() <= 3)
			inliers.insert(rand()%(cloud->points.size()));  // using modulo, value between 0 and the size of cloud  // inliers will hold the index of points

		float x1, y1, z1, x2, y2, z2, x3, y3, z3;

		std::cout << "inliers.size(): " << inliers.size() << std::endl;
		auto itr = inliers.begin();  // pointer to the beginning of inliers
		x1 = cloud->points[*itr].x;  // checking what value it is by dereferencing the pointer
		y1 = cloud->points[*itr].y;
        z1 = cloud->points[*itr].z;
		itr++;  // incrementing the iterator by one
		x2 = cloud->points[*itr].x;
		y2 = cloud->points[*itr].y;
        z2 = cloud->points[*itr].z;
		itr++;
		x3 = cloud->points[*itr].x;
		y3 = cloud->points[*itr].y;
        z3 = cloud->points[*itr].z;

        // 3D
        double u1, u2, u3, v1, v2, v3;
        u1 = (x2 - x1);
        u2 = (y2 - y1);
        u3 = (z2 - z1);
        v1 = (x3 - x1);
        v2 = (y3 - y1);
        v3 = (z3 - z1);
        // v1 [3] = {u1, u2, u3};
        // v2 [3] = {v1, v2, v3};

        double i, j, k;
        i = u2 * v3 - v2 * u3;
        j = v1 * u3 - u1 * v3;
        k = u1 * v2 - v1 * u2;

        // crossProd [3] = {i, j, k};

        double a, b, c, d;
		a = i;
		b = j;
		c = k;
        d = -1.0 * (i * x1 + j * y1 + k * z1);

		for (int i = 0; i < cloud->points.size(); i++)
		{
			if (inliers.count(i) > 0)  // if point is one of the two points that make the line
				continue;

			pcl::PointXYZ point = cloud->points[i];
			std::cout << "Point: " << point << std::endl;

			float x3 = point.x;  // member x from point
			float y3 = point.y;
            float z3 = point.z;

			double distance = fabs(a * x3 + b * y3 + c * z3 + d) / sqrt(a * a + b * b + c * c); 
			
			std::cout << "Distance: " << distance << std::endl;

			if (distance <= distanceTol)
				inliers.insert(i);
		}

		std::cout << "Fitted line: inliers.size(): " << inliers.size() << std::endl;

		if (inliers.size() > inliersResult.size())
		{
			inliersResult = inliers;
		}
	}

	return inliersResult;
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
  	{
  	  viewer->spinOnce ();
  	}
}
