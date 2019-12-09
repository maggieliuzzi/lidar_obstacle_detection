/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
// #include "render/render.h"
// #include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"


std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create LiDAR sensor 
    Lidar* lidarSensor = new Lidar(cars, 0);  // on heap. On stack (less space but faster lookups): Lidar lidarSensor = Lidar(cars, 0);

    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud = lidarSensor->scan();

    // Displaying rays coming out from the LiDAR sensor
    // renderRays(viewer, lidarSensor->position, pointCloud);

    // Displaying full point cloud
    // renderPointCloud(viewer, pointCloud, "pointCloudName");

    ProcessPointClouds<pcl::PointXYZ> pointCloudProcessor;  // Instantiating on the stack. On the heap: ProcessPointClouds<pcl::PointXYZ>* pointCloudProcessor = new ProcessPointClouds<pcl::PointXYZ>();

    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointCloudProcessor.SegmentPlane(pointCloud, 50, 0.3);  // 1000, 0.2  // . because instantiated on the stack. -> if on the heap  // TODO: go back to 100 iterations  // try: 50, 0.3
    // Displaying segmented obstacles and ground points
    // renderPointCloud(viewer, segmentCloud.first, "obstCloud", Color(0.8, 0.8, 0.8));
    // renderPointCloud(viewer, segmentCloud.second, "groundCloud");  // Color(0.3, 0.3, 0.3)


    // Clustering with PCL
    /*
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointCloudProcessor.PCLClustering(segmentCloud.first, 1.5, 3, 30);  // first: obstacles

    int clusterId = 0;
    std::vector<Color> colours = {Color(1, 0, 0), Color(0, 1, 0), Color(0, 0, 1)};

    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)  // iterating through vector of point clouds
    {
        std::cout << "cluster size ";
        pointCloudProcessor.numPoints(cluster);

        renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colours[clusterId % colours.size()]); // prev: clusterId

        Box box = pointCloudProcessor.BoundingBox(cluster);  // if pointCloudProcessor was a pointer, use ->
        renderBox(viewer, box, clusterId);

        ++clusterId;
    }
    */


    // Custom euclidean clustering implementation using 3D k-d tree
    //std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> obstacleClusters = pointCloudProcessor.Clustering(segmentCloud.first, 1.0); // TOTRY: (segmentCloud.first, 1.0, 3, 30);
    // PCL Euclidean Clustering
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> obstacleClusters = pointCloudProcessor.PCLClustering(segmentCloud.first, 1.0, 3, 30); // TOTRY: (segmentCloud.first, 1.0, 3, 30);
    int clusterId = 0;
    std::vector<Color> colours = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};

    for (pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : obstacleClusters)
    {
        renderPointCloud(viewer, cluster, "obstCloud"+std::to_string(clusterId), colours[clusterId % 3]);
    
        Box box = pointCloudProcessor.BoundingBox(cluster);  // if pointCloudProcessor was a pointer, use ->
        renderBox(viewer, box, clusterId);

        ++clusterId;
    }

    renderPointCloud(viewer, segmentCloud.second, "planeCloud");




    /*
    std::vector<std::vector<float>> pointsInPointCloud;
    // getting std::vector<std::vector<float>> from pcl::PointCloud<pcl::PointXYZ>::Ptr
    for (pcl::PointXYZ point : *segmentCloud.first)
    {
        std::vector<float> pointVector = {point.x, point.y, point.z};
        pointsInPointCloud.push_back(pointVector);
    }

    KdTree* tree = new KdTree;  // TODO: initalise directly inside of the clustering function

    for (int i = 0; i < pointsInPointCloud.size(); i++) 
    	tree->insert(pointsInPointCloud[i], i); 
  
    // Segmentation process
  	auto startTime = std::chrono::steady_clock::now();

  	std::vector<std::vector<int>> clusters = euclideanCluster(pointsInPointCloud, tree, 3.0);
	std::cout << "clusters.size(): " << clusters.size() << std::endl;

  	auto endTime = std::chrono::steady_clock::now();
  	auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  	std::cout << "Clustering found " << clusters.size() << " cluster(s) and took " << elapsedTime.count() << " milliseconds" << std::endl;

    // Render clusters
  	int clusterId = 0;
	std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};  // Only because I know there will be 3 clusters only
  	for(std::vector<int> cluster : clusters)
  	{
		std::cout << "cluster.size(): " << cluster.size() << std::endl;
  		pcl::PointCloud<pcl::PointXYZ>::Ptr clusteredObstacleCloud(new pcl::PointCloud<pcl::PointXYZ>());
  		for(int index : cluster)
  			clusteredObstacleCloud->pointsInPointCloud.push_back(pcl::PointXYZ(pointsInPointCloud[index][0], pointsInPointCloud[index][1], pointsInPointCloud[index][2]));
		std::cout << "clusterId % 3: " << clusterId % 3 << std::endl;
		std::cout << "clusteredObstacleCloud->points.size(): " << clusteredObstacleCloud->pointsInPointCloud.size() << std::endl;
  		renderPointCloud(viewer, clusteredObstacleCloud, "cluster_" + std::to_string(clusterId), colors[clusterId % 3]);
  		++clusterId;
  	}
    
  	if(clusters.size()==0)
	{
	  	std::cout << "clusters.size() == 0" << std::endl;
  		renderPointCloud(viewer, cloud, "data");  // default color is white
	}
    */

}


//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor(0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    simpleHighway(viewer);

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce ();
    } 
}