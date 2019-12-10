/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
// #include "render/render.h"
// #include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"


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
    // Virtually-simulated highway environment

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
    
    //std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointCloudProcessor.PCLClustering(segmentCloud.first, 1.5, 3, 30);  // first: obstacles

    //int clusterId = 0;
    //std::vector<Color> colours = {Color(1, 0, 0), Color(0, 1, 0), Color(0, 0, 1)};
    //for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)  // iterating through vector of point clouds
    //{
    //    std::cout << "cluster size ";
    //    pointCloudProcessor.numPoints(cluster);

    //    renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colours[clusterId % colours.size()]); // prev: clusterId

    //    Box box = pointCloudProcessor.BoundingBox(cluster);  // if pointCloudProcessor was a pointer, use ->
    //    renderBox(viewer, box, clusterId);

    //    ++clusterId;
    //}


    // Custom euclidean clustering implementation using 3D k-d tree
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> obstacleClusters = pointCloudProcessor.Clustering(segmentCloud.first, 1.0); // TOTRY: (segmentCloud.first, 1.0, 3, 30);
    // PCL Euclidean Clustering
    // std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> obstacleClusters = pointCloudProcessor.PCLClustering(segmentCloud.first, 1.0, 3, 30); // TOTRY: (segmentCloud.first, 1.0, 3, 30);
    
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
}


void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointCloudXYZIProcessor, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{
    // Opens 3D view and displays city block

    // ProcessPointClouds<pcl::PointXYZI>* pointCloudXYZIProcessor = new ProcessPointClouds<pcl::PointXYZI>(); // on the heap, it will be a pointer, we use new and *  // on the stack: no pointer, no *, no = ..., finishes there ;
    // ProcessPointClouds<pcl::PointXYZI> pointCloudXYZIProcessor;

    // PCL with ~120,000 points
    // pcl::PointCloud<pcl::PointXYZI>::Ptr pcdCloud = pointCloudXYZIProcessor->loadPcd("/Users/maggieliuzzi/Documents/ComputerVision/SensorFusionNanodegree/SFND_Lidar_Obstacle_Detection/src/sensors/data/pcd/data_1/0000000000.pcd");
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcdCloud = pointCloudXYZIProcessor.loadPcd("/Users/maggieliuzzi/Documents/ComputerVision/SensorFusionNanodegree/SFND_Lidar_Obstacle_Detection/src/sensors/data/pcd/data_1/0000000000.pcd");
    // renderPointCloud(viewer, pcdCloud, "pcdCloud");  // if no colour specified, defaults to colour coding from Intensity component

    pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud = pointCloudXYZIProcessor.FilterCloud(pcdCloud, 0.5, Eigen::Vector4f (-10, -10, -5, 1), Eigen::Vector4f (30, 10, 10, 1));
    renderPointCloud(viewer, filteredCloud, "filteredCloud");


    /*
    // Segmentation
    // std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentedCloud = pointCloudXYZIProcessor.SegmentXYZIPlane(filteredCloud, 50, 0.3);
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentedCloud;

    // Clustering

    // PCL Euclidean Clustering  // TODO: try with custom implementation
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> obstacleClusters = pointCloudXYZIProcessor.PCLClustering(segmentedCloud.first, 1.0, 3, 30);
    // TODO: use minSize for cleaning up points scattered off in the edge and distanceTolerance for something
    
    int clusterId = 0;
    std::vector<Color> colours = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};  // TODO: add more colours to cycle through

    for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : obstacleClusters)
    {
        renderPointCloud(viewer, cluster, "obstCloud"+std::to_string(clusterId), colours[clusterId % 3]);
    
        // Bounding boxes
        // Since all the detectable vehicles in this scene are along the same axis as our car, the simple already set up bounding box function in point processor should yield good results.
        Box box = pointCloudXYZIProcessor.BoundingBox(cluster);  // if pointCloudProcessor was a pointer, use ->
        renderBox(viewer, box, clusterId);

        ++clusterId;
    }

    renderPointCloud(viewer, segmentedCloud.second, "groundPlaneCloud");
    */


    // If XYZ, all points are rendered in white
    // PointXYZI template argument
    // Point process pipeline
    // Sensor fusion pipeline
}



int main (int argc, char** argv)
{
    std::cout << "Starting enviroment..." << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    // Simulated pointcloud
    // simpleHighway(viewer);
    // Real pointcloud
    cityBlock(viewer);

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce ();
    } 
}