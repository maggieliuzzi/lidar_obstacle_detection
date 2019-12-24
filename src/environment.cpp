/*
    Obstacle detection pipeline on a cityscape environment using LiDAR:
        Streaming of PCD files, filtering, segmenting ground plane and obstacles, clustering obstacles and displaying bounding boxes around them

    Custom implementation of:
        RANSAC segmentation
        Euclidean clustering optimised with a k-d tree for more efficient nearby-neighbour search

    Uses PCL types and functions for:
        CropBox, VoxelGrid (for filtering)
        PointCloud, with PointXYZ and PointXYZI

Extra:
Rotating bounding boxes for more precise obstacle detection

Challenge problem:
Detecting and tracking a bicylist riding in front of the car, along with detecting/tracking the other surrounding obstacles in the scene, using data from src/sensors/data/pcd/data_2

TOTRYs:
Tracking detections over the history of frames:
    Creating associations between detections in frames and using that to track objects.
    One way to create associations between two different frames is by how close in proximity two detections are to each other and how similar they look. 
Exploring other filtering procedures
    Looking at detections that are seen in consecutive frames before they are considered. 
    Filtering based on bounding boxes, their volume and shapes. 
    Deploying tracking methods and associations to dynamically build the shapes of obstacles. 
        E.g. there is a long truck, but the LiDAR only sees the back of the truck at first. Later on, when driving past the truck, the LiDAR sees the truck's side.
*/


#include <pcl/visualization/pcl_visualizer.h>
#include "sensors/lidar.h"
#include "processPointClouds.h"
#include "processPointClouds.cpp"  // using templates for processPointClouds so including .cpp to help linker


void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    viewer->setBackgroundColor(0, 0, 0);
    viewer->initCameraParameters();  // set camera position and angle
    int distance = 16;  // meters
    
    switch (setAngle)  // switching camera angle
    {
        case XY: viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown: viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side: viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS: viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if (setAngle!=FPS)
        viewer->addCoordinateSystem(1.0);
}


std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    Car egoCar(Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1(Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2(Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3(Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if (renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}


void simulatedXYZHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    /* 
    Simulates PointXYZ highway environment
    */

    // Initialising scene

    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);

    Lidar lidarSensor = Lidar(cars, 0);  // on the heap: Lidar* lidarSensor = new Lidar(cars, 0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud = lidarSensor.scan();
    // renderRays(viewer, lidarSensor.position, pointCloud);  // displaying rays coming out from LiDAR sensor
    ProcessPointClouds<pcl::PointXYZ> pointCloudProcessor;  // on the heap: ProcessPointClouds<pcl::PointXYZ>* pointCloudProcessor = new ProcessPointClouds<pcl::PointXYZ>();

    // Segmenting pointcloud into ground and obstacles

    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointCloudProcessor.SegmentPlane(pointCloud, 50, 0.3);  // TODO: try 1000 or 100, and 0.2 or 0.3
    renderPointCloud(viewer, segmentCloud.second, "groundCloud");
    // renderPointCloud(viewer, segmentCloud.first, "obstacleCloud", Color(0.8, 0.8, 0.8));

    // Clustering obstacles
    
    // std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> obstacleClusters = pointCloudProcessor.PCLClustering(segmentCloud.first, 1.5, 3, 30);  // using PCL built-in functions  // TODO: try with 1.0
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> obstacleClusters = pointCloudProcessor.Clustering(segmentCloud.first, 1.0);  // custom euclidean-clustering implementation using 3D-tree  // TODO: add 3, 30  // TOFIX: not finding clusters

    // Rendering clusters and bounding boxes

    std::vector<Color> colours = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};
    int clusterId = 0;

    for (pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : obstacleClusters)
    {
        std::cout << "Rendering " << "obstacleCloud" + std::to_string(clusterId) << " cluster" << std::endl;
        renderPointCloud(viewer, cluster, "obstacleCloud" + std::to_string(clusterId), colours[clusterId % 3]);
    
        Box box = pointCloudProcessor.BoundingBox(cluster);
        renderBox(viewer, box, clusterId);

        ++clusterId;
    }
}


void objectDetectionXYZ(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZ> pointCloudXYZProcessor, const pcl::PointCloud<pcl::PointXYZ>::Ptr& inputCloud)  // TOTRY: ProcessPointClouds<pcl::PointXYZ>* pointCloudXYZProcessor (* if pointer)  // inputCloud as a constant reference (since it won't be changed within the function), for better memory efficiency/ slight performance increase, by not writing to that variable's memory, just reading from it.
{
    /* Performs object detection pipeline on a real PointXYZ PCD */

    // Filtering

    pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud = pointCloudXYZProcessor.FilterCloud(inputCloud, 0.3, Eigen::Vector4f(-15, -15, -10, 1), Eigen::Vector4f(30, 15, 10, 1));  // TODO: try 0.3, -10, -5, -2, 1, 30, 8, 1, 1
    // renderPointCloud(viewer, filteredCloud, "filteredCloud");

    // Segmentation

    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentedCloud = pointCloudXYZProcessor.SegmentPlane(filteredCloud, 50, 0.2);  // TOTRY: 10, 0.2
    renderPointCloud(viewer, segmentedCloud.second, "groundPlaneCloud");

    // Clustering

    // std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> obstacleClusters = pointCloudXYZProcessor.PCLClustering(segmentedCloud.first, 1.0, 3, 30);  // using built-in PCL euclidean-clustering functions  // TOTRY: 1.0, 0.53, 10, 500
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> obstacleClusters = pointCloudXYZProcessor.Clustering(segmentedCloud.first, 1.0);
    // TODO: use minSize for cleaning up points scattered off in the edge and distanceTolerance for something as well

    std::vector<Color> colours = {Color(1, 0, 0), Color(0, 1, 0), Color(0, 0, 1)};  // TODO: add more colours to cycle through
    int clusterId = 0;

    for (pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : obstacleClusters)
    {
        std::cout << "Cluster size: " << std::endl;
        pointCloudXYZProcessor.numPoints(cluster);

        renderPointCloud(viewer, cluster, "obstacleCloud" + std::to_string(clusterId), colours[clusterId % colours.size()]);
    
        // Bounding boxes

        // Since all the detectable vehicles in this scene are along the same axis as our car, the simple already-set-up bounding box function should yield good results.
        Box box = pointCloudXYZProcessor.BoundingBox(cluster);
        renderBox(viewer, box, clusterId);

        ++clusterId;
    }
}


void objectDetectionXYZI(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI> pointCloudXYZIProcessor, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)  // TOTRY: ProcessPointClouds<pcl::PointXYZI>* pointCloudXYZIProcessor (* if pointer)  // inputCloud as a constant reference (since it won't be changed within the function), for better memory efficiency/ slight performance increase, by not writing to that variable's memory, just reading from it.
{
    /* Performs object detection pipeline on a real PointXYZI PCD */

    // Downsampling and filtering
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud = pointCloudXYZIProcessor.FilterCloud(inputCloud, 0.5, Eigen::Vector4f(-15, -10, -5, 1), Eigen::Vector4f(30, 10, 5, 1));  // TODO: try 1.5, 0.3, 0.5 | -10, -5, -2, 1, 30, 8, 1, 1
    // renderPointCloud(viewer, filteredCloud, "filteredCloud");

    // Segmenting ground and obstacles

    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentedCloud = pointCloudXYZIProcessor.SegmentPlane(filteredCloud, 50, 0.2);  // TOTRY: 10, 0.2
    renderPointCloud(viewer, segmentedCloud.second, "groundPlaneCloud", Color(0, 1, 0));
    // renderPointCloud(viewer, segmentedCloud.first, "obstacleCloud", Color(1, 0, 0));

    // Clustering obstacle points into objects
  
    //std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> obstacleClusters = pointCloudXYZIProcessor.PCLClustering(segmentedCloud.first, 1.0, 3, 30);  // using built-in PCL euclidean-clustering functions  // TOTRY: 1.0, 0.53, 10, 500
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> obstacleClusters = pointCloudXYZIProcessor.Clustering(segmentedCloud.first, 0.5);
    // TODO: use minSize for cleaning up points scattered off in the edge and distanceTolerance for something as well

    std::vector<Color> colours = {Color(1, 0, 0), Color(0, 1, 0), Color(0, 0, 1)};  // TODO: add more colours to cycle through
    int clusterId = 0;

    for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : obstacleClusters)
    {
        std::cout << "Cluster size: " << std::endl;
        pointCloudXYZIProcessor.numPoints(cluster);

        // Rendering cluster
        renderPointCloud(viewer, cluster, "obstacleCloud" + std::to_string(clusterId), colours[clusterId % colours.size()]);
    
        // Rendering bounding box
        // Since all the detectable vehicles in this scene are along the same axis as our car, the simple already-set-up bounding box function should yield good results.
        Box box = pointCloudXYZIProcessor.BoundingBox(cluster);
        renderBox(viewer, box, clusterId);

        ++clusterId;
    }

    // TODO: if clustering parts of the wall, try changing size of box cropped, or look at dimensions and filter based on the bounding box. eg. if object's width < certain width (or certain dimensions), ignore
}



int main(int argc, char** argv)
{
    std::cout << "Starting environment..." << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);

    // simulatedXYZHighway(viewer);  // simulated PointXYZ pointcloud

    /* 
    // PointXYZ object detection

    ProcessPointClouds<pcl::PointXYZ> pointCloudXYZProcessor;  // using PointXYZI template argument  // on the heap: ProcessPointClouds<pcl::PointXYZI>* pointCloudXYZIProcessor = new ProcessPointClouds<pcl::PointXYZI>();
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcdCloud = pointCloudXYZProcessor.loadPcd("/Users/maggieliuzzi/Documents/ComputerVision/SensorFusionNanodegree/LiDAR/SFND_Lidar_Obstacle_Detection/src/sensors/data/pcd/simpleHighway.pcd");
    objectDetectionXYZ(viewer, pointCloudXYZProcessor, pcdCloud);  // obstacle-detection pipeline
    */

    ProcessPointClouds<pcl::PointXYZI> pointCloudXYZIProcessor;
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcdCloud;

    /*
    pcdCloud = pointCloudXYZIProcessor.loadPcd("/Users/maggieliuzzi/Documents/ComputerVision/SensorFusionNanodegree/LiDAR/SFND_Lidar_Obstacle_Detection/src/sensors/data/pcd/data_1/0000000000.pcd");  // PCD with ~120,000 points
    // renderPointCloud(viewer, pcdCloud, "pcdCloud");
    objectDetectionXYZI(viewer, pointCloudXYZIProcessor, pcdCloud);  // obstacle-detection pipeline
    */

    /*
    while (!viewer->wasStopped())  // PC Viewer run cycle  // frame update loop
    {
        viewer->spinOnce();  // Controls the frame rate. By default it waits 1 time step, which would make it run as fast as possible. Frame rate dependings on time efficiency of obstacle-detection functions  // TODO: check for possible optimisations
    }
    */

    // Passing path to directory containing sequentially-ordered PCD files. Returns a chronologically0ordered vector of all those file names
    std::vector<boost::filesystem::path> stream = pointCloudXYZIProcessor.streamPcd("/Users/maggieliuzzi/Documents/ComputerVision/SensorFusionNanodegree/LiDAR/SFND_Lidar_Obstacle_Detection/src/sensors/data/pcd/data_1");  // TODO: change to relative path
    
    auto streamIterator = stream.begin();  // make iterator start from the beginning  // Using an iterator to go through the stream vector

    while (!viewer->wasStopped ())  // PC Viewer run cycle  // frame update loop
    {
        // Clearing previously-rendered pointclouds and shapes from viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Loading PCD
        pcdCloud = pointCloudXYZIProcessor.loadPcd((*streamIterator).string());  // dereferencing streamIterator and converting output path to string

        // Running obstacle-detection pipeline
        objectDetectionXYZI(viewer, pointCloudXYZIProcessor, pcdCloud);

        streamIterator++;  // TODO: check if same as ++streamIterator;
        if (streamIterator == stream.end())  // when it reaches the end
            streamIterator = stream.begin();  // reset back to the beginning

        viewer->spinOnce();  // Controls the frame rate. By default it waits 1 time step, which would make it run as fast as possible. Frame rate dependings on time efficiency of obstacle-detection functions  // TODO: check for possible optimisations
    }
}
