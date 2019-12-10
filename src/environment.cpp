/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

/*
    Obstacle detection pipeline on a cityscape environment using LiDAR

    Streaming of PCD files, filtering, segmenting ground plane and obstacles, clustering obstacles and displaying bounding boxes around them

    Custom implementation of:
        RANSAC Segmentation
        Euclidean Clustering optimised with a k-d tree for more efficient nearby neighbour search

    Uses PCL types and functions for:
        Filtering
        PointCloud, PointXYZ and PointXYZI types

TODO: since I'm able to detect obstacles in single frames, I can make my pipeline even more robust by tracking detections over the history of frames. 
I can create associations between detections in frames and use that to track objects.
One way to create associations between two different frames is by how close in proximity two detections are to each other and how similar they look. 
There are also a lot more filtering procedures that you can explore, such as looking at detection that are seen in consecutive frames before they are considered. 
You could also filter based on bounding boxes, their volume and shapes. By deploying tracking methods and associations you could try to dynamically build the shapes of obstacles. Examples of this might be, maybe you see the backside of a long truck, the lidar only first sees the back of the truck. Then later you drive past the truck. letting the lidar see the trucks side. 
There are many ways to keep exploring and making the detection process more robust.

TODO: challenge problem: detecting and tracking a bicylist riding in front of the car, along with detecting/tracking the other surrounding obstacles in the scene, using data from src/sensors/data/pcd/data_2

TODO: check rubric on Udacity Classroom: https://review.udacity.com/#!/rubrics/2529/view
*/

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



// TODO: try using: const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud to fix linkage errors
// Notice that in the function header you can optionally make inputCloud a constant reference by doing const and & at the end of the variable definition. 
// You don't have to do this but you are not actually changing the inputCloud at all, just using it as an input for your point processor function. 
// The benefit of using a constant reference is better memory efficiency, since you don't have to write to that variable's memory, just read from it, so it's a slight performance increase. 
// If you do make this a const reference though, make sure not to modify it, or else you will get a compile error.



// void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointCloudXYZIProcessor, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)  // TODO: try
void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI> pointCloudXYZIProcessor, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)  // TODO: try without *, const and last &
{
    // Opens 3D view and displays city block

    // ProcessPointClouds<pcl::PointXYZI>* pointCloudXYZIProcessor = new ProcessPointClouds<pcl::PointXYZI>(); // on the heap, it will be a pointer, we use new and *  // on the stack: no pointer, no *, no = ..., finishes there ;
    // ProcessPointClouds<pcl::PointXYZI> pointCloudXYZIProcessor;

    // PCL with ~120,000 points
    // pcl::PointCloud<pcl::PointXYZI>::Ptr pcdCloud = pointCloudXYZIProcessor->loadPcd("/Users/maggieliuzzi/Documents/ComputerVision/SensorFusionNanodegree/SFND_Lidar_Obstacle_Detection/src/sensors/data/pcd/data_1/0000000000.pcd");
    // pcl::PointCloud<pcl::PointXYZI>::Ptr pcdCloud = pointCloudXYZIProcessor.loadPcd("/Users/maggieliuzzi/Documents/ComputerVision/SensorFusionNanodegree/SFND_Lidar_Obstacle_Detection/src/sensors/data/pcd/data_1/0000000000.pcd");
    // renderPointCloud(viewer, pcdCloud, "pcdCloud");  // if no colour specified, defaults to colour coding from Intensity component

    pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud = pointCloudXYZIProcessor.FilterCloud(inputCloud, 0.5, Eigen::Vector4f (-10, -10, -5, 1), Eigen::Vector4f (30, 10, 10, 1));  // TODO: try 0.3, -10, -5, -2, 1, 30, 8, 1, 1
    // or no const nor &, inputCloud = pointCloudXYZIProcessor.filterCloud
    
    renderPointCloud(viewer, filteredCloud, "filteredCloud");  // TODO: remove after implementing segmentation and clustering


    /*
    // Segmentation
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentedCloud = pointCloudXYZIProcessor.SegmentPlane(filteredCloud, 50, 0.3);  // TODO: try 25, 0.3  // Validated, it should look like this, with a dot (no arrow)
    // renderPointCloud(viewer, segmentedCloud.first, "obstacleCloud");
    renderPointCloud(viewer, segmentedCloud.second, "groundPlaneCloud");
    */


    /*
    // Clustering
    // All validated, including . vs ->
    // PCL Euclidean Clustering  // TODO: try with custom implementation
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> obstacleClusters = pointCloudXYZIProcessor.PCLClustering(segmentedCloud.first, 1.0, 3, 30);  // validated, with .  // TODO: try 0.53, 10, 500
    // TODO: use minSize for cleaning up points scattered off in the edge and distanceTolerance for something
    
    int clusterId = 0;
    std::vector<Color> colours = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};  // TODO: add more colours to cycle through

    for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : obstacleClusters)
    {
        std::cout << "Cluster size: " << pointCloudXYZIProcessor.numPoints(cluster);

        renderPointCloud(viewer, cluster, "obstCloud"+std::to_string(clusterId), colours[clusterId % colours.size()]);
    
        // Bounding boxes
        // Since all the detectable vehicles in this scene are along the same axis as our car, the simple already set up bounding box function in point processor should yield good results.
        Box box = pointCloudXYZIProcessor.BoundingBox(cluster);  // if pointCloudProcessor was a pointer, use ->
        renderBox(viewer, box, clusterId);

        ++clusterId;
    }

    // TODO: if clustering parts of the wall, try changing size of box cropped, or look at dimensions and filter based on the bounding box. eg. < certain width ignore, or only consider object > certain dimensions
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

    // ProcessPointClouds<pcl::PointXYZI>* pointCloudXYZIProcessor = new ProcessPointClouds<pcl::PointXYZI>();  // on the heap
    ProcessPointClouds<pcl::PointXYZI> pointCloudXYZIProcessor;  // on the stack


    // pcl::PointCloud<pcl::PointXYZI>::Ptr pcdCloud = pointCloudXYZIProcessor.loadPcd("/Users/maggieliuzzi/Documents/ComputerVision/SensorFusionNanodegree/SFND_Lidar_Obstacle_Detection/src/sensors/data/pcd/data_1/0000000000.pcd");
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcdCloud;

    // argument: folder directory that contains all the sequentially ordered pcd files to process
    // return: a chronologically ordered vector of all those file names
    std::vector<boost::filesystem::path> stream = pointCloudXYZIProcessor.streamPcd("/Users/maggieliuzzi/Documents/ComputerVision/SensorFusionNanodegree/SFND_Lidar_Obstacle_Detection/src/sensors/data/pcd/data_1");  // TODO: change to relative path
    // can go through the stream vector in a couple of ways, one option is to use an iterator
    auto streamIterator = stream.begin();  // make iterator start from the beginning

    // pcl viewer run cycle  // frame update loop
    while (!viewer->wasStopped ())
    {
        // Clear viewer - previously rendered points and shapes
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load PCD and run obstacle detection pipeline
        pcdCloud = pointCloudXYZIProcessor.loadPcd((*streamIterator).string());  // dereferencing streamIterator and convert output path to a string

        // Real pointcloud
        cityBlock(viewer, pointCloudXYZIProcessor, pcdCloud);

        streamIterator++;  // TODO: check if same as ++streamIterator;
        if (streamIterator == stream.end())  // when it reaches the end
            streamIterator = stream.begin();  // reset it back to the beginning

        // Controls the frame rate, by default it waits 1 time step, which would make it run as fast as possible. Depending on how timing efficient your obstacle detection functions were set up the faster the viewer's frame rate will be
        // TODO: check for optimisations
        viewer->spinOnce ();
    } 
}