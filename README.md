# Object detection (LiDAR only)

<img src="media/ObstacleDetectionFPS.gif" width="700" height="400" />

### 

**LiDAR** sensors can give us accurate high-resolution 3D models of the world around us by sending out laser signals. The beams bounce off objects, returning to the sensor where we can then determine how far away objects are by, for instance, timing how long it takes for the signal to return. We can also tell a little bit about the object that was hit by measuring the intensity of the returned signal. Laser rays are in the infrared spectrum and are sent out at many different angles, usually in a 360-degree range.


## Installation

### Ubuntu 

```bash
$> sudo apt install libpcl-dev
$> cd ~
$> git clone git@github.com:maggieliuzzi/lidar_object_detection.git
$> cd lidar_object_detection
$> mkdir build && cd build
$> cmake ..
$> make
$> ./environment
```

### Windows 

http://www.pointclouds.org/downloads/windows.html

### MAC

#### Install via Homebrew
1. install [homebrew](https://brew.sh/)
2. update homebrew 
	```bash
	$> brew update
	```
3. add  homebrew science [tap](https://docs.brew.sh/Taps) 
	```bash
	$> brew tap brewsci/science
	```
4. view pcl install options
	```bash
	$> brew options pcl
	```
5. install PCL 
	```bash
	$> brew install pcl
	```

#### Prebuilt Binaries via Universal Installer
http://www.pointclouds.org/downloads/macosx.html  
NOTE: very old version 

#### Build from Source

[PCL Source Github](https://github.com/PointCloudLibrary/pcl)

[PCL Mac Compilation Docs](http://www.pointclouds.org/documentation/tutorials/compiling_pcl_macosx.php)
