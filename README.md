# phoxi_camera_ros2
This is the development for the ros2 driver of Photoneo Phoxi 3D scanner (refer to its ROS1 driver https://github.com/photoneo/phoxi_camera)

## Installation

* ROS2 (here we used Humble under Ubuntu 22.04)
* PhoXiControl version 1.10.x driver software which you can download on Photoneo website: https://www.photoneo.com/downloads/phoxi-control/ (* contact support@photoneo.com for testing the release candidate)


### Installation steps

* Download latest PhoXi Control according to your ubuntu version
* Add rights to execute downloaded file

```
sudo chmod +x PhotoneoPhoXiControlInstaller-xxxxx.run
```

* Install downloaded file
```
sudo ./PhotoneoPhoXiControlInstaller-xxxxx.run
```

* clone phoxi_camera_ros2 repository to your ROS2 workspace (usually ros2_ws/src)
```
cd ros2_ws/src
git clone https://github.com/zyadan/phoxi_camera_ros2
```

* Change working directory to your root ROS workspace folder (usually ros2_ws )
```
cd ros2_ws
```
* Install all dependencies needed by phoxi_camera package
```
rosdep install --from-paths src --ignore-src -r -y
```
* Build the packages
```
colcon build
```


## Test PhoXi ROS2 interface with real device

* Start PhoXiControl application
* Connect to your device
* Run Interface node
```
ro2 srun phoxi_camera phoxi_camera_node
# or run with launch file
ros2 launch phoxi_camera phoxi_camera_launch.py
```
* Use available ROS services to control your 3D scanner






