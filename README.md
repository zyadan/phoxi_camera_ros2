# phoxi_camera_ros2
This is the development for the ros2 driver of Photoneo Phoxi 3D scanner 
(refer to its ROS1 driver https://github.com/photoneo/phoxi_camera)
![image](https://github.com/zyadan/phoxi_camera_ros2/assets/24379540/846a5034-8807-4f2e-a941-1a0cb3011e51)


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

* Clone phoxi_camera_ros2 repository to your ROS2 workspace (usually ros2_ws/src)
```
cd ros2_ws/src
git clone https://github.com/zyadan/phoxi_camera_ros2
```

* Change working directory to your root ROS workspace folder (usually ros2_ws)
```
cd ros2_ws
```
* Install all dependencies needed by phoxi_camera package
```
rosdep install --from-paths src --ignore-src -r -y
```
* Set Phoxi path
```
export PHOXI_CONTROL_PATH=/opt/Photoneo/PhoXiControl-1.xx.x
export PHOXI_RELEASE_LIBRART=/your PhoxiControl path/PhoXiControl-1.xx.xx/API/lib/
# for example
export PHOXI_CONTROL_PATH=/opt/Photoneo/PhoXiControl-1.10.0
export PHOXI_RELEASE_LIBRART=/opt/Photoneo/PhoXiControl-1.10.0/API/lib/

```

* Build the packages
```
colcon build
```
* Copy liblibPhoXi_API_gcc11.3.0_Release.so.1.10.0 to ros2_ws/install/phoxi_camera/lib folder
```
cd ros2_ws
cp /opt/Photoneo/PhoXiControl-1.10.0/API/lib/libPhoXi_API_gcc11.3.0_Release.so.1.10.0 ./install/phoxi_camera/lib/
```

### Parameters
```
~/scanner_id_                  - Default PhoXi 3D Scannet to connect after startup. Default value: "2019-07-005-LC3"
~/frame_id:                    - Frame id to which captured data relies to. Default value: "PhoXi3Dscanner_sensor"
~/latch_topics_                 - Default value: false
~/topic_queue_size_             - Default value: 1
~/init_from_config_             - Default value: false # if true all following parameters will be initialized from this config otherwise from 
                                  PhoXi control application.
~/organized_cloud_               - Default value: false  # if true organized point cloud will be published, other otherwise unorganized

  
~/log_level                     - Default value: info    # debug log level [DEBUG|INFO|WARN|ERROR|FATAL]
~/config_file                   - Default value:         # yaml config file
~/start_acquisition_            - Default value: false 
~/stop_acquisition_             - Default value: false
~/resolution_                   - Default value:  1
~/scan_multiplier_              - Default value:  1
~/confidence_                   - Default value:  3.0
~/send_confidence_map_          - Default value: true
~/send_depth_map_               - Default value:   true
~/send_normal_map_              - Default value:  true
~/send_point_cloud_             - Default value:  true
~/send_texture_                 - Default value:  true
~/shutter_multiplier_           - Default value:  1
~/timeout_                      - Default value:  -3
~/trigger_mode_                 - Default value:  1
~/coordinate_space_             - Default value:  1
~/ambient_light_suppression_    - Default value:  false'
~/single_pattern_exposure_      - Default value:  2
~/camera_only_mode_             - Default value:  false
```
### Available ROS services
```
~/phoxi_camera/V2/is_acquiring
~/phoxi_camera/V2/is_connected
~/phoxi_camera/V2/start_acquisition
~/phoxi_camera/V2/stop_acquisition
~/phoxi_camera/V2/set_transformation
~/phoxi_camera/V2/set_coordinate_space
~/phoxi_camera/V2/save_last_frame
~/phoxi_camera/connect_camera
~/phoxi_camera/disconnect_camera
~/phoxi_camera/get_device_list
~/phoxi_camera/get_frame
~/phoxi_camera/get_hardware_indentification
~/phoxi_camera/get_supported_capturing_modes
~/phoxi_camera/is_acquiring
~/phoxi_camera/is_connected
~/phoxi_camera/save_frame
~/phoxi_camera/set_parameters
~/phoxi_camera/start_acquisition
~/phoxi_camera/stop_acquisition
~/phoxi_camera/trigger_image

~/phoxi_camera_node/describe_parameters
~/phoxi_camera_node/get_parameter_types
~/phoxi_camera_node/get_parameters
~/phoxi_camera_node/list_parameters
~/phoxi_camera_node/set_parameters
~/phoxi_camera_node/set_parameters_atomically

```

### Available ROS topics
```
~/phoxi_camera/confidence_map
~/phoxi_camera/depth_map
~/phoxi_camera/normal_map
~/phoxi_camera/points
~/phoxi_camera/rgb_texture
~/phoxi_camera/texture
```

## Test PhoXi ROS2 interface with real device

* Start PhoXiControl application
![image](https://github.com/zyadan/phoxi_camera_ros2/assets/24379540/50ef06e1-95d2-4174-94b0-6e597559f6a2)

* Run Interface node
```
ros2 run phoxi_camera phoxi_camera_node
# or run with launch file
ros2 launch phoxi_camera phoxi_camera_launch.py
```
![image](https://github.com/zyadan/phoxi_camera_ros2/assets/24379540/ace95a1c-1ad2-4794-b506-721b347a7356)

* Use available ROS2 services to control your 3D scanner
![image](https://github.com/zyadan/phoxi_camera_ros2/assets/24379540/71e72278-f8cb-4902-88d3-d299fee605f0)






