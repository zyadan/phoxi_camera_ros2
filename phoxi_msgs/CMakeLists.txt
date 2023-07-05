cmake_minimum_required(VERSION 3.8)
project(phoxi_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(PhoXi REQUIRED CONFIG PATHS "$ENV{PHOXI_CONTROL_PATH}")
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


set(srv_files
  "srv/Deprecated/Empty.srv"      #deprecated
  "srv/Deprecated/IsConnected.srv" #deprecated
  "srv/Deprecated/IsAcquiring.srv" #deprecated
  "srv/ConnectCamera.srv"
  "srv/TriggerImage.srv"
  "srv/GetDeviceList.srv"
  "srv/GetBool.srv"
  "srv/GetFrame.srv"
  "srv/GetHardwareIdentification.srv"
  "srv/GetSupportedCapturingModes.srv"
  "srv/SaveFrame.srv"
  "srv/SaveLastFrame.srv"
  "srv/SetCoordinatesSpace.srv"
  "srv/SetTransformationMatrix.srv"
  "srv/GetString.srv"
)


set(msg_files
  "msg/DeviceConnectionStatus.msg"
  "msg/DeviceInformation.msg" 
  "msg/DeviceType.msg"
  "msg/PhoXiSize.msg"
)


rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}

DEPENDENCIES
  std_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
  tf2_geometry_msgs
  rosidl_default_generators
)

ament_export_dependencies(rosidl_default_runtime)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
