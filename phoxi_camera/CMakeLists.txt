cmake_minimum_required(VERSION 3.8)
project(phoxi_camera)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies

set(DEPENDENCIES
    rclcpp 
    rclpy
    pcl_msgs 
    pcl_conversions 
    sensor_msgs
    rclcpp_components
    cv_bridge
    std_msgs
    nav_msgs
    tf2_ros
    tf2
    tf2_geometry_msgs 
    OpenCV
    std_srvs
    PhoXi
    rosidl_default_generators
    geometry_msgs
    phoxi_msgs
    tf2_eigen
    Eigen3
    PCL 
    diagnostic_updater
    ) 



find_package(PhoXi REQUIRED CONFIG PATHS "$ENV{PHOXI_CONTROL_PATH}")
find_package(ament_cmake REQUIRED)

foreach(Dependency IN ITEMS ${DEPENDENCIES})
  find_package(${Dependency} REQUIRED)
endforeach()

set(INCLUDES

    include/phoxi_camera/ros_interface.hpp
    include/phoxi_camera/PhoXiInterface.h
    include/phoxi_camera/PhoXiDeviceInformation.h
    include/phoxi_camera/PhoXiException.h
    include/phoxi_camera/RosConversions.h
    include/phoxi_camera/PhoXiConversions.h

)

include_directories(include
  # ${PHOXI_INCLUDE_DIRS}
)

set(SOURCES

    src/RosInterface.cpp
    src/PhoXiInterface.cpp
    
    )
    
    


add_executable(phoxi_camera_node src/phoxi_camera_node.cpp)


target_compile_options(phoxi_camera_node PRIVATE -g)

add_library(${PROJECT_NAME} SHARED
  ${SOURCES}
  )

ament_target_dependencies(${PROJECT_NAME}
  ${DEPENDENCIES}
)  

target_include_directories(${PROJECT_NAME} PUBLIC
  include
  ${PHOXI_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}

)




target_link_libraries(phoxi_camera 
  ${PHOXI_LIBRARY}
  ${PCL_LIBRARIES}


)

target_link_libraries(phoxi_camera_node 
  ${PROJECT_NAME}
  ${PHOXI_LIBRARY}
  ${PCL_LIBRARIES}
  phoxi_camera
  /opt/Photoneo/PhoXiControl-1.10.0/API/lib/libPhoXi_API_gcc11.3.0_Release.so.1.10.0

)


  
ament_target_dependencies(phoxi_camera_node 
    ${DEPENDENCIES}
)  



install(TARGETS
  phoxi_camera_node
  DESTINATION lib/${PROJECT_NAME})  


install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)



install(TARGETS
phoxi_camera
DESTINATION lib)  

ament_export_dependencies(${DEPENDENCIES})
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
  



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
