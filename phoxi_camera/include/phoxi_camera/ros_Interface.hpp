//
// Created by controller on 1/11/18.
//

#ifndef PROJECT_ROSINTERFACE_H
#define PROJECT_ROSINTERFACE_H

#define OKRESPONSE "Ok"

//ros2
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/parameter.hpp>
#include <cv_bridge/cv_bridge.h>
#include "Eigen/Geometry"
#include <Eigen/Dense>
#include <tf2/convert.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/buffer_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <tf2_eigen/tf2_eigen.h>


//dynamic reconfigure
// #include <dynamic_reconfigure/server.h>
// #include <phoxi_camera/phoxi_cameraConfig.h>

//diagnostic updater
#include <boost/thread/mutex.hpp>
// #include <diagnostic_updater/diagnostic_updater.h>

//messages
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/fill_image.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <std_srvs/srv/empty.hpp>
#include <std_msgs/msg/header.hpp>

#include "sensor_msgs/image_encodings.hpp"
#include <diagnostic_updater/diagnostic_updater.hpp>



#include <phoxi_camera/PhoXiInterface.h>
#include <phoxi_msgs/srv/get_device_list.hpp>
#include <phoxi_msgs/srv/connect_camera.hpp>
#include <phoxi_msgs/srv/is_connected.hpp>
#include <phoxi_msgs/srv/is_acquiring.hpp>
#include <phoxi_msgs/srv/get_bool.hpp>
#include <phoxi_msgs/srv/empty.hpp>
#include <phoxi_msgs/srv/trigger_image.hpp>
#include <phoxi_msgs/srv/get_frame.hpp>
#include <phoxi_msgs/srv/save_frame.hpp>
#include <phoxi_msgs/srv/save_last_frame.hpp>
#include <phoxi_msgs/srv/get_hardware_identification.hpp>
#include <phoxi_msgs/srv/get_supported_capturing_modes.hpp>
#include <phoxi_msgs/srv/set_coordinates_space.hpp>
#include <phoxi_msgs/srv/set_transformation_matrix.hpp>
#include <phoxi_msgs/srv/get_string.hpp>

//std
#include <vector>
#include <algorithm>

#include <iostream>
#include <memory>
#include <string>



#include <builtin_interfaces/msg/time.hpp>

//others
#include <phoxi_camera/PhoXiException.h>
#include <phoxi_camera/RosConversions.h>
#include <rclcpp_components/register_node_macro.hpp>

using namespace std::chrono_literals;


namespace phoxi_camera {

  class RosInterface : protected PhoXiInterface, public rclcpp::Node{
    public:
      RosInterface(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());


    protected:
      

      int triggerImageid();
      void publishFrame(pho::api::PFrame frame);
      pho::api::PFrame getPFrame(int id = -1);
      std::string getTriggerMode(pho::api::PhoXiTriggerMode mode);
      std::string frameId;


    private:

      bool start_acquisition_;
      bool stop_acquisition_;

      rclcpp::TimerBase::SharedPtr timer_;

      

      bool latch_topics_;
      int topic_queue_size_;
      bool init_from_config_;
      bool organized_cloud_;

      int resolution_; 
      int scan_multiplier_;
      float confidence_;
      bool send_confidence_map_;
      bool send_depth_map_;
      bool send_normal_map_;
      bool send_point_cloud_;
      bool send_texture_;
      int shutter_multiplier_;
      int timeout_;
      int trigger_mode_;

      int coordinate_space_;
      bool ambient_light_suppression_;
      int single_pattern_exposure_;
      bool camera_only_mode_;


      void initFromPhoXi();

      std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
      std::shared_ptr<rclcpp::ParameterCallbackHandle> cb_handle_;
      
      void diagnosticCallback(diagnostic_updater::DiagnosticStatusWrapper & status);

      void diagnosticTimerCallback();


      bool getDeviceList(const std::shared_ptr<phoxi_msgs::srv::GetDeviceList::Request> req, 
                         std::shared_ptr<phoxi_msgs::srv::GetDeviceList::Response> res);

      bool isConnected(const std::shared_ptr<phoxi_msgs::srv::IsConnected::Request> req, 
                       std::shared_ptr<phoxi_msgs::srv::IsConnected::Response> res);

      bool isConnectedV2(const std::shared_ptr<phoxi_msgs::srv::GetBool::Request> req, 
                       std::shared_ptr<phoxi_msgs::srv::GetBool::Response> res);

      bool isAcquiring(const std::shared_ptr<phoxi_msgs::srv::IsAcquiring::Request> req, 
                       std::shared_ptr<phoxi_msgs::srv::IsAcquiring::Response> res);

      bool isAcquiringV2(const std::shared_ptr<phoxi_msgs::srv::GetBool::Request> req, 
                         std::shared_ptr<phoxi_msgs::srv::GetBool::Response> res);

      bool startAcquisition(const std::shared_ptr<std_srvs::srv::Empty::Request> req, 
                           std::shared_ptr<std_srvs::srv::Empty::Response> res);

      bool startAcquisitionV2(const std::shared_ptr<phoxi_msgs::srv::Empty::Request> req, 
                             std::shared_ptr<phoxi_msgs::srv::Empty::Response> res);

      bool stopAcquisition(const std::shared_ptr<std_srvs::srv::Empty::Request> req, 
                           std::shared_ptr<std_srvs::srv::Empty::Response> res);

      bool stopAcquisitionV2(const std::shared_ptr<phoxi_msgs::srv::Empty::Request> req, 
                             std::shared_ptr<phoxi_msgs::srv::Empty::Response> res);

      bool triggerImage(const std::shared_ptr<phoxi_msgs::srv::TriggerImage::Request> req, 
                        std::shared_ptr<phoxi_msgs::srv::TriggerImage::Response> res);

      bool getFrame(const std::shared_ptr<phoxi_msgs::srv::GetFrame::Request> req, 
                        std::shared_ptr<phoxi_msgs::srv::GetFrame::Response> res);

      bool saveFrame(const std::shared_ptr<phoxi_msgs::srv::SaveFrame::Request> req, 
                        std::shared_ptr<phoxi_msgs::srv::SaveFrame::Response> res);

      bool connectCamera(const std::shared_ptr<phoxi_msgs::srv::ConnectCamera::Request> req, 
                        std::shared_ptr<phoxi_msgs::srv::ConnectCamera::Response> res);

      void connectCameraV2(std::string HWIdentification,
                           pho::api::PhoXiTriggerMode mode = pho::api::PhoXiTriggerMode::Software,
                           bool startAcquisition = true);

      bool disconnectCamera(const std::shared_ptr<std_srvs::srv::Empty::Request> req, 
                            std::shared_ptr<std_srvs::srv::Empty::Response> res);

      bool getHardwareIdentification(const std::shared_ptr<phoxi_msgs::srv::GetHardwareIdentification::Request> req, 
                                    std::shared_ptr<phoxi_msgs::srv::GetHardwareIdentification::Response> res);

      bool getSupportedCapturingModes(const std::shared_ptr<phoxi_msgs::srv::GetSupportedCapturingModes::Request> req, 
                                    std::shared_ptr<phoxi_msgs::srv::GetSupportedCapturingModes::Response> res);

      bool getApiVersion(const std::shared_ptr<phoxi_msgs::srv::GetString::Request> req, 
                         std::shared_ptr<phoxi_msgs::srv::GetString::Response> res);

      bool getFirmwareVersion(const std::shared_ptr<phoxi_msgs::srv::GetString::Request> req, 
                              std::shared_ptr<phoxi_msgs::srv::GetString::Response> res);

      bool setCoordianteSpace(const std::shared_ptr<phoxi_msgs::srv::SetCoordinatesSpace::Request> req, 
                              std::shared_ptr<phoxi_msgs::srv::SetCoordinatesSpace::Response> res);

      bool setTransformation(const std::shared_ptr<phoxi_msgs::srv::SetTransformationMatrix::Request> req, 
                             std::shared_ptr<phoxi_msgs::srv::SetTransformationMatrix::Response> res);
                    
      bool saveLastFrame(const std::shared_ptr<phoxi_msgs::srv::SaveLastFrame::Request> req, 
                         std::shared_ptr<phoxi_msgs::srv::SaveLastFrame::Response> res);



    private:
 
      rclcpp::Service<phoxi_msgs::srv::GetDeviceList>::SharedPtr getDeviceListService_;
      rclcpp::Service<phoxi_msgs::srv::IsConnected>::SharedPtr isConnectedService_;
      rclcpp::Service<phoxi_msgs::srv::GetBool>::SharedPtr isConnectedServiceV2_;
      rclcpp::Service<phoxi_msgs::srv::IsAcquiring>::SharedPtr isAcquiringService_;
      rclcpp::Service<phoxi_msgs::srv::GetBool>::SharedPtr isAcquiringServiceV2_;
      rclcpp::Service<std_srvs::srv::Empty>::SharedPtr startAcquisitionService_;
      rclcpp::Service<phoxi_msgs::srv::Empty>::SharedPtr startAcquisitionServiceV2_;
      rclcpp::Service<std_srvs::srv::Empty>::SharedPtr stopAcquisitionService_;
      rclcpp::Service<phoxi_msgs::srv::Empty>::SharedPtr stopAcquisitionServiceV2_;
      rclcpp::Service<phoxi_msgs::srv::TriggerImage>::SharedPtr triggerImageService_;
      rclcpp::Service<phoxi_msgs::srv::GetFrame>::SharedPtr getFrameService_;
      rclcpp::Service<phoxi_msgs::srv::SaveFrame>::SharedPtr saveFrameService_;
      rclcpp::Service<phoxi_msgs::srv::ConnectCamera>::SharedPtr connectCameraService_;
      rclcpp::Service<std_srvs::srv::Empty>::SharedPtr disconnectCameraService_;
      rclcpp::Service<phoxi_msgs::srv::GetHardwareIdentification>::SharedPtr getHardwareIdentificationService_;
      rclcpp::Service<phoxi_msgs::srv::GetSupportedCapturingModes>::SharedPtr getSupportedCapturingModesService_;
      rclcpp::Service<phoxi_msgs::srv::GetString>::SharedPtr getApiVersionService_;
      rclcpp::Service<phoxi_msgs::srv::GetString>::SharedPtr getFirmwareVersionService_;
      rclcpp::Service<phoxi_msgs::srv::SetCoordinatesSpace>::SharedPtr setCoordianteSpaceService_;
      rclcpp::Service<phoxi_msgs::srv::SetTransformationMatrix>::SharedPtr setTransformationService_;
      rclcpp::Service<phoxi_msgs::srv::SaveLastFrame>::SharedPtr saveLastFrameService_;

      

    //ROS2 
    rclcpp::Node::SharedPtr node_;

    //ROS Topic
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloudPub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr normalMapPub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr confidenceMapPub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rawTexturePub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgbTexturePub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depthMapPub_;


    //diagnostic
    
    // diagnostic_updater::FunctionDiagnosticTask PhoXi3DscannerDiagnosticTask;
    rclcpp::TimerBase::SharedPtr diagnosticTimer;
    // rclcpp::TimerBase::SharedPtr timer_;
        // rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
      };
}

// RCLCPP_COMPONENTS_REGISTER_NODE(phoxi_camera::RosInterface)

#endif //PROJECT_ROSINTERFACE_H