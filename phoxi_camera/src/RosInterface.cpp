
#include "phoxi_camera/ros_Interface.hpp"

namespace phoxi_camera {

    RosInterface::RosInterface(const rclcpp::NodeOptions& options)
        : Node("phoxi_camera_node", options) {
 
        this->declare_parameter<bool>("start_acquisition_", true);
        this->declare_parameter<bool>("stop_acquisition_", false);


        this->declare_parameter<std::string>("scanner_id_", "2019-07-005-LC3"); // keep "" if scanner id is number

        this->declare_parameter<std::string>("frame_id", "PhoXi3Dscanner_sensor");
        this->declare_parameter<bool>("latch_topics_", false);
        this->declare_parameter<int>("topic_queue_size_", 1);
        this->declare_parameter<bool>("init_from_config_", false); //if true all following parameters will be initialized from this config otherwise from PhoXi control application.
        this->declare_parameter<bool>("organized_cloud_", false); //if true organized point cloud will be published, other otherwise unorganized (Default value true)

        // All following parameters are for PhoXi Control and they can override all dynamic_reconfigure parameters in cfg file.
        // This values are set to scanner after successful connection only if init_from_config parameter is true.
        this->declare_parameter<int>("resolution_", 1);
        this->declare_parameter<int>("scan_multiplier_", 1);
        this->declare_parameter<float>("confidence_", 3.0);
        this->declare_parameter<bool>("send_confidence_map_", true);
        this->declare_parameter<bool>("send_depth_map_", true);
        this->declare_parameter<bool>("send_normal_map_", true);
        this->declare_parameter<bool>("send_point_cloud_", true);
        this->declare_parameter<bool>("send_texture_", true);
        this->declare_parameter<int>("shutter_multiplier_", 1);
        this->declare_parameter<int>("timeout_", -3); // in ms, special parameters: 0 = Zero, -1 = Infinity, -2 = Last stored, -3 = Defaul
        this->declare_parameter<int>("trigger_mode_", 1); // 0 = Free run, 1 = Software

        // Setting available only for PhoXi Control 1.2 and higher. Also for dynamic reconfigure
        this->declare_parameter<int>("coordinate_space_", 1); // 1 = Camera, 2 =  Mounting, 3 = Marker, 4 = Robot, 5 = Custom
        this->declare_parameter<bool>("ambient_light_suppression_", false); //Ambient light suppression samples the scene multiple times during one pattern exposure.
                                                                            // This multiple samples are then used to suppress the effect of ambient illumination by eliminating most of the shot noise caused by longer exposure of ambient light.
                                                                            // Enabling the mode will set Shutter multiplier to fixed value of 2.
        this->declare_parameter<int>("single_pattern_exposure_", 2); //The time for projection of one pattern. Use only provided values form PhoXi control settings!
                                                                    // 0 = 10.240; 1 = 14.336; 2 = 20.480; 3 = 24.576; 4 = 30.720;  5 = 34.816;
                                                                    // 6 = 40.960; 7 = 49.152; 8 = 75.776; 9 = 79.872; 10 = 90.112; 11 = 100.352
        this->declare_parameter<bool>("camera_only_mode_", false);


        //create publishers

        topic_queue_size_ = this->get_parameter("topic_queue_size_").as_int();

        cloudPub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/phoxi_camera/points", 1);
        normalMapPub_ = this->create_publisher<sensor_msgs::msg::Image>("/phoxi_camera/normal_map", topic_queue_size_);
        confidenceMapPub_ = this->create_publisher<sensor_msgs::msg::Image>("/phoxi_camera/confidence_map", topic_queue_size_);
        rawTexturePub_ = this->create_publisher<sensor_msgs::msg::Image>("/phoxi_camera/texture", topic_queue_size_);
        rgbTexturePub_ = this->create_publisher<sensor_msgs::msg::Image>("/phoxi_camera/rgb_texture", topic_queue_size_);
        depthMapPub_ = this->create_publisher<sensor_msgs::msg::Image>("/phoxi_camera/depth_map", topic_queue_size_);

        param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
        auto cb = [this](const rclcpp::Parameter & p) {
        RCLCPP_INFO(
          this->get_logger(), "cb: Received an update to parameter \"%s\" of type %s: \"%ld\"",
          p.get_name().c_str(),
          p.get_type_name().c_str(),
          p.as_int());
        };
        cb_handle_ = param_subscriber_->add_parameter_callback("organized_cloud_", cb);



        
        //service
        getDeviceListService_ = this->create_service<phoxi_msgs::srv::GetDeviceList>("phoxi_camera/get_device_list", std::bind(&RosInterface::getDeviceList, this, std::placeholders::_1, std::placeholders::_2));
        isConnectedService_ = this->create_service<phoxi_msgs::srv::IsConnected>("phoxi_camera/is_connected", std::bind(&RosInterface::isConnected, this, std::placeholders::_1, std::placeholders::_2));
        isConnectedServiceV2_ = this->create_service<phoxi_msgs::srv::GetBool>("phoxi_camera/V2/is_connected", std::bind(&RosInterface::isConnectedV2, this, std::placeholders::_1, std::placeholders::_2));
        isAcquiringService_ = this->create_service<phoxi_msgs::srv::IsAcquiring>("phoxi_camera/is_acquiring", std::bind(&RosInterface::isAcquiring, this, std::placeholders::_1, std::placeholders::_2));
        isAcquiringServiceV2_ = this->create_service<phoxi_msgs::srv::GetBool>("phoxi_camera/V2/is_acquiring", std::bind(&RosInterface::isAcquiringV2, this, std::placeholders::_1, std::placeholders::_2));
        startAcquisitionService_ = this->create_service<std_srvs::srv::Empty>("phoxi_camera/start_acquisition", std::bind(&RosInterface::startAcquisition, this, std::placeholders::_1, std::placeholders::_2));
        startAcquisitionServiceV2_ = this->create_service<phoxi_msgs::srv::Empty>("phoxi_camera/V2/start_acquisition", std::bind(&RosInterface::startAcquisitionV2, this, std::placeholders::_1, std::placeholders::_2));
        stopAcquisitionService_ = this->create_service<std_srvs::srv::Empty>("phoxi_camera/stop_acquisition", std::bind(&RosInterface::stopAcquisition, this, std::placeholders::_1, std::placeholders::_2));
        stopAcquisitionServiceV2_ = this->create_service<phoxi_msgs::srv::Empty>("phoxi_camera/V2/stop_acquisition", std::bind(&RosInterface::stopAcquisitionV2, this, std::placeholders::_1, std::placeholders::_2));
        triggerImageService_ =this->create_service<phoxi_msgs::srv::TriggerImage>("phoxi_camera/trigger_image", std::bind(&RosInterface::triggerImage, this, std::placeholders::_1, std::placeholders::_2));
        getFrameService_ = this->create_service<phoxi_msgs::srv::GetFrame>("phoxi_camera/get_frame", std::bind(&RosInterface::getFrame, this, std::placeholders::_1, std::placeholders::_2));
        saveFrameService_ = this->create_service<phoxi_msgs::srv::SaveFrame>("phoxi_camera/save_frame", std::bind(&RosInterface::saveFrame, this, std::placeholders::_1, std::placeholders::_2));  
        connectCameraService_ = this->create_service<phoxi_msgs::srv::ConnectCamera>("phoxi_camera/connect_camera", std::bind(&RosInterface::connectCamera, this, std::placeholders::_1, std::placeholders::_2));
        disconnectCameraService_ = this->create_service<std_srvs::srv::Empty>("phoxi_camera/disconnect_camera", std::bind(&RosInterface::disconnectCamera, this, std::placeholders::_1, std::placeholders::_2));  
        getHardwareIdentificationService_ = this->create_service<phoxi_msgs::srv::GetHardwareIdentification>("phoxi_camera/get_hardware_indentification", std::bind(&RosInterface::getHardwareIdentification, this, std::placeholders::_1, std::placeholders::_2));
        getSupportedCapturingModesService_ = this->create_service<phoxi_msgs::srv::GetSupportedCapturingModes>("phoxi_camera/get_supported_capturing_modes", std::bind(&RosInterface::getSupportedCapturingModes, this, std::placeholders::_1, std::placeholders::_2));
        getApiVersionService_ = this->create_service<phoxi_msgs::srv::GetString>("phoxi_camera/get_api_version", std::bind(&RosInterface::getApiVersion, this, std::placeholders::_1, std::placeholders::_2));
        getFirmwareVersionService_ = this->create_service<phoxi_msgs::srv::GetString>("phoxi_camera/get_firmware_version", std::bind(&RosInterface::getFirmwareVersion, this, std::placeholders::_1, std::placeholders::_2));
        setCoordianteSpaceService_ = this->create_service<phoxi_msgs::srv::SetCoordinatesSpace>("phoxi_camera/V2/set_transformation", std::bind(&RosInterface::setCoordianteSpace, this, std::placeholders::_1, std::placeholders::_2));
        setTransformationService_ = this->create_service<phoxi_msgs::srv::SetTransformationMatrix>("phoxi_camera/V2/set_coordinate_space", std::bind(&RosInterface::setTransformation, this, std::placeholders::_1, std::placeholders::_2));
        saveLastFrameService_ = this->create_service<phoxi_msgs::srv::SaveLastFrame>("phoxi_camera/V2/save_last_frame", std::bind(&RosInterface::saveLastFrame, this, std::placeholders::_1, std::placeholders::_2));

        
        
        
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to get_device_list");

        diagnosticTimer = this->create_wall_timer(5000ms, std::bind(&RosInterface::diagnosticTimerCallback, this));

        this->get_parameter_or<std::string>("frame_id", frameId, "PhoXi3Dscanner_sensor");


        //connect to default scanner
        std::string scannerId;
        scannerId = "2019-07-005-LC3";
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "scannerId id %s", scannerId.c_str());
        if (scannerId == "2019-07-005-LC3" && !scannerId.empty()) {
            try {
                RosInterface::connectCameraV2(scannerId);
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Connected to %s", scannerId.c_str());
            } catch (PhoXiInterfaceException& e) {
                RCLCPP_WARN(this->get_logger(), "Connection to default scanner %s failed. %s ", scannerId.c_str(), e.what());

            }
        }
        if (!PhoXiInterface::isConnected()) {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Not Connected ");
            
        }





    }



    void 
    RosInterface::diagnosticTimerCallback() {
        // diagnosticUpdater.force_update();
    }

    bool 
    RosInterface::getDeviceList(const std::shared_ptr<phoxi_msgs::srv::GetDeviceList::Request> req, 
                                std::shared_ptr<phoxi_msgs::srv::GetDeviceList::Response> res) {
        try {
            res->out_id = PhoXiInterface::cameraList();
            phoXiDeviceInforamtionToRosMsg(PhoXiInterface::deviceList(), res->device_information_list);
            res->len = res->out_id.size();
            res->success = true;
            res->message = OKRESPONSE;
        } catch (PhoXiInterfaceException& e) {
            res->success = false;
            res->message = e.what();
        }
        return true;
 
    }

    bool
    RosInterface::connectCamera(const std::shared_ptr<phoxi_msgs::srv::ConnectCamera::Request> req, 
                              std::shared_ptr<phoxi_msgs::srv::ConnectCamera::Response> res) {
        try {
            RosInterface::connectCameraV2(req->camera_name);
            res->success = true;
            res->message = OKRESPONSE;
        } catch (PhoXiInterfaceException& e) {
            res->success = false;
            res->message = e.what();
        }
        return true;
    }

    void
    RosInterface::connectCameraV2(std::string HWIdentification, pho::api::PhoXiTriggerMode mode, bool startAcquisition) {
        PhoXiInterface::connectCamera(HWIdentification, mode, startAcquisition);
        bool initFromConfig = false;

        RCLCPP_INFO(this->get_logger(), "init_from_config", initFromConfig);
   
        if (initFromConfig) {
            // getDefaultDynamicReconfigureConfig(dynamicReconfigureConfig);
            // this->dynamicReconfigureCallback(dynamicReconfigureConfig, std::numeric_limits<uint32_t>::max());
        } else {
            initFromPhoXi();
        }
        // dynamicReconfigureServer.updateConfig(dynamicReconfigureConfig);
        // diagnosticUpdater.force_update();
    }



    bool 
    RosInterface::isConnected(const std::shared_ptr<phoxi_msgs::srv::IsConnected::Request> req, 
                              std::shared_ptr<phoxi_msgs::srv::IsConnected::Response> res) {
        res->connected = PhoXiInterface::isConnected();
        return true;
    }

    bool 
    RosInterface::isConnectedV2(const std::shared_ptr<phoxi_msgs::srv::GetBool::Request> req, 
                                std::shared_ptr<phoxi_msgs::srv::GetBool::Response> res) {
        res->value = PhoXiInterface::isConnected();
        res->message = OKRESPONSE; //todo tot este premysliet
        res->success = true;
        return true;
    }

    bool 
    RosInterface::isAcquiring(const std::shared_ptr<phoxi_msgs::srv::IsAcquiring::Request> req, 
                              std::shared_ptr<phoxi_msgs::srv::IsAcquiring::Response> res) {
        res->is_acquiring = PhoXiInterface::isAcquiring();
        return true;
    }

    bool 
    RosInterface::isAcquiringV2(const std::shared_ptr<phoxi_msgs::srv::GetBool::Request> req, 
                                std::shared_ptr<phoxi_msgs::srv::GetBool::Response> res) {
        res->value = PhoXiInterface::isAcquiring();
        res->message = OKRESPONSE; //todo tot este premysliet
        res->success = true;
        return true;
    }



    bool 
    RosInterface::startAcquisition(const std::shared_ptr<std_srvs::srv::Empty::Request> req, 
                                    std::shared_ptr<std_srvs::srv::Empty::Response> res) {
        try {
            PhoXiInterface::startAcquisition();
            start_acquisition_ = true; 
            this->set_parameter(rclcpp::Parameter("start_acquisition_", start_acquisition_));

            start_acquisition_ = this->get_parameter("start_acquisition_").as_bool();
            RCLCPP_INFO(this->get_logger(), "Hello %s", start_acquisition_ ? "true" : "false");

            // dynamicReconfigureConfig.start_acquisition = true;
            // dynamicReconfigureServer.updateConfig(dynamicReconfigureConfig);
            // diagnosticUpdater.force_update();
        } catch (PhoXiInterfaceException& e) {
            RCLCPP_ERROR(this->get_logger(), "%s", e.what());
        }
        return true;
    }

 

    bool 
    RosInterface::startAcquisitionV2(const std::shared_ptr<phoxi_msgs::srv::Empty::Request> req, 
                                     std::shared_ptr<phoxi_msgs::srv::Empty::Response> res) {
        try {
            start_acquisition_ = true; 
            this->set_parameter(rclcpp::Parameter("start_acquisition_", start_acquisition_));
            // dynamicReconfigureConfig.start_acquisition = true;
            // dynamicReconfigureServer.updateConfig(dynamicReconfigureConfig);
            PhoXiInterface::startAcquisition();
            res->message = OKRESPONSE;
            res->success = true;
        } catch (PhoXiInterfaceException& e) {
            res->message = e.what();
            res->success = false;
        }
        return true;
    }

    bool
    RosInterface::stopAcquisition(const std::shared_ptr<std_srvs::srv::Empty::Request> req, 
                                   std::shared_ptr<std_srvs::srv::Empty::Response> res) {
        try {
            PhoXiInterface::stopAcquisition();
            start_acquisition_ = false; 
            this->set_parameter(rclcpp::Parameter("start_acquisition_", start_acquisition_));
            // dynamicReconfigureConfig.start_acquisition = false;
            // dynamicReconfigureServer.updateConfig(dynamicReconfigureConfig);
            // diagnosticUpdater.force_update();
        } catch (PhoXiInterfaceException& e) {
            RCLCPP_ERROR(this->get_logger(), "%s", e.what());
        }
        return true;
    }

    bool 
    RosInterface::stopAcquisitionV2(const std::shared_ptr<phoxi_msgs::srv::Empty::Request> req, 
                                     std::shared_ptr<phoxi_msgs::srv::Empty::Response> res) {
        try {
            start_acquisition_ = false; 
            this->set_parameter(rclcpp::Parameter("start_acquisition_", start_acquisition_));
            // dynamicReconfigureConfig.start_acquisition = false;
            // dynamicReconfigureServer.updateConfig(dynamicReconfigureConfig);
            PhoXiInterface::stopAcquisition();
            res->message = OKRESPONSE;
            res->success = true;
        } catch (PhoXiInterfaceException& e) {
            res->message = e.what();
            res->success = false;
        }
        return true;
    }

    int 
    RosInterface::triggerImageid() {
        int id = PhoXiInterface::triggerImage(false);
        //update dynamic reconfigure

        this->set_parameter(rclcpp::Parameter("coordinate_space_", pho::api::PhoXiTriggerMode::Software));
        this->set_parameter(rclcpp::Parameter("start_acquisition_", true));
        // dynamicReconfigureConfig.coordinate_space = pho::api::PhoXiTriggerMode::Software;
        // dynamicReconfigureConfig.start_acquisition = true;
        // dynamicReconfigureServer.updateConfig(dynamicReconfigureConfig);
        return id;
    }

    bool
    RosInterface::triggerImage(const std::shared_ptr<phoxi_msgs::srv::TriggerImage::Request> req, 
                                std::shared_ptr<phoxi_msgs::srv::TriggerImage::Response> res) {
        try {
            res->id = RosInterface::triggerImageid();
            res->success = true;
            res->message = OKRESPONSE;
        } catch (PhoXiInterfaceException& e) {
            res->success = false;
            res->message = e.what();
        }
        return true;
    }

    

    pho::api::PFrame RosInterface::getPFrame(int id) {
        pho::api::PFrame frame = PhoXiInterface::getPFrame(id);
        trigger_mode_ = PhoXiInterface::scanner->TriggerMode.GetValue();
        start_acquisition_ = PhoXiInterface::scanner->isAcquiring();
        this->set_parameter(rclcpp::Parameter("trigger_mode_", trigger_mode_));
        this->set_parameter(rclcpp::Parameter("start_acquisition_", start_acquisition_));
        //update dynamic reconfigure
        // dynamicReconfigureConfig.trigger_mode = PhoXiInterface::scanner->TriggerMode.GetValue();
        // dynamicReconfigureConfig.start_acquisition = PhoXiInterface::scanner->isAcquiring();
        // dynamicReconfigureServer.updateConfig(dynamicReconfigureConfig);
        return frame;
    }

    bool 
    RosInterface::getFrame(const std::shared_ptr<phoxi_msgs::srv::GetFrame::Request> req, 
                                std::shared_ptr<phoxi_msgs::srv::GetFrame::Response> res) {
        try {
            pho::api::PFrame frame = getPFrame(req->in_id);
            publishFrame(frame);
            if (!frame) {
                res->success = false;
                res->message = "Null frame!";
            } else {
                res->success = true;
                res->message = OKRESPONSE;
            }
        } catch (PhoXiInterfaceException& e) {
            res->success = false;
            res->message = e.what();
        }
        return true;
    }

    bool 
    RosInterface::saveFrame(const std::shared_ptr<phoxi_msgs::srv::SaveFrame::Request> req, 
                                std::shared_ptr<phoxi_msgs::srv::SaveFrame::Response> res) {
        try {
            pho::api::PFrame frame = RosInterface::getPFrame(req->in_id);
            if (!frame) {
                res->success = false;
                res->message = "Null frame!";
                return true;
            }
            size_t pos = req->path.find("~");
            if (pos != std::string::npos) {
                char* home = std::getenv("HOME");
                if (!home) {
                    res->message = "'~' found in 'path' parameter but environment variable 'HOME' not found. Export' HOME' variable or pass absolute value to 'path' parameter.";
                    res->success = false;
                    return true;
                }
                req->path.replace(pos, 1, home);
            }
            RCLCPP_INFO(this->get_logger(), "path: %s", req->path.c_str());
            
            frame->SaveAsPly(req->path);
            res->message = OKRESPONSE;
            res->success = true;
        } catch (PhoXiInterfaceException& e) {
            res->success = false;
            res->message = e.what();
        }
        return true;
    }


    void RosInterface::diagnosticCallback(diagnostic_updater::DiagnosticStatusWrapper & status) {
        if (PhoXiInterface::isConnected()) {
            if (PhoXiInterface::isAcquiring()) {
                status.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Ready");
            } else {
                status.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Acquisition not started");
            }
            status.add("HardwareIdentification", std::string(scanner->HardwareIdentification));
            status.add("Trigger mode", getTriggerMode(scanner->TriggerMode));

        } else {
            status.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Not connected");
        }
    }

    

    std::string RosInterface::getTriggerMode(pho::api::PhoXiTriggerMode mode) {
        switch (mode) {
            case pho::api::PhoXiTriggerMode::Freerun:
                return "Freerun";
            case pho::api::PhoXiTriggerMode::Software:
                return "Software";
            case pho::api::PhoXiTriggerMode::Hardware:
                return "Hardware";
            case pho::api::PhoXiTriggerMode::NoValue:
                return "NoValue";
            default:
                return "Undefined";
        }
    }

    void 
    RosInterface::publishFrame(pho::api::PFrame frame) {
        if (!frame) {
            RCLCPP_WARN(this->get_logger(), "NUll frame!");
            return;
        }

        rclcpp::Time timeNow = this->now();
 

        std_msgs::msg::Header header;
        header.stamp = timeNow;
        header.frame_id = frameId;
    

        if (frame->PointCloud.Empty()) {
            RCLCPP_WARN(this->get_logger(), "Empty point cloud!");
        } else {
            organized_cloud_ = this->get_parameter("organized_cloud_").as_bool();
            auto cloud = PhoXiInterface::getPointCloudFromFrame(frame, organized_cloud_);
            sensor_msgs::msg::PointCloud2 output_cloud;
            pcl::toROSMsg(*cloud, output_cloud);
            output_cloud.header = header;
            cloudPub_->publish(output_cloud);
        }

        if (frame->DepthMap.Empty()) {
            RCLCPP_WARN(this->get_logger(), "Empty depth map!");

        } else {
            auto depth_map = std::make_shared<sensor_msgs::msg::Image>();
            depth_map->header = header;
            depth_map->encoding = sensor_msgs::image_encodings::TYPE_32FC1;

            depth_map->width = frame->DepthMap.Size.Width;
            depth_map->height = frame->DepthMap.Size.Height;
            depth_map->step = frame->DepthMap.Size.Width * sizeof(float);
            size_t size = frame->DepthMap.Size.Width * frame->DepthMap.Size.Height * sizeof(float);
            depth_map->data.resize(size);
            memcpy(depth_map->data.data(), frame->DepthMap.operator[](0), size);

            depthMapPub_->publish(*depth_map);
        }

        if (frame->Texture.Empty()) {
            RCLCPP_WARN(this->get_logger(), "Empty texture!");

        } else {
            auto texture = std::make_shared<sensor_msgs::msg::Image>();
            texture->header = header;
            texture->encoding = sensor_msgs::image_encodings::TYPE_32FC1;
            texture->width = frame->Texture.Size.Width;
            texture->height = frame->Texture.Size.Height;
            texture->step = frame->Texture.Size.Width * sizeof(float);
            size_t size = frame->Texture.Size.Width * frame->Texture.Size.Height * sizeof(float);
            texture->data.resize(size);
            memcpy(texture->data.data(), frame->Texture.operator[](0), size);

            rawTexturePub_->publish(*texture);

            cv::Mat cvGreyTexture(frame->Texture.Size.Height, frame->Texture.Size.Width, CV_32FC1,
                                  frame->Texture.operator[](0));
            cv::normalize(cvGreyTexture, cvGreyTexture, 0, 255, CV_MINMAX);
            cvGreyTexture.convertTo(cvGreyTexture, CV_8U);
            cv::equalizeHist(cvGreyTexture, cvGreyTexture);
            cv::Mat cvRgbTexture;
            cv::cvtColor(cvGreyTexture, cvRgbTexture, CV_GRAY2RGB);

            std_msgs::msg::Header header;
            sensor_msgs::msg::Image::SharedPtr rgbTexturemsg;
            cv_bridge::CvImage cv_image(header, sensor_msgs::image_encodings::RGB8, cvRgbTexture);
            rgbTexturemsg = cv_image.toImageMsg();

            rgbTexturePub_->publish(*rgbTexturemsg);
        }
        if (frame->ConfidenceMap.Empty()) {
            RCLCPP_WARN(this->get_logger(), "Empty confidence map!");

        } else {
            auto confidence_map = std::make_shared<sensor_msgs::msg::Image>();
            confidence_map->header = header;
            confidence_map->encoding = sensor_msgs::image_encodings::TYPE_32FC1;
            confidence_map->width = frame->ConfidenceMap.Size.Width;
            confidence_map->height = frame->ConfidenceMap.Size.Height;
            confidence_map->step = frame->ConfidenceMap.Size.Width * sizeof(float);
            size_t size = frame->ConfidenceMap.Size.Width * frame->ConfidenceMap.Size.Height * sizeof(float);
            confidence_map->data.resize(size);
            memcpy(confidence_map->data.data(), frame->ConfidenceMap.operator[](0), size);
            
            confidenceMapPub_->publish(*confidence_map);
        }

        if (frame->NormalMap.Empty()) {
            RCLCPP_WARN(this->get_logger(), "Empty normal map!");

        } else {
            auto normal_map = std::make_shared<sensor_msgs::msg::Image>();
            normal_map->header = header;
            normal_map->encoding = sensor_msgs::image_encodings::TYPE_32FC3;
            normal_map->width = frame->NormalMap.Size.Width;
            normal_map->height = frame->NormalMap.Size.Height;
            normal_map->step = frame->NormalMap.Size.Width * sizeof(float);
            size_t size = frame->NormalMap.Size.Width * frame->NormalMap.Size.Height * sizeof(float);
            normal_map->data.resize(size);
            memcpy(normal_map->data.data(), frame->NormalMap.operator[](0), size);
           
            normalMapPub_->publish(*normal_map);
        }

    }




    void RosInterface::initFromPhoXi() {
        
        if (!scanner->isConnected()) {
            RCLCPP_WARN(this->get_logger(), "Scanner not connected.");
            return;
        }
        ///resolution
        pho::api::PhoXiCapturingMode mode = scanner->CapturingMode;
        if ((mode.Resolution.Width == 2064) && (mode.Resolution.Height = 1544)) {
            resolution_ = 1; 
            this->set_parameter(rclcpp::Parameter("resolution_", resolution_));
        } else {
            resolution_ = 0; 
            this->set_parameter(rclcpp::Parameter("resolution_", resolution_));
        }
        

        pho::api::PhoXiCapturingSettings capturingSettings;
        capturingSettings = scanner->CapturingSettings;
        scan_multiplier_ = capturingSettings.ScanMultiplier;
        shutter_multiplier_ = capturingSettings.ShutterMultiplier;
        confidence_ = scanner->ProcessingSettings->Confidence;

        this->set_parameter(rclcpp::Parameter("scan_multiplier_", scan_multiplier_));
        this->set_parameter(rclcpp::Parameter("shutter_multiplier_", shutter_multiplier_));
        this->set_parameter(rclcpp::Parameter("confidence_", confidence_));


        pho::api::FrameOutputSettings outputSettings;
        outputSettings = scanner->OutputSettings;
        send_point_cloud_ = outputSettings.SendPointCloud;
        send_normal_map_ = outputSettings.SendNormalMap;
        send_confidence_map_ = outputSettings.SendConfidenceMap;
        send_depth_map_ = outputSettings.SendDepthMap;
        send_texture_ = outputSettings.SendTexture;
        

        this->set_parameter(rclcpp::Parameter("send_point_cloud_", send_point_cloud_));
        this->set_parameter(rclcpp::Parameter("send_normal_map_", send_normal_map_));
        this->set_parameter(rclcpp::Parameter("send_confidence_map_", send_confidence_map_));
        this->set_parameter(rclcpp::Parameter("send_depth_map_", send_depth_map_));
        this->set_parameter(rclcpp::Parameter("send_texture_", send_texture_));


#ifndef PHOXI_API_v1_1
        coordinate_space_ =  scanner->CoordinatesSettings->CoordinateSpace;
        ambient_light_suppression_ =  capturingSettings.AmbientLightSuppression;

        this->set_parameter(rclcpp::Parameter("coordinate_space_", coordinate_space_));
        this->set_parameter(rclcpp::Parameter("ambient_light_suppression_", ambient_light_suppression_));

        std::vector<double> supportedSPE = scanner->SupportedSinglePatternExposures;
        if (!supportedSPE.empty()) {
            auto actualParam_it = std::find(supportedSPE.begin(), supportedSPE.end(), capturingSettings.SinglePatternExposure);
            if (actualParam_it != supportedSPE.end()) {
                single_pattern_exposure_ =  actualParam_it - supportedSPE.begin();
                this->set_parameter(rclcpp::Parameter("single_pattern_exposure_", single_pattern_exposure_));
            } else {
                int singlePatternExposure_index;
                this->set_parameter(rclcpp::Parameter("single_pattern_exposure_", singlePatternExposure_index));
                RCLCPP_INFO(this->get_logger(), "single_pattern_exposure", singlePatternExposure_index);
 
                RCLCPP_WARN(this->get_logger(), "Can not update Single Pattern Exposure parameter in dynamic reconfigure, set default value from config.");
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "Scanner setting 'Single pattern exposure' is not supported by the scanner firmware.");
        }
        camera_only_mode_ = capturingSettings.CameraOnlyMode;
        this->set_parameter(rclcpp::Parameter("camera_only_mode_", camera_only_mode_));
#endif
        trigger_mode_ = scanner->TriggerMode.GetValue();
        start_acquisition_ = scanner->isAcquiring();
        timeout_ = scanner->Timeout.GetValue();
        this->set_parameter(rclcpp::Parameter("trigger_mode_", trigger_mode_));
        this->set_parameter(rclcpp::Parameter("start_acquisition_", start_acquisition_));
        this->set_parameter(rclcpp::Parameter("timeout_", timeout_));

    }

    bool RosInterface::disconnectCamera(const std::shared_ptr<std_srvs::srv::Empty::Request> req, 
                                        std::shared_ptr<std_srvs::srv::Empty::Response> res) {
        try {
            PhoXiInterface::disconnectCamera();
            // diagnosticUpdater.force_update();
        } catch (PhoXiInterfaceException& e) {
            //scanner is already disconnected on exception
        }
        return true;
    }

    bool RosInterface::getHardwareIdentification(const std::shared_ptr<phoxi_msgs::srv::GetHardwareIdentification::Request> req, 
                                                 std::shared_ptr<phoxi_msgs::srv::GetHardwareIdentification::Response> res) {
        try {
            res->hardware_identification = PhoXiInterface::getHardwareIdentification();
            res->success = true;
            res->message = OKRESPONSE;
        } catch (PhoXiInterfaceException& e) {
            res->success = false;
            res->message = e.what();
        }
        return true;
    }

    bool RosInterface::getSupportedCapturingModes(const std::shared_ptr<phoxi_msgs::srv::GetSupportedCapturingModes::Request> req, 
                                                    std::shared_ptr<phoxi_msgs::srv::GetSupportedCapturingModes::Response> res) {
        try {
            std::vector<pho::api::PhoXiCapturingMode> modes = PhoXiInterface::getSupportedCapturingModes();
            for (int i = 0; i < modes.size(); i++) {
                phoxi_msgs::msg::PhoXiSize size;
                size.height = modes[i].Resolution.Height;
                size.width = modes[i].Resolution.Width;
                res->supported_capturing_modes.push_back(size);
            }
            res->success = true;
            res->message = OKRESPONSE;
        } catch (PhoXiInterfaceException& e) {
            res->success = false;
            res->message = e.what();
        }
        return true;
    }

    bool 
    RosInterface::getApiVersion(const std::shared_ptr<phoxi_msgs::srv::GetString::Request> req, 
                                    std::shared_ptr<phoxi_msgs::srv::GetString::Response> res) {
        res->value = PhoXiInterface::getApiVersion();
        res->success = true;
        return true;
    }

    bool
    RosInterface::getFirmwareVersion(const std::shared_ptr<phoxi_msgs::srv::GetString::Request> req, 
                                     std::shared_ptr<phoxi_msgs::srv::GetString::Response> res) {
        try {
            auto dl = PhoXiInterface::deviceList();
            auto it = std::find(dl.begin(), dl.end(), PhoXiInterface::getHardwareIdentification());
            if (it != dl.end()) {
                res->value = it->firmwareversion;
            }
        }
        catch (PhoXiInterfaceException& e) {
            res->message = e.what();
            res->success = false;
        }
        res->success = true;
        return true;
    }


// #ifndef PHOXI_API_v1_1

    bool 
    RosInterface::setCoordianteSpace(const std::shared_ptr<phoxi_msgs::srv::SetCoordinatesSpace::Request> req, 
                                         std::shared_ptr<phoxi_msgs::srv::SetCoordinatesSpace::Response> res) {
        try {
            PhoXiInterface::setCoordinateSpace(req->coordinates_space);
            //update dynamic reconfigure
            coordinate_space_ = req->coordinates_space;
            this->set_parameter(rclcpp::Parameter("coordinate_space_", coordinate_space_));
            res->success = true;
            res->message = OKRESPONSE;
        } catch (PhoXiInterfaceException& e) {
            res->success = false;
            res->message = e.what();
        }
        return true;
    }

    bool 
    RosInterface::setTransformation(const std::shared_ptr<phoxi_msgs::srv::SetTransformationMatrix::Request> req, 
                                         std::shared_ptr<phoxi_msgs::srv::SetTransformationMatrix::Response> res) {
        try {

            // tf2::Transform tf2Transform;
            Eigen::Affine3d transform = tf2::transformToEigen(req->transform);
           
            // tf2::convert(tf2Transform, transform);
            // tf::transformMsgToEigen(req->transform, transform);
            PhoXiInterface::setTransformation(transform.matrix(), req->coordinates_space, req->set_space,
                                              req->save_settings);
            //update dynamic reconfigure
            if (req->set_space) {
                coordinate_space_ = req->coordinates_space;
                this->set_parameter(rclcpp::Parameter("coordinate_space_", coordinate_space_));
            }         
            res->success = true;
            res->message = OKRESPONSE;
        } catch (PhoXiInterfaceException& e) {
            res->success = false;
            res->message = e.what();
        }
        return true;
    }

    bool
    RosInterface::saveLastFrame(const std::shared_ptr<phoxi_msgs::srv::SaveLastFrame::Request> req, 
                                std::shared_ptr<phoxi_msgs::srv::SaveLastFrame::Response> res) {
        std::string file_path = req->file_path;

        try {
            // ~ error handling
            size_t pos = file_path.find('~');
            if (pos != std::string::npos) {
                char* home = std::getenv("HOME");
                if (!home) {
                    res->message = "'~' found in 'file_path' parameter but environment variable 'HOME' not found. Export' HOME' variable or pass absolute value to 'path' parameter.";
                    res->success = false;
                    return true;
                }
                file_path.replace(pos, 1, home);
            }

            // extension error handling
            const std::string extensions[] = {"praw", "ply", "ptx", "tif", "prawf"};
            bool correct_ext = false;
            pos = file_path.rfind('.') + 1;
            std::string extension = file_path.substr(pos, file_path.length());

            for (const auto& ext: extensions) {
                if (ext == extension) {
                    correct_ext = true;
                }
            }
            if (!correct_ext) {
                res->message = "Wrong extension.";
                res->success = false;
                return true;
            }

            // save last frame
            RCLCPP_INFO(this->get_logger(),"File path: %s", file_path.c_str());
            bool result = PhoXiInterface::saveLastFrame(file_path);
            if (result) {
                res->message = OKRESPONSE;
                res->success = true;
            } else {
                res->message = "Unsuccessful save.";
                res->success = false;
            }

        } catch (PhoXiInterfaceException& e) {
            res->success = false;
            res->message = e.what();
        }

        return true;
    }

// #endif


    // RosInterface::~RosInterface(){
    // }  


}                                                                          