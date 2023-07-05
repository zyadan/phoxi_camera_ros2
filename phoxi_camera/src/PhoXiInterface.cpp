//
// Created by controller on 1/11/18.
//

#include <phoxi_camera/PhoXiInterface.h>
#include <phoxi_camera/PhoXiConversions.h>

namespace phoxi_camera {
    std::vector<PhoXiDeviceInformation> PhoXiInterface::deviceList() {
        if (!phoXiFactory.isPhoXiControlRunning()) {
            scanner.Reset();
            throw PhoXiControlNotRunning("PhoXi Control is not running");
        }
        std::vector<PhoXiDeviceInformation> deviceInfo;
        auto dl = phoXiFactory.GetDeviceList();
        toPhoXiCameraDeviceInforamtion(dl, deviceInfo);

        std::map<std::string, std::string> scannersIPs;

        try {
            scannersIPs = PhoXiInterface::getScannersIPs();
        }
        catch (PhoXiInterfaceException& e){}

        for (auto &device : deviceInfo) {
            if (scannersIPs.empty() || (scannersIPs.find(device.hwidentification) == scannersIPs.end())) {
                device.ipaddress = "unknown";
            } else {
                device.ipaddress = scannersIPs.at(device.hwidentification);
            }
        }

        return deviceInfo;
    }

    std::vector<std::string> PhoXiInterface::cameraList() {

        auto dl = deviceList();
        std::vector<std::string> hwidentificationList;
        for (const auto& device : dl) {
            hwidentificationList.push_back(device);
        }
        return hwidentificationList;
    }

    void PhoXiInterface::connectCamera(std::string hwidentification, pho::api::PhoXiTriggerMode mode,
                                       bool startAcquisition) {
        if (this->isConnected()) {
            if (scanner->HardwareIdentification == hwidentification) {
                this->setTriggerMode(mode, startAcquisition);
                return;
            }
        }
        std::vector<std::string> cl = cameraList();
        auto it = std::find(cl.begin(), cl.end(), hwidentification);
        if (it == cl.end()) {
            throw PhoXiScannerNotFound("Scanner not found");
        }
        disconnectCamera();
        scanner.Reset();    // This is needed before assign new instance
        if (!(scanner = phoXiFactory.CreateAndConnect(*it, 5000))) {
            disconnectCamera();
            throw UnableToStartAcquisition("Scanner was not able to connect. Disconnected.");
        }
        this->setTriggerMode(mode, startAcquisition);
    }

    void PhoXiInterface::disconnectCamera() {
        if (scanner && scanner->isConnected()) {
            scanner->Disconnect(false);
        }
        scanner.Reset();    // Scanner instance is not usable after disconnect, call destructor
        last_frame_id = -1;
    }

    pho::api::PFrame PhoXiInterface::getPFrame(int id) {
        this->isOk();

        if (id < 0) {
            try {
                id = this->triggerImage(true);
            } catch (UnableToTriggerFrame& e) {
                throw;
            }
        }

        return scanner->GetSpecificFrame(id, 10000);
    }

    std::shared_ptr<pcl::PointCloud<pcl::PointNormal>> PhoXiInterface::getPointCloud(bool organized) {
        return getPointCloudFromFrame(getPFrame(-1), organized);
    }

    std::shared_ptr<pcl::PointCloud<pcl::PointNormal>> PhoXiInterface::getPointCloudFromFrame(pho::api::PFrame frame, bool organized) {
        if (organized) {
            return getOrganizedCloudFromFrame(frame);
        }
        return getUnorganizedCloudFromFrame(frame);
    }

    std::shared_ptr<pcl::PointCloud<pcl::PointNormal>>
    PhoXiInterface::getOrganizedCloudFromFrame(pho::api::PFrame frame) {
        if (!frame || !frame->Successful) {
            throw CorruptedFrame("Corrupted frame!");
        }
        std::shared_ptr<pcl::PointCloud<pcl::PointNormal>> cloud(
                new pcl::PointCloud<pcl::PointNormal>(frame->GetResolution().Width, frame->GetResolution().Height));
        for (int r = 0; r < frame->GetResolution().Height; r++) {
            for (int c = 0; c < frame->GetResolution().Width; c++) {
                auto point = frame->PointCloud.At(r, c);
                pcl::PointNormal pclPoint;
                pclPoint.x = point.x / 1000;                 //to [m]
                pclPoint.y = point.y / 1000;                 //to [m]
                pclPoint.z = point.z / 1000;                 //to [m]
                if (!frame->NormalMap.Empty()) {
                    auto normal = frame->NormalMap.At(r, c);
                    pclPoint.normal_x = normal.x / 1000;    //to [m]
                    pclPoint.normal_y = normal.y / 1000;    //to [m]
                    pclPoint.normal_z = normal.z / 1000;    //to [m]
                }
                cloud->at(c, r) = pclPoint;
            }
        }
        return cloud;
    }

    std::shared_ptr<pcl::PointCloud<pcl::PointNormal>> PhoXiInterface::getUnorganizedCloudFromFrame(pho::api::PFrame frame) {
        if (!frame || !frame->Successful) {
            throw CorruptedFrame("Corrupted frame!");
        }
        std::shared_ptr<pcl::PointCloud<pcl::PointNormal>> cloud(new pcl::PointCloud<pcl::PointNormal>());
        for (int r = 0; r < frame->GetResolution().Height; r++) {
            for (int c = 0; c < frame->GetResolution().Width; c++) {
                auto point = frame->PointCloud.At(r, c);
                if (point == pho::api::Point3_32f(0, 0, 0)) {
                    continue;
                }
                pcl::PointNormal pclPoint;
                pclPoint.x = point.x / 1000;                 //to [m]
                pclPoint.y = point.y / 1000;                 //to [m]
                pclPoint.z = point.z / 1000;                 //to [m]
                if (!frame->NormalMap.Empty()) {
                    auto normal = frame->NormalMap.At(r, c);
                    pclPoint.normal_x = normal.x / 1000;    //to [m]
                    pclPoint.normal_y = normal.y / 1000;    //to [m]
                    pclPoint.normal_z = normal.z / 1000;    //to [m]
                }
                cloud->push_back(pclPoint);
            }
        }
        return cloud;
    }

    void PhoXiInterface::isOk() {
        if (!scanner || !scanner->isConnected()) {
            throw PhoXiScannerNotConnected("No scanner connected");
        }
    }

    std::string PhoXiInterface::getHardwareIdentification() {
        this->isOk();
        return scanner->HardwareIdentification;
    }

    bool PhoXiInterface::isConnected() {
        return (scanner && scanner->isConnected());
    }

    bool PhoXiInterface::isAcquiring() {
        return (scanner && scanner->isAcquiring());
    }

    void PhoXiInterface::startAcquisition() {
        this->isOk();
        if (scanner->isAcquiring()) {
            return;
        }
        scanner->StartAcquisition();
        if (!scanner->isAcquiring()) {
            throw UnableToStartAcquisition("Unable to start acquisition.");
        }
    }

    void PhoXiInterface::stopAcquisition() {
        this->isOk();
        if (!scanner->isAcquiring()) {
            return;
        }
        scanner->StopAcquisition();
        if (scanner->isAcquiring()) {
            throw UnableToStopAcquisition("Unable to stop acquisition.");
        }
    }

    int PhoXiInterface::triggerImage(bool waitForGrab) {
        this->setTriggerMode(pho::api::PhoXiTriggerMode::Software, true);
        int frame_id = scanner->TriggerFrame(true, waitForGrab);
        last_frame_id = frame_id;

        if (frame_id < 0) {
            switch (frame_id) {
                case -1:
                    throw UnableToTriggerFrame("Trigger not accepted.");
                case -2:
                    throw UnableToTriggerFrame("Device is not running.");
                case -3:
                    throw UnableToTriggerFrame("Communication Error.");
                case -4:
                    throw UnableToTriggerFrame("WaitForGrabbingEnd is not supported.");
                default:
                    throw UnableToTriggerFrame("Unknown error.");
            }
        }
        return frame_id;
    }

    std::vector<pho::api::PhoXiCapturingMode> PhoXiInterface::getSupportedCapturingModes() {
        this->isOk();
        return scanner->SupportedCapturingModes;
    }

    void PhoXiInterface::setHighResolution() {
        this->isOk();
        pho::api::PhoXiCapturingMode mode = scanner->CapturingMode;
        mode.Resolution.Width = 2064;
        mode.Resolution.Height = 1544;
        scanner->CapturingMode = mode;
    }

    void PhoXiInterface::setLowResolution() {
        this->isOk();
        pho::api::PhoXiCapturingMode mode = scanner->CapturingMode;
        mode.Resolution.Width = 1032;
        mode.Resolution.Height = 772;
        scanner->CapturingMode = mode;
    }

    void PhoXiInterface::setTriggerMode(pho::api::PhoXiTriggerMode mode, bool startAcquisition) {
        if (!((mode == pho::api::PhoXiTriggerMode::Software) || (mode == pho::api::PhoXiTriggerMode::Hardware) ||
              (mode == pho::api::PhoXiTriggerMode::Freerun) || (mode == pho::api::PhoXiTriggerMode::NoValue))) {
            throw InvalidTriggerMode("Invalid trigger mode " + std::to_string(mode) + ".");
        }
        this->isOk();
        if (mode == scanner->TriggerMode.GetValue()) {
            if (startAcquisition) {
                this->startAcquisition();
            } else {
                this->stopAcquisition();
            }
            return;
        }
        this->stopAcquisition();
        scanner->TriggerMode = mode;
        if (startAcquisition) {
            this->startAcquisition();
        }
    }

    pho::api::PhoXiTriggerMode PhoXiInterface::getTriggerMode() {
        this->isOk();
        return scanner->TriggerMode;
    }

#ifndef PHOXI_API_v1_1

    void PhoXiInterface::setCoordinateSpace(pho::api::PhoXiCoordinateSpace space) {
        this->isOk();
        scanner->CoordinatesSettings->CoordinateSpace = space;
    }

    pho::api::PhoXiCoordinateSpace PhoXiInterface::getCoordinateSpace() {
        this->isOk();
        return scanner->CoordinatesSettings->CoordinateSpace;
    }

    void PhoXiInterface::setTransformation(pho::api::PhoXiCoordinateTransformation coordinateTransformation,
                                           pho::api::PhoXiCoordinateSpace space, bool setSpace,
                                           bool saveSettings) {
        this->isOk();
        pho::api::PhoXiCoordinatesSettings settings = scanner->CoordinatesSettings;
        switch (space) {
            case pho::api::PhoXiCoordinateSpace::RobotSpace:
                if (!settings.RobotTransformation.isSupported()) {
                    throw CoordinateSpaceNotSupported("Coordinate space is not supported");
                }
                settings.RobotTransformation = coordinateTransformation;
                settings.CoordinateSpace = pho::api::PhoXiCoordinateSpace::RobotSpace;
                break;
            case pho::api::PhoXiCoordinateSpace::CustomSpace:
                if (!settings.CustomTransformation.isSupported()) {
                    throw CoordinateSpaceNotSupported("Coordinate space is not supported");
                }
                settings.CustomTransformation = coordinateTransformation;
                settings.CoordinateSpace = pho::api::PhoXiCoordinateSpace::CustomSpace;
                break;
            default:
                throw CoordinateSpaceNotSupported("Coordinate space is not supported");
        }
        if (setSpace) {
            settings.CoordinateSpace = space;
        }
        this->isOk();
        scanner->CoordinatesSettings = settings;
        if (saveSettings) {
            scanner->SaveSettings();
        }
    }

    bool PhoXiInterface::saveLastFrame(const std::string& path) {
        if (last_frame_id == -1) {
            // this is needed due to PhoXi API bug
            // if you didn't trigger frame before, you can not save it
            return false;
        }

        bool success = scanner->SaveLastOutput(path, this->last_frame_id);
        return success;
    }

#endif

    std::string PhoXiInterface::getApiVersion() {
        return PHOXI_API_VERSION;
    }

    int PhoXiInterface::countRowsWithStartingSign(char sign, std::vector<std::string> rowsVector){
        int signRowsCounter = 0;
        for(std::string row: rowsVector){
            if(row.at(0) == sign)
                signRowsCounter++;
        }
        return signRowsCounter;
    }

    std::string PhoXiInterface::getContentBetweenSign(const std::string &str, const std::string &begin, const std::string &end) {
        auto first = str.find_first_of(begin);
        if (first == std::string::npos)
            return "";

        auto last = str.find_last_of(end);
        if (last == std::string::npos)
            return "";

        return str.substr(first + 1, last - first - 1);
    }

    bool PhoXiInterface::checkScannersPresence(const std::vector<std::string> &scannersList, const std::string &scannerID) {
        for (auto &currScannerID : scannersList) {
            if (currScannerID == scannerID)
                return true;
        }
        return false;
    }

    std::vector<std::string> PhoXiInterface::getAvailableScanersID(AvahiRowDataType type, const std::vector<std::string> &stdoutPipe) {
        std::vector<std::string> scannersList;
        std::string scannerMark = "PhoXi3DScan-";
        std::string scriptMark = "_3d-camera._tcp";

        for (auto &row : stdoutPipe) {
            if (row.front() == type) {
                std::size_t scannerMarkerPos = row.find(scannerMark);
                if (scannerMarkerPos != std::string::npos) {
                    std::string scannerRow = row.substr(scannerMarkerPos);
                    std::size_t scriptMarkerPos = scannerRow.find(scriptMark);

                    std::size_t scannersNameBegin = scannerMarkerPos + (std::size_t) scannerMark.size();
                    std::size_t scannersNameEnd = scriptMarkerPos - (std::size_t) scannerMark.size();

                    std::string scannersName = row.substr(scannersNameBegin, scannersNameEnd);
                    scannersName.erase(remove_if(scannersName.begin(), scannersName.end(), isspace), scannersName.end());

                    if(!(std::find(scannersList.begin(), scannersList.end(), scannersName) != scannersList.end())){
                        scannersList.emplace_back(scannersName);
                    }
                }
            }
        }
        return scannersList;
    }

    std::map<std::string, std::string> PhoXiInterface::getScannersIPs() {
        std::map<std::string, std::string> scannersIPs;

        // stderr from avahi-browse is redirected (and handled later) to stdout
        const char *command = "(timeout 0.5 avahi-browse -r -t _3d-camera._tcp) 2>&1";
        char buffer[256];
        std::string result;
        FILE *pipe = popen(command, "r");

        std::vector<std::string> connectedScanners;
        std::vector<std::string> disappearedScanners;
        std::vector<std::string> stderrOutput;

        if (!pipe) {
            pclose(pipe);
            throw AvahiFailed("Avahi-browse operation failed");
        }

        while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
            if (ferror(pipe)) {
                pclose(pipe);
                throw AvahiUnexpectedResults("Unexpected results from avahi-browse command!");
            }
            char startingSign = buffer[0];

            if ((startingSign == AvahiRowDataType::available) || (startingSign == AvahiRowDataType::scannersInfo)
                || (startingSign == AvahiRowDataType::other)) {
                connectedScanners.emplace_back(buffer);
            } else if (startingSign == AvahiRowDataType::disappeared) {
                disappearedScanners.emplace_back(buffer);
            } else {
                stderrOutput.emplace_back(buffer);
            }
        }
        pclose(pipe);

        if (connectedScanners.empty()) {
            return scannersIPs;
        }

        std::vector<std::string> scannersList = getAvailableScanersID(AvahiRowDataType::available, connectedScanners);
        std::vector<std::string> disappearedScannersList = getAvailableScanersID(AvahiRowDataType::disappeared, disappearedScanners);

        int numberRowsToDelete = countRowsWithStartingSign(AvahiRowDataType::available, connectedScanners);
        connectedScanners.erase(connectedScanners.begin(), connectedScanners.begin() + numberRowsToDelete);

        for (std::string scannerID: scannersList) {
            if (!checkScannersPresence(scannersList, scannerID) ||
                checkScannersPresence(disappearedScannersList, scannerID)) {
                scannersIPs.insert(std::pair<std::string, std::string>(scannerID, "unknown"));
                continue;
            }

            // find corresponding row for current scanner information
            bool rowFound = false;
            int scannersInfoRowNum = 0;

            // we need first occurrence of scannerID to get right row number
            for (int i = 0; (i <= connectedScanners.size() - 1) && !rowFound; i++) {
                std::size_t scannersInfoPos = connectedScanners.at((unsigned long) i).find(scannerID);
                if (scannersInfoPos != std::string::npos) {
                    scannersInfoRowNum = i;
                    rowFound = true;
                }
            }

            // scanners response not present in stdout
            if (!rowFound) {
                continue;
            }

            std::vector<std::string> scannersInfo;
            for (int j = scannersInfoRowNum + 1; j < scannersInfoRowNum + 5; j++) {
                scannersInfo.emplace_back(getContentBetweenSign(connectedScanners.at(j), "[", "]"));
            }

//            Other scanners parameters can be obtained from:
//              hostname => scannersInfo.at(0);
//              IPaddress => scannersInfo.at(1);
//              port => scannersInfo.at(2);
//              [Occupied_By, changeId, version, status, description, id...,
//              contain depends on FW version] => scannersInfo.at(3);

            std::string scannersIP = scannersInfo.at(1);
            scannersIPs.insert(std::pair<std::string, std::string>(scannerID, scannersIP));
        }

        // SILENT avahi-browse warnings/errors
        // we are terminating avahi-browse after 500ms (all available scanners provides their info under 500ms)
//        if (!stderrOutput.empty()) {
//            for (auto &err : stderrOutput) {
//                if (err.find("Got SIGTERM, quitting") != std::string::npos) {
//                } else {
//                    std::cout << err << std::endl;
//                }
//            }
//        }
        return scannersIPs;
    }
}
