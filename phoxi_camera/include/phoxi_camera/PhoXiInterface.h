//
// Created by controller on 1/11/18.
//

#ifndef PROJECT_PHOXIINTERFACE_H
#define PROJECT_PHOXIINTERFACE_H

#include <PhoXi.h>
#include <pcl/point_types.h>
// #include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Core>
#include <phoxi_camera/PhoXiException.h>
#include <phoxi_camera/PhoXiDeviceInformation.h>

// PhoXi API version
#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)
#define PHOXI_API_VERSION STR(PHOXI_API_VERSION_MAJOR) "." STR(PHOXI_API_VERSION_MINOR) "." STR(PHOXI_API_VERSION_PATCH)
#if PHOXI_API_VERSION_MAJOR == 1
#if PHOXI_API_VERSION_MINOR == 1
#define PHOXI_API_v1_1
#elif PHOXI_API_VERSION_MINOR == 2
#define PHOXI_API_v1_2
#endif
#endif

//* PhoXiInterface
/**
 * Wrapper to PhoXi 3D Scanner api to make interface easier
 *
 */
namespace phoxi_camera {
    class PhoXiInterface {
    public:

        /**
        * Default constructor.
        */
        PhoXiInterface() = default;

        /**
        * Types of rows (chars representing starting sign of row) returned by avahi-browse call.
        */
        enum AvahiRowDataType : char {
            available = '+',
            disappeared = '-',
            scannersInfo = '=',
            other = ' '
        };

        /**
        * Return all PhoXi 3D Scanners ids connected on network with all informations about dcevice.
        *
        * \throw PhoXiControlNotRunning when PhoXi Control is not running
        */
        std::vector<PhoXiDeviceInformation> deviceList();

        /**
        * Return all PhoXi 3D Scanners ids connected on network.
        *
        * \note returned id can be passed to connectCamera method
        * \throw PhoXiControlNotRunning when PhoXi Control is not running
        */
        std::vector<std::string> cameraList();

        /**
        * Connect to camera.
        *
        * \param hwidentification - identification number
        * \param mode - trigger mode to set after connection
        * \param startAcquisition if true Acquisition will be started
        * \throw PhoXiControlNotRunning when PhoXi Control is not running
        * \throw PhoXiScannerNotFound when PhoXi 3D Scanner with HWIDENTIFICATION is not available on network
        * \throw UnableToStartAcquisition when connection failed
        */
        void connectCamera(std::string hwidentification,
                           pho::api::PhoXiTriggerMode mode = pho::api::PhoXiTriggerMode::Software,
                           bool startAcquisition = true);


        /**
        * Return rows count with starting sign - for avahi-browse output parsing.
        *
        * \param rowsVector - Vector of strings, containig avahi-browse call output
        * \param sign - considers rows that starts with this sign
        * \return rows count
        */
        int countRowsWithStartingSign(char sign, std::vector<std::string> rowsVector);

        /**
        * Disconnect from camera if connected to any.
        */
        void disconnectCamera();

        /**
        * Return all available scanners on network from avahi-browse call output
        *
        * \param stdoutPipe - avahi-browse call output
        * \param type - type of avahi-browse output row
        * \return vector of all available scanners on network
        */
        std::vector<std::string> getAvailableScanersID(AvahiRowDataType type, const std::vector<std::string> &stdoutPipe);

        /**
        * Return content from avahi-browse output row between begin and end signs.
        *
        * \param str - output row
        * \param begin - begin sign
        * \param end - end sign
        * \return string content
        */
        std::string getContentBetweenSign(const std::string &str, const std::string &begin,
                                          const std::string &end);

        /**
        * Get frame based on id. If id is negative new image is triggered and new PFrame returned.
        *
        * \note only last triggered frame can be returned - recommended usage is with negative number
        * \param id - frame id to return
        * \throw PhoXiScannerNotConnected when no scanner is connected
        */
        pho::api::PFrame getPFrame(int id = -1);

        /**
        * Get point cloud
        *
        * \param organized - if true return organized point cloud
        * \throw PhoXiScannerNotConnected when no scanner is connected
        */
        std::shared_ptr<pcl::PointCloud<pcl::PointNormal>> getPointCloud(bool organized = true);

        /**
        * Convert PFrame to point cloud
        *
        * \param organized - if true return organized point cloud
        */
        static std::shared_ptr<pcl::PointCloud<pcl::PointNormal>>
        getPointCloudFromFrame(pho::api::PFrame frame, bool organized = true);

        /**
        * Convert PFrame to organized point cloud
        *
        * \return organized point cloud
        */
        static std::shared_ptr<pcl::PointCloud<pcl::PointNormal>> getOrganizedCloudFromFrame(pho::api::PFrame frame);


        /**
        * Return map of all available scanners IP addresses, map key is scanner ID (avahi-browse is called)
        *
        * \return map of all available scanners IP addresses
        */
        std::map<std::string, std::string> getScannersIPs();


        /**
        * Convert PFrame to unorganized point cloud
        *
        * \return unorganized point cloud
        */
        static std::shared_ptr<pcl::PointCloud<pcl::PointNormal>> getUnorganizedCloudFromFrame(pho::api::PFrame frame);

        /**
        * Check if is scanner with scannerId present in scannersList
        *
        * \param scannersList - list of scannerIds
        * \param scannerID - scannerID to search for
        * \return true if is scanner present, false otherwise
        */
        bool checkScannersPresence(const std::vector<std::string> &scannersList, const std::string &scannerID);

        /**
        * Test if connection to PhoXi 3D Scanner is working
        *
        * \throw PhoXiScannerNotConnected when no scanner is connected
        */
        void isOk();

        /**
        * Test if connection to PhoXi 3D Scanner is working
        */
        bool isConnected();

        /**
        * Test if PhoXi 3D Scanner is Acquiring
        */
        bool isAcquiring();

        /**
        * Start acquisition
        *
        * \throw PhoXiScannerNotConnected when no scanner is connected
        * \throw UnableToStartAcquisition if acquisition was not started
        */
        void startAcquisition();

        /**
        * Stop acquisition
        *
        * \throw PhoXiScannerNotConnected when no scanner is connected
        * \throw UnableToStartAcquisition if acquisition was not stopped
        */
        void stopAcquisition();

        /**
        * Trigger new Image
        *
        * \return @return positive id on success, negative number on failure (-1 Trigger not accepted, -2 Device is not running, -3 Communication Error, -4 WaitForGrabbingEnd is not supported)
        * \note id can be passed to getPFrame method
        */
        int triggerImage(bool waitForGrab = false);

        /**
        * Get hardware identification number of currently connected PhoXi 3D Scanner
        *
        * \throw PhoXiScannerNotConnected when no scanner is connected
        */
        std::string getHardwareIdentification();

        /**
        * Get supported capturing modes
        *
        * \throw PhoXiScannerNotConnected when no scanner is connected
        */
        std::vector<pho::api::PhoXiCapturingMode> getSupportedCapturingModes();

        /**
        * Set high resolution (2064 x 1544)
        *
        * \throw PhoXiScannerNotConnected when no scanner is connected
        */
        void setHighResolution();

        /**
        * Set low resolution (1032 x 772)
        *
        * \throw PhoXiScannerNotConnected when no scanner is connected
        */
        void setLowResolution();

        /**
        * Set trigger mode
        *
        * \param mode new trigger mode
        * \param startAcquisition if true Acquisition will be started
        * \note if mode is Freerun new PFrames will be triggered immediately after acquisition is started
        *
        * \throw PhoXiScannerNotConnected when no scanner is connected
        * \throw InvalidTriggerMode when invalid trigger mode is passed
        */
        void setTriggerMode(pho::api::PhoXiTriggerMode mode, bool startAcquisition = false);

        /**
        * Get trigger mode
        *
        * \throw PhoXiScannerNotConnected when no scanner is connected
        */
        pho::api::PhoXiTriggerMode getTriggerMode();

#ifndef PHOXI_API_v1_1

        /**
        * Set coordinate space
        *
        * \param space - new coordinate space
        * \throw PhoXiScannerNotConnected when no scanner is connected
        */
        void setCoordinateSpace(pho::api::PhoXiCoordinateSpace space);

        /**
        * Get coordinate space
        *
        * \throw PhoXiScannerNotConnected when no scanner is connected
        */
        pho::api::PhoXiCoordinateSpace getCoordinateSpace();

        /**
        * Set transformation matrix space
        *
        * \param coordinateTransformation - transformation
        * \param coordinate coordinate space where to set transformation
        * \param setSpace if true space will be set
        * \param saveSettings if true settings will persist after restart (disconnection from device)
        * \throw PhoXiScannerNotConnected when no scanner is connected
        * \throw CoordinateSpaceNotSupported when space is not supported
        */
        void setTransformation(pho::api::PhoXiCoordinateTransformation coordinateTransformation,
                               pho::api::PhoXiCoordinateSpace space, bool setSpace = true, bool saveSettings = true);

        /**
        * Set transformation matrix space
        *
        * \param coordinateTransformation - transformation
        * \param coordinate coordinate space where to set transformation
        * \param setSpace if true space will be set
        * \param saveSettings if true settings will persist after restart (disconnection from device)
        * \note transformation can be set only to RobotSpace and CustomSpace
        * \throw PhoXiScannerNotConnected when no scanner is connected
        * \throw CoordinateSpaceNotSupported when space is not supported
        */
        template <typename T>
        void setTransformation(Eigen::Matrix<T, 4, 4> transformation, pho::api::PhoXiCoordinateSpace space,
                               bool setSpace = true, bool saveSettings = true) {
            setTransformation(getPhoXiCoordinateTransformation(transformation), space, setSpace, saveSettings);
        }

        template <typename T>

        /**
        * Convert eigen matrix to pho::api::PhoXiCoordinateTransformation
        */
        static pho::api::PhoXiCoordinateTransformation getPhoXiCoordinateTransformation(Eigen::Matrix<T, 4, 4> mat) {
            pho::api::PhoXiCoordinateTransformation coordinateTransformation;
            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 3; j++) {
                    coordinateTransformation.Rotation[i][j] = mat(i, j);
                }
            }
            coordinateTransformation.Translation.x = mat(0, 3);
            coordinateTransformation.Translation.y = mat(1, 3);
            coordinateTransformation.Translation.z = mat(2, 3);
            return coordinateTransformation;
        }

        /**
         * Save last frame to file
         *
         * \param path full path of file with extension
         * \return bool whether saving proceed successful
         */
        bool saveLastFrame(const std::string& path);

        /**
         * get PhoXi Api Version
         *
         * \return PhoXi Api Version
         */

        std::string getApiVersion();

#endif
    protected:
        pho::api::PPhoXi scanner;
        pho::api::PhoXiFactory phoXiFactory;

    private:
        int last_frame_id = -1;
    };
}


#endif //PROJECT_PHOXIINTERFACE_H
