//
// Created by controller on 2/12/19.
//

#ifndef PROJECT_PHOXIDEVICEINFORMATION_H
#define PROJECT_PHOXIDEVICEINFORMATION_H

#include <string>
#include <PhoXi.h>

namespace phoxi_camera {
    class PhoXiInterface;

    class PhoXiDeviceInformation {
    public:
        friend PhoXiInterface;
        enum PhoXiConnectionStatus {
            UNKNOW = 0,
            READY = 1,
            OCCUPIED = 2,
            STARTING = 3
        };
        enum PhoXiDeviceType {
#if PHOXI_SCANNER_ON
            PHOXISCANNER,
#endif
#if PHOXI_CAMERA_ON
            PHOXICAMERA,
#endif
            NOVALUE
        };

        operator std::string() const {
            return hwidentification;
        }

        bool operator==(const PhoXiDeviceInformation& other) {
            return hwidentification == other.hwidentification;
        }

        bool operator==(const std::string& hwidentification) {
            return this->hwidentification == hwidentification;
        }

        std::string name;
        PhoXiDeviceType type;
        std::string hwidentification;
        std::string ipaddress;
        PhoXiConnectionStatus status;
        std::string firmwareversion;
    };
}

#endif //PROJECT_PHOXIDEVICEINFORMATION_H
