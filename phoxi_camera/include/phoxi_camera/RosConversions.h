//
// Created by controller on 2/12/19.
//

#ifndef PROJECT_ROSCONVERSIONS_H
#define PROJECT_ROSCONVERSIONS_H

#include <phoxi_msgs/msg/device_information.hpp>
#include <phoxi_camera/PhoXiDeviceInformation.h>

void phoXiDeviceInforamtionToRosMsg(const phoxi_camera::PhoXiDeviceInformation& phoXiDeviceInformation,
                                    phoxi_msgs::msg::DeviceInformation& deviceInformationMsg) {

    deviceInformationMsg.name = phoXiDeviceInformation.name;
    deviceInformationMsg.type.type = (int)phoXiDeviceInformation.type;
    deviceInformationMsg.hwidentification = phoXiDeviceInformation.hwidentification;
    deviceInformationMsg.ipaddress = phoXiDeviceInformation.ipaddress;
    deviceInformationMsg.status.status = phoXiDeviceInformation.status;
    deviceInformationMsg.firmwareversion = phoXiDeviceInformation.firmwareversion;
}

void phoXiDeviceInforamtionToRosMsg(const std::vector<phoxi_camera::PhoXiDeviceInformation>& phoXiDeviceInformation,
                                    std::vector<phoxi_msgs::msg::DeviceInformation>& deviceInformationMsg) {
    deviceInformationMsg.clear();
    deviceInformationMsg.resize(phoXiDeviceInformation.size());
    for (const auto& device : phoXiDeviceInformation) {
        phoXiDeviceInforamtionToRosMsg(device, deviceInformationMsg.back());
    }
    for (int i = 0; i < phoXiDeviceInformation.size(); ++i) {
        phoXiDeviceInforamtionToRosMsg(phoXiDeviceInformation[i], deviceInformationMsg[i]);
    }
}

#endif //PROJECT_ROSCONVERSIONS_H
