//
// Created by controller on 2/12/19.
//

#ifndef PROJECT_PHOXICONVERSIONS_H
#define PROJECT_PHOXICONVERSIONS_H

#include <PhoXi.h>
#include <phoxi_camera/PhoXiDeviceInformation.h>

void toPhoXiCameraDeviceInforamtion(const pho::api::PhoXiDeviceInformation& phoXiDeviceInformation,
                                    phoxi_camera::PhoXiDeviceInformation& phoXiCameraDeviceInformation) {
    phoXiCameraDeviceInformation.name = phoXiDeviceInformation.Name;
    phoXiCameraDeviceInformation.type = phoxi_camera::PhoXiDeviceInformation::PhoXiDeviceType(
            (int)phoXiDeviceInformation.Type);
    phoXiCameraDeviceInformation.hwidentification = phoXiDeviceInformation.HWIdentification;
    if (phoXiDeviceInformation.Status.Ready) {
        phoXiCameraDeviceInformation.status =  phoxi_camera::PhoXiDeviceInformation::PhoXiConnectionStatus::READY;
    } else {
        if (phoXiDeviceInformation.Status.Attached) {
            phoXiCameraDeviceInformation.status = phoxi_camera::PhoXiDeviceInformation::PhoXiConnectionStatus::STARTING;
        } else {
            phoXiCameraDeviceInformation.status = phoxi_camera::PhoXiDeviceInformation::PhoXiConnectionStatus::OCCUPIED;
        }
    }

    phoXiCameraDeviceInformation.firmwareversion = phoXiDeviceInformation.FirmwareVersion;
}

void toPhoXiCameraDeviceInforamtion(const std::vector<pho::api::PhoXiDeviceInformation>& phoXiDeviceInformation,
                                    std::vector<phoxi_camera::PhoXiDeviceInformation>& phoXiCameraDeviceInformation) {
    phoXiCameraDeviceInformation.clear();
    phoXiCameraDeviceInformation.resize(phoXiDeviceInformation.size());
    for (int i = 0; i < phoXiDeviceInformation.size(); ++i) {
        toPhoXiCameraDeviceInforamtion(phoXiDeviceInformation[i], phoXiCameraDeviceInformation[i]);
    }
}

#endif //PROJECT_PHOXICONVERSIONS_H
