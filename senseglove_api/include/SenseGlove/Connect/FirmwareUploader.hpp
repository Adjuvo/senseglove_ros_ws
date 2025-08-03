/**
 * @file
 *
 * @author  Max Lammers <max@senseglove.com>
 *
 * @section LICENSE
 *
 * Copyright (c) 2020 - 2025 SenseGlove
 *
 * @section DESCRIPTION
 *
 * Uploads Firmware files to a Specific SGDevice.
 */


#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include "Platform.hpp"
#include "EFWErrorCodes.hpp"

namespace SGConnect
{
    class FirmwareUploader;
    class SGDevice;
    class Nova2BleDevice;
}// namespace SGConnect

class SGConnect::FirmwareUploader
{
private:

    struct Impl;

    std::unique_ptr<Impl> Pimpl;

public:

    FirmwareUploader();

    virtual ~FirmwareUploader();

    /// <summary> Uplaod new firmware to the specified device, with additional parameters: </summary>
    /// <param name="toDevice"></param>
    /// <param name="filePath"></param>
    /// <param name="chunkSize"></param>
    /// <returns></returns>
    EFWErrorCodes UploadFirmware_BTS_to_BTS(std::shared_ptr<SGDevice> toDevice, const std::string& filePath, const int32_t chunkSize);

    EFWErrorCodes UploadFirmware_BTS_to_BLE(std::shared_ptr<SGDevice> toDevice, const std::string& filePath, const int32_t chunkSize);
    void Continue_BTS_to_BLE();

    EFWErrorCodes UploadFirmware_BLE_to_BLE(std::shared_ptr<Nova2BleDevice> toDevice, const std::string& filePath, const int32_t chunkSize);


    int32_t UploadProgress();

    bool IsUploading();

    EFWErrorCodes GetLastResult();
    void SetLastResult(const EFWErrorCodes newCode);

};