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
 * Since our BLE library can only be included ONCE or cause issues, I'll just add it here...
 */


#pragma once

#include "Platform.hpp"

#include <memory>
#include <string>
#include <vector>

namespace SGConnect
{
    class SGDevice;
    class BleUtil;
}// namespace SGConnect

namespace SGBLExx
{
    class SgBlePeripheral;
}// namespace SGBLExx


class SGConnect::BleUtil
{

public:

    static bool InitializeBLE();
    static bool TerminateBLE();

    static bool CreateInstance(std::shared_ptr<SGDevice>& device);

    static bool ReadSensorData(int32_t bleIndex, std::string& sensorData);
    
private:
    struct Impl;

private:
    /// <summary> A list of all devices that have been detected via the SenseCom executable. </summary>
    static std::vector<SGBLExx::SgBlePeripheral>& GetBlePeripherals();

public:
    BleUtil() = delete;
    virtual ~BleUtil() = delete;

};

