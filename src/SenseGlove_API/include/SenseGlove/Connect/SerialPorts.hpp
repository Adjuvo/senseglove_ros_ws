/**
 * @file
 *
 * @author  Max Lammers <max@senseglove.com>
 * @author  Mamadou Babaei <mamadou@senseglove.com>
 *
 * @section LICENSE
 *
 * Copyright (c) 2020 - 2024 SenseGlove
 *
 * @section DESCRIPTION
 *
 * Used to retrieve serial port information on this PC, before actually
 * connecting.
 * Placed in a separate class because so that this is the only one accessing
 * the Serial library.
 */


#pragma once

#include <string>
#include <vector>

#include "Platform.hpp"
#include "PortInfo.hpp"

namespace SGConnect
{
    class SerialPorts;
}// namespace SGConnect

namespace serial
{
    struct PortInfo;
}

/// <summary> Utility Class to retrieve serial ports and their relevant information. </summary>
class SGConnect::SerialPorts
{
private:
    struct Impl;

public:
    SerialPorts() = delete;
    virtual ~SerialPorts() = delete;

public:
    /// <summary> Retrieve detailed port into of everything connected to this system </summary>
    SG_NODISCARD static std::vector<std::string> GetPorts();

    /// <summary> Get ports that represent Sense Glove connections, Optional Bluetooth filter </summary>
    SG_NODISCARD static std::vector<PortInfo> GetSGPorts(bool bWithBluetooth);

    /// <summary> Get all bluetooth ports that represent Sense Glove connections, Optional Bluetooth filter </summary>
    SG_NODISCARD static std::vector<PortInfo> GetBluetoothPorts(bool bWithWired);

    /// <summary> Returns true if a hardware_id indicated that is a Bluetooth connection </summary>
    SG_NODISCARD static bool IsBluetoothPort(const serial::PortInfo& portInfo);

    /// <summary> Returns true if this Bluetooth port can be opened without locking the application. </summary>
    SG_NODISCARD static bool IsValidBluetoothPort(const serial::PortInfo& portInfo);

    /// <summary> Returns true if this hardware id belongs to a Sense Glove Device. </summary>
    SG_NODISCARD static bool IsSenseGlovePort(const serial::PortInfo& portInfo);

    /// <summary> Returns true if this hardware id belongs to a Sense Glove Device. </summary>
    SG_NODISCARD static bool AddToWhiteList(const std::string& pidVidContains);

    /// <summary> Returns true if this hardware id belongs to a Sense Glove Device. </summary>
    SG_NODISCARD static bool RemoveFromWhiteList(const std::string& pidVidContains);
};