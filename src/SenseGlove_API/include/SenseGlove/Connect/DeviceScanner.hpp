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
 * A threading resource that scans the computer for connections and Sense Gloves.
 */


#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "ExitCodes.hpp"
#include "Platform.hpp"
#include "PortInfo.hpp" /* Do not forward declare this, or else we'll encounter LINK2019 build error on MSVC */

namespace SGConnect
{
    class Connection;
    class DeviceScanner;
    class IdResponse;
}// namespace SGConnect

/// <summary> A threading resource that scans the computer for connections and Sense Gloves. </summary>
class SGConnect::DeviceScanner// only one of these should be running at a time.
{
public:
private:
    struct Impl;
    std::unique_ptr<Impl> Pimpl;

public:
    /// <summary> Creates a new instance of a DeviceScanner, which immedeately begins scanning for SG Devices.
    /// </summary>
    DeviceScanner();

    /// <summary> Cleans up any remaining unmanaged resources. </summary>
    ~DeviceScanner();

public:
    /// <summary> Check if the Device Scanner is still alive. </summary>
    static bool IsLive();

    /// <summary> Set the live value of the deviceScanner from an external source. </summary>
    static void SetLive(bool bLive);

    static bool AlreadyDetected(const IdResponse& Id);

    /// <summary> (Re)Connect a Device to the DeviceScanner list. </summary>
    /// <param name="deviceId"></param>
    /// <param name="constants"></param>
    /// <param name="newAddress"></param>
    /// <returns></returns>
    static bool ConnectDevice(const IdResponse& deviceId, const std::string& constants, const std::string& newAddress);

    static bool GetConnection(int32_t connectionIndex, std::shared_ptr<Connection>& out_connection);

    static bool Send(int32_t connectionIndex, const std::string& command, bool bHapticCommand);
    static bool GetLastData(int32_t connectionIndex, std::string& out_data);
    static bool GetLastCommand(int32_t connectionIndex, std::string& out_cmd);
    static bool IsConnected(int32_t connectionIndex);
    static bool Disconnect(int32_t connectionIndex, EExitCode exitCode);
    static bool GetPortInfo(int32_t connectionIndex, PortInfo& out_portInfo);
};