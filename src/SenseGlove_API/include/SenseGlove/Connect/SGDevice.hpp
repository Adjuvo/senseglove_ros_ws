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
 * Class that contains data of a Sense Glove device that has been detected on
 * the system.
 * Has an interchangeable communications component, which it uses to exchange
 * data based on its transmit mode.
 */


#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include "Platform.hpp"
#include "PortInfo.hpp" /* Do not forward declare this, or else we'll encounter LINK2019 build error on MSVC */
#include "TransmissionMode.hpp"

namespace SGConnect
{
    class Connection;
    class IdResponse;
    class SGDevice;
}// namespace SGConnect

class SGConnect::SGDevice
{
public:
    /// <summary> Extract firmware main/sub versions from a constants string </summary>
    static void ExtractFirmware(const std::string& constantsString, int32_t& out_mainVersion, int32_t& out_subVersion);

    /// <summary> Determine the communications mode based on this device type and firmware version. </summary>
    static ETransmissionMode CommunicationMode(int32_t deviceType,
                                               int32_t mainFirmwareVersion, int32_t subFirmwareVersion);

    /// <summary> Convert a TransmissionMode into a string for debugging purposes. </summary>
    static std::string ToString(ETransmissionMode transmissionMode);

private:
    struct Impl;
    std::unique_ptr<Impl> Pimpl;

public:
    /// <summary> The default constructor. </summary>
    SGDevice();

    /// <summary> Create a new SGDevice based on a connection and its responses. </summary>
    SGDevice(std::shared_ptr<Connection> startComm, const IdResponse& idInfo, const std::string& constantsResponse,
             std::string& hapticsResponse, int32_t deviceIndex);

    /// <summary> Destructor cleans up any remaining resources. </summary>
    virtual ~SGDevice();

public:
    SG_NODISCARD const IdResponse& GetDeviceInfo() const;

    /// <summary> Update this device's index in the DeviceScanner list, and generate a new IpcAddress. </summary>
    void SetDeviceIndex(int32_t deviceIndex);

    /// <summary> Check if this device's connection has the same address a port on the system. </summary>
    SG_NODISCARD bool SameAddress(const PortInfo& port) const;

    /// <summary> Check if this device's connection is still active. </summary>
    SG_NODISCARD bool IsConnected() const;

    /// <summary> End any existing connection and link this device to a new connection. </summary>
    void ReConnect(std::shared_ptr<Connection> newComm);

    /// <summary> Update the Packets/Per/Second of this SGDevice. </summary>
    void UpdatePps(double deltaTime);

    /// <summary> Retrieve the IPC Address of this SG Device </summary>
    SG_NODISCARD const std::string& GetIpcAddress() const;

    /// <summary> Retrieve the IPC string for deviceInfo. </summary>
    SG_NODISCARD std::string GetDeviceString() const;

    void StopUpdating();
};