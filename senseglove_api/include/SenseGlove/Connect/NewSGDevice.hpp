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
#include "PortInfo.hpp"
#include "TransmissionMode.hpp"

namespace SGConnect
{
    class IdResponse;

    class NewSGDevice;
}// namespace SGConnect

class SGConnect::NewSGDevice
{
public:
    static std::string ToString(ETransmissionMode transmissionMode);

private:
    struct Impl;
    std::unique_ptr<Impl> Pimpl;

public:
    NewSGDevice();

    NewSGDevice(const IdResponse& id, const std::string& constants, int32_t deviceIndex,
                ETransmissionMode transmissionMode = ETransmissionMode::Async);

    virtual ~NewSGDevice();

public:
    SG_NODISCARD bool CanUpdate() const;

    /// <summary> Stops this SG Device's data thread </summary>
    void StopUpdating();

    /// <summary> Update this device's index in the DeviceScanner list, and generate a new IpcAddress. </summary>
    void SetDeviceIndex(int32_t deviceIndex);

    /// <summary> Retrieve the IPC Address of this SG Device </summary>
    SG_NODISCARD const std::string& GetIpcAddress() const;

    /// <summary> Retrieve the IPC string for deviceInfo. </summary>
    SG_NODISCARD std::string GetDeviceString() const;

    SG_NODISCARD const IdResponse& GetDeviceInfo() const;

    void Reconnect(int32_t newConnectionIndex);

    SG_NODISCARD bool SameAddress(const PortInfo& portInfo);

    SG_NODISCARD bool IsConnected() const;
};