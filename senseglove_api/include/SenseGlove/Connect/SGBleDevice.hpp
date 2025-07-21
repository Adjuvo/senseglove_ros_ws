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
 */


#pragma once

#include "Platform.hpp"

#include "SGDevice.hpp"


namespace SGConnect
{
    class IdResponse;
    class SGBleDevice;

}// namespace SGConnect


class SGConnect::SGBleDevice : public SGConnect::SGDevice
{

private:
    struct Impl;
    std::unique_ptr<Impl> Pimpl;

public:

    SGBleDevice();
    explicit SGBleDevice(const IdResponse& idInfo, const std::string& constantsResponse,
                         std::string& hapticsResponse, int32_t deviceIndex);

    virtual ~SGBleDevice();

public:

    virtual void StopUpdating() override;

    /// <summary> Retrieve the IPC string for deviceInfo. </summary>
    virtual std::string GetDeviceString() const override;

    [[nodiscard]] virtual bool IsConnected() const override;
};