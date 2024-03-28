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
 * A class made specifically to test connections for SG Devices in the background.
 */


#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include "Platform.hpp"

namespace SGConnect
{
    class Connection;
    class ConnectionTester;
    class IdResponse;
    class PortInfo;
}// namespace SGConnect

class SGConnect::ConnectionTester
{
private:
    struct Impl;
    std::unique_ptr<Impl> Pimpl;

public:
    ConnectionTester();

    /// <summary>  </summary>
    /// <param name="toTest"></param>
    /// <returns></returns>
    ConnectionTester(const std::shared_ptr<Connection>& connectionToTest,
                     int32_t maximumIdAttempts = 1, int32_t maximumConnectAttempts = -1);

    virtual ~ConnectionTester();

public:
    SG_NODISCARD bool HandShakeComplete() const;

    SG_NODISCARD bool ValidDevice() const;

    SG_NODISCARD bool KeepTesting() const;

    SG_NODISCARD PortInfo GetPortInfo() const;

    SG_NODISCARD std::string GetAddress() const;

    SG_NODISCARD const std::string& GetConstants() const;

    SG_NODISCARD const std::string& GetHapticChannels() const;

    SG_NODISCARD const IdResponse& GetId() const;

    SG_NODISCARD bool IsConnected() const;

    void StopTesting();

    bool GetConnection(std::shared_ptr<Connection>& out_connection) const;

    /// <summary> Sets the release of Idle connections to true / false; </summary>
    /// <param name="value"></param>
    void SetReleaseIdle(bool value);
};