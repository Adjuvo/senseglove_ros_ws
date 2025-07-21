/**
 * @file
 *
 * @author  Max Lammers <max@senseglove.com>
 * @author  Mamadou Babaei <mamadou@senseglove.com>
 *
 * @section LICENSE
 *
 * Copyright (c) 2020 - 2025 SenseGlove
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
    [[nodiscard]] bool HandShakeComplete() const;

    [[nodiscard]] bool ValidDevice() const;

    [[nodiscard]] bool KeepTesting() const;

    [[nodiscard]] PortInfo GetPortInfo() const;

    [[nodiscard]] std::string GetAddress() const;

    [[nodiscard]] const std::string& GetConstants() const;

    [[nodiscard]] const std::string& GetHapticChannels() const;

    [[nodiscard]] const IdResponse& GetId() const;

    [[nodiscard]] bool IsConnected() const;

    void StopTesting();

    bool GetConnection(std::shared_ptr<Connection>& out_connection) const;

    /// <summary> Sets the release of Idle connections to true / false; </summary>
    /// <param name="value"></param>
    void SetReleaseIdle(bool value);
};