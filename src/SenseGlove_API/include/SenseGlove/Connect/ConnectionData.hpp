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
 * A single piece of connection data with helper functions.
 */


#pragma once

#include <memory>
#include <string>

#include "Connection.hpp"
#include "Platform.hpp"
#include "PortInfo.hpp"

namespace SGConnect
{
    class ConnectionData;
}

class SGConnect::ConnectionData
{
private:
    struct Impl;
    std::unique_ptr<Impl> Pimpl;

private:
    ConnectionData();

public:
    ConnectionData(const std::string& address, EConnectionType connectionType, bool bConnected);

    /**
     * The copy constructor.
     */
    ConnectionData(const ConnectionData& rhs);

    /**
     * The move constructor.
     */
    ConnectionData(ConnectionData&& rhs) noexcept;

    virtual ~ConnectionData();

public:
    /**
     * The copy assignment operator.
     */
    ConnectionData& operator=(const ConnectionData& rhs);

    /**
     * The move assignment operator.
     */
    ConnectionData& operator=(ConnectionData&& rhs) noexcept;

public:
    SG_NODISCARD const std::string& GetAddress() const;
    void SetAddress(const std::string& Address);

    SG_NODISCARD EConnectionType GetConnectionType() const;
    void SetConnectionType(EConnectionType connectionType);

    SG_NODISCARD bool IsConnected() const;
    void SetConnected(bool bConnected);

    SG_NODISCARD int32_t GetLastConnectCode() const;
    void SetLastConnectCode(int32_t connectCode);

    SG_NODISCARD int32_t GetLastExitCode() const;
    void SetLastExitCode(int32_t exitCode);

    SG_NODISCARD int32_t GetLastTestState() const;
    void SetLastTestState(int32_t TestState);

    SG_NODISCARD int32_t GetDeviceType() const;
    void SetDeviceType(int32_t DeviceType);

public:
    SG_NODISCARD bool Matches(const std::string& address, EConnectionType connectionType) const;

    SG_NODISCARD std::string Report() const;
};