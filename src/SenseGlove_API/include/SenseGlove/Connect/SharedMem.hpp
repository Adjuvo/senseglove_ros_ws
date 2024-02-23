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
 * Utility Class used to write and read to/from IPC.
 * a.k.a. The (only) one with access to Boost::Interprocess.
 */


#pragma once

#include <cstdint>
#include <string>

#include "Platform.hpp"

namespace SGConnect
{
    class SharedMem;
}

/// <summary> Utility Class used to write and read to/from IPC </summary>
class SGConnect::SharedMem
{
public:
    /// <summary> Shared Memory block containing number of devices and deviceInfo from each device </summary>
    SG_NODISCARD static const std::string& GetDevices();

    /// <summary> Shared Memory block containing haptic commands from each device </summary>
    SG_NODISCARD static const std::string& GetHaptics();

    /// <summary>  Shared Memory block containing sensor data from each device </summary>
    SG_NODISCARD static const std::string& GetData();

    /// <summary> Shared Memory block containing commands to be sent to the devices </summary>
    SG_NODISCARD static const std::string& GetInCommands();

    /// <summary> Shared Memory block containing commands/responses from the devices </summary>
    SG_NODISCARD static const std::string& GetOutCommands();

    /// <summary> Address in devices where the number of devices is stored. </summary>
    SG_NODISCARD static const std::string& GetNumDevices();

    /// <summary> Address in devices where the scanner's processing id is stored. </summary>
    SG_NODISCARD static const std::string& GetScanActive();

    /// <summary> Data block containing Connection states & pared devices </summary>
    SG_NODISCARD static const std::string& GetConnectionsDebug();

    /// <summary> Address in connectionsDebug that shows connections, testers & states. </summary>
    SG_NODISCARD static const std::string& GetActiveConnections();

    /// <summary> Delimiter to split data in Shared Memory </summary>
    SG_NODISCARD static char GetListDelimiter();

    /// <summary> Standard size of Shared Memory. It works with this but could probably use a lower value. </summary>
    SG_NODISCARD static uint32_t GetStandardSize();

    SG_NODISCARD static const std::string& GetReleaseIdleConnections();// = "idleRelease";

    SG_NODISCARD static const std::string& GetHapticUpdate();//"hUpd";

public:
    /// <summary> Write a string to Shared Memory at block-addess, but also creates one if it doesn't yet exist.
    /// Returns true if successful. </summary>
    static bool WriteTo(const std::string& block, const std::string& address, const std::string& value,
                        uint32_t size = GetStandardSize());

    /// <summary> Write a string to Shared Memory at block-addess, but don't create on if it doesn't exist. Returns
    /// true if successful. </summary>
    static bool WriteTo_NotCreate(const std::string& block, const std::string& address, const std::string& value);

    /// <summary> Retrieve value from Shared Memory at block-address. Returns true if successful. </summary>
    static bool ReadFrom(const std::string& block, const std::string& address, std::string& out_value);

    /// <summary> Dispose of a shared memory block so it no longer takes up space on the PC. Returns true if successful.
    /// </summary>
    static bool Dispose(const std::string& block);

    static bool TimeSinceLastUpdate(long& time);

    static bool UpdateTime();

public:
    SharedMem() = delete;
    virtual ~SharedMem() = delete;
};
