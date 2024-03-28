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
 * Diagnostics class used to send messages to developers from the Sense Glove
 * API.
 */

//#define NO_TIMESTAMPS //if defined, we do not add TimeStamps to our Debug Logs. (Saves on calculation speed, I'm sure).

#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "Platform.hpp"

namespace SGConnect
{
    /// <summary> Used to indicate what 'level' of debugging certain messages are meant for. If the DebugLevel of our
    /// debugger is 'higher' than an incoming message, it is not actually logged. </summary>
    enum class SGCONNECT_API EDebugLevel : uint8_t
    {
        /// <summary> No debug messages will be sent. </summary>
        Disabled = 0,

        /// <summary> Send only messages if errors are caught. </summary>
        ErrorsOnly,

        /// <summary> Report on SGConnect's Init / Dispose methods </summary>
        Initialization,

        /// <summary> Messages about new connections or device reconnection. </summary>
        NewDevices,

        /// <summary> Messages about connection/disconnection of connections that may not be SGDevices. </summary>
        AllConnections,

        /// <summary> Messages about the entering and exiting of (read)threads. </summary>
        Threads,

        /// <summary> Messages what is sent to any device. </summary>
        Commands,

        /// <summary> Messages what is sent to any device. </summary>
        HapticsSent,

        /// <summary> Messages what is received from any device. </summary>
        DataReceived,

        /// <summary> For those we have no place for yet. </summary>
        All,
    };

    class Debugger;
}// namespace SGConnect

class SGConnect::Debugger
{

public:
    static void SetDebugQueue(bool bActive);

    static void ClearQueue();

    static void GetMessages(std::vector<std::string>& out_messages);

    SGCONNECT_API static bool KeepsQueue();

    static void LogExternal(const std::string& msg);

    /// <summary> The Default debug level for any application </summary>
    SGCONNECT_API static EDebugLevel GetDefaultLevel();// defined separately so we can say debugLevel = default on init

    /// <summary> The current debug level of the Debugger, can be changed through code. </summary>
    SGCONNECT_API static EDebugLevel GetDebugLevel();

    SGCONNECT_API static void SetDebugLevel(EDebugLevel level);

    static void SetDebugLevel(int32_t newLevel);

    /// <summary> Print a single message with an end-line to the console. </summary>
    static void Log(const std::string& msg, EDebugLevel level, bool bLogExternal = false);

    /// <summary> Set the current system time as the startup time... </summary>
    static void LogStartTime();
    /// <summary> Returns the local time since startup for this process. Useful only for debugging for now... </summary>
    /// <returns></returns>
    static float ElapsedTime_ms();

private:
    struct Impl;

public:
    Debugger() = delete;
    virtual ~Debugger() = delete;
};