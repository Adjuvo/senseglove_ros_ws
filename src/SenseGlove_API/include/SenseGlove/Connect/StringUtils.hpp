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
 * String conversion, parsing and other utility stuff. Much more prominent in
 * C++.
 */


#pragma once

#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "ExitCodes.hpp"
#include "Platform.hpp"

namespace SGConnect
{
    class StringUtils;
}

/// <summary> Utility class to convert strings into usable values. </summary>
class SGConnect::StringUtils
{
public:
    /// <summary> Split a std::string into a vector of std::strings by a delimiter </summary>
    static std::vector<std::string> Split(const std::string& str, char delimiter);

    /// <summary> Convert a string into an integer value. </summary>
    static int32_t ToInt(const std::string& str);

    /// <summary> Convert a string into a decimal value. </summary>
    static float ToFloat(const std::string& str);

    static std::string ToString(EExitCode code);

    template<typename... Args>
    static std::string FormatString(const std::string& format, const Args&... args)
    {
        int32_t size = std::snprintf(nullptr, 0, format.c_str(), args...) + 1;// extra space for '\0'
        if (size <= 0) {
            throw std::runtime_error("Error during formatting.");
        }
        std::unique_ptr<char[]> buffer(new char[size]);
        std::snprintf(buffer.get(), size, format.c_str(), args...);
        return std::string{buffer.get(), buffer.get() + size - 1};// we don't want the '\0' inside
    }

public:
    StringUtils() = delete;
    virtual ~StringUtils() = delete;
};