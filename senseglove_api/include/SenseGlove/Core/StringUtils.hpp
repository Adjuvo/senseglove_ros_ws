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
 * String conversion, parsing and other utility stuff. Much more prominent in
 * C++.
 */


#pragma once

#include "Platform.hpp"

#include <memory>
#if SG_HAS_STD_FORMAT
#include <format>
#endif  /* SG_HAS_STD_FORMAT */
#include <stdexcept>
#include <string>
#include <vector>

#include "Fingers.hpp"

namespace SGCore
{
    namespace Util
    {
        class SGCORE_API StringUtils;
    }// namespace Util
}// namespace SGCore

class SGCORE_API SGCore::Util::StringUtils
{
public:
    /// <summary> Split a std::string into a vector of std::strings by a delimiter </summary>
    static std::vector<std::string> Split(const std::string& str, char delimiter);

    static std::string QuickSplit(const std::string& input, char delimiter, int32_t getIndex);

    /// <summary> Convert a string into an integer value. </summary>
    static int32_t ToInt(const std::string& str, int32_t fallback = 0);

    /// <summary> Convert a string into a decimal value. </summary>
    static float ToFloat(const std::string& str, float fallback = 0.0f);

    /// <summary> Convert a finger into a string notation. </summary>
    static std::string ToString(SGCore::EFinger finger);

    /// <summary> Replace all occurrences of a character with another one. </summary>
    static std::string ReplaceChars(const std::string& str, char from, char to);

    /// <summary> Convert a value in radians into a rounded integer, as a string. e.g. 4.999999999999999 -> 5.
    /// </summary>
    static std::string RoundedDegrees(float valueInRadians);

    static bool Contains(const std::string& haystack, const std::string& needle);

    /// <summary> Print the different sizes within a 2D array </summary>
    /// <typeparam name="T"></typeparam>
    /// <param name="vect"></param>
    /// <returns></returns>
    template<typename T>
    static std::string PrintSize(const std::vector<std::vector<T>>& vect)
    {
        const std::size_t size = vect.size();
        std::string res("(" + std::to_string(size) + ")[");
        for (std::size_t i = 0; i < size; ++i) {
            res += std::to_string(vect[i].size());
            if (i < size - 1) {
                res += ", ";
            }
        }
        return res + std::string("]");
    }

#if SG_PLATFORM_WINDOWS
    static std::wstring Utf8StringToWideString(const std::string& utf8String);
    static std::string WideStringToUtf8String(const std::wstring& wideString);
#endif /* SG_PLATFORM_WINDOWS */

    // NOTE:
    // SG_HAS_STD_FORMAT is a mandatory workaround for Android NDK r25b and Epic
    // Native or Cross Toolchains v22, which lack full <format> support.
    // Once support for these toolchains is dropped, SG_HAS_STD_FORMAT and
    // StringUtils::FormatString() can be removed in favor of direct usage of
    // std::format().
#if SG_HAS_STD_FORMAT
    template<typename... Args>
    SG_FORCEINLINE static std::string FormatString(std::format_string<Args...> fmt, Args&&... args)
    {
        return std::format(fmt, std::forward<Args>(args)...);
    }
#else   /* SG_HAS_STD_FORMAT */
    template<typename... Args>
    static std::string FormatString(const std::string& fmt, const Args&... args)
    {
        int32_t size = std::snprintf(nullptr, 0, fmt.c_str(), args...) + 1;// extra space for '\0'
        if (size <= 0) {
            throw std::runtime_error("Error during formatting.");
        }
        std::unique_ptr<char[]> buffer{std::make_unique<char[]>(size)};
        std::snprintf(buffer.get(), size, fmt.c_str(), args...);
        return std::string{buffer.get(), buffer.get() + size - 1};// we don't want the '\0' inside
    }
#endif   /* SG_HAS_STD_FORMAT */

private:
    struct Impl;

public:
    StringUtils() = delete;
    virtual ~StringUtils() = delete;
};