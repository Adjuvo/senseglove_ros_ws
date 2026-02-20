/**
 * @file
 *
 * @author  Mamadou Babaei <mamadou@senseglove.com>
 *
 * @section LICENSE
 *
 * Copyright (c) 2020 - 2026 SenseGlove
 *
 * @section DESCRIPTION
 *
 * A custom hybrid spin mutex implementation (a.k.a. adaptive mutex).
 */


#pragma once

#include <memory>

#include <SenseGlove/Common/Platform.hpp>

namespace SGCommon
{
    class SGCOMMON_API HybridMutex;
}// namespace SGCommon

class SGCOMMON_API SGCommon::HybridMutex
{
private:
    struct Impl;
    std::unique_ptr<Impl> Pimpl;

public:
    static HybridMutex& GetInstance();

public:
    /**
     * @brief The default constructor.
     */
    HybridMutex();

    /**
     * @brief The destructor.
     */
    virtual ~HybridMutex();

private:
    /**
     * @brief The copy constructor.
     */
    HybridMutex(const HybridMutex& rhs) = delete;

    /**
     * @brief The move constructor.
     */
    HybridMutex(HybridMutex&& rhs) noexcept = delete;

private:
    /**
     * @brief The copy assignment operator.
     */
    HybridMutex& operator=(const HybridMutex& rhs) = delete;

    /**
     * @brief The move assignment operator.
     */
    HybridMutex& operator=(HybridMutex&& rhs) noexcept = delete;

public:
    bool try_lock();
    void lock();
    void unlock();
};
