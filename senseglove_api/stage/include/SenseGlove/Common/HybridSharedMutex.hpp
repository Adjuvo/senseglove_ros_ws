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
 * A custom shared hybrid spin mutex implementation
 * (a.k.a. shared adaptive mutex).
 */


#pragma once

#include <memory>

#include <SenseGlove/Common/Platform.hpp>

namespace SGCommon
{
    class SGCOMMON_API HybridSharedMutex;
}// namespace SGCommon

class SGCOMMON_API SGCommon::HybridSharedMutex
{
private:
    struct Impl;
    std::unique_ptr<Impl> Pimpl;

public:
    static HybridSharedMutex& GetInstance();

public:
    /**
     * @brief The default constructor.
     */
    HybridSharedMutex();

    /**
     * @brief The destructor.
     */
    virtual ~HybridSharedMutex();

private:
    /**
     * @brief The copy constructor.
     */
    HybridSharedMutex(const HybridSharedMutex& rhs) = delete;

    /**
     * @brief The move constructor.
     */
    HybridSharedMutex(HybridSharedMutex&& rhs) noexcept = delete;

private:
    /**
     * @brief The copy assignment operator.
     */
    HybridSharedMutex& operator=(const HybridSharedMutex& rhs) = delete;

    /**
     * @brief The move assignment operator.
     */
    HybridSharedMutex& operator=(HybridSharedMutex&& rhs) noexcept = delete;

public:
    bool try_lock();
    void lock();
    void unlock();

    bool try_lock_shared();
    void lock_shared();
    void unlock_shared();
};
