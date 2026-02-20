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
 * Floating point related utility macros and methods.
 */


#pragma once

 /*******************************************************************************
 * Include directives
 *******************************************************************************/

#include <atomic>

#include <SenseGlove/Common/MathDefines.hpp>
#include <SenseGlove/Common/Platform.hpp>

 /*******************************************************************************
 * Macro definitions
 *******************************************************************************/

#define SG_SMALL_NUMBER (1.e-8f)
#define SG_KINDA_SMALL_NUMBER (1.e-4f)

 /*******************************************************************************
 * Atomics' macro definitions
 *******************************************************************************/

#if SG_PLATFORM_LINUX && !SG_PLATFORM_ANDROID
#if defined(_LIBCPP_VERSION)
#if _LIBCPP_VERSION >= 180100
#define SG_HAS_ATOMIC_FLOAT_FETCH_ADD 1
#else   /* _LIBCPP_VERSION >= 180100 */
#define SG_HAS_ATOMIC_FLOAT_FETCH_ADD 0
#endif  /* _LIBCPP_VERSION >= 180100 */
#else   /* defined(_LIBCPP_VERSION) */
// Fallback: assume std::atomic<float>::fetch_add is supported. If not, the
// build will fail here. This is intentional so that when porting to a new C++
// standard library implementaion, we must explicitly check and set
// SG_HAS_ATOMIC_FLOAT_FETCH_ADD for that targeted standard library
// implementaion.
#define SG_HAS_ATOMIC_FLOAT_FETCH_ADD 1
#endif  /* defined(_LIBCPP_VERSION) */
#elif SG_PLATFORM_WINDOWS
#define SG_HAS_ATOMIC_FLOAT_FETCH_ADD 1
#elif SG_PLATFORM_ANDROID
#define SG_HAS_ATOMIC_FLOAT_FETCH_ADD 0
#else   /* SG_PLATFORM_LINUX && !SG_PLATFORM_ANDROID */
// Fallback: assume std::atomic<float>::fetch_add is supported. If not, the
// build will fail here. This is intentional so that when porting to a new
// platform, we must explicitly check and set SG_HAS_ATOMIC_FLOAT_FETCH_ADD for
// that platform.
#define SG_HAS_ATOMIC_FLOAT_FETCH_ADD 1
#endif  /* SG_PLATFORM_LINUX && !SG_PLATFORM_ANDROID */

 /*******************************************************************************
 * FloatUtils implementation
 *******************************************************************************/

namespace SGCommon
{
    class SGCOMMON_API FloatUtils;
}// namespace SGCommon

class SGCOMMON_API SGCommon::FloatUtils
{
public:
    static SG_FORCEINLINE bool IsNearlyEqual(const float a, const float b, const float errorTolerance = SG_SMALL_NUMBER)
    {
        return std::abs(a - b) <= errorTolerance;
    }

    static SG_FORCEINLINE bool IsNearlyZero(const float value, const float errorTolerance = SG_SMALL_NUMBER)
    {
        return std::abs(value) <= errorTolerance;
    }

    static SG_FORCEINLINE float AtomicFetchAdd(
        std::atomic<float>& out_object, const float arg,
        const std::memory_order order = std::memory_order_seq_cst)
    {
#if SG_HAS_ATOMIC_FLOAT_FETCH_ADD
        return out_object.fetch_add(arg, order);
#else   /* SG_HAS_ATOMIC_FLOAT_FETCH_ADD */
        // Portable CAS-based fallback
        float expected = out_object.load(order);
        while (!out_object.compare_exchange_weak(expected, expected + arg, order,
            std::memory_order_relaxed)) {
            /* Old is updated automatically if CAS fails. */
        }
        return expected;
#endif  /* SG_HAS_ATOMIC_FLOAT_FETCH_ADD */
    }

public:
    FloatUtils() = delete;
    virtual ~FloatUtils() = delete;
};