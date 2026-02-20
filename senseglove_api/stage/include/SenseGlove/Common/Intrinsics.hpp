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
 * Cross-(architecture/compiler/platform) intrinsics.
 */


#pragma once

#include <SenseGlove/Common/Platform.hpp>

#if SG_COMPILER_MSVC
#include <intrin.h>
#elif SG_COMPILER_CLANG
#if SG_CPU_ANY_X86
#include <emmintrin.h>
#endif  /* SG_CPU_ANY_X86 */
#elif SG_COMPILER_GCC
#if SG_CPU_ANY_X86
#include <xmmintrin.h>
#endif  /* SG_CPU_ANY_X86 */
#endif  /* SG_COMPILER_MSVC */

namespace SGCommon
{
    class SGCOMMON_API Intrinsics;
}// namespace SGCommon

class SGCOMMON_API SGCommon::Intrinsics
{
public:
    static SG_FORCEINLINE void Yield() noexcept
    {
#if SG_CPU_ANY_X86
        _mm_pause();
#elif SG_CPU_ANY_ARM
#if SG_COMPILER_MSVC
        __yield();
#else   /* SG_COMPILER_MSVC */
        __asm__ __volatile__("yield");
#endif  /* SG_COMPILER_MSVC */
#else   /* SG_CPU_ANY_X86 */
        // Fallback: no CPU hint instruction available.
        // Do nothing (burn a cycle) or use a compiler barrier.
        asm volatile("" ::: "memory");
#endif  /* SG_CPU_ANY_X86 */
    }
};
