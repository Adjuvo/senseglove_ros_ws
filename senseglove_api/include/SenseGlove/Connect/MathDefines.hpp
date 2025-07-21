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
 * Contains reusable piece of code to include the proper math defines like
 * Pi / Sin / Sqrt.
 */


#pragma once

#include "Platform.hpp"

#if !defined ( _USE_MATH_DEFINES )
#define _USE_MATH_DEFINES
#endif  /* ! defined ( _USE_MATH_DEFINES ) */

#if SG_PLATFORM_WINDOWS
#include <algorithm>
#include "math.h"
#endif  /*SG_PLATFORM_WINDOWS*/

#include <cmath>

SG_FORCEINLINE float sg_acosf(const float num)
{
#if ! SG_LIBCPP_GNU
    return std::acosf(num);
#else  /* ! SG_LIBCPP_GNU */
    return ::acosf(num);
#endif  /* ! SG_LIBCPP_GNU */
}

SG_FORCEINLINE float sg_atan2f(const float y, const float x)
{
#if ! SG_LIBCPP_GNU
    return std::atan2f(y, x);
#else  /* ! SG_LIBCPP_GNU */
    return ::atan2f(y, x);
#endif  /* ! SG_LIBCPP_GNU */
}

template<class Integer>
SG_FORCEINLINE double sg_fabs(const Integer num)
{
#if ! SG_LIBCPP_GNU
    return std::fabs(num);
#else  /* ! SG_LIBCPP_GNU */
    return ::fabs(num);
#endif  /* ! SG_LIBCPP_GNU */
}