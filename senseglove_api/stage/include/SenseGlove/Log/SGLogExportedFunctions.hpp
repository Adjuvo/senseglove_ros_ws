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
 * SGLog C-compatible FFI exported functions.
 */


#pragma once

/**
 * Defines a C++ like static_assert.
 */
#if ! __cplusplus
#include <assert.h>
#endif  /* ! __cplusplus */

/**
 * Contains __bool_true_false_are_defined macro that can be used in order to
 * check whether boolean type is supported by the compiler or not.
 */
#include <stdbool.h>

/**
 * Check at compile time whether the C compiler has support for bool, true,
 * and false macros.
 */
static_assert(__bool_true_false_are_defined, "Error: bool, true, and false are not defined!");

/**
 * Defines DLL EXPORT/IMPORT macros.
 */
#include <SenseGlove/Common/Platform.hpp>

/**
 * If C++ is used, switch to C-mode in order to prevent the C++ name mangling of
 * method names.
 */
#if __cplusplus
extern "C" {
#endif  /* __cplusplus */

/*******************************************************************************
 * SGLogImpl.hpp
 ******************************************************************************/

SGLOG_API bool SGLog_IsInitialized();

SGLOG_API bool SGLog_Initialize(void* out_error);

SGLOG_API bool SGLog_Terminate(void* out_error);

#if __cplusplus
}
#endif  /* __cplusplus */
