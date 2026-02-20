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
 * SenseGlove logging utility class and macros.
 */


#pragma once

/*******************************************************************************
* Include directives
*******************************************************************************/

#include <string>

#include <SenseGlove/Common/Platform.hpp>

#include "SGLogFfiTypes.hpp"
#include "SGLogExportedFunctions.hpp"

/*******************************************************************************
* Determine whether internal logging is enabled or not.
*******************************************************************************/

#if defined( SENSEGLOVE_DEBUG_BUILD ) || ! defined ( SG_LOG_EXTERNAL_ONLY )
#define SG_INTERNAL_LOGGING_ENABLED  1
#else   /* defined( SENSEGLOVE_DEBUG_BUILD ) || ! defined ( SG_LOG_EXTERNAL_ONLY ) */
#define SG_INTERNAL_LOGGING_ENABLED  0
#endif  /* defined( SENSEGLOVE_DEBUG_BUILD ) || ! defined ( SG_LOG_EXTERNAL_ONLY ) */

/*******************************************************************************
* SenseGlove logging macros equivalent to emilk/loguru macros
*******************************************************************************/

#define SG_VLOG_F(verbosity, ...)                                              \
    ((verbosity) > loguru::current_verbosity_cutoff())                         \
        ? (void)0                                                              \
        : loguru::log(verbosity, __FILE__, __LINE__, __VA_ARGS__)

// LOG_F(INFO, "Foo: %d", some_number);
#define SG_LOG_F(verbosity_name, ...)                                          \
    SG_VLOG_F(loguru::Verbosity_##verbosity_name, __VA_ARGS__)

#define SG_VLOG_IF_F(verbosity, cond, ...)                                     \
    ((verbosity) > loguru::current_verbosity_cutoff() || (cond) == false)      \
        ? (void)0                                                              \
        : loguru::log(verbosity, __FILE__, __LINE__, __VA_ARGS__)

#define SG_LOG_IF_F(verbosity_name, cond, ...)                                 \
    SG_VLOG_IF_F(loguru::Verbosity_##verbosity_name, cond, __VA_ARGS__)

#define SG_VLOG_SCOPE_F(verbosity, ...)                                        \
    loguru::LogScopeRAII LOGURU_ANONYMOUS_VARIABLE(error_context_RAII_) =      \
        ((verbosity) > loguru::current_verbosity_cutoff())                     \
            ? loguru::LogScopeRAII()                                           \
            : loguru::LogScopeRAII(verbosity, __FILE__, __LINE__, __VA_ARGS__)

// Raw logging - no preamble, no indentation. Slightly faster than full logging.
#define SG_RAW_VLOG_F(verbosity, ...)                                          \
    ((verbosity) > loguru::current_verbosity_cutoff())                         \
        ? (void)0                                                              \
        : loguru::raw_log(verbosity, __FILE__, __LINE__, __VA_ARGS__)

#define SG_RAW_LOG_F(verbosity_name, ...)                                      \
    SG_RAW_VLOG_F(loguru::Verbosity_##verbosity_name, __VA_ARGS__)

// Use to book-end a scope. Affects logging on all threads.
#define SG_LOG_SCOPE_F(verbosity_name, ...)                                    \
    SG_VLOG_SCOPE_F(loguru::Verbosity_##verbosity_name, __VA_ARGS__)

#define SG_LOG_SCOPE_FUNCTION(verbosity_name)                                  \
    SG_LOG_SCOPE_F(verbosity_name, __func__)

// -----------------------------------------------
// ABORT_F macro. Usage:  ABORT_F("Cause of error: %s", error_str);

// Message is optional
#define SG_ABORT_F(...)                                                        \
    loguru::log_and_abort(0, "ABORT: ", __FILE__, __LINE__, __VA_ARGS__)

// --------------------------------------------------------------------
// CHECK_F macros:

#define SG_CHECK_WITH_INFO_F(test, info, ...)                                  \
    LOGURU_PREDICT_TRUE((test) == true)                                        \
        ? (void)0                                                              \
        : loguru::log_and_abort(0, "CHECK FAILED:  " info "  ", __FILE__,      \
                                __LINE__, ##__VA_ARGS__)

/* Checked at runtime too. Will print error, then call fatal_handler (if any),
   then 'abort'. Note that the test must be boolean. CHECK_F(ptr); will not
   compile, but CHECK_F(ptr != nullptr); will. */
#define SG_CHECK_F(test, ...) SG_CHECK_WITH_INFO_F(test, #test, ##__VA_ARGS__)

#define SG_CHECK_NOTNULL_F(x, ...)                                             \
    SG_CHECK_WITH_INFO_F((x) != nullptr, #x " != nullptr", ##__VA_ARGS__)

#define SG_CHECK_OP_F(expr_left, expr_right, op, ...)                                 \
    do {                                                                              \
        auto val_left = expr_left;                                                    \
        auto val_right = expr_right;                                                  \
        if (!LOGURU_PREDICT_TRUE(val_left op val_right)) {                            \
            auto str_left = loguru::format_value(val_left);                           \
            auto str_right = loguru::format_value(val_right);                         \
            auto fail_info = loguru::textprintf(                                      \
                "CHECK FAILED:  " LOGURU_FMT(s) " " LOGURU_FMT(s) " " LOGURU_FMT(     \
                    s) "  (" LOGURU_FMT(s) " " LOGURU_FMT(s) " " LOGURU_FMT(s) ")  ", \
                #expr_left, #op, #expr_right, str_left.c_str(), #op,                  \
                str_right.c_str());                                                   \
            auto user_msg = loguru::textprintf(__VA_ARGS__);                          \
            loguru::log_and_abort(0, fail_info.c_str(), __FILE__, __LINE__,           \
                                  LOGURU_FMT(s), user_msg.c_str());                   \
        }                                                                             \
    } while (false)

#if LOGURU_DEBUG_LOGGING
// Debug logging enabled:
#define SG_DLOG_F(verbosity_name, ...) SG_LOG_F(verbosity_name, __VA_ARGS__)
#define SG_DVLOG_F(verbosity, ...) SG_VLOG_F(verbosity, __VA_ARGS__)
#define SG_DLOG_IF_F(verbosity_name, ...)                                      \
    SG_LOG_IF_F(verbosity_name, __VA_ARGS__)
#define SG_DVLOG_IF_F(verbosity, ...) SG_VLOG_IF_F(verbosity, __VA_ARGS__)
#define SG_DRAW_LOG_F(verbosity_name, ...)                                     \
    SG_RAW_LOG_F(verbosity_name, __VA_ARGS__)
#define SG_DRAW_VLOG_F(verbosity, ...) SG_RAW_VLOG_F(verbosity, __VA_ARGS__)
#else   /* LOGURU_DEBUG_LOGGING */
// Debug logging disabled:
#define SG_DLOG_F(verbosity_name, ...)
#define SG_DVLOG_F(verbosity, ...)
#define SG_DLOG_IF_F(verbosity_name, ...)
#define SG_DVLOG_IF_F(verbosity, ...)
#define SG_DRAW_LOG_F(verbosity_name, ...)
#define SG_DRAW_VLOG_F(verbosity, ...)
#endif  /* LOGURU_DEBUG_LOGGING */

#define SG_CHECK_EQ_F(a, b, ...) SG_CHECK_OP_F(a, b, ==, ##__VA_ARGS__)
#define SG_CHECK_NE_F(a, b, ...) SG_CHECK_OP_F(a, b, !=, ##__VA_ARGS__)
#define SG_CHECK_LT_F(a, b, ...) SG_CHECK_OP_F(a, b, <, ##__VA_ARGS__)
#define SG_CHECK_GT_F(a, b, ...) SG_CHECK_OP_F(a, b, >, ##__VA_ARGS__)
#define SG_CHECK_LE_F(a, b, ...) SG_CHECK_OP_F(a, b, <=, ##__VA_ARGS__)
#define SG_CHECK_GE_F(a, b, ...) SG_CHECK_OP_F(a, b, >=, ##__VA_ARGS__)

#if LOGURU_DEBUG_CHECKS
// Debug checks enabled:
#define SG_DCHECK_F(test, ...) SG_CHECK_F(test, ##__VA_ARGS__)
#define SG_DCHECK_NOTNULL_F(x, ...) SG_CHECK_NOTNULL_F(x, ##__VA_ARGS__)
#define SG_DCHECK_EQ_F(a, b, ...) SG_CHECK_EQ_F(a, b, ##__VA_ARGS__)
#define SG_DCHECK_NE_F(a, b, ...) SG_CHECK_NE_F(a, b, ##__VA_ARGS__)
#define SG_DCHECK_LT_F(a, b, ...) SG_CHECK_LT_F(a, b, ##__VA_ARGS__)
#define SG_DCHECK_LE_F(a, b, ...) SG_CHECK_LE_F(a, b, ##__VA_ARGS__)
#define SG_DCHECK_GT_F(a, b, ...) SG_CHECK_GT_F(a, b, ##__VA_ARGS__)
#define SG_DCHECK_GE_F(a, b, ...) SG_CHECK_GE_F(a, b, ##__VA_ARGS__)
#else   /* LOGURU_DEBUG_CHECKS */
// Debug checks disabled:
#define SG_DCHECK_F(test, ...)
#define SG_DCHECK_NOTNULL_F(x, ...)
#define SG_DCHECK_EQ_F(a, b, ...)
#define SG_DCHECK_NE_F(a, b, ...)
#define SG_DCHECK_LT_F(a, b, ...)
#define SG_DCHECK_LE_F(a, b, ...)
#define SG_DCHECK_GT_F(a, b, ...)
#define SG_DCHECK_GE_F(a, b, ...)
#endif  /* LOGURU_DEBUG_CHECKS */

/*******************************************************************************
* SenseGlove-specific logging macros
*******************************************************************************/

#if SG_INTERNAL_LOGGING_ENABLED

#define SG_LOG(verbosity_name, ...)                                            \
    SG_VLOG_F(loguru::Verbosity_##verbosity_name, __VA_ARGS__)

#define SG_LOG_EXTERNAL(verbosity_name, ...)                                   \
    do {                                                                       \
        SG_VLOG_F(loguru::Verbosity_##verbosity_name, __VA_ARGS__);            \
        Debugger::LogExternal(fmt::format(__VA_ARGS__));                       \
    } while (0)

#else   /* SG_INTERNAL_LOGGING_ENABLED */

#define SG_LOG(verbosity_name, ...)

#define SG_LOG_EXTERNAL(verbosity_name, ...)                                   \
        Debugger::LogExternal(fmt::format(__VA_ARGS__));

#endif  /* SG_INTERNAL_LOGGING_ENABLED */

/*******************************************************************************
* SGLog implementation
*******************************************************************************/

namespace SGLog
{
    SG_FORCEINLINE bool IsInitialized()
    {
        const bool bResult = SGLog_IsInitialized();
        return bResult;
    }

    SG_FORCEINLINE bool Initialize(std::string& out_error)
    {
        SGLogFfiType_std_string outErrorContainer;
        const bool bResult = SGLog_Initialize(
            &outErrorContainer);
        out_error = std::move(outErrorContainer.String);
        return bResult;
    }

    SG_FORCEINLINE bool Terminate(std::string& out_error)
    {
        SGLogFfiType_std_string outErrorContainer;
        const bool bResult = SGLog_Terminate(
            &outErrorContainer);
        out_error = std::move(outErrorContainer.String);
        return bResult;
    }
}// namespace SGlog
