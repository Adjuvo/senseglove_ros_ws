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
 * SGLog C-compatible FFI types for interoperability between modules with
 * different RTTI and Exceptions settings.
 */


#pragma once

#include <string>

struct SGLogFfiType_std_string
{
    std::string String;
};
