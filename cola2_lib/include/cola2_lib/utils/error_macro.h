/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

/**
 * @file
 * @brief This file contains functions to throw error exceptions.
 * @addtogroup utils
 * @{
 */

#pragma once

#include <iostream>
#include <string>

/**
 * @brief Error macro to throw an exception of the type error_type without error message.
 *
 * @param error_type Some instance of a class that can be thrown.
 */
#define THROW_NO_MESSAGE(error_type)                                                                                   \
  throw error_type("Runtime error in " + std::string(__FILE__) + std::string(" at line ") + std::to_string(__LINE__));

/**
 * @brief Error macro to throw an exception of the type error_type with the message and adding the file and line where
 * the error was risen.
 *
 * @param error_type Some instance of a class that can be thrown.
 * @param message The error message.
 */
#define THROW_MESSAGE(error_type, message)                                                                             \
  throw error_type(std::string(message) + std::string(" in file: ") + std::string(__FILE__) +                          \
                   std::string(" at line: ") + std::to_string(__LINE__));

// Macro that does the macro overloading of the THROW into the two with message and no message.
#define RESOLVE_THROW_MACRO(_1, _2, NAME, ...) NAME

/**
 * @brief Overloaded runtime error macro. Depending on the parameters will end up calling the THROW_MESSAGE or
 * THROW_NO_MESSAGE macros.
 */
#define THROW(...) RESOLVE_THROW_MACRO(__VA_ARGS__, THROW_MESSAGE, THROW_NO_MESSAGE)(__VA_ARGS__)

/** @} */
