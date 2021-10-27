/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

/**
 * @file
 * @brief This file contains functions related to files and directories.
 */

#pragma once

#include <string>

namespace cola2
{
namespace utils
{
/**
 * @addtogroup utils
 * @{
 */

/**
 * @brief This function returns true if the given file is accessible and false otherwise.
 *
 * @param path Path.
 * @return Returns true if the given file is accessible and false otherwise.
 */
bool isFileAccessible(const std::string& path);

/**
 * @brief This function returns true if the given path is a symlink and false otherwise.
 *
 * @param path Path.
 * @return Returns true if the given path is a symlink and false otherwise.
 */
bool isSymlink(const std::string& path);

/**
 * @brief This function returns the target file of a symlink.
 *
 * @param symlink Symlink.
 * @return Target file.
 */
std::string getSymlinkTarget(const std::string& symlink);

/**
 * @brief This function creates a directory.
 *
 * @param directory Directory path.
 */
void createDirectory(const std::string& directory);

/**
 * @brief This function deletes a file.
 *
 * @param target_file Target file.
 */
void deleteFile(const std::string& target_file);

/** @} */
}  // namespace utils
}  // namespace cola2
