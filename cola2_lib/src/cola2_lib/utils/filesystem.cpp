/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#include <cola2_lib/utils/filesystem.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <cstdlib>
#include <boost/filesystem.hpp>

bool cola2::utils::isFileAccessible(const std::string& path)
{
  struct stat buf;
  return (stat(path.c_str(), &buf) == 0);
}

bool cola2::utils::isSymlink(const std::string& path)
{
  struct stat buf;
  if (lstat(path.c_str(), &buf) == 0)
  {
    return (S_ISLNK(buf.st_mode));
  }
  return false;
}

std::string cola2::utils::getSymlinkTarget(const std::string& path)
{
  struct stat buf;
  std::string output;
  if (lstat(path.c_str(), &buf) == 0)
  {
    char* linkname = reinterpret_cast<char*>(malloc(buf.st_size + 1));
    const ssize_t res = readlink(path.c_str(), linkname, buf.st_size + 1);
    if (res == -1)
    {
      free(linkname);
      return output;
    }
    linkname[buf.st_size] = '\0';
    output = std::string(linkname);
    free(linkname);
  }
  return output;
}

void cola2::utils::createDirectory(const std::string& path)
{
  boost::filesystem::path boost_path(path);
  if (!boost::filesystem::exists(boost_path))
  {
    boost::filesystem::create_directories(boost_path);
  }
}

void cola2::utils::deleteFile(const std::string& path)
{
  boost::filesystem::remove(path);
}
