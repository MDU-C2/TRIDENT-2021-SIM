/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

/**
 * @file
 * @brief C++ input output base for network and serial communications.
 */

#pragma once

#include <boost/asio.hpp>
#include <vector>

namespace cola2
{
namespace io
{
/**
 * @addtogroup io
 * @{
 */

typedef std::vector<unsigned char> DataBuffer;  //!< Buffer for reading/writing in the io module of Cola2 Lib

/**
 * @brief Base class that provides an interface to manage external connections through serial or ethernet using the same
 * read/write methods.
 */
class IOBase
{
private:
  boost::asio::io_service io_service_;  //!< I/O service for reading/writing

public:
  /**
   * @brief Empty constructor
   */
  IOBase();

  virtual ~IOBase();

  /**
   * @brief Open the i/o service
   */
  virtual void open() = 0;

  /**
   * @brief Close the i/o service
   */
  virtual void close() = 0;

  /**
   * @brief Read one byte from the i/o service with msTimeout Timeout.
   *
   * @param msTimeout Timeout in milliseconds. Set 0 for not using Timeout.
   * @return Read byte as an unsigned char.
   */
  virtual unsigned char readByte(const unsigned int msTimeout = 0) = 0;

  /**
   * @brief Read the number of bytes numOfBytes with msTimeout timeout in milliseconds.
   *
   * @param dataBuffer Buffer to store the read bytes.
   * @param numOfBytes Number of bytes to read.
   * @param msTimeout Timeout in milliseconds. Set 0 for not using Timeout.
   */
  virtual void read(DataBuffer& dataBuffer, const unsigned int numOfBytes = 0, const unsigned int msTimeout = 0) = 0;

  /**
   * @brief Read the i/o service until the delimiter is reached with a Timeout of msTimeout in milliseconds.
   *
   * @param dataBuffer Buffer to store the read bytes.
   * @param delimiter Delimiter to finilize the i/o read.
   * @param msTimeout Timeout in milliseconds. Set 0 for not using Timeout.
   */
  virtual void readUntil(DataBuffer& dataBuffer, const unsigned char delimiter, const unsigned int msTimeout = 0) = 0;

  /**
   * @brief Read the i/o service unthil the line terminator is reached with a Timeout of msTimeout in milliseconds.
   *
   * @param msTimeout Timeout in milliseconds. Set 0 for not using Timeout.
   * @param lineTerminator Delimiter for line ending.
   * @return Read from the i/o service.
   */
  virtual std::string readLine(const unsigned int msTimeout = 0, const unsigned char lineTerminator = '\n') = 0;

  /**
   * @brief Write the byte dataByte to the i/o service.
   *
   * @param dataByte Byte to write to the i/o service.
   */
  virtual void writeByte(const unsigned char dataByte) = 0;

  /**
   * @brief Write the buffer dataBuffer to the i/o service.
   *
   * @param dataBuffer Buffer to write to the i/o service.
   */
  virtual void write(const DataBuffer& dataBuffer) = 0;

  /**
   * @brief Write the string dataString to the i/o service.
   *
   * @param dataString String to write to the i/o service.
   */
  virtual void write(const std::string& dataString) = 0;

  /**
   * @brief Get a reference to the i/o service.
   *
   * @return Reference to the i/o service used.
   */
  boost::asio::io_service& get_io_service()
  {
    return io_service_;
  }
};
/** @} */
}  // namespace io
}  // namespace cola2
