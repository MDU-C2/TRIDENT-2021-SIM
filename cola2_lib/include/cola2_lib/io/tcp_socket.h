/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

/**
 * @file
 * @brief C++ TCP network communications.
 */

#pragma once

#include <boost/asio.hpp>
#include "cola2_lib/io/io_base.h"

#include <boost/asio.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/optional.hpp>
#include <boost/thread.hpp>

#include <algorithm>
#include <iostream>
#include <string>
#include <vector>

namespace cola2
{
namespace io
{
/**
 * @addtogroup io
 * @{
 */

/**
 * @brief TCP socket configuration.
 */
struct TCPConfig
{
  std::string ip;     //!< IP of the TCP connection.
  unsigned int port;  //!< Port of the TCP connection.
};

/**
 * @brief Class to handle device connection and data transmission through a TCP socket
 */
class TcpSocket : public IOBase
{
private:
#if BOOST_VERSION >= 107100
  boost::asio::io_context io_context_;
#endif
  boost::asio::ip::tcp::socket socket_;  //!< Handler for the boost object that provides TCP socket functionality
  TCPConfig config_;                     //!< Structure to hold the configuration of the TCP socket

  void timedReadHandler(boost::optional<boost::system::error_code>* a, const boost::system::error_code& b);

public:
  /**
   * @brief Constructor that configures the TCP socket given the parameters of the TCPConfig structure.
   *
   * @param config TCP configuration.
   */
  TcpSocket(cola2::io::TCPConfig config);

  ~TcpSocket();

  /**
   * @brief Opens the connection to the TCP socket. config_ parameters must be previously set.
   */
  void open();

  /**
   * @brief Closes the TCP socket connection.
   */
  void close();

  /**
   * @brief Reconnects the TCP socket.
   */
  void reconnect();

  //*****************************************************************************
  // Read methods
  //*****************************************************************************

  /**
   * @brief Reads a byte from the TCP socket. The read is synchronous if msTimeout is 0,
   * otherwise it is an asynchronous timed read.
   *
   * @param msTimeout Timeout in milliseconds. If set to 0, read synchronous.
   * @return Byte read from the TCP socket.
   */
  unsigned char readByte(const unsigned int msTimeout);

  /**
   * @brief Reads numOfBytes bytes from the TCP socket and puts them in dataBuffer.
   * The read is synchronous if msTimeout is 0, otherwise it is an asynchronous timed read.
   *
   * @param dataBuffer Buffer to store the read bytes.
   * @param numOfBytes Number of bytes to read.
   * @param msTimeout Timeout in milliseconds.
   */
  void read(DataBuffer& dataBuffer, const unsigned int numOfBytes, const unsigned int msTimeout);

  /**
   * @brief Reads bytes from the TCP socket until it finds a delimiter byte and puts them in dataBuffer.
   *
   * The read is synchronous if msTimeout is 0, otherwise it is an asynchronous timed read.
   *
   * @param dataBuffer Buffer to store the read bytes.
   * @param delimiter Delimiter to finilize reading.
   * @param msTimeout Timeout in milliseconds.
   */
  void readUntil(DataBuffer& dataBuffer, const unsigned char delimiter, const unsigned int msTimeout);

  /**
   * @brief Reads a line from the TCP socket, delimited by lineTerminator.
   *
   * The read is synchronous if msTimeout is 0, otherwise it is an asynchronous timed read.
   *
   * @param msTimeout Timeout in milliseconds.
   * @param lineTerminator Delimiter for line ending.
   * @return String read from the TCP socket.
   */
  std::string readLine(const unsigned int msTimeout, const unsigned char lineTerminator);

  //*****************************************************************************
  // Write methods
  //*****************************************************************************

  /**
   * @brief Writes the dataByte byte to the TCP socket.
   *
   * @param dataByte Byte to write to TCP socket.
   */
  void writeByte(const unsigned char dataByte);

  /**
   * @brief Writes the bytes contained in the dataBuffer DataBuffer to the TCP socket.
   * @param dataBuffer Data to write to the TCP socket.
   */
  void write(const DataBuffer& dataBuffer);

  /**
   * @brief Writes the bytes contained in the dataString string to the TCP socket.
   * @param dataString String to write to the TCP socket.
   */
  void write(const std::string& dataString);
};
/** @} */

}  // namespace io
}  // namespace cola2
