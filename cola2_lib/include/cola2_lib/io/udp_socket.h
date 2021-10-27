/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

/**
 * @file
 * @brief C++ UDP netowrk communications.
 */

#pragma once

#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/thread.hpp>

#include <iostream>
#include <string>

namespace cola2
{
namespace io
{
/**
 * @addtogroup io
 * @{
 */

namespace udp_socket
{
/**
 * @brief UDP client socket.
 */
class Client
{
private:
  std::string ip_;     //!< IP for the UDP connection.
  std::string port_;   //!< Port for the UDP connection.
  bool enabled_port_;  //!< Flag to check if port_ is set before reading from the connection.

public:
  /**
   * @brief Empty constructor.
   */
  Client();

  /**
   * @brief Set the connection IP.
   *
   * @param ip IP of the UDP server.
   * @return Reference to the clinet udp.
   */
  Client& setIp(const std::string& ip);

  /**
   * @brief Set the connection port.
   *
   * @param port Port of the UDP server.
   * @return Reference to the clinet udp.
   */
  Client& setPort(const std::string& port);

  /**
   * @brief Read from the UDP socket
   *
   * @return String read.
   */
  std::string read();
};

/**
 * @brief UDP server socket.
 */
class Server
{
private:
  boost::mutex message_mutex_;                   //!< Mutex for avoding changing the message while it is being sent.
  std::string port_;                             //!< Port of the UDP server socket.
  bool enabled_port_;                            //!< Flag to check if port_ is set before writting to the socket.
  std::string message_;                          //!< Message to write to the UDP socket.
  boost::shared_ptr<boost::thread> thread_ptr_;  //!< Thread for the Server::run method.

public:
  /**
   * @brief Empty constructor
   */
  Server();

  /**
   * @brief Set the connection port.
   *
   * @param port Port of the UDP server.
   * @return Reference to the clinet udp.
   */
  Server& setPort(const std::string& port);

  /**
   * @brief If the port is set, starts the run() method in a thread.
   *
   * @return Reference to the udp server.
   */
  Server& launch();

  /**
   * @brief Run the server and continuously send the last message set with the update(const std::string& new_message)
   * method.
   *
   * @return Reference to the udp server.
   */
  Server& run();

  /**
   * @brief Join the thread executing the run() method.
   *
   * @return Reference to the udp server.
   */
  Server& join();

  /**
   * @brief Updates the message to be sent to the UDP socket.
   * @param new_message Message to send through UDP socket.
   *
   * @return Reference to the udp server.
   */
  Server& update(const std::string& new_message);
};

}  // namespace udp_socket
/** @} */

}  // namespace io
}  // namespace cola2
