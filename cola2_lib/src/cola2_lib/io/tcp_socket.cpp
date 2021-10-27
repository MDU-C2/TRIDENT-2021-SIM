
/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#include "cola2_lib/io/tcp_socket.h"

namespace cola2
{
namespace io
{
TcpSocket::TcpSocket(TCPConfig config)
  : IOBase()
#if BOOST_VERSION >= 107100
  , socket_(io_context_)
#else
  , socket_(get_io_service())
#endif
  , config_(config)
{
}

TcpSocket::~TcpSocket()
{
}

void TcpSocket::open()
{
  boost::asio::ip::tcp::resolver resolver(get_io_service());
  boost::asio::ip::tcp::resolver::query query(config_.ip, boost::lexical_cast<std::string>(config_.port));
  boost::asio::ip::tcp::resolver::iterator endpoint_iterator = resolver.resolve(query);

  boost::system::error_code error = boost::asio::error::host_not_found;
  boost::asio::connect(socket_, endpoint_iterator, error);

  if (error)
    throw boost::system::system_error(error);
}

void TcpSocket::close()
{
  boost::system::error_code ec;

  socket_.close(ec);

  if (ec)
    throw boost::system::system_error(ec);
}

void TcpSocket::reconnect()
{
  close();
  open();
}

//*****************************************************************************
// Read methods
//*****************************************************************************

unsigned char TcpSocket::readByte(const unsigned int msTimeout)
{
  DataBuffer dataBuffer;
  read(dataBuffer, 1, msTimeout);

  return *(dataBuffer.data());
}

void TcpSocket::read(DataBuffer& dataBuffer, const unsigned int numOfBytes, const unsigned int msTimeout)
{
  unsigned int readBytes;

  if (numOfBytes == 0)
  {
    readBytes = socket_.available();
  }
  else
  {
    readBytes = numOfBytes;
  }

  dataBuffer.resize(readBytes);

  if (msTimeout == 0)
  {
    // Perform synchronous read
    boost::system::error_code ec;

    size_t numOfBytesRead =
        boost::asio::read(socket_, boost::asio::buffer(dataBuffer), boost::asio::transfer_all(), ec);

    if (numOfBytesRead != readBytes)
      throw std::runtime_error("Unexpected number of bytes read");

    if (ec)
      throw std::runtime_error("Could not read from TCP socket : " + ec.message());
  }
  else
  {
    // Perform asynchronous timed read
    boost::optional<boost::system::error_code> timerResult;

#if BOOST_VERSION >= 107100
    boost::asio::deadline_timer timer(socket_.get_executor());
#else
    boost::asio::deadline_timer timer(socket_.get_io_service());
#endif
    timer.expires_from_now(boost::posix_time::milliseconds(msTimeout));
    timer.async_wait(boost::bind(&TcpSocket::timedReadHandler, this, &timerResult, _1));

    boost::optional<boost::system::error_code> readResult;
    boost::asio::async_read(socket_, boost::asio::buffer(dataBuffer),
                            boost::bind(&TcpSocket::timedReadHandler, this, &readResult, _1));

#if BOOST_VERSION >= 107100
    io_context_.restart();
#else
    socket_.get_io_service().reset();
#endif

    boost::system::error_code ec;
    bool timeoutOccurred = false;

#if BOOST_VERSION >= 107100
    while (io_context_.run_one(ec))
#else
    while (socket_.get_io_service().run_one(ec))
#endif
    {
      if (readResult)
      {
        timer.cancel(ec);
      }
      else if (timerResult)
      {
        socket_.cancel(ec);
        timeoutOccurred = true;
      }
    }

    if (timeoutOccurred)
      throw std::runtime_error("TimeOut: " + readResult->message());

    if (*readResult)
      throw std::runtime_error("Could not perform timed read from TCP socket (1): " + readResult->message());

    if (ec)
      throw std::runtime_error("Could not perform timed read from TCP socket (2): " + ec.message());
  }
}

void TcpSocket::readUntil(DataBuffer& dataBuffer, const unsigned char delimiter, const unsigned int msTimeout)
{
  std::string line = readLine(msTimeout, delimiter);
  dataBuffer.clear();
  dataBuffer.resize(line.length());
  std::copy(line.begin(), line.end(), dataBuffer.begin());
}

std::string TcpSocket::readLine(const unsigned int msTimeout, const unsigned char lineTerminator)
{
  std::string result;
  boost::asio::streambuf input_buffer;

  if (msTimeout == 0)
  {
    // Perform synchronous read

    boost::system::error_code ec;
    boost::asio::read_until(socket_, input_buffer, lineTerminator, ec);
    if (ec)
      throw std::runtime_error("Could not read from TCP socket : " + ec.message());
  }
  else
  {
    // Perform asynchronous timed read

    boost::optional<boost::system::error_code> timerResult;

#if BOOST_VERSION >= 107100
    boost::asio::deadline_timer timer(socket_.get_executor());
#else
    boost::asio::deadline_timer timer(socket_.get_io_service());
#endif
    timer.expires_from_now(boost::posix_time::milliseconds(msTimeout));

    timer.async_wait(boost::bind(&TcpSocket::timedReadHandler, this, &timerResult, _1));

    boost::optional<boost::system::error_code> readResult;

    boost::asio::async_read_until(socket_, input_buffer, lineTerminator,
                                  boost::bind(&TcpSocket::timedReadHandler, this, &readResult, _1));

    boost::system::error_code ec;

    bool timeoutOccurred = false;

#if BOOST_VERSION >= 107100
    while (io_context_.run_one(ec))
#else
    while (socket_.get_io_service().run_one(ec))
#endif
    {
      if (readResult)
      {
        timer.cancel(ec);
      }
      else if (timerResult)
      {
        socket_.cancel(ec);
        timeoutOccurred = true;
      }
    }

#if BOOST_VERSION >= 107100
    io_context_.restart();
#else
    socket_.get_io_service().reset();
#endif

    if (timeoutOccurred)
      throw std::runtime_error("TimeOut): " + readResult->message());

    if (*readResult)
      throw std::runtime_error("Could not perform timed read from TCP socket (1): " + readResult->message());

    if (ec)
      throw std::runtime_error("Could not perform timed read from TCP socket (2): " + ec.message());
  }
  std::istream is(&input_buffer);
  std::getline(is, result);
  return result;
}

//*****************************************************************************
// Write methods
//*****************************************************************************

void TcpSocket::writeByte(const unsigned char dataByte)
{
  DataBuffer dataBuffer;

  dataBuffer.resize(1);
  *(dataBuffer.data()) = dataByte;

  write(dataBuffer);
}

void TcpSocket::write(const DataBuffer& dataBuffer)
{
  boost::system::error_code ec;
  size_t numOfBytesWrite =
      boost::asio::write(socket_, boost::asio::buffer(dataBuffer), boost::asio::transfer_all(), ec);

  if (ec)
    throw std::runtime_error("Could not write to TCP socket: " + ec.message());

  if (numOfBytesWrite != dataBuffer.size())
    throw std::runtime_error("Unexpected number of bytes sent");
}

void TcpSocket::write(const std::string& dataString)
{
  boost::system::error_code ec;
  size_t numOfBytesWrite = boost::asio::write(socket_, boost::asio::buffer(dataString.data(), dataString.length()),
                                              boost::asio::transfer_all(), ec);

  if (ec)
    throw std::runtime_error("Could not write to TCP socket: " + ec.message());

  if (numOfBytesWrite != dataString.length())
    throw std::runtime_error("Unexpected number of bytes sent");
}

void TcpSocket::timedReadHandler(boost::optional<boost::system::error_code>* a, const boost::system::error_code& b)
{
  a->reset(b);
}

}  // namespace io
}  // namespace cola2
