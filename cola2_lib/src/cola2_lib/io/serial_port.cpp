
/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#include "cola2_lib/io/serial_port.h"
#include "cola2_lib/utils/error_macro.h"

namespace cola2
{
namespace io
{
SerialPort::SerialPort()
  : IOBase()
#if BOOST_VERSION >= 107100
  , serialPort_(io_context_)
#else
  , serialPort_(get_io_service())
#endif
{
}

SerialPort::SerialPort(const cola2::io::SPConfig config)
  : IOBase()
#if BOOST_VERSION >= 107100
  , serialPort_(io_context_)
#else
  , serialPort_(get_io_service())
#endif
  , config_(config)
{
}

void SerialPort::open()
{
  if (serialPort_.is_open())
    THROW(AlreadyOpen);

  boost::system::error_code ec;
  serialPort_.open(config_.sp_path, ec);

  // Set serial port to raw mode
  struct termios settings;
  if (tcgetattr(serialPort_.native_handle(), &settings))
    THROW(std::runtime_error, "Unable to get termios serial port settings");

  // From https://www.cmrr.umn.edu/~strupp/serial.html. Some of these settings might be unnecessary
  settings.c_cflag &= ~(CSIZE | PARENB);
  settings.c_cflag |= CS8;
  settings.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG | ECHOE | ECHOK | ECHOCTL | ECHOKE);
  settings.c_iflag &= ~(BRKINT | ICRNL | IGNBRK | IGNCR | INLCR | ISTRIP | IXON | PARMRK | INPCK | IGNPAR);
  settings.c_oflag &= ~(OPOST | OLCUC | OCRNL | ONLCR | ONOCR | ONLRET | OFILL | OFDEL);
  settings.c_cc[VTIME] = 0;
  settings.c_cc[VMIN] = 1;

  if (tcsetattr(serialPort_.native_handle(), TCSANOW, &settings))
    THROW(std::runtime_error, "Unable to set termios serial port settings to RAW mode");

  configure();

  if (ec)
    THROW(OpenFailed, ec.message());
}

// For back-compatibility
void SerialPort::open(const std::string& port)
{
  config_.sp_path = port;
  open();
}

void SerialPort::setConfig(const SPConfig config, bool do_configure)
{
  config_ = config;
  if (do_configure)
    configure();
}

void SerialPort::configure()
{
  setBaudRate(baudRateFromInteger(config_.sp_baud_rate));
  setCharSize(charSizeFromInteger(config_.sp_char_size));
  setNumOfStopBits(numOfStopBitsFromInteger(config_.sp_stop_bits));
  setParity(parityFromString(config_.sp_parity));
  setFlowControl(flowControlFromString(config_.sp_flow_control));
}

//*****************************************************************************
// Getters for Serial port configuration settings.
//*****************************************************************************

EBaudRate SerialPort::getBaudRate() const
{
  if (!serialPort_.is_open())
    THROW(NotOpen);

  boost::system::error_code ec;
  boost::asio::serial_port::baud_rate br;
  serialPort_.get_option(br, ec);

  if (ec)
    THROW(std::runtime_error, "Error getting baud rate");

  switch (br.value())
  {
    case 50:
      return BAUD_50;

    case 75:
      return BAUD_75;

    case 110:
      return BAUD_110;

    case 134:
      return BAUD_134;

    case 150:
      return BAUD_150;

    case 200:
      return BAUD_200;

    case 300:
      return BAUD_300;

    case 600:
      return BAUD_600;

    case 1200:
      return BAUD_1200;

    case 1800:
      return BAUD_1800;

    case 2400:
      return BAUD_2400;

    case 4800:
      return BAUD_4800;

    case 9600:
      return BAUD_9600;

    case 19200:
      return BAUD_19200;

    case 38400:
      return BAUD_38400;

    case 57600:
      return BAUD_57600;

    case 115200:
      return BAUD_115200;

    case 230400:
      return BAUD_230400;

#ifdef __linux__
    case 460800:
      return BAUD_460800;
#endif

    default:
      return BAUD_INVALID;
  }
}

ECharacterSize SerialPort::getCharSize() const
{
  if (!serialPort_.is_open())
    THROW(NotOpen);

  boost::system::error_code ec;
  boost::asio::serial_port::character_size cs;
  serialPort_.get_option(cs, ec);

  if (ec)
    THROW(std::runtime_error, "Error getting character size");

  switch (cs.value())
  {
    case 5:
      return CHAR_SIZE_5;

    case 6:
      return CHAR_SIZE_6;

    case 7:
      return CHAR_SIZE_7;

    case 8:
      return CHAR_SIZE_8;

    default:
      return CHAR_SIZE_INVALID;
  }
}

EStopBits SerialPort::getNumOfStopBits() const
{
  if (!serialPort_.is_open())
    THROW(NotOpen);

  boost::system::error_code ec;
  boost::asio::serial_port::stop_bits sb;
  serialPort_.get_option(sb, ec);

  if (ec)
    THROW(std::runtime_error, "Error getting num. of stop bits");

  switch (sb.value())
  {
    case boost::asio::serial_port_base::stop_bits::one:
      return STOP_BITS_1;

    case boost::asio::serial_port_base::stop_bits::two:
      return STOP_BITS_2;

    default:
      return STOP_BITS_INVALID;  // TODO: Add 'stop_bits::one_point_five' option
  }
}

EParity SerialPort::getParity() const
{
  if (!serialPort_.is_open())
    THROW(NotOpen);

  boost::system::error_code ec;
  boost::asio::serial_port::parity p;
  serialPort_.get_option(p, ec);

  if (ec)
    THROW(std::runtime_error, "Error getting parity");

  switch (p.value())
  {
    case boost::asio::serial_port_base::parity::even:
      return PARITY_EVEN;

    case boost::asio::serial_port_base::parity::odd:
      return PARITY_ODD;

    case boost::asio::serial_port_base::parity::none:
      return PARITY_NONE;

    default:
      return PARITY_INVALID;
  }
}

EFlowControl SerialPort::getFlowControl() const
{
  if (!serialPort_.is_open())
    THROW(NotOpen);

  boost::system::error_code ec;
  boost::asio::serial_port::flow_control fc;
  serialPort_.get_option(fc, ec);

  if (ec)
    THROW(std::runtime_error, "Error getting flow control");

  switch (fc.value())
  {
    case boost::asio::serial_port_base::flow_control::hardware:
      return FLOW_CONTROL_HARD;

    case boost::asio::serial_port_base::flow_control::software:
      return FLOW_CONTROL_SOFT;

    case boost::asio::serial_port_base::flow_control::none:
      return FLOW_CONTROL_NONE;

    default:
      return FLOW_CONTROL_INVALID;
  }
}

/*
 * Helper functions.
 */
EBaudRate SerialPort::baudRateFromInteger(const unsigned int baudRate)
{
  switch (baudRate)
  {
    case 50:
      return BAUD_50;
    case 75:
      return BAUD_75;
    case 110:
      return BAUD_110;
    case 134:
      return BAUD_134;
    case 150:
      return BAUD_150;
    case 200:
      return BAUD_200;
    case 300:
      return BAUD_300;
    case 600:
      return BAUD_600;
    case 1200:
      return BAUD_1200;
    case 1800:
      return BAUD_1800;
    case 2400:
      return BAUD_2400;
    case 4800:
      return BAUD_4800;
    case 9600:
      return BAUD_9600;
    case 19200:
      return BAUD_19200;
    case 38400:
      return BAUD_38400;
    case 57600:
      return BAUD_57600;
    case 115200:
      return BAUD_115200;
    case 230400:
      return BAUD_230400;
#ifdef __linux__
    case 460800:
      return BAUD_460800;
#endif
    default:
      THROW(std::invalid_argument, "Invalid baud rate string");
  }
}

ECharacterSize SerialPort::charSizeFromInteger(const unsigned int charSize)
{
  switch (charSize)
  {
    case 5:
      return CHAR_SIZE_5;
    case 6:
      return CHAR_SIZE_6;
    case 7:
      return CHAR_SIZE_7;
    case 8:
      return CHAR_SIZE_8;
    default:
      THROW(std::invalid_argument, "Invalid char size string");
  }
}

EStopBits SerialPort::numOfStopBitsFromInteger(const unsigned int stopBits)
{
  switch (stopBits)
  {
    case 1:
      return STOP_BITS_1;
    case 2:
      return STOP_BITS_2;
    default:
      THROW(std::invalid_argument, "Invalid num. of stop bits string");
  }
}

EParity SerialPort::parityFromString(const std::string& parity)
{
  if (parity == "EVEN")
    return PARITY_EVEN;
  else if (parity == "ODD")
    return PARITY_ODD;
  else if (parity == "NONE")
    return PARITY_NONE;
  else
    THROW(std::invalid_argument, "Invalid parity string");
}

EFlowControl SerialPort::flowControlFromString(const std::string& flowControl)
{
  if (flowControl == "HARD")
    return FLOW_CONTROL_HARD;
  else if (flowControl == "SOFT")
    return FLOW_CONTROL_SOFT;
  else if (flowControl == "NONE")
    return FLOW_CONTROL_NONE;
  else
    THROW(std::invalid_argument, "Invalid flow control string");
}

bool SerialPort::isOpen() const
{
  return serialPort_.is_open();
}

//*****************************************************************************
// Setters for Serial port configuration settings.
//*****************************************************************************
void SerialPort::setBaudRate(const EBaudRate baudRate)
{
  if (!serialPort_.is_open())
    THROW(NotOpen);

  boost::system::error_code ec;

  switch (baudRate)
  {
    case BAUD_50:
      serialPort_.set_option(boost::asio::serial_port_base::baud_rate(50), ec);
      break;

    case BAUD_75:
      serialPort_.set_option(boost::asio::serial_port_base::baud_rate(75), ec);
      break;

    case BAUD_110:
      serialPort_.set_option(boost::asio::serial_port_base::baud_rate(110), ec);
      break;
    case BAUD_134:
      serialPort_.set_option(boost::asio::serial_port_base::baud_rate(134), ec);
      break;
    case BAUD_150:
      serialPort_.set_option(boost::asio::serial_port_base::baud_rate(150), ec);
      break;
    case BAUD_200:
      serialPort_.set_option(boost::asio::serial_port_base::baud_rate(200), ec);
      break;
    case BAUD_300:
      serialPort_.set_option(boost::asio::serial_port_base::baud_rate(300), ec);
      break;
    case BAUD_600:
      serialPort_.set_option(boost::asio::serial_port_base::baud_rate(600), ec);
      break;
    case BAUD_1200:
      serialPort_.set_option(boost::asio::serial_port_base::baud_rate(1200), ec);
      break;
    case BAUD_1800:
      serialPort_.set_option(boost::asio::serial_port_base::baud_rate(1800), ec);
      break;
    case BAUD_2400:
      serialPort_.set_option(boost::asio::serial_port_base::baud_rate(2400), ec);
      break;
    case BAUD_4800:
      serialPort_.set_option(boost::asio::serial_port_base::baud_rate(4800), ec);
      break;
    case BAUD_9600:
      serialPort_.set_option(boost::asio::serial_port_base::baud_rate(9600), ec);
      break;
    case BAUD_19200:
      serialPort_.set_option(boost::asio::serial_port_base::baud_rate(19200), ec);
      break;
    case BAUD_38400:
      serialPort_.set_option(boost::asio::serial_port_base::baud_rate(38400), ec);
      break;
    case BAUD_57600:
      serialPort_.set_option(boost::asio::serial_port_base::baud_rate(57600), ec);
      break;
    case BAUD_115200:
      serialPort_.set_option(boost::asio::serial_port_base::baud_rate(115200), ec);
      break;
    case BAUD_230400:
      serialPort_.set_option(boost::asio::serial_port_base::baud_rate(230400), ec);
      break;
#ifdef __linux__
    case BAUD_460800:
      serialPort_.set_option(boost::asio::serial_port_base::baud_rate(460800), ec);
      break;
#endif
    default:
      THROW(std::invalid_argument, "Invalid baud rate");
      break;
  }

  if (ec)
    THROW(std::runtime_error, "Error setting baud rate");
}

void SerialPort::setCharSize(const ECharacterSize charSize)
{
  if (!serialPort_.is_open())
    THROW(NotOpen);

  boost::system::error_code ec;

  switch (charSize)
  {
    case CHAR_SIZE_5:
      serialPort_.set_option(boost::asio::serial_port_base::character_size(5), ec);
      break;
    case CHAR_SIZE_6:
      serialPort_.set_option(boost::asio::serial_port_base::character_size(6), ec);
      break;
    case CHAR_SIZE_7:
      serialPort_.set_option(boost::asio::serial_port_base::character_size(7), ec);
      break;
    case CHAR_SIZE_8:
      serialPort_.set_option(boost::asio::serial_port_base::character_size(8), ec);
      break;
    default:
      THROW(std::invalid_argument, "Invalid character size");
      break;
  }

  if (ec)
    THROW(std::runtime_error, "Error setting character size");
}

void SerialPort::setNumOfStopBits(const EStopBits stopBits)
{
  if (!serialPort_.is_open())
    THROW(NotOpen);

  boost::system::error_code ec;

  switch (stopBits)
  {
    case STOP_BITS_1:
      serialPort_.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one),
                             ec);
      break;
    case STOP_BITS_2:
      serialPort_.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::two),
                             ec);
      break;
    default:
      serialPort_.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one),
                             ec);
      break;
  }
  if (ec)
    THROW(std::runtime_error, "Error setting num. of stop bits");
}

void SerialPort::setParity(const EParity parity)
{
  if (!serialPort_.is_open())
    THROW(NotOpen);

  boost::system::error_code ec;

  switch (parity)
  {
    case PARITY_EVEN:
      serialPort_.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::even), ec);
      break;

    case PARITY_ODD:
      serialPort_.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::odd), ec);
      break;

    case PARITY_NONE:
      serialPort_.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none), ec);
      break;

    default:
      THROW(std::invalid_argument, "Invalid parity type");
      break;
  }

  if (ec)
    THROW(std::runtime_error, "Error setting parity");
}

void SerialPort::setFlowControl(const EFlowControl flowControl)
{
  if (!serialPort_.is_open())
    THROW(NotOpen);

  boost::system::error_code ec;

  switch (flowControl)
  {
    case FLOW_CONTROL_HARD:
      serialPort_.set_option(
          boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::hardware), ec);
      break;

    case FLOW_CONTROL_SOFT:
      serialPort_.set_option(
          boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::software), ec);
      break;

    case FLOW_CONTROL_NONE:
      serialPort_.set_option(
          boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none), ec);
      break;

    default:
      THROW(std::invalid_argument, "Invalid flow control type");
      break;
  }

  if (ec)
    THROW(std::runtime_error, "Error setting flow control type");
}

//*****************************************************************************
// Methods to read
//*****************************************************************************

unsigned char SerialPort::readByte(const unsigned int msTimeout)
{
  if (!serialPort_.is_open())
    THROW(NotOpen);

  unsigned char byte;

  if (msTimeout == 0)
  {
    // Perform synchronous read
    boost::system::error_code ec;
    size_t numOfBytesRead =
        boost::asio::read(serialPort_, boost::asio::buffer(&byte, 1), boost::asio::transfer_all(), ec);

    if (ec)
      THROW(std::runtime_error, "Could not read synchronously from serial port: " + ec.message());

    if (numOfBytesRead != 1)
      THROW(std::runtime_error, "Unexpected number of bytes read");
  }
  else
  {
    // Perform asynchronous timed read
    boost::optional<boost::system::error_code> timerResult;
#if BOOST_VERSION >= 107100
    boost::asio::deadline_timer timer(serialPort_.get_executor());
#else
    boost::asio::deadline_timer timer(serialPort_.get_io_service());
#endif
    timer.expires_from_now(boost::posix_time::milliseconds(msTimeout));
    timer.async_wait(boost::bind(&SerialPort::timedReadHandler, this, &timerResult, _1));

    boost::optional<boost::system::error_code> readResult;
    boost::asio::async_read(serialPort_, boost::asio::buffer(&byte, 1),
                            boost::bind(&SerialPort::timedReadHandler, this, &readResult, _1));

#if BOOST_VERSION >= 107100
    io_context_.restart();
#else
    serialPort_.get_io_service().reset();
#endif

    boost::system::error_code ec;
    bool timeoutOccurred = false;

#if BOOST_VERSION >= 107100
    while (io_context_.run_one(ec))
#else
    while (serialPort_.get_io_service().run_one(ec))
#endif
    {
      if (readResult)
      {
        timer.cancel(ec);
      }
      else if (timerResult)
      {
        serialPort_.cancel(ec);
        timeoutOccurred = true;
      }
    }

    if (timeoutOccurred)
      THROW(ReadTimeout);

    if (*readResult)
      THROW(std::runtime_error, "Could not perform timed read from serial port (1): " + readResult->message());

    if (ec)
      THROW(std::runtime_error, "Could not perform timed read from serial port (2): " + ec.message());
  }

  return byte;
}

// TODO: check boost::asio::read_some and change reading bytes in a loop
void SerialPort::read(DataBuffer& dataBuffer, const unsigned int numOfBytes, const unsigned int msTimeout)
{
  if (!serialPort_.is_open())
    THROW(NotOpen);

  dataBuffer.resize(0);
  dataBuffer.reserve(numOfBytes);

  for (size_t i = 0; i < numOfBytes; ++i)
    dataBuffer.push_back(readByte(msTimeout));
}

// TODO: check boost::asio::read_until and change reading bytes in a loop
void SerialPort::readUntil(DataBuffer& dataBuffer, const unsigned char delimiter, const unsigned int msTimeout)
{
  if (!serialPort_.is_open())
    THROW(NotOpen);

  char nextChar = 0;
  do
  {
    nextChar = this->readByte(msTimeout);
    dataBuffer.push_back(nextChar);
  } while (nextChar != delimiter);
}

// TODO: check boost::asio::read_until and change reading bytes in a loop
std::string SerialPort::readLine(const unsigned int msTimeout, const unsigned char lineTerminator)
{
  if (!serialPort_.is_open())
    THROW(NotOpen);

  std::string result;
  char nextChar = 0;
  do
  {
    nextChar = this->readByte(msTimeout);
    result += nextChar;
  } while (nextChar != lineTerminator);
  return result;
}

//*****************************************************************************
// Methods to write
//*****************************************************************************

void SerialPort::writeByte(const unsigned char dataByte)
{
  if (!serialPort_.is_open())
    THROW(NotOpen);

  boost::system::error_code ec;
  size_t n = boost::asio::write(serialPort_, boost::asio::buffer(&dataByte, 1), boost::asio::transfer_all(), ec);

  if (ec)
    THROW(std::runtime_error, "Could not write to serial port: " + ec.message());

  if (n != 1)
    THROW(std::runtime_error, "Unexpected number of bytes sent");
}

void SerialPort::write(const DataBuffer& dataBuffer)
{
  BOOST_FOREACH (DataBuffer::value_type byte, dataBuffer)
  {
    writeByte(byte);
  }
}

void SerialPort::write(const std::string& dataString)
{
  BOOST_FOREACH (std::string::value_type c, dataString)
  {
    writeByte(c);
  }
}

/*
void SerialPort::write(const DataBuffer& dataBuffer, std::string node_name, double critic_time)
{
  // Timer
  boost::posix_time::ptime lastSetpointCallback;

  if (isOpen())
  {
    boost::posix_time::ptime previousTime = boost::posix_time::microsec_clock::universal_time();
    lastSetpointCallback = boost::posix_time::microsec_clock::universal_time();

    BOOST_FOREACH(DataBuffer::value_type byte, dataBuffer)
          {
            writeByte(byte);
          }

    double dt = 0.001 * boost::posix_time::time_period(previousTime,
boost::posix_time::microsec_clock::universal_time()).length().total_microseconds();

    // Compute time
    if (dt > critic_time)
    {
      std::cout << node_name << ": slow serial port. Total elapsed time: " << " ms" << std::endl;
    }
  }
  else
  {
    std::cerr << node_name << ": serial port not open when sending!" << std::endl;
  }
}
*/

void SerialPort::sendBreak()
{
  if (!serialPort_.is_open())
    THROW(NotOpen);
  boost::system::error_code ec;
  serialPort_.send_break(ec);
  if (ec)
    THROW(std::runtime_error, ec.message());
}

void SerialPort::close()
{
  if (!serialPort_.is_open())
    THROW(NotOpen);

  boost::system::error_code ec;
  serialPort_.close(ec);
}

void SerialPort::timedReadHandler(boost::optional<boost::system::error_code>* a, const boost::system::error_code& b)
{
  a->reset(b);
}

}  // namespace io
}  // namespace cola2
