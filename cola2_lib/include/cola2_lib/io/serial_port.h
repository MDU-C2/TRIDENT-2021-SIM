/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

/**
 * @file
 * @brief C++ serial port communications.
 */

#pragma once

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/date_time.hpp>
#include <boost/foreach.hpp>
#include <boost/noncopyable.hpp>
#include <boost/optional.hpp>

#include <string>

#include "cola2_lib/io/io_base.h"

namespace cola2
{
namespace io
{
/**
 * @addtogroup io
 * @{
 */

/**
 * @brief Serial port configuration structure.
 */
struct SPConfig
{
  std::string sp_path;          //!< Path.
  int sp_baud_rate;             //!< Baud rate.
  int sp_char_size;             //!< Char size.
  int sp_stop_bits;             //!< Stop bits.
  std::string sp_parity;        //!< Parity.
  std::string sp_flow_control;  //!< Flow control.
  int sp_timeout;               //!< Timeout.
};

/**
 * @brief Serial port baud rates.
 */
enum EBaudRate
{
  BAUD_50,      //!< Baud rate 50.
  BAUD_75,      //!< Baud rate 75.
  BAUD_110,     //!< Baud rate 110.
  BAUD_134,     //!< Baud rate 134.
  BAUD_150,     //!< Baud rate 150.
  BAUD_200,     //!< Baud rate 200.
  BAUD_300,     //!< Baud rate 300.
  BAUD_600,     //!< Baud rate 600.
  BAUD_1200,    //!< Baud rate 1200.
  BAUD_1800,    //!< Baud rate 1800.
  BAUD_2400,    //!< Baud rate 2400.
  BAUD_4800,    //!< Baud rate 4800.
  BAUD_9600,    //!< Baud rate 9600.
  BAUD_19200,   //!< Baud rate 19200.
  BAUD_38400,   //!< Baud rate 38400.
  BAUD_57600,   //!< Baud rate 57600.
  BAUD_115200,  //!< Baud rate 115200.
  BAUD_230400,  //!< Baud rate 230400.
//
// Note: B460800 is defined on Linux but not on Mac OS
//
#ifdef __linux__
  BAUD_460800,  //!< Baud rate 460800.
#endif
  BAUD_INVALID,              //!< Invalid baud rate.
  BAUD_DEFAULT = BAUD_57600  //!< Default baud rate 57600.
};

/**
 * @brief Serial port character sizes.
 */
enum ECharacterSize
{
  CHAR_SIZE_5,                     //!< 5 bit characters.
  CHAR_SIZE_6,                     //!< 6 bit characters.
  CHAR_SIZE_7,                     //!< 7 bit characters.
  CHAR_SIZE_8,                     //!< 8 bit characters.
  CHAR_SIZE_INVALID,               //!< Invalid character size.
  CHAR_SIZE_DEFAULT = CHAR_SIZE_8  //!< Default char size 8 bit.
};

/**
 * @brief Serial port numbers of stop bits.
 */
enum EStopBits
{
  STOP_BITS_1,                     //!< 1 stop bit.
  STOP_BITS_2,                     //!< 2 stop bits.
  STOP_BITS_INVALID,               //!< Invalid stop bits.
  STOP_BITS_DEFAULT = STOP_BITS_1  //!< Default stop bit 1.
};

/**
 * @brief Serial port parity checking modes.
 */
enum EParity
{
  PARITY_EVEN,                  //!< Even parity.
  PARITY_ODD,                   //!< Odd parity.
  PARITY_NONE,                  //!< No parity i.e. parity checking disabled.
  PARITY_INVALID,               //!< Invalid parity checking.
  PARITY_DEFAULT = PARITY_NONE  //!< Default parity none.
};

/**
 * @brief Serial port flow control modes.
 */
enum EFlowControl
{
  FLOW_CONTROL_HARD,                        //!< Flow control: hardware.
  FLOW_CONTROL_SOFT,                        //!< Flow control: software.
  FLOW_CONTROL_NONE,                        //!< Flow control: none.
  FLOW_CONTROL_INVALID,                     //!< Invalid flow control.
  FLOW_CONTROL_DEFAULT = FLOW_CONTROL_NONE  //!< Default flow control: none.
};

/**
 * @brief Class to handle device connection and data transmission through serial port.
 */

class SerialPort : public IOBase, private boost::noncopyable
{
private:
#if BOOST_VERSION >= 107100
  boost::asio::io_context io_context_;
#endif
  mutable boost::asio::serial_port serialPort_;  //!< Handler for the boost object that provides serial port
                                                 //!< functionality.
  cola2::io::SPConfig config_;                   //!< Structure to hold the configuration of the serial port.

  void timedReadHandler(boost::optional<boost::system::error_code>* a, const boost::system::error_code& b);

public:
  /**
   * @brief Default constructor, initializes the SerialPort object without configuring it.
   */
  SerialPort();

  /**
   * @brief Constructor that configures the serial port given the parameters of the SPConfig structure.
   *
   * @param config Serial port configuration.
   */
  SerialPort(const SPConfig config);

  /**
   * @brief Opens the connection to the serial port. config_ parameters must be previously set.
   */
  void open();

  /**
   * @brief Opens the connection to the serial port using the port name passed as parameter. It uses the already given
   * (or default if none was given) SPConfig.
   *
   * @param port Serial port to open.
   */
  void open(const std::string& port);

  /**
   * @brief Checks whether the serial port is open or not.
   *
   * @return Bolean true/false depending on the port state (open/close).
   */
  bool isOpen() const;

  /**
   * @brief Closes the serial port connection.
   */
  void close();

  /**
   * @brief Changes the current configuration for the given one.
   *
   * @param config Serial port configuration.
   * @param do_configure Reconfigures the serial port with the given configuration.
   */
  void setConfig(const SPConfig config, bool do_configure = true);

  /**
   * @brief Resets the serial port to fit the current configuration.
   */
  void configure();

  //*****************************************************************************
  // Getters for Serial port configuration settings.
  //*****************************************************************************

  /**
   * @brief Gets the currently configured EBaudRate.
   *
   * @return Currently configured EBaudRate.
   */
  EBaudRate getBaudRate() const;

  /**
   * @brief Gets the currently configured ECharacterSize.
   *
   * @return Currently configured ECharacterSize.
   */
  ECharacterSize getCharSize() const;

  /**
   * @brief Gets the currently configured EStopBits.
   *
   * @return Currently configured EStopBits.
   */
  EStopBits getNumOfStopBits() const;

  /**
   * @brief Gets the currently configured EParity.
   *
   * @return Currently configured EParity.
   */
  EParity getParity() const;

  /**
   * @brief Gets the currently configured EFlowControl.
   *
   * @return Currently configured EFlowControl.
   */
  EFlowControl getFlowControl() const;

  //*****************************************************************************
  // Setters for Serial port configuration settings.
  //*****************************************************************************

  /**
   * @brief Sets the serial port baud rate.
   *
   * @param baudRate Baud rate to set the serial port to.
   */
  void setBaudRate(const EBaudRate baudRate);
  /**
   * @brief Sets the serial port character size.
   *
   * @param charSize Character size to set the serial port to.
   */
  void setCharSize(const ECharacterSize charSize);

  /**
   * @brief Sets the serial port number of stop bits.
   *
   * @param stopBits Stop bits to set the serial port to.
   */
  void setNumOfStopBits(const EStopBits stopBits);

  /**
   * @brief Sets the serial port parity.
   *
   * @param parity Parity to set the serial port to.
   */
  void setParity(const EParity parity);

  /**
   * @brief Sets the serial port flow control.
   *
   * @param flowControl Flow control to set the serial port to.
   */
  void setFlowControl(const EFlowControl flowControl);

  //********************************************************************************
  // Helper functions to map the configuration parameters to the corresponding Enums
  //********************************************************************************

  /**
   * @brief Gets the EBaudRate corresponding to the integer baudRate parameter.
   *
   * @return EBaudRate corresponding to baudRate.
   */
  static EBaudRate baudRateFromInteger(const unsigned int baudRate);
  /**
   * @brief Gets the ECharacterSize corresponding to the integer charSize parameter.
   *
   * @return ECharacterSize corresponding to charSize.
   */
  static ECharacterSize charSizeFromInteger(const unsigned int charSize);
  /**
   * @brief Gets the EStopBits corresponding to the integer stopBits parameter.
   *
   * @return EStopBits corresponding to stopBits.
   */
  static EStopBits numOfStopBitsFromInteger(const unsigned int stopBits);
  /**
   * @brief Gets the EParity corresponding to the integer parity parameter.
   *
   * @return EParity corresponding to parity.
   */
  static EParity parityFromString(const std::string& parity);
  /**
   * @brief Gets the EFlowControl corresponding to the integer flowControl parameter.
   *
   * @return EFlowControl corresponding to flowControl.
   */
  static EFlowControl flowControlFromString(const std::string& flowControl);

  //*****************************************************************************
  // Methods to read
  //*****************************************************************************

  /**
   * @brief Reads a byte from the serial port. The read is synchronous if msTimeout is 0,
   * otherwise it is an asynchronous timed read.
   *
   * @param msTimeout Timeout in milliseconds. If it is 0, the read is synchronous.
   * @return Read byte
   */
  unsigned char readByte(const unsigned int msTimeout = 0);

  /**
   * @brief Reads numOfBytes bytes from the serial port and puts them in dataBuffer.
   * The read is synchronous if msTimeout is 0, otherwise it is an asynchronous timed read.
   *
   * @param dataBuffer Buffer to store the read bytes.
   * @param numOfBytes Number of bytes to read.
   * @param msTimeout Timeout in milliseconds. Set 0 for synchronous read.
   */
  void read(DataBuffer& dataBuffer, const unsigned int numOfBytes = 0, const unsigned int msTimeout = 0);

  /**
   * @brief Reads bytes from the serial port until it finds a delimiter byte and puts them in dataBuffer.
   * The read is synchronous if msTimeout is 0, otherwise it is an asynchronous timed read.
   *
   * @param dataBuffer Buffer to store the read bytes.
   * @param delimiter Delimiter to finilize the serial port read.
   * @param msTimeout Timeout in milliseconds. Set 0 for synchronous read.
   */
  void readUntil(DataBuffer& dataBuffer, const unsigned char delimiter, const unsigned int msTimeout = 0);

  /**
   * @brief Reads a line from the serial port, delimited by lineTerminator.
   * The read is synchronous if msTimeout is 0, otherwise it is an asynchronous timed read.
   *
   * @param msTimeout Timeout in milliseconds. Set 0 for synchronous read.
   * @param lineTerminator Delimiter for line ending.
   * @return String read from the serial port.
   */
  std::string readLine(const unsigned int msTimeout = 0, const unsigned char lineTerminator = '\n');

  //*****************************************************************************
  // Methods to write
  //*****************************************************************************

  /**
   * @brief Writes the dataByte byte to the serial port.
   *
   * @param dataByte Byte to write to serial port.
   */
  void writeByte(const unsigned char dataByte);

  /**
   * @brief Writes the bytes contained in the dataBuffer DataBuffer to the serial port.
   *
   * @param dataBuffer Buffer to write to serial port.
   */
  void write(const DataBuffer& dataBuffer);

  /**
   * @brief Writes the bytes contained in the dataString string to the serial port.
   *
   * @param dataString String to write to the serial port.
   */
  void write(const std::string& dataString);

  /**
   * @brief Writes the bytes contained in the dataBuffer DataBuffer serial port ensuring that
   * we can write respecting a critic_time speed.
   */
  /*void write(const DataBuffer& dataBuffer, std::string node_name, double critic_time);*/

  /**
   * @brief Sends a break to the serial port.
   */
  void sendBreak();

  //*****************************************************************************
  // Classes to handle Exceptions
  //*****************************************************************************

  /**
   * @brief Class to handle the Exception that is arised when the serial port is not open.
   */
  class NotOpen : public std::logic_error
  {
  public:
    /**
     * @brief Empty constructor
     */
    NotOpen() : logic_error("Serial port is not open")
    {
    }

    /**
     * @brief Custom error message constructor.
     *
     * @param whatArg Error message.
     */
    NotOpen(const std::string& whatArg) : logic_error(whatArg)
    {
    }
  };

  /**
   * @brief Class to handle the Exception that is arised when the serial port fails to open.
   */
  class OpenFailed : public std::runtime_error
  {
  public:
    /**
     * @brief Empty constructor
     */
    OpenFailed() : runtime_error("Serial port failed to open")
    {
    }

    /**
     * @brief Custom error message constructor.
     *
     * @param whatArg Error message.
     */
    OpenFailed(const std::string& whatArg) : runtime_error(whatArg)
    {
    }
  };

  /**
   * @brief Class to handle the Exception that is arised when the serial port was already open.
   */
  class AlreadyOpen : public std::logic_error
  {
  public:
    /**
     * @brief Empty constructor
     */
    AlreadyOpen() : logic_error("Serial port was already open")
    {
    }

    /**
     * @brief Custom error message constructor.
     *
     * @param whatArg Error message.
     */
    AlreadyOpen(const std::string& whatArg) : logic_error(whatArg)
    {
    }
  };

  /**
   * @brief Class to handle the Exception that is arised when trying to configure an unsupported baud rate.
   */
  class UnsupportedBaudRate : public std::runtime_error
  {
  public:
    /**
     * @brief Empty constructor
     */
    UnsupportedBaudRate() : runtime_error("Unsupported baud rate")
    {
    }

    /**
     * @brief Custom error message constructor.
     *
     * @param whatArg Error message.
     */
    UnsupportedBaudRate(const std::string& whatArg) : runtime_error(whatArg)
    {
    }
  };

  /**
   * @brief Class to handle the Exception that is arised when the serial port timeout expires while attempting to read.
   */
  class ReadTimeout : public std::runtime_error
  {
  public:
    /**
     * @brief Empty constructor
     */
    ReadTimeout() : runtime_error("Serial port read timeout")
    {
    }

    /**
     * @brief Custom error message constructor.
     *
     * @param whatArg Error message.
     */
    ReadTimeout(const std::string& whatArg) : runtime_error(whatArg)
    {
    }
  };
};
/** @} */

}  // namespace io
}  // namespace cola2
