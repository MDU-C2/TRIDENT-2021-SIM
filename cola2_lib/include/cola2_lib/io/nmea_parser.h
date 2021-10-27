/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

/**
 * @file
 * @brief NMEA encoder and decoder with checsum computation and check.
 * @addtogroup io
 * @{
 */

#pragma once

#include <boost/algorithm/string.hpp>
#include <iostream>
#include <string>
#include <vector>

/**
 * @brief Class to parse NMEA streams.
 *
 * An NMEA stream is a sequence of coma separated strings in this format:
 * $HEADER,field1,field2,field3,...fieldN*CK
 */
class NMEAParser
{
private:
  bool valid_;                       //!< Flag to mark if parsed stream is valid according to its checksum field
  std::vector<std::string> fields_;  //!< Array of strings to store each parsed field

  /**
   * @brief Creates a hexadecimal string (string containing 0F1FAB..., not a byte buffer) for the hex value of num with
   * the maximum lenght len.
   *
   * @param num Number to convert to hex string.
   * @param len Maximum nubmer of bytes do convert.
   * @return String containing the hex value of num.
   */
  std::string toHex(uint64_t num, int len) const;

public:
  /**
   * @brief Empty constructor
   */
  NMEAParser();

  /**
   * @brief Constructor parsing the input string s.
   *
   * @param s String to parse.
   */
  NMEAParser(std::string s);

  /**
   * @brief Splits NMEA string s by comas and puts each field in array of strings (fields_).
   *
   * @param s String to parse.
   * @return The number of fields.
   */
  int setFields(std::string s);

  /**
   * @brief Get the number of fields of the last parsed message.
   *
   * @return Number of fields of the last parsed message.
   */
  int size();

  /**
   * @brief Computes checksum for NMEA string str.
   * XOR of all the bytes between the $ and the * (not including the delimiters themselves) and writes the result in
   * hexadecimal.
   *
   * @param str String to compute the check sum for.
   * @return Checksum as a hexadecimal string (string containing 0F1FAB..., not a byte buffer).
   */
  std::string checksum(std::string str) const;

  /**
   * @brief Gets the contents of field i.
   *
   * @param i Index of the field to get data from.
   * @return String of data at field i.
   */
  std::string field(int i) const;

  /**
   * @brief Overload of the operator [] to access the contents of field i.
   *
   * @param i Index of the field to get data from.
   * @return String of data at field i.
   */
  std::string operator[](int i) const;

  /**
   * @brief Sets the contents of the field i to text.
   *
   * @param i Index of the field to set data to.
   * @param text Data to set at field i.
   */
  void setField(int i, const std::string& text);

  /**
   * @brief Returns if the parsed NMEA string is valid (according to the checksum).
   *
   * @return True/false depending on if the parsed NMEA string is valid.
   */
  bool isValid() const;

  /**
   * @brief Returns the header (first field after the $) of the NMEA string.
   *
   * @return String containing the header of the NMEA stream.
   */
  std::string header() const;

  /**
   * @brief Returns the full composed NMEA string (including the checksum).
   *
   * @return Get the NMEA sentence joining all the fields.
   */
  std::string getSentence() const;
};
/** @} */
