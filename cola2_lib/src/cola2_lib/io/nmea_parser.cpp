/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#include "cola2_lib/io/nmea_parser.h"

NMEAParser::NMEAParser() : valid_(false)
{
}

NMEAParser::NMEAParser(std::string s)
{
  setFields(s);
}

int NMEAParser::setFields(std::string t)
{
  fields_.clear();
  try
  {
    boost::trim(t);
    if (t.at(0) == '$')
    {
      valid_ = true;
      int idx = t.find('*');
      if (idx >= 0)
      {
        if (static_cast<std::size_t>(idx + 3) == t.size())
        {
          if (t.compare(idx + 1, 2, checksum(t)))
          {                  // returns 0 if equal
            valid_ = false;  // checksum not valid
          }
        }
        t.resize(idx);
      }
      boost::split(fields_, t, boost::is_any_of(","));
    }
    else
      valid_ = false;
  }
  catch (std::exception& e)
  {
    valid_ = false;
  }
  return fields_.size();
}

std::string NMEAParser::checksum(std::string s) const
{
  uint8_t chk = 0;
  std::string::iterator it = s.begin() + 1;

  while ((*it != '*') && (it != s.end()))
  {
    chk ^= *it++;  // bitwise XOR
  }
  return (toHex(chk, 2));
}

std::string NMEAParser::header() const
{
  if (fields_.size() > 0)
    return fields_.at(0);
  return "";
}

std::string NMEAParser::field(int i) const
{
  if (fields_.size() > size_t(i))
    return fields_.at(i);
  return "";
}

std::string NMEAParser::operator[](int i) const
{
  return field(i);
}

void NMEAParser::setField(int i, const std::string& text)
{
  if (fields_.size() <= size_t(i))
  {
    fields_.resize(i + 1);
  }
  fields_[i] = text;
}

int NMEAParser::size()
{
  return fields_.size();
}

bool NMEAParser::isValid() const
{
  return valid_;
}

std::string NMEAParser::toHex(uint64_t num, int len) const
{
  static const char* hex = "0123456789ABCDEF";

  len = std::min(size_t(len), sizeof(uint64_t));

  std::string s(len, '0');
  std::string::reverse_iterator it = s.rbegin();

  while (it != s.rend())
  {
    *it++ = hex[num & 0x0F];
    num >>= 4;
  }
  return s;
}

/*
 * Compose sentence joining all fields with comas in between
 * and adds also the checksum at the end
 */
std::string NMEAParser::getSentence() const
{
  if (!fields_.size())
    return std::string();

  std::string s = boost::algorithm::join(fields_, ",");

  s = s + "*" + checksum(s);

  s += "\r\n";
  return s;
}
