# Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
#
# This file is subject to the terms and conditions defined in file
# 'LICENSE.txt', which is part of this source code package.

"""
NMEA encoder and decoder with checsum computation and check.
"""

class NMEAParser(object):
    """
    Class to parse NMEA streams. 
    
    An NMEA stream is a sequence of coma separated strings in this format:

        $HEADER,field1,field2,field3,...fieldN*CK

    :param data: Optional argument to process a raw NMEA string
    :type data: str, optional
    """

    def __init__(self, data=""):
        """
        Constructor.
        """
        self.valid = False
        self.fields = []
        self.hex_lst = list("0123456789ABCDEF")
        if data != "":
            self.setFields(data)

    def setFields(self, data):
        """
        Splits NMEA string by comas and puts each field in array of strings (self.fields).
        Return the self.fields lenght.

        :param data: NMEA raw string (with ',' separating elements)
        :type data: str

        :return: Length of the readed fields
        :rtype: int
        """
        self.fields = []
        try:
            t = data.strip()
            if t[0] == '$':
                self.valid = True
                idx = t.find('*')
                if idx > 0:
                    if idx + 3 == len(t):
                        if t[(idx + 1):] != self.checksum(t):
                            self.valid = False  # Invalid checksum
                    t = t[:idx]
                self.fields = t.split(',')
        except:
            self.valid = False
        return len(self.fields)

    def checksum(self, data):
        """
        Computes checksum for an NMEA string.

        XOR of all the bytes between the $ and the * (not including the delimiters themselves) and writes the result
        in hexadecimal.

        :param data: NMEA raw string without $ and * delimiters
        :type data: str

        :return: Checksum represented in hexadecimal
        :rtype: str
        """
        data_list = list(data)
        chk = 0
        index = 1
        while index < len(data) and data[index] != '*':
            chk = chk ^ ord(data_list[index])  # Bitwise XOR
            index += 1
        return self.toHex(chk, 2)

    def field(self, i):
        """
        Get the contents of the specified field.

        :param i: Index of the field
        :type i: int

        :return: Contents of the field or empty string if index outside bounds
        :rtype: str
        """
        if len(self.fields) > i:
            return self.fields[i]
        return ""

    def setField(self, i, text):
        """
        Set the contents of the specified field to the specified text.

        :param i: Index of the field
        :type i: int
        :param text: Text to set as field value
        :type text: int
        """
        while len(self.fields) <= i:
            self.fields.append("")
        self.fields[i] = text

    def header(self):
        """
        Return the header (first field after the $) of the NMEA string.

        :return: First field of the NMEA string
        :rtype: str
        """
        return self.field(0)

    def size(self):
        """
        Return the amount of fields (length of self.fields)

        :return: Number of fields in the NMEA string
        :rtype: int
        """
        return len(self.fields)

    def isValid(self):
        """
        Return if the parsed NMEA string is valid (according to the checksum).

        :return: Checksum in the NMEA string and computed checksum correspond
        :rtype: bool
        """
        return self.valid

    def toHex(self, num, lenght):
        """
        Convert from the num integer to an hex string of a given length.

        :param num: Value to convert to a hex string
        :type num: int
        :param lenght: Length of the final hex string
        :type lenght: int
        
        :return: Hex string of the input number
        :rtype: str
        """
        s = ["0"] * lenght
        for i in range(lenght - 1, -1, -1):
            tr = num & 0x0F
            s[i] = self.hex_lst[tr]
            num = num >> 4
        return "".join(s)

    def getSentence(self):
        """
        Compose sentence joining all fields with comas in between and adds also the checksum at the end.

        :return: Composed NMEA sequence or empty if no information available
        :rtype: str
        """
        if not self.fields:
            return ""
        s = ','.join(self.fields)
        return s + "*" + self.checksum(s) + "\r\n"
