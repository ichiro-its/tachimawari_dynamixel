// Copyright (c) 2021 Ichiro ITS
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#ifndef TACHIMAWARI_DYNAMIXEL__BULK_READ_DATA_HPP_
#define TACHIMAWARI_DYNAMIXEL__BULK_READ_DATA_HPP_

#include "controller/mx28.h"

namespace tachimawari_dynamixel
{

class BulkReadData
{
public:
  int start_address;
  int length;
  int error;
  unsigned char table[MX28::MAXNUM_ADDRESS];

  BulkReadData();

  int read_byte(int address);
  int read_word(int address);
};

}  // namespace tachimawari_dynamixel

#endif  // TACHIMAWARI_DYNAMIXEL__BULK_READ_DATA_HPP_