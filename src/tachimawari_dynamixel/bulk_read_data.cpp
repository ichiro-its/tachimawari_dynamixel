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

#include <tachimawari_dynamixel/bulk_read_data.hpp>

#include <tachimawari_dynamixel/cm740.hpp>

namespace tachimawari_dynamixel
{

BulkReadData::BulkReadData()
: start_address(0),
  length(0),
  error(-1)
{
  for (int i = 0; i < MX28::MAXNUM_ADDRESS; i++) {
    table[i] = 0;
  }
}

int BulkReadData::read_byte(int address)
{
  if (address >= start_address && address < (start_address + length)) {
    return (int)table[address];
  }

  return 0;
}

int BulkReadData::read_word(int address)
{
  if (address >= start_address && address < (start_address + length)) {
    return CM740::make_word(table[address], table[address + 1]);
  }

  return 0;
}

}  // namespace tachimawari_dynamixel
