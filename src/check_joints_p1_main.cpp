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

#include <tachimawari_dynamixel/cm740.hpp>

#include <iostream>
#include <iomanip>
#include <vector>
#include <numeric>
#include <string>

int main(int argc, char * argv[])
{
  std::string port_name = "/dev/ttyUSB0";
  int baudrate = 1000000;

  std::vector<uint8_t> ids(20);
  std::iota(ids.begin(), ids.end(), 1);

  if (argc > 1) {
    port_name = argv[1];

    if (argc > 2) {
      baudrate = std::stoi(argv[2]);
    }
  }

  std::cout << "set the port name as " << port_name << "\n";
  tachimawari_dynamixel::CM740 cm(port_name);

  std::cout << "set the baudrate to " << baudrate << "\n";
  cm.set_baudrate(baudrate);

  std::cout << "open the port\n";
  if (cm.connect()) {
    std::cout << "succeeded to open the port!\n";
  } else {
    std::cout << "failed to open the port!\n" <<
      "try again!\n";
    return 0;
  }

  std::cout << "\033c";

  std::cout << "ping the joints\n\n";
  for (auto id : ids) {
    if (cm.ping(id, 0) != tachimawari_dynamixel::SUCCESS) {
      std::cout << "[ID: " << std::setfill('0') << std::setw(2) << int(id) << "] ping failed.\n";
    } else {
      std::cout << "[ID: " << std::setfill('0') << std::setw(2) << int(id) << "] ping succeeded.\n";
    }
  }

  std::cout << "\nping done\n" <<
    "close the port\n";

  return 0;
}
