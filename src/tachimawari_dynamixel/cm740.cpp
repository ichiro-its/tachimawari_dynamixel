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

#include <string>
#include <chrono>
#include <thread>

#include "tachimawari_dynamixel/cm740.hpp"

#include "./errno.h"
#include "./fcntl.h"
#include "./stdio.h"
#include "./string.h"
#include "./termios.h"
#include "./unistd.h"
#include "linux/serial.h"
#include "sys/ioctl.h"
#include "sys/time.h"

using namespace std::chrono_literals;  // NOLINT

namespace tachimawari_dynamixel
{

CM740::BulkReadData::BulkReadData()
: start_address(0),
  length(0),
  error(-1)
{
  for (int i = 0; i < MXP1_MAXNUM_ADDRESS; i++) {
    table[i] = 0;
  }
}

int CM740::BulkReadData::read_byte(int address)
{
  if (address >= start_address && address < (start_address + length)) {
    return static_cast<int>(table[address]);
  }

  return 0;
}

int CM740::BulkReadData::read_word(int address)
{
  if (address >= start_address && address < (start_address + length)) {
    return CM740::make_word(table[address], table[address + 1]);
  }

  return 0;
}

CM740::CM740(std::string port_name)
{
  DEBUG_PRINT = false;
  m_Socket_fd = -1;
  m_PacketStartTime = 0;
  m_PacketWaitTime = 0;
  m_ByteTransferTime = 0;

  sem_init(&m_LowSemID, 0, 1);
  sem_init(&m_MidSemID, 0, 1);
  sem_init(&m_HighSemID, 0, 1);

  set_port_name(port_name);
  baudrate = 1000000;

  DEBUG_PRINT = false;

  for (int i = 0; i < ID_BROADCAST; i++) {
    m_bulk_readData[i] = BulkReadData();
  }
}

CM740::~CM740()
{
  close_port();
  disconnect();
}

int CM740::txrx_packet(unsigned char * txpacket, unsigned char * rxpacket, int priority)
{
  if (priority > 1) {
    low_priority_wait();
  }
  if (priority > 0) {
    mid_priority_wait();
  }
  high_priority_wait();

  int res = TX_FAIL;
  int length = txpacket[INDEX_LENGTH] + 4;

  txpacket[0] = 0xFF;
  txpacket[1] = 0xFF;
  txpacket[length - 1] = calculate_checksum(txpacket);

  if (DEBUG_PRINT == true) {
    fprintf(stderr, "\nTX: ");
    for (int n = 0; n < length; n++) {
      fprintf(stderr, "%.2X ", txpacket[n]);
    }

    fprintf(stderr, "INST: ");
    switch (txpacket[INDEX_INSTRUCTION]) {
      case INST_PING:
        fprintf(stderr, "PING\n");
        break;

      case INST_READ:
        fprintf(stderr, "READ\n");
        break;

      case INST_WRITE:
        fprintf(stderr, "WRITE\n");
        break;

      case INST_REG_WRITE:
        fprintf(stderr, "REG_WRITE\n");
        break;

      case INST_ACTION:
        fprintf(stderr, "ACTION\n");
        break;

      case INST_RESET:
        fprintf(stderr, "RESET\n");
        break;

      case INST_SYNC_WRITE:
        fprintf(stderr, "SYNC_WRITE\n");
        break;

      case INST_BULK_READ:
        fprintf(stderr, "BULK_READ\n");
        break;

      default:
        fprintf(stderr, "UNKNOWN\n");
        break;
    }
  }

  if (length < (MAXNUM_TXPARAM + 6)) {
    clear_port();
    if (write_port(txpacket, length) == length) {
      if (txpacket[INDEX_ID] != ID_BROADCAST) {
        int to_length = 0;

        if (txpacket[INDEX_INSTRUCTION] == INST_READ) {
          to_length = txpacket[INDEX_PARAMETER + 1] + 6;
        } else {
          to_length = 6;
        }

        set_packet_timeout(length);

        int get_length = 0;
        if (DEBUG_PRINT == true) {
          fprintf(stderr, "RX: ");
        }

        while (1) {
          length = read_port(&rxpacket[get_length], to_length - get_length);
          if (DEBUG_PRINT == true) {
            for (int n = 0; n < length; n++) {
              fprintf(stderr, "%.2X ", rxpacket[get_length + n]);
            }
          }
          get_length += length;

          if (get_length == to_length) {
            // Find packet header
            int i;
            for (i = 0; i < (get_length - 1); i++) {
              if (rxpacket[i] == 0xFF && rxpacket[i + 1] == 0xFF) {
                break;
              } else if (i == (get_length - 2) && rxpacket[get_length - 1] == 0xFF) {
                break;
              }
            }

            if (i == 0) {
              // Check checksum
              unsigned char checksum = calculate_checksum(rxpacket);
              if (DEBUG_PRINT == true) {
                fprintf(stderr, "CHK:%.2X\n", checksum);
              }

              if (rxpacket[get_length - 1] == checksum) {
                res = SUCCESS;
              } else {
                res = RX_CORRUPT;
              }

              break;
            } else {
              for (int j = 0; j < (get_length - i); j++) {
                rxpacket[j] = rxpacket[j + i];
              }
              get_length -= i;
            }
          } else {
            if (is_packet_timeout() == true) {
              if (get_length == 0) {
                res = RX_TIMEOUT;
              } else {
                res = RX_CORRUPT;
              }

              break;
            }
          }
        }
      } else if (txpacket[INDEX_INSTRUCTION] == INST_BULK_READ) {
        int to_length = 0;
        int num = (txpacket[INDEX_LENGTH] - 3) / 3;

        for (int x = 0; x < num; x++) {
          int _id = txpacket[INDEX_PARAMETER + (3 * x) + 2];
          int _len = txpacket[INDEX_PARAMETER + (3 * x) + 1];
          int _addr = txpacket[INDEX_PARAMETER + (3 * x) + 3];

          to_length += _len + 6;
          m_bulk_readData[_id].length = _len;
          m_bulk_readData[_id].start_address = _addr;
        }

        set_packet_timeout(to_length * 1.5);

        int get_length = 0;
        if (DEBUG_PRINT == true) {
          fprintf(stderr, "RX: ");
        }

        while (1) {
          length = read_port(&rxpacket[get_length], to_length - get_length);
          if (DEBUG_PRINT == true) {
            for (int n = 0; n < length; n++) {
              fprintf(stderr, "%.2X ", rxpacket[get_length + n]);
            }
          }
          get_length += length;

          if (get_length == to_length) {
            res = SUCCESS;
            break;
          } else {
            if (is_packet_timeout() == true) {
              if (get_length == 0) {
                res = RX_TIMEOUT;
              } else {
                res = RX_CORRUPT;
              }

              break;
            }
          }
        }

        for (int x = 0; x < num; x++) {
          int _id = txpacket[INDEX_PARAMETER + (3 * x) + 2];
          m_bulk_readData[_id].error = -1;
        }

        while (1) {
          int i;
          for (i = 0; i < get_length - 1; i++) {
            if (rxpacket[i] == 0xFF && rxpacket[i + 1] == 0xFF) {
              break;
            } else if (i == (get_length - 2) && rxpacket[get_length - 1] == 0xFF) {
              break;
            }
          }

          if (i == 0) {
            // Check checksum
            unsigned char checksum = calculate_checksum(rxpacket);
            if (DEBUG_PRINT == true) {
              fprintf(stderr, "CHK:%.2X\n", checksum);
            }

            if (rxpacket[INDEX_LENGTH + rxpacket[INDEX_LENGTH]] == checksum) {
              for (int j = 0; j < (rxpacket[INDEX_LENGTH] - 2); j++) {
                m_bulk_readData[rxpacket[INDEX_ID]].table[m_bulk_readData[rxpacket[INDEX_ID]].
                  start_address +
                  j] = rxpacket[INDEX_PARAMETER + j];
              }

              m_bulk_readData[rxpacket[INDEX_ID]].error = static_cast<int>(rxpacket[INDEX_ERRBIT]);

              int cur_packet_length = INDEX_LENGTH + 1 + rxpacket[INDEX_LENGTH];
              to_length = get_length - cur_packet_length;
              for (int j = 0; j <= to_length; j++) {
                rxpacket[j] = rxpacket[j + cur_packet_length];
              }

              get_length = to_length;
              num--;
            } else {
              res = RX_CORRUPT;

              for (int j = 0; j <= get_length - 2; j++) {
                rxpacket[j] = rxpacket[j + 2];
              }

              to_length = get_length -= 2;
            }

            if (num == 0) {
              break;
            } else if (get_length <= 6) {
              if (num != 0) {
                res = RX_CORRUPT;
              }
              break;
            }
          } else {
            for (int j = 0; j < (get_length - i); j++) {
              rxpacket[j] = rxpacket[j + i];
            }
            get_length -= i;
          }
        }
      } else {
        res = SUCCESS;
      }
    } else {
      res = TX_FAIL;
    }
  } else {
    res = TX_CORRUPT;
  }

  if (DEBUG_PRINT == true) {
    fprintf(stderr, "Time:%.2fms  ", get_packet_time());
    fprintf(stderr, "RETURN: ");
    switch (res) {
      case SUCCESS:
        fprintf(stderr, "SUCCESS\n");
        break;

      case TX_CORRUPT:
        fprintf(stderr, "TX_CORRUPT\n");
        break;

      case TX_FAIL:
        fprintf(stderr, "TX_FAIL\n");
        break;

      case RX_FAIL:
        fprintf(stderr, "RX_FAIL\n");
        break;

      case RX_TIMEOUT:
        fprintf(stderr, "RX_TIMEOUT\n");
        break;

      case RX_CORRUPT:
        fprintf(stderr, "RX_CORRUPT\n");
        break;

      default:
        fprintf(stderr, "UNKNOWN\n");
        break;
    }
  }

  high_priority_release();
  if (priority > 0) {
    mid_priority_release();
  }
  if (priority > 1) {
    low_priority_release();
  }

  return res;
}

unsigned char CM740::calculate_checksum(unsigned char * packet)
{
  unsigned char checksum = 0x00;
  for (int i = 2; i < packet[INDEX_LENGTH] + 3; i++) {
    checksum += packet[i];
  }
  return ~checksum;
}

void CM740::make_bulk_read_packet()
{
  int number = 0;

  m_bulk_readTxPacket[INDEX_ID] = (unsigned char)ID_BROADCAST;
  m_bulk_readTxPacket[INDEX_INSTRUCTION] = INST_BULK_READ;
  m_bulk_readTxPacket[INDEX_PARAMETER] = (unsigned char)0x0;

  if (ping(ID_CM, 0) == SUCCESS) {
    m_bulk_readTxPacket[INDEX_PARAMETER + 3 * number + 1] = 30;
    m_bulk_readTxPacket[INDEX_PARAMETER + 3 * number + 2] = ID_CM;
    m_bulk_readTxPacket[INDEX_PARAMETER + 3 * number + 3] = CM_DXL_POWER;
    number++;
  }

  // for(int id = 1; id < JointData::NUMBER_OF_JOINTS; id++)
  // {
  //   if(MotionStatus::m_CurrentJoints.GetEnable(id))
  //   {
  //     m_bulk_readTxPacket[INDEX_PARAMETER+3*number+1] = 2;  // length
  //     m_bulk_readTxPacket[INDEX_PARAMETER+3*number+2] = id;  // id
  //     m_bulk_readTxPacket[INDEX_PARAMETER+3*number+3] = MX28::P_PRESENT_POSITION_L;
  //     number++;
  //   }
  // }

  if (ping(FSR::ID_L_FSR, 0) == SUCCESS) {
    printf("CONNECTED TO Left FSR");
    m_bulk_readTxPacket[INDEX_PARAMETER + 3 * number + 1] = 10;  // length
    m_bulk_readTxPacket[INDEX_PARAMETER + 3 * number + 2] = FSR::ID_L_FSR;  // id
    m_bulk_readTxPacket[INDEX_PARAMETER + 3 * number + 3] = FSRAddress::FSR_FSR1_L;
    number++;
  }

  if (ping(FSR::ID_R_FSR, 0) == SUCCESS) {
    printf("CONNECTED TO right FSR");
    m_bulk_readTxPacket[INDEX_PARAMETER + 3 * number + 1] = 10;  // length
    m_bulk_readTxPacket[INDEX_PARAMETER + 3 * number + 2] = FSR::ID_R_FSR;  // id
    m_bulk_readTxPacket[INDEX_PARAMETER + 3 * number + 3] = FSRAddress::FSR_FSR1_L;
    number++;
  }

  m_bulk_readTxPacket[INDEX_LENGTH] = (number * 3) + 3;
}

int CM740::bulk_read()
{
  unsigned char rxpacket[MAXNUM_RXPARAM + 10] = {
    0,
  };

  if (m_bulk_readTxPacket[INDEX_LENGTH] != 0) {
    return txrx_packet(m_bulk_readTxPacket, rxpacket, 0);
  } else {
    make_bulk_read_packet();
    return TX_FAIL;
  }
}

int CM740::sync_write(int start_addr, int each_length, int number, int * pParam)
{
  unsigned char txpacket[MAXNUM_TXPARAM + 10] = {
    0,
  };
  unsigned char rxpacket[MAXNUM_RXPARAM + 10] = {
    0,
  };
  int n;

  txpacket[INDEX_ID] = (unsigned char)ID_BROADCAST;
  txpacket[INDEX_INSTRUCTION] = INST_SYNC_WRITE;
  txpacket[INDEX_PARAMETER] = (unsigned char)start_addr;
  txpacket[INDEX_PARAMETER + 1] = (unsigned char)(each_length - 1);
  for (n = 0; n < (number * each_length); n++) {
    txpacket[INDEX_PARAMETER + 2 + n] = (unsigned char)pParam[n];
  }
  txpacket[INDEX_LENGTH] = n + 4;

  return txrx_packet(txpacket, rxpacket, 0);
}

bool CM740::connect()
{
  // disconnect();
  if (open_port() == false) {
    return false;
  }

  return dxl_power_on();
}

bool CM740::dxl_power_on()
{
  if (write_byte(ID_CM, CM_DXL_POWER, 1, 0) == SUCCESS) {
    if (DEBUG_PRINT == true) {
      fprintf(stderr, " Succeed to change Dynamixel power!\n");
    }

    write_word(ID_CM, CM_LED_HEAD_L, make_color(255, 128, 0), 0);
    std::this_thread::sleep_for(300ms);  // about 300msec
  } else {
    if (DEBUG_PRINT == true) {
      fprintf(stderr, " Fail to change Dynamixel power!\n");
    }
    return false;
  }

  return true;
}

void CM740::disconnect()
{
  // Make the Head LED to green
  // write_word(ID_CM, CM_LED_HEAD_L, make_color(0, 255, 0), 0);
  fprintf(stderr, "\n Close Port\n");
  unsigned char txpacket[] = {0xFF, 0xFF, 0xC8, 0x05, 0x03, 0x1A, 0xE0, 0x03, 0x32};
  write_port(txpacket, 9);

  close_port();
}

int CM740::write_byte(int address, int value, int * error)
{
  return write_byte(ID_CM, address, value, error);
}

int CM740::write_word(int address, int value, int * error)
{
  return write_word(ID_CM, address, value, error);
}

int CM740::ping(int id, int * error)
{
  unsigned char txpacket[MAXNUM_TXPARAM + 10] = {
    0,
  };
  unsigned char rxpacket[MAXNUM_RXPARAM + 10] = {
    0,
  };
  int result;

  txpacket[INDEX_ID] = (unsigned char)id;
  txpacket[INDEX_INSTRUCTION] = INST_PING;
  txpacket[INDEX_LENGTH] = 2;

  result = txrx_packet(txpacket, rxpacket, 2);
  if (result == SUCCESS && txpacket[INDEX_ID] != ID_BROADCAST) {
    if (error != 0) {
      *error = static_cast<int>(rxpacket[INDEX_ERRBIT]);
    }
  }

  return result;
}

int CM740::read_byte(int id, int address, int * pValue, int * error)
{
  unsigned char txpacket[MAXNUM_TXPARAM + 10] = {
    0,
  };
  unsigned char rxpacket[MAXNUM_RXPARAM + 10] = {
    0,
  };
  int result;

  txpacket[INDEX_ID] = (unsigned char)id;
  txpacket[INDEX_INSTRUCTION] = INST_READ;
  txpacket[INDEX_PARAMETER] = (unsigned char)address;
  txpacket[INDEX_PARAMETER + 1] = 1;
  txpacket[INDEX_LENGTH] = 4;

  result = txrx_packet(txpacket, rxpacket, 2);
  if (result == SUCCESS) {
    *pValue = static_cast<int>(rxpacket[INDEX_PARAMETER]);
    if (error != 0) {
      *error = static_cast<int>(rxpacket[INDEX_ERRBIT]);
    }
  }

  return result;
}

int CM740::read_word(int id, int address, int * pValue, int * error)
{
  unsigned char txpacket[MAXNUM_TXPARAM + 10] = {
    0,
  };
  unsigned char rxpacket[MAXNUM_RXPARAM + 10] = {
    0,
  };
  int result;

  txpacket[INDEX_ID] = (unsigned char)id;
  txpacket[INDEX_INSTRUCTION] = INST_READ;
  txpacket[INDEX_PARAMETER] = (unsigned char)address;
  txpacket[INDEX_PARAMETER + 1] = 2;
  txpacket[INDEX_LENGTH] = 4;

  result = txrx_packet(txpacket, rxpacket, 2);
  if (result == SUCCESS) {
    *pValue =
      make_word(
      static_cast<int>(rxpacket[INDEX_PARAMETER]),
      static_cast<int>(rxpacket[INDEX_PARAMETER + 1]));

    if (error != 0) {
      *error = static_cast<int>(rxpacket[INDEX_ERRBIT]);
    }
  }

  return result;
}

int CM740::read_table(int id, int start_addr, int end_addr, unsigned char * table, int * error)
{
  unsigned char txpacket[MAXNUM_TXPARAM + 10] = {
    0,
  };
  unsigned char rxpacket[MAXNUM_RXPARAM + 10] = {
    0,
  };
  int result;
  int length = end_addr - start_addr + 1;

  txpacket[INDEX_ID] = (unsigned char)id;
  txpacket[INDEX_INSTRUCTION] = INST_READ;
  txpacket[INDEX_PARAMETER] = (unsigned char)start_addr;
  txpacket[INDEX_PARAMETER + 1] = (unsigned char)length;
  txpacket[INDEX_LENGTH] = 4;

  result = txrx_packet(txpacket, rxpacket, 1);
  if (result == SUCCESS) {
    for (int i = 0; i < length; i++) {
      table[start_addr + i] = rxpacket[INDEX_PARAMETER + i];
    }

    if (error != 0) {
      *error = static_cast<int>(rxpacket[INDEX_ERRBIT]);
    }
  }

  return result;
}

int CM740::write_byte(int id, int address, int value, int * error)
{
  unsigned char txpacket[MAXNUM_TXPARAM + 10] = {
    0,
  };
  unsigned char rxpacket[MAXNUM_RXPARAM + 10] = {
    0,
  };
  int result;

  txpacket[INDEX_ID] = (unsigned char)id;
  txpacket[INDEX_INSTRUCTION] = INST_WRITE;
  txpacket[INDEX_PARAMETER] = (unsigned char)address;
  txpacket[INDEX_PARAMETER + 1] = (unsigned char)value;
  txpacket[INDEX_LENGTH] = 4;

  result = txrx_packet(txpacket, rxpacket, 2);
  if (result == SUCCESS && id != ID_BROADCAST) {
    if (error != 0) {
      *error = static_cast<int>(rxpacket[INDEX_ERRBIT]);
    }
  }

  return result;
}

int CM740::write_word(int id, int address, int value, int * error)
{
  unsigned char txpacket[MAXNUM_TXPARAM + 10] = {
    0,
  };
  unsigned char rxpacket[MAXNUM_RXPARAM + 10] = {
    0,
  };
  int result;

  txpacket[INDEX_ID] = (unsigned char)id;
  txpacket[INDEX_INSTRUCTION] = INST_WRITE;
  txpacket[INDEX_PARAMETER] = (unsigned char)address;
  txpacket[INDEX_PARAMETER + 1] = (unsigned char)get_low_byte(value);
  txpacket[INDEX_PARAMETER + 2] = (unsigned char)get_high_byte(value);
  txpacket[INDEX_LENGTH] = 5;

  result = txrx_packet(txpacket, rxpacket, 2);
  if (result == SUCCESS && id != ID_BROADCAST) {
    if (error != 0) {
      *error = static_cast<int>(rxpacket[INDEX_ERRBIT]);
    }
  }

  return result;
}

int CM740::make_word(int lowbyte, int highbyte)
{
  uint16_t word;

  word = highbyte;
  word = word << 8;
  word = word + lowbyte;

  return static_cast<int>(word);
}

int CM740::get_low_byte(int word)
{
  uint16_t temp;
  temp = word & 0x00FF;
  return static_cast<int>(temp);
}

int CM740::get_high_byte(int word)
{
  uint16_t temp;
  temp = word & 0xFF00;
  return static_cast<int>((temp >> 8));
}

int CM740::make_color(int red, int green, int blue)
{
  int r = (red & 0xFF) >> 3;
  int g = (green & 0xFF) >> 3;
  int b = (blue & 0xFF) >> 3;

  return static_cast<int>(((b << 10) | (g << 5) | r));
}

int CM740::get_read_byte(int color)
{
  uint16_t red = color << 3;
  return static_cast<int>((red & 0xFF));
}

int CM740::get_green_byte(int color)
{
  uint16_t green = (color >> 5) << 3;
  return static_cast<int>((green & 0xFF));
}

int CM740::get_blue_byte(int color)
{
  uint16_t blue = (color >> 10) << 3;
  return static_cast<int>((blue & 0xFF));
}

// connection
void CM740::set_port_name(const std::string & port_name)
{
  m_PortName = port_name;
}

void CM740::set_baudrate(int baudrate)
{
  this->baudrate = baudrate;
}

bool CM740::open_port()
{
  struct termios newtio;
  struct serial_struct serinfo;

  close_port();

  if (DEBUG_PRINT == true) {
    printf("\n%s open ", m_PortName);
  }

  if ((m_Socket_fd = open(m_PortName.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK)) < 0) {
    if (DEBUG_PRINT == true) {
      printf("failed! %d %d %d %d %d\n", errno, EBADF, EFAULT, EINVAL, ENOTTY);
    }
    close_port();
    return false;
  }

  if (DEBUG_PRINT == true) {
    printf("success!\n");
  }

  // You must set 38400bps!
  memset(&newtio, 0, sizeof(newtio));
  newtio.c_cflag = B38400 | CS8 | CLOCAL | CREAD;
  newtio.c_iflag = IGNPAR;
  newtio.c_oflag = 0;
  newtio.c_lflag = 0;
  newtio.c_cc[VTIME] = 0;
  newtio.c_cc[VMIN] = 0;
  tcsetattr(m_Socket_fd, TCSANOW, &newtio);

  if (DEBUG_PRINT == true) {
    printf("Set %.1fbps ", baudrate);
  }

  // Set non-standard baudrate
  if (ioctl(m_Socket_fd, TIOCGSERIAL, &serinfo) < 0) {
    if (DEBUG_PRINT == true) {
      printf("failed! %d %d %d %d %d\n", errno, EBADF, EFAULT, EINVAL, ENOTTY);
    }
    close_port();
    return false;
  }

  serinfo.flags &= ~ASYNC_SPD_MASK;
  serinfo.flags |= ASYNC_SPD_CUST;
  serinfo.flags |= ASYNC_LOW_LATENCY;
  serinfo.custom_divisor = serinfo.baud_base / baudrate;

  if (int res = ioctl(m_Socket_fd, TIOCSSERIAL, &serinfo) < 0) {
    if (DEBUG_PRINT == true) {
      printf("failed! %d %d %d %d %d\n", errno, EBADF, EFAULT, EINVAL, ENOTTY);
    }
    close_port();
    return false;
  }

  if (DEBUG_PRINT == true) {
    printf("success!\n");
  }

  tcflush(m_Socket_fd, TCIFLUSH);

  m_ByteTransferTime = (1000.0 / baudrate) * 12.0;

  return true;
}

void CM740::close_port()
{
  if (m_Socket_fd != -1) {
    close(m_Socket_fd);
  }
  m_Socket_fd = -1;
}

void CM740::clear_port()
{
  tcflush(m_Socket_fd, TCIFLUSH);
}

int CM740::write_port(unsigned char * packet, int numPacket)
{
  return write(m_Socket_fd, packet, numPacket);
}

int CM740::read_port(unsigned char * packet, int numPacket)
{
  return read(m_Socket_fd, packet, numPacket);
}

void sem_wait_nointr(sem_t * sem)
{
  int sem_result, sem_count = 0;
  do {
    sem_result = sem_wait(sem);
  } while ((sem_result == -1) && (errno == EINTR));
}

void CM740::low_priority_wait()
{
  sem_wait_nointr(&m_LowSemID);
}

void CM740::mid_priority_wait()
{
  sem_wait_nointr(&m_MidSemID);
}

void CM740::high_priority_wait()
{
  sem_wait_nointr(&m_HighSemID);
}

void CM740::low_priority_release()
{
  sem_post(&m_LowSemID);
}

void CM740::mid_priority_release()
{
  sem_post(&m_MidSemID);
}

void CM740::high_priority_release()
{
  sem_post(&m_HighSemID);
}

double CM740::get_current_time()
{
  struct timeval tv;
  gettimeofday(&tv, NULL);

  return static_cast<double>(tv.tv_sec) * 1000.0 + static_cast<double>(tv.tv_usec) / 1000.0;
}

void CM740::set_packet_timeout(int lenPacket)
{
  m_PacketStartTime = get_current_time();
  m_PacketWaitTime = m_ByteTransferTime * static_cast<double>(lenPacket) + 40.0;
}

bool CM740::is_packet_timeout()
{
  if (get_packet_time() > m_PacketWaitTime) {
    return true;
  }

  return false;
}

double CM740::get_packet_time()
{
  double time;

  time = get_current_time() - m_PacketStartTime;
  if (time < 0.0) {
    m_PacketStartTime = get_current_time();
  }

  return time;
}

}  // namespace tachimawari_dynamixel
