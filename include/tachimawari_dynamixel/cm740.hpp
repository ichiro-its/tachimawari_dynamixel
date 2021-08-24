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

#ifndef TACHIMAWARI_DYNAMIXEL__CM740_HPP_
#define TACHIMAWARI_DYNAMIXEL__CM740_HPP_

#include <string>

#include "./semaphore.h"

namespace tachimawari_dynamixel
{

enum PacketStatus
{
  SUCCESS,
  TX_CORRUPT,
  TX_FAIL,
  RX_FAIL,
  RX_TIMEOUT,
  RX_CORRUPT
};

enum PacketSpecs
{
  INPUT_VOLTAGE   = 1,
  ANGLE_LIMIT     = 2,
  OVERHEATING     = 4,
  RANGE           = 8,
  CHECKSUM        = 16,
  OVERLOAD        = 32,
  INSTRUCTION     = 64,
  MAXNUM_TXPARAM  = 256,
  MAXNUM_RXPARAM  = 1024
};

enum PacketIndex
{
  INDEX_ID                   = 2,
  INDEX_LENGTH               = 3,
  INDEX_INSTRUCTION          = 4,
  INDEX_ERRBIT               = 4,
  INDEX_PARAMETER            = 5,
  INDEX_DEFAULT_BAUDNUMBER   = 1
};

enum PacketInstruction
{
  INST_PING        = 1,
  INST_READ        = 2,
  INST_WRITE       = 3,
  INST_REG_WRITE   = 4,
  INST_ACTION      = 5,
  INST_RESET       = 6,
  INST_SYNC_WRITE  = 131,
  INST_BULK_READ   = 146
};

enum Address
{
  CM_MODEL_NUMBER_L    = 0,
  CM_MODEL_NUMBER_H    = 1,
  CM_VERSION           = 2,
  CM_ID                = 3,
  CM_BAUD_RATE         = 4,
  CM_RETURN_DELAY_TIME = 5,
  CM_RETURN_LEVEL      = 16,
  CM_DXL_POWER         = 24,
  CM_LED_PANNEL        = 25,
  CM_LED_HEAD_L        = 26,
  CM_LED_HEAD_H        = 27,
  CM_LED_EYE_L         = 28,
  CM_LED_EYE_H         = 29,
  CM_BUTTON            = 30,
  CM_GYRO_Z_L          = 38,
  CM_GYRO_Z_H          = 39,
  CM_GYRO_Y_L          = 40,
  CM_GYRO_Y_H          = 41,
  CM_GYRO_X_L          = 42,
  CM_GYRO_X_H          = 43,
  CM_ACCEL_X_L         = 44,
  CM_ACCEL_X_H         = 45,
  CM_ACCEL_Y_L         = 46,
  CM_ACCEL_Y_H         = 47,
  CM_ACCEL_Z_L         = 48,
  CM_ACCEL_Z_H         = 49,
  CM_VOLTAGE           = 50,
  CM_LEFT_MIC_L        = 51,
  CM_LEFT_MIC_H        = 52,
  CM_ADC2_L            = 53,
  CM_ADC2_H            = 54,
  CM_ADC3_L            = 55,
  CM_ADC3_H            = 56,
  CM_ADC4_L            = 57,
  CM_ADC4_H            = 58,
  CM_ADC5_L            = 59,
  CM_ADC5_H            = 60,
  CM_ADC6_L            = 61,
  CM_ADC6_H            = 62,
  CM_ADC7_L            = 63,
  CM_ADC7_H            = 64,
  CM_ADC8_L            = 65,
  CM_ADC8_H            = 66,
  CM_RIGHT_MIC_L       = 67,
  CM_RIGHT_MIC_H       = 68,
  CM_ADC10_L           = 69,
  CM_ADC10_H           = 70,
  CM_ADC11_L           = 71,
  CM_ADC11_H           = 72,
  CM_ADC12_L           = 73,
  CM_ADC12_H           = 74,
  CM_ADC13_L           = 75,
  CM_ADC13_H           = 76,
  CM_ADC14_L           = 77,
  CM_ADC14_H           = 78,
  CM_ADC15_L           = 79,
  CM_ADC15_H           = 80,
  CM_MAXNUM_ADDRESS
};

enum CM
{
  ID_CM         = 200,
  ID_BROADCAST  = 254
};

enum MXP1Address : uint8_t
{
  // EEPROM Area
  MXP1_MODEL_NUMBER_L            = 0,
  MXP1_MODEL_NUMBER_H            = 1,
  MXP1_VERSION                   = 2,
  MXP1_ID                        = 3,
  MXP1_BAUD_RATE                 = 4,
  MXP1_RETURN_DELAY_TIME         = 5,
  MXP1_CW_ANGLE_LIMIT_L          = 6,
  MXP1_CW_ANGLE_LIMIT_H          = 7,
  MXP1_CCW_ANGLE_LIMIT_L         = 8,
  MXP1_CCW_ANGLE_LIMIT_H         = 9,
  MXP1_SYSTEM_DATA2              = 10,
  MXP1_HIGH_LIMIT_TEMPERATURE    = 11,
  MXP1_LOW_LIMIT_VOLTAGE         = 12,
  MXP1_HIGH_LIMIT_VOLTAGE        = 13,
  MXP1_MAX_TORQUE_L              = 14,
  MXP1_MAX_TORQUE_H              = 15,
  MXP1_RETURN_LEVEL              = 16,
  MXP1_ALARM_LED                 = 17,
  MXP1_ALARM_SHUTDOWN            = 18,
  MXP1_OPERATING_MODE            = 19,
  MXP1_LOW_CALIBRATION_L         = 20,
  MXP1_LOW_CALIBRATION_H         = 21,
  MXP1_HIGH_CALIBRATION_L        = 22,
  MXP1_HIGH_CALIBRATION_H        = 23,

  // RAM Area
  MXP1_TORQUE_ENABLE             = 24,
  MXP1_LED                       = 25,
  MXP1_D_GAIN                    = 26,
  MXP1_I_GAIN                    = 27,
  MXP1_P_GAIN                    = 28,
  MXP1_RESERVED                  = 29,
  MXP1_GOAL_POSITION_L           = 30,
  MXP1_GOAL_POSITION_H           = 31,
  MXP1_MOVING_SPEED_L            = 32,
  MXP1_MOVING_SPEED_H            = 33,
  MXP1_TORQUE_LIMIT_L            = 34,
  MXP1_TORQUE_LIMIT_H            = 35,
  MXP1_PRESENT_POSITION_L        = 36,
  MXP1_PRESENT_POSITION_H        = 37,
  MXP1_PRESENT_SPEED_L           = 38,
  MXP1_PRESENT_SPEED_H           = 39,
  MXP1_PRESENT_LOAD_L            = 40,
  MXP1_PRESENT_LOAD_H            = 41,
  MXP1_PRESENT_VOLTAGE           = 42,
  MXP1_PRESENT_TEMPERATURE       = 43,
  MXP1_REGISTERED_INSTRUCTION    = 44,
  MXP1_PAUSE_TIME                = 45,
  MXP1_MOVING                    = 46,
  MXP1_LOCK                      = 47,
  MXP1_PUNCH_L                   = 48,
  MXP1_PUNCH_H                   = 49,
  MXP1_RESERVED4                 = 50,
  MXP1_RESERVED5                 = 51,
  MXP1_POT_L                     = 52,
  MXP1_POT_H                     = 53,
  MXP1_PWM_OUT_L                 = 54,
  MXP1_PWM_OUT_H                 = 55,
  MXP1_P_ERROR_L                 = 56,
  MXP1_P_ERROR_H                 = 57,
  MXP1_I_ERROR_L                 = 58,
  MXP1_I_ERROR_H                 = 59,
  MXP1_D_ERROR_L                 = 60,
  MXP1_D_ERROR_H                 = 61,
  MXP1_P_ERROR_OUT_L             = 62,
  MXP1_P_ERROR_OUT_H             = 63,
  MXP1_I_ERROR_OUT_L             = 64,
  MXP1_I_ERROR_OUT_H             = 65,
  MXP1_D_ERROR_OUT_L             = 66,
  MXP1_D_ERROR_OUT_H             = 67,
  MXP1_MAXNUM_ADDRESS
};

enum FSR
{
  ID_R_FSR = 111,
  ID_L_FSR = 30
};

enum FSRAddress
{
  FSR_MODEL_NUMBER_L            = 0,
  FSR_MODEL_NUMBER_H            = 1,
  FSR_VERSION                   = 2,
  FSR_ID                        = 3,
  FSR_BAUD_RATE                 = 4,
  FSR_RETURN_DELAY_TIME         = 5,
  FSR_RETURN_LEVEL              = 16,
  FSR_OPERATING_MODE            = 19,
  FSR_LED                       = 25,
  FSR_FSR1_L                    = 36,
  FSR_FSR1_H                    = 37,
  FSR_FSR2_L                    = 38,
  FSR_FSR2_H                    = 39,
  FSR_FSR3_L                    = 30,
  FSR_FSR3_H                    = 31,
  FSR_FSR4_L                    = 32,
  FSR_FSR4_H                    = 33,
  FSR_FSR_X                     = 34,
  FSR_FSR_Y                     = 35,
  FSR_PRESENT_VOLTAGE           = 42,
  FSR_REGISTERED_INSTRUCTION    = 44,
  FSR_LOCK                      = 47,
  FSR_MAXNUM_ADDRESS
};

class CM740
{
public:
  struct BulkReadData
  {
    int start_address;
    int length;
    int error;
    unsigned char table[MXP1_MAXNUM_ADDRESS];

    BulkReadData();

    int read_byte(int address);
    int read_word(int address);
  };

  explicit CM740(std::string port_name);
  ~CM740();

  bool connect();
  bool dxl_power_on();
  void disconnect();

  int ping(int id, int * error);

  int read_byte(int id, int address, int * pValue, int * error);
  int read_word(int id, int address, int * pValue, int * error);
  int read_table(int id, int start_addr, int end_addr, unsigned char * table, int * error);

  int write_byte(int address, int value, int * error);
  int write_word(int address, int value, int * error);
  int write_byte(int id, int address, int value, int * error);
  int write_word(int id, int address, int value, int * error);
  int sync_write(int start_addr, int each_length, int number, int * pParam);

  int bulk_read();

  static int make_word(int lowbyte, int highbyte);
  static int make_color(int red, int green, int blue);
  void make_bulk_read_packet();

  static int get_low_byte(int word);
  static int get_high_byte(int word);
  static int get_read_byte(int color);
  static int get_green_byte(int color);
  static int get_blue_byte(int color);

  // connection
  void set_port_name(const std::string & port_name);
  const std::string & get_port_name() const {return m_PortName;}

  void set_baudrate(int baudrate);

  bool open_port();
  void close_port();
  void clear_port();
  int write_port(unsigned char * packet, int numPacket);
  int read_port(unsigned char * packet, int numPacket);

  void low_priority_wait();
  void mid_priority_wait();
  void high_priority_wait();
  void low_priority_release();
  void mid_priority_release();
  void high_priority_release();

  void set_packet_timeout(int lenPacket);
  bool is_packet_timeout();
  double get_packet_time();

  bool DEBUG_PRINT;
  BulkReadData m_bulk_readData[ID_BROADCAST];

private:
  static const int RefreshTime = 6;  // ms
  unsigned char m_ControlTable[CM_MAXNUM_ADDRESS];

  unsigned char m_bulk_readTxPacket[MAXNUM_TXPARAM + 10];

  int txrx_packet(unsigned char * txpacket, unsigned char * rxpacket, int priority);
  unsigned char calculate_checksum(unsigned char * packet);

  // connection
  int m_Socket_fd;
  double m_PacketStartTime;
  double m_PacketWaitTime;
  double m_ByteTransferTime;

  std::string m_PortName;
  int baudrate;

  sem_t m_LowSemID;
  sem_t m_MidSemID;
  sem_t m_HighSemID;

  double get_current_time();
};

}  // namespace tachimawari_dynamixel

#endif  // TACHIMAWARI_DYNAMIXEL__CM740_HPP_
