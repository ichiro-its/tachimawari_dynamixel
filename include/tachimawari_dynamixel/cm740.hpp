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
  ID                   = 2,
  LENGTH               = 3,
  INSTRUCTION          = 4,
  ERRBIT               = 4,
  PARAMETER            = 5,
  DEFAULT_BAUDNUMBER   = 1
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
  P_MODEL_NUMBER_L    = 0,
  P_MODEL_NUMBER_H    = 1,
  P_VERSION           = 2,
  P_ID                = 3,
  P_BAUD_RATE         = 4,
  P_RETURN_DELAY_TIME = 5,
  P_RETURN_LEVEL      = 16,
  P_DXL_POWER         = 24,
  P_LED_PANNEL        = 25,
  P_LED_HEAD_L        = 26,
  P_LED_HEAD_H        = 27,
  P_LED_EYE_L         = 28,
  P_LED_EYE_H         = 29,
  P_BUTTON            = 30,
  P_GYRO_Z_L          = 38,
  P_GYRO_Z_H          = 39,
  P_GYRO_Y_L          = 40,
  P_GYRO_Y_H          = 41,
  P_GYRO_X_L          = 42,
  P_GYRO_X_H          = 43,
  P_ACCEL_X_L         = 44,
  P_ACCEL_X_H         = 45,
  P_ACCEL_Y_L         = 46,
  P_ACCEL_Y_H         = 47,
  P_ACCEL_Z_L         = 48,
  P_ACCEL_Z_H         = 49,
  P_VOLTAGE           = 50,
  P_LEFT_MIC_L        = 51,
  P_LEFT_MIC_H        = 52,
  P_ADC2_L            = 53,
  P_ADC2_H            = 54,
  P_ADC3_L            = 55,
  P_ADC3_H            = 56,
  P_ADC4_L            = 57,
  P_ADC4_H            = 58,
  P_ADC5_L            = 59,
  P_ADC5_H            = 60,
  P_ADC6_L            = 61,
  P_ADC6_H            = 62,
  P_ADC7_L            = 63,
  P_ADC7_H            = 64,
  P_ADC8_L            = 65,
  P_ADC8_H            = 66,
  P_RIGHT_MIC_L       = 67,
  P_RIGHT_MIC_H       = 68,
  P_ADC10_L           = 69,
  P_ADC10_H           = 70,
  P_ADC11_L           = 71,
  P_ADC11_H           = 72,
  P_ADC12_L           = 73,
  P_ADC12_H           = 74,
  P_ADC13_L           = 75,
  P_ADC13_H           = 76,
  P_ADC14_L           = 77,
  P_ADC14_H           = 78,
  P_ADC15_L           = 79,
  P_ADC15_H           = 80,
  MAXNUM_ADDRESS
};

enum CM
{
  ID_CM         = 200,
  ID_BROADCAST  = 254
};

enum MXP1Address : uint8_t
{
  // EEPROM Area
  P_MODEL_NUMBER_L            = 0,
  P_MODEL_NUMBER_H            = 1,
  P_VERSION                   = 2,
  P_ID                        = 3,
  P_BAUD_RATE                 = 4,
  P_RETURN_DELAY_TIME         = 5,
  P_CW_ANGLE_LIMIT_L          = 6,
  P_CW_ANGLE_LIMIT_H          = 7,
  P_CCW_ANGLE_LIMIT_L         = 8,
  P_CCW_ANGLE_LIMIT_H         = 9,
  P_SYSTEM_DATA2              = 10,
  P_HIGH_LIMIT_TEMPERATURE    = 11,
  P_LOW_LIMIT_VOLTAGE         = 12,
  P_HIGH_LIMIT_VOLTAGE        = 13,
  P_MAX_TORQUE_L              = 14,
  P_MAX_TORQUE_H              = 15,
  P_RETURN_LEVEL              = 16,
  P_ALARM_LED                 = 17,
  P_ALARM_SHUTDOWN            = 18,
  P_OPERATING_MODE            = 19,
  P_LOW_CALIBRATION_L         = 20,
  P_LOW_CALIBRATION_H         = 21,
  P_HIGH_CALIBRATION_L        = 22,
  P_HIGH_CALIBRATION_H        = 23,

  // RAM Area
  P_TORQUE_ENABLE             = 24,
  P_LED                       = 25,
  P_D_GAIN                    = 26,
  P_I_GAIN                    = 27,
  P_P_GAIN                    = 28,
  P_RESERVED                  = 29,
  P_GOAL_POSITION_L           = 30,
  P_GOAL_POSITION_H           = 31,
  P_MOVING_SPEED_L            = 32,
  P_MOVING_SPEED_H            = 33,
  P_TORQUE_LIMIT_L            = 34,
  P_TORQUE_LIMIT_H            = 35,
  P_PRESENT_POSITION_L        = 36,
  P_PRESENT_POSITION_H        = 37,
  P_PRESENT_SPEED_L           = 38,
  P_PRESENT_SPEED_H           = 39,
  P_PRESENT_LOAD_L            = 40,
  P_PRESENT_LOAD_H            = 41,
  P_PRESENT_VOLTAGE           = 42,
  P_PRESENT_TEMPERATURE       = 43,
  P_REGISTERED_INSTRUCTION    = 44,
  P_PAUSE_TIME                = 45,
  P_MOVING                    = 46,
  P_LOCK                      = 47,
  P_PUNCH_L                   = 48,
  P_PUNCH_H                   = 49,
  P_RESERVED4                 = 50,
  P_RESERVED5                 = 51,
  P_POT_L                     = 52,
  P_POT_H                     = 53,
  P_PWM_OUT_L                 = 54,
  P_PWM_OUT_H                 = 55,
  P_P_ERROR_L                 = 56,
  P_P_ERROR_H                 = 57,
  P_I_ERROR_L                 = 58,
  P_I_ERROR_H                 = 59,
  P_D_ERROR_L                 = 60,
  P_D_ERROR_H                 = 61,
  P_P_ERROR_OUT_L             = 62,
  P_P_ERROR_OUT_H             = 63,
  P_I_ERROR_OUT_L             = 64,
  P_I_ERROR_OUT_H             = 65,
  P_D_ERROR_OUT_L             = 66,
  P_D_ERROR_OUT_H             = 67,
  MAXNUM_ADDRESS
};

enum FSR
{
  ID_R_FSR = 111,
  ID_L_FSR = 30
};

enum FSRAddress
{
  P_MODEL_NUMBER_L            = 0,
  P_MODEL_NUMBER_H            = 1,
  P_VERSION                   = 2,
  P_ID                        = 3,
  P_BAUD_RATE                 = 4,
  P_RETURN_DELAY_TIME         = 5,
  P_RETURN_LEVEL              = 16,
  P_OPERATING_MODE            = 19,
  P_LED                       = 25,
  P_FSR1_L                    = 36,
  P_FSR1_H                    = 37,
  P_FSR2_L                    = 38,
  P_FSR2_H                    = 39,
  P_FSR3_L                    = 30,
  P_FSR3_H                    = 31,
  P_FSR4_L                    = 32,
  P_FSR4_H                    = 33,
  P_FSR_X                     = 34,
  P_FSR_Y                     = 35,
  P_PRESENT_VOLTAGE           = 42,
  P_REGISTERED_INSTRUCTION    = 44,
  P_LOCK                      = 47,
  MAXNUM_ADDRESS
};

class CM740
{
public:
  struct BulkReadData
  {
    int start_address;
    int length;
    int error;
    unsigned char table[MXP1Address::MAXNUM_ADDRESS];

    BulkReadData();

    int read_byte(int address);
    int read_word(int address);
  };

  CM740(std::string port_name);
  ~CM740();

  bool connect();
  void disconnect();

  bool change_baud(int baud);
  bool dxl_power_on();

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

  bool open_port();
  bool set_baud(int baud);
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
  void set_update_timeout(int msec);
  bool is_update_timeout();
  double get_update_time();

  void sleep(double msec);

  bool DEBUG_PRINT;
  BulkReadData m_bulk_readData[ID_BROADCAST];

private:
  static const int RefreshTime = 6;  // ms
  unsigned char m_ControlTable[MAXNUM_ADDRESS];

  unsigned char m_bulk_readTxPacket[MAXNUM_TXPARAM + 10];

  int txrx_packet(unsigned char * txpacket, unsigned char * rxpacket, int priority);
  unsigned char calculate_checksum(unsigned char * packet);

  // connection
  int m_Socket_fd;
  double m_PacketStartTime;
  double m_PacketWaitTime;
  double m_UpdateStartTime;
  double m_UpdateWaitTime;
  double m_ByteTransferTime;
  std::string m_PortName;

  sem_t m_LowSemID;
  sem_t m_MidSemID;
  sem_t m_HighSemID;

  double get_current_time();
};

}  // namespace tachimawari_dynamixel

#endif  // TACHIMAWARI_DYNAMIXEL__CM740_HPP_
