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

#include <stdio.h>

#include <tachimawari_dynamixel/cm740.hpp>

#include "controller/fsr.h"
#include "controller/cm730.h"
#include "motion/joint_data.h"
#include "motion/motion_status.h"

namespace tachimawari_dynamixel
{

CM740::CM740(PlatformCM740 *platform)
{
	m_Platform = platform;
	DEBUG_PRINT = false;
	for (int i = 0; i < ID_BROADCAST; i++)
		m_bulk_readData[i] = bulk_readData();
}

CM740::~CM740()
{
	disconnect();
}

int CM740::txrx_packet(unsigned char *txpacket, unsigned char *rxpacket, int priority)
{
	if (priority > 1)
		m_Platform->LowPriorityWait();
	if (priority > 0)
		m_Platform->MidPriorityWait();
	m_Platform->HighPriorityWait();

	int res = TX_FAIL;
	int length = txpacket[LENGTH] + 4;

	txpacket[0] = 0xFF;
	txpacket[1] = 0xFF;
	txpacket[length - 1] = calculate_checksum(txpacket);

	if (DEBUG_PRINT == true)
	{
		fprintf(stderr, "\nTX: ");
		for (int n = 0; n < length; n++)
			fprintf(stderr, "%.2X ", txpacket[n]);

		fprintf(stderr, "INST: ");
		switch (txpacket[INSTRUCTION])
		{
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

	if (length < (MAXNUM_TXPARAM + 6))
	{
		m_Platform->ClearPort();
		if (m_Platform->WritePort(txpacket, length) == length)
		{
			if (txpacket[ID] != ID_BROADCAST)
			{
				int to_length = 0;

				if (txpacket[INSTRUCTION] == INST_READ)
					to_length = txpacket[PARAMETER + 1] + 6;
				else
					to_length = 6;

				m_Platform->SetPacketTimeout(length);

				int get_length = 0;
				if (DEBUG_PRINT == true)
					fprintf(stderr, "RX: ");

				while (1)
				{
					length = m_Platform->ReadPort(&rxpacket[get_length], to_length - get_length);
					if (DEBUG_PRINT == true)
					{
						for (int n = 0; n < length; n++)
							fprintf(stderr, "%.2X ", rxpacket[get_length + n]);
					}
					get_length += length;

					if (get_length == to_length)
					{
						// Find packet header
						int i;
						for (i = 0; i < (get_length - 1); i++)
						{
							if (rxpacket[i] == 0xFF && rxpacket[i + 1] == 0xFF)
								break;
							else if (i == (get_length - 2) && rxpacket[get_length - 1] == 0xFF)
								break;
						}

						if (i == 0)
						{
							// Check checksum
							unsigned char checksum = calculate_checksum(rxpacket);
							if (DEBUG_PRINT == true)
								fprintf(stderr, "CHK:%.2X\n", checksum);

							if (rxpacket[get_length - 1] == checksum)
								res = SUCCESS;
							else
								res = RX_CORRUPT;

							break;
						}
						else
						{
							for (int j = 0; j < (get_length - i); j++)
								rxpacket[j] = rxpacket[j + i];
							get_length -= i;
						}
					}
					else
					{
						if (m_Platform->IsPacketTimeout() == true)
						{
							if (get_length == 0)
								res = RX_TIMEOUT;
							else
								res = RX_CORRUPT;

							break;
						}
					}
				}
			}
			else if (txpacket[INSTRUCTION] == INST_BULK_READ)
			{
				int to_length = 0;
				int num = (txpacket[LENGTH] - 3) / 3;

				for (int x = 0; x < num; x++)
				{
					int _id = txpacket[PARAMETER + (3 * x) + 2];
					int _len = txpacket[PARAMETER + (3 * x) + 1];
					int _addr = txpacket[PARAMETER + (3 * x) + 3];

					to_length += _len + 6;
					m_bulk_readData[_id].length = _len;
					m_bulk_readData[_id].start_address = _addr;
				}

				m_Platform->SetPacketTimeout(to_length * 1.5);

				int get_length = 0;
				if (DEBUG_PRINT == true)
					fprintf(stderr, "RX: ");

				while (1)
				{
					length = m_Platform->ReadPort(&rxpacket[get_length], to_length - get_length);
					if (DEBUG_PRINT == true)
					{
						for (int n = 0; n < length; n++)
							fprintf(stderr, "%.2X ", rxpacket[get_length + n]);
					}
					get_length += length;

					if (get_length == to_length)
					{
						res = SUCCESS;
						break;
					}
					else
					{
						if (m_Platform->IsPacketTimeout() == true)
						{
							if (get_length == 0)
								res = RX_TIMEOUT;
							else
								res = RX_CORRUPT;

							break;
						}
					}
				}

				for (int x = 0; x < num; x++)
				{
					int _id = txpacket[PARAMETER + (3 * x) + 2];
					m_bulk_readData[_id].error = -1;
				}

				while (1)
				{
					int i;
					for (i = 0; i < get_length - 1; i++)
					{
						if (rxpacket[i] == 0xFF && rxpacket[i + 1] == 0xFF)
							break;
						else if (i == (get_length - 2) && rxpacket[get_length - 1] == 0xFF)
							break;
					}

					if (i == 0)
					{
						// Check checksum
						unsigned char checksum = calculate_checksum(rxpacket);
						if (DEBUG_PRINT == true)
							fprintf(stderr, "CHK:%.2X\n", checksum);

						if (rxpacket[LENGTH + rxpacket[LENGTH]] == checksum)
						{
							for (int j = 0; j < (rxpacket[LENGTH] - 2); j++)
								m_bulk_readData[rxpacket[ID]].table[m_bulk_readData[rxpacket[ID]].start_address + j] = rxpacket[PARAMETER + j];

							m_bulk_readData[rxpacket[ID]].error = (int)rxpacket[ERRBIT];

							int cur_packet_length = LENGTH + 1 + rxpacket[LENGTH];
							to_length = get_length - cur_packet_length;
							for (int j = 0; j <= to_length; j++)
								rxpacket[j] = rxpacket[j + cur_packet_length];

							get_length = to_length;
							num--;
						}
						else
						{
							res = RX_CORRUPT;

							for (int j = 0; j <= get_length - 2; j++)
								rxpacket[j] = rxpacket[j + 2];

							to_length = get_length -= 2;
						}

						if (num == 0)
							break;
						else if (get_length <= 6)
						{
							if (num != 0)
								res = RX_CORRUPT;
							break;
						}
					}
					else
					{
						for (int j = 0; j < (get_length - i); j++)
							rxpacket[j] = rxpacket[j + i];
						get_length -= i;
					}
				}
			}
			else
				res = SUCCESS;
		}
		else
			res = TX_FAIL;
	}
	else
		res = TX_CORRUPT;

	if (DEBUG_PRINT == true)
	{
		fprintf(stderr, "Time:%.2fms  ", m_Platform->GetPacketTime());
		fprintf(stderr, "RETURN: ");
		switch (res)
		{
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

	m_Platform->HighPriorityRelease();
	if (priority > 0)
		m_Platform->MidPriorityRelease();
	if (priority > 1)
		m_Platform->LowPriorityRelease();

	return res;
}

unsigned char CM740::calculate_checksum(unsigned char *packet)
{
	unsigned char checksum = 0x00;
	for (int i = 2; i < packet[LENGTH] + 3; i++)
		checksum += packet[i];
	return (~checksum);
}

void CM740::make_bulk_read_packet()
{
	int number = 0;

	m_bulk_readTxPacket[ID] = (unsigned char)ID_BROADCAST;
	m_bulk_readTxPacket[INSTRUCTION] = INST_BULK_READ;
	m_bulk_readTxPacket[PARAMETER] = (unsigned char)0x0;

	if (ping(CM740::ID_CM, 0) == SUCCESS)
	{
		m_bulk_readTxPacket[PARAMETER + 3 * number + 1] = 30;
		m_bulk_readTxPacket[PARAMETER + 3 * number + 2] = CM740::ID_CM;
		m_bulk_readTxPacket[PARAMETER + 3 * number + 3] = CM740::P_DXL_POWER;
		number++;
	}

	//    for(int id = 1; id < JointData::NUMBER_OF_JOINTS; id++)
	//    {
	//        if(MotionStatus::m_CurrentJoints.GetEnable(id))
	//        {
	//            m_bulk_readTxPacket[PARAMETER+3*number+1] = 2;   // length
	//            m_bulk_readTxPacket[PARAMETER+3*number+2] = id;  // id
	//            m_bulk_readTxPacket[PARAMETER+3*number+3] = MX28::P_PRESENT_POSITION_L; // start address
	//            number++;
	//        }
	//    }

	if (ping(FSR::ID_L_FSR, 0) == SUCCESS)
	{
		printf("CONNECTED TO Left FSR");
		m_bulk_readTxPacket[PARAMETER + 3 * number + 1] = 10;						// length
		m_bulk_readTxPacket[PARAMETER + 3 * number + 2] = FSR::ID_L_FSR; // id
		m_bulk_readTxPacket[PARAMETER + 3 * number + 3] = FSR::P_FSR1_L; // start address
		number++;
	}

	if (ping(FSR::ID_R_FSR, 0) == SUCCESS)
	{
		printf("CONNECTED TO right FSR");
		m_bulk_readTxPacket[PARAMETER + 3 * number + 1] = 10;						// length
		m_bulk_readTxPacket[PARAMETER + 3 * number + 2] = FSR::ID_R_FSR; // id
		m_bulk_readTxPacket[PARAMETER + 3 * number + 3] = FSR::P_FSR1_L; // start address
		number++;
	}

	m_bulk_readTxPacket[LENGTH] = (number * 3) + 3;
}

int CM740::bulk_read()
{
	unsigned char rxpacket[MAXNUM_RXPARAM + 10] = {
			0,
	};

	if (m_bulk_readTxPacket[LENGTH] != 0)
	{
		return txrx_packet(m_bulk_readTxPacket, rxpacket, 0);
	}
	else
	{
		make_bulk_read_packet();
		return TX_FAIL;
	}
}

int CM740::sync_write(int start_addr, int each_length, int number, int *pParam)
{
	unsigned char txpacket[MAXNUM_TXPARAM + 10] = {
			0,
	};
	unsigned char rxpacket[MAXNUM_RXPARAM + 10] = {
			0,
	};
	int n;

	txpacket[ID] = (unsigned char)ID_BROADCAST;
	txpacket[INSTRUCTION] = INST_SYNC_WRITE;
	txpacket[PARAMETER] = (unsigned char)start_addr;
	txpacket[PARAMETER + 1] = (unsigned char)(each_length - 1);
	for (n = 0; n < (number * each_length); n++)
		txpacket[PARAMETER + 2 + n] = (unsigned char)pParam[n];
	txpacket[LENGTH] = n + 4;

	return txrx_packet(txpacket, rxpacket, 0);
}

bool CM740::connect()
{
	// disconnect();
	if (m_Platform->OpenPort() == false)
	{
		return false;
	}

	return dxl_power_on();
}

bool CM740::change_baud(int baud)
{
	printf("baud\n");
	if (m_Platform->SetBaud(baud) == false)
	{
		fprintf(stderr, "\n Fail to change baudrate\n");
		return false;
	}

	return dxl_power_on();
}

bool CM740::dxl_power_on()
{
	if (write_byte(CM740::ID_CM, CM740::P_DXL_POWER, 1, 0) == CM740::SUCCESS)
	{
		if (DEBUG_PRINT == true)
			fprintf(stderr, " Succeed to change Dynamixel power!\n");

		write_word(CM740::ID_CM, CM740::P_LED_HEAD_L, make_color(255, 128, 0), 0);
		m_Platform->Sleep(300); // about 300msec
	}
	else
	{
		if (DEBUG_PRINT == true)
			fprintf(stderr, " Fail to change Dynamixel power!\n");
		return false;
	}

	return true;
}

void CM740::disconnect()
{
	// Make the Head LED to green
	//write_word(CM740::ID_CM, CM740::P_LED_HEAD_L, make_color(0, 255, 0), 0);
	fprintf(stderr, "\n Close Port\n");
	unsigned char txpacket[] = {0xFF, 0xFF, 0xC8, 0x05, 0x03, 0x1A, 0xE0, 0x03, 0x32};
	m_Platform->WritePort(txpacket, 9);

	m_Platform->ClosePort();
}

int CM740::write_byte(int address, int value, int *error)
{
	return write_byte(ID_CM, address, value, error);
}

int CM740::write_word(int address, int value, int *error)
{
	return write_word(ID_CM, address, value, error);
}

int CM740::ping(int id, int *error)
{
	unsigned char txpacket[MAXNUM_TXPARAM + 10] = {
			0,
	};
	unsigned char rxpacket[MAXNUM_RXPARAM + 10] = {
			0,
	};
	int result;

	txpacket[ID] = (unsigned char)id;
	txpacket[INSTRUCTION] = INST_PING;
	txpacket[LENGTH] = 2;

	result = txrx_packet(txpacket, rxpacket, 2);
	if (result == SUCCESS && txpacket[ID] != ID_BROADCAST)
	{
		if (error != 0)
			*error = (int)rxpacket[ERRBIT];
	}

	return result;
}

int CM740::read_byte(int id, int address, int *pValue, int *error)
{
	unsigned char txpacket[MAXNUM_TXPARAM + 10] = {
			0,
	};
	unsigned char rxpacket[MAXNUM_RXPARAM + 10] = {
			0,
	};
	int result;

	txpacket[ID] = (unsigned char)id;
	txpacket[INSTRUCTION] = INST_READ;
	txpacket[PARAMETER] = (unsigned char)address;
	txpacket[PARAMETER + 1] = 1;
	txpacket[LENGTH] = 4;

	result = txrx_packet(txpacket, rxpacket, 2);
	if (result == SUCCESS)
	{
		*pValue = (int)rxpacket[PARAMETER];
		if (error != 0)
			*error = (int)rxpacket[ERRBIT];
	}

	return result;
}

int CM740::read_word(int id, int address, int *pValue, int *error)
{
	unsigned char txpacket[MAXNUM_TXPARAM + 10] = {
			0,
	};
	unsigned char rxpacket[MAXNUM_RXPARAM + 10] = {
			0,
	};
	int result;

	txpacket[ID] = (unsigned char)id;
	txpacket[INSTRUCTION] = INST_READ;
	txpacket[PARAMETER] = (unsigned char)address;
	txpacket[PARAMETER + 1] = 2;
	txpacket[LENGTH] = 4;

	result = txrx_packet(txpacket, rxpacket, 2);
	if (result == SUCCESS)
	{
		*pValue = make_word((int)rxpacket[PARAMETER], (int)rxpacket[PARAMETER + 1]);

		if (error != 0)
			*error = (int)rxpacket[ERRBIT];
	}

	return result;
}

int CM740::read_table(int id, int start_addr, int end_addr, unsigned char *table, int *error)
{
	unsigned char txpacket[MAXNUM_TXPARAM + 10] = {
			0,
	};
	unsigned char rxpacket[MAXNUM_RXPARAM + 10] = {
			0,
	};
	int result;
	int length = end_addr - start_addr + 1;

	txpacket[ID] = (unsigned char)id;
	txpacket[INSTRUCTION] = INST_READ;
	txpacket[PARAMETER] = (unsigned char)start_addr;
	txpacket[PARAMETER + 1] = (unsigned char)length;
	txpacket[LENGTH] = 4;

	result = txrx_packet(txpacket, rxpacket, 1);
	if (result == SUCCESS)
	{
		for (int i = 0; i < length; i++)
			table[start_addr + i] = rxpacket[PARAMETER + i];

		if (error != 0)
			*error = (int)rxpacket[ERRBIT];
	}

	return result;
}

int CM740::write_byte(int id, int address, int value, int *error)
{
	unsigned char txpacket[MAXNUM_TXPARAM + 10] = {
			0,
	};
	unsigned char rxpacket[MAXNUM_RXPARAM + 10] = {
			0,
	};
	int result;

	txpacket[ID] = (unsigned char)id;
	txpacket[INSTRUCTION] = INST_WRITE;
	txpacket[PARAMETER] = (unsigned char)address;
	txpacket[PARAMETER + 1] = (unsigned char)value;
	txpacket[LENGTH] = 4;

	result = txrx_packet(txpacket, rxpacket, 2);
	if (result == SUCCESS && id != ID_BROADCAST)
	{
		if (error != 0)
			*error = (int)rxpacket[ERRBIT];
	}

	return result;
}

int CM740::write_word(int id, int address, int value, int *error)
{
	unsigned char txpacket[MAXNUM_TXPARAM + 10] = {
			0,
	};
	unsigned char rxpacket[MAXNUM_RXPARAM + 10] = {
			0,
	};
	int result;

	txpacket[ID] = (unsigned char)id;
	txpacket[INSTRUCTION] = INST_WRITE;
	txpacket[PARAMETER] = (unsigned char)address;
	txpacket[PARAMETER + 1] = (unsigned char)get_low_byte(value);
	txpacket[PARAMETER + 2] = (unsigned char)get_high_byte(value);
	txpacket[LENGTH] = 5;

	result = txrx_packet(txpacket, rxpacket, 2);
	if (result == SUCCESS && id != ID_BROADCAST)
	{
		if (error != 0)
			*error = (int)rxpacket[ERRBIT];
	}

	return result;
}

int CM740::make_word(int lowbyte, int highbyte)
{
	unsigned short word;

	word = highbyte;
	word = word << 8;
	word = word + lowbyte;

	return (int)word;
}

int CM740::get_low_byte(int word)
{
	unsigned short temp;
	temp = word & 0x00FF;
	return (int)temp;
}

int CM740::get_high_byte(int word)
{
	unsigned short temp;
	temp = word & 0xFF00;
	return (int)(temp >> 8);
}

int CM740::make_color(int red, int green, int blue)
{
	int r = (red & 0xFF) >> 3;
	int g = (green & 0xFF) >> 3;
	int b = (blue & 0xFF) >> 3;

	return (int)((b << 10) | (g << 5) | r);
}

int CM740::get_read_byte(int color)
{
	unsigned short red = color << 3;
	return (int)(red & 0xFF);
}

int CM740::get_green_byte(int color)
{
	unsigned short green = (color >> 5) << 3;
	return (int)(green & 0xFF);
}

int CM740::get_blue_byte(int color)
{
	unsigned short blue = (color >> 10) << 3;
	return (int)(blue & 0xFF);
}

}  // namespace tachimawari_dynamixel
