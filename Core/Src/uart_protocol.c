#include "uart_protocol.h"
#include "string.h"
#include "usart.h"
#include "main.h"
//#include "sys.h"


#define FRAME_HEAD 0xA8
#define SETPOSITION 0x01
/**
 * @brief 将接收到的4个uint8_t数据转换为float型数据
 *
 * @param data_start 数据开始地址
 * @return float
 */
float float_data_decode(uint8_t* data_start)
{
	float data;

	memcpy(&data, data_start, 4);

	return data;
}


void data_encode_v2(void* data, uint8_t* buf, uint8_t func, uint16_t size)
{
	uint8_t sum = 0;

	buf[0] = FRAME_HEAD;
	buf[1] = func;
	buf[2] = size & 0xff;
	buf[3] = (size & 0xff00) >> 8;
	memcpy(&buf[4], data, size);

	for(uint16_t i = 0; i < size + FRAME_OVERHEAD_LENGTH - 1; i++)
	{
		sum += buf[i];
	}
	buf[size + FRAME_OVERHEAD_LENGTH - 1] = sum;
}

//void sensor_data_encode_v2(sensor_data sdata, uint8_t* buf, uint8_t func, uint16_t size)
//{
//	uint8_t sum = 0;
//
//	buf[0] = FRAME_HEAD;
//	buf[1] = func;
//	buf[2] = size & 0xff;
//	buf[3] = (size >> 8) & 0xff;
//	buf[4] = sdata.sensor_type;
//	buf[5] = sdata.sensor_model;
//	buf[6] = sdata.id & 0xff;
//	buf[7] = (sdata.id >> 8) & 0xff;
//
//	memcpy(&buf[8], sdata.data, size - 4);
//
//	for(uint16_t i = 0; i < size + FRAME_OVERHEAD_LENGTH - 1; i++)
//	{
//		sum += buf[i];
//	}
//	buf[size + FRAME_OVERHEAD_LENGTH - 1] = sum;
//}

/**
 * @brief 数据解码
 *
 * @param buf 接收数据缓存
 * @param num 接收到的数据数量
 */
void data_decode_v2(uint8_t* buf, int16_t num)
{
	uint8_t sum;
	uint8_t* data_start = 0;    // data start pointer
	uint16_t data_length = 0;
	uint8_t func = 0;
//	float data = 0;
//	HAL_UART_Transmit(&huart1, RxBuf1, sizeof(RxBuf1), 100);
	while(num > 0)
	{
		if(buf[0] == FRAME_HEAD)
		{
			data_length = buf[2] + (buf[3] << 8);//小端模式
			if(num < FRAME_OVERHEAD_LENGTH + data_length)
				break;
			// check sum
			sum = 0;
			for(uint16_t i = 0; i < FRAME_OVERHEAD_LENGTH + data_length - 1; i++)
			{
				sum += buf[i];
			}
			sum &= 0xff;
			 //循环累加从帧头到数据部分最后一个字节的所有字节,最后一个字节是接收到的校验和，不参与计算
			if(sum == buf[FRAME_OVERHEAD_LENGTH + data_length - 1])
			{
				func = buf[1];
				data_start = &buf[4];

				if(data_length == 0)
				{
					// 没有数据部分，通常是查询或获取指令
				}
				else
				{
					// 带有数据部分，通常是设置或配置指令
					  // 1. 判断功能码是否匹配
					if(func == SETPOSITION)
					{
						// 2. 确认数据长度是否符合预期
						// 9个舵机 * (1字节ID + 2字节位置) = 27字节
						if(data_length == 27)
						{
//							uint8_t servo_id;
//							uint16_t servo_pos;
							// 循环解析9组数据
							for(int i = 0; i < 9; i++)
							{
								// 计算当前舵机数据的偏移量: 0, 3, 6, 9...
								int offset = i * 3;
								 ServoTargets[i].id = data_start[offset];
								 ServoTargets[i].position = data_start[offset + 1] + (data_start[offset + 2] << 8);

//								// 提取ID (1字节)
//								servo_id = data_start[offset];
//								// 提取位置 (2字节，小端模式)
//								// Low Byte 在前 (offset+1), High Byte 在后 (offset+2)
//								servo_pos = data_start[offset + 1] + (data_start[offset + 2] << 8);
//								// 调用驱动函数驱动物理舵机
//								xl320SendPosition(servo_id, servo_pos);
//								HAL_Delay(200);
							}
							NewDataAvailable = 1;
						}
					}

				}
			}
			// 将buf指针向后移动整个已处理过的帧的长度
			buf += FRAME_OVERHEAD_LENGTH + data_length;
			num -= FRAME_OVERHEAD_LENGTH + data_length;
		}
		else
		{
			buf++;
			num--;
		}
	}
}

void uart_protocol_decode()
{
	if(rx1_num != 0)
	{
		data_decode_v2(RxBuf1, rx1_num);
		rx1_num = 0;
	}
}


