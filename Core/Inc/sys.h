/*
 * sys.h
 *
 *  Created on: Jul 25, 2025
 *      Author: 46984
 */

#ifndef INC_SYS_H_
#define INC_SYS_H_

#include "main.h"

#define PARAM_BUFFER_SIZE 1000
#define TASK_FREQ_MAX 200
#define SYS_SCHEDULING_PERIOD 5
#define SYS_COUNT_MAX 1000

#define CALIBRATION_C_ROW	25
#define CALIBRATION_C_COL	8

//#define CALIBRATION_PARAM_SIZE		(CALIBRATION_C_ROW * CALIBRATION_C_COL + 16)		// 参数大小，以4字节计数
//#define CALIBRATION_PARAM_LENGTH	CALIBRATION_PARAM_SIZE * 4							// 参数长度，以1字节计数

#define CALIBRATION_PARAM_SIZE		(CALIBRATION_C_ROW * CALIBRATION_C_COL)		// 参数大小，以4字节计数
#define CALIBRATION_PARAM_LENGTH	CALIBRATION_PARAM_SIZE * 4							// 参数长度，以1字节计数

enum
{
	SAMPLING_RATE_1HZ = 0,
	SAMPLING_RATE_5HZ,
	SAMPLING_RATE_10HZ,
	SAMPLING_RATE_25HZ,
	SAMPLING_RATE_50HZ,
	SAMPLING_RATE_75HZ,
	SAMPLING_RATE_100HZ,
	SAMPLING_RATE_200HZ,
	SAMPLING_RATE_400HZ,
	SAMPLING_RATE_500HZ,
	SAMPLING_RATE_1000HZ,
};

enum
{
	UART_BPS_9600 = 0,
	UART_BPS_19200,
	UART_BPS_38400,
	UART_BPS_57600,
	UART_BPS_74800,
	UART_BPS_115200,
	UART_BPS_230400,
	UART_BPS_460800,
	UART_BPS_500000,
	UART_BPS_576000,
	UART_BPS_921600,
	UART_BPS_1000000,
};

enum
{
	RAW_DATA = 0,
	CALC_DATA,
};

typedef struct
{
	__IO uint32_t uart_bps;
	__IO uint16_t sampling_rate;
	__IO uint8_t update_status;
	__IO uint8_t data_type;
}system_info;

typedef struct
{
	__IO uint8_t uart_bps_id;
	__IO uint8_t sampling_rate_id;
	__IO uint8_t data_type;
}param_info;

typedef struct
{
//	float arg[8];
//	float std[8];
	float C[CALIBRATION_C_ROW * CALIBRATION_C_COL];
}calibration_param;

extern __IO uint8_t sys_task_flag;
extern __IO uint16_t sys_count;
extern uint32_t param_buffer[PARAM_BUFFER_SIZE];
extern system_info sys_data;
extern __IO uint16_t sys_scheduling_period;
extern param_info sys_param;
extern calibration_param calib_param;


void system_task_schedule();



#endif /* INC_SYS_H_ */
