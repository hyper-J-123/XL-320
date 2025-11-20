/*
 * sys.c
 *
 *  Created on: Jul 25, 2025
 *      Author: 46984
 */

#include "sys.h"
#include "stdio.h"
#include "usart.h"
#include "stm_flash.h"
#include "uart_protocol.h"
#include "string.h"
#include "stdbool.h"
#include "main.h"


#define SYS_PARAM_SIZE 1


__IO uint16_t sys_count = 1;
__IO uint8_t sys_task_flag = false;
__IO uint8_t sys_task_enable = 0;
__IO uint16_t sys_scheduling_period = 10;

system_info sys_data = {
		.uart_bps = 115200,
		.sampling_rate = 100,
		.update_status = false,
		.data_type = RAW_DATA,
};

param_info sys_param;
calibration_param calib_param;


uint32_t param_buffer[PARAM_BUFFER_SIZE] = {0};


/**
 * @brief 任务设置函数
 *
 * @param p 	任务函数指针
 * @param freq	任务运行频率，1~200Hz
 */
static void sched_task(void (*p)(void), uint16_t freq)
{
	if((sys_count * sys_scheduling_period) % (SYS_COUNT_MAX / freq) == 0)
	{
		(*p)();
	}
}

/*
 * @brief 系统控制周期任务
 */
void system_task_schedule()
{
	if(sys_task_flag == true)
	{
		sched_task(Control9Servos,sys_data.sampling_rate);
		sched_task(Read9ServosAndSend_Protocol,sys_data.sampling_rate);
		sys_task_flag = false;
	}
}

void sys_uart_bps_init(uint32_t bps)
{
	huart1.Instance = USART1;
	huart1.Init.BaudRate = bps;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart1) != HAL_OK)
	{
	Error_Handler();
	}

	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
	HAL_UART_Receive_DMA(&huart1, RxBuf1, RX_MAX_BUF);
}

void set_sampling_rate(uint16_t freq)
{
	sys_data.sampling_rate = freq;
	sys_scheduling_period = 1000 / freq;



	sys_param_write();
}

void set_uart_bps(uint32_t bps)
{
	sys_data.uart_bps = bps;

	sys_param_write();
}

void sys_param_init()
{
	uint16_t param_size = sizeof(sys_param);

	STMFLASH_Read(ADDR_FLASH_PARAM_BASE, param_buffer, (param_size / 4) + ((param_size % 4) > 0));		// 按4字节对齐读取数据

	memcpy(&sys_param, param_buffer, sizeof(sys_param));

	sys_data.uart_bps = ID_to_uart_bps(sys_param.uart_bps_id);
	sys_data.sampling_rate = ID_to_sampling_rate(sys_param.sampling_rate_id);
	sys_data.data_type = range_judgement(sys_param.data_type, 0, 1) ? sys_param.data_type : RAW_DATA;
	sys_scheduling_period = SYS_COUNT_MAX / sys_data.sampling_rate;
}

void sys_param_write()
{
	sys_param.uart_bps_id = uart_bps_to_ID(sys_data.uart_bps);
	sys_param.sampling_rate_id = sampling_rate_to_ID(sys_data.sampling_rate);
	sys_param.data_type = sys_data.data_type;

	memcpy(param_buffer, &sys_param, sizeof(sys_param));

	STMFLASH_Write(ADDR_FLASH_PARAM_BASE, param_buffer, SYS_PARAM_SIZE);
}

void system_init()
{
	sys_param_init();
	sys_uart_bps_init(sys_data.uart_bps);
	HAL_Delay(5);

	sys_data.update_status = true;
}



