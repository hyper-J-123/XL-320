/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// 舵机位置信息结构体
typedef struct {
    uint8_t id;
    uint16_t length;
    uint8_t inst;
    uint8_t error;
    uint32_t position;
    uint8_t crc1;
    uint8_t crc2;
} XPositionInfo;

typedef struct {
    uint8_t id;
    uint16_t length;
    uint8_t inst;
    uint8_t error;
    uint16_t position;
    uint8_t crc1;
    uint8_t crc2;
} X320PositionInfo;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/*                  		֡ͷ                  id      length       instruct    address     parameters               crc          */
uint8_t torqueFrame[]={  0xFF,0xFF,0xFD,0x00,  0x00,   0x06,0x00,   0x03,      0x40,0x00,   0x01,                   0x00,0x00};
uint8_t ledFrame[]={     0xFF,0xFF,0xFD,0x00,  0x00,   0x06,0x00,   0x03,      0x19,0x00,   0x01,                   0x00,0x00};
uint8_t ctrlModeFrame[]={0xFF,0xFF,0xFD,0x00,  0x00,   0x06,0x00,   0x03,      0x0B,0x00,   0x02,                   0x00,0x00};
uint8_t xl320PFrame[]={  0xFF,0xFF,0xFD,0x00,  0x00,   0x07,0x00,   0x03,      0x1e,0x00,   0x01,0x00,              0x65,0x6D};
uint8_t xl320MSFrame[]={ 0xFF,0xFF,0xFD,0x00,  0x00,   0x07,0x00,   0x03,      0x20,0x00,   0x01,0x00,              0x65,0x6D};
uint8_t xl320PGFrame[]={ 0xFF,0xFF,0xFD,0x00,  0x00,   0x06,0x00,   0x03,      0x1D,0x00,   0x20,                   0x65,0x6D};

uint8_t xl320PRead[]={   0xFF,0xFF,0xFD,0x00,  0x00,   0x07,0x00,   0x02,      0x25,0x00,   0x02,0x00,              0x21,0xB5};

X320PositionInfo *xl320_1=0;
uint8_t xl320_1_data[9];

// 添加读取位置指令帧
uint8_t xl320ReadGoalPosition[] = {0xFF,0xFF,0xFD,0x00,0x00,0x07,0x00,0x02,0x25,0x00,0x02,0x00,0x21,0xB5};

// 串口接收缓冲区
uint8_t uart2_rx_buffer[16];
volatile uint8_t position_received = 0;
uint16_t current_position = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
// XL系列舵机函数声明
void xlSeriesStart(void);
void xlSeriesControlMode(uint8_t id, uint8_t mode);
void xlSeriesSetDirection(uint8_t tx_mode);
void xlSeriesSendFrame(UART_HandleTypeDef *huart, uint8_t *frame, uint16_t length);
uint16_t updateCRC(uint16_t crc_accum, uint8_t *data_blk_ptr, uint16_t data_blk_size);
void xlSeriesLed(uint8_t id, uint8_t on, uint8_t address);
void xlSeriesTorque(uint8_t id, uint8_t on, uint8_t address);
void xl320SendPosition(uint8_t id, uint16_t position);
void xl320SendMovingSpeed(uint8_t id, uint16_t movingSpeed);
void xl320SendPGain(uint8_t id, uint8_t pGain);
void xl320ReadPosition(uint8_t id);
void xlPowerOff(uint8_t isOn);
// 新增函数声明
void xl320ReadPosition(uint8_t id);
uint8_t verifyPositionPacket(uint8_t *data);
uint16_t parsePositionValue(uint8_t *data);
void processPositionData(void);
void USART2_ReadCallback(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();

  /* USER CODE BEGIN 2 */
  // 初始化舵机系统
  xlSeriesStart();

  // 设置舵机到初始位置
  //xl320SendMovingSpeed(SERVO_ID, 100);
  xl320SendPosition(SERVO_ID, 510);
  xlSeriesLed(SERVO_ID, LED_CYAN, XL320Led);
  HAL_Delay(2000);
//  xl320SendPosition(SERVO_ID, 0);
//  xlSeriesLed(SERVO_ID, LED_PURPLE, XL320Led);
//  HAL_Delay(1000);

  // === 新增：读取位置并控制LED ===
  // 1. 发送读取位置指令
  xl320ReadPosition(SERVO_ID);

  // 2. 等待接收数据
  HAL_Delay(10); // 短暂等待数据接收

  // 3. 处理接收到的数据
  processPositionData();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    // 任务2: 转到1023
    xlSeriesLed(SERVO_ID, LED_PURPLE, XL320Led);
    //xl320SendPosition(SERVO_ID, 1023);
    HAL_Delay(1000);

    // 任务3: 转到0
	xlSeriesLed(SERVO_ID, LED_CYAN, XL320Led);
	//xl320SendPosition(SERVO_ID, 0);
	HAL_Delay(1000);

  }
  /* USER CODE END 3 */
}
/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
// ============================================================================
// XL系列舵机控制函数
// ============================================================================

/**
  * @brief  初始化舵机系统
  * @retval None
  */
void xlSeriesStart(void)
{
    // 初始化LED和扭矩

    xlSeriesLed(SERVO_ID, 0x01, XL320Led);
    HAL_Delay(100);

    xlSeriesTorque(SERVO_ID, 0x01, XL320Torque);
    HAL_Delay(100);

    xlSeriesControlMode(SERVO_ID, 2);
    HAL_Delay(100);

    xl320SendMovingSpeed(SERVO_ID, 100);
}


/**
  * @brief  设置通信方向
  * @param  tx_mode: 1=发送模式, 0=接收模式
  * @retval None
  */
void xlSeriesSetDirection(uint8_t tx_mode)
{
    if (tx_mode) {
        HAL_GPIO_WritePin(DIR1_GPIO_Port, DIR1_Pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(DIR1_GPIO_Port, DIR1_Pin, GPIO_PIN_RESET);
    }
}

/**
  * @brief  发送数据帧
  * @param  huart: 串口句柄
  * @param  frame: 数据帧指针
  * @param  length: 数据长度
  * @retval None
  */
void xlSeriesSendFrame(UART_HandleTypeDef *huart, uint8_t *frame, uint16_t length)
{
    xlSeriesSetDirection(1); // 设置为发送模式

    HAL_UART_Transmit(huart, frame, length, 100);

    // 等待发送完成
    while (HAL_UART_GetState(huart) != HAL_UART_STATE_READY);

    xlSeriesSetDirection(0); // 切换回接收模式
}

/**
  * @brief  CRC计算函数
  * @param  crc_accum: 初始CRC值
  * @param  data_blk_ptr: 数据指针
  * @param  data_blk_size: 数据长度
  * @retval 计算后的CRC值
  */
uint16_t updateCRC(uint16_t crc_accum, uint8_t *data_blk_ptr, uint16_t data_blk_size)
{
	  uint16_t i, j;
	  static const uint16_t crc_table[256] = { 0x0000,
	    0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
	    0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027,
	    0x0022, 0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D,
	    0x8077, 0x0072, 0x0050, 0x8055, 0x805F, 0x005A, 0x804B,
	    0x004E, 0x0044, 0x8041, 0x80C3, 0x00C6, 0x00CC, 0x80C9,
	    0x00D8, 0x80DD, 0x80D7, 0x00D2, 0x00F0, 0x80F5, 0x80FF,
	    0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1, 0x00A0, 0x80A5,
	    0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1, 0x8093,
	    0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
	    0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197,
	    0x0192, 0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE,
	    0x01A4, 0x81A1, 0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB,
	    0x01FE, 0x01F4, 0x81F1, 0x81D3, 0x01D6, 0x01DC, 0x81D9,
	    0x01C8, 0x81CD, 0x81C7, 0x01C2, 0x0140, 0x8145, 0x814F,
	    0x014A, 0x815B, 0x015E, 0x0154, 0x8151, 0x8173, 0x0176,
	    0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162, 0x8123,
	    0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
	    0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104,
	    0x8101, 0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D,
	    0x8317, 0x0312, 0x0330, 0x8335, 0x833F, 0x033A, 0x832B,
	    0x032E, 0x0324, 0x8321, 0x0360, 0x8365, 0x836F, 0x036A,
	    0x837B, 0x037E, 0x0374, 0x8371, 0x8353, 0x0356, 0x035C,
	    0x8359, 0x0348, 0x834D, 0x8347, 0x0342, 0x03C0, 0x83C5,
	    0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1, 0x83F3,
	    0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
	    0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7,
	    0x03B2, 0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E,
	    0x0384, 0x8381, 0x0280, 0x8285, 0x828F, 0x028A, 0x829B,
	    0x029E, 0x0294, 0x8291, 0x82B3, 0x02B6, 0x02BC, 0x82B9,
	    0x02A8, 0x82AD, 0x82A7, 0x02A2, 0x82E3, 0x02E6, 0x02EC,
	    0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2, 0x02D0, 0x82D5,
	    0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1, 0x8243,
	    0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
	    0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264,
	    0x8261, 0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E,
	    0x0234, 0x8231, 0x8213, 0x0216, 0x021C, 0x8219, 0x0208,
	    0x820D, 0x8207, 0x0202 };

	  for (j = 0; j < data_blk_size; j++)
	  {
	    i = ((uint16_t)(crc_accum >> 8) ^ *data_blk_ptr++) & 0xFF;
	    crc_accum = (crc_accum << 8) ^ crc_table[i];
	  }

	  return crc_accum;
}

/**
  * @brief  控制舵机LED
  * @param  id: 舵机ID
  * @param  on: LED状态
  * @param  address: LED地址
  * @retval None
  */
void xlSeriesLed(uint8_t id, uint8_t on, uint8_t address)
{
    uint16_t crc;

    ledFrame[4] = id;
    ledFrame[8] = address;
    ledFrame[10] = on;

    crc = updateCRC(0, ledFrame, 11);
    ledFrame[11] = (uint8_t)(crc & 0xff);
    ledFrame[12] = (uint8_t)((crc >> 8) & 0xff);

    xlSeriesSendFrame(&huart2, ledFrame, 13);
    HAL_Delay(1);
}

/**
  * @brief  控制舵机模式
  * @param  id: 舵机ID
  * @param  mode: 控制模式：1、滚动模式 2、关节模式
  * @retval None
  */
void xlSeriesControlMode(uint8_t id, uint8_t mode)
{
    uint16_t crc;

    ctrlModeFrame[4] = id;
    ctrlModeFrame[10] = mode;

    crc = updateCRC(0, ctrlModeFrame, 11);
    ctrlModeFrame[11] = (uint8_t)(crc & 0xff);
    ctrlModeFrame[12] = (uint8_t)((crc >> 8) & 0xff);

    xlSeriesSendFrame(&huart2, ctrlModeFrame, 13);
    HAL_Delay(1);
}

/**
  * @brief  控制舵机扭矩
  * @param  id: 舵机ID
  * @param  on: 扭矩状态
  * @param  address: 扭矩地址
  * @retval None
  */
void xlSeriesTorque(uint8_t id, uint8_t on, uint8_t address)
{
    uint16_t crc;

    torqueFrame[4] = id;
    torqueFrame[8] = address;
    torqueFrame[10] = on;

    crc = updateCRC(0, torqueFrame, 11);
    torqueFrame[11] = (uint8_t)(crc & 0xff);
    torqueFrame[12] = (uint8_t)((crc >> 8) & 0xff);

    xlSeriesSendFrame(&huart2, torqueFrame, 13);
    HAL_Delay(1);
}

/**
  * @brief  设置XL320舵机位置
  * @param  id: 舵机ID
  * @param  position: 目标位置
  * @retval None
  */
void xl320SendPosition(uint8_t id, uint16_t position)
{
    uint16_t crc;

    xl320PFrame[4] = id;
    xl320PFrame[10] = (uint8_t)(position & 0xFF);
    xl320PFrame[11] = (uint8_t)((position >> 8) & 0xFF);

    crc = updateCRC(0, xl320PFrame, 12);
    xl320PFrame[12] = (uint8_t)(crc & 0xff);
    xl320PFrame[13] = (uint8_t)((crc >> 8) & 0xff);

    xlSeriesSendFrame(&huart2, xl320PFrame, 14);
    HAL_Delay(1);
}

/**
  * @brief  设置XL320转速
  * @param  id: 舵机ID
  * @param  movingSpeed: 目标位置
  * @retval None
  */
void xl320SendMovingSpeed(uint8_t id, uint16_t movingSpeed)
{
    uint16_t crc;

    xl320MSFrame[4] = id;
    xl320MSFrame[10] = (uint8_t)(movingSpeed & 0xFF);
    xl320MSFrame[11] = (uint8_t)((movingSpeed >> 8) & 0xFF);

    crc = updateCRC(0, xl320MSFrame, 12);
    xl320MSFrame[12] = (uint8_t)(crc & 0xff);
    xl320MSFrame[13] = (uint8_t)((crc >> 8) & 0xff);

    xlSeriesSendFrame(&huart2, xl320MSFrame, 14);
    HAL_Delay(1);
}

/**
  * @brief  设置XL320舵机P增益
  * @param  id: 舵机ID
  * @param  pGain: P增益值
  * @retval None
  */
void xl320SendPGain(uint8_t id, uint8_t pGain)
{
    uint16_t crc;

    xl320PGFrame[4] = id;
    xl320PGFrame[10] = pGain;

    crc = updateCRC(0, xl320PGFrame, 11);
    xl320PGFrame[11] = (uint8_t)(crc & 0xff);
    xl320PGFrame[12] = (uint8_t)((crc >> 8) & 0xff);

    xlSeriesSendFrame(&huart2, xl320PGFrame, 13);
    HAL_Delay(2);
}

/**
  * @brief  舵机电源控制
  * @param  isOn: 电源状态 (1=开启, 0=关闭)
  * @retval None
  */
void xlPowerOff(uint8_t isOn)
{
    if(isOn){
        // 开启所有舵机
        xlSeriesTorque(SERVO_ID_1, 0x01, XL430Torque);
        xlSeriesTorque(SERVO_ID_2, 0x01, XL430Torque);
        xlSeriesTorque(SERVO_ID, 0x01, XL320Torque);

        xlSeriesLed(SERVO_ID_1, 0x01, XL430Led);
        xlSeriesLed(SERVO_ID_2, 0x01, XL430Led);
        xlSeriesLed(SERVO_ID, 0x01, XL320Led);
    } else {
        // 关闭所有舵机
        xlSeriesTorque(SERVO_ID_1, 0x00, XL430Torque);
        xlSeriesTorque(SERVO_ID_2, 0x00, XL430Torque);
        xlSeriesTorque(SERVO_ID, 0x00, XL320Torque);

        xlSeriesLed(SERVO_ID_1, 0x00, XL430Led);
        xlSeriesLed(SERVO_ID_2, 0x00, XL430Led);
        xlSeriesLed(SERVO_ID, 0x00, XL320Led);
    }
}

/* USER CODE END 4 */

///**
//  * @brief  调试函数，打印数据帧（可选）
//  * @param  frame: 数据帧指针
//  * @param  length: 数据长度
//  * @retval None
//  */
//void debugFrame(uint8_t *frame, uint8_t length)
//{
//
//    for(int i = 0; i < length; i++){
//        printf("%02X ", frame[i]);
//    }
//    printf("\r\n");
//
//}

/**
  * @brief  通过串口1发送字符串到PC端（不换行）
  * @param  message: 要发送的字符串
  * @retval None
  */
void Debug_Print(const char* message)
{
    uint16_t len = 0;
    while (message[len] != '\0') {
        len++;
    }
    HAL_UART_Transmit(&huart1, (uint8_t*)message, len, 100);
}

/**
  * @brief  通过串口1发送字符串到PC端（自动换行）
  * @param  message: 要发送的字符串
  * @retval None
  */
void Debug_Println(const char* message)
{
    Debug_Print(message);
    Debug_Print("\r\n");  // 添加换行符
}

/**
 * @brief 读取XL320舵机位置
 * @param id: 舵机ID
 * @retval None
 */
void xl320ReadPosition(uint8_t id)
{
    uint16_t crc;

    // 更新ID
    xl320ReadGoalPosition[4] = id;

    // 计算CRC（从包头到参数结束，不包括CRC字段）
    crc = updateCRC(0, xl320ReadGoalPosition, 12);
    xl320ReadGoalPosition[12] = (uint8_t)(crc & 0xFF);
    xl320ReadGoalPosition[13] = (uint8_t)((crc >> 8) & 0xFF);

    // 发送读取指令
    xlSeriesSendFrame(&huart2, xl320ReadGoalPosition, 14);

    // 启动串口接收
    // 使用轮询接收（推荐）
    uint8_t length = 13;
	HAL_StatusTypeDef status = HAL_UART_Receive(&huart2, uart2_rx_buffer, length, 500);

	if (status == HAL_OK) {
		// 接收成功 - 黄色
		xlSeriesLed(SERVO_ID, LED_YELLOW, XL320Led);
		position_received = 1;
	} else {
		// 接收失败 - 红色
		xlSeriesLed(SERVO_ID, LED_RED, XL320Led);
		position_received = 0;
	}

    HAL_Delay(2000);
}

/**
 * @brief 验证位置数据包的有效性
 * @param data: 接收到的数据
 * @retval 1:有效 0:无效
 */
uint8_t verifyPositionPacket(uint8_t *data)
{
    // 检查包头
    if (data[0] != 0xFF || data[1] != 0xFF || data[2] != 0xFD || data[3] != 0x00) {
        return 0;
    }

    // 检查指令（应该是0x55，即读取应答）
    if (data[7] != 0x55) {
        return 0;
    }

    return 1;
}

/**
 * @brief 从数据包中解析位置值
 * @param data: 接收到的数据
 * @retval 位置值
 */
uint16_t parsePositionValue(uint8_t *data)
{
    // 位置值在数据包的参数部分（索引9和10）
    return (uint16_t)(data[10] << 8) | data[9];
}


/**
 * @brief 处理接收到的位置数据
 * @retval None
 */
void processPositionData(void)
{
    if (position_received) {
        if (verifyPositionPacket(uart2_rx_buffer)) {
            current_position = parsePositionValue(uart2_rx_buffer);
            uint8_t point = current_position % 3;
            // 根据位置值控制LED：奇数亮红灯，偶数亮蓝灯
            if (point == 0) {
                // 被3整除 - 白灯
                xlSeriesLed(SERVO_ID, LED_WHITE, XL320Led);
            } else if(point == 1){
                // 余数为1 - 蓝灯
                xlSeriesLed(SERVO_ID, LED_BLUE, XL320Led);
            } else if(point == 2){
            	// 余数为2 -  绿灯
            	xlSeriesLed(SERVO_ID, LED_GREEN, XL320Led);
            }
            HAL_Delay(2000);
            position_received = 0; // 重置标志
		} else {
			xlSeriesLed(SERVO_ID, LED_YELLOW, XL320Led);
		}
    }
}

/**
 * @brief 串口2接收完成回调函数
 * @retval None
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        position_received = 1; // 设置接收完成标志
    }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
