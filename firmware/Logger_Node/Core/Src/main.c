/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c (Logger Node)
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "fatfs.h"
#include "sd_spi.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DS3231_ADDR (0x68 << 1)
FIL log_file;
uint8_t log_file_open = 0;
uint32_t log_row_count = 0;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
int CAN_Logger_Init(void);
uint8_t DecToBCD(uint8_t val);
int Init_DS3231(void);
int Read_DS3231(void);
uint8_t BCDToDec(uint8_t val);
int SD_CardInserted(void);
int DS3231_GetTimestamp(char *timestamp);
int Mount_SD_With_Retry(FATFS *fs);
int Log_File_Open(void);
int Log_IMU_Row_Fast(int16_t accel_x, int16_t accel_y, int16_t accel_z, int16_t gyro_x, int16_t gyro_y, int16_t gyro_z);
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
  MX_CAN1_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */
  char bootmsg[] = "\r\n--- LOGGER NODE BOOT ---\r\n";
  HAL_UART_Transmit(&huart2, (uint8_t*)bootmsg, strlen(bootmsg), 100);

  CAN_RxHeaderTypeDef rx_header;
  uint8_t rx_data[8];

  int16_t accel_x = 0;
  int16_t accel_y = 0;
  int16_t accel_z = 0;

  int16_t gyro_x = 0;
  int16_t gyro_y = 0;
  int16_t gyro_z = 0;

  uint8_t accel_received = 0;
  uint8_t gyro_received = 0;
  uint8_t sd_ready = 0;

  if (CAN_Logger_Init() != 0){
  	  char msg[] = "CAN init failed\r\n";
  	  HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);}
  else{
  	  char msg[] = "CAN init success\r\n";
  	  HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);}


  if (!SD_CardInserted()){
	  char msg[] = "No SD card detected. Logging disabled.\r\n";
	  HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);}
  else{
	  char msg[] = "SD card detected\r\n";
  	  HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);

  	  static FATFS fs;

  	  if (Mount_SD_With_Retry(&fs) != 0){
  		  char fail[] = "SD mount failed after retry. Logging disabled.\r\n";
  		  HAL_UART_Transmit(&huart2, (uint8_t*)fail, strlen(fail), 100);}
  	  else{
  		  sd_ready = 1;
  		  char ok[] = "SD ready for CAN logging\r\n";
  		  HAL_UART_Transmit(&huart2, (uint8_t*)ok, strlen(ok), 100);

  		  if(Log_File_Open() != 0){
  			  sd_ready = 0;
  			  char fail[] = "LOG.CSV open failed. Logging disabled.\r\n";
  			  HAL_UART_Transmit(&huart2, (uint8_t*)fail, strlen(fail), 100);
  		}
  	  }
    }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) > 0){
		  HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rx_header, rx_data);

		  if (rx_header.StdId == 0x120 && rx_header.DLC == 8){
			  accel_x = (int16_t)((rx_data[0] << 8) | rx_data[1]);
			  accel_y = (int16_t)((rx_data[2] << 8) | rx_data[3]);
			  accel_z = (int16_t)((rx_data[4] << 8) | rx_data[5]);

			  accel_received = 1;

			  char msg[80];
			  sprintf(msg, "RX ACCEL: %d,%d,%d\r\n", accel_x, accel_y, accel_z);
			  HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);}

		  else if (rx_header.StdId == 0x121 && rx_header.DLC == 8){
			  gyro_x = (int16_t)((rx_data[0] << 8) | rx_data[1]);
			  gyro_y = (int16_t)((rx_data[2] << 8) | rx_data[3]);
			  gyro_z = (int16_t)((rx_data[4] << 8) | rx_data[5]);

			  gyro_received = 1;

			  char msg[80];
			  sprintf(msg, "RX GYRO: %d,%d,%d\r\n", gyro_x, gyro_y, gyro_z);
			  HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);}
	  	  }

	  	  if (accel_received && gyro_received){
	  		  accel_received = 0;
	  		  gyro_received = 0;

	  		  if (sd_ready){
	  			Log_IMU_Row_Fast(accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z);}
	  		  else{
	  			  char msg[] = "CAN sample received but SD not ready\r\n";
	  			  HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);}
	  	  }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 2;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = ENABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 208;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : SD_CD_Pin */
  GPIO_InitStruct.Pin = SD_CD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(SD_CD_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SD_CS_Pin */
  GPIO_InitStruct.Pin = SD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(SD_CS_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
  * @brief configuring init
  * @retval None
  */
int Init_DS3231(void){
	uint8_t sec   = 0;
	uint8_t min   = 44;
	uint8_t hr    = 18;
	uint8_t day   = 6;   // day of week: 1-7
	uint8_t date  = 25;  // day of month: 1-31
	uint8_t month = 7;   // month: 1-12
	uint8_t year  = 26;  // 2026 -> 26

	uint8_t rtc_data[7];

	rtc_data[0] = DecToBCD(sec);    // 0x00 seconds, 00–59
	rtc_data[1] = DecToBCD(min);    // 0x01 minutes, 00–59
	rtc_data[2] = DecToBCD(hr);     // 0x02 hours, 24-hour mode, 00–23
	rtc_data[3] = DecToBCD(day);    // 0x03 day of week, 1–7
	rtc_data[4] = DecToBCD(date);   // 0x04 day of month, 1–31
	rtc_data[5] = DecToBCD(month);  // 0x05 month, 1–12, century bit = 0
	rtc_data[6] = DecToBCD(year);   // 0x06 year, 00–99

	if (HAL_I2C_Mem_Write(&hi2c1, DS3231_ADDR, 0x00, I2C_MEMADD_SIZE_8BIT, rtc_data, 7, 100) != HAL_OK){
		return -1;}

	return 0;
}
/**
  * @brief configuring allow all filter for loop back mode
  * @retval None
  */
int CAN_Logger_Init(void){
    CAN_FilterTypeDef filter = {0};

    filter.FilterActivation = CAN_FILTER_ENABLE;
    filter.FilterBank = 0;
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    filter.FilterIdHigh = 0;
    filter.FilterIdLow = 0;
    filter.FilterMaskIdHigh = 0;
    filter.FilterMaskIdLow = 0;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.SlaveStartFilterBank = 14;

    if(HAL_CAN_ConfigFilter(&hcan1, &filter) != HAL_OK){
        return -1;
    }

    if(HAL_CAN_Start(&hcan1) != HAL_OK){
        return -2;
    }

    return 0;
}
/**
 * @brief reading from the RTC
  * @retval None
  */
int Read_DS3231(void){
    uint8_t rtc_data[7];
    char msg[100];

    if (HAL_I2C_Mem_Read(&hi2c1, DS3231_ADDR, 0x00, I2C_MEMADD_SIZE_8BIT, rtc_data, 7, 100) != HAL_OK){
        return -1;}
    uint8_t sec   = BCDToDec(rtc_data[0] & 0x7F);
    uint8_t min   = BCDToDec(rtc_data[1] & 0x7F);
    uint8_t hr    = BCDToDec(rtc_data[2] & 0x3F); // 24-hour mode
    uint8_t day   = BCDToDec(rtc_data[3] & 0x07);
    uint8_t date  = BCDToDec(rtc_data[4] & 0x3F);
    uint8_t month = BCDToDec(rtc_data[5] & 0x1F);
    uint8_t year  = BCDToDec(rtc_data[6]);

    sprintf(msg, "RTC: %02d:%02d:%02d  Date: %02d/%02d/20%02d  Day:%d\r\n", hr, min, sec, date, month, year, day);

    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);

    return 0;
}
/**
  * @brief Switch From Decimal to BCD for DS3231
  * @retval BCD value
  */
uint8_t DecToBCD(uint8_t val){
    return ((val / 10) << 4) | (val % 10);
}
/**
  * @brief Switch From BCD to decimal
  * @retval decimal value
  */
uint8_t BCDToDec(uint8_t val){
    return ((val >> 4) * 10) + (val & 0x0F);
}
/**
  * @brief detects if SD card is in or not
  * @retval boolean yes or no
  */
int SD_CardInserted(void){
	if (HAL_GPIO_ReadPin(SD_CD_GPIO_Port, SD_CD_Pin) == GPIO_PIN_SET){
		return 1;}
	else{
		return 0;}
}
/**
  * @brief writes data in csv format
  * @retval success or fail
  */
int Log_IMU_Row(int16_t accel_x, int16_t accel_y, int16_t accel_z, int16_t gyro_x, int16_t gyro_y, int16_t gyro_z){
	FIL file;
	FRESULT fres;
	UINT bytesWritten;
	char timestamp[25];
	char row[160];
	char msg[80];
	uint32_t time_ms;

	if (DS3231_GetTimestamp(timestamp) != 0){
		char err[] = "RTC timestamp failed\r\n";
		HAL_UART_Transmit(&huart2, (uint8_t*)err, strlen(err), 100);
		return -1;}

	time_ms = HAL_GetTick();

	fres = f_open(&file, "LOG.CSV", FA_OPEN_ALWAYS | FA_WRITE);

	if (fres != FR_OK){
		sprintf(msg, "LOG.CSV open failed: %d\r\n", fres);
		HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
		return -1;}

	if (f_size(&file) == 0){
		char header[] = "timestamp,time_ms,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,status\r\n";

		fres = f_write(&file, header, strlen(header), &bytesWritten);

		if (fres != FR_OK){
			sprintf(msg, "CSV header write failed: %d\r\n", fres);
			HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
			f_close(&file);
			return -1;}
	}

	f_lseek(&file, f_size(&file));

	sprintf(row, "%s,%lu,%d,%d,%d,%d,%d,%d,OK\r\n",
			timestamp,
			time_ms,
			accel_x,
			accel_y,
			accel_z,
			gyro_x,
			gyro_y,
			gyro_z);

	fres = f_write(&file, row, strlen(row), &bytesWritten);

	if (fres != FR_OK){
		sprintf(msg, "CSV row write failed: %d\r\n", fres);
		HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
		f_close(&file);
		return -1;}

	f_close(&file);

	sprintf(msg, "CSV row written: %u bytes\r\n", bytesWritten);
	HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);

	return 0;
}
/**
  * @brief gets time stamp for writing
  * @retval success or fail
  */
int DS3231_GetTimestamp(char *timestamp)
{
	uint8_t rtc_data[7];

	if (HAL_I2C_Mem_Read(&hi2c1, DS3231_ADDR, 0x00, I2C_MEMADD_SIZE_8BIT, rtc_data, 7, 100) != HAL_OK){
		return -1;}

	uint8_t sec   = BCDToDec(rtc_data[0] & 0x7F);
	uint8_t min   = BCDToDec(rtc_data[1] & 0x7F);
	uint8_t hr    = BCDToDec(rtc_data[2] & 0x3F);
	uint8_t date  = BCDToDec(rtc_data[4] & 0x3F);
	uint8_t month = BCDToDec(rtc_data[5] & 0x1F);
	uint8_t year  = BCDToDec(rtc_data[6]);

	sprintf(timestamp, "20%02d-%02d-%02d %02d:%02d:%02d", year, month, date, hr, min, sec);

	return 0;
}
/**
  * @brief retry mount if failed
  * @retval success or fail
  */
int Mount_SD_With_Retry(FATFS *fs){
	FRESULT fres;
	char msg[80];

	fres = f_mount(fs, "", 1);

	if (fres == FR_OK){
		char ok[] = "f_mount success\r\n";
		HAL_UART_Transmit(&huart2, (uint8_t*)ok, strlen(ok), 100);
		return 0;}

	sprintf(msg, "f_mount failed: %d\r\n", fres);
	HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);

	char retrymsg[] = "Retrying SD mount...\r\n";
	HAL_UART_Transmit(&huart2, (uint8_t*)retrymsg, strlen(retrymsg), 100);

	f_mount(NULL, "", 0);

	SD_SPI_ResetState();

	HAL_Delay(100);

	fres = f_mount(fs, "", 1);

	if (fres == FR_OK){
		char ok[] = "f_mount success\r\n";
		HAL_UART_Transmit(&huart2, (uint8_t*)ok, strlen(ok), 100);
		return 0;}

	sprintf(msg, "f_mount retry failed: %d\r\n", fres);
	HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);

	return -1;
}

int Log_File_Open(void){
    FRESULT fres;
    UINT bytesWritten;
    char msg[80];

    fres = f_open(&log_file, "LOG.CSV", FA_OPEN_ALWAYS | FA_WRITE);

    if(fres != FR_OK){
        sprintf(msg, "LOG.CSV open failed: %d\r\n", fres);
        HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
        return -1;
    }

    if(f_size(&log_file) == 0){
        char header[] = "timestamp,time_ms,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,status\r\n";

        fres = f_write(&log_file, header, strlen(header), &bytesWritten);

        if(fres != FR_OK){
            sprintf(msg, "CSV header write failed: %d\r\n", fres);
            HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
            f_close(&log_file);
            return -1;
        }

        f_sync(&log_file);
    }

    f_lseek(&log_file, f_size(&log_file));

    log_file_open = 1;
    log_row_count = 0;

    char ok[] = "LOG.CSV open and ready\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t*)ok, strlen(ok), 100);

    return 0;
}

int Log_IMU_Row_Fast(int16_t accel_x, int16_t accel_y, int16_t accel_z, int16_t gyro_x, int16_t gyro_y, int16_t gyro_z){
    FRESULT fres;
    UINT bytesWritten;
    char timestamp[25];
    char row[160];
    char msg[80];
    uint32_t time_ms;

    if(log_file_open == 0){
        return -1;
    }

    if(DS3231_GetTimestamp(timestamp) != 0){
        char err[] = "RTC timestamp failed\r\n";
        HAL_UART_Transmit(&huart2, (uint8_t*)err, strlen(err), 100);
        return -1;
    }

    time_ms = HAL_GetTick();

    sprintf(row, "%s,%lu,%d,%d,%d,%d,%d,%d,OK\r\n",
            timestamp,
            time_ms,
            accel_x,
            accel_y,
            accel_z,
            gyro_x,
            gyro_y,
            gyro_z);

    fres = f_write(&log_file, row, strlen(row), &bytesWritten);

    if(fres != FR_OK){
        sprintf(msg, "CSV row write failed: %d\r\n", fres);
        HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
        return -1;
    }

    log_row_count++;

    if(log_row_count % 10 == 0){
        fres = f_sync(&log_file);

        if(fres != FR_OK){
            sprintf(msg, "CSV sync failed: %d\r\n", fres);
            HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
            return -1;
        }

        sprintf(msg, "CSV synced: %lu rows\r\n", log_row_count);
        HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
    }

    return 0;
}

/* USER CODE END 4 */

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
