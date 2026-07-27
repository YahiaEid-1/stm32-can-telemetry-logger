/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c (Vehicle Dynamics Node)
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MPU6050_ADDR (0x68 << 1)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
volatile uint8_t imu_data_ready = 0;
volatile uint32_t imu_timestamp_ms = 0;
uint8_t imu_sequence_number = 0;
uint8_t uart_print_counter = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CAN1_Init(void);
/* USER CODE BEGIN PFP */
int MPU6050_selftest (void);
int MPU6050_normal_init(void);
int MPU6050_read_sample(int16_t *acc_x, int16_t *acc_y, int16_t *acc_z, int16_t *gyro_x, int16_t *gyro_y, int16_t *gyro_z);
int CAN_Vehicle_init(void);
int CAN_send_imu_frames(int16_t acc_x, int16_t acc_y, int16_t acc_z, int16_t gyro_x, int16_t gyro_y, int16_t gyro_z, uint8_t sequence_number, uint32_t timestamp_ms);
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
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_CAN1_Init();
  /* USER CODE BEGIN 2 */
  char bootmsg[] = "\r\n--- VEHICLE BOOT ---\r\n";
  HAL_UART_Transmit(&huart2, (uint8_t *)bootmsg, strlen(bootmsg), 100);

  if(MPU6050_normal_init() != 0){
	  char error[] = "MPU init failed\r\n";
      HAL_UART_Transmit(&huart2, (uint8_t *)error, strlen(error), 100);

      while(1){
      }
  }

  if(CAN_Vehicle_init() != 0){
      char error[] = "CAN init failed\r\n";
      HAL_UART_Transmit(&huart2, (uint8_t *)error, strlen(error), 100);

      while(1){
      }
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1){
	  if(imu_data_ready == 1){
	  		  imu_data_ready = 0;
	  		  int16_t acc_x, acc_y, acc_z;
	  		  int16_t gyro_x, gyro_y, gyro_z;

	  		if(MPU6050_read_sample(&acc_x, &acc_y, &acc_z, &gyro_x, &gyro_y, &gyro_z) == 0){

	  			uart_print_counter++;

	  			if(uart_print_counter >= 20){

	  				uart_print_counter = 0;

	  				if(CAN_send_imu_frames(acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, imu_sequence_number, imu_timestamp_ms) == 0){

	  					char message[120];

	  					snprintf(message, sizeof(message), "TX IMU sample | Seq=%d | Time=%lu ms\r\n", imu_sequence_number, imu_timestamp_ms);

	  					HAL_UART_Transmit(&huart2, (uint8_t *)message, strlen(message), 100);

	  					imu_sequence_number++;
	  				}
	  			}
	  		}
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
  hi2c1.Init.OwnAddress1 = 0;
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
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : MPU_INT_Pin */
  GPIO_InitStruct.Pin = MPU_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MPU_INT_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
  * @brief MPU-6050 self test percentage
  * @retval None
  */
int MPU6050_selftest(void)
{
    const int SELF_TEST_SAMPLES = 200;

    uint8_t gyrost = 0xE0;
    uint8_t accst = 0xF0;
    uint8_t wake = 0x00;
    uint8_t gyro_normal = 0x00;
    uint8_t acc_normal = 0x10;

    int16_t gyro_measurement_x, gyro_measurement_y, gyro_measurement_z;
    int16_t acc_measurement_x, acc_measurement_y, acc_measurement_z;
    int16_t gyrost_measurement_x, gyrost_measurement_y, gyrost_measurement_z;
    int16_t accst_measurement_x, accst_measurement_y, accst_measurement_z;

    int32_t gyro_sum_x, gyro_sum_y, gyro_sum_z;
    int32_t acc_sum_x, acc_sum_y, acc_sum_z;

    uint8_t XGst, YGst, ZGst;
    uint8_t Ast;

    uint8_t gyro_raw[6];
    uint8_t acc_raw[6];

    float FTXG, FTYG, FTZG, FTXA, FTYA, FTZA;

    // MPU comes up in sleep mode upon power-up so Set sleep bit to 0
    if(HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x6B, I2C_MEMADD_SIZE_8BIT, &wake, 1, 100) != HAL_OK){
        return -1;}
    //

    //Setting normal ranges before baseline readings
    if(HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x1B, I2C_MEMADD_SIZE_8BIT, &gyro_normal, 1, 100) != HAL_OK){
        return -1;}

    if(HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x1C, I2C_MEMADD_SIZE_8BIT, &acc_normal, 1, 100) != HAL_OK){
        return -1;}
    //

    HAL_Delay(1000);

    //Gyroscope measurements without self test
    gyro_sum_x = 0;
    gyro_sum_y = 0;
    gyro_sum_z = 0;

    for(int i = 0; i < SELF_TEST_SAMPLES; i++){
        if(HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, 0x43, I2C_MEMADD_SIZE_8BIT, gyro_raw, 6, 100) != HAL_OK){
            return -1;}

        gyro_sum_x += ((int16_t)gyro_raw[0] << 8) | gyro_raw[1];
        gyro_sum_y += ((int16_t)gyro_raw[2] << 8) | gyro_raw[3];
        gyro_sum_z += ((int16_t)gyro_raw[4] << 8) | gyro_raw[5];

        HAL_Delay(2);}

    gyro_measurement_x = gyro_sum_x / SELF_TEST_SAMPLES;
    gyro_measurement_y = gyro_sum_y / SELF_TEST_SAMPLES;
    gyro_measurement_z = gyro_sum_z / SELF_TEST_SAMPLES;
    //

    //Accelerometer measurements without self test
    acc_sum_x = 0;
    acc_sum_y = 0;
    acc_sum_z = 0;

    for(int i = 0; i < SELF_TEST_SAMPLES; i++){
        if(HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, 0x3B, I2C_MEMADD_SIZE_8BIT, acc_raw, 6, 100) != HAL_OK){
            return -1;}

        acc_sum_x += ((int16_t)acc_raw[0] << 8) | acc_raw[1];
        acc_sum_y += ((int16_t)acc_raw[2] << 8) | acc_raw[3];
        acc_sum_z += ((int16_t)acc_raw[4] << 8) | acc_raw[5];

        HAL_Delay(2);}

    acc_measurement_x = acc_sum_x / SELF_TEST_SAMPLES;
    acc_measurement_y = acc_sum_y / SELF_TEST_SAMPLES;
    acc_measurement_z = acc_sum_z / SELF_TEST_SAMPLES;
    //

    //Turning on Self Test
    if(HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x1B, I2C_MEMADD_SIZE_8BIT, &gyrost, 1, 100) != HAL_OK){
        return -1;}

    if(HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x1C, I2C_MEMADD_SIZE_8BIT, &accst, 1, 100) != HAL_OK){
        return -1;}
    //

    HAL_Delay(100);

    //Obtaining [X-Z]g/a
    uint8_t self_test_codes[4];

    if(HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, 0x0D, I2C_MEMADD_SIZE_8BIT, self_test_codes, 4, 100) != HAL_OK){
        return -1;}

    XGst = self_test_codes[0];
    YGst = self_test_codes[1];
    ZGst = self_test_codes[2];
    Ast = self_test_codes[3];
    //

    //Gyroscope measurements with self test
    gyro_sum_x = 0;
    gyro_sum_y = 0;
    gyro_sum_z = 0;

    for(int i = 0; i < SELF_TEST_SAMPLES; i++){
        if(HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, 0x43, I2C_MEMADD_SIZE_8BIT, gyro_raw, 6, 100) != HAL_OK){
            return -1;}

        gyro_sum_x += ((int16_t)gyro_raw[0] << 8) | gyro_raw[1];
        gyro_sum_y += ((int16_t)gyro_raw[2] << 8) | gyro_raw[3];
        gyro_sum_z += ((int16_t)gyro_raw[4] << 8) | gyro_raw[5];

        HAL_Delay(2);}

    gyrost_measurement_x = gyro_sum_x / SELF_TEST_SAMPLES;
    gyrost_measurement_y = gyro_sum_y / SELF_TEST_SAMPLES;
    gyrost_measurement_z = gyro_sum_z / SELF_TEST_SAMPLES;
    //

    //Accelerometer measurements with self test
    acc_sum_x = 0;
    acc_sum_y = 0;
    acc_sum_z = 0;

    for(int i = 0; i < SELF_TEST_SAMPLES; i++){
        if(HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, 0x3B, I2C_MEMADD_SIZE_8BIT, acc_raw, 6, 100) != HAL_OK){
            return -1;}

        acc_sum_x += ((int16_t)acc_raw[0] << 8) | acc_raw[1];
        acc_sum_y += ((int16_t)acc_raw[2] << 8) | acc_raw[3];
        acc_sum_z += ((int16_t)acc_raw[4] << 8) | acc_raw[5];

        HAL_Delay(2);}

    accst_measurement_x = acc_sum_x / SELF_TEST_SAMPLES;
    accst_measurement_y = acc_sum_y / SELF_TEST_SAMPLES;
    accst_measurement_z = acc_sum_z / SELF_TEST_SAMPLES;
    //

    //Turning off Self Test
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x1B, I2C_MEMADD_SIZE_8BIT, &gyro_normal, 1, 100);
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x1C, I2C_MEMADD_SIZE_8BIT, &acc_normal, 1, 100);
    //

    //Using DataSheet Equations to calculate FT[g/a]
    uint8_t XGvalue = XGst & 0x1F;
    uint8_t YGvalue = YGst & 0x1F;
    uint8_t ZGvalue = ZGst & 0x1F;

    if(XGvalue != 0){
        FTXG = 25.0f * 131.0f * powf(1.046f, (float)(XGvalue - 1));}
    else{
        FTXG = 0;}

    if(YGvalue != 0){
        FTYG = -25.0f * 131.0f * powf(1.046f, (float)(YGvalue - 1));}
    else{
        FTYG = 0;}

    if(ZGvalue != 0){
        FTZG = 25.0f * 131.0f * powf(1.046f, (float)(ZGvalue - 1));}
    else{
        FTZG = 0;}

    uint8_t XAvalue = (((XGst >> 5) & 0x07) << 2) | ((Ast >> 4) & 0x03);
    uint8_t YAvalue = (((YGst >> 5) & 0x07) << 2) | ((Ast >> 2) & 0x03);
    uint8_t ZAvalue = (((ZGst >> 5) & 0x07) << 2) | (Ast & 0x03);

    if(XAvalue != 0){
        FTXA = 4096.0f * 0.34f * powf(0.92f / 0.34f, (XAvalue - 1) / 30.0f);}
    else{
        FTXA = 0;}

    if(YAvalue != 0){
        FTYA = 4096.0f * 0.34f * powf(0.92f / 0.34f, (YAvalue - 1) / 30.0f);}
    else{
        FTYA = 0;}

    if(ZAvalue != 0){
        FTZA = 4096.0f * 0.34f * powf(0.92f / 0.34f, (ZAvalue - 1) / 30.0f);}
    else{
        FTZA = 0;}
    //

    //STR Gyro calculation
    int32_t STRgyro_x = (int32_t)gyrost_measurement_x - gyro_measurement_x;
    int32_t STRgyro_y = (int32_t)gyrost_measurement_y - gyro_measurement_y;
    int32_t STRgyro_z = (int32_t)gyrost_measurement_z - gyro_measurement_z;
    //

    //STR Acc Calculation
    int32_t STRacc_x = (int32_t)accst_measurement_x - acc_measurement_x;
    int32_t STRacc_y = (int32_t)accst_measurement_y - acc_measurement_y;
    int32_t STRacc_z = (int32_t)accst_measurement_z - acc_measurement_z;
    //

    //Change from factory trim Self Test (%)
    if((FTXG == 0) || (FTYG == 0) || (FTZG == 0) || (FTXA == 0) || (FTYA == 0) || (FTZA == 0)){
        return -2;}

    float FTGyroX = 100.0f * ((float)STRgyro_x - FTXG) / FTXG;
    float FTGyroY = 100.0f * ((float)STRgyro_y - FTYG) / FTYG;
    float FTGyroZ = 100.0f * ((float)STRgyro_z - FTZG) / FTZG;

    float FTAccX = 100.0f * ((float)STRacc_x - FTXA) / FTXA;
    float FTAccY = 100.0f * ((float)STRacc_y - FTYA) / FTYA;
    float FTAccZ = 100.0f * ((float)STRacc_z - FTZA) / FTZA;
    //

    char message[240];

    snprintf(message, sizeof(message),
             "Gyro self-test change:\r\n"
             "X = %.2f%%\r\n"
             "Y = %.2f%%\r\n"
             "Z = %.2f%%\r\n"
             "Accel self-test change:\r\n"
             "X = %.2f%%\r\n"
             "Y = %.2f%%\r\n"
             "Z = %.2f%%\r\n",
             FTGyroX, FTGyroY, FTGyroZ,
             FTAccX, FTAccY, FTAccZ);

    HAL_UART_Transmit(&huart2, (uint8_t *)message, strlen(message), 100);

    return 0;
}

/**
  * @brief Normal Initialisation
  * @retval None
  */
int MPU6050_normal_init(void)
{
    uint8_t wake = 0x00;
    uint8_t sample_rate_divider = 4;
    uint8_t config = 0x03;
    uint8_t gyro_config = 0x00;
    uint8_t accel_config = 0x10;
    uint8_t int_enable = 0x01;

    // Wake MPU
    if(HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x6B, I2C_MEMADD_SIZE_8BIT, &wake, 1, 100) != HAL_OK){
        return -1;}

    // DLPF enabled, approximately 42 Hz gyro bandwidth
    if(HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x1A, I2C_MEMADD_SIZE_8BIT, &config, 1, 100) != HAL_OK){
        return -1;}

    // 1 kHz / (1 + 4) = 200 Hz sample rate
    if(HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x19, I2C_MEMADD_SIZE_8BIT, &sample_rate_divider, 1, 100) != HAL_OK){
        return -1;}

    // Gyro ±250 dps
    if(HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x1B, I2C_MEMADD_SIZE_8BIT, &gyro_config, 1, 100) != HAL_OK){
        return -1;}

    // Accelerometer ±8 g
    if(HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x1C, I2C_MEMADD_SIZE_8BIT, &accel_config, 1, 100) != HAL_OK){
        return -1;}

    // Enable Data Ready interrupt
    if(HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x38, I2C_MEMADD_SIZE_8BIT, &int_enable, 1, 100) != HAL_OK){
        return -1;}

    return 0;
}

/**
  * @brief acquiring data from MPU-6050 registers
  * @retval accelerometer and gyroscope data
  */
int MPU6050_read_sample(int16_t *acc_x, int16_t *acc_y, int16_t *acc_z, int16_t *gyro_x, int16_t *gyro_y, int16_t *gyro_z){
    uint8_t raw_data[14]; //Temperature reading not needed so it is not stored and separating the reading results
    					  //two different I2C which will result uncorrelated gyroscope and accelerometer

    if(HAL_I2C_Mem_Read(&hi2c1, 0x68 << 1, 0x3B, I2C_MEMADD_SIZE_8BIT, raw_data, 14, 100) != HAL_OK){
        return -1;}

    *acc_x = ((int16_t)raw_data[0] << 8) | raw_data[1];
    *acc_y = ((int16_t)raw_data[2] << 8) | raw_data[3];
    *acc_z = ((int16_t)raw_data[4] << 8) | raw_data[5];

    *gyro_x = ((int16_t)raw_data[8] << 8) | raw_data[9];
    *gyro_y = ((int16_t)raw_data[10] << 8) | raw_data[11];
    *gyro_z = ((int16_t)raw_data[12] << 8) | raw_data[13];

    return 0;
}

/**
  * @brief detecting MPU_INT_PIN
  * @retval Imu_data_ready set
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == MPU_INT_Pin){
        imu_data_ready = 1;
        imu_timestamp_ms = HAL_GetTick();
    }
}

/**
  * @brief configuring allow all filter for loop back mode
  * @retval None
  */
int CAN_Vehicle_init(void)
{
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
  * @brief CAN bus send
  * @retval None
  */
int CAN_send_imu_frames(int16_t acc_x, int16_t acc_y, int16_t acc_z, int16_t gyro_x, int16_t gyro_y, int16_t gyro_z, uint8_t sequence_number, uint32_t timestamp_ms){
    CAN_TxHeaderTypeDef tx_header;
    uint32_t tx_mailbox;
    uint8_t tx_data[8];

    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8;
    tx_header.TransmitGlobalTime = DISABLE;

    // Accel frame: CAN ID 0x120
    tx_header.StdId = 0x120;

    tx_data[0] = (uint8_t)(acc_x >> 8);
    tx_data[1] = (uint8_t)acc_x;
    tx_data[2] = (uint8_t)(acc_y >> 8);
    tx_data[3] = (uint8_t)acc_y;
    tx_data[4] = (uint8_t)(acc_z >> 8);
    tx_data[5] = (uint8_t)acc_z;
    tx_data[6] = sequence_number;
    tx_data[7] = 0x00;


    if(HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, &tx_mailbox) != HAL_OK){
        return -1;}

    // Gyro frame: CAN ID 0x121
    tx_header.StdId = 0x121;

    tx_data[0] = (uint8_t)(gyro_x >> 8);
    tx_data[1] = (uint8_t)gyro_x;
    tx_data[2] = (uint8_t)(gyro_y >> 8);
    tx_data[3] = (uint8_t)gyro_y;
    tx_data[4] = (uint8_t)(gyro_z >> 8);
    tx_data[5] = (uint8_t)gyro_z;
    tx_data[6] = sequence_number;
    tx_data[7] = 0x00;

    if(HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, &tx_mailbox) != HAL_OK){
        return -1;}
    //TimeStamp frame: CAN ID 0x122
    /*tx_header.StdId = 0x122;
    tx_header.DLC = 5;

    tx_data[0] = (uint8_t)(timestamp_ms >> 24);
    tx_data[1] = (uint8_t)(timestamp_ms >> 16);
    tx_data[2] = (uint8_t)(timestamp_ms >> 8);
    tx_data[3] = (uint8_t)timestamp_ms;
    tx_data[4] = sequence_number;

    if(HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, &tx_mailbox) != HAL_OK){
        return -1;
    }
	*/
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
