/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include <stdint.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
static const uint8_t LDC_I2C_ADDR = 0x2A << 1;
static const uint8_t CONFIG_ADDR = 0x1A;
static const uint8_t MUX_CONFIG_ADDR = 0x1B;

static const uint8_t DATA0_MSB = 0x00;
static const uint8_t DATA0_LSB = 0x01;
static const uint8_t DATA1_MSB = 0x02;
static const uint8_t DATA1_LSB = 0x03;
static const uint8_t DATA2_MSB = 0x04;
static const uint8_t DATA2_LSB = 0x05;
static const uint8_t DATA3_MSB = 0x06;
static const uint8_t DATA3_LSB = 0x07;
static const uint8_t RCOUNT0 = 0x08;
static const uint8_t RCOUNT1 = 0x09;
static const uint8_t RCOUNT2 = 0x0A;
static const uint8_t RCOUNT3 = 0x0B;

static const uint8_t SETTLECOUNT0 = 0x10;
static const uint8_t SETTLECOUNT1 = 0x11;
static const uint8_t SETTLECOUNT2 = 0x12;
static const uint8_t SETTLECOUNT3 = 0x13;

static const uint8_t CLOCK_DIVIDERS0 = 0x14;
static const uint8_t CLOCK_DIVIDERS1 = 0x15;
static const uint8_t CLOCK_DIVIDERS2 = 0x16;
static const uint8_t CLOCK_DIVIDERS3 = 0x17;

static const uint8_t STATUS = 0x18;
static const uint8_t ERROR_CONFIG = 0x19;

static const uint8_t DRIVE_CURRENT0 = 0x1E;
static const uint8_t DRIVE_CURRENT1 = 0x1F;
static const uint8_t DRIVE_CURRENT2 = 0x20;
static const uint8_t DRIVE_CURRENT3 = 0x21;

HAL_StatusTypeDef hal_status;
uint8_t buf[6];

static int read_error = 0;
static uint32_t f_ref = 43400000;
int conv_resolution = 28;

int64_t f_sensor0_baseline = 0;
int64_t f_sensor1_baseline = 0;
int64_t f_sensor2_baseline = 0;
int64_t f_sensor3_baseline = 0;

static int32_t f_sensor0 = 0;
static int32_t f_sensor1 = 0;
static int32_t f_sensor2 = 0;
static int32_t f_sensor3 = 0;

double f0 = 0.0;
double f1 = 0.0;
double f2 = 0.0;
double f3 = 0.0;
float current_position = 0.0;
float max_position = 0.0;
uint32_t duty_cycle = 0;

int changes = 0;

static int data_ready = 0;

uint16_t config_reg = 0;
uint16_t mux_config_reg = 0;
uint16_t rcountX;
uint16_t clock_dividerX;
uint16_t drive_currentX;
uint16_t status_reg_read;
uint16_t error_config_read;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

// Helper to set buffer based on a 16-bit value
void setBufferFromValue(uint8_t* buf, uint16_t value) {
    buf[0] = (value >> 8) & 0xFF;
    buf[1] = value & 0xFF;
}
// Read I2C memory and return the 16-bit value
uint16_t readI2CRegister(uint16_t memAddr) {
    uint8_t buf[2];
    HAL_StatusTypeDef hal_status = HAL_I2C_Mem_Read(&hi2c1, LDC_I2C_ADDR, memAddr, I2C_MEMADD_SIZE_8BIT, buf, 2, HAL_MAX_DELAY);
    return ((uint16_t)buf[0] << 8) | buf[1];
}
// Generic I2C write function
HAL_StatusTypeDef writeI2CMem(uint8_t* buf, uint16_t memAddr) {
    return HAL_I2C_Mem_Write(&hi2c1, LDC_I2C_ADDR, memAddr, I2C_MEMADD_SIZE_8BIT, buf, 2, HAL_MAX_DELAY);
}
// Function to write a value to multiple sequential registers
void writeToMultipleRegisters(uint16_t value, uint16_t startAddress, uint16_t endAddress) {
    uint8_t buf[2];
    setBufferFromValue(buf, value);
    for (uint16_t addr = startAddress; addr <= endAddress; ++addr) {
        HAL_StatusTypeDef hal_status = writeI2CMem(buf, addr);
    }
}

int32_t Read_DataXbits(uint8_t msb_addr, uint8_t lsb_addr){
	int32_t f_sensor;

	hal_status = HAL_I2C_Mem_Read(&hi2c1, LDC_I2C_ADDR, msb_addr, I2C_MEMADD_SIZE_8BIT, buf, 2, HAL_MAX_DELAY);
	if (hal_status == HAL_OK) {
	  f_sensor = (int32_t)buf[0] << 24 | (int32_t)buf[1] << 16;
	} else {read_error = 1;}

	hal_status = HAL_I2C_Mem_Read(&hi2c1, LDC_I2C_ADDR, lsb_addr, I2C_MEMADD_SIZE_8BIT, buf, 2, HAL_MAX_DELAY);
	if (hal_status == HAL_OK) {
		f_sensor |= (int32_t)buf[0] << 8 | (int32_t)buf[1] << 0;
	} else {read_error = 1;}
	f_sensor &= 0x0FFFFFFF;  // Masking out the error bits

	return f_sensor;
}

void calibrate(){

	int baselineReadings = 50;
	int iterations = baselineReadings;

	while(iterations>0){
		__NOP();
		if(data_ready){

			f_sensor0_baseline += Read_DataXbits(DATA0_MSB, DATA0_LSB);
			f_sensor1_baseline += Read_DataXbits(DATA1_MSB, DATA1_LSB);
			f_sensor2_baseline += Read_DataXbits(DATA2_MSB, DATA2_LSB);
			f_sensor3_baseline += Read_DataXbits(DATA3_MSB, DATA3_LSB);

			// Read the "STATUS" address to de-assert the interrupt on the LDC side.
			data_ready = 0;

			status_reg_read = readI2CRegister(STATUS);

			iterations--;
		}
	}

	f_sensor0_baseline /= baselineReadings;
	f_sensor1_baseline /= baselineReadings;
	f_sensor2_baseline /= baselineReadings;
	f_sensor3_baseline /= baselineReadings;
}

double correct_neg_f(double f){
	if(f<0)
		return 0;
	else
		return f;
}

void position_decoder(double f0,double f1,double f2,double f3){

	// The right-most case is not resolved yet, when the metal goes beyond the
	// last coil, and the values starts decreasing...

	double right_most = correct_neg_f(f3);
	double right_mid = correct_neg_f(f1);
	double left_mid = correct_neg_f(f2);
	double left_most = correct_neg_f(f0);

	double right_edge = 0;
	double left_edge = 0;

	// Differences in coils (in mm) as measured on the eagle PCB.
	float p_min = 0.0, p0=10.0, p1=10+21.0, p2=0+21+21.0, p3=10+21+21+20.0, p_max=10+21+21+20.0+10;
	max_position = p_max;

	double totalWeight =right_edge + right_most + right_mid + left_mid + left_most + left_edge;
	if (totalWeight == 0)
		totalWeight = 1;

	current_position = (right_edge*p_max + right_most*p0 + right_mid*p1 + left_mid*p2 + left_most*p3 + left_edge*p_min) / totalWeight;
	set_duty_cycle();
}

void set_duty_cycle(){

	duty_cycle = (current_position/max_position)*100;

	TIM1->CCR1 = duty_cycle*10;

	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
}


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
  MX_TIM2_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  /*
   * LDC config
   *
   * Each channel current drive is set independently between 16 µA and 1.6 mA by setting the corresponding IDRIVEx register field
   *
   * */

  /* Mux Config register configuration */
  uint16_t AUTOSCAN_EN = 0b1 << 15;
  uint16_t RR_SEQUENCE = 0b10 << 13;
  uint16_t RESERVED = 0b0001000001 << 3;
  uint16_t DEGLITCH = 0b101 << 0;

  mux_config_reg = AUTOSCAN_EN | RR_SEQUENCE | RESERVED | DEGLITCH;

  // Store it in the buffer for transmission
  buf[0] = (mux_config_reg >> 8) & 0xFF;
  buf[1] = mux_config_reg & 0xFF;
  // Send the buffer over
  hal_status = writeI2CMem(buf, MUX_CONFIG_ADDR);

  /* Resolution configuration */
  uint16_t max_res = 0xFFFF;
  writeToMultipleRegisters(max_res, RCOUNT0, RCOUNT3); // Max Resolution

  /* Setting the Settling Time */
  uint16_t settling_time = 0x0014;
  writeToMultipleRegisters(settling_time, SETTLECOUNT0, SETTLECOUNT3); // Setting the Settling Time

  /* CLOCK_DIVIDERS configuration */
  uint16_t clock_divider = 0x1001;

  writeToMultipleRegisters(clock_divider, CLOCK_DIVIDERS0, CLOCK_DIVIDERS3); // CLOCK_DIVIDERS configuration

  /* DRIVE CURRENT configuration */
  uint16_t IDRIVE = 0b11011 << 11;
  uint16_t INIT_IDRIVE = 0b00000 << 6;
  uint16_t DR_RESERVED = 0b000000 << 0;
  uint16_t drive_cur_reg = drive_cur_reg = IDRIVE | INIT_IDRIVE | DR_RESERVED;

  writeToMultipleRegisters(drive_cur_reg, DRIVE_CURRENT0, DRIVE_CURRENT3); // DRIVE CURRENT configuration

  /* Config register configuration */
  uint16_t SLEEP_MODE_EN = 0b1 << 13;
  uint16_t RP_OVERRIDE_EN = 0b1 << 12;
  uint16_t SENSOR_ACTIVATE_SEL = 0b1 << 11;

  // read register
  config_reg = readI2CRegister(CONFIG_ADDR);
  // Store it in the buffer for transmission
  CLEAR_BIT(config_reg, SENSOR_ACTIVATE_SEL);
  CLEAR_BIT(config_reg, SLEEP_MODE_EN);
  SET_BIT(config_reg, RP_OVERRIDE_EN);

  buf[0] = (config_reg >> 8) & 0xFF;
  buf[1] = config_reg & 0xFF;
  hal_status = writeI2CMem(buf, CONFIG_ADDR);

  /* Interrupt Setup */
  uint16_t DRDY_2INT = 0b1 << 0;
  uint16_t error_config_reg = DRDY_2INT;
  buf[0] = (error_config_reg >> 8) & 0xFF;
  buf[1] = error_config_reg & 0xFF;

  hal_status = writeI2CMem(buf, ERROR_CONFIG);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  /* Read registers before looping */
  mux_config_reg = readI2CRegister(MUX_CONFIG_ADDR);
  config_reg = readI2CRegister(CONFIG_ADDR);
  rcountX = readI2CRegister(RCOUNT0);
  clock_dividerX = readI2CRegister(CLOCK_DIVIDERS0);
  drive_currentX = readI2CRegister(DRIVE_CURRENT0);

  status_reg_read = readI2CRegister(STATUS);
  error_config_read = readI2CRegister(ERROR_CONFIG);

  calibrate();

  while (1)
  {
	  __NOP();
	  if(data_ready){
		  __NOP();
		  // Read "STATUS" to de-assert the interrupt
		  data_ready = 0;
		  hal_status = HAL_I2C_Mem_Read(&hi2c1, LDC_I2C_ADDR, STATUS, I2C_MEMADD_SIZE_8BIT, buf, 2, HAL_MAX_DELAY);
		  status_reg_read = (uint16_t)buf[0] << 8 | (buf[1]);

		  f_sensor0 = Read_DataXbits(DATA0_MSB, DATA0_LSB) - (int32_t)f_sensor0_baseline;
		  f_sensor1 = Read_DataXbits(DATA1_MSB, DATA1_LSB) - (int32_t)f_sensor1_baseline;
		  f_sensor2 = Read_DataXbits(DATA2_MSB, DATA2_LSB) - (int32_t)f_sensor2_baseline;
		  f_sensor3 = Read_DataXbits(DATA3_MSB, DATA3_LSB) - (int32_t)f_sensor3_baseline;

		  f0 = f_sensor0/400.0;
		  f1 = f_sensor1/400.0;
		  f2 = f_sensor2/400.0;
		  f3 = f_sensor3/400.0;

		  position_decoder(f0,f1,f2,f3);
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
  hi2c1.Init.ClockSpeed = 400000;
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 800;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC14 PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PD0 PD1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA5 PA6 PA7
                           PA9 PA10 PA11 PA12
                           PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7
                          |GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12
                          |GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB10
                           PB11 PB12 PB13 PB14
                           PB15 PB3 PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10
                          |GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14
                          |GPIO_PIN_15|GPIO_PIN_3|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure peripheral I/O remapping */
  __HAL_AFIO_REMAP_PD01_ENABLE();

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == GPIO_PIN_5) {
	  data_ready = 1;
  } else {
      __NOP();
  }
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

#ifdef  USE_FULL_ASSERT
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
