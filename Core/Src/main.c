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

#include "LDC1614.h"
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

HAL_StatusTypeDef hal_status;

static int read_error = 0;
int conv_resolution = 28;

int32_t f_sensor0_baseline = 0;
int32_t f_sensor1_baseline = 0;
int32_t f_sensor2_baseline = 0;
int32_t f_sensor3_baseline = 0;

static int32_t f_sensor0 = 0;
static int32_t f_sensor1 = 0;
static int32_t f_sensor2 = 0;
static int32_t f_sensor3 = 0;

double f0 = 0.0;
double f1 = 0.0;
double f2 = 0.0;
double f3 = 0.0;
double freq_sum = 0.0;

double current_position = 0.0;
float corrected_cur_pos = 0;
float section_1_threshold = 19.0;
float section_2_threshold = 39.3;

float max_position = 0.0;
float duty_cycle = 0;

int data_ready = 0;

// Debug vars
uint16_t config_reg;
uint16_t error_reg;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
void set_duty_cycle();

void retrieve_calibration_data() {
  // Read the calibration data from the flash
  uint32_t flash_data[4];
  // Flash_Read_Data(0x08010000, flash_data, 4);
  LDC1614_Use_CalibrationData(flash_data);

  f_sensor0_baseline = flash_data[0];
  f_sensor1_baseline = flash_data[1];
  f_sensor2_baseline = flash_data[2];
  f_sensor3_baseline = flash_data[3];
}

double correct_neg_f(double f) {
  if (f < 0)
    return 0;
  else
    return f;
}
// Define the polynomial functions
double section_1_poln(double x) {
  return 0.000152012 * x * x * x * x * x - 0.00745952 * x * x * x * x +
         0.1386879 * x * x * x - 1.2114062 * x * x + 5.5180193 * x - 5.1243179;
}

double section_2_poln(double x) {
  return 0.000153428 * x * x * x * x * x - 0.022352 * x * x * x * x +
         1.289149 * x * x * x - 36.796475 * x * x + 520.31012 * x - 2893.7671;
}

double section_3_poln(double x) {
  return 0.00017577 * x * x * x * x * x - 0.04326238 * x * x * x * x +
         4.2515465 * x * x * x - 208.51761 * x * x + 5104.0767 * x - 49840.312;
}
void position_decoder(double f0, double f1, double f2, double f3) {
  double right_most = correct_neg_f(f3);
  double right_mid = correct_neg_f(f1);
  double left_mid = correct_neg_f(f2);
  double left_most = correct_neg_f(f0);

  // Differences in coils (in mm) as measured on the eagle PCB.
  float p0 = 0.0, p1 = 0 + 20.0, p2 = 0 + 20 + 20.0, p3 = 0 + 20 + 20 + 20.0;
  max_position = p3;

  double total_freq = right_most + right_mid + left_mid + left_most;

  freq_sum += total_freq;

  if (total_freq == 0) total_freq = 1;  // Prevent a div by ZERO

  current_position =
      (right_most * p0 + right_mid * p1 + left_mid * p2 + left_most * p3) /
      total_freq;

  if (current_position < section_1_threshold) {
    corrected_cur_pos = section_1_poln(current_position);
  } else if (current_position < section_2_threshold) {
    corrected_cur_pos = section_2_poln(current_position);
  } else {
    corrected_cur_pos = section_3_poln(current_position);
  }

  set_duty_cycle();
}

void set_duty_cycle() {
  duty_cycle = (corrected_cur_pos / max_position);

  if (duty_cycle > 1.0f) duty_cycle = 1.0f;
  if (duty_cycle < 0.0f) duty_cycle = 0.0f;

  TIM1->CCR1 = duty_cycle * (TIM1->ARR);  // To the op-amp
  TIM2->CCR1 = duty_cycle * (TIM2->ARR);  // To the
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// TESTING CODE
#define MAX_DATA_POINTS 200
struct SensorData {
  uint32_t timestamp;
  uint16_t out_duty_cycle;
  float target_position;
  double f_isum;
};

struct SensorData sensor_data[MAX_DATA_POINTS];
uint16_t data_index = 0;

void collect_data(uint32_t timestamp) {
  if (data_index < MAX_DATA_POINTS) {
    sensor_data[data_index].timestamp = timestamp;
    sensor_data[data_index].out_duty_cycle = current_position;
    sensor_data[data_index].target_position = corrected_cur_pos;
    sensor_data[data_index].f_isum = freq_sum / 1000;
    data_index++;
  }
}

int enough_samples() { return (data_index >= MAX_DATA_POINTS); }

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
  /* USER CODE BEGIN 1 */
  uint8_t buf[6];
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick.
   */
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
   * Each channel current drive is set independently between 16 µA and 1.6 mA by
   * setting the corresponding IDRIVEx register field
   *
   * */

  /* Mux Config register configuration */
  uint16_t AUTOSCAN_EN = 0b1 << 15;
  uint16_t RR_SEQUENCE = 0b10 << 13;
  uint16_t RESERVED = 0b0001000001 << 3;
  uint16_t DEGLITCH = 0b101 << 0;

  uint16_t mux_config_reg = AUTOSCAN_EN | RR_SEQUENCE | RESERVED | DEGLITCH;

  LDC1614_Configure_MUX(mux_config_reg);

  /* Resolution configuration */
  uint16_t max_res = 0xFFFF;
  // writeToMultipleRegisters(max_res, RCOUNT0, RCOUNT3);  // Max Resolution
  LDC1614_Set_Resolution(max_res);

  /* Setting the Settling Time */
  uint16_t settling_time = 0x0014;
  LDC1614_Set_SettlingTime(settling_time);

  /* CLOCK_DIVIDERS configuration */
  uint16_t clock_divider = 0x1001;
  LDC1614_Set_ClockDividers(clock_divider);

  /* DRIVE CURRENT configuration */
  uint16_t IDRIVE = 0b11011 << 11;
  uint16_t INIT_IDRIVE = 0b00000 << 6;
  uint16_t DR_RESERVED = 0b000000 << 0;
  uint16_t drive_cur_reg = drive_cur_reg = IDRIVE | INIT_IDRIVE | DR_RESERVED;

  LDC1614_Set_DriveCurrent(drive_cur_reg);

  LDC1614_Interrupt_init();
  LDC1614_Init_Common_Config();

  LDC1614_WakeUP();

  if (HAL_GPIO_ReadPin(GPIOA, Calib_Trigger_Pin) == GPIO_PIN_RESET) {
    // Calibrate the sensors
    LDC1614_Calibrate();
    __NOP();
    __NOP();
    __NOP();
  }
  retrieve_calibration_data();

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    __NOP();
    if (data_ready) {
      data_ready = 0;
      LDC1614_DeAssert_Interrupt();

      f_sensor0 = LDC1614_Read_SensorData(0) - (int32_t)f_sensor0_baseline;
      f_sensor1 = LDC1614_Read_SensorData(1) - (int32_t)f_sensor1_baseline;
      f_sensor2 = LDC1614_Read_SensorData(2) - (int32_t)f_sensor2_baseline;
      f_sensor3 = LDC1614_Read_SensorData(3) - (int32_t)f_sensor3_baseline;

      f0 = f_sensor0;
      f1 = f_sensor1;
      f2 = f_sensor2;
      f3 = f_sensor3;

      position_decoder(f0, f1, f2, f3);
      __NOP();
      // TESTING CODE
      //      uint32_t current_time = HAL_GetTick();
      //      collect_data(current_time);
    }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }

  // export_data();

  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
    Error_Handler();
  }
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {
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
  if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
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
static void MX_TIM1_Init(void) {
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
  htim1.Init.Period = 799;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK) {
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
static void MX_TIM2_Init(void) {
  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 799;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK) {
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
static void MX_GPIO_Init(void) {
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
  GPIO_InitStruct.Pin = GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PD0 PD1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : Calib_Trigger_Pin */
  GPIO_InitStruct.Pin = Calib_Trigger_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(Calib_Trigger_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA5 PA6 PA7
                           PA9 PA10 PA11 PA12
                           PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 |
                        GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 |
                        GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB10
                           PB11 PB12 PB13 PB14
                           PB15 PB3 PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_10 |
                        GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 |
                        GPIO_PIN_15 | GPIO_PIN_3 | GPIO_PIN_8 | GPIO_PIN_9;
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
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if (GPIO_Pin == GPIO_PIN_5) {
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
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1) {
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
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
