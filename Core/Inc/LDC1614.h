/*
 * LDC1614.h
 *
 *  Created on: Apr 30, 2024
 *      Author: NduvhoEdward
 */

#ifndef INC_LDC1614_H_
#define INC_LDC1614_H_

#include <stdint.h>

#include "stm32f1xx_hal.h"

// Variables in the main.c file
extern I2C_HandleTypeDef hi2c1;
extern HAL_StatusTypeDef hal_status;

void LDC1614_WakeUp();
void LDC1614_Sleep();
void LDC1614_Init_Common_Config();
void LDC1614_DeAssert_Interrupt();
void LDC1614_Interrupt_init();

uint16_t LDC1614_Error_Register();
uint16_t LDC1614_Config_Register();

uint16_t LDC1614_Read_Register(uint8_t regAddr);
void LDC1614_Write_Register(uint8_t regAddr, uint16_t value);
void LDC1614_Configure_MUX(uint16_t config);
void LDC1614_Set_SettlingTime(uint16_t settlingTime);
void LDC1614_Set_Resolution(uint16_t resolution);
void LDC1614_Set_DriveCurrent(uint16_t driveCurrent);
void LDC1614_Set_ClockDividers(uint16_t clockDividers);
int32_t LDC1614_Read_SensorData(uint8_t channel);

#endif /* INC_LDC1614_H_ */
