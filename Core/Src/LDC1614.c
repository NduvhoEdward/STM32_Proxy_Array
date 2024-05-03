/*
 * LDC1614.c
 *
 *  Created on: Apr 30, 2024
 *      Author: NduvhoEdward */
// Helper to set buffer based on a 16-bit value
#include "LDC1614.h"

#include <stdint.h>

#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_def.h"
#include "stm32f1xx_hal_i2c.h"

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

// Private functions

void setBufferFromValue(uint8_t* buf, uint16_t value) {
  buf[0] = (value >> 8) & 0xFF;
  buf[1] = value & 0xFF;
}

// Read I2C memory and return the 16-bit value
uint16_t readI2CRegister(uint16_t memAddr) {
  uint8_t buf[2];
  HAL_StatusTypeDef hal_status =
      HAL_I2C_Mem_Read(&hi2c1, LDC_I2C_ADDR, memAddr, I2C_MEMADD_SIZE_8BIT, buf,
                       2, HAL_MAX_DELAY);
  return ((uint16_t)buf[0] << 8) | buf[1];
}

// Generic I2C write function
HAL_StatusTypeDef writeI2CMem(uint16_t memAddr, uint8_t* buf) {
  return HAL_I2C_Mem_Write(&hi2c1, LDC_I2C_ADDR, memAddr, I2C_MEMADD_SIZE_8BIT,
                           buf, 2, HAL_MAX_DELAY);
}

// Function to write a value to multiple sequential registers
void writeToMultipleRegisters(uint16_t value, uint16_t startAddress,
                              uint16_t endAddress) {
  uint8_t buf[2];
  setBufferFromValue(buf, value);
  for (uint16_t addr = startAddress; addr <= endAddress; ++addr) {
    HAL_StatusTypeDef hal_status = writeI2CMem(addr, buf);
  }
}

int32_t Read_DataXbits(uint8_t msb_addr, uint8_t lsb_addr) {
  int32_t f_sensor;
  union {
    uint8_t error;
    struct {
      unsigned amp : 1;
      unsigned wd : 1;
      unsigned or : 1;
      unsigned ur : 1;
    } flags;
  } u_error;

  uint8_t buf[6];
  hal_status = HAL_I2C_Mem_Read(&hi2c1, LDC_I2C_ADDR, msb_addr,
                                I2C_MEMADD_SIZE_8BIT, buf, 2, HAL_MAX_DELAY);
  if (hal_status == HAL_OK) {
    f_sensor = (int32_t)buf[0] << 24 | (int32_t)buf[1] << 16;

  } else {
    // read_error = 1;
  }

  hal_status = HAL_I2C_Mem_Read(&hi2c1, LDC_I2C_ADDR, lsb_addr,
                                I2C_MEMADD_SIZE_8BIT, buf, 2, HAL_MAX_DELAY);
  if (hal_status == HAL_OK) {
    f_sensor |= (int32_t)buf[0] << 8 | (int32_t)buf[1] << 0;
  } else {
    // read_error = 1;
  }
  u_error.error = (f_sensor & 0xf0000000) >> 28;
  f_sensor &= 0x0FFFFFFF;  // Masking out the error bits

  if (u_error.error) {
    __NOP();
  }
  return f_sensor;
}

// Public Functions

void LDC1614_Init_Common_Config() {
  uint16_t RP_OVERRIDE_EN = 0b1 << 12;
  uint16_t SENSOR_ACTIVATE_SEL = 0b1 << 11;

  // read register
  uint16_t config_reg = readI2CRegister(CONFIG_ADDR);
  // Store it in the buffer for transmission
  CLEAR_BIT(config_reg, SENSOR_ACTIVATE_SEL);
  SET_BIT(config_reg, RP_OVERRIDE_EN);
  // Send the modified reg. to the LDC chip
  LDC1614_Write_Register(CONFIG_ADDR, config_reg);
}

void LDC1614_WakeUP(void) {
  uint16_t SLEEP_MODE_EN = 0b1 << 13;
  // read register
  uint16_t config_reg = LDC1614_Read_Register(CONFIG_ADDR);
  // Clear the sleep mode bit to disable sleep mode
  CLEAR_BIT(config_reg, SLEEP_MODE_EN);

  LDC1614_Write_Register(CONFIG_ADDR, config_reg);
}

void LDC1614_Interrupt_init() {
  /* Interrupt Setup */
  uint16_t DRDY_2INT = 0b1 << 0;
  uint16_t error_config_reg = DRDY_2INT;
  LDC1614_Write_Register(ERROR_CONFIG, error_config_reg);
}

void LDC1614_Sleep(void) {
  uint16_t SLEEP_MODE_EN = 0b1 << 13;
  // read register
  uint16_t config_reg = LDC1614_Read_Register(CONFIG_ADDR);
  // Clear the sleep mode bit to disable sleep mode
  SET_BIT(config_reg, SLEEP_MODE_EN);

  LDC1614_Write_Register(CONFIG_ADDR, config_reg);
}

void LDC1614_DeAssert_Interrupt(void) {
  // Read the "STATUS" address to de-assert the interrupt on the LDC side.
  uint16_t status_reg_read = readI2CRegister(STATUS);
}

uint16_t LDC1614_Read_Register(uint8_t regAddr) {
  return readI2CRegister(regAddr);
}
uint16_t LDC1614_Config_Register() {
  return LDC1614_Read_Register(CONFIG_ADDR);
}
uint16_t LDC1614_Error_Register() {
  return LDC1614_Read_Register(ERROR_CONFIG);
}

void LDC1614_Write_Register(uint8_t regAddr, uint16_t value) {
  uint8_t buf[2];
  setBufferFromValue(buf, value);
  writeI2CMem(regAddr, buf);
}

/*To be abstracted further*/
void LDC1614_Configure_MUX(uint16_t config) {
  LDC1614_Write_Register(MUX_CONFIG_ADDR, config);
}
/*To be abstracted further*/
void LDC1614_Set_SettlingTime(uint16_t settlingTime) {
  writeToMultipleRegisters(settlingTime, SETTLECOUNT0, SETTLECOUNT3);
}
/*To be abstracted further*/
void LDC1614_Set_Resolution(uint16_t resolution) {
  writeToMultipleRegisters(resolution, RCOUNT0, RCOUNT3);
}
/*To be abstracted further*/
void LDC1614_Set_DriveCurrent(uint16_t driveCurrent) {
  writeToMultipleRegisters(driveCurrent, DRIVE_CURRENT0, DRIVE_CURRENT3);
}
/*To be abstracted further*/
void LDC1614_Set_ClockDividers(uint16_t clockDividers) {
  writeToMultipleRegisters(clockDividers, CLOCK_DIVIDERS0, CLOCK_DIVIDERS3);
}

int32_t LDC1614_Read_SensorData(uint8_t channel) {
  uint8_t msb_addr = DATA0_MSB + channel * 2;
  uint8_t lsb_addr = DATA0_LSB + channel * 2;
  return Read_DataXbits(msb_addr, lsb_addr);
}
