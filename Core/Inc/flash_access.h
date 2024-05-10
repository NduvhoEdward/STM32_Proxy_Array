// /*
//  * flash_access.h
//  *
//  *  Created on: May 3, 2024
//  *      Author: NduvhoEdward
//  *
//  * These functions is adapted from ControllersTech.com
//  * Original source:
//  https://github.com/controllerstech/STM32/tree/master/FLASH_PROGRAM/F1%20SERIES
//  * Licensed under the GNU General Public License version 3.
//  */

#ifndef INC_FLASH_PAGE_F1_H_
#define INC_FLASH_PAGE_F1_H_

#include "stm32f1xx_hal.h"

uint32_t Flash_Write_Data(uint32_t StartPageAddress, uint32_t *Data,
                          uint16_t numberofwords);

void Flash_Read_Data(uint32_t StartPageAddress, uint32_t *RxBuf,
                     uint16_t numberofwords);

#endif /* INC_FLASH_PAGE_F1_H_ */