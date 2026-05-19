#ifndef __IO_SPI_H__
#define __IO_SPI_H__

#include "main.h"

/*
 * Legacy CS5530 bit-banged SPI GPIO definitions are disabled during
 * CS5552 migration. Keep the file and public function names so the
 * project structure remains stable until the old source group is removed.
 */
#if 0
#define NSS_RESET   HAL_GPIO_WritePin(P_ADCCS1_GPIO_Port, P_ADCCS1_Pin, GPIO_PIN_RESET)
#define NSS_SET     HAL_GPIO_WritePin(P_ADCCS1_GPIO_Port, P_ADCCS1_Pin, GPIO_PIN_SET)
#define CS1_RESET   HAL_GPIO_WritePin(P_ADCCS1_GPIO_Port, P_ADCCS1_Pin, GPIO_PIN_RESET)
#define CS1_SET     HAL_GPIO_WritePin(P_ADCCS1_GPIO_Port, P_ADCCS1_Pin, GPIO_PIN_SET)
#define CS2_RESET   HAL_GPIO_WritePin(P_ADCCS2_GPIO_Port, P_ADCCS2_Pin, GPIO_PIN_RESET)
#define CS2_SET     HAL_GPIO_WritePin(P_ADCCS2_GPIO_Port, P_ADCCS2_Pin, GPIO_PIN_SET)
#define CS3_RESET   HAL_GPIO_WritePin(P_ADCCS3_GPIO_Port, P_ADCCS3_Pin, GPIO_PIN_RESET)
#define CS3_SET     HAL_GPIO_WritePin(P_ADCCS3_GPIO_Port, P_ADCCS3_Pin, GPIO_PIN_SET)
#define SCK_RESET   HAL_GPIO_WritePin(P_ADCSCLK_GPIO_Port, P_ADCSCLK_Pin, GPIO_PIN_RESET)
#define SCK_SET     HAL_GPIO_WritePin(P_ADCSCLK_GPIO_Port, P_ADCSCLK_Pin, GPIO_PIN_SET)
#define SDO_READ    HAL_GPIO_ReadPin(P_ADCMISO_GPIO_Port, P_ADCMISO_Pin)
#define SDI_RESET   HAL_GPIO_WritePin(P_ADCMOSI_GPIO_Port, P_ADCMOSI_Pin, GPIO_PIN_RESET)
#define SDI_SET     HAL_GPIO_WritePin(P_ADCMOSI_GPIO_Port, P_ADCMOSI_Pin, GPIO_PIN_SET)
#endif

void spi_io_Initl(void);
void spi_Write_Byte(unsigned char byte_value);
unsigned char spi_Read_Byte(void);

#endif