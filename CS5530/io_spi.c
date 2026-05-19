/*
 * Legacy CS5530 bit-banged SPI implementation is disabled during
 * CS5552 migration. Keep no-op stubs here so the old source file can
 * remain in the project until the CS5552 hardware SPI driver is ready.
 */

#include "io_spi.h"

#if 0
void spi_io_Initl(void)
{
    NSS_SET;
    CS1_SET;
    CS2_SET;
    CS3_SET;
    SCK_RESET;
    SDI_SET;
}

void spi_Write_Byte(unsigned char byte_value)
{
    unsigned char bit_index;

    SCK_RESET;
    for (bit_index = 0; bit_index < 8; bit_index++) {
        if ((byte_value & 0x80U) == 0x80U) {
            SDI_SET;
        } else {
            SDI_RESET;
        }

        SCK_SET;
        SCK_RESET;
        byte_value <<= 1;
    }
}

unsigned char spi_Read_Byte(void)
{
    unsigned char byte_value = 0;
    unsigned char bit_index;

    SCK_RESET;
    for (bit_index = 0; bit_index < 8; bit_index++) {
        byte_value <<= 1;
        SCK_SET;
        if (SDO_READ == GPIO_PIN_SET) {
            byte_value |= 0x01U;
        }
        SCK_RESET;
    }

    return byte_value;
}
#endif

void spi_io_Initl(void)
{
}

void spi_Write_Byte(unsigned char byte_value)
{
    (void)byte_value;
}

unsigned char spi_Read_Byte(void)
{
    return 0;
}