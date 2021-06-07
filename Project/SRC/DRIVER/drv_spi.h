#ifndef __DRV_SPI_H__
#define __DRV_SPI_H__

#include "board.h"

#include "spi.h"

void Spi_Init(void);

void Spi_GyroSingleWrite(uint8_t reg, uint8_t value);
void Spi_GyroMultiRead(uint8_t reg,uint8_t *data, uint8_t length);
void Spi_BaroSingleWrite(uint8_t reg, uint8_t value);
void Spi_BaroMultiRead(uint8_t reg,uint8_t *data, uint8_t length);

#endif










