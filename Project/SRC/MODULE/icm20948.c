/*
 * ICM20948.c
 *
 *  Created on: Oct 26, 2018
 *      Author: cory
 */

#include "main.h"
#include "ICM20948.h"
#include "drv_spi.h"

uint16_t accel_data[3];
uint16_t gyro_data[3];
int16_t mag_data[3];

/*
 *
 * AUX I2C abstraction for magnetometer
 *
 */
void i2c_Mag_write(uint8_t reg, uint8_t value) {
    SPI_GyroSingleWrite(0x7F, 0x30);

    HAL_Delay(1);
    SPI_GyroSingleWrite(0x03, 0x0C);//mode: write

    HAL_Delay(1);
    SPI_GyroSingleWrite(0x04, reg);//set reg addr

    HAL_Delay(1);
    SPI_GyroSingleWrite(0x06, value);//send value

    HAL_Delay(1);
}

static uint8_t ICM_Mag_Read(uint8_t reg) {
    uint8_t Data;
    SPI_GyroSingleWrite(0x7F, 0x30);
    HAL_Delay(1);
    SPI_GyroSingleWrite(0x03, 0x0C | 0x80);
    HAL_Delay(1);
    SPI_GyroSingleWrite(0x04, reg);// set reg addr
    HAL_Delay(1);
    SPI_GyroSingleWrite(0x06, 0xff);//read
    HAL_Delay(1);
    SPI_GyroSingleWrite(0x7F, 0x00);
    SPI_GyroMultiRead(0x3B, &Data, 1);
    HAL_Delay(1);
    return Data;
}

void ICM20948_READ_MAG(int16_t magn[3]) {
    uint8_t mag_buffer[10];

    mag_buffer[0] = ICM_Mag_Read(0x01);

    mag_buffer[1] = ICM_Mag_Read(0x11);
    mag_buffer[2] = ICM_Mag_Read(0x12);
    magn[0] = mag_buffer[1] | mag_buffer[2] << 8;
    mag_buffer[3] = ICM_Mag_Read(0x13);
    mag_buffer[4] = ICM_Mag_Read(0x14);
    magn[1] = mag_buffer[3] | mag_buffer[4] << 8;
    mag_buffer[5] = ICM_Mag_Read(0x15);
    mag_buffer[6] = ICM_Mag_Read(0x16);
    magn[2] = mag_buffer[5] | mag_buffer[6] << 8;

    i2c_Mag_write(0x31, 0x01);
}

/*
 *
 * Read magnetometer
 *
 */
void ICM_ReadMag(int16_t magn[3]) {
    uint8_t mag_buffer[10];

    mag_buffer[0] = ICM_Mag_Read(0x01);

    mag_buffer[1] = ICM_Mag_Read(0x11);
    mag_buffer[2] = ICM_Mag_Read(0x12);
    magn[0] = mag_buffer[1] | mag_buffer[2] << 8;
    mag_buffer[3] = ICM_Mag_Read(0x13);
    mag_buffer[4] = ICM_Mag_Read(0x14);
    magn[1] = mag_buffer[3] | mag_buffer[4] << 8;
    mag_buffer[5] = ICM_Mag_Read(0x15);
    mag_buffer[6] = ICM_Mag_Read(0x16);
    magn[2] = mag_buffer[5] | mag_buffer[6] << 8;

    i2c_Mag_write(0x31, 0x01);
}

/*
 *
 * Sequence to setup ICM290948 as early as possible after power on
 *
 */
void ICM_PowerOn(void) {
    char uart_buffer[200];

    //if (test == whoami) {
    ICM_CSHigh();
    HAL_Delay(10);
    ICM_SelectBank(USER_BANK_0);
    HAL_Delay(10);
    ICM_Disable_I2C();
    HAL_Delay(10);
    ICM_SetClock((uint8_t) CLK_BEST_AVAIL);
    HAL_Delay(10);
    ICM_AccelGyroOff();
    HAL_Delay(20);
    ICM_AccelGyroOn();
    HAL_Delay(10);
    ICM_Initialize();
}

uint16_t ICM_Initialize(void) {
    ICM_SelectBank(USER_BANK_2);
    HAL_Delay(20);
    ICM_SetGyroRateLPF(GYRO_RATE_250, GYRO_LPF_17HZ);
    HAL_Delay(10);

    // Set gyroscope sample rate to 100hz (0x0A) in GYRO_SMPLRT_DIV register (0x00)
    SPI_GyroSingleWrite(0x00, 0x0A);
    HAL_Delay(10);

    // Set accelerometer low pass filter to 136hz (0x11) and the rate to 8G (0x04) in register ACCEL_CONFIG (0x14)
    SPI_GyroSingleWrite(0x14, (0x04 | 0x11));

    // Set accelerometer sample rate to 225hz (0x00) in ACCEL_SMPLRT_DIV_1 register (0x10)
    SPI_GyroSingleWrite(0x10, 0x00);
    HAL_Delay(10);

    // Set accelerometer sample rate to 100 hz (0x0A) in ACCEL_SMPLRT_DIV_2 register (0x11)
    SPI_GyroSingleWrite(0x11, 0x0A);
    HAL_Delay(10);

    ICM_SelectBank(USER_BANK_2);
    HAL_Delay(20);

    // Configure AUX_I2C Magnetometer (onboard ICM-20948)
    SPI_GyroSingleWrite(0x7F, 0x00); // Select user bank 0
    SPI_GyroSingleWrite(0x0F, 0x30); // INT Pin / Bypass Enable Configuration
    SPI_GyroSingleWrite(0x03, 0x20); // I2C_MST_EN
    SPI_GyroSingleWrite(0x7F, 0x30); // Select user bank 3
    SPI_GyroSingleWrite(0x01, 0x4D); // I2C Master mode and Speed 400 kHz
    SPI_GyroSingleWrite(0x02, 0x01); // I2C_SLV0 _DLY_ enable
    SPI_GyroSingleWrite(0x05, 0x81); // enable IIC	and EXT_SENS_DATA==1 Byte

    // Initialize magnetometer
    i2c_Mag_write(0x32, 0x01); // Reset AK8963
    HAL_Delay(1000);
    i2c_Mag_write(0x31, 0x02); // use i2c to set AK8963 working on Continuous measurement mode1 & 16-bit output

    return 1337;
}

void ICM_ReadAccelGyro(void) {
    uint8_t raw_data[12];
    SPI_GyroMultiRead(0x2D, raw_data, 12);

    accel_data[0] = (raw_data[0] << 8) | raw_data[1];
    accel_data[1] = (raw_data[2] << 8) | raw_data[3];
    accel_data[2] = (raw_data[4] << 8) | raw_data[5];

    gyro_data[0] = (raw_data[6] << 8) | raw_data[7];
    gyro_data[1] = (raw_data[8] << 8) | raw_data[9];
    gyro_data[2] = (raw_data[10] << 8) | raw_data[11];

    accel_data[0] = accel_data[0] / 8;
    accel_data[1] = accel_data[1] / 8;
    accel_data[2] = accel_data[2] / 8;

    gyro_data[0] = gyro_data[0] / 250;
    gyro_data[1] = gyro_data[1] / 250;
    gyro_data[2] = gyro_data[2] / 250;
}

void ICM_SelectBank(uint8_t bank) {
    SPI_GyroSingleWrite(USER_BANK_SEL, bank);
}

void ICM_Disable_I2C(void) {
    SPI_GyroSingleWrite(0x03, 0x78);
}

void ICM_SetClock(uint8_t clk) {
    SPI_GyroSingleWrite(PWR_MGMT_1, clk);
}

void ICM_AccelGyroOff(void) {
    SPI_GyroSingleWrite(PWR_MGMT_2, (0x38 | 0x07));
}

void ICM_AccelGyroOn(void) {
    SPI_GyroSingleWrite(0x07, (0x00 | 0x00));
}

bool ICM20948_Detect(void) {
    uint8_t temp;
    SPI_GyroMultiRead(0x00, &temp, 1);
    if (temp == 0xEA) {
        return true;
    } else {
        return false;
    }
}

void ICM_SetGyroRateLPF(uint8_t rate, uint8_t lpf) {
    SPI_GyroSingleWrite(GYRO_CONFIG_1, (rate | lpf));
}
/*
 *
 * Read Accelerometer and Gyro data
 *
 */