/*
 * mpu6050_sensor.h
 *
 *  Created on: 21 но€б. 2021 г.
 *      Author: mikhail
 */

#ifndef MAIN_MPU6050_SENSOR_H_
#define MAIN_MPU6050_SENSOR_H_

#include "driver/i2c.h"
#include <stdio.h>
#include "esp_log.h"
#include "sdkconfig.h"
#include "esp_err.h"

#define SD_MPU6050_DataRate_8KHz       0   /*!< Sample rate set to 8 kHz */
#define SD_MPU6050_DataRate_4KHz       1   /*!< Sample rate set to 4 kHz */
#define SD_MPU6050_DataRate_2KHz       3   /*!< Sample rate set to 2 kHz */
#define SD_MPU6050_DataRate_1KHz       7   /*!< Sample rate set to 1 kHz */
#define SD_MPU6050_DataRate_500Hz      15  /*!< Sample rate set to 500 Hz */
#define SD_MPU6050_DataRate_250Hz      31  /*!< Sample rate set to 250 Hz */
#define SD_MPU6050_DataRate_125Hz      63  /*!< Sample rate set to 125 Hz */
#define SD_MPU6050_DataRate_100Hz      79  /*!< Sample rate set to 100 Hz */

/**
 * @defgroup SD_MPU6050_Typedefs
 * @brief    Library Typedefs
 * @{
 */

/**
 * @brief  MPU6050 can have 2 different slave addresses, depends on it's input AD0 pin
 *         This feature allows you to use 2 different sensors with this library at the same time
 */
typedef enum  {
	SD_MPU6050_Device_0 = 0x00, /*!< AD0 pin is set to low */
	SD_MPU6050_Device_1 = 0x02  /*!< AD0 pin is set to high */
} SD_MPU6050_Device;

/**
 * @brief  MPU6050 result enumeration
 */
typedef enum  {
	SD_MPU6050_Result_Ok = 0x00,          /*!< Everything OK */
	SD_MPU6050_Result_Error,              /*!< Unknown error */
	SD_MPU6050_Result_DeviceNotConnected, /*!< There is no device with valid slave address */
	SD_MPU6050_Result_DeviceInvalid       /*!< Connected device with address is not MPU6050 */
} SD_MPU6050_Result;

/**
 * @brief  Parameters for accelerometer range
 */
typedef enum  {
	SD_MPU6050_Accelerometer_2G = 0x00, /*!< Range is +- 2G */
	SD_MPU6050_Accelerometer_4G = 0x01, /*!< Range is +- 4G */
	SD_MPU6050_Accelerometer_8G = 0x02, /*!< Range is +- 8G */
	SD_MPU6050_Accelerometer_16G = 0x03 /*!< Range is +- 16G */
} SD_MPU6050_Accelerometer;

/**
 * @brief  Parameters for gyroscope range
 */
typedef enum {
	SD_MPU6050_Gyroscope_250s = 0x00,  /*!< Range is +- 250 degrees/s */
	SD_MPU6050_Gyroscope_500s = 0x01,  /*!< Range is +- 500 degrees/s */
	SD_MPU6050_Gyroscope_1000s = 0x02, /*!< Range is +- 1000 degrees/s */
	SD_MPU6050_Gyroscope_2000s = 0x03  /*!< Range is +- 2000 degrees/s */
} SD_MPU6050_Gyroscope;

/**
 * @brief  Main MPU6050 structure
 */
typedef struct  {
	/* Private */
	uint8_t Address;         /*!< I2C address of device. */
	float Gyro_Mult;         /*!< Gyroscope corrector from raw data to "degrees/s". Only for private use */
	float Acce_Mult;         /*!< Accelerometer corrector from raw data to "g". Only for private use */
	/* Public */
	int16_t Accelerometer_X; /*!< Accelerometer value X axis */
	int16_t Accelerometer_Y; /*!< Accelerometer value Y axis */
	int16_t Accelerometer_Z; /*!< Accelerometer value Z axis */
	int16_t Gyroscope_X;     /*!< Gyroscope value X axis */
	int16_t Gyroscope_Y;     /*!< Gyroscope value Y axis */
	int16_t Gyroscope_Z;     /*!< Gyroscope value Z axis */
	float   Temperature;       /*!< Temperature in degrees */
	//I2C_HandleTypeDef* I2Cx;
} SD_MPU6050;

/**
 * @brief  Interrupts union and structure
 */
typedef union {
	struct {
		uint8_t DataReady:1;       /*!< Data ready interrupt */
		uint8_t reserved2:2;       /*!< Reserved bits */
		uint8_t Master:1;          /*!< Master interrupt. Not enabled with library */
		uint8_t FifoOverflow:1;    /*!< FIFO overflow interrupt. Not enabled with library */
		uint8_t reserved1:1;       /*!< Reserved bit */
		uint8_t MotionDetection:1; /*!< Motion detected interrupt */
		uint8_t reserved0:1;       /*!< Reserved bit */
	} F;
	uint8_t Status;
} SD_MPU6050_Interrupt;



esp_err_t i2c_master_init(void);

esp_err_t mpu6050_init(i2c_port_t i2c_num);

esp_err_t i2c_master_sensor_test(i2c_port_t i2c_num, uint8_t *data_h, uint8_t *data_l);

esp_err_t i2c_master_read_gyro(i2c_port_t i2c_num, uint16_t *data_x, uint16_t *data_y, uint16_t *data_z);

esp_err_t i2c_master_read_reg16(i2c_port_t i2c_num, uint8_t reg_addr , uint8_t *data_h, uint8_t *data_l);

esp_err_t i2c_master_write_reg8(i2c_port_t i2c_num, uint8_t reg_addr , uint8_t data);

esp_err_t i2c_master_write_reg16(i2c_port_t i2c_num, uint8_t reg_addr , uint8_t data_h, uint8_t data_l);

esp_err_t i2c_master_read_reg8(i2c_port_t i2c_num, uint8_t reg_addr , uint8_t *data);

esp_err_t i2c_master_read_mpu6050(i2c_port_t i2c_num, uint8_t *data);

#endif /* MAIN_MPU6050_SENSOR_H_ */
