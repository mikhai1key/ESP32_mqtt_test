/*
 * mpu6050_sensor.c
 *
 *  Created on: 21 но€б. 2021 г.
 *      Author: mikhail
 */

#include "mpu6050_sensor.h"

#define I2C_MASTER_SCL_IO 22               					/*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO 21              			 		/*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM 	I2C_NUM_0							/*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ 100000        					/*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0                         /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0                         /*!< I2C master doesn't need buffer */

#define DATA_LENGTH 512                  /*!< Data buffer length of test buffer */
#define RW_TEST_LENGTH 128               /*!< Data length for r/w test, [0,DATA_LENGTH] */
#define DELAY_TIME_BETWEEN_ITEMS_MS 1000 /*!< delay time between different test items */

#define MPU_REG_GYROX 				0x43								/**/
#define MPU_REG_GYROY	 			0x45								/**/
#define MPU_REG_GYROZ 				0x47								/**/
#define MPU_REG_TEST 				0x10								/**/
#define MPU_REG_WHO_A_MI	 		0x75								/**/

#define MPU6050_SENSOR_ADDR 		MPU6050_SENSOR_ADDR0  			/*!< slave address for MPU6050 sensor */
#define MPU6050_SENSOR_ADDR0 		0x68  							/*!< slave address for MPU6050 sensor */
#define MPU6050_SENSOR_ADDR1 		0x69  							/*!< slave address for MPU6050 sensor */
/* MPU6050 registers */
#define MPU6050_AUX_VDDIO			0x01
#define MPU6050_SMPLRT_DIV			0x19
#define MPU6050_CONFIG				0x1A
#define MPU6050_GYRO_CONFIG			0x1B
#define MPU6050_ACCEL_CONFIG		0x1C
#define MPU6050_MOTION_THRESH		0x1F
#define MPU6050_INT_PIN_CFG			0x37
#define MPU6050_INT_ENABLE			0x38
#define MPU6050_INT_STATUS			0x3A
#define MPU6050_ACCEL_XOUT_H		0x3B
#define MPU6050_ACCEL_XOUT_L		0x3C
#define MPU6050_ACCEL_YOUT_H		0x3D
#define MPU6050_ACCEL_YOUT_L		0x3E
#define MPU6050_ACCEL_ZOUT_H		0x3F
#define MPU6050_ACCEL_ZOUT_L		0x40
#define MPU6050_TEMP_OUT_H			0x41
#define MPU6050_TEMP_OUT_L			0x42
#define MPU6050_GYRO_XOUT_H			0x43
#define MPU6050_GYRO_XOUT_L			0x44
#define MPU6050_GYRO_YOUT_H			0x45
#define MPU6050_GYRO_YOUT_L			0x46
#define MPU6050_GYRO_ZOUT_H			0x47
#define MPU6050_GYRO_ZOUT_L			0x48
#define MPU6050_MOT_DETECT_STATUS	0x61
#define MPU6050_SIGNAL_PATH_RESET	0x68
#define MPU6050_MOT_DETECT_CTRL		0x69
#define MPU6050_USER_CTRL			0x6A
#define MPU6050_PWR_MGMT_1			0x6B
#define MPU6050_PWR_MGMT_2			0x6C
#define MPU6050_FIFO_COUNTH			0x72
#define MPU6050_FIFO_COUNTL			0x73
#define MPU6050_FIFO_R_W			0x74
#define MPU6050_WHO_AM_I			0x75


#define MPU6050_CMD_START CONFIG_BH1750_OPMODE   			/*!< Operation mode */
#define ESP_SLAVE_ADDR CONFIG_I2C_SLAVE_ADDRESS 			/*!< ESP32 slave address, you can set any 7bit value */
#define WRITE_BIT I2C_MASTER_WRITE              			/*!< I2C master write */
#define READ_BIT I2C_MASTER_READ                			/*!< I2C master read */
#define ACK_CHECK_EN 0x1                        			/*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0                       			/*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                             			/*!< I2C ack value */
#define NACK_VAL 0x1                            			/*!< I2C nack value */

esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
        // .clk_flags = 0,          /*!< Optional, you can use I2C_SCLK_SRC_FLAG_* flags to choose i2c source clock here. */
    };
    esp_err_t err = i2c_param_config(i2c_master_port, &conf);
    if (err != ESP_OK) {
        return err;
    }
    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

/**
 * @brief test code to operate on MPU6050 sensor
 *
 * 1. set operation mode(e.g One time L-resolution mode)
 * _________________________________________________________________
 * | start | slave_addr + wr_bit + ack | write 1 byte + ack  | stop |
 * --------|---------------------------|---------------------|------|
 * 2. wait more than 24 ms
 * 3. read data
 * __________________________________________________________________
 * | start | slave_addr + rd_bit + ack | read 1 byte + nack |  stop |
 * --------|---------------------------|--------------------|-------|
 */
 esp_err_t i2c_master_sensor_test(i2c_port_t i2c_num, uint8_t *data_h, uint8_t *data_l)
{
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    // send sensor_id
    i2c_master_write_byte(cmd, MPU6050_SENSOR_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
    // send reg_addr
    i2c_master_write_byte(cmd, MPU_REG_WHO_A_MI, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        return ret;
    }
    vTaskDelay(30 / portTICK_RATE_MS);
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, MPU6050_SENSOR_ADDR << 1 | READ_BIT, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, data_h, NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

 esp_err_t i2c_master_write_reg16(i2c_port_t i2c_num, uint8_t reg_addr , uint8_t data_h, uint8_t data_l)
 {
	 int ret;
	 i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	 i2c_master_start(cmd);
	 // send sensor_id
	 i2c_master_write_byte(cmd, MPU6050_SENSOR_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
	 // send reg_addr
	 i2c_master_write_byte(cmd, MPU_REG_WHO_A_MI, ACK_CHECK_EN);
	 i2c_master_stop(cmd);
	 ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
	 i2c_cmd_link_delete(cmd);
	 if (ret != ESP_OK)
	 {
		 return ret;
	 }
	 vTaskDelay(30 / portTICK_RATE_MS);
	 cmd = i2c_cmd_link_create();
	 i2c_master_start(cmd);
	 i2c_master_write_byte(cmd, MPU6050_SENSOR_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
	 i2c_master_write_byte(cmd, data_h, ACK_VAL);
	 i2c_master_write_byte(cmd, data_l, NACK_VAL);
	 i2c_master_stop(cmd);
	 ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
	 i2c_cmd_link_delete(cmd);
	 return ret;
 }

 esp_err_t mpu6050_init(i2c_port_t i2c_num)
 {
	 int ret = 0;
	 uint8_t data;
	 //SD_MPU6050_SetPWR
	 data = 0;
	 ret = i2c_master_write_reg8(i2c_num, MPU6050_PWR_MGMT_1, data);
	 vTaskDelay(30 / portTICK_RATE_MS);
	 //SD_MPU6050_SetDataRate
	 data = SD_MPU6050_DataRate_1KHz;
	 ret =i2c_master_write_reg8(i2c_num, MPU6050_SMPLRT_DIV, data);
	 vTaskDelay(30 / portTICK_RATE_MS);
	 //SD_MPU6050_ACCEL_CONFIG
	 data = 0x00;
	 ret = i2c_master_write_reg8(i2c_num, MPU6050_ACCEL_CONFIG, data);
	 vTaskDelay(30 / portTICK_RATE_MS);
	 //SD_MPU6050_SetGyroscope
	 data = (uint8_t)(SD_MPU6050_Gyroscope_250s << 3);
	 ret = i2c_master_write_reg8(i2c_num, MPU6050_GYRO_CONFIG, data);
	 vTaskDelay(30 / portTICK_RATE_MS);
	 return ret;
 }


 esp_err_t i2c_master_write_reg8(i2c_port_t i2c_num, uint8_t reg_addr , uint8_t data)
 {
	 int ret;
	 i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	 i2c_master_start(cmd);
	 i2c_master_write_byte(cmd, (MPU6050_SENSOR_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
	 i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);
	 i2c_master_write_byte(cmd, data, ACK_CHECK_EN);
	 i2c_master_stop(cmd);
	 ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
	 i2c_cmd_link_delete(cmd);
	 return ret;

 }

 esp_err_t i2c_master_read_reg8(i2c_port_t i2c_num, uint8_t reg_addr , uint8_t *data)
 {
	 int ret;
	 i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	 i2c_master_start(cmd);
	 i2c_master_write_byte(cmd, (MPU6050_SENSOR_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
	 i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);
	 i2c_master_stop(cmd);
	 ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
	 i2c_cmd_link_delete(cmd);
	 if (ret != ESP_OK) {
	     return ret;
	 }
	 vTaskDelay(30 / portTICK_RATE_MS);
	 cmd = i2c_cmd_link_create();
	 i2c_master_start(cmd);
	 i2c_master_write_byte(cmd, (MPU6050_SENSOR_ADDR << 1) | READ_BIT, ACK_CHECK_EN);
	 i2c_master_read_byte(cmd, data, NACK_VAL);
	 i2c_master_stop(cmd);
	 ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
	 i2c_cmd_link_delete(cmd);
	 return ret;
 }

 esp_err_t i2c_master_read_reg16(i2c_port_t i2c_num, uint8_t reg_addr , uint8_t *data_h, uint8_t *data_l)
 {
	 int ret;
	 i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	 i2c_master_start(cmd);
	 i2c_master_write_byte(cmd, (MPU6050_SENSOR_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
	 i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);
	 i2c_master_stop(cmd);
	 ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
	 i2c_cmd_link_delete(cmd);
	 if (ret != ESP_OK) {
	     return ret;
	 }
	 vTaskDelay(30 / portTICK_RATE_MS);
	 cmd = i2c_cmd_link_create();
	 i2c_master_start(cmd);
	 i2c_master_write_byte(cmd, (MPU6050_SENSOR_ADDR << 1) | READ_BIT, ACK_CHECK_EN);
	 i2c_master_read_byte(cmd, data_h, ACK_VAL);
	 i2c_master_read_byte(cmd, data_l, NACK_VAL);
	 i2c_master_stop(cmd);
	 ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
	 i2c_cmd_link_delete(cmd);
	 return ret;
 }


 esp_err_t i2c_master_read_gyro(i2c_port_t i2c_num, uint16_t *data_x, uint16_t *data_y, uint16_t *data_z)
 {
	uint8_t sen_data_h = 0, sen_data_l = 0;
	int ret;
	ret = i2c_master_read_reg16(I2C_MASTER_NUM, MPU6050_ACCEL_XOUT_H, &sen_data_h, &sen_data_l);
	*data_x = (uint16_t) (sen_data_h<<8| sen_data_l);
	ret = i2c_master_read_reg16(I2C_MASTER_NUM, MPU6050_ACCEL_YOUT_H, &sen_data_h, &sen_data_l);
	*data_y = (uint16_t) (sen_data_h<<8| sen_data_l);
	ret = i2c_master_read_reg16(I2C_MASTER_NUM, MPU6050_ACCEL_ZOUT_H, &sen_data_h, &sen_data_l);
	*data_z = (uint16_t) (sen_data_h<<8| sen_data_l);

	return ret;
 }
