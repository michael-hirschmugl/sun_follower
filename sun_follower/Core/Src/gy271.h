/*
 * gy271.h
 *
 *  Created on: Aug 13, 2023
 *      Author: Michael
 */

#ifndef SRC_GY271_H_
#define SRC_GY271_H_

#define GY271_I2C_ADDR (0x0DU << 1) /* Compass */

// Compass Data Output Registers
#define GY271_DATA_X_LSB 0x00
#define GY271_DATA_X_MSB 0x01
#define GY271_DATA_Y_LSB 0x02
#define GY271_DATA_Y_MSB 0x03
#define GY271_DATA_Z_LSB 0x04
#define GY271_DATA_Z_MSB 0x05

// Compass Status Register
#define GY271_STATUS_FLAGS 0x06
#define GY271_STATUS_FLAG_DOR (0x01U << 2)
#define GY271_STATUS_FLAG_OVL (0x01U << 1)
#define GY271_STATUS_FLAG_DRDY (0x01U << 0) /* Data Ready Register */

// Compass Control Register
#define GY271_CONTROL_REGISTER_1 0x09
#define GY271_CONTROL_MODE (0x03U << 0) /* 00 standby, 01 cont */
#define GY271_CONTROL_ODR (0x03U << 2) /* 00 10Hz, 01 50Hz, 10 100Hz, 11 200Hz */
#define GY271_CONTROL_RNG (0x03U << 4) /* full scale: 00 2G, 01 8G */
#define GY271_CONTROL_OSR (0x03U << 6) /* over sampling ratio: 00 512, 01 256, 10 128, 11 64 */

#define GY271_CONTROL_REGISTER_2 0x0B

typedef struct direction {
  int x_vector;
  int y_vector;
  int z_vector;

} direction;

direction vectors;

/* function to set the compass modes */
void Setup_Compass(I2C_HandleTypeDef * hi2c1) {
  uint8_t OSR = 0b00; /* 512 oversampling */
  uint8_t RNG = 0b00; /* 2G full scale */
  uint8_t ODR = 0b00; /* 10Hz */
  uint8_t MODE = 0b01; /* cont */
  uint8_t data[2] = {
    0,
    0
  };

  HAL_Delay(500);
  data[0] = 0x0A;
  data[1] = 0xC1; /* soft reset */
  HAL_I2C_Master_Transmit(hi2c1, GY271_I2C_ADDR, data, 2, 10);

  HAL_Delay(2000);
  data[0] = GY271_CONTROL_REGISTER_2;
  data[1] = 0x01; /* Define Set/Reset period */
  HAL_I2C_Master_Transmit(hi2c1, GY271_I2C_ADDR, data, 2, 10);

  HAL_Delay(500);
  data[0] = GY271_CONTROL_REGISTER_1;
  //data[1] = 0x1D;
  data[1] = (OSR << 6) | (RNG << 4) | (ODR << 2) | (MODE << 0);
  HAL_I2C_Master_Transmit(hi2c1, GY271_I2C_ADDR, data, 2, 10);
}

/* check if compass data is ready */
int Compass_Data_Ready(I2C_HandleTypeDef * hi2c1) {

  uint8_t reg_val[1];

  if (HAL_I2C_Mem_Read(hi2c1, GY271_I2C_ADDR, 0x06U, 1, reg_val, 1, 1000) == HAL_OK) {
    if (reg_val[0] & GY271_STATUS_FLAG_DRDY)
      return 1;
    else
      return reg_val[0];
  } else
    return 0;

}

/* read compass x data */
volatile int Compass_x_read(I2C_HandleTypeDef * hi2c1) {

  uint8_t reg_lsb[1];
  uint16_t lsb;
  uint8_t reg_msb[1];
  uint16_t msb;
  uint32_t buffer;

  HAL_I2C_Mem_Read(hi2c1, GY271_I2C_ADDR, GY271_DATA_X_LSB, 1, reg_lsb, 1, 1000);
  HAL_I2C_Mem_Read(hi2c1, GY271_I2C_ADDR, GY271_DATA_X_MSB, 1, reg_msb, 1, 1000);

  lsb = (uint16_t) reg_lsb[0];
  msb = (uint16_t) reg_msb[0];
  msb = msb << 8;
  buffer = (lsb | msb) << 16;

  //buffer = (int)(((0b0000000011111111)&(uint16_t)reg_lsb[0]) | ((0b1111111100000000)&(((uint16_t)reg_msb[0])<<8)));

  return ((int) buffer) >> 16;

}

/* read compass y data */
volatile int Compass_y_read(I2C_HandleTypeDef * hi2c1) {

  uint8_t reg_lsb[1];
  uint16_t lsb;
  uint8_t reg_msb[1];
  uint16_t msb;
  uint32_t buffer;

  HAL_I2C_Mem_Read(hi2c1, GY271_I2C_ADDR, GY271_DATA_Y_LSB, 1, reg_lsb, 1, 1000);
  HAL_I2C_Mem_Read(hi2c1, GY271_I2C_ADDR, GY271_DATA_Y_MSB, 1, reg_msb, 1, 1000);

  lsb = (uint16_t) reg_lsb[0];
  msb = (uint16_t) reg_msb[0];
  msb = msb << 8;
  buffer = (lsb | msb) << 16;

  //buffer = (int)(((0b0000000011111111)&(uint16_t)reg_lsb[0]) | ((0b1111111100000000)&(((uint16_t)reg_msb[0])<<8)));

  return ((int) buffer) >> 16;

}

/* read compass z data */
volatile int Compass_z_read(I2C_HandleTypeDef * hi2c1) {

  uint8_t reg_lsb[1];
  uint16_t lsb;
  uint8_t reg_msb[1];
  uint16_t msb;
  uint32_t buffer;

  HAL_I2C_Mem_Read(hi2c1, GY271_I2C_ADDR, GY271_DATA_Z_LSB, 1, reg_lsb, 1, 1000);
  HAL_I2C_Mem_Read(hi2c1, GY271_I2C_ADDR, GY271_DATA_Z_MSB, 1, reg_msb, 1, 1000);

  lsb = (uint16_t) reg_lsb[0];
  msb = (uint16_t) reg_msb[0];
  msb = msb << 8;
  buffer = (lsb | msb) << 16;

  //buffer = (int)(((0b0000000011111111)&(uint16_t)reg_lsb[0]) | ((0b1111111100000000)&(((uint16_t)reg_msb[0])<<8)));

  return ((int) buffer) >> 16;
}

int Read_Compass_Data(I2C_HandleTypeDef * hi2c1) {
	vectors.x_vector = Compass_x_read(hi2c1);
	vectors.y_vector = Compass_y_read(hi2c1);
	vectors.z_vector = Compass_z_read(hi2c1);
	return 0;
}

int Get_Compass_x_Vector()
{
	return vectors.x_vector;
}

int Get_Compass_y_Vector()
{
	return vectors.y_vector;
}

int Get_Compass_z_Vector()
{
	return vectors.z_vector;
}

#endif /* SRC_GY271_H_ */