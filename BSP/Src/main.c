/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "application.hpp"
#include <stdio.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

//#define SET_TIME_ENABLE

#ifdef SET_TIME_ENABLE
#define HOURS 12
#define MINUTES 32
#define SECONDS 0
#define YEARS 2023
#define MONTHS 2
#define DAYS 26
#endif

#define M_PI (3.14159265358979323846)  /* pi */
#define TWOPI (M_PI*2)
#define RPD (M_PI/180.)

#define DS3231_I2C_ADDR (0x68U<<1)  /* RTC */
#define GY271_I2C_ADDR (0x0DU<<1)  /* Compass */

// time keeping registers
#define DS3231_TIME_CAL_ADDR        0x00
#define DS3231_ALARM1_ADDR          0x07
#define DS3231_ALARM2_ADDR          0x0B
#define DS3231_CONTROL_ADDR         0x0E
#define DS3231_STATUS_ADDR          0x0F
#define DS3231_AGING_OFFSET_ADDR    0x10
#define DS3231_TEMPERATURE_ADDR     0x11

// control register bits
#define DS3231_CONTROL_A1IE     	0x1		/* Alarm 2 Interrupt Enable */
#define DS3231_CONTROL_A2IE     	0x2		/* Alarm 2 Interrupt Enable */
#define DS3231_CONTROL_INTCN    	0x4		/* Interrupt Control */
#define DS3231_CONTROL_RS1	    	0x8		/* square-wave rate select 2 */
#define DS3231_CONTROL_RS2    		0x10	/* square-wave rate select 2 */
#define DS3231_CONTROL_CONV    		0x20	/* Convert Temperature */
#define DS3231_CONTROL_BBSQW    	0x40	/* Battery-Backed Square-Wave Enable */
#define DS3231_CONTROL_EOSC	    	0x80	/* not Enable Oscillator, 0 equal on */

// status register bits
#define DS3231_STATUS_A1F      		0x01		/* Alarm 1 Flag */
#define DS3231_STATUS_A2F      		0x02		/* Alarm 2 Flag */
#define DS3231_STATUS_BUSY     		0x04		/* device is busy executing TCXO */
#define DS3231_STATUS_EN32KHZ  		0x08		/* Enable 32KHz Output  */
#define DS3231_STATUS_OSF      		0x80		/* Oscillator Stop Flag */

// Compass Data Output Registers
#define GY271_DATA_X_LSB  0x00
#define GY271_DATA_X_MSB  0x01
#define GY271_DATA_Y_LSB  0x02
#define GY271_DATA_Y_MSB  0x03
#define GY271_DATA_Z_LSB  0x04
#define GY271_DATA_Z_MSB  0x05

// Compass Status Register
#define GY271_STATUS_FLAGS  0x06
#define GY271_STATUS_FLAG_DOR (0x01U << 2)
#define GY271_STATUS_FLAG_OVL (0x01U << 1)
#define GY271_STATUS_FLAG_DRDY (0x01U << 0) /* Data Ready Register */

// Compass Control Register
#define GY271_CONTROL_REGISTER_1  0x09
#define GY271_CONTROL_MODE (0x03U << 0) /* 00 standby, 01 cont */
#define GY271_CONTROL_ODR (0x03U << 2) /* 00 10Hz, 01 50Hz, 10 100Hz, 11 200Hz */
#define GY271_CONTROL_RNG (0x03U << 4) /* full scale: 00 2G, 01 8G */
#define GY271_CONTROL_OSR (0x03U << 6) /* over sampling ratio: 00 512, 01 256, 10 128, 11 64 */

#define GY271_CONTROL_REGISTER_2  0x0B



typedef struct ts {
    uint8_t sec;         /* seconds */
    uint8_t min;         /* minutes */
    uint8_t hour;        /* hours */
    uint8_t mday;        /* day of the month */
    uint8_t mon;         /* month */
    int16_t year;        /* year */
    uint8_t wday;        /* day of the week */
    uint8_t yday;        /* day in the year */
    uint8_t isdst;       /* daylight saving time */
    uint8_t year_s;      /* year in short notation*/

}ts;

enum Months {
  JAN,
  FEB,
  MAR,
  APR,
  MAY,
  JUN,
  JUL,
  AUG,
  SEP,
  OCT,
  NOV,
  DEC
};

typedef struct direction {
  int16_t x_dir;
  int16_t y_dir;
  uint8_t angle;
}direction;

// Convert normal decimal numbers to binary coded decimal
uint8_t decToBcd(int val)
{
  return (uint8_t)( (val/10*16) + (val%10) );
}

// Convert binary coded decimal to normal decimal numbers
int bcdToDec(uint8_t val)
{
  return (int)( (val/16*10) + (val%16) );
}

/* function to get the time */
int Get_Time(ts *t)
{
	uint8_t TimeDate[7];        //second,minute,hour,dow,day,month,year
  uint8_t i;
	uint16_t year_full;

	if (HAL_I2C_Mem_Read(&hi2c1, DS3231_I2C_ADDR, DS3231_TIME_CAL_ADDR, 1, TimeDate, 7, 1000) == HAL_OK)
  {
    for (i = 0; i <= 6; i++)
    {
      if (i == 5)
      {
        TimeDate[5] = bcdToDec(TimeDate[i] & 0x1F);
      }
      else
      {
        TimeDate[i] = bcdToDec(TimeDate[i]);
      }
	  }

    year_full = 2000 + TimeDate[6];

    t->sec = TimeDate[0];
    t->min = TimeDate[1];
    t->hour = TimeDate[2];
    t->mday = TimeDate[4];
    t->mon = TimeDate[5];
    t->year = year_full;
    t->wday = TimeDate[3];
    t->year_s = TimeDate[6];

    return 0x01U;
  }
  else
  {
    return 0xFFU;
  }
}

#ifdef SET_TIME_ENABLE
/* function to set the time */
void Set_Time(ts t)
{
    uint8_t i, century;
    century = 0x80;
    t.year_s = t.year - 2000;
    uint8_t TimeDate[7] = { t.sec, t.min, t.hour, t.wday, t.mday, t.mon, t.year_s };

    for (i=0;i<=6;i++)
    	{
    	TimeDate[i] = decToBcd(TimeDate[i]);
    	if(i == 5){TimeDate[5] |= century;}
    	}

    HAL_I2C_Mem_Write(&hi2c1, DS3231_I2C_ADDR, DS3231_TIME_CAL_ADDR, 1, (char*)TimeDate, 7, 1000);
    //i2c_WriteMulti(DS3231_I2C_ADDR,DS3231_TIME_CAL_ADDR,(char*)TimeDate,7);
}
#endif

/* function to set the compass modes */
void Setup_Compass()
{
    uint8_t OSR = 0b00; /* 512 oversampling */
    uint8_t RNG = 0b00; /* 2G full scale */
    uint8_t ODR = 0b00; /* 10Hz */
    uint8_t MODE = 0b01; /* cont */
    uint8_t data[2] = {0, 0};

    HAL_Delay(500);
    data[0] = 0x0A;
    data[1] = 0xC1; /* soft reset */
    HAL_I2C_Master_Transmit(&hi2c1,GY271_I2C_ADDR,data,2,10);

    HAL_Delay(2000);
    data[0] = GY271_CONTROL_REGISTER_2;
    data[1] = 0x01; /* Define Set/Reset period */
    HAL_I2C_Master_Transmit(&hi2c1,GY271_I2C_ADDR,data,2,10);

    HAL_Delay(500);
    data[0] = GY271_CONTROL_REGISTER_1;
    //data[1] = 0x1D;
    data[1] = (OSR<<6) | (RNG<<4) | (ODR<<2) | (MODE<<0);
    HAL_I2C_Master_Transmit(&hi2c1,GY271_I2C_ADDR,data,2,10);
}

/* check if compass data is ready */
int Compass_Data_Ready()
{

  uint8_t reg_val[1];
  
  if (HAL_I2C_Mem_Read(&hi2c1, GY271_I2C_ADDR, 0x06U, 1, reg_val, 1, 1000) == HAL_OK)
  {
    if (reg_val[0] & GY271_STATUS_FLAG_DRDY)
      return 1;
    else
      return reg_val[0];
  }
  else
    return 0;

}

/* read compass x data */
int Compass_x_read()
{

  uint8_t reg_lsb[1];
  uint16_t lsb;
  uint8_t reg_msb[1];
  uint16_t msb;
  uint32_t buffer;
  
  HAL_I2C_Mem_Read(&hi2c1, GY271_I2C_ADDR, GY271_DATA_X_LSB, 1, reg_lsb, 1, 1000);
  HAL_I2C_Mem_Read(&hi2c1, GY271_I2C_ADDR, GY271_DATA_X_MSB, 1, reg_msb, 1, 1000);

  lsb = (uint16_t)reg_lsb[0];
  msb = (uint16_t)reg_msb[0];
  msb = msb << 8;
  buffer = (lsb | msb)<<16;

  //buffer = (int)(((0b0000000011111111)&(uint16_t)reg_lsb[0]) | ((0b1111111100000000)&(((uint16_t)reg_msb[0])<<8)));

  return ((int)buffer)>>16;

}

/* read compass y data */
int Compass_y_read()
{

  uint8_t reg_lsb[1];
  uint16_t lsb;
  uint8_t reg_msb[1];
  uint16_t msb;
  uint32_t buffer;
  
  HAL_I2C_Mem_Read(&hi2c1, GY271_I2C_ADDR, GY271_DATA_Z_LSB, 1, reg_lsb, 1, 1000);
  HAL_I2C_Mem_Read(&hi2c1, GY271_I2C_ADDR, GY271_DATA_Z_MSB, 1, reg_msb, 1, 1000);

  lsb = (uint16_t)reg_lsb[0];
  msb = (uint16_t)reg_msb[0];
  msb = msb << 8;
  buffer = (lsb | msb)<<16;

  //buffer = (int)(((0b0000000011111111)&(uint16_t)reg_lsb[0]) | ((0b1111111100000000)&(((uint16_t)reg_msb[0])<<8)));

  return ((int)buffer)>>16;

}

/* read compass z data */
void Compass_z_read()
{

  uint8_t reg_lsb[1];
  uint8_t reg_msb[1];
  //uint16_t buffer;
  
  HAL_I2C_Mem_Read(&hi2c1, GY271_I2C_ADDR, GY271_DATA_Y_LSB, 1, reg_lsb, 1, 1000);
  HAL_I2C_Mem_Read(&hi2c1, GY271_I2C_ADDR, GY271_DATA_Y_MSB, 1, reg_msb, 1, 1000);

  //buffer = (((uint16_t)reg_lsb[0]) | (((uint16_t)reg_msb[0])<<8));

  //return buffer;
}

int lastval = 0;

int x_val[10] = {0,0,0,0,0,0,0,0,0,0};
int y_val[10] = {0,0,0,0,0,0,0,0,0,0};
int val_count = 0;

/**
 * C++ version 0.4 char* style "itoa":
 * Written by Lukás Chmela
 * Released under GPLv3.
 */
char* itoa(int value, char* result, int base) {
    // check that the base if valid
    if (base < 2 || base > 36) { *result = '\0'; return result; }

    char* ptr = result, *ptr1 = result, tmp_char;
    int tmp_value;

    do {
        tmp_value = value;
        value /= base;
        *ptr++ = "zyxwvutsrqponmlkjihgfedcba9876543210123456789abcdefghijklmnopqrstuvwxyz" [35 + (tmp_value - value * base)];
    } while ( value );

    // Apply negative sign
    if (tmp_value < 0) *ptr++ = '-';
    *ptr-- = '\0';
    while(ptr1 < ptr) {
        tmp_char = *ptr;
        *ptr--= *ptr1;
        *ptr1++ = tmp_char;
    }
    return result;
}

int read_direction()
{
  int x_dir = 0;
  int y_dir = 0;
  int count = 0;
  double fazi;
  int iazi;
  char msgs[15];

  while(count < 1)
  {
    if(Compass_Data_Ready() == 1)
    {
      //x_dir += (int)((double)((Compass_x_read()) + 33)*1.8181);
      x_dir = Compass_x_read();
      Compass_z_read();
      y_dir = Compass_y_read();
      count++;
      //val_count++;
      //if(val_count > 9) val_count = 0;
    }
  }

  //x_dir = (x_val[0] + x_val[1] + x_val[2] + x_val[3] + x_val[4] + x_val[5] + x_val[6] + x_val[7] + x_val[8] + x_val[9])/10;
  //y_dir = (y_val[0] + y_val[1] + y_val[2] + y_val[3] + y_val[4] + y_val[5] + y_val[6] + y_val[7] + y_val[8] + y_val[9])/10;


      //x_dir = x_dir / 1;
      x_dir = (int)(((double)x_dir + 0.0)*1.0);
      //y_dir = y_dir / 1;
      y_dir = (int)(((double)y_dir - 0.0)*1.0);
      //y_dir = 1000;
      //if (y_dir < lastval) lastval = y_dir;
  


  //x_dir = Compass_x_read();
  //y_dir = Compass_y_read();
  //Compass_z_read();

  //x_dir = 900;
  //y_dir = 900;

  fazi = atan2((double)y_dir,(double)x_dir) * 180.0/M_PI;
  iazi = (int)(fazi);

  iazi = iazi - 0;
  //iazi = iazi * (-1);

  memset(msgs,0,15);
  itoa(x_dir, msgs, 10);
  HAL_UART_Transmit(&huart1, (unsigned char *)msgs, sizeof(msgs), 1000);
  HAL_UART_Transmit(&huart1, ";", sizeof(";"), 1000);

  memset(msgs,0,15);
  itoa(y_dir, msgs, 10);
  HAL_UART_Transmit(&huart1, (unsigned char *)msgs, sizeof(msgs), 1000);
  HAL_UART_Transmit(&huart1, "\n\r", sizeof("\n\r"), 1000);

  return x_dir;
}



/**
 * Write an integer value to the UART interface.
*/
void write_int_to_uart(char msg[], UART_HandleTypeDef huart1, int value)
{
  itoa(value, msg, 10);
  HAL_UART_Transmit(&huart1, (unsigned char *)msg, sizeof(msg), 1000);
  HAL_UART_Transmit(&huart1, (unsigned char *)"\n", sizeof("\n"), 1000);
}

/* 1 if leap year */
int is_leap_year(int year) {
    if (year % 4 == 0) {
        if (year % 100 == 0) {
            if (year % 400 == 0) {
                return 1;
            } else {
                return 0;
            }
        } else {
            return 1;
        }
    } else {
        return 0;
    }
}

int sun_azimut(int year, int month, int day, int hour, int min)
{
  int delta, leap, feb_days;
  double jd, time, mnlong, mnanom, eclong, qblqec, NUM, den, ra, dec, gmst, lmst, ha, el, az, fhour;
  double lon = 15.60140;
  double lat = 47.07930;

  fhour = (double)hour + ((double)min/60.0);

  if (is_leap_year(year))
    feb_days = 29;
  else
    feb_days = 28;

  switch (month)
  {
    case 1: /* jan */
      day = day;
      break;
    case 2: /* feb */
      day = day + 31;
      break;
    case 3: /* mar */
      day = day + 31 + feb_days;
      break;
    case 4: /* apr */
      day = day + 31 + feb_days + 31;
      break;
    case 5: /* may */
      day = day + 31 + feb_days + 31 + 30;
      break;
    case 6: /* jun */
      day = day + 31 + feb_days + 31 + 30 + 31;
      break;
    case 7: /* jul */
      day = day + 31 + feb_days + 31 + 30 + 31 + 30;
      break;
    case 8: /* aug */
      day = day + 31 + feb_days + 31 + 30 + 31 + 30 + 31;
      break;
    case 9: /* sep */
      day = day + 31 + feb_days + 31 + 30 + 31 + 30 + 31 + 31;
      break;
    case 10: /* oct */
      day = day + 31 + feb_days + 31 + 30 + 31 + 30 + 31 + 31 + 30;
      break;
    case 11:
      day = day + 31 + feb_days + 31 + 30 + 31 + 30 + 31 + 31 + 30 + 31;
      break;
    case 12:
      day = day + 31 + feb_days + 31 + 30 + 31 + 30 + 31 + 31 + 30 + 31 + 30;
      break;
  }

  fhour--;

  delta = year - 1949;
  leap = delta / 4;
  jd = 32916.5 + (delta*365 + leap + day) + fhour / 24.;

  if(((year % 100) == 0) && ((year % 400) != 0))
  {
    jd = jd - 1.;
  }

  time  = jd - 51545.0;

  mnlong = 280.460 + 0.9856474*time;
  mnlong =  fmod(mnlong, 360. );

  if(mnlong < 0.)
  {
    mnlong = mnlong + 360.;
  }

  mnanom = 357.528 + 0.9856003*time;
  mnanom = fmod(mnanom, 360.);

  if(mnanom < 0.)
  {
    mnanom = mnanom + 360.;
  }

  mnanom = mnanom*RPD;

  eclong = mnlong + 1.915*sin( mnanom ) + 0.020*sin( 2.*mnanom );
  eclong = fmod(eclong, 360);

  if( eclong < 0. )
  {
    eclong = eclong + 360.;
  }

  qblqec = 23.439 - 0.0000004*time;
  eclong = eclong*RPD;
  qblqec = qblqec*RPD;

  NUM  = cos( qblqec )*sin( eclong );
  den  = cos( eclong );
  ra   = atan( NUM / den );

  if( den < 0.0 )
  {
    ra  = ra + M_PI;
  }
  else
  {
    if( NUM < 0.0 )
    {
      ra  = ra + TWOPI;
    }
  }

  dec  = asin( sin( qblqec )*sin( eclong ) );

  gmst = 6.697375 + 0.0657098242*time + fhour;
  gmst  =  fmod(gmst , 24.);

  if( gmst < 0. )
  {
    gmst   = gmst + 24.;
  }

  lmst  = gmst + lon / 15.;
  lmst  = fmod( lmst , 24.);

  if( lmst < 0. )
  {
    lmst   = lmst + 24.;
  }

  lmst   = lmst*15.*RPD;

  ha  = lmst - ra;

  if( ha < - M_PI ) ha  = ha + TWOPI;
  if( ha > M_PI )   ha  = ha - TWOPI;

  el  = asin( sin( dec )*sin( lat*RPD ) + cos( dec )*cos( lat*RPD )*cos( ha ) );

  az  = asin( - cos( dec )*sin( ha ) / cos( el ) );

  if( sin( dec ) - sin( el )*sin( lat*RPD ) >= 0. ){
         if( sin(az) < 0.) az  = az + TWOPI;}
      else
         az  = M_PI - az;
  
  return az / RPD;
}

// double fmod(double a, double b)
// {
//     return a - (round(a / b) * b);
// }

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  char msg[15];
  ts ds3231_data;
#ifdef SET_TIME_ENABLE
  ds3231_data.sec = SECONDS;
	ds3231_data.min = MINUTES;
	ds3231_data.hour = HOURS;
	ds3231_data.mday = DAYS;
	ds3231_data.mon = MONTHS;
	ds3231_data.year = YEARS;
#endif


  int az;
  int test = 0;
  uint8_t reg_val;
  uint8_t data[2];
  int16_t temp = -350;
  int temp2;

  temp2 = (int)temp;

  //memset(msg,0,15);

  //az = sun_azimut(2023, 53, 21.0);




  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  setup();
#ifdef SET_TIME_ENABLE
  Set_Time(ds3231_data);
#endif
  Setup_Compass();
  // HAL_Delay(500);
  // //test = HAL_I2C_Mem_Write(&hi2c1, GY271_I2C_ADDR, 0x0BU, 1, 0x01U, 1, 1000);
  // data[0] = 0x0B;
  // data[1] = 0x01;
  // test = HAL_I2C_Master_Transmit(&hi2c1,GY271_I2C_ADDR,data,2,10);
  // HAL_Delay(500);
  // data[0] = 0x09;
  // data[1] = 0x1D;
  // test = HAL_I2C_Master_Transmit(&hi2c1,GY271_I2C_ADDR,data,2,10);
  //HAL_Delay(1000);
  //test = HAL_I2C_Mem_Write(&hi2c1, GY271_I2C_ADDR, 0x09U, 1, 0x1DU, 1, 1000);
  //HAL_Delay(1000);
  //test = HAL_I2C_Mem_Read(&hi2c1, GY271_I2C_ADDR, 0x09U, 1, reg_val, 1, 1000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    loop(huart1);
    HAL_Delay(500U);

    if (HAL_I2C_IsDeviceReady(&hi2c1, DS3231_I2C_ADDR, 1, 1000) == HAL_OK)
    {
      // Get_Time(&ds3231_data);
      // HAL_UART_Transmit(&huart1, (unsigned char *)"Current time:", sizeof("Current time:"), 1000);
      // write_int_to_uart(msg, huart1, ds3231_data.hour);
      // write_int_to_uart(msg, huart1, ds3231_data.min);
      // write_int_to_uart(msg, huart1, ds3231_data.sec);
      // HAL_UART_Transmit(&huart1, (unsigned char *)"Current date:", sizeof("Current time:"), 1000);
      // write_int_to_uart(msg, huart1, ds3231_data.mday);
      // write_int_to_uart(msg, huart1, ds3231_data.mon);
      // write_int_to_uart(msg, huart1, ds3231_data.year);
      // az = sun_azimut(ds3231_data.year, ds3231_data.mon, ds3231_data.mday, ds3231_data.hour, ds3231_data.min);
      // HAL_UART_Transmit(&huart1, (unsigned char *)"sun azimut:", sizeof("sun azimut:"), 1000);
      // write_int_to_uart(msg, huart1, (int)az);
      // memset(msg,0,strlen(msg));

      // HAL_UART_Transmit(&huart1, (unsigned char *)"compass:", sizeof("compass:"), 1000);
      // HAL_UART_Transmit(&huart1, (unsigned char *)"\r\n", sizeof("\r\n"), 1000);
      // memset(msg,0,strlen(msg));
      //write_int_to_uart(msg, huart1, read_direction());
      read_direction();
      //memset(msg,0,15);
      
      //HAL_UART_Transmit(&huart1, (unsigned char *)"\r\n", sizeof("\r\n"), 1000);
      //memset(msg,0,strlen(msg));
    }

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x2000090E;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 38400;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, USR_TEMP_SENS_Pin|USR_LED_GREEN_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, USR_RLY_4_Pin|USR_RLY_3_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, USR_RLY_1_Pin|USR_RLY_2_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : USR_BTN_2_Pin USR_WIND_SENS_Pin USR_BTN_3_Pin */
  GPIO_InitStruct.Pin = USR_BTN_2_Pin|USR_WIND_SENS_Pin|USR_BTN_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USR_TEMP_SENS_Pin */
  GPIO_InitStruct.Pin = USR_TEMP_SENS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(USR_TEMP_SENS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USR_RLY_1_Pin USR_RLY_2_Pin */
  GPIO_InitStruct.Pin = USR_RLY_1_Pin|USR_RLY_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : USR_BTN_1_Pin */
  GPIO_InitStruct.Pin = USR_BTN_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(USR_BTN_1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USR_LED_GREEN_Pin USR_RLY_4_Pin USR_RLY_3_Pin */
  GPIO_InitStruct.Pin = USR_LED_GREEN_Pin|USR_RLY_4_Pin|USR_RLY_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
