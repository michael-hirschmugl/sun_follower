/*
 * ds3231.h
 *
 *  Created on: Aug 9, 2023
 *      Author: u15q86
 */

#ifndef SRC_DS3231_H_
#define SRC_DS3231_H_

//#define SET_TIME_ENABLE

#ifdef SET_TIME_ENABLE
#define HOURS 15
#define MINUTES 8
#define SECONDS 0
#define YEARS 2023
#define MONTHS 8
#define DAYS 9
#endif

#define DS3231_I2C_ADDR (0x68U<<1)  /* RTC */

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

uint8_t decToBcd(int val)
{
  return (uint8_t)( (val/10*16) + (val%10) );
}

int bcdToDec(uint8_t val)
{
  return (int)( (val/16*10) + (val%16) );
}

#ifdef SET_TIME_ENABLE
/* function to set the time */
void Set_Time(ts t, I2C_HandleTypeDef *hi2c1)
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

    HAL_I2C_Mem_Write(hi2c1, DS3231_I2C_ADDR, DS3231_TIME_CAL_ADDR, 1, (uint8_t*)TimeDate, 7, 1000);
    //i2c_WriteMulti(DS3231_I2C_ADDR,DS3231_TIME_CAL_ADDR,(char*)TimeDate,7);
}
#endif

int Get_Time(ts *t, I2C_HandleTypeDef *hi2c1)
{
	uint8_t TimeDate[7];
    uint8_t i;
	uint16_t year_full;

	if (HAL_I2C_Mem_Read(hi2c1, DS3231_I2C_ADDR, DS3231_TIME_CAL_ADDR, 1, TimeDate, 7, 1000) == HAL_OK)
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

#endif /* SRC_DS3231_H_ */