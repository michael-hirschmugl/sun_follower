/*
 * sun_prediction.h
 *
 *  Created on: Aug 9, 2023
 *      Author: Michael
 */

#ifndef SRC_SUN_PREDICTION_H_
#define SRC_SUN_PREDICTION_H_

#include <math.h>

//#define M_PI (3.14159265358979323846)  /* pi */
#define TWOPI (M_PI * 2)
#define RPD (M_PI / 180.)

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

int sun_azimut(int year, int month, int day, int hour, int min) {
  int delta, leap, feb_days;
  double jd, time, mnlong, mnanom, eclong, qblqec, NUM, den, ra, dec, gmst,
      lmst, ha, el, az, fhour;
  double lon = 15.60140;
  double lat = 47.07930;

  fhour = (double)hour + ((double)min / 60.0);

  if (is_leap_year(year))
    feb_days = 29;
  else
    feb_days = 28;

  switch (month) {
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
  jd = 32916.5 + (delta * 365 + leap + day) + fhour / 24.;

  if (((year % 100) == 0) && ((year % 400) != 0)) {
    jd = jd - 1.;
  }

  time = jd - 51545.0;

  mnlong = 280.460 + 0.9856474 * time;
  mnlong = fmod(mnlong, 360.);

  if (mnlong < 0.) {
    mnlong = mnlong + 360.;
  }

  mnanom = 357.528 + 0.9856003 * time;
  mnanom = fmod(mnanom, 360.);

  if (mnanom < 0.) {
    mnanom = mnanom + 360.;
  }

  mnanom = mnanom * RPD;

  eclong = mnlong + 1.915 * sin(mnanom) + 0.020 * sin(2. * mnanom);
  eclong = fmod(eclong, 360);

  if (eclong < 0.) {
    eclong = eclong + 360.;
  }

  qblqec = 23.439 - 0.0000004 * time;
  eclong = eclong * RPD;
  qblqec = qblqec * RPD;

  NUM = cos(qblqec) * sin(eclong);
  den = cos(eclong);
  ra = atan(NUM / den);

  if (den < 0.0) {
    ra = ra + M_PI;
  } else {
    if (NUM < 0.0) {
      ra = ra + TWOPI;
    }
  }

  dec = asin(sin(qblqec) * sin(eclong));

  gmst = 6.697375 + 0.0657098242 * time + fhour;
  gmst = fmod(gmst, 24.);

  if (gmst < 0.) {
    gmst = gmst + 24.;
  }

  lmst = gmst + lon / 15.;
  lmst = fmod(lmst, 24.);

  if (lmst < 0.) {
    lmst = lmst + 24.;
  }

  lmst = lmst * 15. * RPD;

  ha = lmst - ra;

  if (ha < -M_PI) ha = ha + TWOPI;
  if (ha > M_PI) ha = ha - TWOPI;

  el = asin(sin(dec) * sin(lat * RPD) + cos(dec) * cos(lat * RPD) * cos(ha));

  az = asin(-cos(dec) * sin(ha) / cos(el));

  if (sin(dec) - sin(el) * sin(lat * RPD) >= 0.) {
    if (sin(az) < 0.) az = az + TWOPI;
  } else
    az = M_PI - az;

  return az / RPD;
}

#endif /* SRC_SUN_PREDICTION_H_ */