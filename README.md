# sun_follower
Digital compass controlled sun follower for PV modules.

![block diagram](weather_station.drawio.png)

This project provides firmware for a device that aligns a PV installation with the sun.  
- The sun's position is calculated, and the orientation of the panel is measured using a magnetic field sensor (digital compass).  
- Relays are used to drive an actuator (e.g., motor) that adjusts the PV panel's position.  
- A Real-Time Clock (RTC) module ensures that date and time are accurate for calculating the sun’s position.  
- A light sensor is planned to determine whether the sun is actually shining.  
- There are also planned sensors for temperature, air pressure, and wind speed, though they are not yet integrated.  

Currently, only the digital compass and the RTC module are operational. In the `python` folder, you will find a script that reads and processes the compass data. Note that the compass must be calibrated for accurate readings.

In the `doc` folder, you can find datasheets for the various components and a block diagram of the system.