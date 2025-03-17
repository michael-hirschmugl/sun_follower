#!/bin/bash

st-info --probe
cd cmake-build-debug
st-flash write sun-follower.bin 0x8000000

# openocd -f interface/stlink.cfg -f target/stm32f2x.cfg -c "program build/SunFollowerFw.elf verify reset exit"