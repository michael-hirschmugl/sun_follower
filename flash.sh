#!/bin/bash

st-info --probe
cd cmake-build-debug
st-flash write sun-follower.bin 0x8000000

