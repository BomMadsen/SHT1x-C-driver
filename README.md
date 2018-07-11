# SHT1x-C-driver
SHT1x (SHT10, SHT11, SHT15) humidity and temperature sensor C driver library for Microchip or similar MCU's.

## Requirements
The driver needs access to two blocking delay routines:

* delay_us(t) - should delay for t microseconds
* delay_ms(t) - should delay for t milliseconds

math.h is required for the natural logarithm - log()

## Configuration
SHT1x is commected via CLK and DAT pins. 
These must be configured in "sht15.h" respectively by their tri-state register (to toggle input/output-mode) and their port-bit, to enable reading and writing the IO's. 
