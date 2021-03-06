#!/usr/bin/python

# Raspberry Pi SPI Port and Device
spi_port = 0
spi_dev = 0

# Pin # for relay connected to heating element
he_pin = 7

# Default goal temperature
set_temp = 221.

# Default alarm time
snooze = '07:00'

# Main loop sample rate in seconds
sample_time = 0.3

# PID Proportional, Integral, and Derivative values
Pc = 3.4
Ic = 0.3
Dc = 40.0

Pw = 2.9
Iw = 0.3
Dw = 40.0

# Bars of pressure to use as a detector for the start of a shot.
shot_pressure_threshold = 7

# Web/REST Server Options
port = 8080

# Loadcell Calibration Value for 1lb of force
loadcell_calibration = -9480