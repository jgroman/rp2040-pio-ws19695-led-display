# rp2040-pio-ws19695-led-display

Waveshare Electronics (Pico-Clock-Green (a.k.a. ws-19695))[https://www.waveshare.com/pico-clock-green.htm]
contains a large LED matrix display which is driven by two SM16106 shift registers
and one SM5166P address decoder.

This project creates a controller for Pico-Clock-Green display using Raspberry Pi
Pico (RP2040) PIO and DMA functions, thus taking no CPU cycles for controlling
and refreshing of LED matrix display.
