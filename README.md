# RP2040 Guitar Tuner (MAX9814 + DSP)

A real-time embedded guitar tuner built on the Raspberry Pi Pico.

## Features
- 10 kHz ADC sampling
- DMA-based data capture
- Goertzel frequency detection
- Hann windowing
- Cents calculation
- LCD display + LED indicators

## Hardware
- Raspberry Pi Pico (RP2040)
- MAX9814 microphone amplifier
- 1602 I2C LCD
- LEDs

## Demo
(Add your demo video link here)

## Skills Demonstrated
- Embedded C
- ADC + DMA
- Digital signal processing
- Hardware/software integration
- Real-time systems design

## Build Instructions
```bash
mkdir build
cd build
cmake ..
make

