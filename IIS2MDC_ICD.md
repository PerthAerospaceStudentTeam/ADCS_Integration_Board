# Interface Control Document for IIS2MDC Magnetometer

## Scope
This document outlines communication interface for the [IIS2MDC Magnetometer](https://www.st.com/resource/en/datasheet/iis2mdc.pdf).

## Physical Constraints

### Electrical Characteristics
Supply voltage: 
- Minimum: 1.71V, Absolute Minimum: -0.3V 
- Typical: 2.5V 
- Maximum: 3.6V, Absolute Maximum: 4.8V

Current consumption:
- high-resolution mode (offset cancellation turned on): 1130 μA
- low-power mode (offset cancellation turned off): 23 μA
- power-down: 1.5 μA

## Communication Protocol and Message Structure
IIS2MDC Magnetometer is capable of both Serial Peripheral Interface (SPI) and Inter-Intergrated Communication (I2C) serial communication protocols.

SPI clock frequency: 10000kHz

I2C:  
Capable of a 'fast', 'fast+' and 'high speed' mode.
Clock frequency:  
- standard: 100kHz
- fast: 400kHz
- fast+: 1000kHz
- high speed: 3400kHz
