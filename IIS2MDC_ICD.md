# IIS2MDC Magnetometer Interface Control Document

## Scope
This document defines the interface enabling the AIB's MCU and one or more external controllers to coordinate the operation of the AIB's [IIS2MDC magnetometers](https://www.st.com/resource/en/datasheet/iis2mdc.pdf) with external activities that may generate magnetic fields capable of interfering with magnetometer measurements.

It covers:
- Physical interface constrains
- Communication protocol
- Timing requirements
- Message format and definitions
- Coordination protocol (interface behaviour)
- Message sequences

This document does not detail the actual software implementation of the coordination interface.

## Definitions and Acronyms
AIB - ADCS Integration Board  
AIB MCU refers to the [STM32H7A3RGT6](https://www.st.com/resource/en/datasheet/stm32h7a3ai.pdf)

## Physical Interface Constraints  
### Electrical Characteristics
Tolerable Supply voltage range by IIS2MDC Magnetometer: 
- Minimum: 1.71V, Absolute Minimum: -0.3V 
- Typical: 2.5V 
- Maximum: 3.6V, Absolute Maximum: 4.8V

Current consumption:
- high-resolution mode (offset cancellation turned on): 1130 μA
- low-power mode (offset cancellation turned off): 23 μA
- power-down: 1.5 μA

## Communication Protocol and Message Structure
IIS2MDC Magnetometer is connected to AIB MCU via SPI2 Interface. Thus IIS2MDC is restricted to use SPI communication with MCU.

IIS2MDC Magnetometer is capable of both Serial Peripheral Interface (SPI) and Inter-Intergrated Communication (I2C) serial communication protocols.

SPI clock frequency: 10000kHz

I2C:  
Capable of a 'fast', 'fast+' and 'high speed' mode.
Clock frequency:  
- standard: 100kHz
- fast: 400kHz
- fast+: 1000kHz
- high speed: 3400kHz
