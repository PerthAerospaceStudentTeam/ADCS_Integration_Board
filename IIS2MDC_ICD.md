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
### Electrical Characteristics of IIS2MDC Magnetometer
Tolerable Supply voltage range: 
- Minimum: 1.71V, Absolute Minimum: -0.3V 
- Typical: 2.5V 
- Maximum: 3.6V, Absolute Maximum: 4.8V

Current consumption:
- high-resolution mode (offset cancellation turned on): 1130 μA
- low-power mode (offset cancellation turned off): 23 μA
- power-down: 1.5 μA

### Communication Constraints of IIS2MDC Magnetometer
IIS2MDC Magnetometer is capable of both Serial Peripheral Interface (SPI) and Inter-Intergrated Communication (I2C) serial communication protocols.  
->I2C Slave Address: _0011110b_.

#### Communication Pin Descriptions:
- MAG_CS: Communciation protocol enable.
    - 1: SPI idle mode, I2C Communication enabled.
    - 2: SPI active mode, I2C Communication disabled.
- MAG_SCL/SPC: 
    - I2C, serial clock (SCL).
    - SPI, serial port clock (SPC).
- MAG_SDA/SDI/SDO:
    - SDA: I2C Serial Data.
    - SDI: SPI Serial Data Input.
    - SDO: 3-wire SPI Interface Serial Data Output.

### Pin Interface between AIB MCU and Magnetometer
AIB MCU is connected to the IIS2MDC magnetometer using the SPI2 interface, requiring the following pins to be used on AIB MCU:
- PB10 -> SPI2_SCK, Connected to MAG_SPC/SCL
- PC3 -> SPI2_MOSI, Connected to MAG_SDO/SDA
- PB2 -> SPI2_CS, Connect to MAG_CS

## Communication Protocol
IIS2MDC Magnetometer is connected to AIB MCU via SPI2 Interface, thus communication between the AIB MCU and IIS2MDC is restricted to use SPI communication. The SPI bus on IIS2MDC Magnetometer is a bus slave, allowing a master to read from/write to its registers using CS, SPC & SDI/O pins.  

A SPI write/read operation is performed in 16 clock pulses, inital 8 pulses used to transmit first 8 bits corresponding to address of register and read/write flag, subsequent 8 pulses then for transmitting single byte to/from IIS2MDC, additional octet-byte read/writes require an additional 8 clock pulses ontop of previous pulses.  

#### SPI Slave (IIS2MDC) Timings:
- SPI minimum clock cycle: 100ns 
- SPI maximum clock frequency: 10000kHz
- SPI minimum CS setup time: 5ns
- SPI minimum CS hold time: 20ns

## Message Structure 
