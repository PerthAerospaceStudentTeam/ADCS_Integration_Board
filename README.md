# ADCS_Integration_Board
Code base for the ADCS integration board.

## Installation Instructions
- Navigate to this website https://community.st.com/t5/stm32-mcus/how-to-use-github-with-stm32cubeide/ta-p/793250
- Complete step 5 and onward

## Configuration

## Usage
Code to be used on PAST's ADCS Integration board, aims to run/test ADCS algorithms.
Utilisises the STM32H7A3RGT6 as central MCU to retrieve data from sensors:
	- LSM6DSOTR, 6-axis Inertial Measurement Unit
	- IIS2MDCTR, Magnetometer 
	- Sun Sensors

## Project Structure
__Will update when able to consult with Jayden in person for more info on exact project structure__
Sun_Sensor_algo directory contains algorithms for handling raw data retrieved from the sun sensors.
Integration_Board_Project directory contains code for pulling data from the LSM6DSOTR and IIS2MDCTR sensors, converting such raw data into usable information.

## Contributing
- Jayden Z
- Aiden H
- Bodhi B
- Luke L

## Contact

## License
