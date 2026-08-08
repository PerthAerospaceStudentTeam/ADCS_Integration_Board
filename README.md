# ADCS_Integration_Board
Code base for the ADCS integration board, used to test ADCS algorithms.  
Utilises the STM32H7A3RGT6 as central MCU to retrieve data, used to control output to magnetorquer, from sensors:
- LSM6DSOTR, 6-axis Inertial Measurement Unit
- IIS2MDCTR, Magnetometer 
- Sun Sensors  

Codebase also intends to handle filtering of immediate data recieved from such sensors as well as performing sensor fusion to produce meaningful results from Integration Board sensors.

## Installation Instructions
- Navigate to this website https://community.st.com/t5/stm32-mcus/how-to-use-github-with-stm32cubeide/ta-p/793250
- Complete step 5 and onward

## Configuration
As of now, there are currently no environment variables which must be set/defined.

## Usage
Codebase is to be stored and ran on the STM32H7A3RGT6 MCU attached to the ADCS Integration Board, aims to run/test ADCS algorithms related to sensor-based output to AIB's magnetorquers. 

## Project Structure
Sun_Sensor_algo directory is used for creation and testing of algorithms related to converting received data from the sun sensors into usable data.  
Integration_Board_Project directory acts as the primary codebase of the repository. This directory is used to store code for pulling data from all sensors attached to the ADCS_Integration_Board for use in ADCS algorithms.  
- Integration_Board_Project/Core/Src/main.c contains the main codebase used for project

## Contributing
- Jayden Z
- Aiden H
- Bodhi B
- Luke L

## Contact

## License
