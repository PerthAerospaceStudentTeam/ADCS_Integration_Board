# ADCS_Integration_Board
Code base for the ADCS integration board, primarily used to test ADCS algorithms.  
Utilises the STM32H7A3RGT6 as central MCU to retrieve data, used to control output to magnetorquer, from sensors:
- LSM6DSOTR, 6-axis Inertial Measurement Unit
- IIS2MDCTR, Magnetometer 
- Sun Sensors  

Codebase also intends to handle filtering of immediate data recieved from such sensors as well as performing sensor fusion to produce meaningful results from Integration Board sensors.

## Installation Instructions
### STM32CubeIDE Github Setup 
- Navigate to this website https://community.st.com/t5/stm32-mcus/how-to-use-github-with-stm32cubeide/ta-p/793250
- Complete step 5 and onward

## Configuration
(May be updated during integration of the sensor filtering algorithms.)

### Integration_Board_Project Compiledb Build Configuration
> **Note**: This build configuration is only required if you plan to write code in another IDE using `clangd` instead of the STM32CubeIDE. You can format files using the included `.clang-format` without having to use this Compiledb build configuration.

The Compiledb build configuration uses the `compliedb` tool to generate the `compile_commands.json` file required by `clangd`, as the native STM32CubeIDE JSON generator does not function correctly in CubeIDE V2.2.0 as of August 8th, 2026. 

- Install `compiledb` from: https://github.com/nickdiego/compiledb
- Record the path to the directory containing `compiledb.exe`
- Open the **ADCS_Integration_Board** project in the STM32CubeIDE
- Right-click **Integration_Board_Project** -> **properties** -> **C/C++ Build** -> **Environment**
- Select the `PATH` variable and click **Edit...**
- Append to the end of the `PATH` variable's value `;<path_to_compiledb_dir>`
- Click `OK` then `Apply and Close`

You should now be able to build and clean using the Compiledb build configuration. This should create a `compile_commands.json` file under the *Integration_Board_Project* folder when building.

## Usage
Codebase is to be stored and ran on the STM32H7A3RGT6 MCU attached to the ADCS Integration Board, aims to run/test ADCS algorithms related to sensor-based output to AIB's magnetorquers. 

## Project Structure
Sun_Sensor_algo directory is used for creation and testing of algorithms related to converting received data from the sun sensors into usable data.  

Integration_Board_Project directory acts as the primary codebase of the repository and represents the actual development within the ADCS_Integration_Board Project. This directory is used to store code for pulling data from all sensors attached to the ADCS_Integration_Board for use in ADCS algorithms.  
- Integration_Board_Project/Core/Src/main.c contains the main codebase used for project

## Contributing
- Jayden Z
- Aiden H
- Michael W
- Bodhi B
- Luke L

## Contact


## License
- Project's usage of STM32CubeIDE is licensed under: [license](https://www.st.com/sla0048)
- Project's usage of IIS2MDC Driver code is licensed under: [license](https://github.com/STMicroelectronics/iis2mdc-pid/blob/master/LICENSE)
- Project's usage of LSM6DSOTR Driver code is licensed under: [license](https://github.com/STMicroelectronics/stm32-lsm6dso/blob/main/LICENSE.md)
- Project's usage of stm32h7xx_hal code is licensed under: [license](https://github.com/Selectronic-AU/stm32h7xx_hal_driver/blob/master/LICENSE.md)  