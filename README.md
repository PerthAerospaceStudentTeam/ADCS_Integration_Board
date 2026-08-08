# ADCS_Integration_Board
Code base for the ADCS integration board, used to test ADCS algorithms.  
Utilises the STM32H7A3RGT6 as central MCU to retrieve data, used to control output to magnetorquer, from sensors:
- LSM6DSOTR, 6-axis Inertial Measurement Unit
- IIS2MDCTR, Magnetometer 
- Sun Sensors 

## Installation Instructions

### STM32CubeIDE GitHub Setup
- Navigate to this website https://community.st.com/t5/stm32-mcus/how-to-use-github-with-stm32cubeide/ta-p/793250
- Complete step 5 and onward

## Configuration

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
Code to be used on PAST's ADCS Integration board, aims to run/test ADCS algorithms.

## Project Structure
***Will update when able to consult with Jayden in person for more info on exact project structure***
Sun_Sensor_algo directory contains algorithms for handling raw data retrieved from the sun sensors.
Integration_Board_Project directory contains code for pulling data from the LSM6DSOTR and IIS2MDCTR sensors, converting such raw data into usable information.

## Contributing
- Jayden Z
- Aiden H
- Bodhi B
- Luke L

## Contact

## License
