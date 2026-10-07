/* define self, prevent recursive inclusion issues */
#ifndef SENSOR_FILTERING_ALG_H
	#define SENSOR_FILTERING_ALG_H

	/* allow compatibility with .cpp files */
	#ifdef __cplusplus
	extern "C" {
	#endif

	#include <stdint.h>

	/* Define enum used to indicate sensor, used in filtering function to apply correct fixed bias removal */
	/* Passed alongside raw data into 'filter_sensor_data' to indicate which sensor that data to be filtered belongs to */
	typedef enum {
		ACCELEROMETER = 0,
		GYROSCOPE,
		MAGNETOMETER
	} Sensor_Type;

	/*
	* Function used to filter x, y, z data from a particular sensor
	* Imports:
	* 	-data (int16_t[3]): 1D Array of 3 ints representing data to be filtered
	*		-IMPORTANT: data must be in format: new_raw_measurements_for_sensor[x, y, z]
	* 	-data_source (Sensor_Type): used to apply and update correct state prediction variables based on sensor
	* Values in 'data' are updated with estimated state for each sensor axis produced by filtering algorithms 
	*/
	void filter_sensor_data(int16_t data[3], Sensor_Type data_source);
	#ifdef __cplusplus
	}
	#endif

#endif
