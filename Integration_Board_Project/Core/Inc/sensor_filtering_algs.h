/* define self, prevent inclusion recursion issues */
#ifndef SENSOR_FILTERING_ALG_H
	#define SENSOR_FILTERING_ALG_H

	/* allow compatibility with .cpp files */
	#ifdef __cplusplus
	extern "C" {
	#endif

	/* Define enum used to indicate sensor, used in filtering function to apply correct fixed bias removal */
	typedef enum {
		ACCELEROMETER = 0,
		GYROSCOPE,
		MAGNETOMETER
	} Sensor_Type;


	/* Function declarations */
	int* filter_fixed_bias(int* raw_data, Sensor_Type data_source);

	#ifdef __cplusplus
	}
	#endif

#endif
