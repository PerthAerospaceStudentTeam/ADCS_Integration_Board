/* define self, prevent inclusion recursion issues */
#ifndef SENSOR_FILTERING_ALG_H
	#define SENSOR_FILTERING_ALG_H

	/* allow compatibility with .cpp files */
	#ifdef __cplusplus
	extern "C" {
	#endif

	#include <stdint.h>

	/* Define enum used to indicate sensor, used in filtering function to apply correct fixed bias removal */
	typedef enum {
		ACCELEROMETER = 0,
		GYROSCOPE,
		MAGNETOMETER
	} Sensor_Type;


	/* Function declaration (actual function provided to filter sensor data) */
	void filter_sensor_data(int16_t data[3], Sensor_Type data_source);

	/* Testing functions (these functions are temporarily available to other files to test independently) */
	int16_t predict_system_state_test(int16_t data, double* k, int16_t* e, int16_t* s);
	void filter_fixed_bias(int16_t raw_data[3], Sensor_Type data_source);
	void kalman_state_estimation(int16_t data[3], Sensor_Type data_source);

	#ifdef __cplusplus
	}
	#endif

#endif
