/*
 * This source file contains logic for filtering raw sensor data received from: IIS2MDC Magnetometer AND LSM6DSO Inertial Measurement Unit
 * Such sensor filtering is intended to be completed before sensor fusion, hence filtering is only to handle biases in sensor measurements
 * Approach to sensor filtering:
 * 	->Raw sensor has initIal fixed bias removed, simply: filtered = raw - <calculated/tested fixed bias>
 * 	->Apply Kalman Filtering (or a modified form) to handle instability/stability biases resulting from runtime-based, unpredictable external factors (i.e. Temperature/EMI)
*/


/* include header files */
#include "sensor_filtering_algs.h"
#include <stdint.h>

/* Define Structs containing fixed bias for x+y+z axis on each sensor */
/* define placeholder until all bias values can be determined */
#define PLACEHOLDER_BIAS 0

typedef struct {
	int16_t x;
	int16_t y;
	int16_t z;
} Fixed_Bias;

/* Will update each struct instance to contain appropriate value when able */
const static Fixed_Bias accel_fixed_bias = {PLACEHOLDER_BIAS, PLACEHOLDER_BIAS, PLACEHOLDER_BIAS};
const static Fixed_Bias gyro_fixed_bias = {PLACEHOLDER_BIAS, PLACEHOLDER_BIAS, PLACEHOLDER_BIAS};
const static Fixed_Bias mag_fixed_bias = {PLACEHOLDER_BIAS, PLACEHOLDER_BIAS, PLACEHOLDER_BIAS};

/*
* Function to filter fixed bias from raw sensor readings
* Imports reference to int pointer (expects array of length 3, [0]=x, [1]=y, [2]=z)
* Imports enum type indicating which sensor raw data is from
* Returns int pointer (int array of length 3) containg raw data with removed biases (chosen as opposed to updating raw data itself for testing)
*/
int* filter_fixed_bias(int* raw_data, Sensor_Type data_source) {
	int filtered_data[3];

	/* apply bias removal based on source of raw data */
	switch(data_source) {
		case ACCELEROMETER:
			filtered_data[0] = raw_data[0] - accel_fixed_bias.x; 
			filtered_data[1] = raw_data[1] - accel_fixed_bias.y;
			filtered_data[2] = raw_data[2] - accel_fixed_bias.z;
			break;
		case GYROSCOPE:
			filtered_data[0] = raw_data[0] - gyro_fixed_bias.x; 
			filtered_data[1] = raw_data[1] - gyro_fixed_bias.y;
			filtered_data[2] = raw_data[2] - gyro_fixed_bias.z;
			break;
		case MAGNETOMETER:
			filtered_data[0] = raw_data[0] - mag_fixed_bias.x; 
			filtered_data[1] = raw_data[1] - mag_fixed_bias.y;
			filtered_data[2] = raw_data[2] - mag_fixed_bias.z;
			break;
	}

	return filtered_data;
}
