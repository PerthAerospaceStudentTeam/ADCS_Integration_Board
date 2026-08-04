/*
 * This source file contains logic for filtering raw sensor data received from: IIS2MDC Magnetometer AND LSM6DSO Inertial Measurement Unit
 * Such sensor filtering is intended to be completed before sensor fusion, hence filtering is only to handle biases in sensor measurements
 * Approach to sensor filtering:
 * 	->Raw sensor has initIal fixed bias removed, simply: filtered = raw - <calculated/tested fixed bias>
 * 	->Apply Kalman Filtering (or a modified form) to handle instability/stability biases resulting from runtime-based, unpredictable external factors (i.e. Temperature/EMI)
 * 
 * Filtering functions currently return arrays of filtered data, this approach is likely temporary, and is mainly for testing (allows for both the raw and filtered data to be compared)
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


/* UNSURE if this will be final implementation, may hopefully figure out how to optomise in future (i.e. requires less struct instances) */

/* Struct used to maintain variables required for state prediction algorithm */
typedef struct {
	double kalman_gain;
	int16_t estimation_variation;
	int16_t state_estimation;
} State_Prediction_Variables;

/* Struct used to store variables required to filter each reading from IMU && MAG */
typedef struct {
	State_Prediction_Variables x;
	State_Prediction_Variables y;
	State_Prediction_Variables z;
} Sensor_Reading_Filtering;

/* Will update each struct instance to contain appropriate value when able */
const static Fixed_Bias accel_fixed_bias = {PLACEHOLDER_BIAS, PLACEHOLDER_BIAS, PLACEHOLDER_BIAS};
const static Fixed_Bias gyro_fixed_bias = {PLACEHOLDER_BIAS, PLACEHOLDER_BIAS, PLACEHOLDER_BIAS};
const static Fixed_Bias mag_fixed_bias = {PLACEHOLDER_BIAS, PLACEHOLDER_BIAS, PLACEHOLDER_BIAS};

/* Store variables required to complete state prediction filtering algorithms for each sensor measurement */
Sensor_Reading_Filtering accel_filtered_state = { {0.0, 0, 0}, {0.0, 0, 0}, {0.0, 0, 0} };
Sensor_Reading_Filtering gyro_filtered_state = { {0.0, 0, 0}, {0.0, 0, 0}, {0.0, 0, 0} };
Sensor_Reading_Filtering mag_filtered_state = { {0.0, 0, 0}, {0.0, 0, 0}, {0.0, 0, 0} };

/*
* Function to filter fixed bias from raw sensor readings
* Imports reference to int pointer (expects array of length 3, [0]=x, [1]=y, [2]=z)
* Imports enum type indicating which sensor raw data is from
* Returns int pointer (int array of length 3) containg raw data with removed biases (chosen as opposed to updating raw data itself for testing)
*/
int16_t* filter_fixed_bias(int16_t* raw_data, Sensor_Type data_source) {
	int16_t filtered_data[3];

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

/*
* Following set of functions aims to implement the 3 algorithms used in state prediction Kalman filtering algorithm
* Heavily based on algorithms found at: https://kalmanfilter.net/kalman1d.html
*/

/*
* Algorithm to calculate the Kalman gain, (determines the 'strength' given to new measurements)
* imports: p (representing extrapolated variance estimation), r (representing variance in current measurement)
* exports: new value of kalman gain (K), 0.0 <= K <= 1.0
 */
static double calculate_kalman_gain(int16_t p, int16_t r) {
	//Kalman-Gain = variance_in_estimation / (variance_in_estimation + variance_in_measurement)
	return ( (double)p / ( (double)p + (double)r ) );
}

/*
* Algorithm to calculate variance_in_estimation (p), determines the variance in current state prediction from prev.
* imports: k (representing kalman gain), p (previous variance_in_estimation)
* exports: new value for variance_in_estimation
*/
static int16_t calculate_estimate_variation(double k, int16_t p) {
	return (int16_t)( (1.0 - k) * (double)p );
}

/*
* Algorithm to calculate current state_estimation, actually
* imports: x (Previous state_estimation), k (kalman gain), z (measured system state)
* exports: current estimation for state (i.e. filtered measurement for sensor reading)
*/
static int16_t calculate_state_estimation(int16_t x, double k, int16_t z) {
	return (int16_t)( (double)x + k * (double)(z - x) );
}

/*
* Function used to combine kalman state estimation filtering algorithms into single filtering operation for a single data reading
* imports data (new measurement to be filtered), state_predict_vars (reference to struct containing appropriate variables to apply state prediction)
* Updates values stored within state_predict_vars to reflect new state prediction variables (state_predict_vars->state_estimation is filtered data)
* Exports: value of state_predict_vars->state_estimation after prediction occurs
*/
int16_t predict_system_state(int16_t data, State_Prediction_Variables* state_predict_vars) {
	int16_t measurement_variance;
	
	// calculate variance in current measurement from estimated state
	measurement_variance = ( data - state_predict_vars->state_estimation );

	//apply state estimation algorithms in order
	state_predict_vars->kalman_gain = calculate_kalman_gain(state_predict_vars->estimation_variation, measurement_variance);
	state_predict_vars->estimation_variation = calculate_estimate_variation(state_predict_vars->kalman_gain, state_predict_vars->estimation_variation);
	state_predict_vars->state_estimation = calculate_state_estimation(state_predict_vars->state_estimation, state_predict_vars->kalman_gain, data);

	return state_predict_vars->state_estimation;
}

/*
* Function to apply kalman state estimation filtering for x, y, z readings from sensor
* imports data (1D Array of 3 ints represnting data to be filtered), data_source (used to apply and update correct state prediction variables)
* Exports: 1d array of 3 ints representing new data after filtering (system state representing data has been predicted)
*/
int16_t* kalman_state_estimation(int16_t* data, Sensor_Type data_source) {
	int16_t predicted_states[3];

	//filter data using appropriate state-estimation variables determined on source of data
	switch(data_source) {
		case ACCELEROMETER:
			predicted_states[0] = predict_system_state(data[0], &(accel_filtered_state.x)); // x
			predicted_states[1] = predict_system_state(data[1], &(accel_filtered_state.x)); // y
			predicted_states[2] = predict_system_state(data[2], &(accel_filtered_state.x)); // z
			break;
		case GYROSCOPE:
			predicted_states[0] = predict_system_state(data[0], &(gyro_filtered_state.x)); // x
			predicted_states[1] = predict_system_state(data[1], &(gyro_filtered_state.x)); // y
			predicted_states[2] = predict_system_state(data[2], &(gyro_filtered_state.x)); // z
			break;
		case MAGNETOMETER:
			predicted_states[0] = predict_system_state(data[0], &(mag_filtered_state.x)); // x
			predicted_states[1] = predict_system_state(data[1], &(mag_filtered_state.x)); // y
			predicted_states[2] = predict_system_state(data[2], &(mag_filtered_state.x)); // z
			break;
	}

	return predicted_states;
}

/*
* Function used to filter x, y, z data from a particular sensor
* Imports: raw_data (1D Array of 3 ints represnting data to be filtered), data_source (used to apply and update correct state prediction variables)
* Exports: 1d array of 3 ints representing new data after filtering
*/
int16_t* filter_sensor_data(int16_t* raw_data, Sensor_Type data_source) {
	int16_t filtered_data;

	//First filter fixed biases from readings, then apply kalman filtering
	filtered_data = filter_fixed_bias(raw_data, data_source);
	filtered_data = kalman_state_estimation(filtered_data, data_source);

	return filtered_data;
}