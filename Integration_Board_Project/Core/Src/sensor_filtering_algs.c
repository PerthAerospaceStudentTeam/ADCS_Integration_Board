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
static Sensor_Reading_Filtering accel_filtered_state = { {0.0, 0, 0}, {0.0, 0, 0}, {0.0, 0, 0} };
static Sensor_Reading_Filtering gyro_filtered_state = { {0.0, 0, 0}, {0.0, 0, 0}, {0.0, 0, 0} };
static Sensor_Reading_Filtering mag_filtered_state = { {0.0, 0, 0}, {0.0, 0, 0}, {0.0, 0, 0} };

/*
* Function to filter fixed bias from raw sensor readings
* Imports reference to int array (expects array of length 3, [0]=x, [1]=y, [2]=z)
* Imports enum type indicating which sensor raw data is from
* Updates imported data to filtered version of data
*/
void filter_fixed_bias(int16_t data[3], Sensor_Type data_source) {

	// function is only temporarily visible outside of file for testing

	/* apply bias removal based on source of raw data */
	switch(data_source) {
		case ACCELEROMETER:
			data[0] -= accel_fixed_bias.x; 
			data[0] -= accel_fixed_bias.y; 
			data[0] -= accel_fixed_bias.z; 
			break;
		case GYROSCOPE:
			data[0] -= gyro_fixed_bias.x; 
			data[0] -= gyro_fixed_bias.y; 
			data[0] -= gyro_fixed_bias.z; 
			break;
		case MAGNETOMETER:
			data[0] -= mag_fixed_bias.x; 
			data[0] -= mag_fixed_bias.y; 
			data[0] -= mag_fixed_bias.z; 
			break;
	}
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

	//apply state estimation algorithms in order (kalman->estimate_variation->state_estimation)
	state_predict_vars->kalman_gain = calculate_kalman_gain(state_predict_vars->estimation_variation, measurement_variance);
	state_predict_vars->estimation_variation = calculate_estimate_variation(state_predict_vars->kalman_gain, state_predict_vars->estimation_variation);
	state_predict_vars->state_estimation = calculate_state_estimation(state_predict_vars->state_estimation, state_predict_vars->kalman_gain, data);

	return state_predict_vars->state_estimation;
}

/*
* Function used to test kalman state estimation filtering algorithm, modified to be used by external files (i.e. does not import struct specific to this file)
* Imports data (new measurement), k (kalman gain), e (estimation variation), s (state estimation)
* Exports new data (after filtering applied)
*/
int16_t predict_system_state_test(int16_t data, double k, int16_t e, int16_t s) {
	// This function is only to temporarily exist to allow external files to test exlsuively the kalman filtering function (without fixed bias removal
	// Function simply calls the predict_system_state function with imported vars

	//create the state_prediction_variables struct
	State_Prediction_Variables state_predict_vars = { k, e, s };

	// call predict_system_state function and return result
	return predict_system_state(data, &state_predict_vars);

}

/*
* Function to apply kalman state estimation filtering for x, y, z readings from sensor
* imports data (1D Array of 3 ints represnting data to be filtered), data_source (used to apply and update correct state prediction variables)
* updates each value in imported array to reflect predicted state for that value after kalman state estimation function applied
*/
void kalman_state_estimation(int16_t data[3], Sensor_Type data_source) {

	//filter data using appropriate state-estimation variables determined on source of data
	switch(data_source) {
		case ACCELEROMETER:
			data[0] = predict_system_state(data[0], &(accel_filtered_state.x)); // x
			data[1] = predict_system_state(data[1], &(accel_filtered_state.y)); // y
			data[2] = predict_system_state(data[2], &(accel_filtered_state.z)); // z
			break;
		case GYROSCOPE:
			data[0] = predict_system_state(data[0], &(gyro_filtered_state.x)); // x
			data[1] = predict_system_state(data[1], &(gyro_filtered_state.y)); // y
			data[2] = predict_system_state(data[2], &(gyro_filtered_state.z)); // z
			break;
		case MAGNETOMETER:
			data[0] = predict_system_state(data[0], &(mag_filtered_state.x)); // x
			data[1] = predict_system_state(data[1], &(mag_filtered_state.y)); // y
			data[2] = predict_system_state(data[2], &(mag_filtered_state.z)); // z
			break;
	}
}

/*
* Function used to filter x, y, z data from a particular sensor
* Imports: data (1D Array of 3 ints represnting data to be filtered), data_source (used to apply and update correct state prediction variables)
* Updates imported data so that values stored in array represent new, filtered data fater fixed bias and instability/stability biases are removed
*/
void filter_sensor_data(int16_t data[3], Sensor_Type data_source) {
	//First filter fixed biases from readings, then apply kalman filtering for instability/stability random biases
	filter_fixed_bias(data, data_source);
	kalman_state_estimation(data, data_source);
}