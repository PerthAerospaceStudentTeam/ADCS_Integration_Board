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
#include <stdlib.h>


/* Define Structs containing fixed bias for x+y+z axis on each sensor */
/* define placeholder until all bias values can be determined */
#define PLACEHOLDER_BIAS 0
#define PLACEHOLDER_MEASUREMENT_VARIATION 1000

/* Will update each struct instance to contain appropriate value when able */
const static Fixed_Bias accel_fixed_bias = {PLACEHOLDER_BIAS, PLACEHOLDER_BIAS, PLACEHOLDER_BIAS};
const static Fixed_Bias gyro_fixed_bias = {PLACEHOLDER_BIAS, PLACEHOLDER_BIAS, PLACEHOLDER_BIAS};
const static Fixed_Bias mag_fixed_bias = {PLACEHOLDER_BIAS, PLACEHOLDER_BIAS, PLACEHOLDER_BIAS};

/* STRUCTS/VARIABLES FOR FIXED/STATE ESTIMATION FILTERING*/
typedef struct {
	int16_t x;
	int16_t y;
	int16_t z;
} Fixed_Bias;

#define RAW_MEASUREMENTS_SIZE 10 //define maximum number of raw measurements stored for each axis

/* Struct used to maintain raw measurements + key values for a particular axis */
typedef struct {
	int16_t raw_measurements[RAW_MEASUREMENTS_SIZE]; //array of most recent <RAW_MEASUREMENTS_SIZE> raw measurements for axis
	int16_t index; //stores index to insert next measurement into array
	int16_t min; //minimum raw measurement within most recent measurements
	int16_t max; //maximum raw measurement within most recent measurements
	int16_t avg_difference; //average difference across each measurement in raw_measurements array
	int16_t avg_measurement; //average of all measurement values within the raw_measurements array
} Axis_Measurements;

/* various inline functions used to calculate attributes of Axis_Measurements when updating raw_measurements */
static inline void UPDATE_INDEX(int16_t* index) {
	*(index)++; 
	if (*index % RAW_MEASUREMENTS_SIZE == 0) { *(index) = 0; } 
}

static inline int16_t FIND_MIN(int16_t measurements[RAW_MEASUREMENTS_SIZE]) {
	uint16_t i; int16_t curr_smallest = (measurements)[0]; 
	for (i = 1; i < RAW_MEASUREMENTS_SIZE; i++) { 
		if ((measurements)[i] < curr_smallest) { curr_smallest = (measurements)[i]; } 
	} 
	return curr_smallest;
}

static inline int16_t FIND_MAX(int16_t measurements[RAW_MEASUREMENTS_SIZE]) {
	uint16_t i; int16_t curr_largest = (measurements)[0]; 
	for (i = 1; i < RAW_MEASUREMENTS_SIZE; i++) { 
		if ((measurements)[i] > curr_largest) { curr_largest = (measurements)[i]; } 
	} 
	return curr_largest;
}

static inline int16_t FIND_AVERAGE(int16_t measurements[RAW_MEASUREMENTS_SIZE]) {
	uint16_t i; int32_t sum_average = 0; //upcast sum of measurements to avoid overflow
	for (i = 0; i < RAW_MEASUREMENTS_SIZE; i++) { 
		sum_average += (measurements)[i] ;
	} 
	return (int16_t) ( sum_average / RAW_MEASUREMENTS_SIZE );
}

static inline int16_t FIND_AVERAGE_DIFFERENCE(int16_t measurements[RAW_MEASUREMENTS_SIZE]) {
	uint16_t i; int32_t sum_differences = 0; //upcast sum of differences to avoid overflow
	for (i = 0; i < RAW_MEASUREMENTS_SIZE - 1; i++) { 
		sum_differences += abs((measurements)[i] - (measurements)[i+1]); 
	} 
	return (int16_t) ( sum_differences / (RAW_MEASUREMENTS_SIZE - 1));
}

/* 
* Function to add new measurement into a Axis_Measurements struct
* Must recalculate min, max, avg differnce and avg measurement value as well as ensure measurement added to correct index (within raw_measurements array)
* Imports:
*	-new_measurement (int16_t): new raw measurement to be stored
*	-axis_measurements (Axis_Measurements*): pointer to axis_measurements struct to update
*/
static void add_new_raw_measurement(int16_t new_measurement, Axis_Measurements* axis_measurements) {
	/* insert new measurement into array at next index, update index */
	axis_measurements->raw_measurements[axis_measurements->index] = new_measurement;
	UPDATE_INDEX(axis_measurements->index);

	/* recalculate other attributes for data */
	axis_measurements->min = FIND_MIN(axis_measurements->raw_measurements);
	axis_measurements->max = FIND_MAX(axis_measurements->raw_measurements);
	axis_measurements->avg_difference = FIND_AVERAGE_DIFFERENCE(axis_measurements->raw_measurements);
	axis_measurements->avg_measurement = FIND_AVERAGE(axis_measurements->raw_measurements);
}

/* Struct used to maintain variables required for state prediction algorithm */
typedef struct {
	Axis_Measurements raw_measurements;
	double kalman_gain;
	int16_t estimation_variation;
	int16_t state_estimation;
} State_Prediction_Variables;

/* Struct used to store variables required to filter each reading from IMU && MAG */
typedef struct {
	State_Prediction_Variables x;
	State_Prediction_Variables y;
	State_Prediction_Variables z;
	int16_t sensor_process_noise;
} Sensor_Reading_Filtering;

/* Store variables required to complete state prediction filtering algorithms for each sensor measurement */
static Sensor_Reading_Filtering accel_filtered_state;
static Sensor_Reading_Filtering gyro_filtered_state;
static Sensor_Reading_Filtering mag_filtered_state;

/* VARIABLES FOR PROCESS NOISE CALCULATIONS */

/* variables storing current data measurement rate in Hz for each sensor */
static int16_t accel_measure_rate = 833;
static int16_t gyro_measure_rate = 833;
static int16_t mag_measure_rate = 100;

/* variables storing estimated maximum measurement, currently placeholder, actualvariation will very likely be unique for each sensor */
static int16_t accel_measure_range = PLACEHOLDER_MEASUREMENT_VARIATION;
static int16_t gyro_measure_range = PLACEHOLDER_MEASUREMENT_VARIATION;
static int16_t mag_measure_range = PLACEHOLDER_MEASUREMENT_VARIATION;

/* Simple functions for internal use which simply update internal value for each sensor's process noise*/
static inline void update_accel_process_noise() { accel_filtered_state.sensor_process_noise = accel_measure_range / accel_measure_rate; }
static inline void update_gyro_process_noise() { gyro_filtered_state.sensor_process_noise = gyro_measure_range / gyro_measure_rate; }
static inline void update_mag_process_noise() { mag_filtered_state.sensor_process_noise = mag_measure_range / mag_measure_rate; }

/*
* Function to calculate process noise for each sensor (updates sensor_process_noise in required structs)
* Must be called at least once before data is to be read from sensors
*/
void calculate_sensor_process_noise() {	
	update_accel_process_noise();
	update_gyro_process_noise();
	update_mag_process_noise();
}

/* Functions to update measurement rate for each sensor, used if sensor reading rate is to change */
/* ( associated sensor's process noise is automatically recalculated ) */

/*
* Function to update measurement rate (in Hz) for accelerometer, auto updates process noise for accelerometer
* Imports: new_rate_Hz (new measurement rate of accelerometerin Hz)
*/
void update_accel_measure_rate(int16_t new_rate_Hz) {
	accel_measure_rate = new_rate_Hz;
	update_accel_process_noise();
}

/*
* Function to update measurement rate (in Hz) for gyroscope, auto updates process noise for gyroscope
* Imports: new_rate_Hz (new measurement rate of gyroscope in Hz)
*/
void update_gyro_measure_rate(int16_t new_rate_Hz) {
	gyro_measure_rate = new_rate_Hz;
	update_gyro_process_noise();
}

/*
* Function to update measurement rate (in Hz) for magnetometer, auto updates process noise for magnetometer
* Imports: new_rate_Hz (new measurement rate of magnetometer in Hz)
*/
void update_mag_measure_rate(int16_t new_rate_Hz) {
	mag_measure_rate = new_rate_Hz;
	update_mag_process_noise();
}

/*
* Function to filter fixed bias from raw sensor readings
* Imports: 
* 	-data (int16_t[3]): raw data to have fixed bias removed ([0]=x, [1]=y, [2]=z)
* 	-data_source (Sensor_Type): enum type indicating which sensor raw data is from
* Updates imported data to filtered version of data
*/
static inline void filter_fixed_bias(int16_t data[3], Sensor_Type data_source) {

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
* imports: 
* 	-p (int16_t):  representing extrapolated variance estimation
* 	-r (int16_t): representing variance in current measurement
* exports: new value of kalman gain (K), 0.0 <= K <= 1.0
 */
static inline double calculate_kalman_gain(int16_t p, int16_t r) {
	//Kalman-Gain = variance_in_estimation / (variance_in_estimation + variance_in_measurement)
	return ( (double)p / ( (double)p + (double)r ) );
}

/*
* Algorithm to calculate variance_in_estimation (p), determines the variance in current state prediction from prev.
* imports: 
* 	-k (double): representing kalman gain
* 	-p (int16_t): previous variance_in_estimation
* 	-n (int16_t): process noise
* exports: new value for variance_in_estimation
*/
static inline int16_t calculate_estimate_variation(double k, int16_t p, int16_t n) {
	return (int16_t)( (1.0 - k) * (double)p ) + n;
}

/*
* Algorithm to calculate current state_estimation (filtered measurement) using previous state estimation, kalman gain and new measurement
* imports: 
* 	-x (int16_t): Previous state_estimation
* 	-k (double): kalman gain
* 	-z (int16_t): measured system state
* exports: current estimation for state (i.e. filtered measurement for sensor reading)
*/
static inline int16_t calculate_state_estimation(int16_t x, double k, int16_t z) {
	return (int16_t)( (double)x + k * (double)(z - x) );
}

/*
* Function used to combine kalman state estimation filtering algorithms into single filtering operation for a single data reading
* imports: 
* 	-data (int16_t): new measurement to be filtered
* 	-state_predict_vars (State_Prediction_Variables*): reference to struct containing appropriate variables to apply state prediction
* 	-process_noise (int16_t): process noise of sensor
* Updates values stored within state_predict_vars to reflect new state prediction variables (state_predict_vars->state_estimation is filtered data)
* Exports: value of state_predict_vars->state_estimation after prediction occurs
*/
static inline int16_t predict_system_state(int16_t data, State_Prediction_Variables* state_predict_vars, int16_t process_noise) {
	int16_t measurement_variance;
	
	// calculate variance in current measurement from estimated state
	measurement_variance = abs( data - state_predict_vars->state_estimation );

	//apply state estimation algorithms in order (kalman->estimate_variation->state_estimation)
	state_predict_vars->kalman_gain = calculate_kalman_gain(state_predict_vars->estimation_variation, measurement_variance);
	state_predict_vars->estimation_variation = calculate_estimate_variation(state_predict_vars->kalman_gain, state_predict_vars->estimation_variation, process_noise);
	state_predict_vars->state_estimation = calculate_state_estimation(state_predict_vars->state_estimation, state_predict_vars->kalman_gain, data);

	return state_predict_vars->state_estimation;
}

/*
* Function used to test kalman state estimation filtering algorithm, modified to be used by external files (i.e. does not import struct specific to this file)
* Imports:
* 	-data (int16_t): new measurement
* 	-k (double*): pointer to kalman gain
* 	-e (int16_t*): pointer to estimation variation
* 	-s (int16_t*): pointer to state estimation
* 	-p (int16_t): process noise
* Exports new data (after filtering applied)
*/
int16_t predict_system_state_test(int16_t data, double* k, int16_t* e, int16_t* s, int16_t p) {
	// This function is only to temporarily exist to allow external files to test exlsuively the kalman filtering function (without fixed bias removal
	// Function logic mirrors predict_system_state, using imported values as opposed to struct
	int16_t r = abs(data - *(s));

	*k = calculate_kalman_gain(*e, r);
	*e = calculate_estimate_variation(*k, *e, p);
	*s = calculate_state_estimation(*s, *k, data);

	// explicity return filtered value, other values updated via address 
	return *s;

}

/*
* Function to apply kalman state estimation filtering for x, y, z readings from sensor
* imports:
* 	-data (int16_t[3]): 1D Array of 3 ints representing data to be filtered
* 	-data_source (Sensor_Type): used to apply and update correct state prediction variables
* updates each value in imported array to reflect predicted state for that value after kalman state estimation function applied
*/
void kalman_state_estimation(int16_t data[3], Sensor_Type data_source) {

	//filter data using appropriate state-estimation variables determined on source of data
	switch(data_source) {
		case ACCELEROMETER:
			data[0] = predict_system_state(data[0], &(accel_filtered_state.x), accel_filtered_state.sensor_process_noise); // x
			data[1] = predict_system_state(data[1], &(accel_filtered_state.y), accel_filtered_state.sensor_process_noise); // y
			data[2] = predict_system_state(data[2], &(accel_filtered_state.z), accel_filtered_state.sensor_process_noise); // z
			break;
		case GYROSCOPE:
			data[0] = predict_system_state(data[0], &(gyro_filtered_state.x), gyro_filtered_state.sensor_process_noise); // x
			data[1] = predict_system_state(data[1], &(gyro_filtered_state.y), gyro_filtered_state.sensor_process_noise); // y
			data[2] = predict_system_state(data[2], &(gyro_filtered_state.z), gyro_filtered_state.sensor_process_noise); // z
			break;
		case MAGNETOMETER:
			data[0] = predict_system_state(data[0], &(mag_filtered_state.x), mag_filtered_state.sensor_process_noise); // x
			data[1] = predict_system_state(data[1], &(mag_filtered_state.y), mag_filtered_state.sensor_process_noise); // y
			data[2] = predict_system_state(data[2], &(mag_filtered_state.z), mag_filtered_state.sensor_process_noise); // z
			break;
	}
}

/*
* Function used to filter x, y, z data from a particular sensor
* ASSUMES: process noise has been calculated for each sensor (i.e. invoked calculate_sensor_process_noise() before calling this function)
* Imports:
* 	-data (int16_t[3]): 1D Array of 3 ints representing data to be filtered
* 	-data_source (Sensor_Type): used to apply and update correct state prediction variables
* Updates imported data so that values stored in array represent new, filtered data after fixed bias and instability/stability biases are removed
*/
void filter_sensor_data(int16_t data[3], Sensor_Type data_source) {
	//First filter fixed biases from readings, then apply kalman filtering for instability/stability random biases
	filter_fixed_bias(data, data_source);
	kalman_state_estimation(data, data_source);
}