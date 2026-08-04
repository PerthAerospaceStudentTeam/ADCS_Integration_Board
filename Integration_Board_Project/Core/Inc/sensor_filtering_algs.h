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
	int16_t* filter_fixed_bias(int16_t* raw_data, Sensor_Type data_source);
	int16_t* kalman_state_estimation(int16_t* data, Sensor_Type data_source);
	int16_t* filter_sensor_data(int16_t* raw_data, Sensor_Type data_source);

	/* only included here for basic testing if needed, will likely be removed soon */
	int16_t predict_system_state(int16_t data, State_Prediction_Variables* state_predict_vars);
	
	#ifdef __cplusplus
	}
	#endif

#endif
