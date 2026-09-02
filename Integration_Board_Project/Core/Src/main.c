/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lsm6dso_reg.h"
#include "iis2mdc_reg.h"
#include "sensor_filtering_algs.h" //temporary, just for testing
#include <stdio.h>
#include <string.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

stmdev_ctx_t lsm6dso_ctx;
stmdev_ctx_t iis2mdc_ctx;

/* COMMUNICATION FUNCTIONS */
/* Functions should not handle failed write operation itself (i.e. loop until success), this should be responsibility of calling function */
/* All possible return values of functions correspond to COMMUNICATION_... macros defined in main.h */

// LSM6DSO SPI1, CS = PB4
int32_t lsm6dso_write(void *handle, uint8_t reg,
                      const uint8_t *bufp, uint16_t len)
{
    reg &= 0x7F;
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET); // CS Low

    /* write register address to peripheral */
    int32_t write_status = HAL_SPI_Transmit(&hspi1, &reg, 1, 100);

    /* only attempt to write data to peripheral if register write was successful */
    if (write_status == COMMUNICATION_SUCCESS) {
        write_status = HAL_SPI_Transmit(&hspi1, (uint8_t*)bufp, len, 100);
    }

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET); // CS High
    return write_status;

}

int32_t lsm6dso_read(void *handle, uint8_t reg,
                     uint8_t *bufp, uint16_t len)
{
	  reg |= 0x80;
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET); //CS Low

    /* write register address to peripheral */
	  int32_t comm_status = HAL_SPI_Transmit(&hspi1, &reg, 1, 1000);

    /* only attempt to read data from peripheral if register write was successful */
    if (comm_status == COMMUNICATION_SUCCESS) {
        comm_status = HAL_SPI_Receive(&hspi1, bufp, len, 1000);
    }

	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
    return comm_status;
}

// IIS2MDC SPI2, CS = PB2
int32_t mag_platform_write(void *handle, uint8_t reg,
                           const uint8_t *bufp, uint16_t len)
{
    reg &= 0x7F;
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);

    /* write the register address to peripheral device */
    int32_t write_status = HAL_SPI_Transmit(&hspi2, &reg, 1, HAL_MAX_DELAY);

    /* only attempt to write data to peripheral if register write was successful */
    if (write_status == COMMUNICATION_SUCCESS) {
        write_status = HAL_SPI_Transmit(&hspi2, (uint8_t *)bufp, len, HAL_MAX_DELAY);
    }

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
    return write_status;
}

int32_t mag_platform_read(void *handle, uint8_t reg,
                          uint8_t *bufp, uint16_t len)
{
    reg |= 0x80;
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);

    /* write address of register to peripheral device */
    int32_t comm_status = HAL_SPI_Transmit(&hspi2, &reg, 1, HAL_MAX_DELAY);

    /* only attempt to read data from peripheral device if register write to device was successful */
    if (comm_status == COMMUNICATION_SUCCESS) {
        comm_status = HAL_SPI_Receive(&hspi2, bufp, len, HAL_MAX_DELAY);
    }

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
    return comm_status;
}

/* Conversion helper */
float raw_accel_to_mss(int16_t raw)
{
    return (float)raw * 0.00059f; //conversion found in data sheet says 0.00061 but it overshoots...
}

float raw_gyro_to_degreespersecond(int16_t raw){
	return raw * 0.00875f; //value from data sheet -- angular rate sensitivity type
}


// interrupt handler to ensure smooth ADC readings
uint16_t sun[6];
int ADC_Finished = 0;
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	ADC_Finished = 1;
}



//initiate variables for sun sensors
int maxIntensity[6]; // Maximum sensor values
int minIntensity[6]; // Minimum sensor values
void resetCalibration() {
    for (int i = 0; i < 6; i++) {
        maxIntensity[i] = 0;
        minIntensity[i] = 65535; // 12-bit ADC range
    }
    char SUN_Calibration[] = "Calibration Of ADC Sun Sensors";
    HAL_UART_Transmit(&huart1, (uint8_t*) SUN_Calibration, strlen(SUN_Calibration) ,100);
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_SPI2_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  // LSM6DSO initialize
  lsm6dso_ctx.write_reg = lsm6dso_write;
  lsm6dso_ctx.read_reg  = lsm6dso_read;
  lsm6dso_ctx.handle    = &hspi1;

  // IIS2MDC initialize
  iis2mdc_ctx.write_reg = mag_platform_write;
  iis2mdc_ctx.read_reg  = mag_platform_read;
  iis2mdc_ctx.handle    = &hspi2;

  /* -------- LSM6DSO INIT -------- */

  lsm6dso_spi_mode_set(&lsm6dso_ctx,LSM6DSO_SPI_3_WIRE); //using the stm32 lsm6dso library and should use this to work with 3 wire mode
  lsm6dso_auto_increment_set(&lsm6dso_ctx, 1);
  lsm6dso_xl_data_rate_set(&lsm6dso_ctx, LSM6DSO_XL_ODR_833Hz );
  lsm6dso_block_data_update_set(&lsm6dso_ctx, 1);
  lsm6dso_gy_full_scale_set(&lsm6dso_ctx, LSM6DSO_250dps);
  lsm6dso_gy_data_rate_set(&lsm6dso_ctx, LSM6DSO_GY_ODR_833Hz);


  /* -----iis2mdc INIT ----- */
  iis2mdc_data_rate_set(&iis2mdc_ctx, IIS2MDC_ODR_100Hz);
  iis2mdc_block_data_update_set(&iis2mdc_ctx, 1);

  //WHOAMI
  uint8_t VALUE = 0;
  lsm6dso_read(NULL, 0x0F, &VALUE, 1);

  //testing to check if i can comm with the imu
  char string[32];
  sprintf(string, "LSM6DOWHO_AM_I = 0x%02X \r\n", VALUE);
  HAL_UART_Transmit(&huart1, (uint8_t *)string, strlen(string), HAL_MAX_DELAY);


  uint8_t whoami = 0;
  iis2mdc_device_id_get(&iis2mdc_ctx, &whoami);

  char msg[32];
  sprintf(msg, "IIS2MDC WHO_AM_I = 0x%02X\r\n", whoami);
  HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

  /* Variables */
  int16_t raw_accel[3];
  float accel_mss[3];
  char accel_data[64];

  int16_t raw_gyro[3];
 // volatile char gyro_data[64]; // used for printing the raw gyro data - redundant
  float gyro_intermediate[3];
  float G_X_roll= 0.0f;
  float G_Y_pitch = 0.0f;
  float G_Z_yaw= 0.0f;
  char Gyro_accum[64];
  char OFFSET[64];

  int16_t raw_mag[3];
  char mag_data[64];

  float dt = 0.0f;

  /* testing Sun sensors */

  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)sun, 6);
  int16_t Z_Minus;
  int16_t Z_Plus;
  int16_t X_Plus;
  int16_t Y_Plus;
  int16_t X_Minus;
  int16_t Y_Minus;

  char SUN_DATA[100];





// testing the pwm channels are working for all the magnetometers
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0); // ~50% of 255
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 123); // ~50% of 255
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 123);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 123);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 123);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 123);

  //gyro bias reduction
	/*
  lsm6dso_angular_rate_raw_get(&lsm6dso_ctx, raw_gyro);
  gyro_intermediate[0]= raw_gyro_to_degreespersecond(raw_gyro[0]);
  gyro_intermediate[1]= -raw_gyro_to_degreespersecond(raw_gyro[1]);
  gyro_intermediate[2]= raw_gyro_to_degreespersecond(raw_gyro[2]);
  float sumx = 0.0f, sumy = 0.0f, sumz = 0.0f;
  char waiting[] = "Calibrating offset values";
  HAL_UART_Transmit(&huart1, (uint8_t*) waiting, strlen(waiting) ,100);
  int n = 1; //number of calibration trials
  for (int i = 0; i < n; i++){
  	sumx += gyro_intermediate[0];
  	sumy += gyro_intermediate[1];
  	sumz += gyro_intermediate[2];
  	HAL_Delay (3);
  }
  */

//  float offset_x = sumx/(n-1);
//  float offset_y = sumy/(n-1);
//  float offset_z = sumz/(n-1);
//	sprintf(OFFSET, "OFFSET X=%.5f Y=%.5f Z=%.5f \r\n",offset_x, offset_y, offset_z);
//	HAL_UART_Transmit(&huart1, (uint8_t *)OFFSET, strlen(OFFSET), HAL_MAX_DELAY);
//	HAL_Delay(2000);


  //init sensor filter algs by calculating process noise for each sensor
  //calculate_sensor_process_noise();

  uint32_t last_tick = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  HAL_Delay(500);




	  //code to display raw data from sensors in the form of 2^16- 1
//	  if (ADC_Finished == 1){
//		  ADC_Finished = 0;
//		  for(uint8_t i = 0; i<hadc1.Init.NbrOfConversion; i++){
//			  Z_Minus= sun[0];
//			  Z_Plus = sun[1];
//			  X_Plus = sun[2];
//			  Y_Plus = sun[3];
//			  X_Minus = sun[4];
//			  Y_Minus = sun[5];
//		  }
//		  sprintf(SUN_DATA, "-Z = %u , +Z = %u, -X = %u, +X = %u, -Y = %u, +Y = %u \r\n", Z_Minus, Z_Plus, X_Minus, X_Plus, Y_Minus, Y_Plus);
//		  HAL_UART_Transmit(&huart1, (uint8_t*)SUN_DATA, strlen(SUN_DATA), HAL_MAX_DELAY);
//		  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)sun, 6);
//	  }


//	  uint16_t SUN_Zp = sun[0]; //this block can be deleted
//	  uint16_t SUN_Xm = sun[1];
//	  sprintf(sun_outputs, "Z Positive: %u, X Negative: %u \r\n ", SUN_Zp, SUN_Xm);
//	  HAL_UART_Transmit(&huart1,(uint8_t*)sun_outputs, strlen(sun_outputs), HAL_MAX_DELAY);
//	  HAL_Delay(500);

//	  HAL_ADC_Start(&hadc1); //this block can be deleted
//	  HAL_ADC_PollForConversion(&hadc1, 100);
//	  sun_Zp = HAL_ADC_GetValue(&hadc1);
//	  sprintf(sun_output, "Sun: %u \r\n ", sun_Zp);
//	  HAL_UART_Transmit(&huart1, (uint8_t*)sun_output, strlen(sun_output), HAL_MAX_DELAY);
//	  HAL_Delay(500);

//  	//getting dt from integrating gyro
//  	uint32_t now = HAL_GetTick();
//  	float dt = (now - last_tick	) / 1000.0f;
//  	last_tick = now;
//
	  	//accelerometer
	  	lsm6dso_acceleration_raw_get(&lsm6dso_ctx, raw_accel);
	  	char rawAccel[100];
	  	char filterAccel[100];
	  	sprintf(rawAccel, "RawAccel: x: %i y: %i z: %i.", raw_accel[0], raw_accel[1], raw_accel[2]);
	  	filter_sensor_data(raw_accel, ACCELEROMETER);
	  	sprintf(filterAccel, "\tFilteredAccel: x: %i y: %i z: %i.\r\n", raw_accel[0], raw_accel[1], raw_accel[2]);

	  	HAL_UART_Transmit(&huart1, (int8_t*) rawAccel, strlen(rawAccel), 100);
	  	HAL_UART_Transmit(&huart1, (int8_t*) filterAccel, strlen(filterAccel), 100);

	  	//print raw values for accelerometer
	  	//snprintf(accel_data, sizeof(accel_data), "RawAccel: x: %i\ty: %i\tz: %i.", raw_accel[1], raw_accel[0], raw_accel[2]);
	  	//filter raw data
	  	//filter_sensor_data(raw_accel, Sensor_Type.ACCELEROMETER);
	  	//print filtered values for accelerometer
	  	//snprintf(accel_data, sizeof(accel_data), "\tFilteredAccel: x: %i\ty: %i\tz: %i.\r\n", raw_accel[1], raw_accel[0], raw_accel[2]);
//
//      accel_mss[0] = raw_accel_to_mss(raw_accel[0]);
//      accel_mss[1] = raw_accel_to_mss(raw_accel[1]);
//      accel_mss[2] = raw_accel_to_mss(raw_accel[2]);
//
//      snprintf(accel_data, sizeof(accel_data), "Accel X=%.2f Y=%.2f Z=%.2f\r\n", accel_mss[1], -accel_mss[0], accel_mss[2]);
//      HAL_UART_Transmit(&huart1, (uint8_t *)accel_data, strlen(accel_data), HAL_MAX_DELAY);
//
//      //gyro
	  	lsm6dso_angular_rate_raw_get(&lsm6dso_ctx, raw_gyro);
	  	char rawGyro[100];
	  	char filterGyro[100];
	  	sprintf(rawGyro, "RawGyro: x: %i y: %i z: %i.", raw_gyro[0], raw_gyro[1], raw_gyro[2]);
	  	filter_sensor_data(raw_gyro, GYROSCOPE);
	  	sprintf(filterGyro, "\tFilteredGyro: x: %i y: %i z: %i.\r\n", raw_gyro[0], raw_gyro[1], raw_gyro[2]);

	  	HAL_UART_Transmit(&huart1, (int8_t*) rawGyro, strlen(rawGyro), 100);
	  	HAL_UART_Transmit(&huart1, (int8_t*) filterGyro, strlen(filterGyro), 100);

	  	//snprintf(raw_gyro, sizeof(raw_gyro), "RawGyro: x: %i\ty: %i\tz: %i.", raw_gyro[0], raw_gyro[1], raw_gyro[2]);
	  	//filter raw gyro data
	  	//filter_sensor_data(raw_gyro, Sensor_Type.GYROSCOPE);
	  	//print filtered gyro values
	  	//snprintf(gyro_data, sizeof(gyro_data), "\tFilteredGyro: x: %i\ty: %i\tz: %i.\r\n", raw_gyro[0], raw_gyro[1], raw_gyro[2]);

//
//      gyro_intermediate[0]= raw_gyro_to_degreespersecond(raw_gyro[0]) -offset_x ;
//      gyro_intermediate[1]= -raw_gyro_to_degreespersecond(raw_gyro[1]) - offset_y ;
//      gyro_intermediate[2]= raw_gyro_to_degreespersecond(raw_gyro[2]) - offset_z;
////        snprintf(gyro_data, sizeof(gyro_data), "GYRO X=%.2f Y=%.2f Z=%.2f \r\n", gyro_intermediate[0], gyro_intermediate[1], gyro_intermediate[2]);
////        HAL_UART_Transmit(&huart1, (uint8_t *)gyro_data, strlen(gyro_data), HAL_MAX_DELAY);
//      G_X_roll += (gyro_intermediate[0]) * dt;
//      G_Y_pitch += (gyro_intermediate[1] )* dt ;
//      G_Z_yaw += (gyro_intermediate[2]) * dt ;
//
//      sprintf(Gyro_accum, "GYRO roll=%.2f pitch=%.2f yaw=%.2f \r\n",G_X_roll, G_Y_pitch, G_Z_yaw);
//      HAL_UART_Transmit(&huart1, (uint8_t *)Gyro_accum, strlen(Gyro_accum), HAL_MAX_DELAY);
//
//
//     // loop for magnetometer data
        iis2mdc_magnetic_raw_get(&iis2mdc_ctx, raw_mag);
	  	char rawMag[100];
	  	char filterMag[100];
	  	sprintf(rawMag, "RawMag: x: %i y: %i z: %i.", raw_mag[0], raw_mag[1], raw_mag[2]);
	  	filter_sensor_data(raw_mag, MAGNETOMETER);
	  	sprintf(filterMag, "\tFilteredMag: x: %i y: %i z: %i.\r\n", raw_mag[0], raw_mag[1], raw_mag[2]);

	  	HAL_UART_Transmit(&huart1, (int8_t*) rawMag, strlen(rawMag), 100);
	  	HAL_UART_Transmit(&huart1, (int8_t*) filterMag, strlen(filterMag), 100);

        //sprintf(mag_data, "RawMag: x: %i\ty: %i\tz: %i.", raw_mag[0], raw_mag[1], raw_mag[2]);
        //filter_sensor_data(raw_mag, Sensor_Type.MAGNETOMETER);
        //sprintf(mag_data, "\tFilterMag: x: %i\ty: %i\tz: %i.\r\n", raw_mag[0], raw_mag[1], raw_mag[2]);
//      HAL_UART_Transmit(&huart1, (uint8_t*)mag_data, strlen(mag_data), HAL_MAX_DELAY);
	  	//HAL_Delay(1);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /*AXI clock gating */
  RCC->CKGAENR = 0xE003FFFF;

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = 64;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_16B;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 6;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_ONESHOT;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_1LINE;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 0x0;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi1.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_1LINE;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 0x0;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  hspi2.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi2.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi2.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi2.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi2.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi2.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi2.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi2.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi2.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 255;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 255;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);

  /*Configure GPIO pins : PB2 PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
