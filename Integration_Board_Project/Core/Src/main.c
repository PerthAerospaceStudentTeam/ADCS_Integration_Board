/* USER CODE BEGIN Header */
/**
 *****************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 *****************************************************************************
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
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "iis2mdc_reg.h"
#include "lsm6dso_reg.h"
#include "stm32h7xx_hal_def.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/*
 * Handle for organising sensor interface information for read/write operations
 */
typedef struct {
  void* interface_h;      // Communication interface handle (SPI_HandleTypeDef)
  GPIO_TypeDef* cs_port;  // Chip select port
  uint16_t cs_pin;        // Chip select pin
} Sensor_HandleTypeDef;

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
int sun_ready = 0;

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

/** Sensor Notes ---------------------------------------------------------------
 * LSM6DSO IMU
 * - Uses SPI1
 * - CS = PB4
 * - Automatically increments through registers for multiple consecutive
 *   read/write operations if register CTRL3_C[2] = 1 (set to 1 by default)
 *
 * IIS2MDC MAG
 * - Uses SPI2
 * - CS = PB2
 */

/**
 * Uses the SPI interface defined in `sensor_h->handle` to write `len` bytes
 * from the data buffer `bufp` starting from the sensor register `reg`.
 *
 * Inputs:
 * `sensor_h`:  pointer to the sensor handle (`Sensor_HandleTypeDef`)
 * `reg`:       initial write register address byte. MSB is internally set to 0
 * `bufp`:      pointer to the data buffer
 * `len`:       number of bytes to write
 *
 * Output:      write operation status (0 = success)
 */
int32_t SensorWrite(void* handle, uint8_t reg, const uint8_t* bufp,
                    uint16_t len) {
  Sensor_HandleTypeDef* sensor_h = (Sensor_HandleTypeDef*)handle;
  GPIO_TypeDef* cs_port = sensor_h->cs_port;
  uint16_t cs_pin = sensor_h->cs_pin;
  HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);  // Start SPI with CS = 0

  HAL_StatusTypeDef status = HAL_ERROR;
  SPI_HandleTypeDef* spi_h = sensor_h->interface_h;
  reg &= 0x7F;  // Bit-mask to set MSB = 0 for write operation
  status = HAL_SPI_Transmit(spi_h, &reg, 1, SPI_TIMEOUT);  // Send write address

  if (status == HAL_OK) {
    status = HAL_SPI_Transmit(spi_h, bufp, len, SPI_TIMEOUT);  // Perform write
  }

  HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);  // Stop SPI with CS = 1
  return status;
}

/**
 * Uses the SPI interface defined in `sensor_h->handle` to read `len` bytes
 * starting from the the sensor register `reg` into the data buffer `bufp`.
 *
 * Inputs:
 * `sensor_h`:  pointer to the sensor handle (`Sensor_HandleTypeDef`)
 * `reg`:       initial read register address byte. MSB internally set to 1
 * `bufp`:      pointer to the data buffer
 * `len`:       number of bytes to read
 *
 * Output:      read operation status (0 = success)
 */
int32_t SensorRead(void* handle, uint8_t reg, uint8_t* bufp, uint16_t len) {
  Sensor_HandleTypeDef* sensor_h = (Sensor_HandleTypeDef*)handle;
  GPIO_TypeDef* cs_port = sensor_h->cs_port;
  uint16_t cs_pin = sensor_h->cs_pin;
  HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);  // Start SPI with CS = 0

  HAL_StatusTypeDef status = HAL_ERROR;
  SPI_HandleTypeDef* spi_h = sensor_h->interface_h;
  reg |= 0x80;  // Bit-mask to set MSB = 1 for read operation
  status = HAL_SPI_Transmit(spi_h, &reg, 1, SPI_TIMEOUT);  // Send read address

  if (status == HAL_OK) {
    status = HAL_SPI_Receive(spi_h, bufp, len, SPI_TIMEOUT);  // Perform write
  }

  HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);  // Stop SPI with CS = 1
  return status;
}

/*
 * User defined ADC interrupt handler for callbacks
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) { sun_ready = 1; }

// initiate variables for sun sensors
int maxIntensity[6];  // Maximum sensor values
int minIntensity[6];  // Minimum sensor values
void resetCalibration() {
  for (int i = 0; i <= 5; i++) {
    maxIntensity[i] = 0U;
    minIntensity[i] = 0xFFFFU;  // 16-bit max of ADC range
  }
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick.
   */
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

  // Initialise LSM6DSO inertial measurement unit handle
  Sensor_HandleTypeDef IMU_h;
  IMU_h.interface_h = &hspi1;
  IMU_h.cs_port = IMU_CS_GPIO_Port;
  IMU_h.cs_pin = IMU_CS_Pin;

  stmdev_ctx_t IMU_ctx;
  IMU_ctx.write_reg = SensorWrite;
  IMU_ctx.read_reg = SensorRead;
  IMU_ctx.handle = &IMU_h;

  // Initialise IIS2MDC magnetometer handle
  Sensor_HandleTypeDef MAG_h;
  MAG_h.interface_h = &hspi2;
  MAG_h.cs_port = MAG_CS_GPIO_Port;
  MAG_h.cs_pin = MAG_CS_Pin;

  stmdev_ctx_t MAG_ctx;
  MAG_ctx.write_reg = SensorWrite;
  MAG_ctx.read_reg = SensorRead;
  MAG_ctx.handle = &MAG_h;

  // Configure LSM6DSO settings
  lsm6dso_i3c_disable_set(&IMU_ctx, LSM6DSO_I3C_DISABLE);
  lsm6dso_spi_mode_set(&IMU_ctx, LSM6DSO_SPI_3_WIRE);
  lsm6dso_auto_increment_set(&IMU_ctx, PROPERTY_ENABLE);
  lsm6dso_block_data_update_set(&IMU_ctx, PROPERTY_ENABLE);
  lsm6dso_xl_data_rate_set(&IMU_ctx, LSM6DSO_XL_ODR_833Hz);
  lsm6dso_xl_full_scale_set(&IMU_ctx, LSM6DSO_2g);
  lsm6dso_gy_data_rate_set(&IMU_ctx, LSM6DSO_GY_ODR_833Hz);
  lsm6dso_gy_full_scale_set(&IMU_ctx, LSM6DSO_250dps);

  // Configure IIS2MDC settings
  iis2mdc_block_data_update_set(&MAG_ctx, PROPERTY_ENABLE);
  iis2mdc_data_rate_set(&MAG_ctx, IIS2MDC_ODR_100Hz);
  iis2mdc_offset_temp_comp_set(&MAG_ctx, PROPERTY_ENABLE);
  iis2mdc_operating_mode_set(&MAG_ctx, IIS2MDC_CONTINUOUS_MODE);

  // Basic device ID verification over UART
  uint8_t whoAmI;
  char id_msg[64];

  lsm6dso_device_id_get(&IMU_ctx, &whoAmI);
  sprintf(id_msg, "LSM6DSO ID: expected %d, read %d\n", LSM6DSO_ID, whoAmI);
  HAL_UART_Transmit(&huart1, (uint8_t*)id_msg, strlen(id_msg), 100);

  iis2mdc_device_id_get(&MAG_ctx, &whoAmI);
  sprintf(id_msg, "IIS2MDC ID: expected %d, read %d\n", IIS2MDC_ID, whoAmI);
  HAL_UART_Transmit(&huart1, (uint8_t*)id_msg, strlen(id_msg), 100);

  // Enable ADC1 using DMA with interrupts
  uint16_t sun[6];  // sun[6] = {-Z, +Z, +X, +Y, -X, -Y}
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)sun, 6);

  // Initialising PWM timers for magnetorquer H-bridges
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

  // Setting PWM timers to 50% duty cycle for testing
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 128);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 128);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 128);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 128);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 128);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 128);

  /* Variables */
  int16_t raw_accel[3];
  int16_t raw_mag[3];
  float accel_mss[3];
  float G_X_roll = 0.0f;
  float G_Y_pitch = 0.0f;
  float G_Z_yaw = 0.0f;
  char gyro_data_str[64];
  char accel_data_str[64];
  char heading_data_str[64];
  char mag_data[64];
  char sun_data_str[64];
  char OFFSET[64];
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t prev_tick = HAL_GetTick();
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    // Sun sensor data reporting
    if (sun_ready == 1) {
      sun_ready = 0;
      snprintf(sun_data_str, sizeof(sun_data_str), "SUN,%u,%u,%u,%u,%u,%u\n",
               sun[0], sun[1], sun[2], sun[3], sun[4], sun[5]);
      HAL_UART_Transmit(&huart1, (uint8_t*)sun_data_str, strlen(sun_data_str),
                        HAL_MAX_DELAY);
    }

    // IMU acceleration data reporting
    lsm6dso_acceleration_raw_get(&IMU_ctx, raw_accel);
    accel_mss[0] = lsm6dso_from_fs2_to_mg(raw_accel[0]);
    accel_mss[1] = lsm6dso_from_fs2_to_mg(raw_accel[1]);
    accel_mss[2] = lsm6dso_from_fs2_to_mg(raw_accel[2]);

    sprintf(accel_data_str, "m/s^2: X = %.2f, Y = %.2f, Z = %.2f\n",
            accel_mss[1], -accel_mss[0], accel_mss[2]);
    HAL_UART_Transmit(&huart1, (uint8_t*)accel_data_str, strlen(accel_data_str),
                      HAL_MAX_DELAY);

    // IMU gyroscope data reporting
    lsm6dso_angular_rate_raw_get(&lsm6dso_ctx, raw_gyro);
    gyro_intermediate[0] = lsm6dso_from_fs250_to_mdps(raw_gyro[0]);
    gyro_intermediate[1] = -lsm6dso_from_fs250_to_mdps(raw_gyro[1]);
    gyro_intermediate[2] = lsm6dso_from_fs250_to_mdps(raw_gyro[2]);

    sprintf(gyro_data_str, "mdps: X = %.2f, Y = %.2f, Z = % .2f\n",
            gyro_intermediate[0], gyro_intermediate[1], gyro_intermediate[2]);
    HAL_UART_Transmit(&huart1, (uint8_t*)gyro_data_str, strlen(gyro_data_str),
                      HAL_MAX_DELAY);

    // Calculating time since last gyroscope read, `dt`
    uint32_t curr_tick = HAL_GetTick();
    float dt = (curr_tick - prev_tick) / 1000.0f;
    prev_tick = curr_tick;

    // Heading data deriving and reporting
    G_X_roll += gyro_intermediate[0] * dt;
    G_Y_pitch += gyro_intermediate[1] * dt;
    G_Z_yaw += gyro_intermediate[2] * dt;

    sprintf(heading_data_str, "roll=%.2f pitch=%.2f yaw=%.2f\n", G_X_roll,
            G_Y_pitch, G_Z_yaw);
    HAL_UART_Transmit(&huart1, (uint8_t*)heading_data_str,
                      strlen(heading_data_str), HAL_MAX_DELAY);

    // Magnetometer data reporting
    iis2mdc_magnetic_raw_get(&iis2mdc_ctx, raw_mag);
    sprintf(mag_data, "MAG DATA  X: %i Y: %i Z: %i \r\n", raw_mag[0],
            raw_mag[1], raw_mag[2]);
    HAL_UART_Transmit(&huart1, (uint8_t*)mag_data, strlen(mag_data),
                      HAL_MAX_DELAY);
    HAL_Delay(10);
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
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

  while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {
  }

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
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 |
                                RCC_CLOCKTYPE_D3PCLK1 | RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
    Error_Handler();
  }
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {
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
  if (HAL_ADC_Init(&hadc1) != HAL_OK) {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
   */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK) {
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
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
    Error_Handler();
  }

  /** Configure Regular Channel
   */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
    Error_Handler();
  }

  /** Configure Regular Channel
   */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
    Error_Handler();
  }

  /** Configure Regular Channel
   */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
    Error_Handler();
  }

  /** Configure Regular Channel
   */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
    Error_Handler();
  }

  /** Configure Regular Channel
   */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
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
static void MX_SPI1_Init(void) {
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
  hspi1.Init.TxCRCInitializationPattern =
      SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.RxCRCInitializationPattern =
      SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK) {
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
static void MX_SPI2_Init(void) {
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
  hspi2.Init.TxCRCInitializationPattern =
      SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi2.Init.RxCRCInitializationPattern =
      SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi2.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi2.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi2.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi2.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi2.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK) {
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
static void MX_TIM2_Init(void) {
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
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK) {
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
static void MX_TIM3_Init(void) {
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
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) {
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
static void MX_USART1_UART_Init(void) {
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
  if (HAL_UART_Init(&huart1) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) !=
      HAL_OK) {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) !=
      HAL_OK) {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */
}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {
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
static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MAG_CS_GPIO_Port, MAG_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : MAG_CS_Pin IMU_CS_Pin */
  GPIO_InitStruct.Pin = MAG_CS_Pin | IMU_CS_Pin;
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

void MPU_Config(void) {
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
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state
   */
  __disable_irq();
  while (1) {
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
void assert_failed(uint8_t* file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n",
     file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
