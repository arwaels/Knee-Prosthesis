	/* USER CODE BEGIN Header */
	/**
	  ******************************************************************************
	  * @file           : main.c
	  * @brief          : Main program body - Prosthetic Knee Controller (IMUPLUS Mode)
	  ******************************************************************************
	  * @attention
	  *
	  * Copyright (c) 2025 STMicroelectronics.
	  * All rights reserved.
	  *
	  * Licensed under terms in LICENSE; if none, provided AS-IS.
	  ******************************************************************************
	  */
	/* USER CODE END Header */

	/* Includes ------------------------------------------------------------------*/
	#include "main.h"

	/* Private includes ----------------------------------------------------------*/
	/* USER CODE BEGIN Includes */
	#include <stdio.h>
	#include <math.h>
	#include <stdbool.h>
    #include <string.h>
	#include "bno055.h"
	/* USER CODE END Includes */

	/* Private typedef -----------------------------------------------------------*/
	/* USER CODE BEGIN PTD */
	typedef enum {
		SENSOR_OK = 0,
		SENSOR_COMM_ERROR,
		SENSOR_NOT_CALIBRATED,
		SENSOR_INIT_ERROR
	} SensorStatus;
	typedef enum {
	    GAIT_PHASE_UNKNOWN = 0,
	    GAIT_PHASE_HEEL_STRIKE,
	    GAIT_PHASE_TOE_OFF,
	    GAIT_PHASE_PEAK_SWING_FLEXION,
	    GAIT_PHASE_TERMINAL_SWING,  // CORRECTED SPELLING
	    GAIT_PHASE_SWING,
	    GAIT_PHASE_STALE_DATA,
	    GAIT_PHASE_STABLE_RECAL
	} GaitPhase;
	typedef struct {
	    GaitPhase current_phase;
	    GaitPhase previous_phase;
	    uint32_t phase_entry_time;
	    bool needs_recalibration;
	    bool is_locked;
	} GaitState;

	typedef struct {
		bool initialized;
		bool calibrated;
		uint32_t last_read_time;
		struct bno055_euler_float_t euler;
		uint8_t calib_status[4];  // System, Gyroscope, Accelerometer only
	} IMU_Data_t;
	/* USER CODE END PTD */

	/* USER CODE BEGIN PD */
	#define CALIBRATION_TIMEOUT_MS 30000
	#define IMUPLUS_MODE            BNO055_OPERATION_MODE_IMUPLUS
	#define IMU_READ_INTERVAL_MS    10
	#define MAX_I2C_RETRIES         3
	#define I2C_TIMEOUT_MS          500
	#define GYRO_CALIB_THRESHOLD    2
	#define ACCEL_CALIB_THRESHOLD   2

	// Gait phase detection thresholds
	#define HS_ENTER_ANGLE_MAX   5.0f
	#define HS_EXIT_ANGLE_MIN    8.0f
	#define HS_ENTER_VEL_MAX     100.0f
	#define HS_EXIT_VEL_MIN      120.0f
	#define TO_ENTER_ANGLE_MIN   15.0f
	#define TO_ENTER_VEL_MIN     100.0f
	#define TO_EXIT_ANGLE_MAX    12.0f
	#define TO_EXIT_VEL_MAX      80.0f
	#define PSF_ANGLE_MIN        60.0f
	#define PSF_VEL_MAX          50.0f
	#define PSF_EXIT_ANGLE_MIN   55.0f
	#define PSF_EXIT_VEL_MIN     60.0f
	#define TS_ANGLE_MAX         10.0f
	#define TS_VEL_MAX          -30.0f
	#define TS_EXIT_ANGLE_MIN    13.0f
	#define TS_EXIT_VEL_MIN     -15.0f
	#define RECAL_VEL_MAX        20.0f
	#define RECAL_ANGLE_MAX      5.0f

	// BNO055 I²C addresses (w/ ADDR pin low/high)
	#define THIGH_SENSOR_ADDR  BNO055_I2C_ADDR1
	#define SHANK_SENSOR_ADDR  BNO055_I2C_ADDR2

	/* USER CODE END PD */

	/* Private macro -------------------------------------------------------------*/
	/* USER CODE BEGIN PM */
	#define REFRESH_WATCHDOG() HAL_WWDG_Refresh(&hwwdg)
	/* USER CODE END PM */

	/* Private variables ---------------------------------------------------------*/
	I2C_HandleTypeDef hi2c1;
	I2C_HandleTypeDef hi2c2;
	DMA_HandleTypeDef hdma_i2c2_rx;
	DMA_HandleTypeDef hdma_i2c2_tx;

	TIM_HandleTypeDef htim3;

	UART_HandleTypeDef huart2;
	DMA_HandleTypeDef hdma_usart2_tx;

	WWDG_HandleTypeDef hwwdg;

	/* USER CODE BEGIN PV */
	struct bno055_t thigh_sensor, shank_sensor;
	IMU_Data_t thigh_imu, shank_imu;
	SensorStatus system_status = SENSOR_NOT_CALIBRATED;
	float knee_angle = 0.0f, knee_velocity = 0.0f, prev_knee_angle = 0.0f;
	uint32_t last_orientation_update = 0;
	volatile bool timer_flag = false;
	static float filtered_velocity = 0.0f;
	const float velocity_filter_alpha = 0.3f;
	uint8_t sensors_found = 0;
	bool calibration_complete = false;
	GaitState gait_state = {GAIT_PHASE_UNKNOWN, GAIT_PHASE_UNKNOWN, 0, false, false};
	const uint32_t PHASE_MIN_DURATION_MS = 50; //minimum 50ms per phase
	/* USER CODE END PV */


	/* Private function prototypes -----------------------------------------------*/
	void SystemClock_Config(void);
	static void MX_GPIO_Init(void);
	static void MX_DMA_Init(void);
	static void MX_USART2_UART_Init(void);
	static void MX_I2C1_Init(void);
	static void MX_I2C2_Init(void);
	static void MX_TIM3_Init(void);
	static void MX_WWDG_Init(void);

	/* USER CODE BEGIN PFP */
	GaitPhase detect_gait_phase(float angle, float velocity, uint32_t current_time);
	// Timer and state management
	void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
	bool validate_sensor_data(struct bno055_euler_float_t *euler);

	// BNO055 interface functions
	s8 BNO055_I2C_bus_read(u8 dev, u8 reg, u8 *data, u8 len);
	s8 BNO055_I2C_bus_write(u8 dev, u8 reg, u8 *data, u8 len);
	void BNO055_Delay(u32 ms);

	// Initialization and configuration
	void I2C_Scan_And_Assign(void);
	SensorStatus BNO055_Init_Sensor(struct bno055_t *sensor, const char* name);
	void BNO055_Init_Dual(void);

	// Calibration functions
	void Read_Dual_Calibration(void);
	s8 read_calibration_status(u8 dev, u8 *calib_array);
	bool is_sensor_calibrated(uint8_t *calib_status);
	void print_calibration_status(void);

	// Orientation and data processing
	void Read_Dual_Orientation(void);
	s8 read_euler_angles_safe(u8 dev, struct bno055_euler_float_t *euler);
	float normalize_angle(float angle);
	void calculate_knee_kinematics(void);

	// Utility functions
	void handle_sensor_error(const char* sensor_name, const char* error_msg);
	void system_health_check(void);
	/* USER CODE END PFP */

	/* Private user code ---------------------------------------------------------*/
	/* USER CODE BEGIN 0 */

	/* Redirect printf to UART */
	#ifdef __GNUC__
	#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
	#else
	#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
	#endif
	PUTCHAR_PROTOTYPE {
	  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
	  return ch;
	}

	void BNO055_Delay(u32 ms)
	{
		HAL_Delay(ms);
		//REFRESH_WATCHDOG();
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
		  /* MCU Configuration--------------------------------------------------------*/
		  HAL_Init();
		  SystemClock_Config();
		  MX_GPIO_Init();
		  MX_USART2_UART_Init();
		  MX_DMA_Init();
		  MX_I2C1_Init();
		  MX_I2C2_Init();
		  // Configure I2C noise filtering
			  HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE);
			  HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 3);
			  HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE);
			  HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 3);
		  MX_TIM3_Init();
		  // Override TIM3 prescaler for 100Hz operation (84MHz clock)
		  // Configure TIM3 for 100Hz operation (84MHz clock)
		  __HAL_TIM_SET_PRESCALER(&htim3, 8399);  // 84MHz/8400 = 10kHz
		  __HAL_TIM_SET_AUTORELOAD(&htim3, 99);   // 100Hz (10kHz/100)

		 // MX_WWDG_Init();

	  /* USER CODE BEGIN 2 */
	  // Configure interrupt priorities
	  HAL_NVIC_SetPriority(I2C1_EV_IRQn, 1, 0); // I2C1 Event
	  HAL_NVIC_SetPriority(I2C1_ER_IRQn, 0, 0); // I2C1 Error (highest priority)
	  HAL_NVIC_SetPriority(I2C2_EV_IRQn, 1, 0); // I2C2 Event
	  HAL_NVIC_SetPriority(I2C2_ER_IRQn, 0, 0); // I2C2 Error (highest priority)
	  HAL_NVIC_SetPriority(USART2_IRQn, 1, 0); // USART2 global interrupt
	  HAL_NVIC_SetPriority(TIM3_IRQn, 2, 0); // Timer3 (lower priority)
	  HAL_NVIC_SetPriority(WWDG_IRQn, 0, 0); // Watchdog (highest priority)
	  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 2, 0); // User Button
	  // Configure DMA interrupt priorities
	  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 0, 0); // I2C2_RX
	  HAL_NVIC_SetPriority(DMA1_Stream7_IRQn, 0, 0); // I2C2_TX
	  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0); // USART2_TX

	  // Configure printf buffering
	  setbuf(stdout, NULL); // Disable buffering for immediate output
	  // Initialize Debug LED to OFF
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
	  // Print startup message
	  printf("\r\n=== Prosthetic Knee Controller (BNO055 IMUPLUS) ===\r\n");
	  printf("System Clock: %lu Hz\r\n", HAL_RCC_GetSysClockFreq());
	  printf("Build: %s %s\r\n", __DATE__, __TIME__);

	  // Initialize IMU data structures
	  memset(&thigh_imu, 0, sizeof(IMU_Data_t));
	  memset(&shank_imu, 0, sizeof(IMU_Data_t));

	  // LED startup sequence
	  printf("Starting initialization sequence...\r\n");
	  for(int i = 0; i < 3; i++)
	  {
		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		HAL_Delay(200);
	//	REFRESH_WATCHDOG();
	  }
	  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

	  // Wait for IMU power-up (BNO055 requires ~650ms)
	  printf("Waiting for IMU power-up...\r\n");
	  BNO055_Delay(800);

	  // Scan I2C bus and assign addresses
	  I2C_Scan_And_Assign();

	  // Initialize both sensors
	  BNO055_Init_Dual();

	  // Start calibration process
	  printf("\r\nStarting calibration process...\r\n");
	  printf("Keep the prosthetic stationary during calibration!\r\n");

	  uint32_t calibration_start = HAL_GetTick();
	  uint32_t last_status_print = 0;

	  while (!calibration_complete && (HAL_GetTick() - calibration_start < CALIBRATION_TIMEOUT_MS))
	  {
		Read_Dual_Calibration();

		// raw register dump for debugging
		uint8_t thigh_raw, shank_raw;
		BNO055_I2C_bus_read(thigh_sensor.dev_addr, BNO055_CALIB_STAT_ADDR, &thigh_raw, 1);
		BNO055_I2C_bus_read(shank_sensor.dev_addr, BNO055_CALIB_STAT_ADDR, &shank_raw, 1);
		printf("Raw Calib Regs: Thigh=0x%02X, Shank=0x%02X\r\n", thigh_raw, shank_raw);

		// Print status every second
		if (HAL_GetTick() - last_status_print >= 1000)
		{
		  print_calibration_status();
		  last_status_print = HAL_GetTick();
		}

		// Check if both sensors are calibrated
		if (is_sensor_calibrated(thigh_imu.calib_status) &&
			is_sensor_calibrated(shank_imu.calib_status))
		{
		  calibration_complete = true;
		  // Switch to CONFIG mode to ensure calibration is applied
		  uint8_t config_mode = BNO055_OPERATION_MODE_CONFIG;
		  BNO055_I2C_bus_write(thigh_sensor.dev_addr, BNO055_OPR_MODE_ADDR, &config_mode, 1);
		  BNO055_I2C_bus_write(shank_sensor.dev_addr, BNO055_OPR_MODE_ADDR, &config_mode, 1);
		  BNO055_Delay(25);
		  // Switch back to IMUPLUS mode
		  uint8_t imuplus_mode = IMUPLUS_MODE;
		  BNO055_I2C_bus_write(thigh_sensor.dev_addr, BNO055_OPR_MODE_ADDR, &imuplus_mode, 1);
		  BNO055_I2C_bus_write(shank_sensor.dev_addr, BNO055_OPR_MODE_ADDR, &imuplus_mode, 1);
		  BNO055_Delay(20);

		  printf("\r\n*** CALIBRATION COMPLETE! ***\r\n");
		  HAL_Delay(100);
		  system_status = SENSOR_OK;

		  break;
		}

		BNO055_Delay(100);
	//	REFRESH_WATCHDOG();
	  }

	  if (!calibration_complete)
	  {
		printf("\r\nWARNING: Calibration timeout! Proceeding with partial calibration.\r\n");
		system_status = SENSOR_NOT_CALIBRATED;
	  }

	  // Start timer for 100Hz updates
	  __HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_UPDATE);  // Clear pending interrupts
	  HAL_TIM_Base_Start_IT(&htim3);
	  HAL_TIM_Base_Start(&htim3);
	  HAL_NVIC_EnableIRQ(TIM3_IRQn);  // Enable TIM3 interrupt
	  last_orientation_update = HAL_GetTick();

	  printf("\r\nStarting orientation tracking at 100Hz...\r\n");
	  printf("Format: Knee_Angle(°) | Angular_Velocity(°/s)\r\n");
	  printf("----------------------------------------\r\n");

	  /* USER CODE END 2 */

	  /* USER CODE BEGIN WHILE */
	  while (1)
	  {
	    // Process sensor data when timer flag is set (100Hz)
		  if (timer_flag) {
		      timer_flag = false;
		      Read_Dual_Orientation();
		      calculate_knee_kinematics();

		      // Gait phase detection
		      gait_state.previous_phase = gait_state.current_phase;
		      gait_state.current_phase = detect_gait_phase(
		          knee_angle,
		          filtered_velocity,
		          HAL_GetTick()
		      );

		      // Handle recalibration request
		      if (gait_state.needs_recalibration) {
		          printf("Initiating IMU recalibration...\r\n");
		          BNO055_Init_Dual();  // Reinitialize sensors
		          Read_Dual_Calibration();
		          gait_state.needs_recalibration = false;
		          printf("Recalibration complete\r\n");
		      }

		      // System health monitoring (every 5s)
		      static uint32_t last_health_check = 0;
		      if (HAL_GetTick() - last_health_check >= 5000) {
		          system_health_check();
		          last_health_check = HAL_GetTick();

		          // Print gait state
		          const char* phase_names[] = {
		              "UNKNOWN", "HEEL_STRIKE", "TOE_OFF", "PEAK_SWING_FLEX",
		              "TERMINAL_SWING", "SWING", "STALE_DATA", "STABLE_RECAL"
		          };
		          printf("Gait Phase: %s | Locked: %s\r\n",
		                 phase_names[gait_state.current_phase],
		                 gait_state.is_locked ? "YES" : "NO");
		      }

		  }


	    //REFRESH_WATCHDOG();
	  }

	  /* USER CODE END WHILE */

	  /* USER CODE BEGIN 3 */
	}

	 /* USER CODE END 3 */
	/**
	  * @brief System Clock Configuration
	  * @retval None
	  */
	void SystemClock_Config(void)
	{
	  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	  /** Configure the main internal regulator output voltage
	  */
	  __HAL_RCC_PWR_CLK_ENABLE();
	  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

	  /** Initializes the RCC Oscillators according to the specified parameters
	  * in the RCC_OscInitTypeDef structure.
	  */
	  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	  RCC_OscInitStruct.PLL.PLLM = 16;
	  RCC_OscInitStruct.PLL.PLLN = 336;
	  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
	  RCC_OscInitStruct.PLL.PLLQ = 2;
	  RCC_OscInitStruct.PLL.PLLR = 2;
	  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	  {
	    Error_Handler();
	  }

	  /** Initializes the CPU, AHB and APB buses clocks
	  */
	  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
	                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
	  {
	    Error_Handler();
	  }
	}

	/**
	  * @brief I2C1 Initialization Function
	  * @param None
	  * @retval None
	  */
	static void MX_I2C1_Init(void)
	{

	  /* USER CODE BEGIN I2C1_Init 0 */

	  /* USER CODE END I2C1_Init 0 */

	  /* USER CODE BEGIN I2C1_Init 1 */

	  /* USER CODE END I2C1_Init 1 */
	  hi2c1.Instance = I2C1;
	  hi2c1.Init.ClockSpeed = 100000;
	  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	  hi2c1.Init.OwnAddress1 = 0;
	  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	  hi2c1.Init.OwnAddress2 = 0;
	  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  /* USER CODE BEGIN I2C1_Init 2 */

	  /* USER CODE END I2C1_Init 2 */

	}

	/**
	  * @brief I2C2 Initialization Function
	  * @param None
	  * @retval None
	  */
	static void MX_I2C2_Init(void)
	{

	  /* USER CODE BEGIN I2C2_Init 0 */

	  /* USER CODE END I2C2_Init 0 */

	  /* USER CODE BEGIN I2C2_Init 1 */

	  /* USER CODE END I2C2_Init 1 */
	  hi2c2.Instance = I2C2;
	  hi2c2.Init.ClockSpeed = 100000;
	  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
	  hi2c2.Init.OwnAddress1 = 0;
	  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	  hi2c2.Init.OwnAddress2 = 0;
	  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  /* USER CODE BEGIN I2C2_Init 2 */

	  /* USER CODE END I2C2_Init 2 */

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

	  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	  TIM_MasterConfigTypeDef sMasterConfig = {0};

	  /* USER CODE BEGIN TIM3_Init 1 */

	  /* USER CODE END TIM3_Init 1 */
	  htim3.Instance = TIM3;
	  htim3.Init.Prescaler = 8399;
	  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	  htim3.Init.Period = 99;
	  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  /* USER CODE BEGIN TIM3_Init 2 */

	  /* USER CODE END TIM3_Init 2 */

	}

	/**
	  * @brief USART2 Initialization Function
	  * @param None
	  * @retval None
	  */
	static void MX_USART2_UART_Init(void)
	{

	  /* USER CODE BEGIN USART2_Init 0 */

	  /* USER CODE END USART2_Init 0 */

	  /* USER CODE BEGIN USART2_Init 1 */

	  /* USER CODE END USART2_Init 1 */
	  huart2.Instance = USART2;
	  huart2.Init.BaudRate = 115200;
	  huart2.Init.WordLength = UART_WORDLENGTH_8B;
	  huart2.Init.StopBits = UART_STOPBITS_1;
	  huart2.Init.Parity = UART_PARITY_NONE;
	  huart2.Init.Mode = UART_MODE_TX_RX;
	  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	  huart2.Init.OverSampling = UART_OVERSAMPLING_8;
	  if (HAL_UART_Init(&huart2) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  /* USER CODE BEGIN USART2_Init 2 */

	  /* USER CODE END USART2_Init 2 */

	}

	/**
	  * @brief WWDG Initialization Function
	  * @param None
	  * @retval None
	  */
	static void MX_WWDG_Init(void)
	{

	  /* USER CODE BEGIN WWDG_Init 0 */

	  /* USER CODE END WWDG_Init 0 */

	  /* USER CODE BEGIN WWDG_Init 1 */

	  /* USER CODE END WWDG_Init 1 */
	  hwwdg.Instance = WWDG;
	  hwwdg.Init.Prescaler = WWDG_PRESCALER_1;
	  hwwdg.Init.Window = 64;
	  hwwdg.Init.Counter = 64;
	  hwwdg.Init.EWIMode = WWDG_EWI_DISABLE;
	  if (HAL_WWDG_Init(&hwwdg) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  /* USER CODE BEGIN WWDG_Init 2 */

	  /* USER CODE END WWDG_Init 2 */

	}

	/**
	  * Enable DMA controller clock
	  */
	static void MX_DMA_Init(void)
	{

	  /* DMA controller clock enable */
	  __HAL_RCC_DMA1_CLK_ENABLE();

	  /* DMA interrupt init */
	  /* DMA1_Stream2_IRQn interrupt configuration */
	  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 0, 0);
	  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
	  /* DMA1_Stream6_IRQn interrupt configuration */
	  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
	  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
	  /* DMA1_Stream7_IRQn interrupt configuration */
	  HAL_NVIC_SetPriority(DMA1_Stream7_IRQn, 0, 0);
	  HAL_NVIC_EnableIRQ(DMA1_Stream7_IRQn);

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
	  __HAL_RCC_GPIOH_CLK_ENABLE();
	  __HAL_RCC_GPIOA_CLK_ENABLE();
	  __HAL_RCC_GPIOB_CLK_ENABLE();

	  /*Configure GPIO pin Output Level */
	  HAL_GPIO_WritePin(GPIOA, LD2_Pin|MOTOR_STEP_Pin, GPIO_PIN_RESET);

	  /*Configure GPIO pin Output Level */
	  HAL_GPIO_WritePin(GPIOB, MOTOR_DIR_Pin|MOTOR_ENG_Pin|DEBUG_LED_Pin, GPIO_PIN_RESET);

	  /*Configure GPIO pin : B1_Pin */
	  GPIO_InitStruct.Pin = B1_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	  /*Configure GPIO pins : LD2_Pin MOTOR_STEP_Pin */
	  GPIO_InitStruct.Pin = LD2_Pin|MOTOR_STEP_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	  /*Configure GPIO pins : MOTOR_DIR_Pin MOTOR_ENG_Pin DEBUG_LED_Pin */
	  GPIO_InitStruct.Pin = MOTOR_DIR_Pin|MOTOR_ENG_Pin|DEBUG_LED_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	  /* USER CODE BEGIN MX_GPIO_Init_2 */

	  /* USER CODE END MX_GPIO_Init_2 */
	}


	/* USER CODE BEGIN 4 */
	// ==== BNO055 I2C INTERFACE FUNCTIONS ====
	s8 bno055_read_reg(u8 dev, u8 reg, u8 *data) {
	    return BNO055_I2C_bus_read(dev, reg, data, 1);
	}



	s8 BNO055_I2C_bus_read(u8 dev, u8 reg, u8 *data, u8 len)
	{

	  for (int retry = 0; retry < MAX_I2C_RETRIES; retry++)
	  {
		HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&hi2c2, dev << 1, reg,
													I2C_MEMADD_SIZE_8BIT, data, len, I2C_TIMEOUT_MS);
		if (status == HAL_OK)
		    {

		      return BNO055_SUCCESS;
		    }

		if (retry < MAX_I2C_RETRIES - 1)
		  HAL_Delay(1); // Short delay before retry
	  }

	  return BNO055_ERROR;
	}



	s8 BNO055_I2C_bus_write(u8 dev, u8 reg, u8 *data, u8 len)
	{
	  for (int retry = 0; retry < MAX_I2C_RETRIES; retry++)
	  {
		HAL_StatusTypeDef status = HAL_I2C_Mem_Write(&hi2c2, dev << 1, reg,
													 I2C_MEMADD_SIZE_8BIT, data, len, I2C_TIMEOUT_MS);
		if (status == HAL_OK)
		    {
		      HAL_Delay(1); // Small delay after write as per BNO055 datasheet
		      return BNO055_SUCCESS;
		    }

		if (retry < MAX_I2C_RETRIES - 1)
		  HAL_Delay(1); // Short delay before retry
	  }
	  return BNO055_ERROR;
	}

	// ==== INITIALIZATION FUNCTIONS ====
	void I2C_Scan_And_Assign(void)
	{
	  sensors_found = 0;  // Reset sensor count before scanning
	  printf("Using fixed I2C addresses...\r\n");

	  // Use fixed addresses
	  thigh_sensor.dev_addr = THIGH_SENSOR_ADDR; // 0x28
	  shank_sensor.dev_addr = SHANK_SENSOR_ADDR; // 0x29

	  // Verify devices are present
	  if (HAL_I2C_IsDeviceReady(&hi2c2, thigh_sensor.dev_addr << 1, 3, 100) == HAL_OK)
	  {
	    printf("✓ Thigh sensor found at 0x%02X\r\n", thigh_sensor.dev_addr);
	    sensors_found++;
	  }
	  else
	  {
	    printf("✗ Thigh sensor not responding at 0x%02X\r\n", thigh_sensor.dev_addr);
	  }

	  if (HAL_I2C_IsDeviceReady(&hi2c2, shank_sensor.dev_addr << 1, 3, 100) == HAL_OK)
	  {
	    printf("✓ Shank sensor found at 0x%02X\r\n", shank_sensor.dev_addr);
	    sensors_found++;
	  }
	  else
	  {
	    printf("✗ Shank sensor not responding at 0x%02X\r\n", shank_sensor.dev_addr);
	  }

	  printf("Found %d/2 sensors\r\n", sensors_found);
	}

	GaitPhase detect_gait_phase(float angle, float velocity, uint32_t current_time) {
	    // Check for stale data (100ms timeout)
	    if (current_time - last_orientation_update > 100) {
	        gait_state.current_phase = GAIT_PHASE_STALE_DATA;
	        return gait_state.current_phase;
	    }

	    // Check if we should remain in current phase (hysteresis and min duration)
	    if ((current_time - gait_state.phase_entry_time) < PHASE_MIN_DURATION_MS) {
	        return gait_state.current_phase;
	    }

	    // Heel Strike Detection
	    if (angle >= 0 && angle <= 5 && fabsf(velocity) <= 100) {
	        if (gait_state.current_phase != GAIT_PHASE_HEEL_STRIKE) {
	            gait_state.previous_phase = gait_state.current_phase;
	            gait_state.current_phase = GAIT_PHASE_HEEL_STRIKE;
	            gait_state.phase_entry_time = current_time;
	            gait_state.is_locked = true;
	            printf("HEEL STRIKE DETECTED - LOCKING SCREW\r\n");
	        }
	        return gait_state.current_phase;
	    }

	    // Heel Strike Exit Hysteresis
	    if (gait_state.current_phase == GAIT_PHASE_HEEL_STRIKE &&
	        (angle > 8 || fabsf(velocity) > 120)) {
	        gait_state.previous_phase = gait_state.current_phase;
	        gait_state.current_phase = GAIT_PHASE_SWING;
	        gait_state.phase_entry_time = current_time;
	    }

	    // Toe-Off Detection
	    if (angle > 15 && velocity > 100) {
	        if (gait_state.current_phase != GAIT_PHASE_TOE_OFF) {
	            gait_state.previous_phase = gait_state.current_phase;
	            gait_state.current_phase = GAIT_PHASE_TOE_OFF;
	            gait_state.phase_entry_time = current_time;
	            gait_state.is_locked = false;
	            printf("TOE-OFF DETECTED - UNLOCKING SCREW\r\n");
	        }
	        return gait_state.current_phase;
	    }

	    // Toe-Off Exit Hysteresis
	    if (gait_state.current_phase == GAIT_PHASE_TOE_OFF &&
	        (angle < 12 || velocity < 80)) {
	        gait_state.previous_phase = gait_state.current_phase;
	        gait_state.current_phase = GAIT_PHASE_SWING;
	        gait_state.phase_entry_time = current_time;
	    }

	    // Peak Swing Flexion Detection
	    if (angle >= 60 && fabsf(velocity) < 50) {
	        if (gait_state.current_phase != GAIT_PHASE_PEAK_SWING_FLEXION) {
	            gait_state.previous_phase = gait_state.current_phase;
	            gait_state.current_phase = GAIT_PHASE_PEAK_SWING_FLEXION;
	            gait_state.phase_entry_time = current_time;
	            printf("PEAK SWING FLEXION DETECTED\r\n");
	        }
	        return gait_state.current_phase;
	    }

	    // Peak Swing Exit Hysteresis
	    if (gait_state.current_phase == GAIT_PHASE_PEAK_SWING_FLEXION &&
	        (angle < 55 || fabsf(velocity) > 60)) {
	        gait_state.previous_phase = gait_state.current_phase;
	        gait_state.current_phase = GAIT_PHASE_SWING;
	        gait_state.phase_entry_time = current_time;
	    }

	    // Terminal Swing Detection
	    if (angle <= 10 && velocity < -30) {
	        if (gait_state.current_phase != GAIT_PHASE_TERMINAL_SWING) {
	            gait_state.previous_phase = gait_state.current_phase;
	            gait_state.current_phase = GAIT_PHASE_TERMINAL_SWING;
	            gait_state.phase_entry_time = current_time;
	            printf("TERMINAL SWING DETECTED - GRADUAL TIGHTENING\r\n");
	        }
	        return gait_state.current_phase;
	    }

	    // Terminal Swing Exit Hysteresis
	    if (gait_state.current_phase == GAIT_PHASE_TERMINAL_SWING &&
	        (angle > 13 || velocity > -15)) {
	        gait_state.previous_phase = gait_state.current_phase;
	        gait_state.current_phase = GAIT_PHASE_SWING;
	        gait_state.phase_entry_time = current_time;
	    }

	    // Stable Recalibration Check
	    if (fabsf(velocity) <= 20 && angle <= 5) {
	        if (gait_state.current_phase != GAIT_PHASE_STABLE_RECAL) {
	            gait_state.previous_phase = gait_state.current_phase;
	            gait_state.current_phase = GAIT_PHASE_STABLE_RECAL;
	            gait_state.phase_entry_time = current_time;
	            gait_state.needs_recalibration = true;
	            printf("STABLE POSITION - RECALIBRATION RECOMMENDED\r\n");
	        }
	        return gait_state.current_phase;
	    }

	    // Default to swing phase
	    if (gait_state.current_phase != GAIT_PHASE_SWING) {
	        gait_state.previous_phase = gait_state.current_phase;
	        gait_state.current_phase = GAIT_PHASE_SWING;
	        gait_state.phase_entry_time = current_time;
	    }

	    return gait_state.current_phase;
	}

	SensorStatus BNO055_Init_Sensor(struct bno055_t *sensor, const char* name) {
	    // Setup sensor structure
	    sensor->bus_read = BNO055_I2C_bus_read;
	    sensor->bus_write = BNO055_I2C_bus_write;
	    sensor->delay_msec = BNO055_Delay;

	    // Initialize sensor
	    if (bno055_init(sensor) != BNO055_SUCCESS) {
	        printf("ERROR: %s sensor init failed!\r\n", name);
	        return SENSOR_INIT_ERROR;
	    }

	    // Set to IMUPLUS mode (IMU + Magnetometer)
	    if (bno055_set_operation_mode(IMUPLUS_MODE) != BNO055_SUCCESS) {
	        printf("ERROR: %s failed to set IMUPLUS mode!\r\n", name);
	        return SENSOR_INIT_ERROR;
	    }

	    // Wait for mode change
	    BNO055_Delay(100);

	    // Perform self-test
	    u8 selftest_result;
	    if (BNO055_I2C_bus_read(sensor->dev_addr,
	                            BNO055_SELFTEST_RESULT_ADDR,
	                            &selftest_result, 1) != BNO055_SUCCESS) {
	        printf("ERROR: %s self-test read failed!\r\n", name);
	        return SENSOR_INIT_ERROR;
	    }

	    // Check self-test results (1 = passed, 0 = failed)
	    u8 mcu_pass = (selftest_result >> 0) & 0x01;
	    u8 gyro_pass = (selftest_result >> 1) & 0x01;
	    u8 accel_pass = (selftest_result >> 2) & 0x01;
	    u8 mag_pass = (selftest_result >> 3) & 0x01;

	    if (!(mcu_pass && gyro_pass && accel_pass)) {
	        printf("ERROR: %s self-test failed! MCU:%d GYRO:%d ACC:%d MAG:%d\r\n",
	               name, mcu_pass, gyro_pass, accel_pass, mag_pass);

	        // Lock knee and activate red LED
	        gait_state.is_locked = true;
	        HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
	        printf("CRITICAL ERROR - SYSTEM HALTED!\r\n");
	        while(1);
	    }

	    printf("%s sensor initialized in IMUPLUS mode\r\n", name);
	    return SENSOR_OK;
	}
	void BNO055_Init_Dual(void)
	{
	  printf("\r\nInitializing BNO055 sensors...\r\n");
	  // Exit early if insufficient sensors detected
	  if (sensors_found < 2)
	  {
	    printf("ERROR: Only %d/2 sensors detected! Cannot proceed with initialization.\r\n", sensors_found);
	    system_status = SENSOR_INIT_ERROR;
	    return;
	  }
	  // Initialize thigh sensor
	  SensorStatus thigh_init = BNO055_Init_Sensor(&thigh_sensor, "Thigh");
	  if (thigh_init == SENSOR_OK)
	  {
		thigh_imu.initialized = true;
		printf("✓ Thigh sensor ready\r\n");
	  }
	  else
	  {
		handle_sensor_error("Thigh", "initialization failed");
	  }

	  // Initialize shank sensor
	  SensorStatus shank_init = BNO055_Init_Sensor(&shank_sensor, "Shank");
	  if (shank_init == SENSOR_OK)
	  {
		shank_imu.initialized = true;
		printf("✓ Shank sensor ready\r\n");
	  }
	  else
	  {
		handle_sensor_error("Shank", "initialization failed");
	  }

	  if (thigh_imu.initialized && shank_imu.initialized)
	  {
		printf("Both sensors initialized successfully!\r\n");
	  }
	  else
	  {
		printf("WARNING: One or more sensors failed to initialize!\r\n");
	  }
	}

	// ==== CALIBRATION FUNCTIONS ====
	s8 read_calibration_status(u8 dev, u8 *calib_array)
	{
	  uint8_t calib_status_reg;
	  if (BNO055_I2C_bus_read(dev, BNO055_CALIB_STAT_ADDR, &calib_status_reg, 1) != BNO055_SUCCESS)
	  {
	    return BNO055_ERROR;
	  }

	  // Extract calibration statuses (ignore magnetometer)
	  calib_array[0] = (calib_status_reg >> 6) & 0x03; // System
	  calib_array[1] = (calib_status_reg >> 4) & 0x03; // Gyroscope
	  calib_array[2] = (calib_status_reg >> 2) & 0x03; // Accelerometer
	  calib_array[3] = (calib_status_reg >> 0) & 0x03; // Magnetometer

	  // Note: Magnetometer status ignored for prosthetic application

	  return BNO055_SUCCESS;
	}

	bool is_sensor_calibrated(uint8_t *calib_status)
	{
	  // For prosthetic knee application, prioritize gyro and accel calibration
	  return (calib_status[1] >= GYRO_CALIB_THRESHOLD &&
			  calib_status[2] >= ACCEL_CALIB_THRESHOLD);
	}

	void Read_Dual_Calibration(void)
	{
	  if (thigh_imu.initialized)
	  {
		if (read_calibration_status(thigh_sensor.dev_addr, thigh_imu.calib_status) == BNO055_SUCCESS)
		{
		  thigh_imu.calibrated = is_sensor_calibrated(thigh_imu.calib_status);
		}
	  }

	  if (shank_imu.initialized)
	  {
		if (read_calibration_status(shank_sensor.dev_addr, shank_imu.calib_status) == BNO055_SUCCESS)
		{
		  shank_imu.calibrated = is_sensor_calibrated(shank_imu.calib_status);
		}
	  }
	}

	void print_calibration_status(void)
	{
	    printf("Calibration Status (Sys|Gyr|Acc|Mag):\r\n");
	    printf("  Thigh - %d|%d|%d|%d %s\r\n",
	           thigh_imu.calib_status[0], thigh_imu.calib_status[1],
	           thigh_imu.calib_status[2], thigh_imu.calib_status[3],
	           thigh_imu.calibrated ? "[OK]" : "[PENDING]");

	    printf("  Shank - %d|%d|%d|%d %s\r\n",
	           shank_imu.calib_status[0], shank_imu.calib_status[1],
	           shank_imu.calib_status[2], shank_imu.calib_status[3],
	           shank_imu.calibrated ? "[OK]" : "[PENDING]");
	}

	// ==== ORIENTATION AND DATA PROCESSING ====
	s8 read_euler_angles_safe(u8 dev, struct bno055_euler_float_t *euler)
	{
	  uint8_t euler_data[6];

	  if (BNO055_I2C_bus_read(dev, BNO055_EULER_H_LSB_ADDR, euler_data, 6) != BNO055_SUCCESS)
	  {
		return BNO055_ERROR;
	  }

	  // Convert raw data to euler angles (16-bit signed, LSB first)
	  int16_t heading = (int16_t)((euler_data[1] << 8) | euler_data[0]);
	  int16_t roll    = (int16_t)((euler_data[3] << 8) | euler_data[2]);
	  int16_t pitch   = (int16_t)((euler_data[5] << 8) | euler_data[4]);

	  // Scale to degrees (1 degree = 16 LSB)
	  euler->h = heading / 16.0f;
	  euler->r = roll / 16.0f;
	  euler->p = pitch / 16.0f;

	  return BNO055_SUCCESS;
	}

	float normalize_angle(float angle) {
	    angle = fmodf(angle, 360.0f);
	    if (angle > 180.0f) angle -= 360.0f;
	    if (angle < -180.0f) angle += 360.0f;
	    return angle;
	}

	void Read_Dual_Orientation(void)
	{
	    static uint16_t error_count = 0;  // Error counter local to this function
	    uint32_t current_time = HAL_GetTick();
	    bool read_success = true;

	    // Read thigh IMU
	    if (thigh_imu.initialized) {
	        struct bno055_euler_float_t temp_euler;
	        if (read_euler_angles_safe(thigh_sensor.dev_addr, &temp_euler) == BNO055_SUCCESS) {
	            if (validate_sensor_data(&temp_euler)) {
	                thigh_imu.euler = temp_euler;
	                thigh_imu.last_read_time = current_time;
	            } else {
	                handle_sensor_error("Thigh", "invalid data range");
	                read_success = false;
	            }
	        } else {
	            handle_sensor_error("Thigh", "orientation read failed");
	            read_success = false;
	        }
	    }

	    BNO055_Delay(5);

	    // Read shank IMU
	    if (shank_imu.initialized && read_success) {  // Only proceed if thigh read was successful
	        struct bno055_euler_float_t temp_euler;
	        if (read_euler_angles_safe(shank_sensor.dev_addr, &temp_euler) == BNO055_SUCCESS) {
	            if (validate_sensor_data(&temp_euler)) {
	                shank_imu.euler = temp_euler;
	                shank_imu.last_read_time = current_time;
	            } else {
	                handle_sensor_error("Shank", "invalid data range");
	                read_success = false;
	            }
	        } else {
	            handle_sensor_error("Shank", "orientation read failed");
	            read_success = false;
	        }
	    }

	    // Update global timestamp only if both reads were successful
	    if (read_success) {
	        last_orientation_update = HAL_GetTick();
	        error_count = 0;  // Reset error counter on full success
	    } else {
	        error_count++;  // Increment error counter on any failure

	        // Emergency shutdown after 20 consecutive errors
	        if (error_count > 20) {
	            printf("CRITICAL: Too many consecutive errors - LOCKING KNEE!\r\n");
	            gait_state.is_locked = true;
	            HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
	            // System continues to run but maintains safety lock
	        }
	    }
	}

	void calculate_knee_kinematics(void)
	{
	    static uint32_t prev_update_time = 0;
	    static bool first_sample = true;
	    static uint32_t last_print_time = 0;
	    static float prev_raw_knee_angle = 0.0f;  // Store previous RAW angle

	    // Only calculate if both sensors have recent data
	    if (!thigh_imu.initialized || !shank_imu.initialized)
	        return;

	    uint32_t current_time = HAL_GetTick();

	    // Check for sensor timeouts
	    if ((current_time - thigh_imu.last_read_time > 100) ||
	        (current_time - shank_imu.last_read_time > 100))
	    {
	        return; // Skip calculation if data is stale
	    }

	    // Skip first sample to establish timing baseline
	    if (first_sample)
	    {
	        prev_update_time = current_time;  // Use current time, not orientation update
	        first_sample = false;

	        // Calculate initial raw angle
	        float pitch_diff = shank_imu.euler.p - thigh_imu.euler.p;
	        prev_raw_knee_angle = normalize_angle(pitch_diff);

	        return;
	    }

	    // Calculate time delta in SECONDS
	    float dt = (current_time - prev_update_time) / 1000.0f;
	    prev_update_time = current_time;  // Update immediately after calculation

	    // Prevent division by zero and handle very small time steps
	    if (dt <= 0.001f) return;

	    // Calculate RAW knee angle (signed)
	    float pitch_diff = shank_imu.euler.p - thigh_imu.euler.p;
	    float raw_knee_angle = normalize_angle(pitch_diff);

	    // For display only - clamp to 0-160° (preserve raw for calculations)
	    float display_angle = raw_knee_angle;
	    if (display_angle < 0) display_angle = 0;
	    if (display_angle > 160.0f) display_angle = 160.0f;

	    // Calculate angular velocity using RAW angles
	    float raw_velocity = (raw_knee_angle - prev_raw_knee_angle) / dt;

	    // Clamp to physiological limits for knee motion
	    raw_velocity = fmaxf(fminf(raw_velocity, 300.0f), -300.0f);

	    // Apply low-pass filter
	    filtered_velocity = velocity_filter_alpha * raw_velocity +
	                       (1.0f - velocity_filter_alpha) * filtered_velocity;

	    // Update previous values
	    prev_raw_knee_angle = raw_knee_angle;
	    knee_angle = raw_knee_angle;  // Store SIGNED angle for gait detection

	    // Print results with clamped display angle
	    if (current_time - last_print_time >= 100) {
	        printf("Knee: %6.1f° | Vel: %+7.1f°/s | T_p:%6.1f S_p:%6.1f\r\n",
	               display_angle, filtered_velocity,
	               thigh_imu.euler.p, shank_imu.euler.p);
	        last_print_time = current_time;
	    }
	}

	// ==== UTILITY FUNCTIONS ====
	void handle_sensor_error(const char* sensor_name, const char* error_msg) {
	    static uint32_t last_error_time = 0;
	    static uint16_t error_count = 0;
	    uint32_t current_time = HAL_GetTick();

	    // Rate limit errors
	    if (current_time - last_error_time > 1000) {
	        printf("ERROR: %s - %s (Count: %d)\r\n", sensor_name, error_msg, ++error_count);
	        last_error_time = current_time;

	        // Emergency shutdown after 20 consecutive errors
	        if (error_count > 20) {
	            printf("CRITICAL: Too many errors - LOCKING KNEE!\r\n");
	            gait_state.is_locked = true;
	            HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
	            // Don't halt - maintain safety but keep trying
	        }
	    }

	    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
	    system_status = SENSOR_COMM_ERROR;
	}



	void system_health_check(void)
	{
	  uint32_t current_time = HAL_GetTick();
	  bool system_healthy = true;

	  // Check if sensors are responding
	  if (thigh_imu.initialized && (current_time - thigh_imu.last_read_time > 1000))
	  {
		handle_sensor_error("Thigh", "communication timeout");
		system_healthy = false;
	  }

	  if (shank_imu.initialized && (current_time - shank_imu.last_read_time > 1000))
	  {
		handle_sensor_error("Shank", "communication timeout");
		system_healthy = false;
	  }

	  // Check calibration drift
	  if (system_status == SENSOR_OK && calibration_complete)
	  {
	    Read_Dual_Calibration();
	    if (!is_sensor_calibrated(thigh_imu.calib_status) ||
	        !is_sensor_calibrated(shank_imu.calib_status))
	    {
	      printf("WARNING: Sensor recalibration needed!\r\n");
	      system_status = SENSOR_NOT_CALIBRATED;
	      system_healthy = false;
	    }
	  }
	  // Update system status LED
	  if (system_healthy && system_status == SENSOR_OK)
	  {
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET); // Turn off debug LED
	  }
	}

	// ==== DATA VALIDATION ====
	bool validate_sensor_data(struct bno055_euler_float_t *euler) {
	    // Allow ±200° to accommodate brief glitches
	    return !isnan(euler->h) && !isnan(euler->p) && !isnan(euler->r) &&
	           fabsf(euler->h) <= 200.0f &&
	           fabsf(euler->r) <= 200.0f &&
	           fabsf(euler->p) <= 200.0f;
	}

	// ==== INTERRUPT HANDLERS ====
	void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
	{
	  if (GPIO_Pin == GPIO_PIN_13) // User button pressed
	  {
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14); // Toggle debug LED
		printf("Button pressed - Debug LED toggled\r\n");
	  }
	}

	void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
	{
	  if (htim->Instance == TIM3)
	  {
	    timer_flag = true;  // Trigger 100Hz processing
	  }
	}

	void HAL_WWDG_EarlyWakeupCallback(WWDG_HandleTypeDef *hwwdg)
	{
	  // Watchdog early wakeup - refresh the watchdog
	  HAL_WWDG_Refresh(hwwdg);
	}

	/* USER CODE END 4 */

	/**
	  * @brief  This function is executed in case of error occurrence.
	  * @retval None
	  */
	void Error_Handler(void)
	{
	  /* USER CODE BEGIN Error_Handler_Debug */
	  /* User can add his own implementation to report the HAL error return state */
	  __disable_irq();
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET); // Turn on debug LED
	  printf("FATAL ERROR: System halted!\r\n");
	  while (1)
	  {
		// Blink debug LED rapidly to indicate error
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
		HAL_Delay(100);
	  }
	  /* USER CODE END Error_Handler_Debug */
	}

	#ifdef  USE_FULL_ASSERT
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
	  printf("Assert failed: %s line %lu\r\n", file, line);
	  /* USER CODE END 6 */
	}
	#endif /* USE_FULL_ASSERT */
