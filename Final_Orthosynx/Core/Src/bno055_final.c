/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : bno055_final.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include "bno055.h"
/* Private includes ----------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
 SENSOR_OK = 0,
 SENSOR_COMM_ERROR,
 SENSOR_NOT_CALIBRATED
} SensorStatus;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//------------------------------------------------------------------------------
// System constants (BNO055)
//------------------------------------------------------------------------------
#define CALIBRATION_TIMEOUT_MS    30000                         // ms before giving up on full gyro calibration
#define IMUPLUS_MODE              BNO055_OPERATION_MODE_IMUPLUS // Only accel + gyro, no magnetometer fusion



// Timing & rate constants
#define BNO055_BOOT_DELAY_MS      650   // Wait after reset before commands OK
#define BNO055_UPDATE_INTERVAL_MS 10    // 100 Hz read rate

// BNO055 I²C addresses (w/ ADDR pin low/high)
#define THIGH_SENSOR_ADDR  BNO055_I2C_ADDR1
#define SHANK_SENSOR_ADDR  BNO055_I2C_ADDR2

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

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
//------------------------------------------------------------------------------
// BNO055 state definitions
//------------------------------------------------------------------------------

typedef struct {
  struct bno055_t   dev;           // BNO055 driver handle (bus_read, bus_write, etc.)
  uint8_t           calib[4];      // Calibration status: [sys, gyro, accel, mag]
  SensorStatus      status;        // SENSOR_OK / SENSOR_COMM_ERROR / SENSOR_NOT_CALIBRATED
  float             last_r;        // Last roll angle (for any filtering or delta calc)
  float             last_p;        // Last pitch
  float             last_h;        // Last heading
} BNOState;

// Two sensor instances
static BNOState thigh = {
  .dev       = {0},
  .calib     = {0, 0, 0, 0},
  .status    = SENSOR_NOT_CALIBRATED,
  .last_r    = 0.0f,
  .last_p    = 0.0f,
  .last_h    = 0.0f
};

static BNOState shank = {
  .dev       = {0},
  .calib     = {0, 0, 0, 0},
  .status    = SENSOR_NOT_CALIBRATED,
  .last_r    = 0.0f,
  .last_p    = 0.0f,
  .last_h    = 0.0f
};

// Shared timing/state variables
static uint32_t last_update   = 0;  // Timestamp of last orientation read
static uint8_t  sensors_found = 0;  // How many BNO devices were detected
float knee_angle = 0.0f;
float knee_velocity = 0.0f;
float prev_knee_angle = 0.0f;
u8 i2c_errors = 0;
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
void I2C_Scan_And_Assign(void);
void BNO055_Init_Dual(void);
void Read_Dual_Calibration(void);
void Read_Dual_Orientation(void);
void Reset_BNO055(u8 dev_addr);
void Debug_I2C_Status(void);
void Test_Basic_Communication(void);
s8 read_euler_angles(u8 dev_addr, struct bno055_euler_float_t *euler_float);
s8 read_calibration_status(u8 dev_addr, u8 *calib_data);
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
// Delay function for BNO055
void BNO055_Delay(u32 period) {
   HAL_Delay(period);
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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_TIM3_Init();
  MX_WWDG_Init();
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
  // Configure I2C noise filtering
  HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE);
  HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 3);
  HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE);
  HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 3);
  // Configure printf buffering
  setbuf(stdout, NULL); // Disable buffering for immediate output
  // Initialize Debug LED to OFF
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
  // Print startup message
  printf("Prosthetic Knee Controller Starting...\r\n");
  printf("Clock: %lu MHz\r\n", HAL_RCC_GetSysClockFreq()/1000000);

  /////////start bno055 code ///
  printf("\r\n=== Prosthetic Knee Controller (IMUPLUS Mode) ===\r\n");
  // Flash LED to indicate startup
  for(int i = 0; i < 5; i++) {
      HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
      HAL_Delay(200);
  }
  printf("System Clock: %lu Hz\r\n", HAL_RCC_GetSysClockFreq());
  Debug_I2C_Status();
  // Wait for sensors to power up
  printf("Waiting for sensors to boot...\r\n");
  BNO055_Delay(BNO055_BOOT_DELAY_MS);  // Minimum 650ms per datasheet
  // Scan I2C bus
  I2C_Scan_And_Assign();
  // Initialize sensors
  printf("Initializing sensors...\r\n");
  BNO055_Init_Dual();
  // Calibration phase
  printf("Calibrating sensors - keep stationary!\r\n");
  u32 calib_start = HAL_GetTick();
  u8 calib_printed = 0;
  while(1) {
      Read_Dual_Calibration();
      // Print calibration status every second
      if(HAL_GetTick() - calib_start > calib_printed * 1000) {
          printf("Calib: Thigh(G:%d A:%d) Shank(G:%d A:%d)\r\n",
                 thigh.calib[1], thigh.calib[2],
                 shank.calib[1], shank.calib[2]);
          calib_printed++;
      }
      // Exit when gyros are calibrated
      if(thigh.calib[1] >= 2 && shank.calib[1] >= 2) {
          printf("Calibration complete!\r\n");
          break;
      }
      if(HAL_GetTick() - calib_start > CALIBRATION_TIMEOUT_MS) {
          printf("Calibration timeout! Continuing...\r\n");
          break;
      }
// or change to  Delay input = BNO055_BOOT_DELAY_MS
      BNO055_Delay(100);
  }
  last_update = HAL_GetTick();


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
      // 100Hz update rate (10ms interval)
      if(HAL_GetTick() - last_update >= BNO055_UPDATE_INTERVAL_MS) {
      	Read_Dual_Orientation();
          last_update = HAL_GetTick();
      }
      BNO055_Delay(1); // Prevent watchdog issues
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
  htim3.Init.Prescaler = 0;
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
void Debug_I2C_Status(void) {
   printf("I2C2 Configuration:\r\n");
   printf("- Clock Speed: %lu Hz\r\n", hi2c2.Init.ClockSpeed);
   printf("- State: %d\r\n", hi2c2.State);
   if(__HAL_RCC_I2C2_IS_CLK_ENABLED()) {
       printf("- I2C2 Clock: ENABLED\r\n");
   } else {
       printf("- I2C2 Clock: DISABLED\r\n");
   }
}
void Test_Basic_Communication(void) {
   printf("Testing I2C communication...\r\n");
   u8 addresses[] = {THIGH_SENSOR_ADDR, SHANK_SENSOR_ADDR};
   for(int i = 0; i < 2; i++) {
       u8 chip_id = 0;
       HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&hi2c2, addresses[i] << 1,
                                                   BNO055_CHIP_ID_ADDR,
                                                   I2C_MEMADD_SIZE_8BIT,
                                                   &chip_id, 1, 100);
       printf("Addr 0x%02X: Status=%d, ChipID=0x%02X\r\n",
              addresses[i], status, chip_id);
   }
}
void I2C_Scan_And_Assign(void) {
   printf("Scanning I2C bus...\r\n");
   u8 found_addr[2] = {0};
   sensors_found = 0;
   for (u8 addr = 0x08; addr <= 0x78; addr++) {
       if (HAL_I2C_IsDeviceReady(&hi2c2, addr << 1, 3, 100) == HAL_OK) {
           printf("Device found at 0x%02X\r\n", addr);
           if(sensors_found < 2) {
               found_addr[sensors_found] = addr;
           }
           sensors_found++;
       }
   }
   printf("Total devices: %d\r\n", sensors_found);
   if (sensors_found < 2) {
       printf("Using default addresses\r\n");
       found_addr[0] = THIGH_SENSOR_ADDR;
       found_addr[1] = SHANK_SENSOR_ADDR;
   }
thigh.dev.dev_addr = found_addr[0];
   shank.dev.dev_addr = found_addr[1];
   printf("Thigh: 0x%02X, Shank: 0x%02X\r\n",
         thigh.dev.dev_addr, shank.dev.dev_addr);
}
// I2C read function
s8 BNO055_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt) {
   HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&hi2c2, dev_addr << 1,
                                               reg_addr, I2C_MEMADD_SIZE_8BIT,
                                               reg_data, cnt, 100);
   return (status == HAL_OK) ? BNO055_SUCCESS : BNO055_ERROR;
}
// I2C write function
s8 BNO055_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt) {
   HAL_StatusTypeDef status = HAL_I2C_Mem_Write(&hi2c2, dev_addr << 1,
                                                reg_addr, I2C_MEMADD_SIZE_8BIT,
                                                reg_data, cnt, 100);
   return (status == HAL_OK) ? BNO055_SUCCESS : BNO055_ERROR;
}
void BNO055_Init_Dual(void) {
   // Initialize thigh sensor
   printf("Initializing thigh sensor...\r\n");
  thigh.dev.bus_read = BNO055_I2C_bus_read;
  thigh.dev.bus_write = BNO055_I2C_bus_write;
   thigh.dev.delay_msec = BNO055_Delay;
   s8 result = bno055_init(&thigh.dev);
   if(result == BNO055_SUCCESS) {
       // Set IMUPLUS mode (Accelerometer + Gyroscope only)
       bno055_set_operation_mode(IMUPLUS_MODE);
       printf("Thigh: IMUPLUS mode set\r\n");
   }
   // Initialize shank sensor
   printf("Initializing shank sensor...\r\n");
   shank.dev.bus_read = BNO055_I2C_bus_read;
   shank.dev.bus_write = BNO055_I2C_bus_write;
   shank.dev.delay_msec = BNO055_Delay;
   result = bno055_init(&shank.dev);
   printf("Shank init: %s\r\n", result == BNO055_SUCCESS ? "OK" : "FAIL");
   if(result == BNO055_SUCCESS) {
       // Set IMUPLUS mode (Accelerometer + Gyroscope only)
       bno055_set_operation_mode(IMUPLUS_MODE);
       printf("Shank: IMUPLUS mode set\r\n");
   }
}
// Read calibration status directly from registers
s8 read_calibration_status(u8 dev_addr, u8 *calib_data) {
   u8 calib_stat = 0;
   s8 result = BNO055_I2C_bus_read(dev_addr, BNO055_CALIB_STAT_ADDR, &calib_stat, 1);
   if(result == BNO055_SUCCESS) {
       calib_data[0] = (calib_stat >> 6) & 0x03; // System status
       calib_data[1] = (calib_stat >> 4) & 0x03; // Gyroscope status
       calib_data[2] = (calib_stat >> 2) & 0x03; // Accelerometer status
       calib_data[3] = calib_stat & 0x03;        // Magnetometer status
   }
   return result;
}
void Read_Dual_Calibration(void) {
   read_calibration_status(thigh.dev.dev_addr, thigh.calib);
   read_calibration_status(shank.dev.dev_addr, shank.calib);
}
// Read Euler angles directly from registers
s8 read_euler_angles(u8 dev_addr, struct bno055_euler_float_t *euler_float) {
   u8 euler_data[6];
   s8 result = BNO055_I2C_bus_read(dev_addr, BNO055_EULER_H_LSB_ADDR, euler_data, 6);
   if(result == BNO055_SUCCESS) {
       // Combine bytes (little-endian)
       int16_t h = (int16_t)((euler_data[1] << 8) | euler_data[0]);
       int16_t r = (int16_t)((euler_data[3] << 8) | euler_data[2]);
       int16_t p = (int16_t)((euler_data[5] << 8) | euler_data[4]);
       // Convert to degrees (LSB = 1/16 degree)
       euler_float->h = (float)h / 16.0f;
       euler_float->r = (float)r / 16.0f;
       euler_float->p = (float)p / 16.0f;
   }
   return result;
}
// Helper function to normalize angle to [-180, 180] range
float normalize_angle(float angle) {
   while (angle > 180.0f) angle -= 360.0f;
   while (angle < -180.0f) angle += 360.0f;
   return angle;
}
// Modified Orientation Reading with angle differences
// Modified Orientation Reading with knee angle calculation
void Read_Dual_Orientation(void) {
   struct bno055_euler_float_t euler1, euler2;
   u32 current_time = HAL_GetTick();
   float dt = (current_time - last_update) / 1000.0f; // Convert to seconds
   // Read thigh sensor directly
   if(read_euler_angles(thigh.dev.dev_addr, &euler1) != BNO055_SUCCESS) {
       printf("Error reading thigh sensor\r\n");
       return;
   }
   // Read shank sensor directly
   if(read_euler_angles(shank.dev.dev_addr, &euler2) != BNO055_SUCCESS) {
       printf("Error reading shank sensor\r\n");
       return;
   }
   // Calculate angle differences between sensors
   float roll_diff = normalize_angle(euler2.r - euler1.r);
   float pitch_diff = normalize_angle(euler2.p - euler1.p);
   float yaw_diff = normalize_angle(euler2.h - euler1.h);
   // Store previous knee angle for velocity calculation
   prev_knee_angle = knee_angle;
   // Calculate knee angle (typically pitch difference for knee flexion/extension)
   // Positive value = knee flexion, Negative value = knee extension
   knee_angle = -pitch_diff;  // Critical inversion for backward mounting
   // Calculate knee angular velocity (degrees per second)
   if(dt > 0) {
       knee_velocity = (knee_angle - prev_knee_angle) / dt;
   }
   // Print individual sensor readings
   printf("Thigh: Roll=%.1f Pitch=%.1f Yaw=%.1f | ",
          euler1.r, euler1.p, euler1.h);
   printf("Shank: Roll=%.1f Pitch=%.1f Yaw=%.1f\r\n",
          euler2.r, euler2.p, euler2.h);
   // Print angle differences and knee measurements
   printf("Diff: Roll=%.1f Pitch=%.1f Yaw=%.1f | ",
          roll_diff, pitch_diff, yaw_diff);
   printf("Knee: Angle=%.1f° Velocity=%.1f°/s\r\n",
          knee_angle, knee_velocity);
}
void Reset_BNO055(u8 dev_addr) {
   printf("Resetting sensor 0x%02X...\r\n", dev_addr);
   // Write reset command to SYS_TRIGGER register (0x3F)
   u8 reset_cmd = 0x20;
   HAL_I2C_Mem_Write(&hi2c2, dev_addr << 1,
                     BNO055_SYS_TRIGGER_ADDR, I2C_MEMADD_SIZE_8BIT,
                     &reset_cmd, 1, 100);
   // Wait 650ms as per datasheet requirement
   BNO055_Delay(BNO055_BOOT_DELAY_MS);
}

// User Button Interrupt Handler
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
if(GPIO_Pin == GPIO_PIN_13) // User Button
{
// Toggle Debug LED on button press
HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
printf("User Button Pressed\r\n");
}
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
  while (1)
  {
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
