/*
 * Stepper Motor Test
 * This code helps test the stepper motor driver with the STM32F446RE
 *  Created on: May 17, 2025
 *      Author: Arwar
 */

#include "main.h"
#include <stdio.h>
#include <stdbool.h>

/* Redirect printf to UART */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif


/* External variable declarations */
I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart2;
IWDG_HandleTypeDef hiwdg;
TIM_HandleTypeDef htim3;

/* Motor pin definitions */
#define MOTOR_STEP_PORT      GPIOA
#define MOTOR_STEP_PIN       GPIO_PIN_8
#define MOTOR_DIR_PORT       GPIOB
#define MOTOR_DIR_PIN        GPIO_PIN_0
#define MOTOR_ENG_PORT       GPIOB
#define MOTOR_ENG_PIN        GPIO_PIN_1
#define LED_PORT             GPIOA
#define LED_PIN              GPIO_PIN_5

/* Motor parameters */
#define MIN_SPEED            20.0f      // Minimum speed (steps/sec)
#define MAX_SPEED            1000.0f    // Maximum speed (steps/sec)
#define MAX_ACCEL            2000.0f    // Maximum acceleration (steps/sec²)

/* System state */
typedef struct {
  bool motor_enabled;
  float motor_speed;        // steps/sec
  float target_speed;       // steps/sec after ramping
  bool direction;           // true = forward, false = reverse
} MotorState;

static MotorState motor = {
  .motor_enabled = false,
  .motor_speed = 0.0f,
  .target_speed = 0.0f,
  .direction = true
};

static uint32_t last_time;
static uint32_t last_print;

/* Function prototypes */
void motor_enable(bool enable);
void set_motor_speed(float speed);
void set_motor_direction(bool direction);
void set_target_speed(float speed);
void update_motor(float dt);
void process_command(uint8_t cmd);


/* Private function prototypes */
static void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);

PUTCHAR_PROTOTYPE {
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}


/**
  * @brief Enable or disable motor
  * @param enable Enable state
  */
void motor_enable(bool enable) {
  if (motor.motor_enabled != enable) {
    motor.motor_enabled = enable;

    /* ENG pin is active LOW */
    HAL_GPIO_WritePin(MOTOR_ENG_PORT, MOTOR_ENG_PIN, enable ? GPIO_PIN_RESET : GPIO_PIN_SET);

    /* Print status change */
    printf("Motor %s\r\n", enable ? "ENABLED" : "DISABLED");

    if (!enable) {
      /* Reset speed variables */
      motor.motor_speed = 0.0f;
      motor.target_speed = 0.0f;
    }
  }
}

/**
  * @brief Set motor direction
  * @param direction Direction (true = forward, false = reverse)
  */
void set_motor_direction(bool direction) {
  if (motor.direction != direction) {
    motor.direction = direction;
    HAL_GPIO_WritePin(MOTOR_DIR_PORT, MOTOR_DIR_PIN, direction ? GPIO_PIN_SET : GPIO_PIN_RESET);
    printf("Direction: %s\r\n", direction ? "FORWARD" : "REVERSE");
  }
}

/**
  * @brief Set motor speed immediately
  * @param speed Speed in steps/second (always positive)
  */
void set_motor_speed(float speed) {
  /* Validate speed */
  float abs_speed = (speed < 0) ? -speed : speed;

  /* If speed is below threshold, disable motor */
  if (abs_speed < MIN_SPEED) {
    if (motor.motor_enabled) {
      motor_enable(false);
    }
    return;
  }

  /* Enable motor if needed */
  if (!motor.motor_enabled) {
    motor_enable(true);
  }

  /* Calculate timer period for desired speed */
  uint32_t timer_clock = HAL_RCC_GetPCLK1Freq();

  /* Check if APB1 prescaler is not 1, double the frequency */
  if ((RCC->CFGR & RCC_CFGR_PPRE1) != RCC_CFGR_PPRE1_DIV1) {
    timer_clock *= 2;
  }

  /* Calculate timer frequency */
  uint32_t timer_freq = timer_clock / (htim3.Init.Prescaler + 1);

  /* Calculate auto-reload value */
  uint32_t arr = (timer_freq / (2 * abs_speed)) - 1;

  /* Clamp ARR value to valid range */
  arr = (arr > 0xFFFF) ? 0xFFFF : ((arr < 1) ? 1 : arr);

  /* Update timer auto-reload register */
  __HAL_TIM_SET_AUTORELOAD(&htim3, arr);

  /* Update motor state */
  motor.motor_speed = abs_speed;
}

/**
  * @brief Set target motor speed (with ramping)
  * @param speed Target speed (positive for forward, negative for reverse)
  */
void set_target_speed(float speed) {
  /* Set direction based on sign */
  set_motor_direction(speed >= 0);

  /* Take absolute value for speed calculations */
  float abs_speed = (speed < 0) ? -speed : speed;

  /* Clamp to valid range */
  if (abs_speed > MAX_SPEED) {
    abs_speed = MAX_SPEED;
  }

  /* Update target speed */
  motor.target_speed = abs_speed;

  printf("Target speed: %.1f steps/sec\r\n", speed);
}

/**
  * @brief Update motor speed with ramping
  * @param dt Time delta in seconds
  */
void update_motor(float dt) {
  /* Calculate maximum speed change based on acceleration */
  float speed_change = MAX_ACCEL * dt;

  /* Apply ramping */
  if (motor.motor_speed < motor.target_speed) {
    motor.motor_speed = motor.motor_speed + speed_change;
    if (motor.motor_speed > motor.target_speed) {
      motor.motor_speed = motor.target_speed;
    }
  } else if (motor.motor_speed > motor.target_speed) {
    motor.motor_speed = motor.motor_speed - speed_change;
    if (motor.motor_speed < motor.target_speed) {
      motor.motor_speed = motor.target_speed;
    }
  }

  /* Apply new speed */
  set_motor_speed(motor.motor_speed);
}

/**
  * @brief Process user commands
  * @param cmd Command character
  */
void process_command(uint8_t cmd) {
  switch (cmd) {
    case '0': /* Stop motor */
      set_target_speed(0);
      break;

    case '1': /* Speed level 1 (Forward) */
      set_target_speed(100);
      break;

    case '2': /* Speed level 2 (Forward) */
      set_target_speed(250);
      break;

    case '3': /* Speed level 3 (Forward) */
      set_target_speed(500);
      break;

    case '4': /* Speed level 4 (Forward) */
      set_target_speed(1000);
      break;

    case '5': /* Speed level 1 (Reverse) */
      set_target_speed(-100);
      break;

    case '6': /* Speed level 2 (Reverse) */
      set_target_speed(-250);
      break;

    case '7': /* Speed level 3 (Reverse) */
      set_target_speed(-500);
      break;

    case '8': /* Speed level 4 (Reverse) */
      set_target_speed(-1000);
      break;

    case 'e':
    case 'E': /* Toggle motor enable */
      motor_enable(!motor.motor_enabled);
      break;

    default:
      break;
  }
}

/**
  * @brief Timer period elapsed callback
  * @param htim Timer handle
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM3 && motor.motor_enabled) {
    HAL_GPIO_TogglePin(MOTOR_STEP_PORT, MOTOR_STEP_PIN);
  }
}

/**
  * @brief Main motor test function
  */
void motor_test(void) {
  /* Initialize motor control pins to safe state */
  motor_enable(false);
  HAL_GPIO_WritePin(MOTOR_DIR_PORT, MOTOR_DIR_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MOTOR_STEP_PORT, MOTOR_STEP_PIN, GPIO_PIN_RESET);

  /* Initialize timing variables */
  last_time = HAL_GetTick();
  last_print = last_time;

  /* Start timer for stepper pulses */
  HAL_TIM_Base_Start_IT(&htim3);

  /* Welcome message and menu */
  printf("\r\n\r\n");
  printf("===========================================\r\n");
  printf("       Stepper Motor Controller Test       \r\n");
  printf("===========================================\r\n\r\n");

  printf("Commands:\r\n");
  printf("  '0' - Stop motor\r\n");
  printf("  '1' - Speed 100 steps/sec (Forward)\r\n");
  printf("  '2' - Speed 250 steps/sec (Forward)\r\n");
  printf("  '3' - Speed 500 steps/sec (Forward)\r\n");
  printf("  '4' - Speed 1000 steps/sec (Forward)\r\n");
  printf("  '5' - Speed 100 steps/sec (Reverse)\r\n");
  printf("  '6' - Speed 250 steps/sec (Reverse)\r\n");
  printf("  '7' - Speed 500 steps/sec (Reverse)\r\n");
  printf("  '8' - Speed 1000 steps/sec (Reverse)\r\n");
  printf("  'e' - Toggle motor enable\r\n\r\n");

  /* Main loop */
  while (1) {
    /* Calculate time delta */
    uint32_t now = HAL_GetTick();
    float dt = (now - last_time) * 0.001f;
    if (dt < 0.005f) dt = 0.005f; /* Minimum dt to prevent division by zero */
    last_time = now;

    /* Update motor speed with ramping */
    update_motor(dt);

    /* Print status periodically */
    if (now - last_print >= 1000) {
      printf("Status: %s, Speed: %.1f steps/sec, Direction: %s\r\n",
             motor.motor_enabled ? "ENABLED" : "DISABLED",
             motor.motor_speed,
             motor.direction ? "FORWARD" : "REVERSE");
      last_print = now;
    }

    /* Check for user input */
    uint8_t key = 0;
    if (HAL_UART_Receive(&huart2, &key, 1, 0) == HAL_OK) {
      process_command(key);
    }

    /* Blink LED to indicate program is running */
    HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
    HAL_Delay(50);
  }
}

/* Add to main.c to use this function */
int main(void) {
  /* MCU Configuration */
  HAL_Init();
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();


  /* Call the test function */
  motor_test();

  /* Never reached */
  while (1) {}
}


/* USER CODE END 4 */

/**
  * @brief System Clock Configuration
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}



static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
}



/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 839;
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
  if (HAL_TIM_Base_Start_IT(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
}


static void MX_GPIO_Init(void)
 {
   GPIO_InitTypeDef GPIO_InitStruct = {0};

   /* GPIO Ports Clock Enable */
   __HAL_RCC_GPIOA_CLK_ENABLE();
   __HAL_RCC_GPIOB_CLK_ENABLE();

   /* Configure GPIO pin Output Level */
   HAL_GPIO_WritePin(GPIOA, LED_PIN|MOTOR_STEP_PIN, GPIO_PIN_RESET);
   HAL_GPIO_WritePin(GPIOB, MOTOR_DIR_PIN|MOTOR_ENG_PIN, GPIO_PIN_RESET);

   /* Configure GPIO pins : PA5 (LED) PA8 (MOTOR_STEP) */
   GPIO_InitStruct.Pin = LED_PIN|MOTOR_STEP_PIN;
   GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
   GPIO_InitStruct.Pull = GPIO_NOPULL;
   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
   HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

   /* Configure GPIO pins : PB0 (MOTOR_DIR) PB1 (MOTOR_ENG) */
   GPIO_InitStruct.Pin = MOTOR_DIR_PIN|MOTOR_ENG_PIN;
   GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
   GPIO_InitStruct.Pull = GPIO_NOPULL;
   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
   HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
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

  // Blink LED rapidly to indicate error
  while (1)
  {
    HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
    HAL_Delay(100);
  }
  /* USER CODE END Error_Handler_Debug */
}
