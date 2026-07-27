/* USER CODE BEGIN Header */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "st7789.h"
#include "LSM6DSO.h"
#include "seesaw_driver.h"
#include "stm32l4xx_hal_def.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TIMEOUT 100
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;

UART_HandleTypeDef huart2;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */
volatile uint8_t accel_interrupt_flag; 
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_RTC_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

/* Retarget printf to USART2 */
PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
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
  MX_I2C1_Init();
  MX_RTC_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  ST7789_Init();							
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_LSE
                              |RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
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
  hi2c1.Init.Timing = 0x00B07CB4;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */
  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */
  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */
  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */
  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */
  /* USER CODE END RTC_Init 2 */

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
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */
  /* USER CODE END SPI1_Init 2 */

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
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */
  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

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
  HAL_GPIO_WritePin(ST7789_RST_GPIO_Port, ST7789_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, ST7789_DC_Pin|LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : ST7789_RST_Pin */
  GPIO_InitStruct.Pin = ST7789_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ST7789_RST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ST7789_DC_Pin LD3_Pin */
  GPIO_InitStruct.Pin = ST7789_DC_Pin|LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : ACCEL_INT1_Pin */
  GPIO_InitStruct.Pin = ACCEL_INT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(ACCEL_INT1_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == ACCEL_INT1_Pin) {          // only runs once pending bit confirmed (fixes #2)
    accel_interrupt_flag = 1;
  }
}

void write_to_accel(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t *data, uint16_t size) {
    HAL_StatusTypeDef status = HAL_I2C_Mem_Write(hi2c, ACCEL_ADDRESS << 1, reg, I2C_MEMADD_SIZE_8BIT, data, size, TIMEOUT);
    if (status != HAL_OK) {
        printf("Error writing to accelerometer register 0x%02X: %d\n", reg, status);
    }
}

void read_from_accel(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t *data, uint16_t size) {
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(hi2c, ACCEL_ADDRESS << 1, reg, I2C_MEMADD_SIZE_8BIT, data, size, TIMEOUT);
    if (status != HAL_OK) {
        printf("Error reading from accelerometer register 0x%02X: %d\n", reg, status);
    }
}

void read_from_encoder(I2C_HandleTypeDef *hi2c, uint16_t reg, uint8_t *data, uint16_t size) {
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(hi2c, SEESAW_I2C_ADDRESS << 1, reg, I2C_MEMADD_SIZE_16BIT, data, size, TIMEOUT);
    if (status != HAL_OK) {
        printf("Error reading from encoder register 0x%04X: %d\n", reg, status);
    }
}

/* USER CODE END 4 */

/* USER CODE BEGIN Tasks */

// void EncoderInterruptTask(void *argument) {
//   while (1) {
//     if (accel_interrupt_flag) {
//       accel_interrupt_flag = 0;
//       printf("Accelerometer interrupt detected!\n");
//     }
//     osDelay(10);
//   }
// }

typedef enum {
  STATE_WATCH,
  STATE_CHRONO,
  STATE_GAME
} SystemState;

void SystemStateTask(void *argument) {

  SystemState current_state = STATE_WATCH;

  for (;;) {

    switch (current_state) {
      case STATE_WATCH:
        // Handle watch state
        break;
      case STATE_CHRONO:
        // Handle chronograph state
        break;
      case STATE_GAME:
        // Handle game state
        break;
    }

    osDelay(100);
  }
}

void ChronoTask(void *argument) {

  for (;;) {
    // Handle chronograph functionality
    osDelay(100);
  }
}

void AccelerometerReadTask(void *argument) {

  for (;;) {
    uint8_t accel_data[6];
    uint8_t num_bytes = 6;

    int accel_g_scale = 2;
    int accel_bits_precision = 16;
    float accel_scale_mg = accel_g_scale * 2 * 1000 / (float)(1 << accel_bits_precision);

    read_from_accel(&hi2c1, OUTX_L_A, accel_data, num_bytes);

    int16_t x = ((int16_t)((uint16_t)accel_data[1] << 8) | (uint16_t)accel_data[0]) * accel_scale_mg;
    int16_t y = ((int16_t)((uint16_t)accel_data[3] << 8) | (uint16_t)accel_data[2]) * accel_scale_mg;
    int16_t z = ((int16_t)((uint16_t)accel_data[5] << 8) | (uint16_t)accel_data[4]) * accel_scale_mg;
    
    osDelay(100);
  }
}

void DisplayWriteTask(void *argument) {

  for (;;) {

    snprintf(encoder_value_buffer, sizeof(encoder_value_buffer), "Value: %9ld", encoder_value);
    snprintf(encoder_delta_buffer, sizeof(encoder_delta_buffer), "Delta: %9ld", encoder_delta);

    snprintf(accel_buffer[0], sizeof(accel_buffer[0]), "X: %5d", x);
    snprintf(accel_buffer[1], sizeof(accel_buffer[1]), "Y: %5d", y);
    snprintf(accel_buffer[2], sizeof(accel_buffer[2]), "Z: %5d", z);

    ST7789_WriteString(10, 10, "Accelerometer Data", Font_11x18, BLUE, WHITE);
    ST7789_WriteString(10, 30, accel_buffer[0], Font_11x18, BLUE, WHITE);
    ST7789_WriteString(10, 50, accel_buffer[1], Font_11x18, BLUE, WHITE);
    ST7789_WriteString(10, 70, accel_buffer[2], Font_11x18, BLUE, WHITE);
    ST7789_WriteString(10, 110, encoder_delta_buffer, Font_11x18, BLUE, WHITE);
    ST7789_WriteString(10, 130, encoder_value_buffer, Font_11x18, BLUE, WHITE);

    osDelay(100);

  }
}

void EncoderReadTask(void *argument) {

  for (;;) {

    uint8_t encoder_data[4];
    uint8_t encoder_delta_data[4];

    uint16_t encoder_reg = ((uint16_t)SEESAW_ENCODER_BASE << 8) | (uint16_t)SEESAW_ENCODER_POSITION;
    uint16_t encoder_delta_reg = ((uint16_t)SEESAW_ENCODER_BASE << 8) | (uint16_t)SEESAW_ENCODER_DELTA;

    read_from_encoder(&hi2c1, encoder_reg, encoder_data, 4);
    read_from_encoder(&hi2c1, encoder_delta_reg, encoder_delta_data, 4);

    int32_t encoder_value = ((int32_t)encoder_data[0] << 24) | ((int32_t)encoder_data[1] << 16) | ((int32_t)encoder_data[2] << 8) | (int32_t)encoder_data[3];
    int32_t encoder_delta = ((int32_t)encoder_delta_data[0] << 24) | ((int32_t)encoder_delta_data[1] << 16) | ((int32_t)encoder_delta_data[2] << 8) | (int32_t)encoder_delta_data[3];


    osDelay(100);

  }

}
/* USER CODE END Tasks */


/* USER CODE BEGIN Header_StartDefaultTask */

void InitEncoder(void) {

  // set encoder to 0
  uint16_t encoder_position_reg = ((uint16_t)SEESAW_ENCODER_BASE << 8) | (uint16_t)SEESAW_ENCODER_POSITION;
  uint8_t encoder_position = 0;

  HAL_StatusTypeDef encoder_status = HAL_I2C_Mem_Write(&hi2c1, SEESAW_I2C_ADDRESS << 1, 
    encoder_position_reg, I2C_MEMADD_SIZE_16BIT, &encoder_position, 1, HAL_MAX_DELAY);
  if (encoder_status != HAL_OK) { printf("Error setting encoder position: %d\n", encoder_status); }

}

void InitAccelerometer(void) {
  
  uint8_t accel_init_data = 0x40; // 104 kHz, 2g
  uint16_t reg = CTRL1_XL;
  write_to_accel(&hi2c1, reg, &accel_init_data, 1);

  uint8_t tap_init_data = 0x08; // enable double-tap detection on int2
  reg = MD2_CFG;
  write_to_accel(&hi2c1, reg, &tap_init_data, 1);

  uint8_t tap_dir = 0x02; // z axis
  reg = TAP_CFG0;
  write_to_accel(&hi2c1, reg, &tap_dir, 1);

  uint8_t tap_ths = 0x80; // enable interrupts
  reg = TAP_CFG2;
  write_to_accel(&hi2c1, reg, &tap_ths, 1);

  uint8_t tap_dur2 = 0x10; 
  reg = TAP_THS_6D;
  write_to_accel(&hi2c1, reg, &tap_dur2, 1);

  uint8_t tap_en = 0x80; // enable double-tap detection
  reg = WAKE_UP_THS;
  write_to_accel(&hi2c1, reg, &tap_en, 1);

  uint8_t tap_dur = 0x2A; // ~600ms double tap delay, 2 quiet, 2 shock
  reg = INT_DUR2;
  write_to_accel(&hi2c1, reg, &tap_dur, 1);

}

void PollI2CDevices(void) {

  uint8_t i = 0;
  char buffer[100];
  // Scan the I2C bus for any connected devices
  for (uint8_t a = 1; a < 128; a++) {
    if (HAL_I2C_IsDeviceReady(&hi2c1, a << 1, 2, 10) == HAL_OK) {
      snprintf(buffer, sizeof(buffer), "Found device: 0x%02X\n", a);
      printf(buffer);
      ST7789_WriteString(10, (20 * i) + 10, buffer, Font_11x18, BLUE, WHITE);
      i++;
    }
  }
 
}

void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  // uint8_t i = 0;
  // char buffer[100];
  // char encoder_delta_buffer[100];
  // char encoder_value_buffer[100];
  // char accel_buffer[3][100];
  
  ST7789_Fill_Color(BLACK);

  PollI2CDevices();

  InitAccelerometer();
  InitEncoder();

  osThreadNew(AccelerometerReadTask, NULL, NULL);
  osThreadNew(EncoderReadTask, NULL, NULL);
  osThreadNew(DisplayWriteTask, NULL, NULL);
  osThreadNew(SystemStateTask, NULL, NULL);
  osThreadNew(ChronoTask, NULL, NULL);
  // osThreadNew(GameTask, NULL, NULL);

  printf("Initialization complete. Entering main loop...\n");

  osThreadStop(defaultTaskHandle); // Stop the default task after initialization

  /* Infinite loop */
  for (;;)
  {
    // ST7789_Fill_Color(BLACK);

    // osDelay(100);
  }
  /* USER CODE END 5 */
}
/* USER CODE END Header_StartDefaultTask */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
