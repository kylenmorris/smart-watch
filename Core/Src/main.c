/* USER CODE BEGIN Header */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
// #include "st7789.h"
#include "LSM6DSO.h"
#include "seesaw_driver.h"
#include "stm32l4xx_hal_def.h"
#include "GC9A01.h"
#include "fonts.h"
#include "math.h"
#include "stm32l4xx_hal_pwr.h"
#include "string.h"
#include "stdlib.h"
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

UART_HandleTypeDef huart2;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* USER CODE BEGIN PV */

#define ACCEL_X 0
#define ACCEL_Y 1
#define ACCEL_Z 2

#define EVENT_REDRAW_DISPLAY (1U << 0)
#define EVENT_ENTER_SLEEP_MODE (1U << 1)
#define EVENT_WAKE_UP_FROM_SLEEP (1U << 2)

#define SYSTEM_STATE_TASK_STACK_SIZE 256 * 4
#define ENCODER_TASK_STACK_SIZE 256 * 4         
#define DISPLAY_TASK_STACK_SIZE 256 * 4         // needed 96 * 4 last check
#define ACCELEROMETER_TASK_STACK_SIZE 256 * 4

#define SYSTEM_STATE_TASK_PRIORITY osPriorityNormal
#define ENCODER_TASK_PRIORITY osPriorityNormal
#define DISPLAY_TASK_PRIORITY osPriorityNormal
#define ACCELEROMETER_TASK_PRIORITY osPriorityNormal1

// ############ Wait Periods etc ################

#define SYSTEM_STATE_TASK_WAIT 50
#define DISPLAY_TASK_WAIT 50
#define ENCODER_TASK_WAIT 50

#define SYSTEM_STATE_TASK_DELAY 10
#define ENCODER_TASK_DELAY 100
#define DISPLAY_TASK_DELAY 20
#define ACCELEROMETER_TASK_DELAY 20

#define DISPLAY_REDRAW_PERIOD 1000

#define TIME_TO_SLEEP 10000


// ########### Display State Management ################

typedef struct {
  uint8_t current_state;
  RTC_TimeTypeDef current_time;
  RTC_DateTypeDef current_date;
  uint8_t orientation;
} DisplayState;

DisplayState displayState = {
  .current_state = 0,
  .current_time = {0},
  .current_date = {0},
  .orientation = GC9A01_TOP
};

typedef struct {
  DisplayState *displayState;
  osMutexId_t systemStateMutex;
  osEventFlagsId_t eventFlags;
} DisplayTaskArgs;

DisplayTaskArgs displayTaskArgs = {
  .displayState = &displayState
};

// ############## System State Task ################

typedef struct {
  osMessageQueueId_t encoderQueue;
  osMutexId_t accelMutex;
  DisplayState *displayState;
  osMutexId_t systemStateMutex;
  osEventFlagsId_t eventFlags;
  osTimerId_t timerHandle;
} SystemStateTaskArgs;

SystemStateTaskArgs systemStateTaskArgs = {
  .displayState = &displayState
};

// ############## Sleep Task ################

typedef struct {
  osEventFlagsId_t eventFlags;
  osTimerId_t sleepTimerHandle;
} SleepTaskArgs;

SleepTaskArgs sleepTaskArgs;

// ############## Encoder Task ################

typedef struct {
  I2C_HandleTypeDef *hi2c;
  osMessageQueueId_t queue;
  osTimerId_t sleepTimerHandle;
} EncoderTaskArgs;

EncoderTaskArgs encoderTaskArgs = {
  .hi2c = &hi2c1,
  .queue = NULL
};

// ############## Accelerometer Task ################

typedef struct {
  I2C_HandleTypeDef *hi2c;
  osMutexId_t accelMutex;
} AccelerometerTaskArgs;

AccelerometerTaskArgs accelerometerTaskArgs = {
  .hi2c = &hi2c1,
  .accelMutex = NULL
};

// ############## Task Handles ################

osThreadId_t systemStateTaskHandle;
osThreadId_t encoderTaskHandle;
osThreadId_t accelerometerTaskHandle;
osThreadId_t displayTaskHandle;
osThreadId_t sleepTaskHandle;

// ############## Task Attributes ################

const osThreadAttr_t systemStateTask_attributes = {
  .name = "systemStateTask",
  .stack_size = SYSTEM_STATE_TASK_STACK_SIZE,
  .priority = SYSTEM_STATE_TASK_PRIORITY,
};

const osThreadAttr_t encoderTask_attributes = {
  .name = "encoderTask",
  .stack_size = ENCODER_TASK_STACK_SIZE,
  .priority = ENCODER_TASK_PRIORITY,
};

const osThreadAttr_t displayTask_attributes = {
  .name = "displayTask",
  .stack_size = DISPLAY_TASK_STACK_SIZE,
  .priority = DISPLAY_TASK_PRIORITY,
};

const osThreadAttr_t accelerometerTask_attributes = {
  .name = "accelerometerTask",
  .stack_size = ACCELEROMETER_TASK_STACK_SIZE,
  .priority = ACCELEROMETER_TASK_PRIORITY,
};

const osThreadAttr_t sleepTask_attributes = {
  .name = "sleepTask",
  .stack_size = 512 * 4,
  .priority = osPriorityNormal2,
};

int16_t accel_data_global[3];
volatile uint8_t accel_interrupt_flag; 

volatile uint8_t dma_spi_fl1 = 0;

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
void write_to_accel(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t *data, uint16_t size);
void read_from_accel(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t *data, uint16_t size);
void read_from_encoder(I2C_HandleTypeDef *hi2c, uint16_t reg, uint8_t *data, uint16_t size);

void InitEncoder(void);
void InitAccelerometer(void);

void DrawWatchStateFace(DisplayState *displayState);
void DrawChronoStateFace(DisplayState *displayState);

void SystemStateTask(void *argument);
void AccelerometerTask(void *argument);
void DisplayTask(void *argument);
void EncoderTask(void *argument);
void SleepTask(void *argument);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/

void TimerCallback(void *argument) {
  osEventFlagsSet(systemStateTaskArgs.eventFlags, EVENT_REDRAW_DISPLAY);
}

void SleepTimerCallback(void *argument) {
  osEventFlagsSet(systemStateTaskArgs.eventFlags, EVENT_ENTER_SLEEP_MODE);
}

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
  // ST7789_Init();							

  /* USER CODE BEGIN Init */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
  NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 15, 0));
  /* USER CODE END Init */

  LL_DMA_DisableChannel(DMA_NO, DMA_CHANNEL);
  LL_DMA_ClearFlag_TC(DMA_NO);
  LL_DMA_ClearFlag_TE(DMA_NO);
  LL_DMA_EnableIT_TC(DMA_NO, DMA_CHANNEL);
  LL_DMA_EnableIT_TE(DMA_NO, DMA_CHANNEL);
  LL_SPI_EnableDMAReq_TX(SPI_NO);
  LL_SPI_Enable(SPI_NO);

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  osMutexId_t accelMutexHandle = osMutexNew(NULL);
  accelerometerTaskArgs.accelMutex = accelMutexHandle;

  osMutexId_t systemStateMutexHandle = osMutexNew(NULL);
  systemStateTaskArgs.systemStateMutex = systemStateMutexHandle;
  displayTaskArgs.systemStateMutex = systemStateMutexHandle;
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  osTimerId_t displayTimerHandle = osTimerNew(TimerCallback, osTimerPeriodic, NULL, NULL);
  systemStateTaskArgs.timerHandle = displayTimerHandle;

  osTimerId_t sleepTimerHandle = osTimerNew(SleepTimerCallback, osTimerPeriodic, NULL, NULL);
  encoderTaskArgs.sleepTimerHandle = sleepTimerHandle;
  sleepTaskArgs.sleepTimerHandle = sleepTimerHandle;

  osTimerStart(sleepTimerHandle, TIME_TO_SLEEP); 
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  osMessageQueueId_t encoderQueueHandle = osMessageQueueNew(3, sizeof(int8_t), NULL); 
  encoderTaskArgs.queue = encoderQueueHandle;
  systemStateTaskArgs.encoderQueue = encoderQueueHandle;

  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);
  /* creation of defaultTask */

  /* USER CODE BEGIN RTOS_THREADS */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  osEventFlagsId_t eventFlagsHandle = osEventFlagsNew(NULL);
  displayTaskArgs.eventFlags = eventFlagsHandle;
  sleepTaskArgs.eventFlags = eventFlagsHandle;
  systemStateTaskArgs.eventFlags = eventFlagsHandle;
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

void InitEncoder(void) {

  // set encoder to 0
  uint16_t encoder_position_reg = ((uint16_t)SEESAW_ENCODER_BASE << 8) | (uint16_t)SEESAW_ENCODER_POSITION;
  uint8_t encoder_position = 0;

  HAL_StatusTypeDef encoder_status = HAL_I2C_Mem_Write(&hi2c1, SEESAW_I2C_ADDRESS << 1, 
    encoder_position_reg, I2C_MEMADD_SIZE_16BIT, &encoder_position, 1, HAL_MAX_DELAY);
  if (encoder_status != HAL_OK) { printf("Error setting encoder position: %d\n", encoder_status); }

}

void AccelerometerLowPower(void) {
  uint16_t reg = CTRL5_C;
  uint8_t accel_sleep_data = 0x80; // XL_ULP_EN = 1
  write_to_accel(&hi2c1, reg, &accel_sleep_data, 1);
}

void InitAccelerometer(void) {
  
  uint16_t reg;
  uint8_t data;
  
  reg = CTRL6_C;
  data = 0x10; // disable high-performance mode = 1
  write_to_accel(&hi2c1, reg, &data, 1);
  
  reg = CTRL1_XL;
  data = 0x30; // 52Hz, 2h, high-resolution selection = 0 
  write_to_accel(&hi2c1, reg, &data, 1);

  reg = MD1_CFG;
  data = 0x08; // enable double-tap detection interrupt
  write_to_accel(&hi2c1, reg, &data, 1);

  reg = TAP_CFG0;
  data = 0x02;       // z axis tap
  write_to_accel(&hi2c1, reg, &data, 1);

  reg = TAP_CFG2;
  data = 0x80;       // enable interrupts
  write_to_accel(&hi2c1, reg, &data, 1);

  reg = TAP_THS_6D;
  data = 0x10; 
  write_to_accel(&hi2c1, reg, &data, 1);

  reg = INT_DUR2;
  data = 0x2A; // ~600ms double tap delay, 2 quiet, 2 shock
  write_to_accel(&hi2c1, reg, &data, 1);

  reg = WAKE_UP_THS;
  data = 0x80; // enable double-tap detection
  write_to_accel(&hi2c1, reg, &data, 1);


}

void PollI2CDevices(void) {

  uint8_t i = 0;
  char buffer[100];
  // Scan the I2C bus for any connected devices
  for (uint8_t a = 1; a < 128; a++) {
    if (HAL_I2C_IsDeviceReady(&hi2c1, a << 1, 2, 10) == HAL_OK) {
      snprintf(buffer, sizeof(buffer), "Found device: 0x%02X\n", a);
      printf(buffer);
      // ST7789_WriteString(10, (20 * i) + 10, buffer, Font_11x18, BLUE, WHITE);
      i++;
    }
  }
 
}

// void EncoderInterruptTask(void *argument) {
//   while (1) {
//     if (accel_interrupt_flag) {
//       accel_interrupt_flag = 0;
//       printf("Accelerometer interrupt detected!\n");
//     }
//     osDelay(10);
//   }
// }

void SleepTask(void *argument) {

  SleepTaskArgs *args = (SleepTaskArgs *)argument;
  osEventFlagsId_t eventFlags = args->eventFlags;
  osTimerId_t sleepTimer = args->sleepTimerHandle;

  for (;;) {
    osEventFlagsWait(eventFlags, EVENT_ENTER_SLEEP_MODE, osFlagsWaitAny, osWaitForever);
    

    GC9A01_ClearScreen(BLACK);
    GC9A01_String(40, 120, "Entering Sleep Mode...");
    GC9A01_Sleep();

    HAL_SuspendTick();

    // Enter Stop2 with wake from interrupt
    HAL_PWREx_EnterSTOP2Mode(PWR_STOPENTRY_WFI);

    HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
    // osEventFlagsWait(eventFlags, EVENT_WAKE_UP_FROM_SLEEP, osFlagsWaitAny, osWaitForever);
    // osEventFlagsClear(eventFlags, EVENT_WAKE_UP_FROM_SLEEP);

    // Resume the systick to allow interrupts and waking of mcu
    HAL_ResumeTick();

    // Reconfigure the system clock
    SystemClock_Config();

    GC9A01_WakeUp();

    osEventFlagsClear(eventFlags, EVENT_ENTER_SLEEP_MODE);

    osTimerStart(sleepTimer, TIME_TO_SLEEP);
  }
}

typedef enum {
  STATE_WATCH,
  STATE_CHRONO,
  STATE_GAME
} SystemState;

void DrawWatchStateFace(DisplayState *displayState) {
  
  char time_buffer[20];
  char date_buffer[20];

  RTC_TimeTypeDef sTime = displayState->current_time;
  RTC_DateTypeDef sDate = displayState->current_date;

  snprintf(time_buffer, sizeof(time_buffer), "%02d:%02d:%02d", sTime.Hours, sTime.Minutes, sTime.Seconds);
  snprintf(date_buffer, sizeof(date_buffer), "%02d/%02d/%04d", sDate.Month, sDate.Date, sDate.Year);
  
  GC9A01_ClearScreen(BLACK);

  GC9A01_String(40, 120, time_buffer);
  GC9A01_String(40, 150, date_buffer);
}

void DrawChronoStateFace(DisplayState *displayState) {

  char chrono_buffer[20];

  snprintf(chrono_buffer, sizeof(chrono_buffer), "Timer: 00:00.00");

  GC9A01_ClearScreen(BLACK);
  GC9A01_String(20, 80, chrono_buffer);
}

void DisplayTask(void *argument) {

  UBaseType_t uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);

  DisplayTaskArgs *args = (DisplayTaskArgs *)argument;
  osMutexId_t systemStateMutex = args->systemStateMutex;
  DisplayState *displayState = args->displayState;
  osEventFlagsId_t eventFlags = args->eventFlags;
 
  DisplayState local_display_state;


  for (;;) {

    osEventFlagsWait(eventFlags, EVENT_REDRAW_DISPLAY, osFlagsWaitAny, osWaitForever);
    osEventFlagsClear(eventFlags, EVENT_REDRAW_DISPLAY);

    osMutexAcquire(systemStateMutex, DISPLAY_TASK_WAIT);
    local_display_state = *displayState;
    osMutexRelease(systemStateMutex);

    GC9A01_Set_Orientation(local_display_state.orientation);

    switch (local_display_state.current_state) {
      case STATE_WATCH:
        DrawWatchStateFace(&local_display_state);
        break;
      case STATE_CHRONO:
        DrawChronoStateFace(&local_display_state);
        break;
      case STATE_GAME:
        // DrawGameStateFace();
        break;
      default:
        break;
    }

    uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);

    osDelay(DISPLAY_TASK_DELAY);

  }
}

void SystemStateTask(void *argument) {

  UBaseType_t uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);

  SystemStateTaskArgs *args = (SystemStateTaskArgs *)argument;
  osMessageQueueId_t encoderQueue = args->encoderQueue;
  osMutexId_t accelMutex = args->accelMutex;
  osMutexId_t systemStateMutex = args->systemStateMutex;
  DisplayState *displayState = args->displayState;
  osTimerId_t timerHandle = args->timerHandle;
  osEventFlagsId_t eventFlags = args->eventFlags;
  
  int16_t accelerometer_data_y;
  int8_t encoder_delta;

  DisplayState local_display_state = {
    .current_state = STATE_WATCH,
    .current_time = {0},
    .current_date = {0},
    .orientation = GC9A01_TOP
  };

  uint8_t redraw = 0;
  uint8_t previous_orientation = GC9A01_TOP;

  osTimerStart(timerHandle, DISPLAY_REDRAW_PERIOD);
  
  GC9A01_ClearScreen(BLACK);

  for (;;) {

    if (osMessageQueueGet(encoderQueue, &encoder_delta, NULL, 100) == osOK) {
      if (encoder_delta > 0) {        local_display_state.current_state = STATE_WATCH;  } 
      else if (encoder_delta < 0) {   local_display_state.current_state = STATE_CHRONO; }
      redraw = 1;
    }

    osMutexAcquire(accelMutex, SYSTEM_STATE_TASK_WAIT);
    accelerometer_data_y = accel_data_global[ACCEL_Y];
    osMutexRelease(accelMutex);

    if (accelerometer_data_y < 0) {       local_display_state.orientation = GC9A01_TOP;    }        
    else if (accelerometer_data_y >= 0) { local_display_state.orientation = GC9A01_BOTTOM; }
    
    if (local_display_state.orientation != previous_orientation) {
      redraw = 1;
      previous_orientation = local_display_state.orientation;
    }

    HAL_RTC_GetTime(&hrtc, &local_display_state.current_time, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&hrtc, &local_display_state.current_date, RTC_FORMAT_BIN);

    if (redraw) {
      osEventFlagsSet(eventFlags, EVENT_REDRAW_DISPLAY);
      redraw = 0;
    }

    osMutexAcquire(systemStateMutex, SYSTEM_STATE_TASK_WAIT);
    *displayState = local_display_state;
    osMutexRelease(systemStateMutex);

    uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
    osDelay(SYSTEM_STATE_TASK_DELAY);
  }
}

void ChronoTask(void *argument) {

  for (;;) {
    // Handle chronograph functionality
    osDelay(100);
  }
}

void AccelerometerReadTask(void *argument) {

  UBaseType_t uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);

  AccelerometerTaskArgs *args = (AccelerometerTaskArgs *)argument;
  osMutexId_t accelMutex = args->accelMutex;

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
  
    osMutexAcquire(accelMutex, SYSTEM_STATE_TASK_WAIT);
    accel_data_global[ACCEL_X] = x;
    accel_data_global[ACCEL_Y] = y;
    accel_data_global[ACCEL_Z] = z;
    osMutexRelease(accelMutex);

    uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
    osDelay(ACCELEROMETER_TASK_DELAY);
  }
}

void EncoderReadTask(void *argument) {

  UBaseType_t uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);

  EncoderTaskArgs *args = (EncoderTaskArgs *)argument;
  
  osMessageQueueId_t encoderQueue = args->queue;
  I2C_HandleTypeDef *hi2c = args->hi2c;
  osTimerId_t sleepTimerHandle = args->sleepTimerHandle;

  for (;;) {

    uint8_t encoder_delta_data[4];

    uint16_t encoder_delta_reg = ((uint16_t)SEESAW_ENCODER_BASE << 8) | (uint16_t)SEESAW_ENCODER_DELTA;

    read_from_encoder(hi2c, encoder_delta_reg, encoder_delta_data, 4);

    // int32_t encoder_delta = ((int32_t)encoder_delta_data[0] << 24) | ((int32_t)encoder_delta_data[1] << 16) | ((int32_t)encoder_delta_data[2] << 8) | (int32_t)encoder_delta_data[3];
    int8_t encoder_delta = (int8_t)encoder_delta_data[3]; 

    if (encoder_delta != 0) {
      osMessageQueuePut(encoderQueue, &encoder_delta, 0, 0);
      osTimerStart(sleepTimerHandle, TIME_TO_SLEEP); // Reset sleep timer on encoder activity
    }

    uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
    osDelay(100);
  }

}

/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{

  UBaseType_t uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);

  /* Infinite loop */
  for (;;)
  {
    // ST7789_Fill_Color(BLACK);
    /* USER CODE BEGIN 5 */
    // uint8_t i = 0;
    // char buffer[100];
    // char encoder_delta_buffer[100];
    // char encoder_value_buffer[100];
    // char accel_buffer[3][100];
    
    // ST7789_Fill_Color(BLACK);

    GC9A01_Initial();
    // GC9A01_ClearScreen(WHITE);
    GC9A01_SetBackColor(BLACK);
    GC9A01_SetTextColor(WHITE);
    GC9A01_SetFont(&Font24);
    // GC9A01_Text("Hello world", 1);
    // GC9A01_DrawCircle(120, 120, 60, BLUE);

    // PollI2CDevices();

    InitAccelerometer();
    InitEncoder();

    // osThreadNew(AccelerometerReadTask, NULL, NULL);
    osThreadNew(DisplayTask, &displayTaskArgs, &displayTask_attributes);
    osThreadNew(AccelerometerReadTask, &accelerometerTaskArgs, &accelerometerTask_attributes);
    osThreadNew(EncoderReadTask, &encoderTaskArgs, &encoderTask_attributes);
    osThreadNew(SystemStateTask, &systemStateTaskArgs, &systemStateTask_attributes);
    osThreadNew(SleepTask, &sleepTaskArgs, &sleepTask_attributes);
    // osThreadNew(WakeUpTask, &wakeUpTaskArgs, NULL);
    // osThreadNew(ChronoTask, NULL, NULL);
    // osThreadNew(GameTask, NULL, NULL);

    // printf("Initialization complete. Entering main loop...\n");

    // osThreadStop(defaultTaskHandle); // Stop the default task after initialization
    uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
    osThreadTerminate(defaultTaskHandle); // Terminate the default task after initialization

    // osDelay(1000);
  }
  /* USER CODE END 5 */
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

  LL_SPI_InitTypeDef SPI_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI1);

  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
  /**SPI1 GPIO Configuration
  PA1   ------> SPI1_SCK
  PA12   ------> SPI1_MOSI
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_1|LL_GPIO_PIN_12;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* SPI1 DMA Init */

  /* SPI1_TX Init */
  LL_DMA_SetPeriphRequest(DMA2, LL_DMA_CHANNEL_4, LL_DMA_REQUEST_4);

  LL_DMA_SetDataTransferDirection(DMA2, LL_DMA_CHANNEL_4, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);

  LL_DMA_SetChannelPriorityLevel(DMA2, LL_DMA_CHANNEL_4, LL_DMA_PRIORITY_MEDIUM);

  LL_DMA_SetMode(DMA2, LL_DMA_CHANNEL_4, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(DMA2, LL_DMA_CHANNEL_4, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA2, LL_DMA_CHANNEL_4, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA2, LL_DMA_CHANNEL_4, LL_DMA_PDATAALIGN_BYTE);

  LL_DMA_SetMemorySize(DMA2, LL_DMA_CHANNEL_4, LL_DMA_MDATAALIGN_BYTE);

  /* USER CODE BEGIN SPI1_Init 1 */
  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
  SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
  SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_8BIT;
  SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_HIGH;
  SPI_InitStruct.ClockPhase = LL_SPI_PHASE_1EDGE;
  SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
  SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV2;
  SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
  SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
  SPI_InitStruct.CRCPoly = 7;
  LL_SPI_Init(SPI1, &SPI_InitStruct);
  LL_SPI_SetStandard(SPI1, LL_SPI_PROTOCOL_MOTOROLA);
  LL_SPI_EnableNSSPulseMgt(SPI1);
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
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Channel4_IRQn interrupt configuration */
  NVIC_SetPriority(DMA2_Channel4_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),5, 0));
  NVIC_EnableIRQ(DMA2_Channel4_IRQn);

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
  HAL_GPIO_WritePin(GPIOA, DISP_CS_Pin|ST7789_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, ST7789_DC_Pin|LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : DISP_CS_Pin ST7789_RST_Pin */
  GPIO_InitStruct.Pin = DISP_CS_Pin|ST7789_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
