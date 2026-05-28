/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
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
#include "dma.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "SEGGER_RTT.h"
#include "bq79600.h"
#include "bq79600_def.h"
#include "bq79616_def.h"
#include "stm32f103xb.h"
#include "stm32f1xx_hal_gpio.h"
#include "utils.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define n_devices 2
#define n_cells_per_device 14
#define n_temp_pre_device 7

typedef struct {
  float temperature[n_temp_pre_device];  // GPIO raw voltage (mV) — NTC 校準後換算 °C
  float vcells[n_cells_per_device];      // mV
  float dietemp;                         // degC
  uint32_t timestamp;
  uint8_t fault_summary;                 // FAULT_SUMMARY
  uint8_t fault_ov[2];                   // FAULT_OV1 (cell 1-8), FAULT_OV2 (cell 9-16)
  uint8_t fault_uv[2];                   // FAULT_UV1 (cell 1-8), FAULT_UV2 (cell 9-16)
  uint8_t fault_ot;                      // FAULT_OT (GPIO 1-8)
  uint8_t fault_ut;                      // FAULT_UT (GPIO 1-8)
} module_t;

module_t modules[n_devices - 1] = {0};
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define bms_fault(state) HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, (state) ? GPIO_PIN_SET : GPIO_PIN_RESET)
#define bms_run() HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
float raw_to_float(void *raw) {
  return (float)(int16_t)(((*(uint16_t *)raw & 0xFF) << 8) | ((*(uint16_t *)raw & 0xFF00) >> 8));
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size) {
  static bq79600_t *instance = NULL;
  instance = open_bq79600_instance(0);
  if (instance == NULL) instance = open_bq79600_instance(0);
  instance->rx_len = size;
  bq79600_rx_callback(instance);
  bms_run();
  HAL_UARTEx_ReceiveToIdle_IT(&huart1, instance->rx_buf, sizeof(instance->rx_buf));
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
  SEGGER_RTT_Init();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  bms_fault(1);

  bq79600_t *bms_instance = open_bq79600_instance(0);
  bms_instance->mode = BQ_UART;
  bms_instance->state = BQ_SHUTDOWN;
  bms_instance->rx_port = GPIOA;
  bms_instance->tx_port = GPIOA;
  bms_instance->rx_pin = 9;
  bms_instance->tx_pin = 10;

  bq79600_wakeup(bms_instance);
  HAL_UARTEx_ReceiveToIdle_IT(&huart1, bms_instance->rx_buf, sizeof(bms_instance->rx_buf));
  HAL_Delay(10);

  uint8_t ctrl1_wake = 0x20;
  bq79600_write_reg(bms_instance, 0x00, CONTROL1, &ctrl1_wake, 1);
  HAL_Delay(12 * n_devices);

  bq79600_error_t err = bq79600_auto_addressing(bms_instance, n_devices);
  if (err) {
    SEGGER_RTT_printf(0, "[BQ79600] Auto addressing failed.\n");
    while (1);
  }

  /* Full stack initialization: cell count, GPIO, ADC, OV/UV/OT/UT, fault clear */
  bq79600_init(bms_instance, n_devices, n_cells_per_device);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* 1. Die temperature — frame: 2 data + 6 = 8 bytes/device */
    bq79600_construct_command(bms_instance, STACK_READ, 0, DIETEMP1_HI, 2, NULL);
    bq79600_tx(bms_instance);
    bq79600_bsp_ready(bms_instance);
    for (int i = 0; i < n_devices - 1; i++)
      modules[i].dietemp = raw_to_float(&bms_instance->rx_buf[4 + i * 8]) * 0.025f;

    /* 2. GPIO voltage (NTC) — GPIO1_HI ~ GPIO8_LO, 16 bytes, frame: 16+6=22 bytes/device */
    bq79600_construct_command(bms_instance, STACK_READ, 0, GPIO1_HI, 16, NULL);
    bq79600_tx(bms_instance);
    bq79600_bsp_ready(bms_instance);
    for (int i = 0; i < n_devices - 1; i++)
      for (int j = 0; j < n_temp_pre_device; j++) {
        float _mV = raw_to_float(&bms_instance->rx_buf[4 + i * 22 + j * 2]) * 0.19073f;
        modules[i].temperature[j] = NTC2T(_mV);
      }

    /* 3. Cell voltages — frame: (n_cells*2)+6 bytes/device */
    uint32_t start_vcells = VCELL1_HI - n_cells_per_device * 2 + 2;
    bq79600_construct_command(bms_instance, STACK_READ, 0, start_vcells, n_cells_per_device * 2, NULL);
    bq79600_tx(bms_instance);
    bq79600_bsp_ready(bms_instance);
    for (int i = 0; i < n_devices - 1; i++)
      for (int j = 0; j < n_cells_per_device; j++)
        modules[i].vcells[j] =
            raw_to_float(&bms_instance->rx_buf[4 + i * (n_cells_per_device * 2 + 6) + 2 * j]) * 0.19073f;

    /* 4. Timestamp */
    for (int i = 0; i < n_devices - 1; i++)
      modules[i].timestamp = HAL_GetTick();

    /* 5. Fault summary — 1 byte, frame: 1+6=7 bytes/device */
    bq79600_construct_command(bms_instance, STACK_READ, 0, BQ79616_FAULT_SUMMARY, 1, NULL);
    bq79600_tx(bms_instance);
    bq79600_bsp_ready(bms_instance);
    uint8_t any_fault = 0;
    for (int i = 0; i < n_devices - 1; i++) {
      modules[i].fault_summary = bms_instance->rx_buf[4 + i * 7];
      if (modules[i].fault_summary) any_fault = 1;
    }
    
    bms_fault(any_fault);

    /* 6. Fault detail — 只在有 fault 時才讀 */
    if (any_fault) {
      /* OV — FAULT_OV1(0x053C)+FAULT_OV2(0x053D), 2 bytes, frame: 8 bytes/device */
      bq79600_construct_command(bms_instance, STACK_READ, 0, FAULT_OV1, 2, NULL);
      bq79600_tx(bms_instance);
      bq79600_bsp_ready(bms_instance);
      for (int i = 0; i < n_devices - 1; i++) {
        modules[i].fault_ov[0] = bms_instance->rx_buf[4 + i * 8];
        modules[i].fault_ov[1] = bms_instance->rx_buf[5 + i * 8];
      }
      /* UV — FAULT_UV1(0x053E)+FAULT_UV2(0x053F), 2 bytes */
      bq79600_construct_command(bms_instance, STACK_READ, 0, FAULT_UV1, 2, NULL);
      bq79600_tx(bms_instance);
      bq79600_bsp_ready(bms_instance);
      for (int i = 0; i < n_devices - 1; i++) {
        modules[i].fault_uv[0] = bms_instance->rx_buf[4 + i * 8];
        modules[i].fault_uv[1] = bms_instance->rx_buf[5 + i * 8];
      }
      /* OT+UT — FAULT_OT(0x0540)+FAULT_UT(0x0541), 2 bytes */
      bq79600_construct_command(bms_instance, STACK_READ, 0, FAULT_OT, 2, NULL);
      bq79600_tx(bms_instance);
      bq79600_bsp_ready(bms_instance);
      for (int i = 0; i < n_devices - 1; i++) {
        modules[i].fault_ot = bms_instance->rx_buf[4 + i * 8];
        modules[i].fault_ut = bms_instance->rx_buf[5 + i * 8];
      }
    }

    /* 7. Print — RTT 不支援 %f，浮點轉整數印 */
    for (int i = 0; i < n_devices - 1; i++) {
      SEGGER_RTT_printf(0, "\n[Dev %d | t=%lu ms]\n", i, modules[i].timestamp);

      int d_i = (int)modules[i].dietemp;
      int d_f = (int)((modules[i].dietemp - d_i) * 100);
      SEGGER_RTT_printf(0, "  DieTmp : %d.%02d C\n", d_i, d_f);

      int j = n_cells_per_device;
      while (j--) {
        int v_i = (int)modules[i].vcells[j];
        int v_f = (int)((modules[i].vcells[j] - v_i) * 100);
        SEGGER_RTT_printf(0, "  V[%02d]  : %d.%02d mV\n", j + 1, v_i, v_f);
      }

      for (int j = 0; j < n_temp_pre_device; j++) {
        int t_i = (int)modules[i].temperature[j];
        SEGGER_RTT_printf(0, "  GPIO[%d] : %d degC\n", j + 1, t_i);
      }

      if (modules[i].fault_summary) {
        SEGGER_RTT_printf(0, "  [FAULT] summary=0x%02X\n", modules[i].fault_summary);
        SEGGER_RTT_printf(0, "         OV=0x%02X%02X UV=0x%02X%02X OT=0x%02X UT=0x%02X\n",
          modules[i].fault_ov[0], modules[i].fault_ov[1],
          modules[i].fault_uv[0], modules[i].fault_uv[1],
          modules[i].fault_ot, modules[i].fault_ut);
      } else {
        SEGGER_RTT_printf(0, "  [OK]\n");
      }
    }

    HAL_Delay(50);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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
  /* User can add his own implementation to report the HAL error return state */
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
