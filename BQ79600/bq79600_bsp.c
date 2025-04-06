#include "bq79600.h"

#ifdef USE_HAL_DRIVER

#ifdef STM32F103xB
#include "SEGGER_RTT.h"
#include "main.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_uart.h"
#endif

/* Utilize SysTick to implement uS delay */
extern TIM_HandleTypeDef htim1;
static inline void __delayus(uint32_t us) {
  const uint32_t period = htim1.Init.Period - 1;
  static uint32_t k;
  __disable_irq();
  __HAL_TIM_SET_COUNTER(&htim1, 0);
  while ((k = __HAL_TIM_GET_COUNTER(&htim1)) < us)
    us -= k > period ? period : 0;
  __enable_irq();
}

void bq79600_bsp_wakeup(bq79600_t *instance) {
  __delayus(1);
  ((GPIO_TypeDef *)instance->rx_port)->BSRR = (1 << instance->rx_pin);
  __delayus(2750);
  ((GPIO_TypeDef *)instance->rx_port)->BSRR = (1 << (instance->rx_pin + 16));
  __delayus(600);
  SEGGER_RTT_printf(0, "BQ79600 wakeup.\n");
}

void bq79600_bsp_uart_init(bq79600_t *instance) { MX_USART1_UART_Init(); }

#endif
