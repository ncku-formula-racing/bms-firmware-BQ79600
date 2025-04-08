#include "bq79600.h"

#ifdef USE_HAL_DRIVER

#ifdef STM32F103xB
#include "main.h"
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
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = 1 << instance->rx_pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init((GPIO_TypeDef *)instance->rx_port, &GPIO_InitStruct);
  ((GPIO_TypeDef *)instance->rx_port)->BSRR |= (1 << (16 + instance->rx_pin));
  __delayus(2750);
  ((GPIO_TypeDef *)instance->rx_port)->BSRR |= (1 << instance->rx_pin);
  __delayus(3500);
}

void bq79600_bsp_uart_init(bq79600_t *instance) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = 1 << instance->rx_pin;
  GPIO_InitStruct.Mode =GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init((GPIO_TypeDef *)instance->rx_port, &GPIO_InitStruct);
  MX_USART1_UART_Init();
}

uint32_t bq79600_bsp_crc(uint8_t *buf, size_t len) {
  uint32_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= (uint16_t)(*buf++) & 0x00FF;
    crc = crc16_table[crc & 0x00FF] ^ (crc >> 8);
  }
  return crc;
}
#endif
