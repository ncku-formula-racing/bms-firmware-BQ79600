#include "bq79600.h"
#include "usart.h"

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t size) {
  static bq79600_t* instance = NULL;
  instance = open_bq79600_instance(0);
  if (instance == NULL) instance = open_bq79600_instance(0);
  instance->rx_len = size;
  bq79600_rx_callback(instance);
  HAL_UARTEx_ReceiveToIdle_IT(huart, instance->rx_buf, sizeof(instance->rx_buf));
}
