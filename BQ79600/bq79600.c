#include "bq79600.h"

#define MAX_INSTANCE 1
static bq79600_t instance_list[MAX_INSTANCE];

bq79600_t *open_bq79600_instance(uint32_t id) {
  if (id >= MAX_INSTANCE)
    return NULL;
  return &instance_list[id];
}

void bq79600_wakeup(bq79600_t *instance) {
  bq79600_bsp_wakeup(instance);
  switch (instance->mode) {
  case BQ_UART:
    bq79600_bsp_uart_init(instance);
    break;
  default:
    break;
  }
  instance->state = BQ_ACTIVATE;
}
