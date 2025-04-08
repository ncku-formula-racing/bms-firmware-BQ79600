#include "bq79600.h"
#include "SEGGER_RTT.h"

#define MAX_INSTANCE 1
static bq79600_t instance_list[MAX_INSTANCE];

void bq79600_construct_command(bq79600_t *instance, REQ_TYPE req_type,
                               uint8_t addr, uint16_t reg_addr,
                               uint8_t data_len, uint8_t *data) {
  uint8_t *tx_buf = instance->tx_buf;
  *tx_buf++ =
      0x80 | (req_type << 4) | ((req_type & 1) ? ((data_len - 1) & 0x0F) : 0);
  if (req_type < 2)
    *tx_buf++ = addr & 0x3F;
  *tx_buf++ = (reg_addr >> 8) & 0xFF;
  *tx_buf++ = reg_addr & 0xFF;
  for (int i = 0; i < data_len; i++)
    *tx_buf++ = data[i];
  uint16_t crc = bq79600_bsp_crc(instance->tx_buf, 4 + data_len);
  *tx_buf++ = crc & 0xFF;
  *tx_buf++ = (crc >> 8) & 0xFF;
  instance->tx_len = tx_buf - instance->tx_buf;
}

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
  SEGGER_RTT_printf(0, "BQ79600 wakeup.\n");
}
