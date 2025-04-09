#include "bq79600.h"
#include "SEGGER_RTT.h"

#define MAX_INSTANCE 1
static bq79600_t instance_list[MAX_INSTANCE] = {0};

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
  if (data)
    for (int i = 0; i < data_len; i++)
      *tx_buf++ = data[i];
  else
    *tx_buf++ = data_len - 1;
  uint16_t crc = bq79600_bsp_crc(instance->tx_buf, 4 + data_len);
  *tx_buf++ = crc & 0xFF;
  *tx_buf++ = (crc >> 8) & 0xFF;
  instance->tx_len = tx_buf - instance->tx_buf;
}

void bq79600_tx(bq79600_t *instance) {
  instance->ready = 0;
  SEGGER_RTT_printf(0, "[BQ79600] TX: ");
  for (int i = 0; i < instance->tx_len; i++)
    SEGGER_RTT_printf(0, "%02X ", instance->tx_buf[i]);
  SEGGER_RTT_printf(0, "\n");
  switch (instance->mode) {
  case BQ_UART:
    bq79600_bsp_uart_tx(instance);
    break;
  default:
    break;
  }
}

void bq79600_rx_callback(bq79600_t *instance) {
  uint16_t crc = bq79600_bsp_crc(instance->rx_buf, instance->rx_len - 2);
  uint16_t crc_rx = (instance->rx_buf[instance->rx_len - 2] |
                     (instance->rx_buf[instance->rx_len - 1] << 8));
  if (crc != crc_rx) {
    SEGGER_RTT_printf(0, "[BQ79600] CRC error: %04X %04X\n", crc, crc_rx);
    instance->fault = 1;
    return;
  }
  instance->fault = 0;
  instance->ready = 1;
}

void bq79600_read_reg(bq79600_t *instance, uint8_t dev_addr, uint16_t reg_addr,
                      uint8_t *data) {
  bq79600_construct_command(instance, SINGLE_DEVICE_READ, dev_addr, reg_addr, 1,
                            NULL);
  bq79600_tx(instance);
  bq79600_bsp_ready(instance);
  *data = instance->rx_buf[4];
}

void bq79600_write_reg(bq79600_t *instance, uint8_t dev_addr, uint16_t reg_addr,
                       uint8_t *data, uint8_t data_len) {
  bq79600_construct_command(instance, SINGLE_DEVICE_WRITE, dev_addr, reg_addr,
                            data_len, data);
  bq79600_tx(instance);
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
  SEGGER_RTT_printf(0, "[BQ79600] wakeup.\n");
}
