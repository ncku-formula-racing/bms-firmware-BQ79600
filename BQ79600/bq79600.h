#pragma once

#include <stddef.h>
#include <stdint.h>

#include "bq79600_def.h"

typedef enum {
  BQ_ACTIVATE,
  BQ_SHUTDOWN,
  BQ_SLEEP,
} bq79600_state_t;

typedef enum {
  BQ_SPI,
  BQ_UART,
} bq79600_mode_t;

typedef struct {
  void *rx_port;
  void *tx_port;
  uint8_t rx_pin;
  uint8_t tx_pin;
  bq79600_mode_t mode;
  bq79600_state_t state;
  uint8_t tx_buf[8 + 6];
  uint8_t rx_buf[128 + 6];
  uint8_t tx_len;
  uint8_t rx_len;
} bq79600_t;

bq79600_t *open_bq79600_instance(uint32_t id);

void bq79600_wakeup(bq79600_t *instance);
void bq79600_construct_command(bq79600_t *instance, REQ_TYPE req_type,
                               uint8_t addr, uint16_t reg_addr,
                               uint8_t data_len, uint8_t *data);
void bq79600_bsp_wakeup(bq79600_t *instance);
void bq79600_bsp_spi_init(bq79600_t *instance);
void bq79600_bsp_uart_init(bq79600_t *instance);
uint32_t bq79600_bsp_crc(uint8_t *buf, size_t len);
