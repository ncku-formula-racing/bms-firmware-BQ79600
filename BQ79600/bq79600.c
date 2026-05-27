#include "bq79600.h"

#include "SEGGER_RTT.h"
#include "bq79616_def.h"
#include "stm32f1xx_hal.h"

#define BQ_LOG_ENABLE 0
#define MAX_INSTANCE 1
static bq79600_t instance_list[MAX_INSTANCE] = {0};

void bq79600_construct_command(bq79600_t* instance, REQ_TYPE req_type, uint8_t addr, uint16_t reg_addr,
                               uint8_t data_len, uint8_t* data) {
  uint8_t* tx_buf = instance->tx_buf;
  *tx_buf++ = 0x80 | (req_type << 4) | ((req_type & 1) ? ((data_len - 1) & 0x0F) : 0);
  if (req_type < 2) *tx_buf++ = addr & 0x3F;
  *tx_buf++ = (reg_addr >> 8) & 0xFF;
  *tx_buf++ = reg_addr & 0xFF;
  if (data)
    for (int i = 0; i < data_len; i++) *tx_buf++ = data[i];
  else
    *tx_buf++ = data_len - 1;
  if (req_type >= 2) data_len = 0;
  uint16_t crc = bq79600_bsp_crc(instance->tx_buf, 4 + data_len);
  *tx_buf++ = crc & 0xFF;
  *tx_buf++ = (crc >> 8) & 0xFF;
  instance->tx_len = tx_buf - instance->tx_buf;
}

void bq79600_tx(bq79600_t* instance) {
  instance->ready = 0;
#if BQ_LOG_ENABLE
  SEGGER_RTT_printf(0, "[BQ79600] TX: ");
  for (int i = 0; i < instance->tx_len; i++) SEGGER_RTT_printf(0, "%02X ", instance->tx_buf[i]);
  SEGGER_RTT_printf(0, "\n");
#endif
  switch (instance->mode) {
    case BQ_UART:
      bq79600_bsp_uart_tx(instance);
      break;
    default:
      break;
  }
}

void bq79600_rx_callback(bq79600_t* instance) {
  if (instance->rx_len < 6) return;
#if BQ_LOG_ENABLE
  SEGGER_RTT_printf(0, "[BQ79600] RX[%d]: ", instance->rx_len);
  for (int i = 0; i < instance->rx_len; i++) SEGGER_RTT_printf(0, "%02X ", instance->rx_buf[i]);
  SEGGER_RTT_printf(0, "\n");
#endif
  size_t idx = 0;
  uint8_t crc_buf[128 + 6];
  while (idx < instance->rx_len) {
    for (int i = 0; i < 4; i++) crc_buf[i] = instance->rx_buf[idx++];
    uint8_t len = (crc_buf[0] & 0x7F) + 1;
    for (int i = 0; i < len; i++) crc_buf[4 + i] = instance->rx_buf[idx++];
    crc_buf[4 + len] = instance->rx_buf[idx++];
    crc_buf[5 + len] = instance->rx_buf[idx++];
    uint16_t crc = bq79600_bsp_crc(crc_buf, len + 4);
    uint16_t crc_rx = (crc_buf[5 + len] << 8) | crc_buf[4 + len];
    if ((crc ^ crc_rx)) {
      SEGGER_RTT_printf(0, "[BQ79600] CRC error: %04X %04X\n", crc, crc_rx);
      instance->fault = 1;
      return;
    }
  }
  instance->fault = 0;
  instance->ready = 1;
}

void bq79600_read_reg(bq79600_t* instance, uint8_t dev_addr, uint16_t reg_addr, uint8_t* data) {
  bq79600_construct_command(instance, SINGLE_DEVICE_READ, dev_addr, reg_addr, 1, NULL);
  bq79600_tx(instance);
  bq79600_bsp_ready(instance);
  *data = instance->rx_buf[4];
}

void bq79600_write_reg(bq79600_t* instance, uint8_t dev_addr, uint16_t reg_addr, uint8_t* data,
                       uint8_t data_len) {
  bq79600_construct_command(instance, SINGLE_DEVICE_WRITE, dev_addr, reg_addr, data_len, data);
  bq79600_tx(instance);
}

bq79600_t* open_bq79600_instance(uint32_t id) {
  if (id >= MAX_INSTANCE) return NULL;
  return &instance_list[id];
}

void bq79600_wakeup(bq79600_t* instance) {
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

void bq79600_init(bq79600_t* instance, size_t n_devices, size_t n_cells_per_device) {
  uint8_t buf;

  SEGGER_RTT_printf(0, "[BQ79616] Starting stack init (%u devices, %u cells each).\n", (unsigned)n_devices,
                    (unsigned)n_cells_per_device);

  // -------------------------------------------------------------------------
  // 1. ACTIVE CELL CONFIGURATION
  buf = (uint8_t)(n_cells_per_device - 6);
  bq79600_construct_command(instance, STACK_WRITE, 0, ACTIVE_CELL, 1, &buf);
  bq79600_tx(instance);
  HAL_Delay(2);

  // Enable TSREF
  buf = 0x01;
  bq79600_construct_command(instance, STACK_WRITE, 0, CONTROL2, 1, &buf);
  bq79600_tx(instance);
  HAL_Delay(2);

  // -------------------------------------------------------------------------
  // 2. GPIO CONFIGURATION  (for NTC thermistor measurement)
  buf = 0x09;  // GPIO1, GPIO2 → ADC + OTUT input
  bq79600_construct_command(instance, STACK_WRITE, 0, GPIO_CONF1, 1, &buf);
  bq79600_tx(instance);
  HAL_Delay(1);

  buf = 0x09;  // GPIO3, GPIO4 → ADC + OTUT input
  bq79600_construct_command(instance, STACK_WRITE, 0, GPIO_CONF2, 1, &buf);
  bq79600_tx(instance);
  HAL_Delay(1);

  buf = 0x09;  // GPIO5, GPIO6 → ADC + OTUT input
  bq79600_construct_command(instance, STACK_WRITE, 0, GPIO_CONF3, 1, &buf);
  bq79600_tx(instance);
  HAL_Delay(1);

  buf = 0x01;  // GPIO7 → ADC + OTUT input GPIO8 -> HighZ
  bq79600_construct_command(instance, STACK_WRITE, 0, GPIO_CONF4, 1, &buf);
  bq79600_tx(instance);
  HAL_Delay(1);

  // -------------------------------------------------------------------------
  // 3. ADC CONFIGURATION  (OTP shadow – effective until power-off)
  buf = 0x06;  // AUX_SETTLE=4.3ms, LPF_BB=6.5Hz, LPF_VCELL=600Hz (1.6ms avg)
  bq79600_construct_command(instance, STACK_WRITE, 0, ADC_CONF1, 1, &buf);
  bq79600_tx(instance);
  HAL_Delay(1);

  buf = 0x00;  // ADC_DLY=0 (no delay)(default)
  bq79600_construct_command(instance, STACK_WRITE, 0, ADC_CONF2, 1, &buf);
  bq79600_tx(instance);
  HAL_Delay(1);

  // -------------------------------------------------------------------------
  // 4.1 OVERVOLTAGE THRESHOLD  (OTP shadow)
  buf = 0x25;  // 4250 mV (4.25 V)
  bq79600_construct_command(instance, STACK_WRITE, 0, OV_THRESH, 1, &buf);
  bq79600_tx(instance);
  HAL_Delay(1);

  // -------------------------------------------------------------------------
  // 4.2. UNDERVOLTAGE THRESHOLD  (OTP shadow)
  buf = 0x24;  // 3000 mV (3.0 V)
  bq79600_construct_command(instance, STACK_WRITE, 0, UV_THRESH, 1, &buf);
  bq79600_tx(instance);
  HAL_Delay(1);

  // -------------------------------------------------------------------------
  // 4.3. DISABLE UV ON UNUSED CELLS
  buf = 0x00;  // cells 1–8 all active
  bq79600_construct_command(instance, STACK_WRITE, 0, UV_DISABLE1, 1, &buf);
  bq79600_tx(instance);
  HAL_Delay(1);
  {
    uint8_t unused = (uint8_t)(16u - n_cells_per_device);
    // Shift unused count of '1' bits into the MSBs of the byte
    buf = (unused > 0) ? (uint8_t)((0xFFu << (8u - unused)) & 0xFFu) : 0x00u;
  }
  bq79600_construct_command(instance, STACK_WRITE, 0, UV_DISABLE2, 1, &buf);
  bq79600_tx(instance);
  HAL_Delay(1);

  // -------------------------------------------------------------------------
  // 5.1 OVER/UNDER TEMPERATURE THRESHOLD  (OTP shadow)
  buf = 0xED;  // OT_THR=0x0D(23%~55°C), UT_THR=0x7(80%~-20°C) — assumes 10k NTC + 10k pull-up, must calibrate
  bq79600_construct_command(instance, STACK_WRITE, 0, OTUT_THRESH, 1, &buf);
  bq79600_tx(instance);
  HAL_Delay(1);

  // -------------------------------------------------------------------------
  // 6. FAULT MASKING  (OTP shadow)
  buf = 0x00;
  bq79600_construct_command(instance, STACK_WRITE, 0, FAULT_MSK1, 1, &buf);
  bq79600_tx(instance);
  HAL_Delay(1);

  buf = 0x00;
  bq79600_construct_command(instance, STACK_WRITE, 0, FAULT_MSK2, 1, &buf);
  bq79600_tx(instance);
  HAL_Delay(1);

  // -------------------------------------------------------------------------
  // 7. COMMUNICATION TIMEOUT
  buf = 0x0A;  // CTL_ACT=1, CTL_TIME=010 (2s)
  bq79600_construct_command(instance, STACK_WRITE, 0, COMM_TIMEOUT_CONF, 1, &buf);
  bq79600_tx(instance);
  HAL_Delay(1);

  // -------------------------------------------------------------------------
  // 8. OV/UV HARDWARE PROTECTION CONTROL
  buf = 0x05;  // OVUV_MODE=01 (round-robin), OVUV_GO=1
  bq79600_construct_command(instance, STACK_WRITE, 0, OVUV_CTRL, 1, &buf);
  bq79600_tx(instance);
  HAL_Delay(1);

  // -------------------------------------------------------------------------
  // 9. OT/UT HARDWARE PROTECTION CONTROL
  buf = 0x05;  // OTUT_MODE=01 (round-robin), OTUT_GO=1
  bq79600_construct_command(instance, STACK_WRITE, 0, OTUT_CTRL, 1, &buf);
  bq79600_tx(instance);
  HAL_Delay(1);

  // -------------------------------------------------------------------------
  // 10. ADC RUNTIME CONTROL
  // ADC_CTRL1 (0x030D)
  buf = 0x1E;
  bq79600_construct_command(instance, STACK_WRITE, 0, ADC_CTRL1, 1, &buf);
  bq79600_tx(instance);
  HAL_Delay(1);

  // ADC_CTRL2 (0x030E)
  buf = 0x00;
  bq79600_construct_command(instance, STACK_WRITE, 0, ADC_CTRL2, 1, &buf);
  bq79600_tx(instance);
  HAL_Delay(1);

  // ADC_CTRL3 (0x030F): AUX ADC 控制
  buf = 0x06;
  bq79600_construct_command(instance, STACK_WRITE, 0, ADC_CTRL3, 1, &buf);
  bq79600_tx(instance);
  HAL_Delay(2u * (uint32_t)n_devices);  // 等待第一輪 conversion 完成

  // -------------------------------------------------------------------------
  // 11. CLEAR INITIAL FAULT FLAGS
  buf = 0xFF;
  bq79600_construct_command(instance, STACK_WRITE, 0, FAULT_RST1, 1, &buf);
  bq79600_tx(instance);
  HAL_Delay(1);

  buf = 0xFF;
  bq79600_construct_command(instance, STACK_WRITE, 0, FAULT_RST2, 1, &buf);
  bq79600_tx(instance);
  HAL_Delay(2);

  // Set balance done when Vcell = 2.45V
  buf = 0x01;
  bq79600_construct_command(instance, STACK_WRITE, 0, VCB_DONE_THRESH, 1, &buf);
  bq79600_tx(instance);
  HAL_Delay(2);

  SEGGER_RTT_printf(0, "[BQ79616] Stack init complete.\n");
}

bq79600_error_t bq79600_auto_addressing(bq79600_t* instance, const size_t n_devices) {
  uint8_t buf = 0;
  for (int addr = 0x343; addr < 0x34B; addr++) {
    bq79600_construct_command(instance, STACK_WRITE, 0, addr, 1, &buf);
    bq79600_tx(instance);
  }

  // Enable auto addressing
  buf = 0x01;
  bq79600_construct_command(instance, BROADCAST_WRITE, 0, CONTROL1, 1, &buf);
  bq79600_tx(instance);
  // brdcast write consecutively to 0x306
  for (size_t i = 0; i < n_devices; i++) {
    buf = i;
    bq79600_construct_command(instance, BROADCAST_WRITE, 0, DIR0_ADDR, 1, &buf);
    bq79600_tx(instance);
  }
  // brdcast write 0x02 to address 0x308 (set BQ7961X-Q1 as stack device )
  buf = 0x02;
  bq79600_construct_command(instance, BROADCAST_WRITE, 0, COMM_CTRL, 1, &buf);
  bq79600_tx(instance);

  buf = 0x03;
  bq79600_construct_command(instance, SINGLE_DEVICE_WRITE, n_devices - 1, COMM_CTRL, 1, &buf);
  bq79600_tx(instance);

  for (int addr = 0x343; addr < 0x34B; addr++) {
    bq79600_construct_command(instance, STACK_READ, 0, addr, 1, NULL);
    bq79600_tx(instance);
    bq79600_bsp_ready(instance);
    if (instance->fault) return BQ_ERROR;
  }

  for (size_t i = 0; i < n_devices; i++) {
    bq79600_construct_command(instance, SINGLE_DEVICE_READ, i, DIR0_ADDR, 1, NULL);
    bq79600_tx(instance);
    bq79600_bsp_ready(instance);
    if (instance->fault) return BQ_ERROR;
  }
  return BQ_SUCCESS;
}
