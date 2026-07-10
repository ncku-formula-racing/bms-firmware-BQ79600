#include "SEGGER_RTT.h"
#include "can_addr_def.h"
#include "bq79600.h"
#include "bq79600_def.h"
#include "bq79616_def.h"
#include "can.h"
#include "main.h"
#include "stm32f103xb.h"
#include "usart.h"
#include "utils.h"

#define n_devices 2
#define n_cells_per_device 14
#define n_temp_pre_device 7

typedef struct {
  float temperature[n_temp_pre_device];  // degC
  float vcells[n_cells_per_device];      // mV
  float dietemp;                         // degC
  uint32_t timestamp;
  uint8_t fault_summary;  // FAULT_SUMMARY
  uint8_t fault_ov[2];    // FAULT_OV1 (cell 1-8), FAULT_OV2 (cell 9-16)
  uint8_t fault_uv[2];    // FAULT_UV1 (cell 1-8), FAULT_UV2 (cell 9-16)
  uint8_t fault_ot;       // FAULT_OT (GPIO 1-8)
  uint8_t fault_ut;       // FAULT_UT (GPIO 1-8)
  uint8_t bal_stat;
} module_t;

module_t modules[n_devices - 1] = {0};
static uint8_t fault_count = 0;  // 連續 fault 次數，滿 3 才觸發 BMS_FAULT
static volatile uint8_t can_balance_trigger = 0;  // CAN RX ISR 設旗標，主迴圈才真正觸發平衡

#define bms_fault(state) HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, (state) ? GPIO_PIN_RESET : GPIO_PIN_SET)

static float raw_to_float(void* raw) {
  return (float)(int16_t)(((*(uint16_t*)raw & 0xFF) << 8) | ((*(uint16_t*)raw & 0xFF00) >> 8));
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef* hcan) {
  CAN_RxHeaderTypeDef rxHeader;
  uint8_t rxData[8];
  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &rxHeader, rxData) == HAL_OK) {
    if (rxHeader.IDE == CAN_ID_STD && rxHeader.StdId == CA_BAL_TRIG) {
      can_balance_trigger = 1;
    }
  }
}

int main2(void) {
  HAL_Delay(1000);

  bq79600_t* bms_instance = open_bq79600_instance(0);
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

  MX_CAN_Init();

  CAN_FilterTypeDef canFilter = {0};
  canFilter.FilterBank = 0;
  canFilter.FilterMode = CAN_FILTERMODE_IDMASK;
  canFilter.FilterScale = CAN_FILTERSCALE_32BIT;
  canFilter.FilterIdHigh = (CA_BAL_TRIG << 5) & 0xFFFF;  // 標準 ID 精確比對
  canFilter.FilterIdLow = 0x0000;
  canFilter.FilterMaskIdHigh = 0xFFE0;
  canFilter.FilterMaskIdLow = 0x0000;
  canFilter.FilterFIFOAssignment = CAN_FILTER_FIFO1;  // 對應現有 CAN1_RX1_IRQn
  canFilter.FilterActivation = ENABLE;
  canFilter.SlaveStartFilterBank = 14;
  HAL_CAN_ConfigFilter(&hcan, &canFilter);

  HAL_CAN_Start(&hcan);
  HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO1_MSG_PENDING);

  while (1) {
    uint8_t comm_fault = 0;

    /* CAN Flag trigger balance */
    if (can_balance_trigger) {
      can_balance_trigger = 0;
      SEGGER_RTT_printf(0, "[CAN] 0x%02X received, start balancing.\n", CA_BAL_TRIG);
      bq79600_balance(bms_instance, 0x3FFF);
      bq79600_balance_on(bms_instance);
    }

    /* 1. Die temperature — frame: 2 data + 6 = 8 bytes/device */
    bq79600_construct_command(bms_instance, STACK_READ, 0, DIETEMP1_HI, 2, NULL);
    bq79600_tx(bms_instance);
    bq79600_bsp_ready(bms_instance);
    if (bms_instance->fault) comm_fault = 1;
    for (int i = 0; i < n_devices - 1; i++)
      modules[i].dietemp = raw_to_float(&bms_instance->rx_buf[4 + i * 8]) * 0.025f;

    /* 2. GPIO voltage (NTC) — GPIO1_HI ~ GPIO8_LO, 16 bytes, frame: 16+6=22 bytes/device */
    bq79600_construct_command(bms_instance, STACK_READ, 0, GPIO1_HI, 16, NULL);
    bq79600_tx(bms_instance);
    bq79600_bsp_ready(bms_instance);
    if (bms_instance->fault) comm_fault = 1;
    for (int i = 0; i < n_devices - 1; i++)
      for (int j = 0; j < n_temp_pre_device; j++) {
        float _mV = raw_to_float(&bms_instance->rx_buf[4 + i * 22 + j * 2]) * 0.15259f;
        modules[i].temperature[j] = NTC2T(_mV);
      }

    /* 3. Cell voltages — frame: (n_cells*2)+6 bytes/device */
    uint32_t start_vcells = VCELL1_HI - n_cells_per_device * 2 + 2;
    bq79600_construct_command(bms_instance, STACK_READ, 0, start_vcells, n_cells_per_device * 2, NULL);
    bq79600_tx(bms_instance);
    bq79600_bsp_ready(bms_instance);
    if (bms_instance->fault) comm_fault = 1;
    for (int i = 0; i < n_devices - 1; i++)
      for (int j = 0; j < n_cells_per_device; j++)
        modules[i].vcells[n_cells_per_device - j - 1] =
            raw_to_float(&bms_instance->rx_buf[4 + i * (n_cells_per_device * 2 + 6) + 2 * j]) * 0.19073f;

    /* 4. Timestamp */
    for (int i = 0; i < n_devices - 1; i++) modules[i].timestamp = HAL_GetTick();

    /* 5. Fault summary — 1 byte, frame: 1+6=7 bytes/device */
    bq79600_construct_command(bms_instance, STACK_READ, 0, BQ79616_FAULT_SUMMARY, 1, NULL);
    bq79600_tx(bms_instance);
    bq79600_bsp_ready(bms_instance);
    if (bms_instance->fault) comm_fault = 1;
    uint8_t any_fault = 0;
    for (int i = 0; i < n_devices - 1; i++) {
      modules[i].fault_summary = bms_instance->rx_buf[4 + i * 7];
      if (modules[i].fault_summary) any_fault = 1;
    }
    if (comm_fault) any_fault = 1;

    if (any_fault) {
      if (fault_count < 3) fault_count++;
    } else {
      fault_count = 0;
    }
    bms_fault(fault_count >= 3);

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

      uint8_t rst1 = 0x78;  // RST_UT(bit6) | RST_OT(bit5) | RST_UV(bit4) | RST_OV(bit3)
      bq79600_construct_command(bms_instance, STACK_WRITE, 0, FAULT_RST1, 1, &rst1);
      bq79600_tx(bms_instance);
      HAL_Delay(1);
    }

    bq79600_construct_command(bms_instance, STACK_READ, 0, BAL_STAT, 1, NULL);
    bq79600_tx(bms_instance);
    bq79600_bsp_ready(bms_instance);
    if (bms_instance->fault) comm_fault = 1;
    for (int i = 0; i < n_devices - 1; i++) modules[i].bal_stat = bms_instance->rx_buf[4 + i * 7];

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
                          modules[i].fault_ov[0], modules[i].fault_ov[1], modules[i].fault_uv[0],
                          modules[i].fault_uv[1], modules[i].fault_ot, modules[i].fault_ut);
      } else {
        SEGGER_RTT_printf(0, "  [OK]\n");
      }
      SEGGER_RTT_printf(0, "Balancing status: 0x%02X\n", modules[i].bal_stat);
    }

    HAL_Delay(50);
  }
  return 0;
}
