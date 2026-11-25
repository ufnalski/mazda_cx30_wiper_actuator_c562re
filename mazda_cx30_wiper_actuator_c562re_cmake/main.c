/**
 ******************************************************************************
 * file           : main.c
 * brief          : Main program body
 *                  Calls target system initialization then loop in main.
 ******************************************************************************
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

#include "ssd1306_fonts.h"
#include "ssd1306_mx2.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

#define LIN_USART USART3
#define LIN_TX_BUFFER_SIZE 11
#define LIN_RX_BUFFER_SIZE 16
#define LIN_SEND_PERIOD 80 // too frequent frames are ignored by the drive

#define MAZDA_CX30_RX_LIN_ID 0x10
#define MAZDA_CX30_TX_LIN_ID 0x25

#define PAYLOAD_PATTERN 0x00

// #define SLOW_SPEED

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

hal_uart_handle_t *huart3_lin;
uint8_t LinTxData[LIN_TX_BUFFER_SIZE];
uint8_t LinTxDataBis[LIN_RX_BUFFER_SIZE];
uint8_t LinRxData[LIN_RX_BUFFER_SIZE];
uint8_t LinRxDataBis[LIN_RX_BUFFER_SIZE];
uint32_t LinSoftTimer;
uint8_t lin_data_received_flag = 0;

hal_tim_handle_t *htim2_encoder;
uint8_t adjustable_byte;

uint8_t button_toggle = 0;
uint8_t send_toggle = 0;

char lcd_line[32];

#define SEQUENCE_LENGTH 8
#define REPEAT_LAST 2

// Brute force. There are some glitches. Probably bit flips should be
// synchronized with the status changes. Leaving to my Students :)
#ifdef SLOW_SPEED
uint8_t slow_speed_sequence[SEQUENCE_LENGTH] = {0x00, 0x01, 0x02, 0x03,
                                                0x04, 0x05, 0x01, 0x02};
#else
uint8_t fast_speed_sequence[SEQUENCE_LENGTH] = {0x00, 0x01, 0x02, 0x03,
                                                0x04, 0x05, 0x01, 0x03};
#endif

uint8_t sequence_idx = 0;

/* Private functions prototype -----------------------------------------------*/

// https://controllerstech.com/stm32-uart-8-lin-protocol-part-1/
uint8_t Pid_Calc(uint8_t ID);
uint8_t Checksum_Calc(uint8_t PID, uint8_t *data, uint8_t size);
void Error_Handler(void);

/**
 * brief:  The application entry point.
 * retval: none but we specify int to comply with C99 standard
 */
int main(void) {
  /** System Init: this code placed in targets folder initializes your system.
   * It calls the initialization (and sets the initial configuration) of the
   * peripherals. You can use STM32CubeMX to generate and call this code or not
   * in this project. It also contains the HAL initialization and the initial
   * clock configuration.
   */
  if (mx_system_init() != SYSTEM_OK) {
    return (-1);
  } else {
    /*
     * You can start your application code here
     */

    ssd1306_Init();

    ssd1306_Fill(Black);
    ssd1306_SetCursor(18, 0);
    ssd1306_WriteString("ufnalski.edu.pl", Font_6x8, White);
    ssd1306_SetCursor(6, 12);
    ssd1306_WriteString("Mazda CX-30 (2022+)", Font_6x8, White);
    ssd1306_SetCursor(2, 24);
    ssd1306_WriteString("MITSUBA wiper module", Font_6x8, White);
    ssd1306_UpdateScreen();

    htim2_encoder = mx_tim2_gethandle(); // encoder knob
    HAL_TIM_SetCounter(htim2_encoder, 0);
    HAL_TIM_Start_IT(htim2_encoder);

    huart3_lin = mx_usart3_uart_gethandle();
    HAL_UART_ReceiveToIdle_IT(huart3_lin, LinRxData, LIN_RX_BUFFER_SIZE);
    LIN_USART->CR2 |= USART_CR2_LBDIE; // LIN break detection interrupt enable

    LinSoftTimer = HAL_GetTick();

    while (1) {

      if (lin_data_received_flag == 1) {
        lin_data_received_flag = 0;
        sprintf(lcd_line, "RX:[%02X] %02X%02X %02X%02X %02X",
                MAZDA_CX30_RX_LIN_ID, LinRxDataBis[2], LinRxDataBis[3],
                LinRxDataBis[4], LinRxDataBis[5], LinRxDataBis[6]);
        ssd1306_SetCursor(2, 40);
        ssd1306_WriteString(lcd_line, Font_6x8, White);
        sprintf(lcd_line, "TX:[%02X] %02X%02X %02X%02X %02X",
                MAZDA_CX30_TX_LIN_ID, LinTxDataBis[2], LinTxDataBis[3],
                LinTxDataBis[4], LinTxDataBis[5], LinTxDataBis[6]);
        ssd1306_SetCursor(2, 52);
        ssd1306_WriteString(lcd_line, Font_6x8, White);
        ssd1306_UpdateScreen();
      }

      if (HAL_GetTick() - LinSoftTimer > LIN_SEND_PERIOD) {
        LinSoftTimer = HAL_GetTick();

        // while (!(LIN_USART->ISR & USART_ISR_TXE)) {
        //   __NOP();
        // }
        HAL_UART_SendLINBreak(huart3_lin);
      }
    }
  }
} /* end main */

void HAL_UART_ErrorCallback(hal_uart_handle_t *huart) { // just for debugging
  if (huart->instance == HAL_UART1) {
    if (LIN_USART->ISR & USART_ISR_FE) { // framing error
      LIN_USART->ICR |= USART_ICR_FECF;
      __NOP();
    }
  }
}

void HAL_UART_LINBreakCallback(hal_uart_handle_t *huart) {
  if (huart->instance == HAL_UART3) {
    LIN_USART->ICR |= USART_ICR_LBDCF; // clear flag

    if (send_toggle == 0) {
      send_toggle = 1;
      adjustable_byte = HAL_TIM_GetCounter(htim2_encoder);

      LinTxData[0] = 0x55; // sync field
      LinTxData[1] = Pid_Calc(MAZDA_CX30_TX_LIN_ID);

      if (button_toggle == 0) {
        LinTxData[2] = adjustable_byte;
        LinTxData[3] = PAYLOAD_PATTERN;
        LinTxData[4] = PAYLOAD_PATTERN;
        LinTxData[5] = PAYLOAD_PATTERN;
        LinTxData[6] = PAYLOAD_PATTERN;
        LinTxData[7] = PAYLOAD_PATTERN;
        LinTxData[8] = PAYLOAD_PATTERN;
        LinTxData[9] = PAYLOAD_PATTERN;
      } else {
#ifdef SLOW_SPEED
        LinTxData[2] = slow_speed_sequence[sequence_idx];
#else
        LinTxData[2] = fast_speed_sequence[sequence_idx];
#endif
        sequence_idx++;
        if (sequence_idx == SEQUENCE_LENGTH) {
          sequence_idx -= REPEAT_LAST;
        }
        LinTxData[3] = PAYLOAD_PATTERN;
        LinTxData[4] = PAYLOAD_PATTERN;
        LinTxData[5] = PAYLOAD_PATTERN;
        LinTxData[6] = PAYLOAD_PATTERN;
        LinTxData[7] = PAYLOAD_PATTERN;
        LinTxData[8] = PAYLOAD_PATTERN;
        LinTxData[9] = PAYLOAD_PATTERN;
      }

      LinTxData[10] =
          Checksum_Calc(LinTxData[1], LinTxData + 2, 8); // LIN 2.x includes PID

      LIN_USART->RQR |=
          USART_RQR_RXFRQ; // flush erroneous byte (coming from break field)
      memset(LinRxData, 0x00, LIN_RX_BUFFER_SIZE);
      HAL_UART_ReceiveToIdle_IT(huart3_lin, LinRxData, LIN_RX_BUFFER_SIZE);
      HAL_UART_Transmit_IT(huart3_lin, LinTxData, LIN_TX_BUFFER_SIZE);
      memcpy(LinTxDataBis, LinTxData, LIN_TX_BUFFER_SIZE);

    } else {
      send_toggle = 0;
      // ask servo drive for position/status/diagnostics
      LinTxData[0] = 0x55;
      LinTxData[1] = Pid_Calc(MAZDA_CX30_RX_LIN_ID);

      HAL_GPIO_TogglePin(LOGIC_ANALYZER_PORT, LOGIC_ANALYZER_PIN);

      LIN_USART->RQR |=
          USART_RQR_RXFRQ; // flush erroneous byte (coming from break field)
      memset(LinRxData, 0x00, LIN_RX_BUFFER_SIZE);
      HAL_UART_ReceiveToIdle_IT(huart3_lin, LinRxData, LIN_RX_BUFFER_SIZE);
      HAL_UART_Transmit_IT(huart3_lin, LinTxData, 2);
    }
  }
}

void HAL_UART_RxCpltCallback(hal_uart_handle_t *huart, uint32_t size_byte,
                             hal_uart_rx_event_types_t rx_event) {
  if (huart->instance == HAL_UART3) {
    if ((size_byte == 3 + 8 /* 3 + payload size */) &&
        (LinRxData[1] == Pid_Calc(MAZDA_CX30_RX_LIN_ID))) {
      lin_data_received_flag = 1;
      memcpy(LinRxDataBis, LinRxData, LIN_RX_BUFFER_SIZE);
    }
  }
}

void HAL_EXTI_TriggerCallback(hal_exti_handle_t *hexti,
                              hal_exti_trigger_t trigger) {
  if ((hexti->line == HAL_EXTI_LINE_0) &&
      (trigger == HAL_EXTI_TRIGGER_FALLING)) {
    button_toggle ^= 1;
  }
}

uint8_t Pid_Calc(uint8_t ID) {
  if (ID > 0x3F) {
    Error_Handler();
  }
  uint8_t IDBuf[6];
  for (int i = 0; i < 6; i++) {
    IDBuf[i] = (ID >> i) & 0x01;
  }

  uint8_t P0 = (IDBuf[0] ^ IDBuf[1] ^ IDBuf[2] ^ IDBuf[4]) & 0x01;
  uint8_t P1 = ~((IDBuf[1] ^ IDBuf[3] ^ IDBuf[4] ^ IDBuf[5]) & 0x01);

  ID = ID | (P0 << 6) | (P1 << 7);
  return ID;
}

uint8_t Checksum_Calc(uint8_t PID, uint8_t *data, uint8_t size) {
  uint8_t buffer[size + 2];
  uint16_t sum = 0;
  buffer[0] = PID;
  for (int i = 0; i < size; i++) {
    buffer[i + 1] = data[i];
  }

  for (int i = 0; i < size + 1; i++) {
    sum = sum + buffer[i];
    if (sum > 0xff)
      sum = sum - 0xff;
  }

  sum = 0xff - sum;
  return sum;
}

void Error_Handler(void) {
  while (1) {
    // Choose your favorite way to go into the panic mode :)
  }
}
