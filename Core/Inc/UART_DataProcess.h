/*
 * UART_Data_Process.h
 *
 *  Created on: 2024年9月15日
 *      Author: lin09
 */

#ifndef INC_UART_DATAPROCESS_H_
#define INC_UART_DATAPROCESS_H_

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "main.h"
#include "stm32f1xx_it.h"

typedef unsigned char  QueueData;
typedef unsigned short QueueSize;

#define MAX_SIZE     512
#define BUFFER_SIZE  20

typedef struct{
  int        Send_Size;
  uint8_t    Aisle;
  bool       Connect_flag;
  bool       ERROR_STATE;
  bool       Send_OK;
}ESP32INFO;

typedef struct{
  char       TXBuffer[150];
  QueueData  TmpData[BUFFER_SIZE];
  QueueSize  MessageSize;

  ESP32INFO  ESP32Info;
}ControlMessage;

extern void QueueReset(void);
extern void QueuePush(QueueData Data);
extern QueueSize QueueFindControlMessage(ControlMessage *ControlMsg, uint8_t BufferSize);

void SendChar(UART_HandleTypeDef *huart, uint8_t t);
void SendStrings(UART_HandleTypeDef *huart, uint8_t *str);

/*
 * ESP32 send command
 */
void AT_CIPSEND(UART_HandleTypeDef *huart, uint8_t Aisle, int SendSize);

#endif /* INC_UART_DATAPROCESS_H_ */
