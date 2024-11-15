/*
 * UART_DataProcess.c
 *
 *  Created on: 2024年9月15日
 *      Author: lin09
 */
#include "UART_DataProcess.h"

#define CMD_HEAD   0XF1
#define CMD_TAIL   0XF2F10D0A
#define IPD        0x4950442C  //IPD,
#define CONNECT    0x4E454354  //NECT
#define DISCONNECT 0x434C4F53  //CLOS
#define ESP_ERROR  0x4552524F  //ERRO
#define ESP_OK     0x0D0A4F4B  //\r\nOK

typedef struct{
  QueueSize Head;
  QueueSize Tail;
  QueueData Data[MAX_SIZE];
}DATAQUEUE;

static DATAQUEUE DataQueue  = {0, 0, {0}};
static uint32_t  DataState  = 0;
static QueueSize DataLength = 0;

void QueueReset(void)
{
  DataQueue.Head = DataQueue.Tail = 0;
  DataState = DataLength = 0;
}

void QueuePush(QueueData Data)
{
  QueueSize Position = (DataQueue.Head + 1) % MAX_SIZE;
  if(Position != DataQueue.Tail)
  {
    DataQueue.Data[DataQueue.Head] = Data;
    DataQueue.Head = Position;
  }
}

static void QueuePOP(QueueData *Data)
{
  if(DataQueue.Tail != DataQueue.Head)
  {
    *Data = DataQueue.Data[DataQueue.Tail];
    DataQueue.Tail = (DataQueue.Tail + 1) % MAX_SIZE;
  }
}

static QueueSize QueueDataSize(void)
{
  return ((DataQueue.Head + MAX_SIZE - DataQueue.Tail) % MAX_SIZE);
}

/*
 * ESP32 data message process
 */

static void ESP32_DataProcess(ControlMessage *ControlMsg,  uint32_t *ESP32_HeadData, uint8_t Data)
{
  *ESP32_HeadData = ((*ESP32_HeadData << 8) | Data);

  switch(*ESP32_HeadData)
  {
    case ESP_ERROR:  ControlMsg->ESP32Info.ERROR_STATE = true; break;
    case DISCONNECT: ControlMsg->ESP32Info.Connect_flag = false; break;
    case CONNECT:    ControlMsg->ESP32Info.Connect_flag = true; break;
    case IPD:        ControlMsg->ESP32Info.Aisle = DataQueue.Data[DataQueue.Tail];  break;
  }

#if 0

  if(ControlMsg->ESP32_STATE)
    if(HeadData == ESP_OK)
      ControlMsg->ESP32_Send_OK = true;

#endif

}

QueueSize QueueFindControlMessage(ControlMessage *ControlMsg, uint8_t BufferSize)
{
  QueueSize DataSize = 0;
  QueueData Data     = 0;

//  uint32_t ESP32_HeadData = 0;

  while(QueueDataSize() > 0)
  {
    QueuePOP(&Data);

    if(DataLength == 0 && Data != CMD_HEAD)
    {
//      ESP32_DataProcess(ControlMsg, &ESP32_HeadData, Data);
      continue;
    }

    if(DataLength < BufferSize)
      ControlMsg->TmpData[DataLength++] = Data;

    DataState = ((DataState << 8) | Data);

    if(DataState == CMD_TAIL)
    {
      DataSize   = DataLength;
      DataState  = 0;
      DataLength = 0;

      return DataSize;
    }
  }
  return 0;
}

void SendChar(UART_HandleTypeDef *huart, uint8_t t)
{
    HAL_UART_Transmit(huart, &t , sizeof(t) ,10);
}

void SendStrings(UART_HandleTypeDef *huart, uint8_t *str)
{
  while(*str)
  {
    SendChar(huart, (*str)&0xFF);
    str++;
  }
}

void AT_CIPSEND(UART_HandleTypeDef *huart, uint8_t Aisle, int SendSize)
{
  char buffer[20] = {0};
  sprintf(buffer, "AT+CIPSEND=%c,%d\r\n", Aisle, SendSize);
  SendStrings(huart, (uint8_t *)buffer);
}


