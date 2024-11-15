/*
 * MassageSend.c
 *
 *  Created on: 2024年10月9日
 *      Author: lin09
 */

#include "MessageSend.h"

void SendCatPattern(UART_HandleTypeDef *huart, uint8_t CMD[])
{
  if(CMD[3] == 0x61 && CMD[4] == 0x74) // cat
  {
    SendINFO(huart, "           \n"
                    "     ^~^   \n"
                    "    ('Y') )\n"
                    "    /   \\/\n"
                    "   (\\|||/) <hello, world!>\n");
  }
}

void SendAuthor_Lkl(UART_HandleTypeDef *huart, uint8_t CMD[])
{
  if(CMD[3] == 0x75 && CMD[4] == 0x74 && CMD[5] == 0x68 && CMD[6] == 0x6F && CMD[7] == 0x72) //Author : Lkl
  {
    SendINFO(huart, "       <Author>       \n"
                    "   l       k       ll \n"
                    "   l       k   k    l \n"
                    "   l       k k      l \n"
                    "   lllll   k   k   lll\n");
  }
}
