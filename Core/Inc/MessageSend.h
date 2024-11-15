/*
 * MessageSend.h
 *
 *  Created on: 2024年10月9日
 *      Author: lin09
 */

#ifndef INC_MESSAGESEND_H_
#define INC_MESSAGESEND_H_

#include "main.h"

void SendCatPattern(UART_HandleTypeDef *huart, uint8_t CMD[]);
void SendAuthor_Lkl(UART_HandleTypeDef *huart, uint8_t CMD[]);

#endif /* INC_MESSAGESEND_H_ */
