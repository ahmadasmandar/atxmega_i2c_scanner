/*
 * serial_com.h
 *
 * Created: 13.10.2021 21:41:14
 *  Author: ahmad
 */

#ifndef SERIAL_COM_H_
#define SERIAL_COM_H_

#define F_CPU 32000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <math.h>
#include <util/delay.h>

void PCCommunication_Init();
void PCCommunication_Init_DF(uint16_t baudrate);
void PCCommunication_SendByte(uint8_t data);

void PCCommunication_SendNumber(uint32_t value);
void printHexByte(uint8_t byte);
void UART_Transmit_string(char string[]);
char nibbleToHexCharacter(uint8_t nibble);
#endif /* SERIAL_COM_H_ */