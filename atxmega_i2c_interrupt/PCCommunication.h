
#include <avr/io.h>
#include <math.h>
#include <util/delay.h>

#ifndef _PCCOMMUNICATION_H
#define _PCCOMMUNICATION_H

#define BUFFER_SIZE_USART 64

typedef struct
{
    uint8_t *InPtr;
    uint8_t *OutPtr;
    uint8_t *RingStart;
    uint8_t *RingEnd;
    uint16_t Size;
    uint16_t ByteCount;
} RingBuffer_t;

extern uint8_t PCCommunication_Init(uint32_t baudrate);
extern void PCCommunication_SendByte(uint8_t data);
extern void PCCommunication_SendNumber(uint32_t value);
extern void PCCommunication_SendTime(uint8_t value);
void printString(const char myString[]);
/* Utility function to transmit an entire string from RAM */
uint8_t receiveByte(void);

void readString(char myString[], uint8_t maxLength);
/* Define a string variable, pass it to this function
   The string will contain whatever you typed over serial */

void printByte(uint8_t byte);
/* Prints a byte out as its 3-digit ascii equivalent */
void printWord(uint16_t word);
/* Prints a word (16-bits) out as its 5-digit ascii equivalent */

void printBinaryByte(uint8_t byte);
/* Prints a byte out in 1s and 0s */
char nibbleToHex(uint8_t nibble);
char nibbleToHexCharacter(uint8_t nibble);
/* Prints a byte out in hexadecimal */
void printHexByte(uint8_t byte);

void RingBuffer_Init(RingBuffer_t *Buffer, uint8_t *Data, const uint16_t Size);
uint8_t RingBuffer_IsFull(const RingBuffer_t *Buffer);
uint8_t RingBuffer_Load(RingBuffer_t *Buffer);
void RingBuffer_Save(RingBuffer_t *Buffer, const uint8_t Data);
void UART_Transmit_string(char string[]);

#endif //_PCCOMMUNICATION_H
