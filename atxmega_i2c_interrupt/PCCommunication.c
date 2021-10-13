/*
 * PCCommunication.c
 *
 *  Copyright (C) Vietzke Engineering , Ahmad Asmandar, 2020
 *  Kompass GmbH
 *	Website: https://www.kompass-sensor.com/
 *
 *  File info: PCCommunication Lib
 * */
/** @file PCCommunication.c
 *  @brief PCCommunication USART DRIVER.
 *  @author Vietzke Engineering ,Ahmad Asmandar
 */

#include "PCCommunication.h"

#include <avr/interrupt.h>
#include <avr/io.h>
#include <math.h>
#include <util/delay.h>

/*
ISR(USARTC0_RXC_vect) {
  uint8_t Data = USARTC0.DATA;
  RingBuffer_Save(&Buffer, Data);
  USARTC0.DATA = Data;

  if (RingBuffer_IsFull(&Buffer)) {
    while (!(USARTC0.STATUS & USART_DREIF_bm))
      ;
    USARTC0.DATA = 0x0D;
    while (!(USARTC0.STATUS & USART_DREIF_bm))
      ;
    USARTC0.DATA = 0x0A;

    for (uint8_t i = 0; i < BUFFER_SIZE_USART; i++) {
      while (!(USARTC0.STATUS & USART_DREIF_bm))
        ;
      USARTC0.DATA = RingBuffer_Load(&Buffer);
    }
    while (!(USARTC0.STATUS & USART_DREIF_bm))
      ;
    USARTC0.DATA = 0x0D;
    while (!(USARTC0.STATUS & USART_DREIF_bm))
      ;
    USARTC0.DATA = 0x0A;
  }
  //OnFly ^= 1;
}
*/
// ISR(USARTC0_RXC_vect)
// {
// 	uint8_t data = USARTC0.DATA;

// 	PCCommunication_SendByte(data);
// }

/**/

uint8_t PCCommunication_Init(uint32_t baudrate)
{
    PORTC.DIRSET = PIN3_bm;
    PORTC.DIRCLR = PIN2_bm;

    int8_t exp;
    uint32_t div;
    uint32_t limit;
    uint32_t ratio;
    uint32_t min_rate;
    uint32_t max_rate;

    uint32_t cpu_hz = F_CPU;
    uint32_t baud   = baudrate;

    /*
	 * Check if the hardware supports the given baud rate
	 */
    /* 8 = (2^0) * 8 * (2^0) = (2^BSCALE_MIN) * 8 * (BSEL_MIN) */
    max_rate = cpu_hz / 8;
    /* 4194304 = (2^7) * 8 * (2^12) = (2^BSCALE_MAX) * 8 * (BSEL_MAX+1) */
    min_rate = cpu_hz / 4194304;

    /* double speed is disabled. */
    max_rate /= 2;
    min_rate /= 2;

    if ((baud > max_rate) || (baud < min_rate))
    {
        /* the hardware doesn't supports the given baud rate */
        return -1;
    }

    /* double speed is disabled. */
    baud *= 2;

    /* Find the lowest possible exponent. */
    limit = 0xfffU >> 4;
    ratio = cpu_hz / baud;

    for (exp = -7; exp < 7; exp++)
    {
        if (ratio < limit)
        {
            break;
        }

        limit <<= 1;

        if (exp < -3)
        {
            limit |= 1;
        }
    }

    if (exp < 0)
    {
        cpu_hz -= 8 * baud;

        if (exp <= -3)
        {
            div = ((cpu_hz << (-exp - 3)) + baud / 2) / baud;
        }
        else
        {
            baud <<= exp + 3;
            div = (cpu_hz + baud / 2) / baud;
        }
    }
    else
    {
        baud <<= exp + 3;
        div = (cpu_hz + baud / 2) / baud - 1;
    }

    USARTC0.BAUDCTRLB = (uint8_t)(((div >> 8) & 0X0F) | (exp << 4));
    USARTC0.BAUDCTRLA = (uint8_t)div;
    USARTC0.CTRLA     = USART_RXCINTLVL_LO_gc;
    USARTC0.STATUS |= USART_RXCIF_bm;
    USARTC0.CTRLC = USART_CHSIZE_8BIT_gc;
    USARTC0.CTRLC &= ~(USART_PMODE0_bm | USART_PMODE1_bm | USART_SBMODE_bm);

    USARTC0.CTRLB = 0 << USART_ONEWIRE_bp /* One Wire Mode: disabled */
                    | 0 << USART_SFDEN_bp /* Start Frame Detection Enable: disabled */
                    | 0 << USART_CLK2X_bp /* Double transmission speed: disabled */
                    | 0 << USART_MPCM_bp  /* Multi-processor Communication Mode: disabled */
                    | 1 << USART_RXEN_bp  /* Receiver Enable: enabled */
                    | 1 << USART_TXEN_bp; /* Transmitter Enable: enabled */
    return 0;
}

// void PCCommunication_Init() {
//   PORTC.DIRSET = PIN3_bm;

//   // 9600
//   USARTC0.BAUDCTRLA = (3317 & 0xff) << USART_BSEL_gp;
//   USARTC0.BAUDCTRLB = ((-4) << USART_BSCALE_gp) | ((3317 >> 8) << USART_BSEL_gp);

//   USARTC0.CTRLA = USART_RXCINTLVL_LO_gc;
//   USARTC0.STATUS |= USART_RXCIF_bm;

//   USARTC0.CTRLB = USART_RXEN_bm | USART_TXEN_bm;
// }

void PCCommunication_SendByte(uint8_t data)
{
    while (!(USARTC0.STATUS & USART_DREIF_bm))
        ;
    USARTC0.DATA = data;
}

void PCCommunication_SendNumber(uint32_t value)
{
    uint8_t sendData    = 0;
    uint8_t leadingZero = 1;

    uint32_t divider = 10000000;

    while (1)
    {
        sendData = 0x30 + (value / divider) % 10;
        if (sendData == 0x30 && leadingZero == 1 && divider != 1)
            PCCommunication_SendByte(' ');
        //continue;
        else
        {
            PCCommunication_SendByte(sendData);
            leadingZero = 0;
        }

        divider = divider / 10;

        if (divider == 0)
            break;
    }
}

void PCCommunication_SendTime(uint8_t value)
{
    uint8_t sendData    = 0;
    uint8_t leadingZero = 1;

    uint32_t divider = 1000;

    while (1)
    {
        sendData = 0x30 + (value / divider) % 10;
        if (sendData == 0x30 && leadingZero == 1 && divider != 1)
            PCCommunication_SendByte(' ');
        //continue;
        else
        {
            PCCommunication_SendByte(sendData);
            leadingZero = 0;
        }

        divider = divider / 10;

        if (divider == 0)
            break;
    }
}

uint8_t receiveByte(void)
{
    while (!(USARTC0.STATUS & USART_DREIF_bm))
        ;                /* Wait for incoming data */
    return USARTC0.DATA; /* return register value */
}

/* Here are a bunch of useful printing commands */

void printString(const char myString[])
{
    uint8_t i = 0;
    while (myString[i])
    {
        PCCommunication_SendByte(myString[i]);
        i++;
        _delay_ms(1);
    }
    PCCommunication_SendByte('\r');
    PCCommunication_SendByte('\n');
}

// void readString(const char myString[], uint8_t maxLength)
// {
// 	char response;
// 	uint8_t i;
// 	i = 0;
// 	while (i < (maxLength - 1))
// 	{ /* prevent over-runs */
// 		response = receiveByte();
// 		PCCommunication_SendByte(response); /* echo */
// 		if (response == '\r')
// 		{ /* enter marks the end */
// 			break;
// 		}
// 		else
// 		{
// 			myString[i] = response; /* add in a letter */
// 			i++;
// 		}
// 	}
// 	myString[i] = 0; /* terminal NULL character */
// }

void printByte(uint8_t byte)
{
    /* Converts a byte to a string of decimal text, sends it */
    PCCommunication_SendByte('0' + (byte / 100));       /* Hundreds */
    PCCommunication_SendByte('0' + ((byte / 10) % 10)); /* Tens */
    PCCommunication_SendByte('0' + (byte % 10));        /* Ones */
    PCCommunication_SendByte('\r');
    PCCommunication_SendByte('\n');
}

void printWord(uint16_t word)
{
    PCCommunication_SendByte('0' + (word / 10000));       /* Ten-thousands */
    PCCommunication_SendByte('0' + ((word / 1000) % 10)); /* Thousands */
    PCCommunication_SendByte('0' + ((word / 100) % 10));  /* Hundreds */
    PCCommunication_SendByte('0' + ((word / 10) % 10));   /* Tens */
    PCCommunication_SendByte('0' + (word % 10));          /* Ones */
}

void printBinaryByte(uint8_t byte)
{
    /* Prints out a byte as a series of 1's and 0's */
    uint8_t bit;
    for (bit = 7; bit < 255; bit--)
    {
        if (bit_is_set(byte, bit))
            PCCommunication_SendByte('1');
        else
            PCCommunication_SendByte('0');
    }
}

char nibbleToHexCharacter(uint8_t nibble)
{
    /* Converts 4 bits into hexadecimal */
    if (nibble < 10)
    {
        return ('0' + nibble);
    }
    else
    {
        return ('A' + nibble - 10);
    }
}

void printHexByte(uint8_t byte)
{
    /* Prints a byte as its hexadecimal equivalent */
    uint8_t nibble;
    nibble = (byte & 0b11110000) >> 4;
    PCCommunication_SendByte(nibbleToHexCharacter(nibble));
    nibble = byte & 0b00001111;
    PCCommunication_SendByte(nibbleToHexCharacter(nibble));
}

uint8_t getNumber(void)
{
    // Gets a numerical 0-255 from the serial port.
    // Converts from string to number.
    char hundreds = '0';
    char tens     = '0';
    char ones     = '0';
    char thisChar = '0';
    do
    { /* shift over */
        hundreds = tens;
        tens     = ones;
        ones     = thisChar;
        thisChar = receiveByte();           /* get a new character */
        PCCommunication_SendByte(thisChar); /* echo */
    } while (thisChar != '\r');             /* until type return */
    return (100 * (hundreds - '0') + 10 * (tens - '0') + ones - '0');
}

void RingBuffer_Init(RingBuffer_t *Buffer, uint8_t *Data, const uint16_t Size)
{
    Buffer->InPtr     = Data;
    Buffer->OutPtr    = Data;
    Buffer->RingStart = &Data[0];
    Buffer->RingEnd   = &Data[Size];
    Buffer->Size      = Size;
    Buffer->ByteCount = 0;
}

uint8_t RingBuffer_IsFull(const RingBuffer_t *Buffer)
{
    return (Buffer->ByteCount == Buffer->Size);
}

uint8_t RingBuffer_Load(RingBuffer_t *Buffer)
{
    uint8_t Data = *Buffer->OutPtr;
    if (++Buffer->OutPtr == Buffer->RingEnd)
    {
        Buffer->OutPtr = Buffer->RingStart;
    }
    Buffer->ByteCount--;
    return Data;
}

void RingBuffer_Save(RingBuffer_t *Buffer, const uint8_t Data)
{
    *Buffer->InPtr = Data;
    if (++Buffer->InPtr == Buffer->RingEnd)
    {
        Buffer->InPtr = Buffer->RingStart;
    }
    Buffer->ByteCount++;
}

void UART_Transmit_string(char string[])
{
    int i = 0;
    while (string[i] > 0)
        PCCommunication_SendByte(string[i++]);
}