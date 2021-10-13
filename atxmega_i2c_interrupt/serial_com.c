#include "serial_com.h"
#include <avr/io.h>
#include <math.h>
#include <util/delay.h>
//
void PCCommunication_Init() {
	PORTC.DIRSET = PIN3_bm;

	// 9600
	USARTC0.BAUDCTRLA = (3317 & 0xff) << USART_BSEL_gp;
	USARTC0.BAUDCTRLB = ((-4) << USART_BSCALE_gp) | ((3317 >> 8) << USART_BSEL_gp);

	USARTC0.CTRLA = USART_RXCINTLVL_LO_gc;
	USARTC0.STATUS |= USART_RXCIF_bm;

	USARTC0.CTRLB = USART_RXEN_bm | USART_TXEN_bm;
}

void PCCommunication_SendByte(uint8_t data) {
	while (!(USARTC0.STATUS & USART_DREIF_bm))
		;
	USARTC0.DATA = data;
}

void PCCommunication_SendNumber(uint32_t value) {
	uint8_t sendData    = 0;
	uint8_t leadingZero = 1;

	uint32_t divider = 10000000;

	while (1) {
		sendData = 0x30 + (value / divider) % 10;
		if (sendData == 0x30 && leadingZero == 1 && divider != 1)
			PCCommunication_SendByte(' ');
		//continue;
		else {
			PCCommunication_SendByte(sendData);
			leadingZero = 0;
		}

		divider = divider / 10;

		if (divider == 0)
			break;
	}
}

void printHexByte(uint8_t byte) {
	/* Prints a byte as its hexadecimal equivalent */
	uint8_t nibble;
	nibble = (byte & 0b11110000) >> 4;
	PCCommunication_SendByte(nibbleToHexCharacter(nibble));
	nibble = byte & 0b00001111;
	PCCommunication_SendByte(nibbleToHexCharacter(nibble));
}

void UART_Transmit_string(char string[]) {
	int i = 0;
	while (string[i] > 0)
		PCCommunication_SendByte(string[i++]);
}

char nibbleToHexCharacter(uint8_t nibble) {
	/* Converts 4 bits into hexadecimal */
	if (nibble < 10) {
		return ('0' + nibble);
	} else {
		return ('7' + nibble);
	}
}

void PCCommunication_Init_DF(uint16_t baudrate) {
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

	if ((baud > max_rate) || (baud < min_rate)) {
		/* the hardware doesn't supports the given baud rate */
		return -1;
	}

	/* double speed is disabled. */
	baud *= 2;

	/* Find the lowest possible exponent. */
	limit = 0xfffU >> 4;
	ratio = cpu_hz / baud;

	for (exp = -7; exp < 7; exp++) {
		if (ratio < limit) {
			break;
		}

		limit <<= 1;

		if (exp < -3) {
			limit |= 1;
		}
	}

	if (exp < 0) {
		cpu_hz -= 8 * baud;

		if (exp <= -3) {
			div = ((cpu_hz << (-exp - 3)) + baud / 2) / baud;
		} else {
			baud <<= exp + 3;
			div = (cpu_hz + baud / 2) / baud;
		}
	} else {
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
//

