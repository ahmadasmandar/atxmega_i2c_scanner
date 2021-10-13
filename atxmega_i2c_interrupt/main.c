/*
 * atxmega_i2c_interrupt.c
 *
 * Created: 12.10.2021 22:02:57
 * Author : ahmad
 */

#define F_CPU 32000000UL

#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdio.h>
#include <util/delay.h>


#include "Clock.h"
#include "TWI.h"

#include "serial_com.h"



#define TWI_DEVICE_ADDRESS 0x00

#define TARGET_REGISTER 0x00
#define BYTES_TO_SEND   0x02
//#define  TWI_BUFFER_SIZE						0x08
volatile uint8_t I2C_WriteBuffer[2];
volatile uint8_t I2C_ReadBuffer[TWI_BUFFER_SIZE];
uint8_t I2C_SlaveBuffer[TWI_BUFFER_SIZE];

TWI_MasterStatus_t checkCRC(uint8_t DeviceAddress) {
	TWIM_Transmit(DeviceAddress, 0x90, 0x00, 0x00);
	while (!((TWIM_Status() == TWI_MASTER_SEND) || (TWIM_Status() == TWI_MASTER_ERROR)))
		;
	return TWIM_Status();
}

TWI_MasterStatus_t TWIM_Send(uint8_t DeviceAddress, uint8_t Register, uint8_t Bytes, uint8_t* Data) {
	TWIM_Transmit(DeviceAddress, Register, Bytes, Data);
	while (!((TWIM_Status() == TWI_MASTER_SEND) || (TWIM_Status() == TWI_MASTER_ERROR)))
		;
	return TWIM_Status();
}

int main(void) {
	Clock_Init();
	PCCommunication_Init_DF(57600);
	volatile uint8_t dummy = 0;

	TWIM_InitInterrupt();

	volatile TWI_MasterStatus_t statue_check;
	volatile uint8_t adress_found[8];
	PMIC.CTRL = PMIC_LOLVLEN_bm;
	sei();
	//PCCommunication_Init();

	while (1) {
		//try to build scanner function:
		uint8_t i_d = 0;
		for (uint8_t i = 0; i < 127; i++) {
			statue_check = TWIM_Send(i, 0x00, 0x01, 0x00);
			_delay_ms(10);
			if (statue_check != TWI_MASTER_ERROR) {
				adress_found[i_d] = i;
				i_d++;
			}
		}
		for (uint8_t add=0; add < 8; add++) {

			if (adress_found[add]>0) {
				UART_Transmit_string("the first Device that found : ");
				PCCommunication_SendByte('x');
				printHexByte(adress_found[add]);
				PCCommunication_SendByte('\n');
				PCCommunication_SendByte('\r');
				_delay_ms(50);

			}

		}
		dummy++;
		_delay_ms(1500);
		//I2C_ReadBuffer[7]++;
	}
}


