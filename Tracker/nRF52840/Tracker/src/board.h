/**-------------------------------------------------------------------------
@file	board.h

@brief	Board specific definitions

This file contains all I/O definitions for a specific board for the
application firmware.  This files should be located in each project and
modified to suit the need for the application use case.

@author	Hoang Nguyen Hoan
@date	Nov. 16, 2016

@license

Copyright (c) 2016, I-SYST inc., all rights reserved

Permission to use, copy, modify, and distribute this software for any purpose
with or without fee is hereby granted, provided that the above copyright
notice and this permission notice appear in all copies, and none of the
names : I-SYST or its contributors may be used to endorse or
promote products derived from this software without specific prior written
permission.

For info or contributing contact : hnhoan at i-syst dot com

THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

----------------------------------------------------------------------------*/

#ifndef __BOARD_H__
#define __BOARD_H__

#include "blyst840_boards.h"

#define BOARD	IBK_NRF52840

#define BUT_INT_NUMER		0
#define BUT_INT_PRIO		6

#define IMU_INT_NO			1
#define IMU_INT_PRIO		6


#if BOARD == IBK_NRF52840

#define BUT_PINS			IBK_NRF52840_BUT_PINS_CFG
#define BUT_PIN_SENSE		IBK_NRF52840_BUT1_SENSE


// LEDs - Does not have RGB led
#define LED_RED_PORT		IBK_NRF52840_LED1_PORT
#define LED_RED_PIN			IBK_NRF52840_LED1_PIN
#define LED_RED_PINOP		IBK_NRF52840_LED1_PIOP
#define LED_RED_LOGIC		IBK_NRF52840_LED1_LOGIC

#define LED_GREEN_PORT		IBK_NRF52840_LED2_PORT
#define LED_GREEN_PIN		IBK_NRF52840_LED2_PIN
#define LED_GREEN_PINOP		IBK_NRF52840_LED2_PINOP
#define LED_GREEN_LOGIC		IBK_NRF52840_LED2_LOGIC

#define LED_BLUE_PORT		IBK_NRF52840_LED3_PORT
#define LED_BLUE_PIN		IBK_NRF52840_LED3_PIN
#define LED_BLUE_PINOP		IBK_NRF52840_LED3_PINOP
#define LED_BLUE_LOGIC		IBK_NRF52840_LED3_LOGIC

#define LED_PINS {\
	{LED_RED_PORT, LED_RED_PIN, LED_RED_PINOP, IOPINDIR_OUTPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL}, \
	{LED_GREEN_PORT, LED_GREEN_PIN, LED_GREEN_PINOP, IOPINDIR_OUTPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL}, \
	{LED_BLUE_PORT, LED_BLUE_PIN, LED_BLUE_PINOP, IOPINDIR_OUTPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL}, \
}

// UARTs for debug
#define UART_DEVNO			0
#define UART_RX_PORT		0
#define UART_RX_PIN			8
#define UART_RX_PINOP		1

#define UART_TX_PORT		0
#define UART_TX_PIN			7
#define UART_TX_PINOP		1

#define UART_PINS  		{\
	{UART_RX_PORT, UART_RX_PIN, UART_RX_PINOP, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL},\
	{UART_TX_PORT, UART_TX_PIN, UART_TX_PINOP, IOPINDIR_OUTPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL},\
}

#define I2C_DEVNO			0

#define I2C_SDA_PORT		0
#define I2C_SDA_PIN			4
#define I2C_SDA_PINOP		1

#define I2C_SCL_PORT		0
#define I2C_SCL_PIN			3
#define I2C_SCL_PINOP		1

#define I2C_PINS {\
	{I2C_SDA_PORT, I2C_SDA_PIN, I2C_SDA_PINOP, IOPINDIR_BI, IOPINRES_PULLUP, IOPINTYPE_OPENDRAIN}, \
	{I2C_SCL_PORT, I2C_SCL_PIN, I2C_SCL_PINOP, IOPINDIR_OUTPUT, IOPINRES_PULLUP, IOPINTYPE_OPENDRAIN},\
};

// SPI Bus
#define SPI_DEVNO			2

#define SPI_SCK_PORT		0
#define SPI_SCK_PIN			17
#define SPI_SCK_PINOP		1
// DIN
#define SPI_MISO_PORT		0
#define SPI_MISO_PIN		15
#define SPI_MISO_PINOP		1
// DOUT
#define SPI_MOSI_PORT		0
#define SPI_MOSI_PIN		16
#define SPI_MOSI_PINOP		1

// IMU
#define IMU_CS_PORT			0
#define IMU_CS_PIN			5
#define IMU_CS_PINOP		0

#define IMU_INT_PORT		0
#define IMU_INT_PIN			6
#define IMU_INT_PINOP		0

#define SPI_PINS {\
    {SPI_SCK_PORT, SPI_SCK_PIN, SPI_SCK_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},\
    {SPI_MISO_PORT, SPI_MISO_PIN, SPI_MISO_PINOP, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL},\
    {SPI_MOSI_PORT, SPI_MOSI_PIN, SPI_MOSI_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},\
    {IMU_CS_PORT, IMU_CS_PIN, IMU_CS_PINOP, IOPINDIR_OUTPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL},\
};

#else

#error "Undefined board"

#endif // BOARD == ???

#endif // __BOARD_H__

