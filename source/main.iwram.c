/* 
 * Copyright (c) 2016-2021, Extrems' Corner.org
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdint.h>
#include <gba_dma.h>
#include <gba_input.h>
#include <gba_interrupt.h>
#include <gba_sio.h>
#include <gba_timers.h>
#include "bios.h"

#define struct struct __attribute__((packed, scalar_storage_order("little-endian")))

#define ROM           ((volatile int16_t *)0x08000000)
#define ROM_GPIODATA *((volatile int16_t *)0x080000C4)
#define ROM_GPIODIR  *((volatile int16_t *)0x080000C6)
#define ROM_GPIOCNT  *((volatile int16_t *)0x080000C8)
#define GPIO_IRQ	 0x0100

//#define ANALOG

enum {
	CMD_ID = 0x00,
	CMD_STATUS,
	CMD_READ,
	CMD_WRITE,
	CMD_GBREAD = 0x13,
	CMD_GBWRITE = 0x14,
	CMD_RESET = 0xFF,

	GBP_INIT = 0x01,
	GBP_PRINT = 0x02,
	GBP_COPY = 0x04,
	GBP_AFTER = 0x06,
	GBP_STOP = 0x08,
	GBP_NOP = 0x0F
};

static struct {
	uint16_t type;
	uint8_t status;
} id = {0x0300, 0x00};	/* 64GB Cable (Game Boy Printer Cartridge) */

static uint8_t buffer[128];

uint8_t crc8_lut[256] = {
	0x00, 0x85, 0x8F, 0x0A, 0x9B, 0x1E, 0x14, 0x91, 0xB3, 0x36, 0x3C, 0xB9, 0x28, 0xAD, 0xA7, 0x22,
	0xE3, 0x66, 0x6C, 0xE9, 0x78, 0xFD, 0xF7, 0x72, 0x50, 0xD5, 0xDF, 0x5A, 0xCB, 0x4E, 0x44, 0xC1,
	0x43, 0xC6, 0xCC, 0x49, 0xD8, 0x5D, 0x57, 0xD2, 0xF0, 0x75, 0x7F, 0xFA, 0x6B, 0xEE, 0xE4, 0x61,
	0xA0, 0x25, 0x2F, 0xAA, 0x3B, 0xBE, 0xB4, 0x31, 0x13, 0x96, 0x9C, 0x19, 0x88, 0x0D, 0x07, 0x82,
	0x86, 0x03, 0x09, 0x8C, 0x1D, 0x98, 0x92, 0x17, 0x35, 0xB0, 0xBA, 0x3F, 0xAE, 0x2B, 0x21, 0xA4,
	0x65, 0xE0, 0xEA, 0x6F, 0xFE, 0x7B, 0x71, 0xF4, 0xD6, 0x53, 0x59, 0xDC, 0x4D, 0xC8, 0xC2, 0x47,
	0xC5, 0x40, 0x4A, 0xCF, 0x5E, 0xDB, 0xD1, 0x54, 0x76, 0xF3, 0xF9, 0x7C, 0xED, 0x68, 0x62, 0xE7,
	0x26, 0xA3, 0xA9, 0x2C, 0xBD, 0x38, 0x32, 0xB7, 0x95, 0x10, 0x1A, 0x9F, 0x0E, 0x8B, 0x81, 0x04,
	0x89, 0x0C, 0x06, 0x83, 0x12, 0x97, 0x9D, 0x18, 0x3A, 0xBF, 0xB5, 0x30, 0xA1, 0x24, 0x2E, 0xAB,
	0x6A, 0xEF, 0xE5, 0x60, 0xF1, 0x74, 0x7E, 0xFB, 0xD9, 0x5C, 0x56, 0xD3, 0x42, 0xC7, 0xCD, 0x48,
	0xCA, 0x4F, 0x45, 0xC0, 0x51, 0xD4, 0xDE, 0x5B, 0x79, 0xFC, 0xF6, 0x73, 0xE2, 0x67, 0x6D, 0xE8,
	0x29, 0xAC, 0xA6, 0x23, 0xB2, 0x37, 0x3D, 0xB8, 0x9A, 0x1F, 0x15, 0x90, 0x01, 0x84, 0x8E, 0x0B,
	0x0F, 0x8A, 0x80, 0x05, 0x94, 0x11, 0x1B, 0x9E, 0xBC, 0x39, 0x33, 0xB6, 0x27, 0xA2, 0xA8, 0x2D,
	0xEC, 0x69, 0x63, 0xE6, 0x77, 0xF2, 0xF8, 0x7D, 0x5F, 0xDA, 0xD0, 0x55, 0xC4, 0x41, 0x4B, 0xCE,
	0x4C, 0xC9, 0xC3, 0x46, 0xD7, 0x52, 0x58, 0xDD, 0xFF, 0x7A, 0x70, 0xF5, 0x64, 0xE1, 0xEB, 0x6E,
	0xAF, 0x2A, 0x20, 0xA5, 0x34, 0xB1, 0xBB, 0x3E, 0x1C, 0x99, 0x93, 0x16, 0x87, 0x02, 0x08, 0x8D,
};

static uint8_t crc8(uint8_t* ptr) {
	uint8_t crc = 0;

	for (int i = 0; i < 32; i++) {
		crc ^= *ptr++;
		crc  = crc8_lut[crc];
	}

	return crc;
}

void SISetResponse(const void *buf, unsigned bits);
int SIGetCommand(void *buf, unsigned bits);

#define LINK_SEND 0x3
#define LINK_REP_C0 0x30
#define LINK_REP_F0 0x30+0x21

int main(void)
{
	RegisterRamReset(RESET_ALL_REG);

	REG_IE = IRQ_SERIAL | IRQ_TIMER1 | IRQ_TIMER0;
	REG_IF = REG_IF;

	REG_RCNT = R_GPIO | GPIO_IRQ | GPIO_SO_IO | GPIO_SO;

	REG_TM0CNT_L = -67;
	REG_TM1CNT_H = TIMER_START | TIMER_IRQ | TIMER_COUNT;
	REG_TM0CNT_H = TIMER_START;

	SoundBias(0);
	Halt();

	while (true) {
		int length = SIGetCommand(buffer, sizeof(buffer) * 8 + 1);
		if (length < 9) continue;

		switch (buffer[0]) {
			case CMD_RESET:
			case CMD_ID:
				if (length == 9) SISetResponse(&id, sizeof(id) * 8);
				break;
			case CMD_GBREAD:
				/* RAW: 03 21 - 13 CC LL */
				if (length == 25) {
					/* 0x21 bytes to send */
					if (buffer[1] == 0xC0)
					{
						/* Communication Status */
						SISetResponse(&buffer[LINK_REP_C0], 264);
					}
					else if (buffer[1] == 0xF0)
					{
						/* Game Boy Link Communication Recv */
						SISetResponse(&buffer[LINK_REP_F0], 264);
					}
				}
				break;
			case CMD_GBWRITE:
				/* RAW: 23 01 - 14 CC LL */
				if (length == 281) {
					/* 0x01 byte to send */
					buffer[LINK_SEND + 0x20] = crc8(&buffer[LINK_SEND]);
					SISetResponse(&buffer[LINK_SEND + 0x20], 8);

					/* Prepare Data Response */
					if (buffer[1] == 0x80) {
						/* Initialization Command */
						for (int i = 0; i < sizeof(buffer); i++)
							buffer[i] = 0;
					}
					else if (buffer[1] == 0xC0) {
						/* Communication Configuration Command */
						buffer[LINK_REP_C0] = 0x02;
						buffer[LINK_REP_C0 + 0x20] = crc8(&buffer[LINK_REP_C0]);
					}
					else if (buffer[1] == 0xE0) {
						/* Game Boy Link Communication Send */
						buffer[LINK_REP_F0 + 8] = 0x81;
						buffer[LINK_REP_F0 + 0x20] = crc8(&buffer[LINK_REP_F0]);
					}
				}
				break;
		}
	}
}
