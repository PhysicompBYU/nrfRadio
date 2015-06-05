/*
 * io_buffer.c
 *
 *  Created on: May 28, 2015
 *      Author: bsnga
 */

#include <msp430.h>
#include "io_buffer.h"
#include <stdint.h>
#include <stdlib.h>

#include "interrupts.h"

static const uint16_t sizes[10] = { 1, 2, 4, 8, 16, 32, 64, 128, 256, 512 };
static const uint16_t mods[10] = { ~1, ~2, ~4, ~8, ~16, ~32, ~64, ~128, ~256,
		~512 };

// Private functions
void io_error();

IO_buffer* create_buffer(uint8_t size) {
	IO_buffer* new_iobuf = (IO_buffer*) malloc(sizeof(IO_buffer));
	if (!new_iobuf)
		io_error(0);

	new_iobuf->buffer = (uint8_t*) malloc(sizes[size]);
	if (!new_iobuf->buffer)
		io_error(1);

	new_iobuf->size = size;
	new_iobuf->tail = 0;
	new_iobuf->count = 0;

	return new_iobuf;

}

void destroy_buffer(IO_buffer* buf) {
	free(buf->buffer);
	free(buf);
	return;
}

// Returns a 0 if buffer is full
void io_put_char(IO_buffer* buf, uint8_t data) {
	if (!buf)
		io_error(2);

	uint8_t head = (buf->tail + buf->count) & (mods[buf->size]);
	buf->buffer[head] = data;
	buf->count++;
	if (buf->count > sizes[buf->size]) {
		buf->count = sizes[buf->size];
		buf->tail = (buf->tail + 1) & mods[buf->size];
	}

	return;
}

// returns 0 if no data in buffer
uint8_t io_get_char(IO_buffer* buf, uint8_t* data) {
	if (!buf)
		io_error(3);

	if (!buf->count)
		return 0;

	*data = buf->buffer[buf->tail];
	buf->tail = (buf->tail + 1) & (mods[buf->size]);
	buf->count--;

	return 1;
}

void io_error(volatile int root) {
	while (1) {
		P1OUT &= ~(RLED + GLED);
		while (1) {
			delay(500);
			P1OUT ^= RLED + GLED;
		}

	}
}
