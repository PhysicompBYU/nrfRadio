/*
 * io_buffer.h
 *
 *  Created on: May 28, 2015
 *      Author: bsnga
 */

#ifndef IO_BUFFER_H_
#define IO_BUFFER_H_

#include <stdint.h>

typedef struct {
	uint8_t* buffer;
	uint16_t tail;
	uint16_t count;
	uint8_t size;
} IO_buffer;

// Functions
IO_buffer* create_buffer(uint8_t size);
void io_put_char(IO_buffer* buf, uint8_t data);
uint8_t io_get_char(IO_buffer* buf, uint8_t* data);
void destroy_buffer(IO_buffer* buf);

#endif /* IO_BUFFER_H_ */
