/*
 * minihdlc.h
 *
 *  Created on: 1 de dez de 2019
 *      Author: Gustavo
 */

#ifndef MINIHDLC_H_
#define MINIHDLC_H_

#include <stdint.h>
#include <stdbool.h>

typedef void (*sendchar_type)(uint8_t data); // Função p/ enviar um char // HAL_papapaa etc.
typedef void (*frame_handler_type)(const uint8_t *frame_buffer,
		uint16_t frame_length);

#define MINIHDLC_MAX_FRAME_LENGTH 64

void minihdlc_init(sendchar_type sendchar_function,
		frame_handler_type frame_handler_function);
void minihdlc_char_receiver(uint8_t data);
void minihdlc_send_frame(const uint8_t *frame_buffer, uint8_t frame_length);

void minihdlc_send_frame_to_buffer(const uint8_t *frame_buffer, uint8_t frame_length);
uint8_t *minihdlc_get_buffer();
uint32_t minihdlc_get_buffer_size();

#endif /* MINIHDLC_H_ */
