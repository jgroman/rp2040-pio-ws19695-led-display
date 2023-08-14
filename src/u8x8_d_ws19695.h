#ifndef U8X8_D_WS19695_H
#define U8X8_D_WS19695_H

#include "../lib/u8g2/csrc/u8g2.h"

extern uint32_t buf_ws19695[8] __attribute__ ((aligned(32)));

uint8_t ws19695_byte_cb(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);

uint8_t ws19695_gpio_and_delay_cb(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);

void u8g2_Setup_ws19695_22x7_pio(u8g2_t *u8g2, const u8g2_cb_t *rotation, u8x8_msg_cb byte_cb, u8x8_msg_cb gpio_and_delay_cb);

#endif  // #ifndef U8X8_D_WS19695_H
