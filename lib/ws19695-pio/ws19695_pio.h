#ifndef WS19695_PIO_H
#define WS19695_PIO_H

#include <stdbool.h>
#include <stdint.h>

#define WS19695_LED_MOVE_ON 0
#define WS19695_LED_ALARM_ON 1
#define WS19695_LED_COUNT_DOWN 2
#define WS19695_LED_COUNT_UP 3
#define WS19695_LED_DEG_F 4
#define WS19695_LED_DEG_C 5
#define WS19695_LED_AM 6
#define WS19695_LED_PM 7
#define WS19695_LED_HOURLY 8
#define WS19695_LED_AUTOLIGHT 9
#define WS19695_LED_DAY_MON 10
#define WS19695_LED_DAY_TUE 11
#define WS19695_LED_DAY_WED 12
#define WS19695_LED_DAY_THU 13
#define WS19695_LED_DAY_FRI 14
#define WS19695_LED_DAY_SAT 15
#define WS19695_LED_DAY_SUN 16

void ws19695_pio_init();

void ws19695_pio_set_buffer(uint8_t *data);

void ws19695_pio_set_led(uint8_t led_id, bool state);

#endif /* #ifndef WS19695_PIO_H */
