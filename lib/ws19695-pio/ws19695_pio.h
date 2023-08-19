/**
 * @file ws19695_pio.h
 * @author Jaroslav Groman
 * @brief
 * @version 0.1
 * @date 2023-08-17
 *
 * @copyright Copyright (c) 2023
 *
 */
#ifndef WS19695_PIO_H
#define WS19695_PIO_H

/// @def USE_U8G2
/// @brief Define if you want to include u8g2 library
#define USE_U8G2

/*******************************************************************************
 *   Included Headers
 *******************************************************************************/
#include <stdbool.h>
#include <stdint.h>

#ifdef USE_U8G2
#include "./lib/u8g2/csrc/u8g2.h"
#endif

/*******************************************************************************
 *   Macros and #define Constants
 *******************************************************************************/

#define WS19695_DISPLAY_WIDTH 22
#define WS19695_DISPLAY_HEIGHT 7

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

/*******************************************************************************
 *   Global Variables and Constant Declarations with Applicable Initializations
 *******************************************************************************/

#ifdef USE_U8G2
extern u8g2_t u8g2;
#endif

/*******************************************************************************
 *   Function Declarations
 *******************************************************************************/

void ws19695_pio_init();

void ws19695_pio_set_buffer(uint8_t *p_data);

void ws19695_pio_get_buffer(uint8_t *p_data);

void ws19695_pio_set_pixel(uint8_t x, uint8_t y, bool state);

bool ws19695_pio_get_pixel(uint8_t x, uint8_t y);

void ws19695_pio_set_led(uint8_t led_id, bool state);

bool ws19695_pio_get_led(uint8_t led_id);

#endif /* #ifndef WS19695_PIO_H */

/* [] END OF FILE */
