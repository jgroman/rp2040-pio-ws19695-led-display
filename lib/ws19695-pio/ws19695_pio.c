/**
 * @file ws19695_pio.c
 * @author Jaroslav Groman
 * @brief
 * @version 0.1
 * @date 2023-08-17
 *
 * @copyright Copyright (c) 2023
 *
 */
#include <stdio.h>
#include <string.h>

#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"

#include "ws19695.pio.h" // Generated from ws19695.pio

#include "ws19695_pio.h"

/*
LED display organization
Row addr||     bits 31-24       ||  bits 23-16  ||   bits 15-8   || bits 7-0 ||
--------||----------------------||--------------||---------------||----------||
   0    || Move_On |x|Mon |x|Tue||x|Wed|x|Thu|x|Fri|x|Sat|x|Sun|x||          ||
   1    || Alm_On  | bits 5 - 0 ||  bits 7 - 0  ||   bits 7 - 0  ||    not   ||
   2    || CntDwn  | bits 5 - 0 ||  bits 7 - 0  ||   bits 7 - 0  ||          ||
   3    || F  | C  | bits 5 - 0 ||  bits 7 - 0  ||   bits 7 - 0  ||   used   ||
   4    || AM | PM | bits 5 - 0 ||  bits 7 - 0  ||   bits 7 - 0  ||          ||
   5    || CntUp   | bits 5 - 0 ||  bits 7 - 0  ||   bits 7 - 0  ||          ||
   6    || Hourly  | bits 5 - 0 ||  bits 7 - 0  ||   bits 7 - 0  ||          ||
   7    || AutoLgt | bits 5 - 0 ||  bits 7 - 0  ||   bits 7 - 0  ||          ||
*/

/*******************************************************************************
 * Macros and #define Constants
 *******************************************************************************/
#define PIN_PWM 27 // PWM controlled display brightness

/*******************************************************************************
 * Global variables
 *******************************************************************************/
#ifdef USE_U8G2
u8g2_t u8g2;
#endif

/*******************************************************************************
 * Variables
 *******************************************************************************/
static uint32_t buf_ws19695[8] __attribute__((aligned(32)));
static uint32_t p_buf_ws19695 = (uint32_t)(buf_ws19695);

#ifdef USE_U8G2
static uint8_t buf_transposed[32];
#endif

static uint dma_chan_transfer, dma_chan_control;
static dma_channel_config dma_cfg_transfer, dma_cfg_control;

/*******************************************************************************
 * Private function definitions
 *******************************************************************************/

static void ws19695_configure_pwm(uint pin_pwm, bool pwm_enabled) {
  gpio_init(pin_pwm);
  gpio_set_dir(pin_pwm, GPIO_OUT);

  if (!pwm_enabled) {
    gpio_put(pin_pwm, 1);
    return;
  }

  gpio_set_function(pin_pwm, GPIO_FUNC_PWM);
  // Find out which PWM slice is connected to GPIO pin_pwm
  uint slice_num = pwm_gpio_to_slice_num(pin_pwm);

  // Set period of 4 cycles (0 to 3 inclusive)
  pwm_set_wrap(slice_num, 3);
  pwm_set_clkdiv_int_frac(slice_num, 20, 0);

  // Set channel A output high for one cycle before dropping
  pwm_set_chan_level(slice_num, PWM_CHAN_A, 1);
  // Set initial B output high for three cycles before dropping
  pwm_set_chan_level(slice_num, PWM_CHAN_B, 1);

  // Set the PWM running
  pwm_set_enabled(slice_num, true);
}

static inline void ws19695_configure_dma() {
  // https://forums.raspberrypi.com/viewtopic.php?t=352892

  dma_chan_transfer = dma_claim_unused_channel(true);
  dma_chan_control = dma_claim_unused_channel(true);

  dma_cfg_transfer = dma_channel_get_default_config(dma_chan_transfer);
  channel_config_set_transfer_data_size(&dma_cfg_transfer, DMA_SIZE_32);
  channel_config_set_read_increment(&dma_cfg_transfer, true);
  channel_config_set_write_increment(&dma_cfg_transfer, false);
  channel_config_set_dreq(&dma_cfg_transfer, DREQ_PIO0_TX0);
  channel_config_set_irq_quiet(&dma_cfg_transfer, true);
  channel_config_set_chain_to(&dma_cfg_transfer, dma_chan_control);

  dma_channel_configure(dma_chan_transfer, &dma_cfg_transfer, &pio0_hw->txf[0],
                        &buf_ws19695, 8, false);

  dma_cfg_control = dma_channel_get_default_config(dma_chan_control);
  channel_config_set_transfer_data_size(&dma_cfg_control, DMA_SIZE_32);
  channel_config_set_read_increment(&dma_cfg_control, false);
  channel_config_set_write_increment(&dma_cfg_control, false);
  channel_config_set_irq_quiet(&dma_cfg_control, true);
  // No DREQ
  // No chain

  dma_channel_configure(
      dma_chan_control, &dma_cfg_control,
      // Writing to this register retriggers transfer channel
      &(dma_channel_hw_addr(dma_chan_transfer)->al3_read_addr_trig),
      &p_buf_ws19695, // pointer to pointer to buf_ws19695
      1,
      true // initial DMA trigger
  );

  return;
}

#ifdef USE_U8G2
/**
 * @brief Transpose bit matrix
 *
 * @par Description
 *    Parameter A is the address of the first byte of an 8x8 submatrix of
 * the source matrix, which is of size 8m x 8n bits. Similarly, parameter B is
 * the address of the first byte of an 8x8 submatrix in the target matrix,
 * which is of size 8n x 8m bits. That is, the full source matrix is 8m x n
 * bytes, and the full target matrix is 8n x m bytes.
 *
 * @par Source
 *    http://www.icodeguru.com/Embedded/Hacker's-Delight/048.htm
 */
static void transpose8(unsigned char *A, uint8_t m, uint8_t n,
                       unsigned char *B) {
  uint32_t x, y, t;

  // Load the array and pack it into x and y.

  x = (A[0] << 24) | (A[m] << 16) | (A[2 * m] << 8) | A[3 * m];
  y = (A[4 * m] << 24) | (A[5 * m] << 16) | (A[6 * m] << 8) | A[7 * m];

  t = (x ^ (x >> 7)) & 0x00AA00AA;
  x = x ^ t ^ (t << 7);
  t = (y ^ (y >> 7)) & 0x00AA00AA;
  y = y ^ t ^ (t << 7);

  t = (x ^ (x >> 14)) & 0x0000CCCC;
  x = x ^ t ^ (t << 14);
  t = (y ^ (y >> 14)) & 0x0000CCCC;
  y = y ^ t ^ (t << 14);

  t = (x & 0xF0F0F0F0) | ((y >> 4) & 0x0F0F0F0F);
  y = ((x << 4) & 0xF0F0F0F0) | (y & 0x0F0F0F0F);
  x = t;

  B[0] = x >> 24;
  B[n] = x >> 16;
  B[2 * n] = x >> 8;
  B[3 * n] = x;
  B[4 * n] = y >> 24;
  B[5 * n] = y >> 16;
  B[6 * n] = y >> 8;
  B[7 * n] = y;
}

static uint8_t ws19695_byte_cb(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int,
                        void *arg_ptr) {
  return 1; // All operations are successful
}

static uint8_t ws19695_gpio_and_delay_cb(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int,
                                  void *arg_ptr) {
  return 1; // All operations are successful
}

static const u8x8_display_info_t u8x8_ws19695_22x7_pio_display_info = {
    /* chip_enable_level = */ 0,
    /* chip_disable_level = */ 1,

    /* post_chip_enable_wait_ns = */ 0,
    /* pre_chip_disable_wait_ns = */ 0,
    /* reset_pulse_width_ms = */ 0,
    /* post_reset_wait_ms = */ 0,
    /* sda_setup_time_ns = */ 0,
    /* sck_pulse_width_ns = */ 0,
    /* sck_clock_hz = */ 8000000UL, /* since Arduino 1.6.0, the SPI bus speed in
                                       Hz. Should be
                                       1000000000/sck_pulse_width_ns */
    /* spi_mode = */ 0,             /* active high, rising edge */
    /* i2c_bus_clock_100kHz = */ 4,
    /* data_setup_time_ns = */ 0,
    /* write_pulse_width_ns = */ 0,
    /* tile_width = */ 3,
    /* tile_height = */ 1,
    /* default_x_offset = */ 0,
    /* flipmode_x_offset = */ 0,
    /* pixel_width = */ 22,
    /* pixel_height = */ 7};

static uint8_t u8x8_d_ws19695_22x7_pio(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int,
                                void *arg_ptr) {
  switch (msg) {
  case U8X8_MSG_DISPLAY_INIT:
    break;
  case U8X8_MSG_DISPLAY_SETUP_MEMORY:
    u8x8_d_helper_display_setup_memory(u8x8,
                                       &u8x8_ws19695_22x7_pio_display_info);
    break;

  case U8X8_MSG_DISPLAY_SET_POWER_SAVE:
    break;

  case U8X8_MSG_DISPLAY_SET_CONTRAST:
    break;

  case U8X8_MSG_DISPLAY_DRAW_TILE:
    uint8_t *ptr = ((u8x8_tile_t *)arg_ptr)->tile_ptr;

    uint8_t argint = arg_int;

    // ptr = ((u8x8_tile_t *)arg_ptr)->tile_ptr;
    uint8_t cnt = ((u8x8_tile_t *)arg_ptr)->cnt;

    // Transpose all tile matrixes one by one
    memset(buf_transposed, 0, 32);
    for (uint8_t i = 0; i < ((u8x8_tile_t *)arg_ptr)->cnt; i++) {
      transpose8(ptr + 8 * i, 1, 4, buf_transposed + i);
    }

    // Copy buf_transposed to display DMA buffer
    ws19695_pio_set_buffer(buf_transposed);
    break;

  case U8X8_MSG_DISPLAY_REFRESH:
    break;

  default:
    return 0;
  }
  return 1;
}

void u8g2_Setup_ws19695_22x7_pio(u8g2_t *u8g2, const u8g2_cb_t *rotation,
                                 u8x8_msg_cb byte_cb,
                                 u8x8_msg_cb gpio_and_delay_cb) {
  uint8_t tile_buf_height;
  uint8_t *buf;

  u8g2_SetupDisplay(u8g2, u8x8_d_ws19695_22x7_pio, u8x8_cad_empty, byte_cb,
                    gpio_and_delay_cb);
  buf = u8g2_m_4_1_1(&tile_buf_height);
  u8g2_SetupBuffer(u8g2, buf, tile_buf_height, u8g2_ll_hvline_vertical_top_lsb,
                   rotation);
}
#endif

void ws19695_pio_init() {
  ws19695_configure_pwm(PIN_PWM, false);
  ws19695_program_init();
  ws19695_configure_dma();

#ifdef USE_U8G2
  u8g2_Setup_ws19695_22x7_pio(&u8g2, U8G2_MIRROR_VERTICAL, ws19695_byte_cb,
                              ws19695_gpio_and_delay_cb);
#endif
}

// Copy data into display DMA buffer
void ws19695_pio_set_buffer(uint8_t *data) {
  // Skipping row 0 and first two bits of remaining rows to preserve LED bits
  for (uint8_t i = 1; i < 8; i++) {
    buf_ws19695[i] = buf_ws19695[i] & 0xC0000000 | data[4 * i] << 22 |
                     data[4 * i + 1] << 14 | data[4 * i + 2] << 6;
  }
}

void ws19695_pio_set_led(uint8_t led_id, bool state) {
  switch (led_id) {

  case WS19695_LED_MOVE_ON:
    buf_ws19695[0] = buf_ws19695[0] & 0x3FFFFFFF | state << 30 | state << 31;
    break;

  case WS19695_LED_ALARM_ON:
    buf_ws19695[1] = buf_ws19695[1] & 0x3FFFFFFF | state << 30 | state << 31;
    break;

  case WS19695_LED_COUNT_DOWN:
    buf_ws19695[2] = buf_ws19695[2] & 0x3FFFFFFF | state << 30 | state << 31;
    break;

  case WS19695_LED_COUNT_UP:
    buf_ws19695[5] = buf_ws19695[5] & 0x3FFFFFFF | state << 30 | state << 31;
    break;

  case WS19695_LED_DEG_F:
    buf_ws19695[3] = buf_ws19695[3] & 0x3FFFFFFF | state << 31;
    break;

  case WS19695_LED_DEG_C:
    buf_ws19695[3] = buf_ws19695[3] & 0x3FFFFFFF | state << 30;
    break;

  case WS19695_LED_AM:
    buf_ws19695[4] = buf_ws19695[4] & 0x3FFFFFFF | state << 31;
    break;

  case WS19695_LED_PM:
    buf_ws19695[4] = buf_ws19695[4] & 0x3FFFFFFF | state << 30;
    break;

  case WS19695_LED_HOURLY:
    buf_ws19695[6] = buf_ws19695[6] & 0x3FFFFFFF | state << 30 | state << 31;
    break;

  case WS19695_LED_AUTOLIGHT:
    buf_ws19695[7] = buf_ws19695[7] & 0x3FFFFFFF | state << 30 | state << 31;
    break;

  case WS19695_LED_DAY_MON:
    buf_ws19695[0] = buf_ws19695[0] & 0x3FFFFFFF | state << 27 | state << 28;
    break;

  case WS19695_LED_DAY_TUE:
    buf_ws19695[0] = buf_ws19695[0] & 0x3FFFFFFF | state << 24 | state << 25;
    break;

  case WS19695_LED_DAY_WED:
    buf_ws19695[0] = buf_ws19695[0] & 0x3FFFFFFF | state << 21 | state << 22;
    break;

  case WS19695_LED_DAY_THU:
    buf_ws19695[0] = buf_ws19695[0] & 0x3FFFFFFF | state << 18 | state << 19;
    break;

  case WS19695_LED_DAY_FRI:
    buf_ws19695[0] = buf_ws19695[0] & 0x3FFFFFFF | state << 15 | state << 16;
    break;

  case WS19695_LED_DAY_SAT:
    buf_ws19695[0] = buf_ws19695[0] & 0x3FFFFFFF | state << 12 | state << 13;
    break;

  case WS19695_LED_DAY_SUN:
    buf_ws19695[0] = buf_ws19695[0] & 0x3FFFFFFF | state << 9 | state << 10;
    break;

  default:
    break;
  }
}
