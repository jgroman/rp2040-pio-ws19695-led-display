#include "u8x8_d_ws19695.h"

#include <stdio.h>
#include <string.h>  /* memset */

#define W 8
#define H 1

uint32_t buf_ws19695[8] __attribute__ ((aligned(32)));

static uint8_t buf_transposed[32];

uint8_t bitmap[W*H*8];

/* Parameter A is the address of the first byte of an 8x8 submatrix of 
  the source matrix, which is of size 8m x 8n bits. Similarly, parameter B is 
  the address of the first byte of an 8x8 submatrix in the target matrix, 
  which is of size 8n x 8m bits. That is, the full source matrix is 8m x n bytes, 
  and the full target matrix is 8n x m bytes.
  http://www.icodeguru.com/Embedded/Hacker's-Delight/048.htm
*/
static void transpose8(unsigned char *A, uint8_t m, uint8_t n,
                unsigned char *B) {
   uint32_t x, y, t; 
 
   // Load the array and pack it into x and y. 
 
   x = (A[0]<<24)   | (A[m]<<16)   | (A[2*m]<<8) | A[3*m]; 
   y = (A[4*m]<<24) | (A[5*m]<<16) | (A[6*m]<<8) | A[7*m]; 
 
   t = (x ^ (x >> 7)) & 0x00AA00AA; x = x ^ t ^ (t << 7); 
   t = (y ^ (y >> 7)) & 0x00AA00AA; y = y ^ t ^ (t << 7); 
 
   t = (x ^ (x >>14)) & 0x0000CCCC; x = x ^ t ^ (t <<14); 
   t = (y ^ (y >>14)) & 0x0000CCCC; y = y ^ t ^ (t <<14); 
 
   t = (x & 0xF0F0F0F0) | ((y >> 4) & 0x0F0F0F0F); 
   y = ((x << 4) & 0xF0F0F0F0) | (y & 0x0F0F0F0F); 
   x = t; 
 
   B[0]=x>>24;   B[n]=x>>16;   B[2*n]=x>>8; B[3*n]=x; 
   B[4*n]=y>>24; B[5*n]=y>>16; B[6*n]=y>>8; B[7*n]=y; 
} 

uint8_t ws19695_byte_cb(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
    return 1;  // All operations are successful
}

uint8_t ws19695_gpio_and_delay_cb(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
    return 1;  // All operations are successful
}

static const u8x8_display_info_t u8x8_ws19695_22x7_pio_display_info =
{
  /* chip_enable_level = */ 0,
  /* chip_disable_level = */ 1,
  
  /* post_chip_enable_wait_ns = */ 0,
  /* pre_chip_disable_wait_ns = */ 0,
  /* reset_pulse_width_ms = */ 0,
  /* post_reset_wait_ms = */ 0, 
  /* sda_setup_time_ns = */ 0,
  /* sck_pulse_width_ns = */ 0,
  /* sck_clock_hz = */ 8000000UL,	/* since Arduino 1.6.0, the SPI bus speed in Hz. Should be  1000000000/sck_pulse_width_ns */
  /* spi_mode = */ 0,		/* active high, rising edge */
  /* i2c_bus_clock_100kHz = */ 4,
  /* data_setup_time_ns = */ 0,
  /* write_pulse_width_ns = */ 0,	
  /* tile_width = */ 3,
  /* tile_height = */ 1,
  /* default_x_offset = */ 0,
  /* flipmode_x_offset = */ 0,
  /* pixel_width = */ 22,
  /* pixel_height = */ 7
};

uint8_t u8x8_d_ws19695_22x7_pio(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
  printf("display_cb msg %d\n", msg);

  switch(msg)
  {
    case U8X8_MSG_DISPLAY_INIT:
      break;
    case U8X8_MSG_DISPLAY_SETUP_MEMORY:
      u8x8_d_helper_display_setup_memory(u8x8, &u8x8_ws19695_22x7_pio_display_info);
      break;

    case U8X8_MSG_DISPLAY_SET_POWER_SAVE:
      break;

    case U8X8_MSG_DISPLAY_SET_CONTRAST:
      break;

    case U8X8_MSG_DISPLAY_DRAW_TILE:
        uint8_t *ptr = ((u8x8_tile_t *)arg_ptr)->tile_ptr;

        uint8_t argint = arg_int;

        //ptr = ((u8x8_tile_t *)arg_ptr)->tile_ptr;
        uint8_t cnt = ((u8x8_tile_t *)arg_ptr)->cnt;

        // Transpose all tile matrixes one by one
        memset(buf_transposed, 0, 32);
        for (uint8_t i = 0; i < ((u8x8_tile_t *)arg_ptr)->cnt; i++)
        {
          transpose8(ptr + 8*i, 1, 4, buf_transposed + i);
        }

        // Copy buf_transposed to display DMA buffer
        // Skipping row 0 and first two bits of remaining rows to preserve LED bits
        for (uint8_t i = 1; i < 8; i++)
        {
          buf_ws19695[i] = buf_ws19695[i] & 0xC0000000 | buf_transposed[4*i] << 22 | 
                            buf_transposed[4*i+1] << 14 | buf_transposed[4*i+2] << 6;
        }
      break;
    
    case U8X8_MSG_DISPLAY_REFRESH:
      break;

    default:
      return 0;
  }
  return 1;
}

void u8g2_Setup_ws19695_22x7_pio(u8g2_t *u8g2, const u8g2_cb_t *rotation, u8x8_msg_cb byte_cb, u8x8_msg_cb gpio_and_delay_cb)
{
  uint8_t tile_buf_height;
  uint8_t *buf;
  
  u8g2_SetupDisplay(u8g2, u8x8_d_ws19695_22x7_pio, u8x8_cad_empty, byte_cb, gpio_and_delay_cb);

  buf = u8g2_m_4_1_1(&tile_buf_height);

  u8g2_SetupBuffer(u8g2, buf, tile_buf_height, u8g2_ll_hvline_vertical_top_lsb, rotation);
}
