#include <stdio.h>

#include "pico/stdlib.h"

#ifdef CYW43_WL_GPIO_LED_PIN
#include "pico/cyw43_arch.h"
#else
#include "hardware/gpio.h"
#endif

#include "../lib/ws19695-pio/ws19695_pio.h"

int main() {
  stdio_init_all();

#ifdef CYW43_WL_GPIO_LED_PIN
  if (cyw43_arch_init()) {
    return -1;
  }
#else
  gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
#endif

  ws19695_pio_init();

// Light up onboard LED
#ifdef CYW43_WL_GPIO_LED_PIN
  cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
#else
  gpio_put(PICO_DEFAULT_LED_PIN, 1);
#endif

  u8g2_ClearDisplay(&u8g2);

  uint8_t h = u8g2_GetDisplayHeight(&u8g2);
  uint8_t w = u8g2_GetDisplayWidth(&u8g2);

  printf("Display size w:%d h:%d\n", w, h);

  u8g2_SetFont(&u8g2, u8g2_font_4x6_tr);
  u8g2_DrawStr(&u8g2, 0, 5, "HELLO");

  // u8g2_DrawFrame(&u8g2, 0, 0, w, h);
  u8g2_DrawHLine(&u8g2, 0, h - 1, w);
  u8g2_DrawVLine(&u8g2, w - 1, 0, h);
  u8g2_UpdateDisplay(&u8g2);

  while (true) {
    // tight_loop_contents();
    ws19695_pio_set_led(WS19695_LED_ALARM_ON, true);
    sleep_ms(250);
    ws19695_pio_set_led(WS19695_LED_ALARM_ON, false);
    sleep_ms(500);
  }
}
