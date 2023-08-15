#include <stdio.h>

#include "pico/stdlib.h"

#ifdef CYW43_WL_GPIO_LED_PIN
#include "pico/cyw43_arch.h"
#else
#include "hardware/gpio.h"
#endif

#include "../lib/ws19695-pio/ws19695_pio.h"
#include "../lib/u8g2/csrc/u8g2.h"

#include "u8x8_d_ws19695.h"

u8g2_t u8g2;

int main()
{
    stdio_init_all();

#ifdef CYW43_WL_GPIO_LED_PIN
    if (cyw43_arch_init())
    {
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

    u8g2_Setup_ws19695_22x7_pio(&u8g2, U8G2_MIRROR_VERTICAL, ws19695_byte_cb, ws19695_gpio_and_delay_cb);
    u8g2_ClearDisplay(&u8g2);

    uint8_t h = u8g2_GetDisplayHeight(&u8g2);
    uint8_t w = u8g2_GetDisplayWidth(&u8g2);

    printf("Display size w:%d h:%d\n", w, h);

    u8g2_SetFont(&u8g2, u8g2_font_4x6_tr);
    u8g2_DrawStr(&u8g2, 0, 5, "HELLO");

    //u8g2_DrawFrame(&u8g2, 0, 0, w, h);
    u8g2_DrawHLine(&u8g2, 0, h-1, w);
    u8g2_DrawVLine(&u8g2, w-1, 0, h);
    u8g2_UpdateDisplay(&u8g2);

    uint32_t counter = 0;

    while (true)
    {
        tight_loop_contents();
        //buf_ws19695[4] = 0x20000100 | counter++ << 10;
        //sleep_ms(250);

    }
}

