#include "ws19695_pio.h"

#include <string.h>  /* memcpy */

#include "hardware/gpio.h"
#include "hardware/dma.h"
#include "hardware/pwm.h"

#include "ws19695.pio.h"  // Generated from ws19695.pio

#define PIN_PWM 27

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

static uint32_t buf_ws19695[8] __attribute__ ((aligned(32)));
static uint32_t p_buf_ws19695 = (uint32_t)(buf_ws19695);

static uint dma_chan_transfer, dma_chan_control;
static dma_channel_config dma_cfg_transfer, dma_cfg_control;


static void ws19695_configure_pwm(uint pin_pwm, bool pwm_enabled)
{
    gpio_init(pin_pwm);
    gpio_set_dir(pin_pwm, GPIO_OUT);

    if (! pwm_enabled) {
        gpio_put(pin_pwm, 1);
        return;
    }

    gpio_set_function(pin_pwm, GPIO_FUNC_PWM);
    // Find out which PWM slice is connected to GPIO pin_pwm
    uint slice_num = pwm_gpio_to_slice_num(pin_pwm);

    // Set period of 4 cycles (0 to 3 inclusive)
    pwm_set_wrap(slice_num, 3);
    pwm_set_clkdiv_int_frac (slice_num, 20, 0);

    // Set channel A output high for one cycle before dropping
    pwm_set_chan_level(slice_num, PWM_CHAN_A, 1);
    // Set initial B output high for three cycles before dropping
    pwm_set_chan_level(slice_num, PWM_CHAN_B, 1);
    
    // Set the PWM running
    pwm_set_enabled(slice_num, true);
}

static inline void ws19695_configure_dma()
{
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

	dma_channel_configure(
		dma_chan_transfer,
		&dma_cfg_transfer,
		&pio0_hw->txf[0],
		&buf_ws19695,
		8,
		false
	);

    dma_cfg_control = dma_channel_get_default_config(dma_chan_control);
	channel_config_set_transfer_data_size(&dma_cfg_control, DMA_SIZE_32);
	channel_config_set_read_increment(&dma_cfg_control, false);
	channel_config_set_write_increment(&dma_cfg_control, false); 
    channel_config_set_irq_quiet(&dma_cfg_control, true);
    // No DREQ
    // No chain

	dma_channel_configure(
        dma_chan_control, 
        &dma_cfg_control,
	    // Writing to this register retriggers transfer channel
        &(dma_channel_hw_addr(dma_chan_transfer)->al3_read_addr_trig),
        &p_buf_ws19695, // pointer to pointer to buf_ws19695
        1,
        true // initial DMA trigger
    );

    return;
}

void ws19695_pio_init()
{
  	ws19695_configure_pwm(PIN_PWM, false);
    ws19695_program_init();
    ws19695_configure_dma();
}

// Copy data into display DMA buffer
void ws19695_pio_set_buffer(uint8_t *data)
{
    //memcpy(data, buf_ws19695, 32);

    // Skipping row 0 and first two bits of remaining rows to preserve LED bits
    for (uint8_t i = 1; i < 8; i++)
    {
        buf_ws19695[i] = buf_ws19695[i] & 0xC0000000 | data[4*i] << 22 | 
                        data[4*i+1] << 14 | data[4*i+2] << 6;
    }
}
