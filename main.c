#include <pico/stdlib.h>
#include <pico/binary_info.h>
#include <pico/multicore.h>
#include <hardware/spi.h>
#include <hardware/dma.h>
#include <hardware/clocks.h>
#include <stdio.h>
#include "config.h"
#include "abus.h"
#include "eth.h"
#include "buffers.h"
#include "pico/stdlib.h"


#define CONFIG_SYSCLOCK 133

static void core1_main() {
  printf("initializing eth\n");
  eth_init();
  printf("launching eth loop\n");
  eth_loop();
}

int main() {
    // Adjust system clock for better dividing into other clocks
    set_sys_clock_khz(CONFIG_SYSCLOCK*1000, true);

    // configure the specified clock
    clock_configure(
        clk_peri,
        0,                                                // No glitchless mux
        CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS, // System PLL on AUX mux
        CONFIG_SYSCLOCK * 1000 * 1000,                               // Input frequency
        CONFIG_SYSCLOCK * 1000 * 1000                                // Output (must be same as no divider)
    );

    stdio_init_all();

    gpio_init(25);
    gpio_set_dir(25, GPIO_OUT);

    sleep_ms(5000);

    printf("claiming spinlock\n");
    bus_spinlock = spin_lock_claim_unused(true);
    printf("got spinlock %d\n",bus_spinlock);
    queue_init_with_spinlock(&raw_bus_queue, 256, 64, bus_spinlock);
    printf("initialized bus queue\n");

    multicore_launch_core1(core1_main);

    abus_init();
    abus_loop();
}
