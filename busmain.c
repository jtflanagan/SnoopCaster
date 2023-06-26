#include <pico/stdlib.h>
#include <pico/binary_info.h>
#include <pico/multicore.h>
#include <hardware/spi.h>
#include <hardware/dma.h>
#include <hardware/clocks.h>
#include <stdio.h>
#include <string.h>
#include "busconfig.h"
#include "pico/stdlib.h"

//int bus_spinlock;
volatile uint8_t bus_buffer_begin;
volatile uint8_t bus_buffer_end;
uint32_t bus_buffer[256];
uint16_t devsel_mask;
uint16_t iosel_mask;
uint8_t devsel_memory[16];
uint8_t iosel_memory[256];
uint8_t iostrobe_memory[2048];

extern void abus_core_init();
extern void abus_core_loop();
extern void spi_core_init();
extern void spi_core_loop();

static void core1_main() {
  printf("initializing spi core\n");
  spi_core_init();
  printf("launching spi core loop\n");
  spi_core_loop();
}

int main() {
    // Adjust system clock for better dividing into other clocks
    set_sys_clock_khz(CONFIG_SYSCLOCK*1000, true);

    // configure the specified clock
    clock_configure(
        clk_peri,
        0,                                                // No glitchless mux
        //CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS, // System PLL on AUX mux
	CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLK_SYS,
        CONFIG_SYSCLOCK * 1000 * 1000,                               // Input frequency
        CONFIG_SYSCLOCK * 1000 * 1000                                // Output (must be same as no divider)
    );

    stdio_init_all();

    sleep_ms(5000);

    memset(devsel_memory, 0, sizeof(devsel_memory));
    memset(iosel_memory, 0, sizeof(iosel_memory));
    memset(iostrobe_memory, 0, sizeof(iostrobe_memory));

    //printf("claiming spinlock\n");
    //bus_spinlock = spin_lock_claim_unused(true);
    //printf("got spinlock %d\n",bus_spinlock);
    bus_buffer_begin = 0;
    bus_buffer_end = 0;
    devsel_mask = 0xc080 | (SLOT_ID << 4);
    iosel_mask = 0xc000 | (SLOT_ID << 8);

    multicore_launch_core1(core1_main);

    abus_core_init();
    abus_core_loop();
}
