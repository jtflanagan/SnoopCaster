#include <string.h>
#include <hardware/pio.h>
#include "busconfig.h"
#include <stdio.h>
#include "pico/stdlib.h"
#include "abus_core.pio.h"

static uint8_t emitted_data;
static uint8_t iostrobe;

static void abus_core_setup(PIO pio, uint sm) {
  uint program_offset = pio_add_program(pio, &abus_core_program);
  pio_sm_claim(pio, sm);
  pio_sm_config c = abus_core_program_get_default_config(program_offset);
  sm_config_set_jmp_pin(&c, CONFIG_PIN_APPLEBUS_RW);
  sm_config_set_in_pins(&c, CONFIG_PIN_APPLEBUS_DATA_BASE);
  sm_config_set_set_pins(&c, CONFIG_PIN_APPLEBUS_CONTROL_BASE, 4);
  // note that for data pushes, we push 11, but for address we push
  // 19.  Setting the autopush to 11 lets both work.  The first address
  // push will only be 8 pins (so it won't autopush yet), and the second
  // push will be of 11 which will pass the threshold and trigger the
  // autopush.  For data pushes, those just trigger on the single 11 pin push.
  sm_config_set_in_shift(&c, false, true, 11);

  // set the behavior for MOV STATUS to return ~NULL if the TXFIFO is empty,
  // or NULL if it has at least one entry
  sm_config_set_mov_status(&c, STATUS_TX_LESSTHAN, 1);

  // set control pins to 0b1110, which is all three transceivers disabled,
  // and the direction for the data transceiver set to "in", on GPIO pins
  // 0 through 3.
  pio_sm_set_pins_with_mask(pio, sm,
			    (uint32_t)0xe,
			    (uint32_t)0xf);

  // set pin directions for the used pins: pins 0-3 are out, 12-22 are in,
  // and other pins are left untouched.
  pio_sm_set_pindirs_with_mask(pio, sm,
			       (uint32_t)0xf,
			       (uint32_t)((0xf << 
					   CONFIG_PIN_APPLEBUS_CONTROL_BASE) | 
					  (0x07ff << 
					   CONFIG_PIN_APPLEBUS_DATA_BASE)));
  // disable input sync bypass on the input pins we care about,
  // except for PHI0 where we need it synchronized
  pio->input_sync_bypass |= (0x7ff << CONFIG_PIN_APPLEBUS_DATA_BASE);
  pio->input_sync_bypass &= ~(1 << CONFIG_PIN_APPLEBUS_PHI0);

  for (int pin = CONFIG_PIN_APPLEBUS_CONTROL_BASE;
       pin < CONFIG_PIN_APPLEBUS_CONTROL_BASE + 4;
       ++pin) {
    pio_gpio_init(pio, pin);
  }
  for (int pin = CONFIG_PIN_APPLEBUS_DATA_BASE; 
       pin < CONFIG_PIN_APPLEBUS_DATA_BASE+11; 
       ++pin) {
    pio_gpio_init(pio, pin);
    gpio_set_pulls(pin, false, false);
  }
}

void abus_core_init() {
  iostrobe = false;
  abus_core_setup(CONFIG_ABUS_PIO, ABUS_MAIN_SM);
  pio_enable_sm_mask_in_sync(CONFIG_ABUS_PIO, (1 << ABUS_MAIN_SM));
}

// returns true if we are emitting a read for this address cycle
static inline bool check_address(uint16_t addr, uint8_t ctrl) {
  if (ctrl & 0x1) {
    if ( (addr & 0xff00) == iosel_mask) {
      uint16_t iosel_addr = addr & 0xff;
      emitted_data = iosel_memory[iosel_addr];
      return true;
    } else if (iostrobe && ((addr & 0xc800) == 0xc800)) {
      uint16_t iostrobe_addr = addr & 0x7ff;
      emitted_data = iostrobe_memory[iostrobe_addr];
      return true;
    } else if ( (addr & 0xfff0) == devsel_mask) {
      uint16_t devsel_addr = addr & 0xf;
      emitted_data = devsel_memory[devsel_addr];
      return true;
    }
  }
  return false;
}

static inline void check_data(uint16_t addr, uint8_t data) {
  if (addr == iosel_mask) {
    iostrobe = true;
  } else if (addr == 0xcfff) {
    iostrobe = false;
  }
  if (iostrobe && ((addr & 0xc800) == 0xc800)) {
    uint16_t iostrobe_addr = addr & 0x7ff;
    iostrobe_memory[iostrobe_addr] = data;
  } else if ( (addr & 0xfff0) == devsel_mask) {
    uint16_t devsel_addr = addr & 0xf;
    devsel_memory[devsel_addr] = data;
  }
}

void __time_critical_func(abus_core_loop)() {
  uint32_t value;
  // We process alternating P1 (video) and P0 (cpu) reads from the PIO.
  // We start our loop by discarding either one or two reads, in order to 
  // be cleanly in the state where the next read is a P1 read and we can
  // safely loop forever.
  // If the first discarding read we do, PHI0 is 0 (meaning video phase P1),
  // then we do a second discarding read to throw out cpu phase P0.
  // Otherwise if PHI0 is 1, then we break out after only one discard read.
  do {
    // if PHI0 is 1, this is a P0 read, we eat it, break out, and are on
    // PHI1 for the main loop.  If PHI0 is 0, then 
    value = pio_sm_get_blocking(CONFIG_ABUS_PIO, ABUS_MAIN_SM);
  } while ((value >> 10) & 0x1);
  while (1) {
    uint16_t addr;
    uint8_t ctrl;
    uint8_t data;
    value = pio_sm_get_blocking(CONFIG_ABUS_PIO, ABUS_MAIN_SM);
    // value is 8 bits of addrhi, 3 bits of control, 8 bits of addrlo
    addr = (value >> 11) + (value & 0xff);
    ctrl = (value >> 8) & 0x07;
    if (check_address(addr, ctrl)) {
      // do the emit
      pio_sm_put(CONFIG_ABUS_PIO, ABUS_MAIN_SM, emitted_data);
      // do a dummy read to stay in sync on P1/P0 reads
      pio_sm_get_blocking(CONFIG_ABUS_PIO, ABUS_MAIN_SM);
      data = emitted_data;
    } else {
      value = pio_sm_get_blocking(CONFIG_ABUS_PIO, ABUS_MAIN_SM);
      data = value & 0xff;
    }
    check_data(addr, data);
    bus_buffer[bus_buffer_end] = 
      ((uint32_t)ctrl << 24) + ((uint32_t)addr << 8) + data;
    ++bus_buffer_end;
  }
}
