#include <string.h>
#include <hardware/pio.h>
#include "config.h"
#include "abus.h"
#include "abus.pio.h"
#include "buffers.h"
#include <stdio.h>
#include "pico/stdlib.h"

#if CONFIG_PIN_APPLEBUS_PHI0 != PHI0_GPIO
#error CONFIG_PIN_APPLEBUS_PHI0 and PHI0_GPIO must be set to the same pin
#endif


enum {
    ABUS_MAIN_SM = 0,
    ABUS_DEVICE_READ_SM = 1,
};


static void abus_device_read_setup(PIO pio, uint sm) {
    uint program_offset = pio_add_program(pio, &abus_device_read_program);
    pio_sm_claim(pio, sm);

    pio_sm_config c = abus_device_read_program_get_default_config(program_offset);

    // set the "device selected" pin as the jump pin
    sm_config_set_jmp_pin(&c, CONFIG_PIN_APPLEBUS_DEVSEL);

    // map the OUT pin group to the data signals
    sm_config_set_out_pins(&c, CONFIG_PIN_APPLEBUS_DATA_BASE, 8);

    // map the SET pin group to the Data transceiver control signals
    sm_config_set_set_pins(&c, CONFIG_PIN_APPLEBUS_CONTROL_BASE, 2);

    pio_sm_init(pio, sm, program_offset, &c);

    // All the GPIOs are shared and setup by the main program
}

static void abus_main_setup(PIO pio, uint sm) {
    uint program_offset = pio_add_program(pio, &abus_program);
    pio_sm_claim(pio, sm);

    pio_sm_config c = abus_program_get_default_config(program_offset);

    // set the bus R/W pin as the jump pin
    sm_config_set_jmp_pin(&c, CONFIG_PIN_APPLEBUS_RW);

    // map the IN pin group to the data signals
    sm_config_set_in_pins(&c, CONFIG_PIN_APPLEBUS_DATA_BASE);

    // map the SET pin group to the bus transceiver enable signals
    sm_config_set_set_pins(&c, CONFIG_PIN_APPLEBUS_CONTROL_BASE+1, 3);

    // configure left shift into ISR & autopush every 26 bits
    sm_config_set_in_shift(&c, false, true, 26);

    pio_sm_init(pio, sm, program_offset, &c);

    // configure the GPIOs
    // Ensure all transceivers will start disabled, with Data transceiver direction set to 'in'
    pio_sm_set_pins_with_mask(pio, sm,
        (uint32_t)0xe << CONFIG_PIN_APPLEBUS_CONTROL_BASE,
        (uint32_t)0xf << CONFIG_PIN_APPLEBUS_CONTROL_BASE);
    pio_sm_set_pindirs_with_mask(pio, sm,
        (0xf << CONFIG_PIN_APPLEBUS_CONTROL_BASE),
        (1 << CONFIG_PIN_APPLEBUS_PHI0) | (0xf << CONFIG_PIN_APPLEBUS_CONTROL_BASE) | (0x3ff << CONFIG_PIN_APPLEBUS_DATA_BASE));

    // Disable input synchronization on input pins that are sampled at known stable times
    // to shave off two clock cycles of input latency
    pio->input_sync_bypass |= (0x3ff << CONFIG_PIN_APPLEBUS_DATA_BASE);

    pio_gpio_init(pio, CONFIG_PIN_APPLEBUS_PHI0);
    gpio_set_pulls(CONFIG_PIN_APPLEBUS_PHI0, false, false);
    for(int pin=CONFIG_PIN_APPLEBUS_CONTROL_BASE; pin < CONFIG_PIN_APPLEBUS_CONTROL_BASE+4; pin++) {
        pio_gpio_init(pio, pin);
    }
    for(int pin=CONFIG_PIN_APPLEBUS_DATA_BASE; pin < CONFIG_PIN_APPLEBUS_DATA_BASE+10; pin++) {
        pio_gpio_init(pio, pin);
        gpio_set_pulls(pin, false, false);
    }
}

void abus_init() {
    abus_device_read_setup(CONFIG_ABUS_PIO, ABUS_DEVICE_READ_SM);
    abus_main_setup(CONFIG_ABUS_PIO, ABUS_MAIN_SM);

    pio_enable_sm_mask_in_sync(CONFIG_ABUS_PIO, (1 << ABUS_MAIN_SM) | (1 << ABUS_DEVICE_READ_SM));
}

static inline void __time_critical_func(synth_bus)() {
  uint32_t count = 0;
  uint32_t fails = 0;
  uint8_t data = 0;
  uint16_t prev_address = 0;
  uint16_t address = 0;
  uint8_t rw = 0;
  uint16_t buf_index = 0;
  uint32_t next_tx_seqno = 0;
  uint32_t last_count = 0;
  uint32_t packets = 0;
  while(1) {
    if (count) {
      uint32_t sleep_amount = count - last_count;
      //sleep_amount -= 40;
      sleep_us(sleep_amount);
    }
    last_count = count;
    buf_index = (buf_index + 1) % 8;
    uint8_t* b = bus_buffers[buf_index];
    uint8_t* p = b + 3;
    ++next_tx_seqno;
    *p++ = next_tx_seqno & 0xff;
    *p++ = (next_tx_seqno >> 8) & 0xff;
    *p++ = (next_tx_seqno >> 16) & 0xff;
    *p++ = (next_tx_seqno >> 24) & 0xff;
    *p++ = 0; // bus data packet type
    while (p - b < 1475 - 26) {
      uint8_t* rw_flags = p++;
      uint8_t* seq_flags = p++;
      uint8_t* data_p = p;
      p += 8;
      *rw_flags = 0;
      *seq_flags = 0;
      for (uint8_t i = 0; i < 8; ++i) {
	++data;
	++address;
	if ( (i % 4) == 0) {
	  ++address;
	}
	rw = !rw;
	*rw_flags |= rw << i;
	*data_p++ = data;
	if (address != prev_address + 1) {
	  *seq_flags |= 1 << i;
	  uint8_t address_lo = address & 0xff;
	  *p++ = address_lo;
	  uint8_t address_hi = (address >> 8) & 0xff;
	  *p++ = address_hi;
	}
	prev_address = address;
      }
      count += 8;
    }
    uint32_t val = (uint32_t)buf_index << 16;
    val += (p - b);
    bool ret = queue_try_add(&raw_bus_queue, &val);
    if (!ret) {
      ++fails;
    }
    ++packets;
    if (packets % (4096) == 0) {
      printf("writes: %d: %d\n",packets, fails);
      fails = 0;
    }
  }
}

static uint16_t buf_index = 0;
static uint32_t next_tx_seqno = 0;
static uint8_t* buf;
static uint8_t* p;
static uint8_t* rw_flags;
static uint8_t* seq_flags;
static uint8_t* data_p;

static inline void __time_critical_func(begin_packet)() {
  buf = bus_buffers[buf_index];
  p = buf + 3;
  ++next_tx_seqno;
  *p++ = next_tx_seqno & 0xff;
  *p++ = (next_tx_seqno >> 8) & 0xff;
  *p++ = (next_tx_seqno >> 16) & 0xff;
  *p++ = (next_tx_seqno >> 24) & 0xff;
  *p++ = 0; // bus data packet type
}

static inline void __time_critical_func(begin_batch)() {
  rw_flags = p++;
  seq_flags = p++;
  data_p = p;
  p += 8;
  *rw_flags = 0;
  *seq_flags = 0;
}


void __time_critical_func(abus_loop)() {

  // run the synthesized bus instead
  //synth_bus();

  uint16_t prev_address = 0;
  uint8_t batch_index = 0;
  begin_packet();
  begin_batch();
  while(1) {
    uint32_t value = pio_sm_get_blocking(CONFIG_ABUS_PIO, ABUS_MAIN_SM);
    uint16_t address = value >> 10;
    uint8_t rw = (value >> 9) & 0x1;
    uint8_t data = value & 0xff;
    //printf("%d %d %d\n",address, rw, data);
    *rw_flags |= rw << batch_index;
    *data_p++ = data;
    if (address != prev_address + 1) {
      *seq_flags |= 1 << batch_index;
      uint8_t address_lo = address & 0xff;
      *p++ = address_lo;
      uint8_t address_hi = (address >> 8) & 0xff;
      *p++ = address_hi;
    }
    prev_address = address;
    ++batch_index;
    if (batch_index == 8) {
      uint16_t buf_size = p - buf;
      if (buf_size > 1400) {
	uint32_t value = (uint32_t)buf_index << 16;
	value += buf_size;
	queue_try_add(&raw_bus_queue, &value);
	buf_index = (buf_index + 1) % 8;
	begin_packet();
      }
      begin_batch();
      batch_index = 0;
    }
  }
}
