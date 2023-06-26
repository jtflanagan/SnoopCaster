#pragma once

#define CONFIG_SYSCLOCK 126

#define CONFIG_ABUS_PIO pio0
#define ABUS_MAIN_SM 0

#define CONFIG_PIN_APPLEBUS_CONTROL_BASE 0 /* 4 pins */
#define CONFIG_PIN_APPLEBUS_DATA_BASE 12 /* 15 pins */
#define CONFIG_PIN_APPLEBUS_RW (CONFIG_PIN_APPLEBUS_DATA_BASE+8)
#define CONFIG_PIN_APPLEBUS_PHI0 (CONFIG_PIN_APPLEBUS_DATA_BASE+9)
#define CONFIG_PIN_APPLEBUS_USER1 (CONFIG_PIN_APPLEBUS_DATA_BASE+10)
#define CONFIG_PIN_APPLEBUS_INH 27
#define CONFIG_PIN_APPLEBUS_RDY 28

#define SERIAL_RAM_SPI_PORT spi0
#define SERIAL_RAM_PIN_RX 4
#define SERIAL_RAM_PIN_CS 5
#define SERIAL_RAM_PIN_SCK 6
#define SERIAL_RAM_PIN_TX 7

#define ETH_PICO_SPI_PORT spi1
#define ETH_PICO_PIN_RX 8
#define ETH_PICO_PIN_READY 9
#define ETH_PICO_PIN_SCK 10
#define ETH_PICO_PIN_TX 11

#define SLOT_ID 2

//extern int bus_spinlock;
// try using a ring buffer with minimal synchronization
extern volatile uint8_t bus_buffer_begin;
extern volatile uint8_t bus_buffer_end;
extern uint32_t bus_buffer[256];
extern uint16_t devsel_mask;
extern uint16_t iosel_mask;
extern uint8_t devsel_memory[16];
extern uint8_t iosel_memory[256];
extern uint8_t iostrobe_memory[2048];
