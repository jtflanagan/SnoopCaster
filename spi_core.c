#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/spi.h"
#include "hardware/dma.h"
#include "hardware/clocks.h"
#include "busconfig.h"
#include <string.h>

static uint eth_dma_tx;
static uint eth_dma_rx;
static dma_channel_config eth_dma_channel_config_tx;
static dma_channel_config eth_dma_channel_config_rx;
static uint32_t spi_tx_buf;
static uint32_t spi_rx_buf;
static uint32_t spi_rx_msg_len;
static uint32_t spi_rx_msg_recv;
static uint8_t spi_rx_msg_buf[1024];
static bool remote_ready = false;

void spi_core_init() {
  spi_init(ETH_PICO_SPI_PORT, 62500 * 1000);
  gpio_set_function(ETH_PICO_PIN_SCK, GPIO_FUNC_SPI);
  gpio_set_function(ETH_PICO_PIN_TX, GPIO_FUNC_SPI);
  gpio_set_function(ETH_PICO_PIN_RX, GPIO_FUNC_SPI);
  gpio_init(ETH_PICO_PIN_READY);
  gpio_set_dir(ETH_PICO_PIN_READY, GPIO_IN);
  
  eth_dma_tx = dma_claim_unused_channel(true);
  eth_dma_rx = dma_claim_unused_channel(true);
  eth_dma_channel_config_tx = dma_channel_get_default_config(eth_dma_tx);
  channel_config_set_transfer_data_size(&eth_dma_channel_config_tx, DMA_SIZE_16);
  channel_config_set_dreq(&eth_dma_channel_config_tx, DREQ_SPI1_TX);
  channel_config_set_read_increment(&eth_dma_channel_config_tx, false);
  channel_config_set_write_increment(&eth_dma_channel_config_tx, false);
  dma_channel_configure(eth_dma_tx, &eth_dma_channel_config_tx,
			&spi_get_hw(ETH_PICO_SPI_PORT)->dr,
			&spi_tx_buf,
			4,
			false);

  eth_dma_channel_config_rx = dma_channel_get_default_config(eth_dma_rx);
  channel_config_set_transfer_data_size(&eth_dma_channel_config_rx, DMA_SIZE_16);
  channel_config_set_dreq(&eth_dma_channel_config_rx, DREQ_SPI1_RX);
  channel_config_set_read_increment(&eth_dma_channel_config_rx, false);
  channel_config_set_write_increment(&eth_dma_channel_config_rx, false);
  dma_channel_configure(eth_dma_rx, &eth_dma_channel_config_rx,
			&spi_get_hw(ETH_PICO_SPI_PORT)->dr,
			&spi_rx_buf,
			4,
			false);

  spi_rx_msg_len = 0;
  spi_rx_msg_recv = 0;
}

static void process_rx_msg() {
  // not doing anything with this yet
}

void spi_core_loop() {
  while (1) {
    if (remote_ready && (bus_buffer_begin != bus_buffer_end)) {
      spi_tx_buf = bus_buffer[bus_buffer_begin];
      ++bus_buffer_begin;
      dma_start_channel_mask((1u << eth_dma_tx) | (1u << eth_dma_rx));
      dma_channel_wait_for_finish_blocking(eth_dma_tx);
    }
    if (gpio_get(ETH_PICO_PIN_READY) == 1) {
      // if the remote is not ready, reset any rx state and discard
      // any buffered bus events
      spi_rx_msg_len = 0;
      spi_rx_msg_recv = 0;
      remote_ready = false;
      bus_buffer_begin = bus_buffer_end;
      continue;
    }
    if (!remote_ready) {
      // if the remote is just coming up, whatever is in spi_rx_buf
      // is garbage, skip it and read again
      remote_ready = true;
      continue;
    }
    if (spi_rx_msg_len) {
      memcpy(spi_rx_msg_buf + spi_rx_msg_recv, (char*)spi_rx_buf, 4);
      spi_rx_msg_recv += 4;
      if (spi_rx_msg_recv >= spi_rx_msg_len) {
	process_rx_msg();
      }
      spi_rx_msg_len = 0;
      spi_rx_msg_recv = 0;
    } else {
      spi_rx_msg_len = spi_rx_buf;
    }
  }
}
