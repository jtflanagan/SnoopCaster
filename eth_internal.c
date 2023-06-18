#include "eth_internal.h"
#include <stdio.h>


// this expects that the desired bytes have already been staged into
// msg_buf with the correct 3-byte offset reserved at the start for the
// SPI address
// also expects that the caller checked tx_has_room() before trying
static void add_tx_bytes(int16_t len) {
  if (tx_ring_empty()) {
    tx_ring_end = (tx_ring_end + 1) % 8;
    tx_ring[tx_ring_end].begin = tx_write_ptr;
    tx_ring[tx_ring_end].length = 0;
  } else if (tx_ring[tx_ring_end].length + len > 1472) {
    tx_ring_end = (tx_ring_end + 1) % 8;
    tx_ring[tx_ring_end].begin = tx_write_ptr;
    tx_ring[tx_ring_end].length = 0;
  }
  uint32_t addr = ((uint32_t)tx_write_ptr << 8) + (WIZCHIP_TXBUF_BLOCK(0) << 3);
  eth_write(addr, msg_buf, len+3, false);
  tx_write_ptr += len;
  tx_ring[tx_ring_end].len += len;
}

static uint16_t tx_write_ptr;
static uint16_t tx_read_ptr;



enum CMD_STATE {
  ETH_IDLE,
  ETH_TX_DMA,
  ETH_TX_SEND,
  ETH_TX_WAIT,
  ETH_RX_DMA,
  ETH_RX_RECV,
  ETH_RX_WAIT,
};

static int eth_cmd_state = ETH_IDLE;
static int prev_eth_cmd_state = ETH_IDLE;

void print_eth_cmd_state() {
  switch (eth_cmd_state) {
  case ETH_IDLE:
    printf("ETH_IDLE\n");
    return;
  case ETH_TX_DMA:
    printf("ETH_TX_DMA\n");
    return;
  case ETH_TX_SEND:
    printf("ETH_TX_SEND\n");
    return;
  case ETH_TX_WAIT:
    printf("ETH_TX_WAIT\n");
    return;
  case ETH_RX_DMA:
    printf("ETH_RX_DMA\n");
    return;
  case ETH_RX_RECV:
    printf("ETH_RX_RECV\n");
    return;
  case ETH_RX_WAIT:
    printf("ETH_RX_WAIT\n");
    return;
  default:
    printf("unknown wiz_cmd_state\n");
  }
}

bool is_dma_busy() {
  if (dma_channel_is_busy(eth_dma_rx) || dma_channel_is_busy(eth_dma_tx)) {
    //printf("busy\n");
    return true;
  }
}

bool check_interrupts() {
  if (gpio_get(ETH_PIN_INT) == 0) {
    uint8_t tmp = eth_read8(ETHS_IR(0));
    if (tmp & ETH_IR_RECV) {
      eth_read_packet(rx_header_buf, 8);
      
    }
  }
}

  if (eth_cmd_state == ETH_TX_DMA) {
    eth_deselect();
    eth_write16(ETHS_TX_WR(0), tx_write_ptr);
    eth_write8(ETHS_CR(0), ETH_CR_SEND);
    eth_cmd_state = ETH_TX_SEND;
  } else if (eth_cmd_state == ETH_RX_DMA) {
    printf("completed rx dma\n");
    eth_deselect();
    eth_write16(ETHS_RX_RD(0), tx_read_ptr);
    eth_write8(ETHS_CR(0), ETH_CR_RECV);
    eth_cmd_state = ETH_RX_RECV;
  }
  if (eth_cmd_state == ETH_TX_SEND ||
      eth_cmd_state == ETH_RX_RECV) {
    uint8_t tmp = eth_read8(ETHS_CR(0));
    //printf("checking cmd state: %u\n", tmp);
    if (tmp == 0) {
      if (eth_cmd_state == ETH_TX_SEND) {
	eth_cmd_state = ETH_TX_WAIT;
      } else {
	eth_cmd_state = ETH_RX_WAIT;
      }
    }
  }
  if (gpio_get(ETH_PIN_INT) == 0) {
    //printf("checking interrupt\n");
    uint8_t tmp = eth_read8(ETHS_IR(0));
    //printf("interrupt %d\n",tmp);
    eth_write8(ETHS_IR(0),tmp);
    if (tmp & ETH_IR_SENDOK) {
      if (eth_cmd_state != ETH_TX_WAIT) {
	printf("got SENDOK with no active write\n");
	while(1);
      }
      eth_cmd_state = ETH_IDLE;
      return false;
    }
    if (tmp & ETH_IR_RECV) {
      if (eth_cmd_state != ETH_RX_DMA) {
    	printf("got RECV with no active read\n");
    	while(1);
      }
      eth_cmd_state = ETH_IDLE;
      return false;
    }
    if (tmp & ETH_IR_TIMEOUT) {
      printf("socket timeout\n");
      uint8_t mac_buf[6];
      eth_read(ETHS_DHAR(0), mac_buf, 6, true);
      while(1);
    }
  }
  /* } else { */
  /*   printf("no interrupt\n"); */
  /* } */
  uint8_t sock_state = eth_read8(ETHS_SR(0));
  if (sock_state == ETH_SOCK_CLOSED) {
    printf("socket closed\n");
    while(1);
  }
  return true;
}
